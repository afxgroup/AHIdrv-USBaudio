# EHCI HCD: full-speed isochronous OUT fails when the payload exceeds 188 bytes

## Summary

A full-speed USB Audio Class 1.0 card produces no sound when it is connected
through a high-speed hub. The same card on a root port works. Enumeration,
descriptors, interface claiming and alternate-setting selection are all
identical in both cases, and no error is reported anywhere.

The failure depends on one thing: the number of bytes in a single isochronous
OUT transfer. At or below 188 bytes per frame audio plays. Above it there is
silence. Nothing else needs to change to switch between working and broken.

The 188-byte boundary is `EHCI_MAX_ISO_SPLIT_PAYLOAD_BYTES` in
`hcd/ehci/isochronous.c`, i.e. the point where `LlEp_CommitIsoTransfer_FullSpeed`
switches from a single start-split to a multi-split sequence. So the working
path is the single-split one and the failing path is the multi-split one.

## Why this only happens behind a hub

A full-speed device on an EHCI root port is handed to the companion OHCI/UHCI
controller and never reaches this code. Behind a high-speed hub it stays on
EHCI and its transfers go through split transactions via the hub's transaction
translator. That is the only difference between the two cases, and it is why
the bug looks like "hubs break USB audio" rather than what it is.

## Hardware and configuration

- Card: VID 0x0020 PID 0x0B21, USB Audio Class 1.0, full speed
  (`libusb` reports `LIBUSB_SPEED_FULL`)
- Streaming interface 2, alternate setting 1, isochronous OUT endpoint 0x03,
  wMaxPacketSize 384, 2 channels, 16-bit
- Hub: VID 0x1A40 PID 0x0101, USB 2.0, high speed
- Format: 48000 Hz stereo 16-bit, i.e. 48 sample frames of 4 bytes per USB
  frame = **192 bytes per frame**
- Client: AHI USB audio sub-driver, submitting isochronous `CMD_WRITE`
  IORequests to `usbsys.device` with 8 USB frames per request and 2 requests
  in flight

## Reproduction

1. Connect the card through a high-speed hub.
2. Play audio at 48 kHz stereo 16-bit, so each frame carries 192 bytes.
3. No sound. No error is reported at any level.
4. Change nothing except the per-frame payload, clamping it to 188 bytes.
5. Sound is produced. (Playback is slightly flat and drifts, because the
   device is being underfed by one sample per frame - that is expected and
   irrelevant to the point.)

Steps 2 and 4 differ by four bytes per frame and by nothing else: same device,
same hub, same endpoint, same alternate setting, same driver binary apart from
that one clamp.

## Why the failure is silent

Isochronous OUT has no handshake. The host transmits and never learns whether
the device received anything, so a delivery failure below the driver cannot be
detected from above. Specifically:

- every IORequest completes with `io_Error == USBERR_NOERROR`
- `USBGetIsoTransferResult()->Actual` reports the full requested byte count
- summing those, the driver accounts for exactly the same number of bytes in
  the working and failing cases (identical to the kilobyte over thousands of
  transfers)
- no `USBERR_FRAMEMISSED`, no `USBERR_CHECKRESULTS`, nothing in the log

From the driver's side the two cases are indistinguishable. That is what makes
this expensive to diagnose: everything reports success and the audio is silent.

## Where the problem appears to be

`LlEp_CommitIsoTransfer_FullSpeed` in `hcd/ehci/llendpoint.c` sets the
transaction position and count for an OUT transfer:

```c
if ((llep->ep & USBEPADRM_DIRECTION) == USBEPADR_DIR_OUT)
{
    if (transferLength <= 188)
    {
        xactPos   = EHCI_SITD_XACT_POS_ALL;
        xactCount = 1;
    }
    else
    {
        xactPos   = EHCI_SITD_XACT_POS_BEGIN;
        xactCount = ((transferLength - 1) / 188) + 1;
    }
}
```

Taken on its own this matches the EHCI specification: `TP = ALL` with
`T-Count = 1` for a payload that fits one start-split, `TP = BEGIN` with
`T-Count` start-splits otherwise. The single-split branch demonstrably works,
so the fault is somewhere in what the multi-split branch produces.

One concrete inconsistency, offered as a starting point rather than a
conclusion, since we could not instrument the host controller: the start-split
mask and the transaction count are derived from different quantities.
`ssplitMask` is computed once at scheduling time in
`ScheduleIsochronousEndPoint` from `ep->max_packet_size` - 384 bytes here,
which is `ceil(384/188) = 3` start-splits - while `xactCount` above is
recomputed per transfer from the actual length, giving
`ceil(192/188) = 2`. The schedule therefore reserves and masks three
start-split opportunities while each transfer declares two. The EHCI
specification expects the number of scheduled start-splits for an OUT siTD to
agree with T-Count, so this pairing looks worth checking first. Note the
working case has the same mismatch in a milder form (3 masked, T-Count 1), so
a simple disagreement is evidently tolerated somewhere - which suggests
looking at how the count is consumed rather than merely whether it matches.

## Workaround in use

Our driver now offers 44.1 kHz to the application even when the card does not
declare it in its Format Type I descriptor. At 44.1 kHz stereo 16-bit a frame
carries 176 bytes, below the boundary, so the transfer stays on the working
single-split path. The card accepts a `SET_CUR` for that rate despite not
advertising it, and playback is then correct - right pitch, no drift.

This is a workaround, not a fix. It depends on the card honouring an
undeclared rate - and note we cannot verify that directly, because this card
STALLs a `GET_CUR` on the sampling frequency control, so the only evidence
that it really switched is that playback sounds correct rather than 8.8% sharp.
It is also unavailable to any device whose formats all exceed 188 bytes per
frame. Full-speed isochronous OUT endpoints commonly do: 48 kHz
stereo 16-bit is the obvious case at 192 bytes, and anything with more
channels or higher resolution is well past it.

## The condition is not detectable by a function driver

We tried to have the driver notice the situation and pick a rate that works,
and could not. Querying the function with `USBA_HubPort` and
`USBA_HubHasMultiTT` leaves both values untouched - reading the headers again
afterwards, both are documented "for specifying", i.e. input tags rather than
queryable attributes. `USBA_HCD_Unit` reads 0 both on a root port and behind
the hub, and `USBA_DeviceSpeed` correctly reports full speed in both cases,
which is necessary but not sufficient.

So a function driver appears to have no way to learn that it sits below a
transaction translator, and therefore no way to know that some of the transfer
sizes its endpoint advertises will not be delivered. Combined with the silent
failure above, a driver can neither predict the problem nor observe it.

## Suggested fix

Make the multi-split isochronous OUT path deliver, so that any payload up to
the endpoint's `wMaxPacketSize` works behind a hub as it does on a root port.

If a full fix is not practical soon, it would still help a great deal to make
the failure visible rather than silent - returning an error from the submission
path for the payload sizes that cannot currently be delivered would at least
let drivers detect the condition and tell the user, instead of streaming into
a void that reports success.

## Contact

Happy to run further tests on this hardware, capture logs, or try patched
builds.
