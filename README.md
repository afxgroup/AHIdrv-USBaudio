# usbaudio.audio

AHI sub-driver for USB Audio Class devices on AmigaOS 4.

## Overview

`usbaudio.audio` is an **AHI sub-driver** (audio hardware interface driver) for AmigaOS 4 that provides playback and recording through USB Audio Class (UAC 1.0) devices — USB sound cards, DACs, headsets, and similar peripherals.

It communicates with the USB stack via `libusb-1.library`, which wraps the Sirion `usbsys.device` underneath.  
Hotplug (attach/detach without reboot) is handled in cooperation with [`usbaudio.usbfd`](../../usbaudio_usbfd/README.md).

## Features

- USB Audio Class 1.0 playback (isochronous OUT)
- USB Audio Class 1.0 recording (isochronous IN)
- Automatic device discovery at first use (lazy scan)
- Multiple USB audio devices supported simultaneously
- Per-device output mode selection (multiple alt-settings)
- Volume/gain control via Feature Unit (GET_MIN / GET_MAX / GET_RES / GET_CUR / SET_CUR)
- Volume range cached after first query — no repeated USB control transfers on re-open
- SET_CUR sample rate (UAC 1.0 endpoint frequency control)
- 44.1 kHz always offered as a fallback rate — see *Sample rate and USB hubs* below
- Hotplug: safe detach during active playback — no freeze on device removal
- Serial debug output guarded by `#ifdef DEBUG` (zero overhead in release builds)
- User-defined device names via `ENVARC:USBAudio.prefs`

## Architecture

```
AHI framework
     │
     ▼
usbaudio.audio  (AHI sub-driver — this library)
     │
     ├── scan_usb_audio_device()     lazy USB scan on first AllocAudio/GetAttr
     ├── AHIsub_AllocAudio()         allocates per-instance driver data
     ├── AHIsub_Start()              opens device, claims interface, launches slaves
     ├── AHIsub_Stop()               stops slaves, releases interface (safe if hot-removed)
     ├── AHIsub_FreeAudio()          frees per-instance data
     ├── AHIsub_SetVol/GetAttr/…     AHI attribute handling
     │
     ├── PlaySlave  (separate task)  async isochronous OUT pipeline
     └── RecordSlave (separate task) async isochronous IN pipeline
           │
           ▼
     libusb-1.library  →  usbsys.device  →  Sirion USB stack
```

### Lazy USB scan

`libOpen` does **no** USB access — it only increments the open count.  
All USB work (opening `libusb-1.library`, scanning devices) is deferred to `scan_usb_audio_device()`, called from `AHIsub_AllocAudio` and `AHIsub_GetAttr`. This prevents a deadlock during boot when `usbaudio.usbfd` triggers `AddAudioModes REFRESH` while the USB stack is still initialising.

### Hotplug

When the USB device is physically removed during playback:
- PlaySlave / RecordSlave detect 3 consecutive fatal transfer errors and set `ua_DeviceGone = 1` before exiting.
- `AHIsub_Stop` checks `ua_DeviceGone`: if set, it skips `set_alt_setting` / `release_interface` (which would block on a dead device) and only calls `libusb_close`.
- `usbaudio.usbfd` detects the detach via `USBNM_TYPE_INTERFACEDETACH`, calls `RemLibrary` (force-expunge), then `C:AddAudioModes REFRESH`, so AHI re-discovers the device when it is plugged back in.

### Async isochronous pipeline

The playback and recording slaves use `SendIO` to pre-queue multiple `USBIOReq` isochronous requests with the USB stack (modelled after *usbaudio2* by Chris Handley). When one request completes, it is immediately refilled and re-sent, maintaining a continuous pipeline that is resilient to task scheduling jitter.

## Sample rate and USB hubs

**If audio is silent when the card is connected through a USB hub, select
44100 Hz in AHI Prefs.**

This is the driver working around a defect in the AmigaOS EHCI host controller
driver, not a limitation of the card. The rest of this section explains why,
since the symptom is confusing and easy to misdiagnose.

### The symptom

A full-speed USB Audio card plays correctly on a root port and is completely
silent behind a high-speed hub. Enumeration, descriptors, interface claiming
and alternate-setting selection are all identical in both cases. Every
isochronous transfer completes with `USBERR_NOERROR`, the reported transferred
byte counts are identical to the working case, and nothing appears in any log.

### The cause

The EHCI host controller splits a full-speed isochronous OUT transfer into
188-byte start-split transactions (`EHCI_MAX_ISO_SPLIT_PAYLOAD_BYTES` in the
HCD's `isochronous.c`). At or below 188 bytes per USB frame it issues a single
start-split; above it, a multi-split sequence. **The multi-split path does not
deliver the data.**

Which path is used follows directly from the sample rate:

| Rate | Bytes per frame (stereo, 16-bit) | EHCI path | Result |
|---|---|---|---|
| 48000 Hz | 192 | multi-split | silence |
| 44100 Hz | 176 | single-split | works |

This only matters behind a hub. A full-speed device on an EHCI root port is
handed to the companion OHCI/UHCI controller and never reaches this code —
which is why the same card works when plugged in directly.

The failure is silent because isochronous OUT has no handshake: the host
transmits and never learns whether the device received anything, so nothing
below the driver can report the loss.

### The workaround

The driver adds 44100 Hz to the rate list it publishes to AHI whenever the
card does not already declare it. Cards that report the UAC sampling frequency
control generally honour a `SET_CUR` for 44.1 kHz whether or not they list it.

The rate is offered, not forced: every rate the card declares is still
available, and the choice stays with the user in AHI Prefs. Selecting 44.1 kHz
costs nothing on a direct connection.

### Known limitation

The driver **cannot detect that it is behind a hub**, so it cannot make this
choice automatically. `USBA_HubPort` and `USBA_HubHasMultiTT` are input tags
for specifying a hub port rather than queryable attributes, and
`USBA_HCD_Unit` reads the same value in both configurations. A function driver
therefore has no way to learn that it sits below a transaction translator.

The practical consequence: an application that explicitly requests 48 kHz will
still be silent behind a hub, because 48 kHz remains a legitimate rate that
the card declares and the driver has no basis for refusing. Setting AHI's
mixing frequency to 44.1 kHz covers the normal case, since AHI resamples
application audio to the mixing rate.

`EHCI-iso-split-bug.md` in the repository root is a write-up of this defect
intended for the USB stack maintainers.

## Project structure

```
AHIdrv-USBaudio/
└── Source/
    ├── Makefile
    ├── DevInit/
    │   ├── AHIdrv-USBaudioInit.c      ROMTAG, libInit/libOpen/libClose/libExpunge
    │   └── AHIdrv-USBaudioVectors.c   AHI sub-driver interface vector table
    ├── device/
    │   ├── AHIdrv-USBaudioMain.c      Core AHI functions: Alloc/Free/Start/Stop/GetAttr/SetVol/…
    │   └── AHIdrv-USBaudioAccel.c     Sample format conversion helpers
    ├── hw/
    │   ├── AHIdrv-hwUSBPlayProc.c     Playback slave task (isochronous OUT)
    │   └── AHIdrv-hwUSBRecordProc.c   Recording slave task (isochronous IN)
    ├── includes/
    │   ├── AHIdrv-USBaudio.h          Shared structs, defines, DPRINTF macro
    │   └── AHIdrv-USBaudio.audio_rev.h Version/revision strings
    ├── interfaces/                    Local AHI sub-driver interface headers
    └── proto/                         Local proto headers
```

## Building

Requires the `ppc-amigaos-gcc` cross-compiler (AmigaOS 4 SDK) and the `libusb-1` SDK headers (included in the repository at `../../libusb-1/SDK/`).

```sh
make        # release build — no debug output
make clean  # remove build artefacts
```

Output files: `usbaudio.audio`, `usbaudio.audio.debug`, `USBAUDIO` (AHI mode file).

### Debug build

```sh
make DEBUG="-DDEBUG"          # serial debug prints only
make DEBUG="-gstabs -DDEBUG"  # debug prints + STABS symbols
```

All `DPRINTF(...)` calls compile to nothing in release builds (`DEBUG` not defined).

### Override C runtime

```sh
make CRT=clib2
```

Default is `newlib`.

## Installation

| File | Destination |
|---|---|
| `usbaudio.audio` | `SYS:Storage/AudioModes/` or `DEVS:AHI/` |
| `USBAUDIO` | `DEVS:AudioModes/` |

Also requires:
- `libusb-1.library` → `SYS:Libs/`
- `usbaudio.usbfd` + `usbaudio.fdclass` → `SYS:Storage/USBFDClasses/Audio/`

## Dependencies

- AmigaOS 4.1 with AHI v6+
- `libusb-1.library` (included in this repository)
- `usbresource.library` v53+ (Sirion USB stack)
- `usbsys.device`
- A USB Audio Class 1.0 compatible device
