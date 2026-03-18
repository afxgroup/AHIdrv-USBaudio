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
