#ifndef USBAUDIO_H
#define USBAUDIO_H

#include <exec/libraries.h>
#include <exec/types.h>
#include <dos/dos.h>
#include <libraries/ahi_sub.h>
#include <libraries/libusb-1.h>
#include <interfaces/libusb-1.h>

/* Debug print macro — controlled by -DDEBUG in the makefile.
 * When DEBUG is not defined all DPRINTF calls compile to nothing. */
#ifdef DEBUG
#define DPRINTF IExec->DebugPrintF
#else
#define DPRINTF(...) ((void)0)
#endif

#define LIBNAME  "usbaudio.audio"

#define MAX_USB_FREQUENCIES 24

/* USB Audio Class defines */
#define USB_AUDIO_SET_CUR                0x01
#define USB_AUDIO_GET_CUR                0x81
#define USB_AUDIO_SET_MIN                0x02
#define USB_AUDIO_GET_MIN                0x82
#define USB_AUDIO_SET_MAX                0x03
#define USB_AUDIO_GET_MAX                0x83
#define USB_AUDIO_SAMPLING_FREQ_CONTROL  (0x01 << 8)

/* USB Audio Class Feature Unit control selectors (bmaControls bitmap) */
#define USB_AUDIO_FU_MUTE_CONTROL        0x01
#define USB_AUDIO_FU_VOLUME_CONTROL      0x02

/* USB Audio Class bmRequestType for control interface */
#define USB_AUDIO_REQ_SET_IF  0x21  /* Host-to-Device | Class | Interface */
#define USB_AUDIO_REQ_GET_IF  0xA1  /* Device-to-Host | Class | Interface */

/* Maximum record samples per callback */
#define USB_AUDIO_REC_SAMPLES  1024

/* User-defined device names from ENVARC:USBAudio.prefs */
#define USBAUDIO_PREFS_PATH  "ENVARC:USBAudio.prefs"
#define MAX_USER_DEVICES      16

struct USBAudioUserDevice {
    uint16 vid;
    uint16 pid;
    char   name[48];
};

extern struct USBAudioUserDevice g_user_devices[MAX_USER_DEVICES];
extern int32 g_num_user_devices;

/* Rescan flag — set by libExpunge when deferred (OpenCnt > 0) */
extern int32 g_force_rescan;

void load_user_device_prefs(void);

/* Forward declarations */
struct USBAudioIFace;
struct AHIAudioCtrlDrv;

/* Function prototypes */
uint32 _usbaudio_AHIsub_Stop(struct USBAudioIFace *Self, uint32 Flags, struct AHIAudioCtrlDrv *AudioCtrl);
uint32 hwUSBPlaySlave(STRPTR *args UNUSED, int32 arglen UNUSED, struct ExecBase *sysbase);
uint32 hwUSBRecordSlave(STRPTR *args UNUSED, int32 arglen UNUSED, struct ExecBase *sysbase);

/* Exported interface pointers */
extern struct ExecIFace    *IExec;
extern struct DOSIFace     *IDOS;
extern struct UtilityIFace *IUtility;
extern struct Libusb1IFace *ILibusb1;
extern struct Library      *Libusb1Base;

/*
 * Per-output-mode info: one entry per playback alt setting.
 * Discovered during scan by reading the raw USB config descriptor.
 */
#define MAX_OUTPUT_MODES  8
#define MAX_INPUT_SOURCES 4

struct USBOutputMode {
    uint8  iface_num;
    uint8  alt_setting;
    uint8  ep_addr;
    uint16 max_pkt;
    uint8  nr_channels;
    uint8  subframe_size;
    uint8  bit_resolution;
    uint8  channel_offset;  /* Starting sample offset for this pair (0,2,4,6) */
    uint32 frequencies[MAX_USB_FREQUENCIES];
    int32  num_frequencies;
    char   name[48];        /* e.g. "Front", "Rear" */
};

struct USBInputSource {
    uint8  terminal_id;
    uint16 terminal_type;
    uint8  fu_unit_id;      /* Associated Feature Unit (0 if none) */
    int32  fu_found;
    uint8  su_pin;          /* 1-based pin in Selector Unit (0 = unknown) */
    char   name[48];        /* e.g. "Microphone", "SPDIF In" */
};

/*
 * USB Audio device information, discovered at scan time.
 * Stored globally so GetAttr can report capabilities
 * before AllocAudio is called.
 */
struct USBAudioDeviceInfo
{
    int32   scanned;            /* True if scan has been performed          */
    int32   found;              /* True if a USB audio device was found     */

    uint16  vid;                /* USB Vendor ID                            */
    uint16  pid;                /* USB Product ID                           */

    /* Playback (Audio Streaming OUT) */
    uint8   iface_num;          /* Audio Streaming interface number         */
    uint8   alt_setting;        /* Alt setting with isoch OUT endpoint      */
    uint8   ep_addr;            /* Isochronous OUT endpoint address         */
    uint8   nr_channels;        /* Number of audio channels (1 or 2)       */

    uint16  max_pkt;            /* Max packet size of isoch endpoint        */
    uint8   subframe_size;      /* Bytes per audio sample (1, 2, or 3)     */
    uint8   bit_resolution;     /* Bits per sample (8, 16, or 24)          */

    /* Recording (Audio Streaming IN) */
    int32   rec_found;          /* True if recording endpoint was found     */
    uint8   rec_iface_num;      /* Recording interface number               */
    uint8   rec_alt_setting;    /* Alt setting with isoch IN endpoint       */
    uint8   rec_ep_addr;        /* Isochronous IN endpoint address          */
    uint8   rec_nr_channels;    /* Number of recording channels             */
    uint16  rec_max_pkt;        /* Max packet size of IN endpoint           */
    uint8   rec_subframe_size;  /* Bytes per recording sample               */
    uint8   rec_bit_resolution; /* Bits per recording sample                */

    /* Audio Control - Feature Unit (output volume) */
    uint8   ac_iface_num;       /* Audio Control interface number           */
    uint8   fu_unit_id;         /* Feature Unit ID for output volume        */
    int32   fu_found;           /* True if output Feature Unit was found    */
    int32   fu_vol_failed;      /* True if volume query STALLed (skip next) */
    int32   fu_vol_cached;      /* True if volume range already queried OK  */
    int16   fu_vol_min;         /* Cached Volume MIN (1/256 dB)             */
    int16   fu_vol_max;         /* Cached Volume MAX (1/256 dB)             */
    int16   fu_vol_res;         /* Cached Volume RES (1/256 dB)             */

    /* Audio Control - Feature Unit (input/recording gain) */
    uint8   fu_rec_unit_id;     /* Feature Unit ID for input gain           */
    int32   fu_rec_found;       /* True if input Feature Unit was found     */
    int32   fu_rec_vol_failed;  /* True if input gain query STALLed         */
    int32   fu_rec_vol_cached;  /* True if input gain range already queried */
    int16   fu_rec_vol_min;     /* Cached Input Gain MIN                    */
    int16   fu_rec_vol_max;     /* Cached Input Gain MAX                    */
    int16   fu_rec_vol_res;     /* Cached Input Gain RES                    */

    /* Audio Control - Selector Unit (input source switch) */
    uint8   selector_unit_id;   /* Selector Unit ID (0 if none)             */
    uint8   selector_num_pins;  /* Number of input pins in SU               */
    uint8   selector_sources[MAX_INPUT_SOURCES]; /* Source IDs for each pin */

    /* Supported sample rates from Format Type I descriptor */
    uint32  frequencies[MAX_USB_FREQUENCIES];
    int32   num_frequencies;

    /* Native frequency derived from maxpkt */
    uint32  native_freq;

    /* Human-readable name for AHI Prefs output selector */
    char    name[64];

    /* Config descriptor checksum for device deduplication */
    uint32  config_checksum;

    /* Output modes (one per playback alt setting), enumerated during scan */
    struct USBOutputMode  outputs[MAX_OUTPUT_MODES];
    int32                 num_outputs;
    int32                 selected_output;

    /* Input sources (different input terminals), enumerated during scan */
    struct USBInputSource inputs[MAX_INPUT_SOURCES];
    int32                 num_inputs;
    int32                 selected_input;
};

/* Maximum number of USB audio devices the driver can track */
#define MAX_USB_AUDIO_DEVICES 8

/* Active/selected device (used by AllocAudio, GetAttr, Start, etc.) */
extern struct USBAudioDeviceInfo g_usb_info;

/* All discovered USB audio devices */
extern struct USBAudioDeviceInfo g_usb_devices[MAX_USB_AUDIO_DEVICES];
extern int32 g_usb_num_devices;

/* Flattened output/input arrays for AHI Prefs UI.
 * Each entry references a (device, mode) or (device, source) pair. */
#define MAX_TOTAL_OUTPUTS 32
#define MAX_TOTAL_INPUTS  16

struct USBAudioFlatOutput {
    int32  device_idx;       /* Index into g_usb_devices[] */
    int32  mode_idx;         /* Index into g_usb_devices[].outputs[] */
    char   name[64];         /* "USB Audio (VID:PID) Stereo" */
};

struct USBAudioFlatInput {
    int32  device_idx;
    int32  source_idx;       /* Index into g_usb_devices[].inputs[] */
    char   name[64];         /* "USB Audio (VID:PID) Microphone" */
};

extern struct USBAudioFlatOutput g_usb_flat_outputs[MAX_TOTAL_OUTPUTS];
extern int32 g_num_flat_outputs;
extern struct USBAudioFlatInput  g_usb_flat_inputs[MAX_TOTAL_INPUTS];
extern int32 g_num_flat_inputs;

/* Scan for USB audio devices — called lazily */
void scan_usb_audio_device(void);

/*
 * Library base structure for the AHI sub-driver
 */
struct USBAudioBase
{
    struct Library libNode;
    BPTR segList;
};

/*
 * DriverData base — required by AHI but unused here
 */
struct DriverData
{
};

/*
 * Per-audio-ctrl driver data, stored in AudioCtrl->ahiac_DriverData.
 * Allocated in AllocAudio, freed in FreeAudio.
 */
struct USBAudioData
{
    struct DriverData    ua_DriverData;
    uint8                ua_Flags;
    uint8                ua_Pad1;
    int8                 ua_MasterSignal;
    int8                 ua_SlaveSignal;
    struct Process      *ua_MasterTask;
    struct Process      *ua_SlaveTask;
    struct USBAudioBase *ua_AHIsubBase;

    /* USB device identifiers (copied from g_usb_info) */
    uint16               ua_VendorID;
    uint16               ua_ProductID;
    uint8                ua_InterfaceNum;
    uint8                ua_AltSetting;
    uint8                ua_EndpointAddr;
    uint8                ua_NumChannels;
    uint16               ua_MaxPacketSize;
    uint8                ua_SubframeSize;
    uint8                ua_BitResolution;
    uint8                ua_ChannelOffset;   /* Which channel pair (0,2,4,6) */

    /* Mixing / output buffers (triple-buffered for glitch-free playback) */
    APTR                 ua_MixBuffer;
    uint8               *ua_USBBuffer;
    uint8               *ua_USBBuffer2;     /* Second USB buffer */
    uint8               *ua_USBBuffer3;     /* Third USB buffer (triple-buffering) */
    uint32               ua_USBBufferSize;

    /* libusb handles (opened in Start, closed in Stop) */
    libusb_device_handle *ua_DevHandle;

    /* ---- Volume / Monitor / Gain ---- */
    Fixed                ua_OutputVolume;    /* AHI Fixed 16.16, 0x10000 = 0dB */
    Fixed                ua_MonitorVolume;   /* AHI Fixed 16.16 */
    Fixed                ua_InputGain;       /* AHI Fixed 16.16 */
    uint32               ua_Input;           /* Currently selected input index */
    uint32               ua_Output;          /* Currently selected output index */

    /* Audio Control Feature Unit for USB volume (output) */
    uint8                ua_FUnitID;         /* Feature Unit bUnitID */
    uint8                ua_ACIfaceNum;      /* Audio Control interface number */
    int16                ua_VolMin;          /* USB vol min (1/256 dB) */
    int16                ua_VolMax;          /* USB vol max (1/256 dB) */
    int16                ua_VolRes;          /* USB vol resolution */

    /* Audio Control Feature Unit for USB input gain (recording) */
    uint8                ua_RecFUnitID;      /* Feature Unit bUnitID for input */
    uint8                ua_SelectorUnitID;  /* Selector Unit ID (input mux)  */
    uint8                ua_SelectorPin;     /* Desired SU pin (1-based)      */
    int16                ua_RecVolMin;       /* USB input vol min (1/256 dB) */
    int16                ua_RecVolMax;       /* USB input vol max (1/256 dB) */
    int16                ua_RecVolRes;       /* USB input vol resolution */

    /* ---- Recording ---- */
    int8                 ua_RecSlaveSignal;
    uint8                ua_RecPad;
    struct Process      *ua_RecSlaveTask;
    uint8                ua_RecInterfaceNum;
    uint8                ua_RecAltSetting;
    uint8                ua_RecEndpointAddr;
    uint8                ua_RecNumChannels;
    uint16               ua_RecMaxPacketSize;
    uint8                ua_RecSubframeSize;
    uint8                ua_RecBitResolution;
    uint8               *ua_RecBuffer;
    uint8               *ua_RecBuffer2;
    uint32               ua_RecBufferSize;
    int32                ua_IsRecording;     /* TRUE while recording */
    int32                ua_DeviceGone;      /* TRUE if USB device was hot-removed */
};

#endif /* USBAUDIO_H */
