/*
 * AHIdrv-USBaudioMain.c
 *
 * Core AHI sub-driver functions for USB Audio Class output.
 *
 * AllocAudio — discovers USB audio device, stores capabilities.
 * FreeAudio  — releases per-instance data.
 * Start      — opens USB device, starts playback slave process.
 * Stop       — stops playback, releases USB device.
 * GetAttr    — reports device capabilities to AHI.
 * Disable/Enable — Disable/Enable pair (prevents interrupts during critical sections).
 * Update     — nothing to do (mixing is handled by slave).
 */

#include <exec/exec.h>
#include <exec/libraries.h>
#include <proto/exec.h>
#include <dos/dos.h>
#include <dos/dostags.h>
#include <proto/dos.h>
#include <exec/types.h>
#include <libraries/ahi_sub.h>
#include <proto/usbaudio.h>
#include <utility/utility.h>
#include <proto/utility.h>
#include <devices/timer.h>
#include <proto/timer.h>
#include <usb/system.h>
#include <proto/usbresource.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "includes/AHIdrv-USBaudio.h"
#include "includes/AHIdrv-USBaudio.audio_rev.h"

/* Shortcut to per-audio-ctrl driver data */
#define dd (*((struct USBAudioData **)&AudioCtrl->ahiac_DriverData))

/* Global tracking for the active USB audio session.
 * AHI may create a new session (AllocAudio) without properly tearing
 * down the previous one (no FreeAudio/Stop).  If that happens, the old
 * device handle is leaked and the USB interface stays claimed, causing
 * set_alt_setting to fail on the next Start.  By tracking the handle
 * globally, Start can force-cleanup a stale session before proceeding. */
static libusb_device_handle *g_active_handle = NULL;
static int32                 g_active_iface  = -1;
static int32                 g_active_rec_iface = -1;
static struct Process       *g_active_slave = NULL;
static int8                  g_active_slave_sig = -1;
static struct Process       *g_active_rec_slave = NULL;
static int8                  g_active_rec_sig = -1;
static int8                  g_active_master_sig = -1;
static struct Task          *g_active_master_task = NULL;

/* Set to 1 once we have confirmed USBNM_TYPE_STACKFULLBOOTED.
 * Never cleared — once the stack is booted, we never wait again. */
static int32                 g_stack_booted = 0;

/* Forward declarations for helper functions */
static void set_usb_selector(struct USBAudioData *ua, uint8 pin);

/*
 * load_user_device_prefs
 *
 * Reads ENVARC:USBAudio.prefs to load user-defined VID:PID=Name entries.
 * File format (one entry per line):
 *   ; comment line
 *   VID:PID=Device Name
 *
 * Example:
 *   ; My USB audio devices
 *   0D8C:013C=C-Media CM108
 *   1130:F211=Tenx TP6911 Headset
 */
static uint16 hex4(const char *s)
{
    uint16 v = 0;
    int32 i;
    for (i = 0; i < 4; i++)
    {
        uint8 c = (uint8)s[i];
        v <<= 4;
        if (c >= '0' && c <= '9')      v |= (uint16)(c - '0');
        else if (c >= 'A' && c <= 'F') v |= (uint16)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') v |= (uint16)(c - 'a' + 10);
        else return 0;
    }
    return v;
}

void load_user_device_prefs(void)
{
    BPTR fh;
    char line[128];
    int32 n;

    g_num_user_devices = 0;

    fh = IDOS->Open(USBAUDIO_PREFS_PATH, MODE_OLDFILE);
    if (fh == ZERO)
    {
        DPRINTF("[USBAudio] prefs: %s not found (OK)\n", USBAUDIO_PREFS_PATH);
        return;
    }

    DPRINTF("[USBAudio] prefs: reading %s\n", USBAUDIO_PREFS_PATH);

    while (g_num_user_devices < MAX_USER_DEVICES)
    {
        /* Read one line (FGets reads up to size-1 chars or newline) */
        if (IDOS->FGets(fh, line, (uint32)sizeof(line)) == NULL)
            break;

        /* Strip trailing newline/CR */
        for (n = 0; line[n]; n++) {}
        while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r'))
            line[--n] = '\0';

        /* Skip empty lines and comments */
        if (n == 0 || line[0] == ';' || line[0] == '#')
            continue;

        /* Expect format: XXXX:XXXX=Name (minimum 10 chars: 4+1+4+1) */
        if (n < 10 || line[4] != ':' || line[9] != '=')
        {
            DPRINTF("[USBAudio] prefs: skipping bad line: \"%s\"\n", line);
            continue;
        }

        {
            struct USBAudioUserDevice *ud = &g_user_devices[g_num_user_devices];
            int32 k;

            ud->vid = hex4(&line[0]);
            ud->pid = hex4(&line[5]);

            if (ud->vid == 0 && ud->pid == 0)
            {
                DPRINTF("[USBAudio] prefs: invalid VID:PID in \"%s\"\n", line);
                continue;
            }

            /* Copy name (after '=') */
            for (k = 0; line[10 + k] && k < 47; k++)
                ud->name[k] = line[10 + k];
            ud->name[k] = '\0';

            DPRINTF("[USBAudio] prefs: added %04lx:%04lx = \"%s\"\n",
                               (ULONG)ud->vid, (ULONG)ud->pid, ud->name);
            g_num_user_devices++;
        }
    }

    IDOS->Close(fh);
    DPRINTF("[USBAudio] prefs: loaded %ld user device(s)\n", (LONG)g_num_user_devices);
}


/*
 * scan_usb_audio_device
 *
 * Class-based scan — detects ANY USB Audio Class device, regardless of
 * VID/PID.  Uses libusb_get_config_descriptor() which queries the
 * Sirion USB stack's cached descriptors (NO I/O to the device).
 * Looks for Audio Streaming interfaces (class 0x01, subclass 0x02)
 * with isochronous OUT (playback) or IN (recording) endpoints.
 *
 * Detailed endpoint/format info is discovered later via
 * enumerate_device_modes() which reads the full config descriptor.
 *
 * Results stored in g_usb_devices[] and g_usb_info (first device).
 * VID/PID is used only to generate human-readable names, NOT for
 * device filtering — any compliant USB Audio Class 1.0 device works.
 */

/* Helper: generate human-readable name for a USB audio device */
static void make_device_name(struct USBAudioDeviceInfo *dev)
{
    /* Check user-defined device names first (from ENVARC:USBAudio.prefs) */
    const char *label = NULL;
    {
        int32 ui;
        for (ui = 0; ui < g_num_user_devices; ui++)
        {
            if (g_user_devices[ui].vid == dev->vid &&
                g_user_devices[ui].pid == dev->pid)
            {
                label = g_user_devices[ui].name;
                break;
            }
        }
    }

    /* Hardcoded VID/PID mappings (fallback) */
    if (label == NULL)
    {
        if (dev->vid == 0x1B3F && dev->pid == 0x2008)
            label = "USB Audio (1B3F:2008)";
        else if (dev->vid == 0x0D8C && dev->pid == 0x0102)
            label = "USB Audio (0D8C:0102)";
        else if (dev->vid == 0x0D8C && dev->pid == 0x013C)
            label = "C-Media CM108";
        else if (dev->vid == 0x0D8C && dev->pid == 0x000C)
            label = "C-Media Audio";
        else if (dev->vid == 0x0D8C && dev->pid == 0x0014)
            label = "Unitek Y-247A";
        else if (dev->vid == 0x041E && dev->pid == 0x324D)
            label = "Creative Sound Blaster Play! 3";
        else if (dev->vid == 0x1B3F && dev->pid == 0x2008)
            label = "GeneralPlus USB Audio";    
        else if (dev->vid == 0x1130 && dev->pid == 0xF211)
            label = "Tenx TP6911 Headset";
    }

    if (label)
    {
        int32 k;
        for (k = 0; label[k] && k < 63; k++)
            dev->name[k] = label[k];
        dev->name[k] = '\0';
    }
    else
    {
        /* Generic name from VID:PID */
        static const char hex[] = "0123456789ABCDEF";
        const char *pfx = "USB Audio (";
        int32 k = 0, p;

        for (p = 0; pfx[p]; p++)
            dev->name[k++] = pfx[p];

        dev->name[k++] = hex[(dev->vid >> 12) & 0xF];
        dev->name[k++] = hex[(dev->vid >>  8) & 0xF];
        dev->name[k++] = hex[(dev->vid >>  4) & 0xF];
        dev->name[k++] = hex[ dev->vid        & 0xF];
        dev->name[k++] = ':';
        dev->name[k++] = hex[(dev->pid >> 12) & 0xF];
        dev->name[k++] = hex[(dev->pid >>  8) & 0xF];
        dev->name[k++] = hex[(dev->pid >>  4) & 0xF];
        dev->name[k++] = hex[ dev->pid        & 0xF];
        dev->name[k++] = ')';
        dev->name[k]   = '\0';
    }
}

/*
 * enumerate_device_modes
 *
 * Opens a USB device briefly during scan to read its raw configuration
 * descriptor. Enumerates all playback alt settings (output modes) and
 * input terminals (input sources), storing them in the device info struct.
 * This allows GetAttr to report accurate outputs/inputs before Start().
 */
static void enumerate_device_modes(libusb_device *usbdev,
                                   struct USBAudioDeviceInfo *dev)
{
    libusb_device_handle *handle = NULL;
    /* USB Audio descriptors can be large (PCM2902 with 6 alt-settings is ~1600
     * bytes).  Use a heap buffer so we don't blow the stack and so we can
     * handle any device up to 8 KB without recompiling. */
    #define ENUM_BUF_SIZE 8192
    uint8 *buf = NULL;
    int32 r, pos;
    int32 current_ifc = -1, current_alt = -1;
    int32 current_class = 0, current_subclass = 0;
    uint16 total_len;

    /* Topology tracking */
    #define MAX_ENUM_UNITS 16
    struct { uint8 id; uint16 type; } enum_it[MAX_ENUM_UNITS];
    struct { uint8 id; uint8  src;  } enum_fu[MAX_ENUM_UNITS];
    int32 n_it = 0, n_fu = 0;
    uint8 enum_ac_iface = 0;

    /* Selector Unit tracking */
    uint8 su_id = 0;
    uint8 su_sources[MAX_INPUT_SOURCES];
    uint8 su_num_pins = 0;

    /* Output Terminal tracking: non-USB-Streaming OTs are physical outputs */
    struct { uint8 id; uint16 type; } enum_ot[8];
    int32 n_ot = 0;

    /* Current alt-setting's format info (reset on each new Interface desc) */
    int32 cur_ch = 0, cur_sub = 0, cur_bits = 0, cur_valid = 0;
    uint32 cur_freq[MAX_USB_FREQUENCIES];
    int32  cur_nfreq = 0;

    /* Best playback alt setting: we pick the one with the most channels */
    int32  best_play_ch   = 0;
    uint8  best_play_ifc  = 0, best_play_alt = 0, best_play_ep = 0;
    uint16 best_play_pkt  = 0;
    uint8  best_play_sub  = 0, best_play_bits = 0;
    uint32 best_play_freq[MAX_USB_FREQUENCIES];
    int32  best_play_nfreq = 0;
    int32  best_play_rate_ctrl = 0; /* from CS_ENDPOINT bmAttributes D0 */
    int32  last_was_play_ep    = 0; /* set when we just updated best_play_* */

    dev->num_outputs     = 0;
    dev->num_inputs      = 0;
    dev->selected_output = 0;
    dev->selected_input  = 0;

    /* Reset recording fields so we can detect them from the raw config
       descriptor (the basic scan may have set rec_found=1 with EP=0x00) */
    dev->rec_found          = 0;
    dev->rec_ep_addr        = 0;
    dev->rec_iface_num      = 0;
    dev->rec_alt_setting    = 0;
    dev->rec_max_pkt        = 0;
    dev->rec_nr_channels    = 0;
    dev->rec_subframe_size  = 0;
    dev->rec_bit_resolution = 0;

    /* Open the device to read raw config descriptor */
    r = ILibusb1->libusb_open(usbdev, &handle);
    if (r != 0 || handle == NULL)
    {
        DPRINTF("[USBAudio] enumerate: cannot open device VID=0x%04lx PID=0x%04lx (err=%ld)\n",
                           (ULONG)dev->vid, (ULONG)dev->pid, r);
        return;
    }

    buf = IExec->AllocVecTags(ENUM_BUF_SIZE, AVT_Type, MEMF_SHARED, TAG_END);
    if (buf == NULL)
    {
        DPRINTF("[USBAudio] enumerate: out of memory\n");
        ILibusb1->libusb_close(handle);
        return;
    }

    /* Read config descriptor header for total length */
    r = ILibusb1->libusb_control_transfer(handle,
            0x80, 0x06, (0x02 << 8) | 0, 0, buf, 9, 5000);
    if (r < 9)
    {
        DPRINTF("[USBAudio] enumerate: config header failed (%ld)\n", r);
        IExec->FreeVec(buf);
        ILibusb1->libusb_close(handle);
        return;
    }

    total_len = (uint16)(buf[2] | (buf[3] << 8));
    if (total_len > ENUM_BUF_SIZE)
    {
        DPRINTF("[USBAudio] enumerate: WARNING: config descriptor too large (%ld > %ld), truncating!\n",
                           (LONG)total_len, (LONG)ENUM_BUF_SIZE);
        total_len = ENUM_BUF_SIZE;
    }

    r = ILibusb1->libusb_control_transfer(handle,
            0x80, 0x06, (0x02 << 8) | 0, 0, buf, total_len, 5000);
    if (r < 9)
    {
        DPRINTF("[USBAudio] enumerate: full config read failed (%ld)\n", r);
        IExec->FreeVec(buf);
        ILibusb1->libusb_close(handle);
        return;
    }
    total_len = (uint16)r;

    /* Compute simple checksum for deduplication */
    {
        uint32 cksum = 0;
        int32 ci;
        for (ci = 0; ci < total_len; ci++)
            cksum += buf[ci];
        dev->config_checksum = cksum;
    }

    DPRINTF("[USBAudio] enumerate: config total_len=%ld for %04lx:%04lx\n",
                       (LONG)total_len, (ULONG)dev->vid, (ULONG)dev->pid);

    /* Parse raw descriptor blob */
    pos = 0;
    while (pos + 1 < total_len)
    {
        uint8 dsc_len  = buf[pos];
        uint8 dsc_type = buf[pos + 1];

        if (dsc_len < 2 || pos + dsc_len > total_len)
            break;

        /* ---- Interface descriptor ---- */
        if (dsc_type == 0x04 && dsc_len >= 9)
        {
            current_ifc      = buf[pos + 2];
            current_alt      = buf[pos + 3];
            current_class    = buf[pos + 5];
            current_subclass = buf[pos + 6];
            cur_valid = 0;  /* Reset format tracker for new alt */
            last_was_play_ep = 0;  /* No pending CS_ENDPOINT from previous alt */
        }
        /* ---- CS_INTERFACE: Input Terminal (Audio Control) ---- */
        else if (dsc_type == 0x24 && dsc_len >= 12 &&
                 buf[pos + 2] == 0x02 &&
                 current_class == 0x01 && current_subclass == 0x01)
        {
            uint8  tid   = buf[pos + 3];
            uint16 ttype = (uint16)(buf[pos + 4] | (buf[pos + 5] << 8));

            if (n_it < MAX_ENUM_UNITS)
            {
                enum_it[n_it].id   = tid;
                enum_it[n_it].type = ttype;
                n_it++;
            }
            enum_ac_iface = (uint8)current_ifc;

            /* Non-USB-Streaming terminals are selectable input sources */
            if (ttype != 0x0101 && dev->num_inputs < MAX_INPUT_SOURCES)
            {
                struct USBInputSource *inp = &dev->inputs[dev->num_inputs];
                const char *iname;

                inp->terminal_id   = tid;
                inp->terminal_type = ttype;
                inp->fu_unit_id    = 0;
                inp->fu_found      = 0;

                /* Name from terminal type */
                if      ((ttype & 0xFF00) == 0x0200)  iname = "Microphone";
                else if (ttype == 0x0601)             iname = "Analog In";
                else if (ttype == 0x0603)             iname = "Line In";
                else if (ttype == 0x0605)             iname = "SPDIF In";
                else if ((ttype & 0xFF00) == 0x0500)  iname = "Embedded";
                else                                  iname = "Input";

                {
                    int32 k;
                    for (k = 0; iname[k] && k < 47; k++)
                        inp->name[k] = iname[k];
                    inp->name[k] = '\0';
                }

                DPRINTF("[USBAudio] enumerate: input #%ld \"%s\" IT=%ld type=0x%04lx\n",
                                   (LONG)dev->num_inputs, inp->name,
                                   (LONG)tid, (LONG)ttype);
                dev->num_inputs++;
            }
        }
        /* ---- CS_INTERFACE: Feature Unit (Audio Control) ---- */
        else if (dsc_type == 0x24 && dsc_len >= 7 &&
                 buf[pos + 2] == 0x06 &&
                 current_class == 0x01 && current_subclass == 0x01)
        {
            if (n_fu < MAX_ENUM_UNITS)
            {
                enum_fu[n_fu].id  = buf[pos + 3];
                enum_fu[n_fu].src = buf[pos + 4];
                n_fu++;
            }
            enum_ac_iface = (uint8)current_ifc;
        }
        /* ---- CS_INTERFACE: Selector Unit (Audio Control) ---- */
        else if (dsc_type == 0x24 && dsc_len >= 6 &&
                 buf[pos + 2] == 0x05 &&
                 current_class == 0x01 && current_subclass == 0x01)
        {
            uint8 uid     = buf[pos + 3];
            uint8 nr_pins = buf[pos + 4];
            int32 pi;

            su_id       = uid;
            su_num_pins = 0;

            for (pi = 0; pi < nr_pins && pi < MAX_INPUT_SOURCES; pi++)
            {
                /* Only count a pin if its source byte is actually present in
                 * the descriptor — otherwise su_sources[pi] would stay
                 * uninitialized yet be treated as a valid source below. */
                if (pos + 5 + pi >= total_len)
                    break;
                su_sources[pi] = buf[pos + 5 + pi];
                su_num_pins++;
            }

            DPRINTF("[USBAudio] enumerate: Selector Unit ID=%ld pins=%ld srcs=[",
                               (LONG)su_id, (LONG)su_num_pins);
            for (pi = 0; pi < su_num_pins; pi++)
                DPRINTF("%ld%s", (LONG)su_sources[pi],
                                   pi < su_num_pins - 1 ? "," : "");
            DPRINTF("]\n");
        }
        /* ---- CS_INTERFACE: Output Terminal (Audio Control) ---- */
        else if (dsc_type == 0x24 && dsc_len >= 9 &&
                 buf[pos + 2] == 0x03 &&
                 current_class == 0x01 && current_subclass == 0x01)
        {
            uint16 otype = (uint16)(buf[pos + 4] | (buf[pos + 5] << 8));

            /* Non-USB-Streaming OTs are physical output destinations */
            if (otype != 0x0101 && n_ot < 8)
            {
                enum_ot[n_ot].id   = buf[pos + 3];
                enum_ot[n_ot].type = otype;
                n_ot++;
            }
        }
        /* ---- CS_INTERFACE: Format Type I ---- */
        else if (dsc_type == 0x24 && dsc_len >= 8 &&
                 buf[pos + 2] == 0x02 && buf[pos + 3] == 0x01)
        {
            if (current_class == 0x01 && current_subclass == 0x02 &&
                current_alt > 0)
            {
                cur_ch    = buf[pos + 4];
                cur_sub   = buf[pos + 5];
                cur_bits  = buf[pos + 6];
                cur_valid = 1;

                /* Parse supported frequencies */
                cur_nfreq = 0;
                {
                    uint8 bSamFreqType = buf[pos + 7];
                    if (bSamFreqType == 0 && dsc_len >= 14)
                    {
                        uint32 fmin = buf[pos+8]  | (buf[pos+9]<<8)  | (buf[pos+10]<<16);
                        uint32 fmax = buf[pos+11] | (buf[pos+12]<<8) | (buf[pos+13]<<16);
                        static const uint32 common[] = {
                            8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000
                        };
                        int32 fi;
                        for (fi = 0; fi < 8; fi++)
                        {
                            if (common[fi] >= fmin && common[fi] <= fmax &&
                                cur_nfreq < MAX_USB_FREQUENCIES)
                                cur_freq[cur_nfreq++] = common[fi];
                        }
                    }
                    else
                    {
                        int32 nf = bSamFreqType;
                        int32 fi;
                        if (nf > MAX_USB_FREQUENCIES) nf = MAX_USB_FREQUENCIES;
                        for (fi = 0; fi < nf && (pos + 8 + fi*3 + 2) < total_len; fi++)
                        {
                            int32 off = pos + 8 + fi * 3;
                            cur_freq[cur_nfreq++] =
                                buf[off] | (buf[off+1] << 8) | (buf[off+2] << 16);
                        }
                    }
                }
            }
        }
        /* ---- Standard Endpoint descriptor ---- */
        else if (dsc_type == 0x05 && dsc_len >= 7)
        {
            uint8  ep_addr   = buf[pos + 2];
            uint8  ep_attr   = buf[pos + 3];
            uint16 ep_maxpkt = (uint16)(buf[pos + 4] | (buf[pos + 5] << 8));

            /* Isochronous OUT = playback: remember the alt setting
             * with the MOST channels.  We'll create per-pair output
             * entries after the parse loop. */
            if (cur_valid &&
                current_class == 0x01 && current_subclass == 0x02 &&
                (ep_attr & 0x03) == 0x01 &&    /* Isochronous */
                (ep_addr & 0x80) == 0x00 &&    /* OUT */
                current_alt > 0)
            {
                if (cur_ch > best_play_ch)
                {
                    best_play_ch   = cur_ch;
                    best_play_ifc  = (uint8)current_ifc;
                    best_play_alt  = (uint8)current_alt;
                    best_play_ep   = ep_addr;
                    best_play_pkt  = ep_maxpkt;
                    best_play_sub  = (uint8)cur_sub;
                    best_play_bits = (uint8)cur_bits;
                    best_play_nfreq = cur_nfreq;
                    {
                        int32 fi;
                        for (fi = 0; fi < cur_nfreq && fi < MAX_USB_FREQUENCIES; fi++)
                            best_play_freq[fi] = cur_freq[fi];
                    }
                    best_play_rate_ctrl = 0;  /* updated by the CS_ENDPOINT that follows */
                    last_was_play_ep    = 1;
                    DPRINTF("[USBAudio] enumerate: best play so far: %ldch alt=%ld ep=0x%02lx maxpkt=%ld\n",
                                       (LONG)cur_ch, (LONG)current_alt,
                                       (LONG)ep_addr, (LONG)ep_maxpkt);
                }
                else
                {
                    last_was_play_ep = 0;
                    DPRINTF("[USBAudio] enumerate: skipping %ldch alt=%ld (best=%ldch)\n",
                                       (LONG)cur_ch, (LONG)current_alt,
                                       (LONG)best_play_ch);
                }
                cur_valid = 0;
            }

            /* Isochronous IN = recording endpoint */
            if (current_class == 0x01 && current_subclass == 0x02 &&
                (ep_attr & 0x03) == 0x01 &&
                (ep_addr & 0x80) != 0x00 &&
                current_alt > 0 &&
                !dev->rec_found)
            {
                dev->rec_found       = 1;
                dev->rec_iface_num   = (uint8)current_ifc;
                dev->rec_alt_setting = (uint8)current_alt;
                dev->rec_ep_addr     = ep_addr;
                dev->rec_max_pkt     = ep_maxpkt;

                if (cur_valid)
                {
                    dev->rec_nr_channels    = (uint8)cur_ch;
                    dev->rec_subframe_size  = (uint8)cur_sub;
                    dev->rec_bit_resolution = (uint8)cur_bits;
                    cur_valid = 0;
                }

                DPRINTF("[USBAudio] enumerate: recording EP=0x%02lx ifc=%ld alt=%ld ch=%ld maxpkt=%ld\n",
                                   (LONG)ep_addr, (LONG)current_ifc, (LONG)current_alt,
                                   (LONG)dev->rec_nr_channels, (LONG)dev->rec_max_pkt);
            }
        }
        /* ---- CS_ENDPOINT EP_GENERAL (Class-Specific AS Endpoint) ----
         * Immediately follows the Standard Endpoint descriptor (USB Audio
         * Class 1.0 §3.7.2).  bmAttributes bit D0 = Sampling Frequency
         * Control supported.  If D0=0 the device will always STALL a
         * SET_CUR SAMPLING_FREQ_CONTROL request. */
        else if (dsc_type == 0x25 && dsc_len >= 7 && buf[pos + 2] == 0x01)
        {
            if (last_was_play_ep)
            {
                best_play_rate_ctrl = buf[pos + 3] & 0x01;
                DPRINTF("[USBAudio] enumerate: CS_ENDPOINT bmAttributes=0x%02lx -> rate_ctrl=%ld\n",
                                   (ULONG)buf[pos + 3], (LONG)best_play_rate_ctrl);
            }
            last_was_play_ep = 0;
        }

        pos += dsc_len;
    }

    /* ---- Generate output entries from best playback alt setting ---- */
    if (best_play_ch >= 2)
    {
        if (n_ot > 0 && best_play_ch == 2)
        {
            /* Simple stereo device with named Output Terminals.
             * Expose each physical destination (e.g. Line Out, Headphone)
             * as a separate selectable AHI output entry.
             * All share the same USB endpoint — the AHI output selector
             * is cosmetic but shows the user the physical connectors. */
            static const struct { uint16 type; const char *name; } ot_names[] = {
                { 0x0301, "Speaker"      },
                { 0x0302, "Headphone"    },
                { 0x0303, "HMD Audio"    },
                { 0x0304, "Desktop Spkr" },
                { 0x0401, "Handset"      },
                { 0x0402, "Headset"      },
                { 0x0403, "Speakerphone" },
                { 0x0601, "Analog Out"   },
                { 0x0603, "Line Out"     },
                { 0x0604, "Legacy Out"   },
                { 0x0605, "SPDIF Out"    },
            };
            int32 n_to_add = n_ot;
            int32 oi;

            if (n_to_add > MAX_OUTPUT_MODES) n_to_add = MAX_OUTPUT_MODES;

            for (oi = 0; oi < n_to_add; oi++)
            {
                struct USBOutputMode *out = &dev->outputs[oi];
                const char *oname = "Audio Out";
                int32 k, fi, ni;

                out->iface_num      = best_play_ifc;
                out->alt_setting    = best_play_alt;
                out->ep_addr        = best_play_ep;
                out->max_pkt        = best_play_pkt;
                out->nr_channels    = (uint8)best_play_ch;
                out->subframe_size  = best_play_sub;
                out->bit_resolution = best_play_bits;
                out->channel_offset = 0;
                out->num_frequencies = best_play_nfreq;
                out->rate_ctrl      = (uint8)best_play_rate_ctrl;
                for (fi = 0; fi < best_play_nfreq && fi < MAX_USB_FREQUENCIES; fi++)
                    out->frequencies[fi] = best_play_freq[fi];

                for (ni = 0; ni < (int32)(sizeof(ot_names)/sizeof(ot_names[0])); ni++)
                    if (ot_names[ni].type == enum_ot[oi].type)
                        { oname = ot_names[ni].name; break; }

                for (k = 0; oname[k] && k < 47; k++)
                    out->name[k] = oname[k];
                out->name[k] = '\0';

                DPRINTF("[USBAudio] enumerate: output #%ld \"%s\" (OT type=0x%04lx) alt=%ld ep=0x%02lx ch=%ld\n",
                                   (LONG)oi, out->name, (ULONG)enum_ot[oi].type,
                                   (LONG)out->alt_setting, (LONG)out->ep_addr,
                                   (LONG)out->nr_channels);
            }
            dev->num_outputs = n_to_add;
        }
        else
        {
            /* Multichannel device or no explicit OTs: channel-pair approach
             * so surround devices get Front/Rear/Center-LFE/Side entries.
             * USB Audio Class 1.0 channel order (8ch):
             *   ch0=Front L, ch1=Front R,
             *   ch2=Center,  ch3=LFE,
             *   ch4=Back L,  ch5=Back R,
             *   ch6=Side L,  ch7=Side R */
            static const struct { const char *name; uint8 offset; } pair_map[] = {
                { "Front",      0 },
                { "Center/LFE", 2 },
                { "Back",       4 },
                { "Side",       6 },
            };
            int32 n_pairs = best_play_ch / 2;
            int32 pi;

            if (n_pairs > 4) n_pairs = 4;
            if (n_pairs > MAX_OUTPUT_MODES) n_pairs = MAX_OUTPUT_MODES;

            for (pi = 0; pi < n_pairs; pi++)
            {
                struct USBOutputMode *out = &dev->outputs[pi];
                const char *mname;
                int32 k, fi;

                out->iface_num      = best_play_ifc;
                out->alt_setting    = best_play_alt;
                out->ep_addr        = best_play_ep;
                out->max_pkt        = best_play_pkt;
                out->nr_channels    = (uint8)best_play_ch;
                out->subframe_size  = best_play_sub;
                out->bit_resolution = best_play_bits;
                out->channel_offset = pair_map[pi].offset;
                out->num_frequencies = best_play_nfreq;
                out->rate_ctrl      = (uint8)best_play_rate_ctrl;
                for (fi = 0; fi < best_play_nfreq && fi < MAX_USB_FREQUENCIES; fi++)
                    out->frequencies[fi] = best_play_freq[fi];

                mname = pair_map[pi].name;
                for (k = 0; mname[k] && k < 47; k++)
                    out->name[k] = mname[k];
                out->name[k] = '\0';

                DPRINTF("[USBAudio] enumerate: output #%ld \"%s\" alt=%ld ep=0x%02lx ch=%ld offset=%ld maxpkt=%ld\n",
                                   (LONG)pi, out->name,
                                   (LONG)out->alt_setting, (LONG)out->ep_addr,
                                   (LONG)out->nr_channels, (LONG)out->channel_offset,
                                   (LONG)out->max_pkt);
            }
            dev->num_outputs = n_pairs;
        }
    }

    /* ---- Resolve Feature Unit topology ---- */
    dev->ac_iface_num = enum_ac_iface;
    {
        int32 ii, fi, ti;

        /* Dump full topology for diagnostics */
        DPRINTF("[USBAudio] enumerate: topology: %ld IT(s), %ld FU(s), %ld OT(s)\n",
                           (LONG)n_it, (LONG)n_fu, (LONG)n_ot);
        for (ti = 0; ti < n_it; ti++)
            DPRINTF("[USBAudio] enumerate:   IT id=%ld type=0x%04lx\n",
                               (LONG)enum_it[ti].id, (ULONG)enum_it[ti].type);
        for (fi = 0; fi < n_fu; fi++)
            DPRINTF("[USBAudio] enumerate:   FU id=%ld src=%ld\n",
                               (LONG)enum_fu[fi].id, (LONG)enum_fu[fi].src);
        for (ti = 0; ti < n_ot; ti++)
            DPRINTF("[USBAudio] enumerate:   OT id=%ld type=0x%04lx\n",
                               (LONG)enum_ot[ti].id, (ULONG)enum_ot[ti].type);

        /* Match FUs to input sources */
        for (ii = 0; ii < dev->num_inputs; ii++)
        {
            for (fi = 0; fi < n_fu; fi++)
            {
                if (enum_fu[fi].src == dev->inputs[ii].terminal_id)
                {
                    dev->inputs[ii].fu_unit_id = enum_fu[fi].id;
                    dev->inputs[ii].fu_found   = 1;
                    DPRINTF("[USBAudio] enumerate: input \"%s\" → FU ID=%ld\n",
                                       dev->inputs[ii].name, (LONG)enum_fu[fi].id);
                    break;
                }
            }
        }

        /* Find output FU (source = USB Streaming IT, type 0x0101) */
        dev->fu_found = 0;
        for (fi = 0; fi < n_fu; fi++)
        {
            for (ti = 0; ti < n_it; ti++)
            {
                if (enum_it[ti].id == enum_fu[fi].src &&
                    enum_it[ti].type == 0x0101)
                {
                    dev->fu_found   = 1;
                    dev->fu_unit_id = enum_fu[fi].id;
                    DPRINTF("[USBAudio] enumerate: output FU ID=%ld (USB Streaming)\n",
                                       (LONG)enum_fu[fi].id);
                    break;
                }
            }
            if (dev->fu_found) break;
        }

        /* Find recording FU: source = any non-USB-Streaming IT.
         * Covers Microphone (0x02xx), Line In (0x0603), S/PDIF In (0x0605),
         * and any other physical input besides the USB playback stream.
         * Skip the FU already claimed as output FU to avoid double-use. */
        dev->fu_rec_found = 0;
        for (fi = 0; fi < n_fu; fi++)
        {
            /* Don't reuse the output FU */
            if (dev->fu_found && enum_fu[fi].id == dev->fu_unit_id)
                continue;

            for (ti = 0; ti < n_it; ti++)
            {
                if (enum_it[ti].id == enum_fu[fi].src &&
                    enum_it[ti].type != 0x0101)  /* any non-USB-Streaming input */
                {
                    dev->fu_rec_found   = 1;
                    dev->fu_rec_unit_id = enum_fu[fi].id;
                    DPRINTF("[USBAudio] enumerate: rec FU ID=%ld (IT type=0x%04lx)\n",
                                       (LONG)enum_fu[fi].id, (ULONG)enum_it[ti].type);
                    break;
                }
            }
            if (dev->fu_rec_found) break;
        }

        /* Fallback: if no output FU matched, use first FU */
        if (!dev->fu_found && n_fu > 0)
        {
            dev->fu_found   = 1;
            dev->fu_unit_id = enum_fu[0].id;
            DPRINTF("[USBAudio] enumerate: fallback output FU ID=%ld\n",
                               (LONG)enum_fu[0].id);
        }
    }

    /* ---- Store Selector Unit info & map inputs to SU pins ---- */
    dev->selector_unit_id  = su_id;
    dev->selector_num_pins = su_num_pins;
    {
        int32 pi;
        for (pi = 0; pi < su_num_pins && pi < MAX_INPUT_SOURCES; pi++)
            dev->selector_sources[pi] = su_sources[pi];
    }

    if (su_id != 0 && su_num_pins > 0)
    {
        int32 ii, pi;
        for (ii = 0; ii < dev->num_inputs; ii++)
        {
            dev->inputs[ii].su_pin = 0;  /* Unknown by default */

            for (pi = 0; pi < su_num_pins; pi++)
            {
                /* Match by FU ID (IT → FU → SU) or by IT ID directly (IT → SU) */
                if ((dev->inputs[ii].fu_found && su_sources[pi] == dev->inputs[ii].fu_unit_id) ||
                    su_sources[pi] == dev->inputs[ii].terminal_id)
                {
                    dev->inputs[ii].su_pin = (uint8)(pi + 1);  /* 1-based */
                    DPRINTF("[USBAudio] enumerate: input \"%s\" → SU pin %ld\n",
                                       dev->inputs[ii].name, (LONG)(pi + 1));
                    break;
                }
            }
        }
    }

    /* ---- Apply first (stereo) output as default ---- */
    if (dev->num_outputs > 0)
    {
        dev->selected_output = 0;

        /* Apply selected output's parameters to the device top-level fields */
        {
            struct USBOutputMode *sel = &dev->outputs[dev->selected_output];
            dev->iface_num      = sel->iface_num;
            dev->alt_setting    = sel->alt_setting;
            dev->ep_addr        = sel->ep_addr;
            dev->max_pkt        = sel->max_pkt;
            dev->nr_channels    = sel->nr_channels;
            dev->subframe_size  = sel->subframe_size;
            dev->bit_resolution = sel->bit_resolution;

            dev->num_frequencies = sel->num_frequencies;
            {
                int32 fi;
                for (fi = 0; fi < sel->num_frequencies && fi < MAX_USB_FREQUENCIES; fi++)
                    dev->frequencies[fi] = sel->frequencies[fi];
            }

            if (sel->nr_channels > 0 && sel->subframe_size > 0)
                dev->native_freq = (uint32)sel->max_pkt * 1000 /
                                   ((uint32)sel->nr_channels * (uint32)sel->subframe_size);
            else
                dev->native_freq = 48000;
        }
    }

    DPRINTF("[USBAudio] enumerate: %ld outputs, %ld inputs, default out=#%ld\n",
                       (LONG)dev->num_outputs, (LONG)dev->num_inputs,
                       (LONG)dev->selected_output);

    IExec->FreeVec(buf);
    ILibusb1->libusb_close(handle);
}

/*
 * wait_usb_stack_fullbooted — block until the Sirion USB stack signals
 * USBNM_TYPE_STACKFULLBOOTED, meaning all USBFDs have run and the
 * stack is idle.  A 5-second timeout covers the case where the stack
 * was already fully booted before we subscribed (the notification
 * is only sent once, at the moment it happens).
 */
static void wait_usb_stack_fullbooted(void)
{
    struct Library         *USBResBase = NULL;
    struct USBResourceIFace *IUSBRes   = NULL;
    struct MsgPort         *notifyPort = NULL;
    struct MsgPort         *timerPort  = NULL;
    struct TimeRequest     *timerReq   = NULL;
    APTR                    notifySub  = NULL;
    int32                   timerOpen  = 0;

    /* 1. Open usbresource.library */
    USBResBase = IExec->OpenLibrary("usbresource.library", 53);
    if (!USBResBase)
    {
        DPRINTF("[USBAudio] wait_fullboot: cannot open usbresource.library\n");
        return;
    }

    IUSBRes = (struct USBResourceIFace *)IExec->GetInterface(USBResBase, "main", 1, NULL);
    if (!IUSBRes)
    {
        DPRINTF("[USBAudio] wait_fullboot: cannot get IUSBResource\n");
        goto cleanup;
    }

    /* 2. Create notification port and subscribe */
    notifyPort = IExec->AllocSysObjectTags(ASOT_PORT, TAG_END);
    if (!notifyPort)
        goto cleanup;

    notifySub = IUSBRes->USBResAddNotify(USBNM_TYPE_STACKFULLBOOTED, notifyPort);
    DPRINTF("[USBAudio] wait_fullboot: subscribed for STACKFULLBOOTED (%p)\n", notifySub);

    /* 3. Set up a 5-second timeout via timer.device */
    timerPort = IExec->AllocSysObjectTags(ASOT_PORT, TAG_END);
    if (!timerPort)
        goto cleanup;

    timerReq = (struct TimeRequest *)IExec->AllocSysObjectTags(ASOT_IOREQUEST,
        ASOIOR_Size,      sizeof(struct TimeRequest),
        ASOIOR_ReplyPort, (uint32)timerPort,
        TAG_END);
    if (!timerReq)
        goto cleanup;

    if (IExec->OpenDevice("timer.device", UNIT_VBLANK,
                          (struct IORequest *)timerReq, 0) != 0)
    {
        DPRINTF("[USBAudio] wait_fullboot: cannot open timer.device\n");
        goto cleanup;
    }
    timerOpen = 1;

    /* Fire a 2-second one-shot timer */
    timerReq->Request.io_Command = TR_ADDREQUEST;
    timerReq->Time.Seconds       = 2;
    timerReq->Time.Microseconds  = 0;
    IExec->SendIO((struct IORequest *)timerReq);

    /* 4. Wait for whichever signal comes first */
    {
        ULONG notifySig = 1UL << notifyPort->mp_SigBit;
        ULONG timerSig  = 1UL << timerPort->mp_SigBit;
        ULONG sigs;

        sigs = IExec->Wait(notifySig | timerSig);

        if (sigs & notifySig)
        {
            DPRINTF("[USBAudio] wait_fullboot: STACKFULLBOOTED received\n");
            /* Drain the message */
            struct Message *msg;
            while ((msg = IExec->GetMsg(notifyPort)) != NULL)
                ;  /* no ReplyMsg needed for notify messages */
        }

        if (sigs & timerSig)
        {
            DPRINTF("[USBAudio] wait_fullboot: timeout (stack already booted)\n");
        }
    }

    /* Cancel the timer if the notify arrived first */
    if (!IExec->CheckIO((struct IORequest *)timerReq))
        IExec->AbortIO((struct IORequest *)timerReq);
    IExec->WaitIO((struct IORequest *)timerReq);

cleanup:
    if (notifySub && IUSBRes)
        IUSBRes->USBResRemNotify(notifySub);
    if (timerOpen)
        IExec->CloseDevice((struct IORequest *)timerReq);
    if (timerReq)
        IExec->FreeSysObject(ASOT_IOREQUEST, timerReq);
    if (timerPort)
        IExec->FreeSysObject(ASOT_PORT, timerPort);
    /* Drain remaining notify messages */
    if (notifyPort)
    {
        struct Message *msg;
        while ((msg = IExec->GetMsg(notifyPort)) != NULL)
            ;
        IExec->FreeSysObject(ASOT_PORT, notifyPort);
    }
    if (IUSBRes)
        IExec->DropInterface((struct Interface *)IUSBRes);
    if (USBResBase)
        IExec->CloseLibrary(USBResBase);
}

void scan_usb_audio_device(void)
{
    libusb_device **list = NULL;
    int32 cnt, i;

    if (g_usb_num_devices > 0 && g_usb_info.found)
        return;

    /*
     * If we already scanned and found nothing, do NOT re-scan unless
     * g_force_rescan has been set (e.g. by USBFD hot-plug).
     * Without this guard, AHI retries AllocAudio in a tight loop
     * and each iteration enumerates every USB device with debug
     * prints, flooding the serial console and freezing the UI.
     */
    if (g_usb_info.scanned && !g_usb_info.found && !g_force_rescan)
        return;

    /* Wait for STACKFULLBOOTED once — never again after that. */
    if (!g_stack_booted)
    {
        DPRINTF("[USBAudio] scan: waiting for USB stack full-boot...\n");
        wait_usb_stack_fullbooted();
        g_stack_booted = 1;
        DPRINTF("[USBAudio] scan: USB stack ready\n");
    }

    /* Lazy-open libusb-1.library on first scan.
     * Deferred from libInit/libOpen to avoid deadlocking when the
     * driver is loaded during USBFD/AddAudioModes at boot time
     * (the USB stack is still initialising). */
    if (Libusb1Base == NULL)
    {
        DPRINTF("[USBAudio] scan: opening libusb-1.library...\n");
        Libusb1Base = IExec->OpenLibrary("libusb-1.library", 0L);
        DPRINTF("[USBAudio] scan: OpenLibrary returned %p\n", Libusb1Base);
        if (Libusb1Base)
        {
            ILibusb1 = (struct Libusb1IFace *)IExec->GetInterface(Libusb1Base, "main", 1, NULL);
            if (ILibusb1)
            {
                ILibusb1->libusb_init(NULL);
                DPRINTF("[USBAudio] libusb-1.library opened OK\n");
            }
            else
            {
                DPRINTF("[USBAudio] Failed to get ILibusb1 interface\n");
                IExec->CloseLibrary(Libusb1Base);
                Libusb1Base = NULL;
                return;
            }
        }
        else
        {
            DPRINTF("[USBAudio] scan: cannot open libusb-1.library\n");
            return;
        }
    }

    /* Load user-defined device names once */
    if (g_num_user_devices == 0)
        load_user_device_prefs();

    /* Handle rescan requests from USBFD hot-plug */
    if (g_force_rescan)
    {
        g_force_rescan    = 0;
        g_usb_info.scanned = 0;
        g_usb_num_devices  = 0;
    }

    DPRINTF("[USBAudio] scan_usb_audio_device: entry (scanned=%ld, num_devs=%ld)\n",
                       g_usb_info.scanned, g_usb_num_devices);

    g_usb_info.scanned = 1;
    g_usb_info.found = 0;
    g_usb_info.rec_found = 0;
    g_usb_info.fu_found = 0;
    g_usb_info.fu_vol_failed = 0;
    g_usb_info.fu_vol_cached = 0;
    g_usb_info.fu_rec_vol_failed = 0;
    g_usb_info.fu_rec_vol_cached = 0;
    g_usb_num_devices = 0;

    DPRINTF("[USBAudio] scan: calling libusb_get_device_list...\n");
    cnt = ILibusb1->libusb_get_device_list(NULL, &list);
    DPRINTF("[USBAudio] libusb_get_device_list: cnt=%ld, list=%p\n", cnt, list);
    if (cnt <= 0 || list == NULL)
    {
        DPRINTF("[USBAudio] No USB devices found!\n");
        return;
    }

    for (i = 0; list[i] != NULL; i++)
    {
        struct libusb_device_descriptor desc;
        struct libusb_config_descriptor *cfg = NULL;
        int32 r, j, k;

        r = ILibusb1->libusb_get_device_descriptor(list[i], &desc);
        if (r != LIBUSB_SUCCESS)
            continue;

        DPRINTF("[USBAudio] Device %ld: VID=0x%04lx PID=0x%04lx Class=0x%02lx\n",
                           i, (ULONG)desc.idVendor, (ULONG)desc.idProduct, (ULONG)desc.bDeviceClass);

        /* Skip hubs */
        if (desc.bDeviceClass == 0x09)
        {
            DPRINTF("[USBAudio] Device %ld: hub, skipping\n", i);
            continue;
        }

        /* Use cached config descriptor from USB stack (NO device I/O) */
        r = ILibusb1->libusb_get_config_descriptor(list[i], 0, &cfg);
        if (r != LIBUSB_SUCCESS || cfg == NULL)
        {
            DPRINTF("[USBAudio] Device %ld: get_config_descriptor failed (%ld)\n", i, r);
            continue;
        }

        /* Look for Audio Streaming interface (class 0x01, subclass 0x02) */
        {
            int32 found_as = 0;
            uint8 as_iface_num = 0;
            uint8 as_alt_setting = 1;
            uint8 as_ep_addr = 0;
            uint16 as_max_pkt = 192;
            uint8 as_nr_channels = 2;
            uint8 as_subframe_size = 2;
            uint8 as_bit_resolution = 16;

            /* Recording IN endpoint */
            int32 found_rec = 0;
            uint8 rec_iface_num = 0;
            uint8 rec_alt_setting = 1;
            uint8 rec_ep_addr = 0;
            uint16 rec_max_pkt = 100;
            uint8 rec_nr_channels = 1;
            uint8 rec_subframe_size = 2;
            uint8 rec_bit_resolution = 16;

            /* Audio Control - Feature Units & topology */
            int32 found_ac = 0;
            uint8 ac_iface_num = 0;
            int32 found_fu = 0;
            uint8 fu_unit_id = 0;
            int32 found_fu_rec = 0;
            uint8 fu_rec_unit_id = 0;

            for (j = 0; j < cfg->bNumInterfaces; j++)
            {
                const struct libusb_interface *intf = &cfg->interface[j];
                for (k = 0; k < intf->num_altsetting; k++)
                {
                    const struct libusb_interface_descriptor *as = &intf->altsetting[k];
                    DPRINTF("[USBAudio]   Ifc %ld Alt %ld: Class=0x%02lx Sub=0x%02lx NumEP=%ld\n",
                                       (LONG)as->bInterfaceNumber, (LONG)as->bAlternateSetting,
                                       (LONG)as->bInterfaceClass, (LONG)as->bInterfaceSubClass,
                                       (LONG)as->bNumEndpoints);

                    /* Audio Control interface (class 0x01, subclass 0x01) */
                    if (as->bInterfaceClass == 0x01 &&
                        as->bInterfaceSubClass == 0x01 && !found_ac)
                    {
                        ac_iface_num = as->bInterfaceNumber;
                        found_ac = 1;
                        DPRINTF("[USBAudio]   => Audio Control found! Ifc=%ld\n",
                                           (LONG)as->bInterfaceNumber);

                        /* Parse class-specific AC descriptors for topology:
                         * Input Terminals, Feature Units, Output Terminals.
                         * We trace the signal chain to identify which FU
                         * is on the output path vs the input/recording path. */
                        if (as->extra && as->extra_length > 0)
                        {
                            const uint8 *extra = (const uint8 *)as->extra;
                            int32 elen = as->extra_length;
                            int32 pos = 0;

                            /* Phase 1: collect Input Terminals and Feature Units */
                            #define MAX_AC_UNITS 16
                            struct { uint8 id; uint16 type; } input_terms[MAX_AC_UNITS];
                            struct { uint8 id; uint8 src; }   feat_units[MAX_AC_UNITS];
                            int32 n_it = 0, n_fu = 0;

                            while (pos + 2 < elen)
                            {
                                uint8 dlen = extra[pos];
                                uint8 dtype = extra[pos + 1];
                                if (dlen < 2 || pos + dlen > elen) break;

                                if (dtype == 0x24) /* CS_INTERFACE */
                                {
                                    uint8 subtype = extra[pos + 2];

                                    /* INPUT_TERMINAL (subtype 0x02), len >= 12 */
                                    if (subtype == 0x02 && dlen >= 12 && n_it < MAX_AC_UNITS)
                                    {
                                        input_terms[n_it].id   = extra[pos + 3];
                                        input_terms[n_it].type = (uint16)(extra[pos + 4] | (extra[pos + 5] << 8));
                                        DPRINTF("[USBAudio]     Input Terminal: ID=%ld type=0x%04lx\n",
                                                           (LONG)input_terms[n_it].id,
                                                           (LONG)input_terms[n_it].type);
                                        n_it++;
                                    }
                                    /* FEATURE_UNIT (subtype 0x06), len >= 7 */
                                    else if (subtype == 0x06 && dlen >= 7 && n_fu < MAX_AC_UNITS)
                                    {
                                        feat_units[n_fu].id  = extra[pos + 3];
                                        feat_units[n_fu].src = extra[pos + 4];
                                        DPRINTF("[USBAudio]     Feature Unit: ID=%ld srcID=%ld\n",
                                                           (LONG)feat_units[n_fu].id,
                                                           (LONG)feat_units[n_fu].src);
                                        n_fu++;
                                    }
                                }
                                pos += dlen;
                            }

                            /* Phase 2: match FUs to output (playback) vs input (recording) path.
                             * - FU whose source is an IT of type 0x0101 (USB Streaming) → output FU
                             * - FU whose source is an IT of type 0x02xx (Microphone/Line) → input/rec FU
                             * If only one FU exists, treat it as output FU. */
                            {
                                int32 fi, ti;
                                for (fi = 0; fi < n_fu; fi++)
                                {
                                    uint16 src_type = 0;
                                    /* Look up the source in Input Terminals */
                                    for (ti = 0; ti < n_it; ti++)
                                    {
                                        if (input_terms[ti].id == feat_units[fi].src)
                                        {
                                            src_type = input_terms[ti].type;
                                            break;
                                        }
                                    }

                                    if (src_type == 0x0101 && !found_fu)
                                    {
                                        /* Source is USB streaming → output/playback FU */
                                        fu_unit_id = feat_units[fi].id;
                                        found_fu = 1;
                                        DPRINTF("[USBAudio]     => Output FU: ID=%ld (src=USB Streaming IT)\n",
                                                           (LONG)fu_unit_id);
                                    }
                                    else if ((src_type & 0xFF00) == 0x0200 && !found_fu_rec)
                                    {
                                        /* Source is Microphone/Line → input/recording FU */
                                        fu_rec_unit_id = feat_units[fi].id;
                                        found_fu_rec = 1;
                                        DPRINTF("[USBAudio]     => Input FU: ID=%ld (src=Mic/Line IT type=0x%04lx)\n",
                                                           (LONG)fu_rec_unit_id, (LONG)src_type);
                                    }
                                }

                                /* Fallback: if no output FU matched by topology but we have FUs,
                                 * use the first one as output FU */
                                if (!found_fu && n_fu > 0)
                                {
                                    fu_unit_id = feat_units[0].id;
                                    found_fu = 1;
                                    DPRINTF("[USBAudio]     => Fallback output FU: ID=%ld\n",
                                                       (LONG)fu_unit_id);
                                }
                            }
                        }
                    }

                    /* Audio Streaming interface (class 0x01, subclass 0x02) */
                    if (as->bInterfaceClass == 0x01 &&
                        as->bInterfaceSubClass == 0x02)
                    {
                        /* Mark interface as found even if alt 0 has no endpoints.
                         * On the Sirion USB stack, libusb_get_config_descriptor
                         * only returns alt 0 (zero bandwidth, no endpoints).
                         * The real endpoint is discovered later by
                         * discover_usb_audio_details() via GET_DESCRIPTOR.
                         *
                         * First Audio Streaming interface = playback (OUT).
                         * Second Audio Streaming interface = recording (IN). */
                        if (!found_as)
                        {
                            as_iface_num = as->bInterfaceNumber;
                            found_as = 1;
                            DPRINTF("[USBAudio]   => Audio Streaming (playback) found! Ifc=%ld\n",
                                               (LONG)as->bInterfaceNumber);
                        }
                        else if (!found_rec && as->bInterfaceNumber != as_iface_num)
                        {
                            rec_iface_num = as->bInterfaceNumber;
                            found_rec = 1;
                            DPRINTF("[USBAudio]   => Audio Streaming (recording) found! Ifc=%ld\n",
                                               (LONG)as->bInterfaceNumber);
                        }

                        /* If this alt setting has endpoints, extract them */
                        if (as->bNumEndpoints > 0 && as->endpoint != NULL)
                        {
                            int32 ep_idx;

                            for (ep_idx = 0; ep_idx < as->bNumEndpoints; ep_idx++)
                            {
                                const struct libusb_endpoint_descriptor *ep = &as->endpoint[ep_idx];
                                uint8 ep_dir = ep->bEndpointAddress & 0x80;
                                uint8 ep_type = ep->bmAttributes & 0x03;

                                DPRINTF("[USBAudio]     EP[%ld]: addr=0x%02lx attr=0x%02lx maxpkt=%ld\n",
                                                   (LONG)ep_idx, (LONG)ep->bEndpointAddress,
                                                   (LONG)ep->bmAttributes, (LONG)ep->wMaxPacketSize);

                                /* OUT endpoint = playback */
                                if (ep_dir == LIBUSB_ENDPOINT_OUT &&
                                    (ep_type == LIBUSB_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS ||
                                     ep_type == LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK))
                                {
                                    as_iface_num   = as->bInterfaceNumber;
                                    as_alt_setting = as->bAlternateSetting;
                                    as_ep_addr     = ep->bEndpointAddress;
                                    as_max_pkt     = ep->wMaxPacketSize;
                                    DPRINTF("[USBAudio]     => OUT EP 0x%02lx maxpkt=%ld\n",
                                                       (LONG)as_ep_addr, (LONG)as_max_pkt);
                                }

                                /* IN endpoint = recording */
                                if (ep_dir == LIBUSB_ENDPOINT_IN &&
                                    (ep_type == LIBUSB_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS ||
                                     ep_type == LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK))
                                {
                                    rec_iface_num   = as->bInterfaceNumber;
                                    rec_alt_setting = as->bAlternateSetting;
                                    rec_ep_addr     = ep->bEndpointAddress;
                                    rec_max_pkt     = ep->wMaxPacketSize;
                                    found_rec = 1;
                                    DPRINTF("[USBAudio]     => IN EP 0x%02lx maxpkt=%ld\n",
                                                       (LONG)rec_ep_addr, (LONG)rec_max_pkt);
                                }
                            }

                            /* Parse Format Type I descriptor for this alt setting */
                            if (as->extra && as->extra_length > 0)
                            {
                                const uint8 *extra = (const uint8 *)as->extra;
                                int32 elen = as->extra_length;
                                int32 pos = 0;

                                while (pos + 2 < elen)
                                {
                                    uint8 dlen = extra[pos];
                                    uint8 dtype = extra[pos + 1];

                                    if (dlen < 2 || pos + dlen > elen)
                                        break;

                                    if (dtype == 0x24 && dlen >= 8 && extra[pos + 2] == 0x02)
                                    {
                                        if (extra[pos + 3] == 0x01)
                                        {
                                            as_nr_channels   = extra[pos + 4];
                                            as_subframe_size = extra[pos + 5];
                                            as_bit_resolution = extra[pos + 6];
                                            DPRINTF("[USBAudio]     Format Type I: %ldch %ldbit sub=%ld\n",
                                                               (LONG)as_nr_channels, (LONG)as_bit_resolution,
                                                               (LONG)as_subframe_size);
                                        }
                                    }
                                    pos += dlen;
                                }
                            }
                        }
                    }
                }
            }

            ILibusb1->libusb_free_config_descriptor(cfg);

            if (found_as && g_usb_num_devices < MAX_USB_AUDIO_DEVICES)
            {
                struct USBAudioDeviceInfo *dev = &g_usb_devices[g_usb_num_devices];

                memset(dev, 0, sizeof(*dev));

                dev->vid         = desc.idVendor;
                dev->pid         = desc.idProduct;
                dev->iface_num   = as_iface_num;
                dev->alt_setting = as_alt_setting;
                dev->ep_addr     = as_ep_addr ? as_ep_addr : 0x01;
                dev->max_pkt     = as_max_pkt;
                dev->nr_channels    = as_nr_channels;
                dev->subframe_size  = as_subframe_size;
                dev->bit_resolution = as_bit_resolution;
                dev->found = 1;

                /* Recording endpoint */
                if (found_rec)
                {
                    dev->rec_found          = 1;
                    dev->rec_iface_num      = rec_iface_num;
                    dev->rec_alt_setting    = rec_alt_setting;
                    dev->rec_ep_addr        = rec_ep_addr;
                    dev->rec_max_pkt        = rec_max_pkt;
                    dev->rec_nr_channels    = rec_nr_channels;
                    dev->rec_subframe_size  = rec_subframe_size;
                    dev->rec_bit_resolution = rec_bit_resolution;

                    DPRINTF("[USBAudio] Dev %ld Recording: Ifc=%ld Alt=%ld EP=0x%02lx MaxPkt=%ld %ldch/%ldbit\n",
                                       (LONG)g_usb_num_devices,
                                       (LONG)rec_iface_num, (LONG)rec_alt_setting,
                                       (LONG)rec_ep_addr, (LONG)rec_max_pkt,
                                       (LONG)rec_nr_channels, (LONG)rec_bit_resolution);
                }

                /* Audio Control Feature Unit */
                if (found_ac)
                {
                    dev->ac_iface_num = ac_iface_num;
                    if (found_fu)
                    {
                        dev->fu_found   = 1;
                        dev->fu_unit_id = fu_unit_id;
                        DPRINTF("[USBAudio] Dev %ld Output Feature Unit: ID=%ld on AC Ifc=%ld\n",
                                           (LONG)g_usb_num_devices, (LONG)fu_unit_id, (LONG)ac_iface_num);
                    }
                    if (found_fu_rec)
                    {
                        dev->fu_rec_found   = 1;
                        dev->fu_rec_unit_id = fu_rec_unit_id;
                        DPRINTF("[USBAudio] Dev %ld Input Feature Unit: ID=%ld on AC Ifc=%ld\n",
                                           (LONG)g_usb_num_devices, (LONG)fu_rec_unit_id, (LONG)ac_iface_num);
                    }
                }

                /* Derive native frequency from max packet size */
                if (as_nr_channels > 0 && as_subframe_size > 0)
                    dev->native_freq = (uint32)as_max_pkt * 1000 /
                                       ((uint32)as_nr_channels * (uint32)as_subframe_size);
                else
                    dev->native_freq = 48000;

                dev->num_frequencies = 2;
                dev->frequencies[0]  = 44100;
                dev->frequencies[1]  = 48000;

                /* Generate human-readable name */
                make_device_name(dev);

                /* Enumerate all output modes and input sources from raw config descriptor */
                enumerate_device_modes(list[i], dev);

                /* Deduplicate: skip if config descriptor matches a previous device
                   (Sirion USB stack bug returns same handle for different USB addresses) */
                {
                    int32 dup = 0, pi;
                    for (pi = 0; pi < g_usb_num_devices; pi++)
                    {
                        if (g_usb_devices[pi].config_checksum == dev->config_checksum)
                        {
                            dup = 1;
                            DPRINTF("[USBAudio] Device %ld (%04lx:%04lx): duplicate of device %ld, skipping\n",
                                               i, (ULONG)dev->vid, (ULONG)dev->pid, (LONG)pi);
                            break;
                        }
                    }
                    if (dup)
                    {
                        dev->found = 0;
                        continue;
                    }
                }

                DPRINTF("[USBAudio] USB Audio device #%ld found: \"%s\"\n",
                                   (LONG)g_usb_num_devices, dev->name);
                DPRINTF("[USBAudio]   VID=0x%04lx PID=0x%04lx Iface=%ld Alt=%ld\n",
                                   (ULONG)dev->vid, (ULONG)dev->pid,
                                   (LONG)dev->iface_num, (LONG)dev->alt_setting);
                DPRINTF("[USBAudio]   EP=0x%02lx MaxPkt=%ld %ldch/%ldbit\n",
                                   (LONG)dev->ep_addr, (LONG)dev->max_pkt,
                                   (LONG)dev->nr_channels, (LONG)dev->bit_resolution);

                g_usb_num_devices++;
                /* Do NOT break — continue scanning for more devices */
            }
            else
            {
                DPRINTF("[USBAudio] Device %ld: no Audio Streaming interface\n", i);
            }
        }
    }

    ILibusb1->libusb_free_device_list(list, 1);

    if (g_usb_num_devices == 0)
    {
        DPRINTF("[USBAudio] scan_usb_audio_device: NO USB Audio device found! :-(\n");
        g_usb_info.found = 0;
    }
    else
    {
        /* Copy first device as default active device */
        g_usb_info = g_usb_devices[0];
        g_usb_info.scanned = 1;  /* Preserve: struct copy overwrites it */
        DPRINTF("[USBAudio] scan_usb_audio_device: %ld device(s) found, default=\"%s\"\n",
                           (LONG)g_usb_num_devices, g_usb_info.name);
    }

    /* ---- Build flattened output/input arrays for AHI Prefs ---- */
    g_num_flat_outputs = 0;
    g_num_flat_inputs  = 0;

    {
        int32 di, oi, ii;
        for (di = 0; di < g_usb_num_devices; di++)
        {
            struct USBAudioDeviceInfo *d = &g_usb_devices[di];

            /* Flatten outputs */
            for (oi = 0; oi < d->num_outputs && g_num_flat_outputs < MAX_TOTAL_OUTPUTS; oi++)
            {
                struct USBAudioFlatOutput *fo = &g_usb_flat_outputs[g_num_flat_outputs];
                int32 k = 0, p;

                fo->device_idx = di;
                fo->mode_idx   = oi;

                /* Use just the mode name (e.g. "Stereo") for short AHI Prefs display */
                for (p = 0; d->outputs[oi].name[p] && k < 63; p++)
                    fo->name[k++] = d->outputs[oi].name[p];
                fo->name[k] = '\0';

                DPRINTF("[USBAudio] flat output #%ld: \"%s\"\n",
                                   (LONG)g_num_flat_outputs, fo->name);
                g_num_flat_outputs++;
            }

            /* If device has NO enumerated outputs, create 1 entry from basic scan data */
            if (d->num_outputs == 0 && g_num_flat_outputs < MAX_TOTAL_OUTPUTS)
            {
                struct USBAudioFlatOutput *fo = &g_usb_flat_outputs[g_num_flat_outputs];
                int32 k = 0, p;

                fo->device_idx = di;
                fo->mode_idx   = -1;

                for (p = 0; d->name[p] && k < 63; p++)
                    fo->name[k++] = d->name[p];
                fo->name[k] = '\0';

                g_num_flat_outputs++;
            }

            /* Flatten inputs */
            for (ii = 0; ii < d->num_inputs && g_num_flat_inputs < MAX_TOTAL_INPUTS; ii++)
            {
                struct USBAudioFlatInput *fin = &g_usb_flat_inputs[g_num_flat_inputs];
                int32 k = 0, p;

                fin->device_idx  = di;
                fin->source_idx  = ii;

                /* Use just the source name (e.g. "Microphone") for short AHI Prefs display */
                for (p = 0; d->inputs[ii].name[p] && k < 63; p++)
                    fin->name[k++] = d->inputs[ii].name[p];
                fin->name[k] = '\0';

                DPRINTF("[USBAudio] flat input #%ld: \"%s\"\n",
                                   (LONG)g_num_flat_inputs, fin->name);
                g_num_flat_inputs++;
            }

            /* If device has recording but no enumerated inputs, fallback entry */
            if (d->num_inputs == 0 && d->rec_found && g_num_flat_inputs < MAX_TOTAL_INPUTS)
            {
                struct USBAudioFlatInput *fin = &g_usb_flat_inputs[g_num_flat_inputs];
                int32 k = 0, p;
                const char *suffix = " Line In";

                fin->device_idx = di;
                fin->source_idx = -1;

                for (p = 0; d->name[p] && k < 55; p++)
                    fin->name[k++] = d->name[p];
                for (p = 0; suffix[p] && k < 63; p++)
                    fin->name[k++] = suffix[p];
                fin->name[k] = '\0';

                g_num_flat_inputs++;
            }
        }
    }

    DPRINTF("[USBAudio] scan: %ld total outputs, %ld total inputs\n",
                       (LONG)g_num_flat_outputs, (LONG)g_num_flat_inputs);

    g_usb_info.scanned = 1;
}


/*
 * discover_usb_audio_details
 *
 * Called from Start() AFTER the device is opened.
 * The heavy lifting (alt-setting enumeration, FU topology) was already
 * done by enumerate_device_modes() during scan.  This function now just:
 *   - Applies the selected output mode parameters to dd->
 *   - Applies recording endpoint parameters
 *   - Queries output and input volume ranges from the Feature Units
 */
static void discover_usb_audio_details(struct AHIAudioCtrlDrv *AudioCtrl)
{
    int32 r;
    int32 found_fu = 0, found_fu_rec = 0;

    DPRINTF("[USBAudio] discover_usb_audio_details: entry\n");

    if (dd->ua_DevHandle == NULL)
        return;

    /* ---- Apply selected output mode from pre-enumerated data ---- */
    if (g_usb_info.num_outputs > 0 &&
        g_usb_info.selected_output >= 0 &&
        g_usb_info.selected_output < g_usb_info.num_outputs)
    {
        struct USBOutputMode *sel = &g_usb_info.outputs[g_usb_info.selected_output];

        dd->ua_InterfaceNum  = sel->iface_num;
        dd->ua_AltSetting    = sel->alt_setting;
        dd->ua_EndpointAddr  = sel->ep_addr;
        dd->ua_MaxPacketSize = sel->max_pkt;
        dd->ua_NumChannels   = sel->nr_channels;
        dd->ua_SubframeSize  = sel->subframe_size;
        dd->ua_BitResolution = sel->bit_resolution;
        dd->ua_ChannelOffset = sel->channel_offset;
        dd->ua_RateCtrl      = sel->rate_ctrl;

        /* Update global frequency list */
        g_usb_info.num_frequencies = sel->num_frequencies;
        {
            int32 fi;
            for (fi = 0; fi < sel->num_frequencies && fi < MAX_USB_FREQUENCIES; fi++)
                g_usb_info.frequencies[fi] = sel->frequencies[fi];
        }

        DPRINTF("[USBAudio] discover: output mode #%ld: Ifc=%ld Alt=%ld EP=0x%02lx MaxPkt=%ld Ch=%ld Bits=%ld\n",
                           (LONG)g_usb_info.selected_output,
                           (LONG)sel->iface_num, (LONG)sel->alt_setting,
                           (LONG)sel->ep_addr, (LONG)sel->max_pkt,
                           (LONG)sel->nr_channels, (LONG)sel->bit_resolution);
    }
    else
    {
        DPRINTF("[USBAudio] discover: no output modes, using scan defaults\n");
    }

    /* ---- Apply recording endpoint from pre-enumerated data ---- */
    if (g_usb_info.rec_found)
    {
        dd->ua_RecInterfaceNum  = g_usb_info.rec_iface_num;
        dd->ua_RecAltSetting    = g_usb_info.rec_alt_setting;
        dd->ua_RecEndpointAddr  = g_usb_info.rec_ep_addr;
        dd->ua_RecMaxPacketSize = g_usb_info.rec_max_pkt;
        dd->ua_RecNumChannels   = g_usb_info.rec_nr_channels;
        dd->ua_RecSubframeSize  = g_usb_info.rec_subframe_size;
        dd->ua_RecBitResolution = g_usb_info.rec_bit_resolution;

        DPRINTF("[USBAudio] discover: REC Ifc=%ld Alt=%ld EP=0x%02lx MaxPkt=%ld Ch=%ld Bits=%ld\n",
                           (LONG)dd->ua_RecInterfaceNum, (LONG)dd->ua_RecAltSetting,
                           (LONG)dd->ua_RecEndpointAddr, (LONG)dd->ua_RecMaxPacketSize,
                           (LONG)dd->ua_RecNumChannels, (LONG)dd->ua_RecBitResolution);
    }

    /* ---- Apply Feature Unit info from pre-enumerated data ---- */
    if (g_usb_info.fu_found)
    {
        dd->ua_FUnitID    = g_usb_info.fu_unit_id;
        dd->ua_ACIfaceNum = g_usb_info.ac_iface_num;
        found_fu = 1;
        DPRINTF("[USBAudio] discover: Output FU ID=%ld AC Ifc=%ld\n",
                           (LONG)dd->ua_FUnitID, (LONG)dd->ua_ACIfaceNum);
    }

    if (g_usb_info.fu_rec_found)
    {
        dd->ua_RecFUnitID = g_usb_info.fu_rec_unit_id;
        dd->ua_ACIfaceNum = g_usb_info.ac_iface_num;
        found_fu_rec = 1;
        DPRINTF("[USBAudio] discover: Input FU ID=%ld\n",
                           (LONG)dd->ua_RecFUnitID);
    }

    /* ---- Apply Selector Unit info from pre-enumerated data ---- */
    dd->ua_SelectorUnitID = g_usb_info.selector_unit_id;
    if (g_usb_info.selector_unit_id != 0)
    {
        DPRINTF("[USBAudio] discover: Selector Unit ID=%ld pins=%ld\n",
                           (LONG)g_usb_info.selector_unit_id,
                           (LONG)g_usb_info.selector_num_pins);
        /* Default to first pin (usually Microphone) */
        if (dd->ua_SelectorPin == 0)
            dd->ua_SelectorPin = 1;
    }

    /* ---- Query volume range from output Feature Unit ---- */
    if (found_fu && !g_usb_info.fu_vol_failed)
    {
        if (g_usb_info.fu_vol_cached)
        {
            /* Reuse cached values from previous successful query */
            dd->ua_VolMin = g_usb_info.fu_vol_min;
            dd->ua_VolMax = g_usb_info.fu_vol_max;
            dd->ua_VolRes = g_usb_info.fu_vol_res;
            DPRINTF("[USBAudio] discover: using cached volume range "
                               "MIN=%ld MAX=%ld RES=%ld\n",
                               (LONG)dd->ua_VolMin, (LONG)dd->ua_VolMax,
                               (LONG)dd->ua_VolRes);
        }
        else
        {
            uint8 vol_data[2];
            int16 vol_val;

            DPRINTF("[USBAudio] discover: querying output volume range...\n");

            r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                USB_AUDIO_REQ_GET_IF, USB_AUDIO_GET_MIN,
                (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
                (uint16)((dd->ua_FUnitID << 8) | dd->ua_ACIfaceNum),
                vol_data, 2, 500);
            if (r >= 2)
            {
                vol_val = (int16)(vol_data[0] | (vol_data[1] << 8));
                dd->ua_VolMin = vol_val;
                DPRINTF("[USBAudio] discover: Volume MIN = %ld (1/256 dB)\n", (LONG)vol_val);
            }
            else
            {
                dd->ua_VolMin = (int16)0x8001;
                DPRINTF("[USBAudio] discover: GET_MIN volume failed (%ld), disabling volume queries\n", r);
                g_usb_info.fu_vol_failed = 1;
                goto skip_vol;
            }

            r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                USB_AUDIO_REQ_GET_IF, USB_AUDIO_GET_MAX,
                (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
                (uint16)((dd->ua_FUnitID << 8) | dd->ua_ACIfaceNum),
                vol_data, 2, 500);
            if (r >= 2)
            {
                vol_val = (int16)(vol_data[0] | (vol_data[1] << 8));
                dd->ua_VolMax = vol_val;
                DPRINTF("[USBAudio] discover: Volume MAX = %ld (1/256 dB)\n", (LONG)vol_val);
            }
            else
            {
                dd->ua_VolMax = 0x0000;
                DPRINTF("[USBAudio] discover: GET_MAX volume failed (%ld)\n", r);
            }

            r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                USB_AUDIO_REQ_GET_IF, 0x84, /* GET_RES */
                (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
                (uint16)((dd->ua_FUnitID << 8) | dd->ua_ACIfaceNum),
                vol_data, 2, 500);
            if (r >= 2)
            {
                vol_val = (int16)(vol_data[0] | (vol_data[1] << 8));
                dd->ua_VolRes = vol_val;
                DPRINTF("[USBAudio] discover: Volume RES = %ld (1/256 dB)\n", (LONG)vol_val);
            }
            else
                dd->ua_VolRes = 0x0100;

            /* Cache for subsequent opens */
            g_usb_info.fu_vol_cached = 1;
            g_usb_info.fu_vol_min    = dd->ua_VolMin;
            g_usb_info.fu_vol_max    = dd->ua_VolMax;
            g_usb_info.fu_vol_res    = dd->ua_VolRes;
        }
    }
    else if (found_fu)
    {
        DPRINTF("[USBAudio] discover: skipping volume queries (previously failed)\n");
    }
skip_vol:

    /* ---- Query input gain range from recording Feature Unit ---- */
    if (found_fu_rec && !g_usb_info.fu_rec_vol_failed)
    {
        if (g_usb_info.fu_rec_vol_cached)
        {
            /* Reuse cached values from previous successful query */
            dd->ua_RecVolMin = g_usb_info.fu_rec_vol_min;
            dd->ua_RecVolMax = g_usb_info.fu_rec_vol_max;
            dd->ua_RecVolRes = g_usb_info.fu_rec_vol_res;
            DPRINTF("[USBAudio] discover: using cached input gain range "
                               "MIN=%ld MAX=%ld RES=%ld\n",
                               (LONG)dd->ua_RecVolMin, (LONG)dd->ua_RecVolMax,
                               (LONG)dd->ua_RecVolRes);
        }
        else
        {
            uint8 vol_data[2];
            int16 vol_val;

            DPRINTF("[USBAudio] discover: querying input gain range...\n");

            g_usb_info.fu_rec_found   = 1;
            g_usb_info.fu_rec_unit_id = dd->ua_RecFUnitID;

            r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                USB_AUDIO_REQ_GET_IF, USB_AUDIO_GET_MIN,
                (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
                (uint16)((dd->ua_RecFUnitID << 8) | dd->ua_ACIfaceNum),
                vol_data, 2, 500);
            if (r >= 2)
            {
                vol_val = (int16)(vol_data[0] | (vol_data[1] << 8));
                dd->ua_RecVolMin = vol_val;
                DPRINTF("[USBAudio] discover: Input Gain MIN = %ld (1/256 dB)\n", (LONG)vol_val);
            }
            else
            {
                dd->ua_RecVolMin = 0x0000;
                DPRINTF("[USBAudio] discover: GET_MIN input gain failed (%ld), disabling gain queries\n", r);
                g_usb_info.fu_rec_vol_failed = 1;
                goto skip_rec_vol;
            }

            r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                USB_AUDIO_REQ_GET_IF, USB_AUDIO_GET_MAX,
                (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
                (uint16)((dd->ua_RecFUnitID << 8) | dd->ua_ACIfaceNum),
                vol_data, 2, 500);
            if (r >= 2)
            {
                vol_val = (int16)(vol_data[0] | (vol_data[1] << 8));
                dd->ua_RecVolMax = vol_val;
                DPRINTF("[USBAudio] discover: Input Gain MAX = %ld (1/256 dB)\n", (LONG)vol_val);
            }
            else
            {
                dd->ua_RecVolMax = 0x0000;
                DPRINTF("[USBAudio] discover: GET_MAX input gain failed (%ld)\n", r);
            }

            r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                USB_AUDIO_REQ_GET_IF, 0x84, /* GET_RES */
                (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
                (uint16)((dd->ua_RecFUnitID << 8) | dd->ua_ACIfaceNum),
                vol_data, 2, 500);
            if (r >= 2)
            {
                vol_val = (int16)(vol_data[0] | (vol_data[1] << 8));
                dd->ua_RecVolRes = vol_val;
                DPRINTF("[USBAudio] discover: Input Gain RES = %ld (1/256 dB)\n", (LONG)vol_val);
            }
            else
                dd->ua_RecVolRes = 0x0100;

            /* Cache for subsequent opens */
            g_usb_info.fu_rec_vol_cached = 1;
            g_usb_info.fu_rec_vol_min    = dd->ua_RecVolMin;
            g_usb_info.fu_rec_vol_max    = dd->ua_RecVolMax;
            g_usb_info.fu_rec_vol_res    = dd->ua_RecVolRes;
        }
    }
    else if (found_fu_rec)
    {
        DPRINTF("[USBAudio] discover: skipping input gain queries (previously failed)\n");
    }
    else
    {
        DPRINTF("[USBAudio] discover: no input Feature Unit, input gain not available\n");
    }
skip_rec_vol:
    ;  /* empty statement after label */
}


/*****************************************************************************
 *
 * AHIsub_AllocAudio
 *
 * Called by AHI when a program wants to use this driver.
 * Discovers (or verifies) the USB audio device and allocates
 * per-instance driver data.
 *
 *****************************************************************************/

uint32 _usbaudio_AHIsub_AllocAudio(struct USBAudioIFace    *Self,
                                    struct TagItem          *tagList,
                                    struct AHIAudioCtrlDrv  *AudioCtrl)
{
    static int32 alloc_fail_count = 0;

    DPRINTF("[USBAudio] AHIsub_AllocAudio: entry\n");

    /* Ensure USB audio device has been discovered */
    scan_usb_audio_device();

    if (!g_usb_info.found)
    {
        alloc_fail_count++;
        if (alloc_fail_count <= 3)
            DPRINTF("[USBAudio] AHIsub_AllocAudio: no USB audio device => AHISF_ERROR\n");
        else if (alloc_fail_count == 4)
            DPRINTF("[USBAudio] AHIsub_AllocAudio: suppressing further 'no device' messages\n");
        return AHISF_ERROR;
    }

    alloc_fail_count = 0;  /* Reset on success */

    /* ---- Validate requested AudioID against the detected hardware ----
     *
     * When AHI's mode database is built (via AddAudioModes or
     * AHI_AddAudioMode), AHI calls AllocAudio for each mode with a
     * tagList that may contain either AHIA_AudioID (application tag,
     * TAG_USER+1) or AHIDB_AudioID (database tag, TAG_USER+100).
     * We check both so the filter fires regardless of which tag AHI uses.
     *
     * AudioID map (see gen_modefile.c):
     *   0x00550001 — HiFi 16-bit stereo++      (all devices)
     *   0x00550002 — HiFi 16-bit multichannel++ (devices with ≥4 channels)
     *   0x00550003 — 16-bit stereo             (all devices)
     */
    {
        /* Read AudioID: AHI may pass it as AHIA_AudioID (app tag) or          *
         * AHIDB_AudioID (database tag) depending on the call path.            */
        uint32 req_id = IUtility->GetTagData(AHIA_AudioID,  0, tagList);
        if (req_id == 0)
            req_id = IUtility->GetTagData(AHIDB_AudioID, 0, tagList);

        if (req_id == 0x00550002)
        {
            /* Multichannel mode: require at least 4 physical output channels */
            int32 max_ch = (int32)g_usb_info.nr_channels;  /* fallback: basic scan */
            if (g_usb_info.num_outputs > 0)
                max_ch = (int32)g_usb_info.outputs[0].nr_channels;
            if (max_ch < 4)
            {
                DPRINTF("[USBAudio] AllocAudio: reject AudioID 0x%08lx "
                                   "(multichannel needs >=4ch, device has %ld)\n",
                                   (ULONG)req_id, (LONG)max_ch);
                return AHISF_ERROR;
            }
        }
    }
    dd = IExec->AllocVecTags((uint32)sizeof(struct USBAudioData),
                             AVT_Type,           MEMF_SHARED,
                             AVT_Lock,           TRUE,
                             AVT_ClearWithValue, 0,
                             TAG_END);

    if (dd == NULL)
        return AHISF_ERROR;

    dd->ua_AHIsubBase    = (struct USBAudioBase *)Self->Data.LibBase;
    dd->ua_SlaveSignal   = -1;
    dd->ua_MasterSignal  = IExec->AllocSignal(-1);
    dd->ua_MasterTask    = (struct Process *)IExec->FindTask(NULL);

    if (dd->ua_MasterSignal == -1)
        return AHISF_ERROR;

    /* Copy device info from global scan results */
    dd->ua_VendorID      = g_usb_info.vid;
    dd->ua_ProductID     = g_usb_info.pid;
    dd->ua_InterfaceNum  = g_usb_info.iface_num;
    dd->ua_AltSetting    = g_usb_info.alt_setting;
    dd->ua_EndpointAddr  = g_usb_info.ep_addr;
    dd->ua_MaxPacketSize = g_usb_info.max_pkt;
    dd->ua_NumChannels   = g_usb_info.nr_channels;
    dd->ua_SubframeSize  = g_usb_info.subframe_size;
    dd->ua_BitResolution = g_usb_info.bit_resolution;
    dd->ua_DevHandle     = NULL;

    /* Copy channel offset from selected output mode */
    if (g_usb_info.num_outputs > 0 &&
        g_usb_info.selected_output >= 0 &&
        g_usb_info.selected_output < g_usb_info.num_outputs)
        dd->ua_ChannelOffset = g_usb_info.outputs[g_usb_info.selected_output].channel_offset;
    else
        dd->ua_ChannelOffset = 0;

    /* Copy recording endpoint info (if found during scan) */
    if (g_usb_info.rec_found)
    {
        dd->ua_RecInterfaceNum  = g_usb_info.rec_iface_num;
        dd->ua_RecAltSetting    = g_usb_info.rec_alt_setting;
        dd->ua_RecEndpointAddr  = g_usb_info.rec_ep_addr;
        dd->ua_RecNumChannels   = g_usb_info.rec_nr_channels;
        dd->ua_RecMaxPacketSize = g_usb_info.rec_max_pkt;
        dd->ua_RecSubframeSize  = g_usb_info.rec_subframe_size;
        dd->ua_RecBitResolution = g_usb_info.rec_bit_resolution;
    }

    /* Copy Audio Control Feature Unit info (output) */
    if (g_usb_info.fu_found)
    {
        dd->ua_ACIfaceNum = g_usb_info.ac_iface_num;
        dd->ua_FUnitID    = g_usb_info.fu_unit_id;
    }

    /* Copy Audio Control Feature Unit info (input/recording) */
    if (g_usb_info.fu_rec_found)
    {
        dd->ua_RecFUnitID = g_usb_info.fu_rec_unit_id;
        DPRINTF("[USBAudio] AllocAudio: input FU ID=%ld\n",
                           (LONG)dd->ua_RecFUnitID);
    }

    /* Copy Selector Unit info */
    dd->ua_SelectorUnitID = g_usb_info.selector_unit_id;
    dd->ua_SelectorPin    = 0;  /* Will be set on first AHIC_Input or Start */
    if (g_usb_info.selector_unit_id != 0 &&
        g_usb_info.num_inputs > 0 &&
        g_usb_info.inputs[0].su_pin != 0)
    {
        dd->ua_SelectorPin = g_usb_info.inputs[0].su_pin;
    }

    /* Initialize volume/gain defaults to unity (0 dB) */
    dd->ua_OutputVolume  = 0x10000;  /* 0 dB */
    dd->ua_MonitorVolume = 0x00000;  /* Silent */
    dd->ua_InputGain     = 0x10000;  /* 0 dB */
    dd->ua_Input         = 0;
    dd->ua_Output        = 0;
    dd->ua_RecSlaveSignal = -1;

    /* --- Snap ahiac_MixFreq to the closest device-supported rate -----
     * AHI passes whatever the application requested (e.g. 11025 Hz).
     * USB Audio devices only decode data at their native rate(s).
     * Sending 11025 Hz data to a 44100 Hz DAC starves its buffer
     * and produces silence, even though USB transfers complete fine.
     * By writing the snapped value here, AHI's internal mixer will
     * generate PCM at that rate, SET_CUR will use a valid rate, and
     * the PlaySlave will send the correct byte density to the device. */
    if (g_usb_info.num_frequencies > 0 && AudioCtrl->ahiac_MixFreq > 0)
    {
        uint32 req   = (uint32)AudioCtrl->ahiac_MixFreq;
        uint32 best  = g_usb_info.frequencies[0];
        uint32 bestd = (req > best) ? (req - best) : (best - req);
        int32  fi;

        for (fi = 1; fi < g_usb_info.num_frequencies; fi++)
        {
            uint32 d = (req > g_usb_info.frequencies[fi]) ?
                       (req - g_usb_info.frequencies[fi]) :
                       (g_usb_info.frequencies[fi] - req);
            if (d < bestd) { bestd = d; best = g_usb_info.frequencies[fi]; }
        }

        if (best != req)
        {
            DPRINTF("[USBAudio] AllocAudio: snapping MixFreq %lu -> %lu Hz "
                               "(device supported)\n", (ULONG)req, (ULONG)best);
            AudioCtrl->ahiac_MixFreq = (ULONG)best;
        }
    }

    DPRINTF("[USBAudio] AHIsub_AllocAudio: OK, VID=0x%04lx PID=0x%04lx%s\n",
                       (ULONG)dd->ua_VendorID, (ULONG)dd->ua_ProductID,
                       g_usb_info.rec_found ? " (can record)" : "");
    return (uint32)(AHISF_KNOWHIFI | AHISF_KNOWSTEREO | AHISF_KNOWMULTICHANNEL | AHISF_MIXING | AHISF_TIMING |
                    (g_usb_info.rec_found ? AHISF_CANRECORD : 0));
}


/*****************************************************************************
 *
 * AHIsub_FreeAudio
 *
 *****************************************************************************/

void _usbaudio_AHIsub_FreeAudio(struct USBAudioIFace    *Self,
                                 struct AHIAudioCtrlDrv  *AudioCtrl)
{
    DPRINTF("[USBAudio] AHIsub_FreeAudio\n");
    if (AudioCtrl->ahiac_DriverData)
    {
        /* Safety net: if Stop was never called, clean up USB state */
        if (dd->ua_DevHandle)
        {
            if (dd->ua_DeviceGone)
            {
                DPRINTF("[USBAudio] FreeAudio: device was hot-removed, skipping USB I/O\n");
            }
            else
            {
                DPRINTF("[USBAudio] FreeAudio: device still open, forcing cleanup\n");
                if (dd->ua_RecEndpointAddr != 0)
                {
                    ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle, dd->ua_RecInterfaceNum, 0);
                    ILibusb1->libusb_release_interface(dd->ua_DevHandle, dd->ua_RecInterfaceNum);
                }
                ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle, dd->ua_InterfaceNum, 0);
                ILibusb1->libusb_release_interface(dd->ua_DevHandle, dd->ua_InterfaceNum);
            }
            ILibusb1->libusb_close(dd->ua_DevHandle);
            dd->ua_DevHandle = NULL;
            g_active_handle = NULL;
            g_active_iface  = -1;
            g_active_rec_iface = -1;
        }
        IExec->FreeVec(dd->ua_MixBuffer);
        IExec->FreeVec(dd->ua_USBBuffer);
        IExec->FreeSignal(dd->ua_MasterSignal);
        IExec->FreeVec(AudioCtrl->ahiac_DriverData);
        AudioCtrl->ahiac_DriverData = NULL;
    }
}


/*****************************************************************************
 *
 * AHIsub_Disable / AHIsub_Enable
 *
 *****************************************************************************/

void _usbaudio_AHIsub_Disable(struct USBAudioIFace    *Self,
                               struct AHIAudioCtrlDrv  *AudioCtrl)
{
    /* V6 drivers do not have to preserve all registers */
    IExec->Disable();
}

void _usbaudio_AHIsub_Enable(struct USBAudioIFace    *Self,
                              struct AHIAudioCtrlDrv  *AudioCtrl)
{
    /* V6 drivers do not have to preserve all registers */
    IExec->Enable();
}


/*****************************************************************************
 *
 * AHIsub_Start
 *
 * Opens the USB device, claims the streaming interface,
 * configures sample rate, and launches the playback process.
 *
 *****************************************************************************/

uint32 _usbaudio_AHIsub_Start(struct USBAudioIFace    *Self,
                               uint32                   Flags,
                               struct AHIAudioCtrlDrv  *AudioCtrl)
{
    DPRINTF("[USBAudio] AHIsub_Start: Flags=0x%08lx\n", Flags);

    /* Stop any running playback first */
    _usbaudio_AHIsub_Stop(Self, Flags, AudioCtrl);

    if (Flags & AHISF_PLAY)
    {
        int32 r;

        /* ---------- 0. Reject if another session is still playing ---------- */
        /* USB audio is exclusive: only one playback stream can use the
         * isochronous endpoint at a time.  If a slave process from
         * another AHI session is still running, we must NOT kill it.
         * Instead, fail this Start so the first session keeps playing. */
        if (g_active_handle != NULL && g_active_handle != dd->ua_DevHandle
            && g_active_slave != NULL)
        {
            DPRINTF("[USBAudio] Start: BUSY — another session is still playing, refusing\n");
            return AHIE_UNKNOWN;
        }

        /* If the old handle exists but no slave is running, the previous
         * session ended abnormally (FreeAudio without Stop). Clean up. */
        if (g_active_handle != NULL && g_active_handle != dd->ua_DevHandle)
        {
            DPRINTF("[USBAudio] Start: cleaning up stale handle %p (no active slave)\n", g_active_handle);
            if (g_active_rec_iface >= 0)
            {
                ILibusb1->libusb_set_interface_alt_setting(g_active_handle, g_active_rec_iface, 0);
                ILibusb1->libusb_release_interface(g_active_handle, g_active_rec_iface);
            }
            if (g_active_iface >= 0)
            {
                ILibusb1->libusb_set_interface_alt_setting(g_active_handle, g_active_iface, 0);
                ILibusb1->libusb_release_interface(g_active_handle, g_active_iface);
            }
            ILibusb1->libusb_close(g_active_handle);
            g_active_handle    = NULL;
            g_active_iface     = -1;
            g_active_rec_iface = -1;
            g_active_slave     = NULL;
            g_active_slave_sig = -1;
            g_active_rec_slave = NULL;
            g_active_rec_sig   = -1;
            g_active_master_sig = -1;
            g_active_master_task = NULL;
        }

        /* ---------- 1. Open the USB audio device ---------- */
        dd->ua_DevHandle = ILibusb1->libusb_open_device_with_vid_pid(
                               NULL, dd->ua_VendorID, dd->ua_ProductID);
        DPRINTF("[USBAudio] Start: open_device VID=0x%04lx PID=0x%04lx => handle=%p\n",
                           (ULONG)dd->ua_VendorID, (ULONG)dd->ua_ProductID, dd->ua_DevHandle);
        if (dd->ua_DevHandle == NULL)
            return AHIE_UNKNOWN;

        /* ---------- 2. Discover real endpoint / format ---------- */
        /* The scan phase only sees alt=0 (zero-bandwidth, no endpoints), so
         * ep_addr defaults to 0x01 which is wrong.  This reads the actual USB
         * descriptor blob via GET_DESCRIPTOR control transfer and finds the
         * real isochronous OUT endpoint address (e.g. 0x05). */
        discover_usb_audio_details(AudioCtrl);

        DPRINTF("[USBAudio] Start: after discover => EP=0x%02lx Ifc=%ld Alt=%ld MaxPkt=%ld Ch=%ld Sub=%ld\n",
                           (LONG)dd->ua_EndpointAddr, (LONG)dd->ua_InterfaceNum,
                           (LONG)dd->ua_AltSetting, (LONG)dd->ua_MaxPacketSize,
                           (LONG)dd->ua_NumChannels, (LONG)dd->ua_SubframeSize);

        /* ---------- 3. Allocate mix/USB buffers ---------- */
        /* Done AFTER discover so that channel-count, subframe size and
         * max-packet-size are the real device values. */
        if (!(dd->ua_MixBuffer = IExec->AllocVecTags(AudioCtrl->ahiac_BuffSize,
                                                      AVT_Type, MEMF_SHARED,
                                                      AVT_Lock, TRUE,
                                                      AVT_Contiguous, TRUE,
                                                      AVT_ClearWithValue, 0,
                                                      TAG_END)))
        {
            ILibusb1->libusb_close(dd->ua_DevHandle);
            dd->ua_DevHandle = NULL;
            return AHIE_NOMEM;
        }

        /* USB staging buffer size.
         * Only needs to hold one AHI mix output (used as staging for
         * the async isochronous playback).  The PlaySlave allocates
         * its own DMA buffers for USBIOReqs. */
        {
            uint32 audio_bytes = AudioCtrl->ahiac_MaxBuffSamples *
                                 dd->ua_NumChannels * dd->ua_SubframeSize;
            dd->ua_USBBufferSize = audio_bytes;
        }

        if (!(dd->ua_USBBuffer = (uint8 *)IExec->AllocVecTags(dd->ua_USBBufferSize,
                                                               AVT_Type, MEMF_SHARED,
                                                               AVT_Lock, TRUE,
                                                               AVT_Contiguous, TRUE,
                                                               TAG_END)))
        {
            IExec->FreeVec(dd->ua_MixBuffer);
            dd->ua_MixBuffer = NULL;
            ILibusb1->libusb_close(dd->ua_DevHandle);
            dd->ua_DevHandle = NULL;
            return AHIE_NOMEM;
        }

        /* Buffers 2 and 3 no longer needed — async ISO IO uses its own
         * per-IOReq DMA buffers allocated in the PlaySlave process. */
        dd->ua_USBBuffer2 = NULL;
        dd->ua_USBBuffer3 = NULL;

        /* ---------- 4. Claim the audio streaming interface ---------- */
        r = ILibusb1->libusb_claim_interface(dd->ua_DevHandle, dd->ua_InterfaceNum);
        DPRINTF("[USBAudio] Start: claim_interface(%ld) => %ld\n",
                           (LONG)dd->ua_InterfaceNum, r);
        if (r != LIBUSB_SUCCESS)
        {
            ILibusb1->libusb_close(dd->ua_DevHandle);
            dd->ua_DevHandle = NULL;
            IExec->FreeVec(dd->ua_USBBuffer);
            dd->ua_USBBuffer = NULL;
            IExec->FreeVec(dd->ua_MixBuffer);
            dd->ua_MixBuffer = NULL;
            return AHIE_UNKNOWN;
        }

        /* Set alternate setting 1 to activate the isochronous endpoint.
         * Alt 0 = zero bandwidth (no endpoint), Alt 1 = active streaming.
         * The new implementation in libusb-1 releases the alt=0 raw
         * interface and claims the alt=1 one, so no DoIO blocking. */
        DPRINTF("[USBAudio] Start: calling set_alt_setting(%ld, %ld)...\n",
                           (LONG)dd->ua_InterfaceNum, (LONG)dd->ua_AltSetting);
        r = ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle,
                                                        dd->ua_InterfaceNum,
                                                        dd->ua_AltSetting);
        DPRINTF("[USBAudio] Start: set_alt_setting => %ld\n", r);
        if (r != LIBUSB_SUCCESS)
        {
            DPRINTF("[USBAudio] Start: set_alt_setting FAILED (%ld), aborting\n", r);
            ILibusb1->libusb_release_interface(dd->ua_DevHandle, dd->ua_InterfaceNum);
            ILibusb1->libusb_close(dd->ua_DevHandle);
            dd->ua_DevHandle = NULL;
            g_active_handle = NULL;
            g_active_iface  = -1;
            IExec->FreeVec(dd->ua_USBBuffer);
            dd->ua_USBBuffer = NULL;
            IExec->FreeVec(dd->ua_MixBuffer);
            dd->ua_MixBuffer = NULL;
            return AHIE_UNKNOWN;
        }

        /* Track this session globally so a future Start can clean it up
         * if AHI abandons this session without calling Stop/FreeAudio. */
        g_active_handle     = dd->ua_DevHandle;
        g_active_iface      = dd->ua_InterfaceNum;
        g_active_master_sig = dd->ua_MasterSignal;
        g_active_master_task = (struct Task *)IExec->FindTask(NULL);

        /* ---------- 5. Set sample rate on the endpoint ---------- */
        /* USB Audio Class 1.0: SET_CUR on SAMPLING_FREQ_CONTROL.
         * Without this the device may run at an undefined rate,
         * producing noise or wrong-pitch audio.
         * Only send if the CS_ENDPOINT descriptor declared D0=1
         * (Sampling Frequency Control supported).  Devices with D0=0
         * will always STALL this request per USB Audio Class 1.0 §5.2.3. */
        if (!dd->ua_RateCtrl)
        {
            DPRINTF("[USBAudio] Start: device does not declare sampling freq control (CS_ENDPOINT D0=0), skipping SET_CUR\n");
        }
        else
        {
            uint32 rate = AudioCtrl->ahiac_MixFreq;
            uint8  rate_data[3];

            rate_data[0] = (uint8)(rate & 0xFF);
            rate_data[1] = (uint8)((rate >> 8) & 0xFF);
            rate_data[2] = (uint8)((rate >> 16) & 0xFF);

            DPRINTF("[USBAudio] Start: setting sample rate %lu Hz on EP 0x%02lx\n",
                               (ULONG)rate, (ULONG)dd->ua_EndpointAddr);

            r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                    0x22,  /* Host-to-Device | Class | Endpoint */
                    USB_AUDIO_SET_CUR,
                    USB_AUDIO_SAMPLING_FREQ_CONTROL,
                    (uint16)dd->ua_EndpointAddr,
                    rate_data, 3, 5000);

            if (r < 0)
            {
                if (r == -9 /* LIBUSB_ERROR_PIPE: device STALLed the request */)
                    DPRINTF("[USBAudio] Start: SET_CUR sample rate not accepted (STALL) - device uses fixed hardware rate\n");
                else
                    DPRINTF("[USBAudio] Start: SET_CUR sample rate failed: %ld\n", r);
            }
            else
                DPRINTF("[USBAudio] Start: SET_CUR sample rate OK\n");
        }

        /* Create the playback slave process */
        IExec->Forbid();

        dd->ua_SlaveTask = IDOS->CreateNewProcTags(
            NP_Entry,     (uint32)hwUSBPlaySlave,
            NP_Name,      (uint32)LIBNAME " Playback",
            NP_UserData,  AudioCtrl,
            NP_Priority,  5,
            NP_StackSize, 16000,
            TAG_END);

        IExec->Permit();

        if (dd->ua_SlaveTask)
        {
            DPRINTF("[USBAudio] Start: waiting for slave to signal alive...\n");
            IExec->Wait(1L << dd->ua_MasterSignal);  /* Wait for slave alive */
            if (dd->ua_SlaveTask == NULL)
            {
                DPRINTF("[USBAudio] Start: slave died during init!\n");
                return AHIE_UNKNOWN;   /* Slave died during init */
            }
            DPRINTF("[USBAudio] Start: slave alive, playback running\n");

            /* Track the running slave so the BUSY check can protect it */
            g_active_slave     = dd->ua_SlaveTask;
            g_active_slave_sig = dd->ua_SlaveSignal;
        }
        else
        {
            DPRINTF("[USBAudio] Start: CreateNewProcTags failed!\n");
            return AHIE_NOMEM;
        }
    }

    /* ==================== RECORDING ==================== */
    if (Flags & AHISF_RECORD)
    {
        DPRINTF("[USBAudio] Start: RECORD requested\n");

        if (!g_usb_info.rec_found)
        {
            DPRINTF("[USBAudio] Start: no recording interface found during scan\n");
            return AHIE_UNKNOWN;
        }

        /* Track whether WE opened the device here (record-only mode).  If PLAY
         * already opened it, the handle is shared and must NOT be closed on a
         * recording-setup failure. */
        int32 rec_opened_device = 0;

        /* If device is not yet open (record-only mode), open it now */
        if (dd->ua_DevHandle == NULL)
        {
            dd->ua_DevHandle = ILibusb1->libusb_open_device_with_vid_pid(
                                   NULL, dd->ua_VendorID, dd->ua_ProductID);
            if (dd->ua_DevHandle == NULL)
            {
                DPRINTF("[USBAudio] Start: cannot open device for recording\n");
                return AHIE_UNKNOWN;
            }
            rec_opened_device = 1;
        }

        /* The scan phase only sees alt 0 (zero bandwidth, no endpoints),
         * so the IN endpoint address may still be 0.  Run discover to
         * parse the full config descriptor and find the real endpoint. */
        if (dd->ua_RecEndpointAddr == 0)
        {
            DPRINTF("[USBAudio] Start: rec endpoint unknown, running discover...\n");
            discover_usb_audio_details(AudioCtrl);
            DPRINTF("[USBAudio] Start: after discover => RecEP=0x%02lx RecIfc=%ld RecAlt=%ld\n",
                               (LONG)dd->ua_RecEndpointAddr, (LONG)dd->ua_RecInterfaceNum,
                               (LONG)dd->ua_RecAltSetting);

            if (dd->ua_RecEndpointAddr == 0)
            {
                DPRINTF("[USBAudio] Start: no recording endpoint found even after discover\n");
                return AHIE_UNKNOWN;
            }
        }

        /* Claim the recording interface */
        {
            int32 r;
            r = ILibusb1->libusb_claim_interface(dd->ua_DevHandle, dd->ua_RecInterfaceNum);
            DPRINTF("[USBAudio] Start: claim_interface(%ld) for recording => %ld\n",
                               (LONG)dd->ua_RecInterfaceNum, r);
            if (r != LIBUSB_SUCCESS)
            {
                DPRINTF("[USBAudio] Start: cannot claim recording interface\n");
                if (rec_opened_device)
                {
                    ILibusb1->libusb_close(dd->ua_DevHandle);
                    dd->ua_DevHandle = NULL;
                }
                return AHIE_UNKNOWN;
            }

            /* Set alt setting to activate the isoch IN endpoint */
            r = ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle,
                                                            dd->ua_RecInterfaceNum,
                                                            dd->ua_RecAltSetting);
            DPRINTF("[USBAudio] Start: set_alt_setting(%ld, %ld) for recording => %ld\n",
                               (LONG)dd->ua_RecInterfaceNum, (LONG)dd->ua_RecAltSetting, r);
            if (r != LIBUSB_SUCCESS)
            {
                DPRINTF("[USBAudio] Start: rec set_alt_setting failed, aborting\n");
                ILibusb1->libusb_release_interface(dd->ua_DevHandle, dd->ua_RecInterfaceNum);
                if (rec_opened_device)
                {
                    ILibusb1->libusb_close(dd->ua_DevHandle);
                    dd->ua_DevHandle = NULL;
                }
                return AHIE_UNKNOWN;
            }

            g_active_rec_iface = dd->ua_RecInterfaceNum;

            /* Set sample rate on the recording endpoint */
            {
                uint32 rate = AudioCtrl->ahiac_MixFreq;
                uint8  rate_data[3];

                rate_data[0] = (uint8)(rate & 0xFF);
                rate_data[1] = (uint8)((rate >> 8) & 0xFF);
                rate_data[2] = (uint8)((rate >> 16) & 0xFF);

                DPRINTF("[USBAudio] Start: setting rec sample rate %lu Hz on EP 0x%02lx\n",
                                   (ULONG)rate, (ULONG)dd->ua_RecEndpointAddr);

                r = ILibusb1->libusb_control_transfer(dd->ua_DevHandle,
                        0x22,  /* Host-to-Device | Class | Endpoint */
                        USB_AUDIO_SET_CUR,
                        USB_AUDIO_SAMPLING_FREQ_CONTROL,
                        (uint16)dd->ua_RecEndpointAddr,
                        rate_data, 3, 5000);

                if (r < 0)
                    DPRINTF("[USBAudio] Start: SET_CUR rec sample rate failed: %ld\n", r);
            }

            /* Set Selector Unit to the currently selected input source */
            set_usb_selector(dd, dd->ua_SelectorPin);
        }

        /* Allocate recording buffers */
        dd->ua_RecBufferSize = USB_AUDIO_REC_SAMPLES *
                               (uint32)dd->ua_RecNumChannels *
                               (uint32)dd->ua_RecSubframeSize;
        if (dd->ua_RecBufferSize < (uint32)dd->ua_RecMaxPacketSize * 10)
            dd->ua_RecBufferSize = (uint32)dd->ua_RecMaxPacketSize * 10;

        dd->ua_RecBuffer = (uint8 *)IExec->AllocVecTags(dd->ua_RecBufferSize,
                                                         AVT_Type, MEMF_SHARED,
                                                         AVT_Lock, TRUE,
                                                         AVT_Contiguous, TRUE,
                                                         AVT_ClearWithValue, 0,
                                                         TAG_END);
        if (dd->ua_RecBuffer == NULL)
        {
            DPRINTF("[USBAudio] Start: cannot allocate recording buffer\n");
            ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle,
                                                        dd->ua_RecInterfaceNum, 0);
            ILibusb1->libusb_release_interface(dd->ua_DevHandle, dd->ua_RecInterfaceNum);
            g_active_rec_iface = -1;
            if (rec_opened_device)
            {
                ILibusb1->libusb_close(dd->ua_DevHandle);
                dd->ua_DevHandle = NULL;
            }
            return AHIE_NOMEM;
        }

        dd->ua_IsRecording = TRUE;

        /* Create the recording slave process */
        IExec->Forbid();

        dd->ua_RecSlaveTask = IDOS->CreateNewProcTags(
            NP_Entry,     (uint32)hwUSBRecordSlave,
            NP_Name,      (uint32)LIBNAME " Recording",
            NP_UserData,  AudioCtrl,
            NP_Priority,  1,
            NP_StackSize, 16000,
            TAG_END);

        IExec->Permit();

        if (dd->ua_RecSlaveTask)
        {
            DPRINTF("[USBAudio] Start: waiting for record slave to signal alive...\n");
            IExec->Wait(1L << dd->ua_MasterSignal);
            if (dd->ua_RecSlaveTask == NULL)
            {
                DPRINTF("[USBAudio] Start: record slave died during init!\n");
                dd->ua_IsRecording = FALSE;
                IExec->FreeVec(dd->ua_RecBuffer);
                dd->ua_RecBuffer = NULL;
                ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle,
                                                            dd->ua_RecInterfaceNum, 0);
                ILibusb1->libusb_release_interface(dd->ua_DevHandle, dd->ua_RecInterfaceNum);
                g_active_rec_iface = -1;
                if (rec_opened_device)
                {
                    ILibusb1->libusb_close(dd->ua_DevHandle);
                    dd->ua_DevHandle = NULL;
                }
                return AHIE_UNKNOWN;
            }
            DPRINTF("[USBAudio] Start: record slave alive, recording running\n");

            g_active_rec_slave = dd->ua_RecSlaveTask;
            g_active_rec_sig   = dd->ua_RecSlaveSignal;
        }
        else
        {
            DPRINTF("[USBAudio] Start: CreateNewProcTags for recording failed!\n");
            dd->ua_IsRecording = FALSE;
            IExec->FreeVec(dd->ua_RecBuffer);
            dd->ua_RecBuffer = NULL;
            ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle,
                                                        dd->ua_RecInterfaceNum, 0);
            ILibusb1->libusb_release_interface(dd->ua_DevHandle, dd->ua_RecInterfaceNum);
            g_active_rec_iface = -1;
            if (rec_opened_device)
            {
                ILibusb1->libusb_close(dd->ua_DevHandle);
                dd->ua_DevHandle = NULL;
            }
            return AHIE_NOMEM;
        }
    }

    DPRINTF("[USBAudio] AHIsub_Start: OK\n");
    return (uint32)AHIE_OK;
}


/*****************************************************************************
 *
 * AHIsub_Update
 *
 *****************************************************************************/

uint32 _usbaudio_AHIsub_Update(struct USBAudioIFace    *Self,
                                uint32                   Flags,
                                struct AHIAudioCtrlDrv  *AudioCtrl)
{
    return (uint32)AHIE_OK;
}


/*****************************************************************************
 *
 * AHIsub_Stop
 *
 * Kill the slave process, deactivate the USB endpoint,
 * release interface, close device, free buffers.
 *
 *****************************************************************************/

uint32 _usbaudio_AHIsub_Stop(struct USBAudioIFace    *Self,
                              uint32                   Flags,
                              struct AHIAudioCtrlDrv  *AudioCtrl)
{
    DPRINTF("[USBAudio] AHIsub_Stop: Flags=0x%08lx\n", Flags);

    /* Defensive: AHIsub_Start calls us first thing, and AHI may call Stop on
     * a session whose DriverData was never allocated (or already freed).
     * Every access below goes through dd = *ahiac_DriverData, so bail out. */
    if (AudioCtrl == NULL || AudioCtrl->ahiac_DriverData == NULL)
    {
        DPRINTF("[USBAudio] AHIsub_Stop: no DriverData, nothing to stop\n");
        return AHIE_OK;
    }

    if (Flags & AHISF_PLAY)
    {
        /* Signal slave to terminate */
        if (dd->ua_SlaveTask)
        {
            if (dd->ua_SlaveSignal != -1)
            {
                IExec->Signal((struct Task *)dd->ua_SlaveTask,
                              1L << dd->ua_SlaveSignal);
            }
            IExec->Wait(1L << dd->ua_MasterSignal);  /* Wait for slave death */
        }

        /* Deactivate USB endpoint and close device.
         *
         * set_alt_setting(iface, 0) is required to reset the
         * rawIfc back to alt=0, otherwise claim_interface will
         * fail on the next Start (it searches for alt==0).
         *
         * However, set_alt_setting does a declaim→SetAlt→re-claim
         * cycle that destroys the endpoint object.  If the USB
         * stack task still holds internal references to that
         * endpoint from recent I/O, it will crash accessing
         * freed memory.
         *
         * Workaround: wait 500ms after the slave exits to let
         * the USB stack fully finalize all I/O on the endpoint
         * before we destroy it via set_alt_setting.
         */
        if (dd->ua_DevHandle)
        {
            if (dd->ua_DeviceGone)
            {
                /* Device was hot-removed — skip all USB I/O,
                 * just close the handle to free memory. */
                DPRINTF("[USBAudio] Stop: device was hot-removed, skipping USB I/O\n");
                ILibusb1->libusb_close(dd->ua_DevHandle);
            }
            else
            {
                /* Give the USB stack time to finalize endpoint I/O */
                DPRINTF("[USBAudio] Stop: waiting 500ms for USB stack to settle...\n");
                IDOS->Delay(25);  /* 25 ticks = 500ms (1 tick = 20ms) */

                /* Switch to alt 0 to deactivate the isochronous endpoint
                 * and reset rawIfc so claim_interface can find it again. */
                DPRINTF("[USBAudio] Stop: set_alt_setting(%lu, 0)...\n",
                                   (ULONG)dd->ua_InterfaceNum);
                ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle,
                                                            dd->ua_InterfaceNum, 0);

                DPRINTF("[USBAudio] Stop: release_interface(%lu)...\n",
                                   (ULONG)dd->ua_InterfaceNum);
                ILibusb1->libusb_release_interface(dd->ua_DevHandle,
                                                    dd->ua_InterfaceNum);

                DPRINTF("[USBAudio] Stop: closing device...\n");
                ILibusb1->libusb_close(dd->ua_DevHandle);
            }
            dd->ua_DevHandle = NULL;

            /* Clear global tracking — this session is properly torn down */
            g_active_handle     = NULL;
            g_active_iface      = -1;
            g_active_slave      = NULL;
            g_active_slave_sig  = -1;
            g_active_master_sig = -1;
            g_active_master_task = NULL;

            DPRINTF("[USBAudio] Stop: device closed OK\n");
        }

        IExec->FreeVec(dd->ua_MixBuffer);
        dd->ua_MixBuffer = NULL;
        IExec->FreeVec(dd->ua_USBBuffer);
        dd->ua_USBBuffer = NULL;
    }

    /* ==================== STOP RECORDING ==================== */
    if (Flags & AHISF_RECORD)
    {
        DPRINTF("[USBAudio] Stop: stopping recording\n");

        /* Signal record slave to terminate */
        if (dd->ua_RecSlaveTask)
        {
            if (dd->ua_RecSlaveSignal != -1)
            {
                IExec->Signal((struct Task *)dd->ua_RecSlaveTask,
                              1L << dd->ua_RecSlaveSignal);
            }
            IExec->Wait(1L << dd->ua_MasterSignal);  /* Wait for record slave death */
        }

        dd->ua_IsRecording = FALSE;

        /* Deactivate recording endpoint */
        if (dd->ua_DevHandle && dd->ua_RecEndpointAddr != 0)
        {
            if (dd->ua_DeviceGone)
            {
                DPRINTF("[USBAudio] Stop: rec device was hot-removed, skipping USB I/O\n");
            }
            else
            {
                DPRINTF("[USBAudio] Stop: waiting 500ms for rec USB stack to settle...\n");
                IDOS->Delay(25);

                DPRINTF("[USBAudio] Stop: set_alt_setting(%lu, 0) for recording...\n",
                                   (ULONG)dd->ua_RecInterfaceNum);
                ILibusb1->libusb_set_interface_alt_setting(dd->ua_DevHandle,
                                                            dd->ua_RecInterfaceNum, 0);

                DPRINTF("[USBAudio] Stop: release_interface(%lu) for recording...\n",
                                   (ULONG)dd->ua_RecInterfaceNum);
                ILibusb1->libusb_release_interface(dd->ua_DevHandle,
                                                    dd->ua_RecInterfaceNum);
            }
            g_active_rec_iface = -1;
        }

        g_active_rec_slave = NULL;
        g_active_rec_sig   = -1;

        IExec->FreeVec(dd->ua_RecBuffer);
        dd->ua_RecBuffer = NULL;
    }

    return (uint32)AHIE_OK;
}


/*****************************************************************************
 *
 * AHIsub_GetAttr
 *
 * Reports device capabilities to AHI and AHI preferences.
 * Uses hardcoded defaults matching the MODEFILE.
 * Actual USB device discovery happens in AllocAudio/Start.
 *
 *****************************************************************************/

int32 _usbaudio_AHIsub_GetAttr(struct USBAudioIFace    *Self,
                                uint32                   Attribute,
                                int32                    Argument,
                                int32                    DefValue,
                                struct TagItem          *tagList,
                                struct AHIAudioCtrlDrv  *AudioCtrl)
{
    /* Ensure USB scan has run so we return real device capabilities.
     * AHI calls GetAttr directly (via AHI_GetAudioAttrsA) without always
     * calling AllocAudio first, so the lazy scan must happen here too. */
    scan_usb_audio_device();

    DPRINTF("[USBAudio] GetAttr: Attr=0x%08lx Arg=%ld Def=%ld\n",
                       Attribute, Argument, DefValue);

    switch (Attribute)
    {
        case AHIDB_Bits:
            return 16;

        case AHIDB_MinMixFreq:
            /* Minimum mixing rate: lowest value in the device's frequency list */
            if (g_usb_info.num_frequencies > 0)
            {
                uint32 minf = g_usb_info.frequencies[0];
                int32 fi;
                for (fi = 1; fi < g_usb_info.num_frequencies; fi++)
                    if (g_usb_info.frequencies[fi] < minf)
                        minf = g_usb_info.frequencies[fi];
                return (LONG)minf;
            }
            return 44100;

        case AHIDB_MaxMixFreq:
            /* Maximum mixing rate: highest value in the device's frequency list */
            if (g_usb_info.num_frequencies > 0)
            {
                uint32 maxf = g_usb_info.frequencies[0];
                int32 fi;
                for (fi = 1; fi < g_usb_info.num_frequencies; fi++)
                    if (g_usb_info.frequencies[fi] > maxf)
                        maxf = g_usb_info.frequencies[fi];
                return (LONG)maxf;
            }
            return 48000;

        case AHIDB_Frequencies:
            /* Number of discrete mixing rates supported */
            return (g_usb_info.num_frequencies > 0) ?
                   (LONG)g_usb_info.num_frequencies : 2;

        case AHIDB_Frequency:   /* Index -> Frequency */
            if (g_usb_info.num_frequencies > 0)
            {
                if (Argument >= 0 && Argument < g_usb_info.num_frequencies)
                    return (LONG)g_usb_info.frequencies[Argument];
                return DefValue;
            }
            if (Argument == 0) return 44100;
            if (Argument == 1) return 48000;
            return DefValue;

        case AHIDB_Index:       /* Frequency -> closest Index */
            if (g_usb_info.num_frequencies > 0)
            {
                uint32 req  = (uint32)Argument;
                int32  best = 0;
                uint32 bestd = (req > g_usb_info.frequencies[0]) ?
                               (req - g_usb_info.frequencies[0]) :
                               (g_usb_info.frequencies[0] - req);
                int32 fi;
                for (fi = 1; fi < g_usb_info.num_frequencies; fi++)
                {
                    uint32 d = (req > g_usb_info.frequencies[fi]) ?
                               (req - g_usb_info.frequencies[fi]) :
                               (g_usb_info.frequencies[fi] - req);
                    if (d < bestd) { bestd = d; best = fi; }
                }
                return best;
            }
            if (Argument <= 46050)  /* midpoint between 44100 and 48000 */
                return 0;
            return 1;

        case AHIDB_Author:
            return (LONG)"Andrea Palmate";

        case AHIDB_Copyright:
            return (LONG)"(C) 2026 Andrea Palmatè";

        case AHIDB_Version:
            return (LONG)VSTRING;

        case AHIDB_Record:
            DPRINTF("[USBAudio] GetAttr: AHIDB_Record => %ld (rec_found=%ld scanned=%ld)\n",
                               (LONG)(g_usb_info.rec_found ? TRUE : FALSE),
                               (LONG)g_usb_info.rec_found, (LONG)g_usb_info.scanned);
            return (g_usb_info.rec_found ? TRUE : FALSE);

        case AHIDB_FullDuplex:
            DPRINTF("[USBAudio] GetAttr: AHIDB_FullDuplex => %ld (rec_found=%ld)\n",
                               (LONG)(g_usb_info.rec_found ? TRUE : FALSE),
                               (LONG)g_usb_info.rec_found);
            return (g_usb_info.rec_found ? TRUE : FALSE);

        case AHIDB_Stereo:
            return TRUE;    /* We always work in stereo */

        case AHIDB_MultiChannel:
            /* Return TRUE only for the multichannel AudioID (0x00550002)
             * AND only if the device actually has 4+ physical channels.
             * For all other cases (stereo device, stereo AudioID) → FALSE.
             * Note: AllocAudio already rejects AudioID 0x00550002 when
             * the device has < 4 channels, so this branch only executes
             * if the hardware truly supports multichannel. */
            if (tagList != NULL)
            {
                uint32 aid = IUtility->GetTagData(AHIA_AudioID, 0, tagList);
                if (aid == 0x00550002)
                {
                    int32 max_ch = (int32)g_usb_info.nr_channels;
                    if (g_usb_info.num_outputs > 0)
                        max_ch = (int32)g_usb_info.outputs[0].nr_channels;
                    return (max_ch >= 4) ? TRUE : FALSE;
                }
            }
            return FALSE;

        case AHIDB_MaxChannels:
            /* Number of simultaneous AHI software-mixing voices.
             * This driver does software mixing via AHI's own mixer
             * and just sends the pre-mixed stream over USB, so there
             * is no hardware limit on the number of voices.
             * Return the AHI default (DefValue = 128). */
            return DefValue;

        case AHIDB_Realtime:
            return TRUE;        /* This is a real-time audio output */

        case AHIDB_MaxRecordSamples:
            return USB_AUDIO_REC_SAMPLES;

        /* ---- Volume ranges ---- */
        case AHIDB_MinMonitorVolume:
            return 0x00000;     /* Silence */

        case AHIDB_MaxMonitorVolume:
            return 0x10000;     /* 0 dB (unity) */

        case AHIDB_MinOutputVolume:
            return 0x00000;     /* Silence */

        case AHIDB_MaxOutputVolume:
            return 0x10000;     /* 0 dB (unity) */

        case AHIDB_MinInputGain:
            /* If no input FU, min=max=unity (slider grayed out) */
            return (g_usb_info.fu_rec_found ? 0x00000 : 0x10000);

        case AHIDB_MaxInputGain:
            /* If input FU found, allow full range 0..unity.
             * Actual USB gain range is queried in discover. */
            return 0x10000;     /* 0 dB (unity) */

        /* ---- Inputs / Outputs ---- */
        case AHIDB_Inputs:
            DPRINTF("[USBAudio] GetAttr: AHIDB_Inputs => %ld\n",
                               (LONG)((g_num_flat_inputs > 0) ? g_num_flat_inputs : 0));
            return (g_num_flat_inputs > 0) ? g_num_flat_inputs : 0;

        case AHIDB_Input:
            if (Argument >= 0 && Argument < g_num_flat_inputs)
            {
                DPRINTF("[USBAudio] GetAttr: AHIDB_Input[%ld] => \"%s\"\n",
                                   Argument, g_usb_flat_inputs[Argument].name);
                return (LONG)g_usb_flat_inputs[Argument].name;
            }
            return (LONG)"Line In";

        case AHIDB_Outputs:
            return (g_num_flat_outputs > 0) ? g_num_flat_outputs : 1;

        case AHIDB_Output:
            if (Argument >= 0 && Argument < g_num_flat_outputs)
                return (LONG)g_usb_flat_outputs[Argument].name;
            return (LONG)"USB Audio";

        /* ---- Playback buffer sizing ----
         * USB audio has high latency due to USB frame scheduling,
         * bus contention with other USB devices (mouse, keyboard),
         * and the overhead of bulk transfers through the Sirion stack.
         * Request large mix buffers so the driver has more data per
         * mix cycle, providing a deeper hardware runway.
         *
         * MaxPlaySamples: tells AHI the maximum buffer size (in
         * sample frames) this driver can handle.  AHI uses this
         * together with the user's "buffer length" slider in AHI
         * Prefs to determine ahiac_BuffSamples.
         * 16384 frames @ 48kHz = ~341ms per buffer.
         */
        case AHIDB_MaxPlaySamples:
            return 16384;

        default:
            return DefValue;
    }
}


/*****************************************************************************
 *
 * set_usb_volume
 *
 * Converts an AHI Fixed (16.16) linear volume to USB Audio Class
 * volume (1/256 dB, int16) and sends SET_CUR to the Feature Unit.
 *
 * AHI volume: 0x00000 = silence, 0x10000 = 0dB (unity).
 * USB volume: 0x8000 = silence, 0x0000 = 0dB, values in 1/256 dB.
 *
 *****************************************************************************/

static void set_usb_volume(struct USBAudioData *ua, Fixed ahi_vol)
{
    int16 usb_vol;
    uint8 vol_data[2];
    int32 r;

    if (ua->ua_DevHandle == NULL || ua->ua_FUnitID == 0)
        return;

    if (ahi_vol <= 0)
    {
        usb_vol = (int16)0x8000;  /* Silence */
    }
    else if (ahi_vol >= 0x10000)
    {
        usb_vol = ua->ua_VolMax;  /* Maximum (typically 0 dB) */
    }
    else
    {
        /* Linear interpolation between VolMin and VolMax.
         * ahi_vol is 0..0x10000, map to VolMin..VolMax range.
         * Use int64 for the product: range can be up to ~65535 and ahi_vol
         * up to 0xFFFF, so range*ahi_vol overflows a signed int32. */
        int32 range = (int32)ua->ua_VolMax - (int32)ua->ua_VolMin;
        usb_vol = (int16)((int32)ua->ua_VolMin +
                          (int32)(((int64)range * (int64)ahi_vol) / 0x10000));
    }

    /* Clamp to device range */
    if (usb_vol < ua->ua_VolMin) usb_vol = ua->ua_VolMin;
    if (usb_vol > ua->ua_VolMax) usb_vol = ua->ua_VolMax;

    vol_data[0] = (uint8)(usb_vol & 0xFF);
    vol_data[1] = (uint8)((usb_vol >> 8) & 0xFF);

    DPRINTF("[USBAudio] set_usb_volume: AHI=0x%08lx -> USB=%ld (1/256 dB)\n",
                       (ULONG)ahi_vol, (LONG)usb_vol);

    /* SET_CUR on master channel (channel 0) */
    r = ILibusb1->libusb_control_transfer(ua->ua_DevHandle,
        USB_AUDIO_REQ_SET_IF, USB_AUDIO_SET_CUR,
        (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
        (uint16)((ua->ua_FUnitID << 8) | ua->ua_ACIfaceNum),
        vol_data, 2, 5000);

    if (r < 0)
        DPRINTF("[USBAudio] set_usb_volume: SET_CUR failed (%ld)\n", r);
}


/*****************************************************************************
 *
 * set_usb_input_gain
 *
 * Converts an AHI Fixed (16.16) input gain to USB Audio Class
 * volume (1/256 dB, int16) and sends SET_CUR to the recording
 * Feature Unit.
 *
 *****************************************************************************/

static void set_usb_input_gain(struct USBAudioData *ua, Fixed ahi_gain)
{
    int16 usb_vol;
    uint8 vol_data[2];
    int32 r;

    if (ua->ua_DevHandle == NULL || ua->ua_RecFUnitID == 0)
        return;

    if (ahi_gain <= 0)
    {
        usb_vol = (int16)0x8000;  /* Silence */
    }
    else if (ahi_gain >= 0x10000)
    {
        usb_vol = ua->ua_RecVolMax;
    }
    else
    {
        /* int64 product to avoid signed int32 overflow (see set_usb_volume) */
        int32 range = (int32)ua->ua_RecVolMax - (int32)ua->ua_RecVolMin;
        usb_vol = (int16)((int32)ua->ua_RecVolMin +
                          (int32)(((int64)range * (int64)ahi_gain) / 0x10000));
    }

    /* Clamp to device range */
    if (usb_vol < ua->ua_RecVolMin) usb_vol = ua->ua_RecVolMin;
    if (usb_vol > ua->ua_RecVolMax) usb_vol = ua->ua_RecVolMax;

    vol_data[0] = (uint8)(usb_vol & 0xFF);
    vol_data[1] = (uint8)((usb_vol >> 8) & 0xFF);

    DPRINTF("[USBAudio] set_usb_input_gain: AHI=0x%08lx -> USB=%ld (1/256 dB)\n",
                       (ULONG)ahi_gain, (LONG)usb_vol);

    /* SET_CUR on master channel (channel 0) of recording FU */
    r = ILibusb1->libusb_control_transfer(ua->ua_DevHandle,
        USB_AUDIO_REQ_SET_IF, USB_AUDIO_SET_CUR,
        (USB_AUDIO_FU_VOLUME_CONTROL << 8) | 0x00,
        (uint16)((ua->ua_RecFUnitID << 8) | ua->ua_ACIfaceNum),
        vol_data, 2, 5000);

    if (r < 0)
        DPRINTF("[USBAudio] set_usb_input_gain: SET_CUR failed (%ld)\n", r);
}


/*****************************************************************************
 *
 * set_usb_selector
 *
 * Sends a SET_CUR request to the Selector Unit to switch input source.
 * pin is 1-based (pin 1 = first source, pin 2 = second, etc.)
 *
 *****************************************************************************/

static void set_usb_selector(struct USBAudioData *ua, uint8 pin)
{
    uint8 pin_data[1];
    int32 r;

    if (ua->ua_DevHandle == NULL || ua->ua_SelectorUnitID == 0 || pin == 0)
        return;

    pin_data[0] = pin;

    DPRINTF("[USBAudio] set_usb_selector: SU ID=%ld pin=%ld\n",
                       (LONG)ua->ua_SelectorUnitID, (LONG)pin);

    r = ILibusb1->libusb_control_transfer(ua->ua_DevHandle,
        USB_AUDIO_REQ_SET_IF, USB_AUDIO_SET_CUR,
        USB_AUDIO_SU_SELECTOR_CONTROL,  /* Selector Unit CS=0x01: wValue=0x0100 */
        (uint16)((ua->ua_SelectorUnitID << 8) | ua->ua_ACIfaceNum),
        pin_data, 1, 5000);

    if (r < 0)
        DPRINTF("[USBAudio] set_usb_selector: SET_CUR failed (%ld)\n", r);
    else
        DPRINTF("[USBAudio] set_usb_selector: OK\n");
}


/*****************************************************************************
 *
 * AHIsub_HardwareControl
 *
 * Handles volume, monitor, gain, input/output selection from AHI.
 *
 *****************************************************************************/

int32 _usbaudio_AHIsub_HardwareControl(struct USBAudioIFace    *Self,
                                        uint32                   Attribute,
                                        int32                    Argument,
                                        struct AHIAudioCtrlDrv  *AudioCtrl)
{
    /* Every case dereferences dd = *ahiac_DriverData; guard against AHI
     * calling us with no allocated driver data. */
    if (AudioCtrl == NULL || AudioCtrl->ahiac_DriverData == NULL)
        return FALSE;

    switch (Attribute)
    {
        case AHIC_MonitorVolume:
            dd->ua_MonitorVolume = (Fixed)Argument;
            return TRUE;

        case AHIC_MonitorVolume_Query:
            return (int32)dd->ua_MonitorVolume;

        case AHIC_OutputVolume:
            dd->ua_OutputVolume = (Fixed)Argument;
            /* Send volume to USB device if Feature Unit is available */
            set_usb_volume(dd, dd->ua_OutputVolume);
            return TRUE;

        case AHIC_OutputVolume_Query:
            return (int32)dd->ua_OutputVolume;

        case AHIC_InputGain:
            dd->ua_InputGain = (Fixed)Argument;
            /* Send gain to USB device if recording FU is available */
            set_usb_input_gain(dd, dd->ua_InputGain);
            return TRUE;

        case AHIC_InputGain_Query:
            return (int32)dd->ua_InputGain;

        case AHIC_Input:
        {
            uint32 idx = (uint32)Argument;
            dd->ua_Input = idx;

            /* Switch input source from flat input array */
            if ((int32)idx >= 0 && (int32)idx < g_num_flat_inputs)
            {
                int32 di = g_usb_flat_inputs[idx].device_idx;
                int32 si = g_usb_flat_inputs[idx].source_idx;

                /* If this input belongs to a different device, switch the active recording device */
                if (di >= 0 && di < g_usb_num_devices)
                {
                    struct USBAudioDeviceInfo *dev = &g_usb_devices[di];

                    /* Update recording fields from the device */
                    if (dev->rec_found)
                    {
                        dd->ua_RecInterfaceNum  = dev->rec_iface_num;
                        dd->ua_RecAltSetting    = dev->rec_alt_setting;
                        dd->ua_RecEndpointAddr  = dev->rec_ep_addr;
                        dd->ua_RecNumChannels   = dev->rec_nr_channels;
                        dd->ua_RecMaxPacketSize = dev->rec_max_pkt;
                        dd->ua_RecSubframeSize  = dev->rec_subframe_size;
                        dd->ua_RecBitResolution = dev->rec_bit_resolution;
                    }

                    /* Update input FU from the selected source */
                    if (si >= 0 && si < dev->num_inputs && dev->inputs[si].fu_found)
                    {
                        dd->ua_RecFUnitID = dev->inputs[si].fu_unit_id;
                        dd->ua_ACIfaceNum = dev->ac_iface_num;
                    }

                    /* Update Selector Unit pin and switch source on USB device */
                    dd->ua_SelectorUnitID = dev->selector_unit_id;
                    if (si >= 0 && si < dev->num_inputs && dev->inputs[si].su_pin != 0)
                    {
                        dd->ua_SelectorPin = dev->inputs[si].su_pin;
                        set_usb_selector(dd, dd->ua_SelectorPin);
                    }

                    DPRINTF("[USBAudio] AHIC_Input: switched to \"%s\" (FU=%ld SU pin=%ld)\n",
                                       g_usb_flat_inputs[idx].name,
                                       (LONG)dd->ua_RecFUnitID,
                                       (LONG)dd->ua_SelectorPin);
                }
            }
            return TRUE;
        }

        case AHIC_Input_Query:
            return (int32)dd->ua_Input;

        case AHIC_Output:
        {
            uint32 idx = (uint32)Argument;
            dd->ua_Output = idx;

            /* Switch active device + output mode from flat output array */
            if ((int32)idx >= 0 && (int32)idx < g_num_flat_outputs)
            {
                int32 di = g_usb_flat_outputs[idx].device_idx;
                int32 mi = g_usb_flat_outputs[idx].mode_idx;

                if (di >= 0 && di < g_usb_num_devices)
                {
                    struct USBAudioDeviceInfo *dev = &g_usb_devices[di];

                    /* Set the selected output mode for this device */
                    if (mi >= 0 && mi < dev->num_outputs)
                        dev->selected_output = mi;

                    /* Copy full device info to active global */
                    g_usb_info = *dev;

                    /* Apply selected output mode's parameters */
                    if (mi >= 0 && mi < dev->num_outputs)
                    {
                        struct USBOutputMode *out = &dev->outputs[mi];
                        g_usb_info.iface_num      = out->iface_num;
                        g_usb_info.alt_setting    = out->alt_setting;
                        g_usb_info.ep_addr        = out->ep_addr;
                        g_usb_info.max_pkt        = out->max_pkt;
                        g_usb_info.nr_channels    = out->nr_channels;
                        g_usb_info.subframe_size  = out->subframe_size;
                        g_usb_info.bit_resolution = out->bit_resolution;
                    }

                    /* Update dd fields for next Start() */
                    dd->ua_VendorID      = g_usb_info.vid;
                    dd->ua_ProductID     = g_usb_info.pid;
                    dd->ua_InterfaceNum  = g_usb_info.iface_num;
                    dd->ua_AltSetting    = g_usb_info.alt_setting;
                    dd->ua_EndpointAddr  = g_usb_info.ep_addr;
                    dd->ua_MaxPacketSize = g_usb_info.max_pkt;
                    dd->ua_NumChannels   = g_usb_info.nr_channels;
                    dd->ua_SubframeSize  = g_usb_info.subframe_size;
                    dd->ua_BitResolution = g_usb_info.bit_resolution;

                    /* Channel pair offset for multichannel routing */
                    if (mi >= 0 && mi < dev->num_outputs)
                        dd->ua_ChannelOffset = dev->outputs[mi].channel_offset;
                    else
                        dd->ua_ChannelOffset = 0;

                    if (g_usb_info.rec_found)
                    {
                        dd->ua_RecInterfaceNum  = g_usb_info.rec_iface_num;
                        dd->ua_RecAltSetting    = g_usb_info.rec_alt_setting;
                        dd->ua_RecEndpointAddr  = g_usb_info.rec_ep_addr;
                        dd->ua_RecNumChannels   = g_usb_info.rec_nr_channels;
                        dd->ua_RecMaxPacketSize = g_usb_info.rec_max_pkt;
                        dd->ua_RecSubframeSize  = g_usb_info.rec_subframe_size;
                        dd->ua_RecBitResolution = g_usb_info.rec_bit_resolution;
                    }

                    if (g_usb_info.fu_found)
                    {
                        dd->ua_ACIfaceNum = g_usb_info.ac_iface_num;
                        dd->ua_FUnitID    = g_usb_info.fu_unit_id;
                    }

                    if (g_usb_info.fu_rec_found)
                        dd->ua_RecFUnitID = g_usb_info.fu_rec_unit_id;

                    DPRINTF("[USBAudio] AHIC_Output: switched to \"%s\" (dev=%ld mode=%ld)\n",
                                       g_usb_flat_outputs[idx].name, (LONG)di, (LONG)mi);
                }
            }
            return TRUE;
        }

        case AHIC_Output_Query:
            return (int32)dd->ua_Output;

        default:
            return FALSE;
    }
}
