/*
 * AHIdrv-hwUSBPlayProc.c
 *
 * USB Audio playback slave process for the AHI sub-driver.
 *
 * This process runs as a separate AmigaOS task. It:
 *   1. Receives mixed audio from AHI via MixerFunc
 *   2. Converts the mix buffer to USB audio format (16-bit LE)
 *   3. Sends data to the USB audio device via asynchronous isochronous
 *      transfers directly to usbsys.device, with multiple USBIOReqs
 *      pre-queued to eliminate stuttering.
 *
 * The async approach (modelled after usbaudio2 by Chris Handley) uses
 * SendIO to pre-queue N isochronous IORequests with the Sirion USB stack.
 * When one completes, it is immediately refilled and re-sent, keeping a
 * continuous pipeline of audio data flowing to the USB device.
 *
 * Refilling a completed request is a plain memcpy out of a double-buffered
 * staging area; the AHI software mixer runs only *after* the request has been
 * re-sent, never between completion and re-send.  See the StagingCtx comment
 * below for why that ordering matters under load.
 */

#include <exec/exec.h>
#include <proto/exec.h>
#include <dos/dos.h>
#include <proto/dos.h>
#include <utility/utility.h>
#include <proto/utility.h>
#include <exec/types.h>
#include <libraries/ahi_sub.h>
#include <proto/usbaudio.h>
#include <interfaces/usbsys.h>
#include <usb/system.h>
#include <usb/usb.h>
#include <stdarg.h>
#include <string.h>
#include "includes/AHIdrv-USBaudio.h"

/*
 * Isochronous pipeline geometry.
 *
 * USBA_HCD_CachedIsochronousFrames is a count of USB *frames*, not of
 * IORequests: it is how far ahead, in 1ms frames, the HCD needs transfers
 * queued in order to keep its cache fed (see usbsys.doc, USBGetEndPointAttrs).
 *
 * The earlier code used it directly as the number of IORequests and then
 * packed FRAMES_PER_IOR frames into each one, so the number of frames
 * actually placed in the endpoint's schedule was cachedFrames *
 * FRAMES_PER_IOR — up to an order of magnitude more than intended.  That is
 * what provokes the HCD's "[EHCI] EndPoint iso schedule full (by node)": the
 * per-endpoint schedule node cannot hold that many frames, and the condition
 * shows up in bursts, because after a scheduling stall (dragging a window)
 * every in-flight request completes at once and is resubmitted back-to-back.
 *
 * So the budget is expressed in frames and then divided into requests:
 *
 *   total frames in flight = cachedFrames * SCHEDULE_DEPTH_FACTOR
 *                            (clamped to [MIN_INFLIGHT_FRAMES,
 *                                         MAX_INFLIGHT_FRAMES])
 *   iorCount               = total frames / FRAMES_PER_IOR   (at least 2)
 *
 * FRAMES_PER_IOR sets the refill deadline (one completed request must be
 * refilled within FRAMES_PER_IOR milliseconds); the total frame count sets how
 * much audio is buffered against a stall.  All four knobs are overridable from
 * the Makefile so the geometry can be bisected on real hardware.
 */

/* USB frames packed into each isochronous IORequest = refill deadline in ms.
 * NOTE: this also sets how *often* requests are submitted, which matters more
 * than the buffering does — see the submission-rejection comment in the main
 * loop.  Lowering it makes the driver submit more often, which is harmful
 * while submissions can be rejected. */
#ifndef FRAMES_PER_IOR
#define FRAMES_PER_IOR 8
#endif

/* How many times the HCD's stated lookahead requirement to keep queued.
 * 1 is the documented minimum; more buys resilience at the cost of schedule
 * occupancy and output latency. */
#ifndef SCHEDULE_DEPTH_FACTOR
#define SCHEDULE_DEPTH_FACTOR 2
#endif

/* Hard bounds on total frames in flight, in ms of audio. */
#ifndef MIN_INFLIGHT_FRAMES
#define MIN_INFLIGHT_FRAMES 16
#endif
#ifndef MAX_INFLIGHT_FRAMES
#define MAX_INFLIGHT_FRAMES 32
#endif

#define dd ((struct USBAudioData *)AudioCtrl->ahiac_DriverData)

/*
 * Byte-swap a 16-bit value (BE ↔ LE)
 */
static inline uint16 swap16(uint16 x)
{
    return (uint16)((x >> 8) | (x << 8));
}

/*
 * mix_to_usb
 *
 * Performs one complete mix cycle:
 *   1. Calls PlayerFunc (advances AHI playback position)
 *   2. Calls MixerFunc  (fills ua_MixBuffer with mixed audio)
 *   3. Converts mix buffer to USB format (16-bit LE) into outbuf
 *   4. Handles mono-to-stereo duplication if needed
 *
 * Returns the number of bytes written to outbuf.
 */
static uint32 mix_to_usb(struct AHIAudioCtrlDrv *AudioCtrl, uint8 *outbuf)
{
    uint32 samples, i;

    /* Advance AHI playback position */
    IUtility->CallHookPkt(AudioCtrl->ahiac_PlayerFunc, (APTR)AudioCtrl, NULL);

    /* Mix audio into ua_MixBuffer */
    IUtility->CallHookPkt(AudioCtrl->ahiac_MixerFunc,
                           (APTR)AudioCtrl, dd->ua_MixBuffer);

    /* Compute output sample count.
     * In multichannel mode, AHI provides 4 stereo pairs (8 channels)
     * interleaved: FL,FR,RL,RR,SL,SR,C,LFE.
     * In stereo mode, it provides L,R pairs.
     * In mono mode, just single samples. */
    samples = AudioCtrl->ahiac_BuffSamples;
    if (AudioCtrl->ahiac_Flags & AHIACF_MULTICHANNEL)
        samples *= dd->ua_NumChannels;    /* 8 channels per sample frame */
    else if (AudioCtrl->ahiac_Flags & AHIACF_STEREO)
        samples <<= 1;

    /* Convert AHI mix buffer (big-endian) to USB output (16-bit signed LE) */
    if (AudioCtrl->ahiac_Flags & AHIACF_HIFI)
    {
        /* 32-bit BE -> 16-bit LE */
        int16 *dst = (int16 *)outbuf;
        int32 *src = (int32 *)dd->ua_MixBuffer;
        for (i = 0; i < samples; i++)
        {
            int16 s = (int16)(src[i] >> 16);
            dst[i] = (int16)swap16((uint16)s);
        }
    }
    else
    {
        /* 16-bit BE -> 16-bit LE (byte-swap) */
        int16 *dst = (int16 *)outbuf;
        int16 *src = (int16 *)dd->ua_MixBuffer;
        for (i = 0; i < samples; i++)
        {
            dst[i] = (int16)swap16((uint16)src[i]);
        }
    }

    /* Mono -> stereo duplication (only in non-multichannel mode) */
    if (!(AudioCtrl->ahiac_Flags & (AHIACF_STEREO | AHIACF_MULTICHANNEL)) &&
        dd->ua_NumChannels >= 2)
    {
        int16 *buf = (int16 *)outbuf;
        for (i = samples; i > 0; i--)
        {
            buf[(i - 1) * 2 + 1] = buf[i - 1];
            buf[(i - 1) * 2]     = buf[i - 1];
        }
        samples *= 2;
    }

    /* Stereo → multichannel routing (non-multichannel mode only):
     * place the stereo pair at the correct channel offset and
     * zero-fill all other channels.
     * E.g. for 8ch with offset=2 (Center/LFE):
     *   L,R → 0,0,L,R,0,0,0,0 per sample frame. */
    if (!(AudioCtrl->ahiac_Flags & AHIACF_MULTICHANNEL) &&
        dd->ua_NumChannels > 2)
    {
        int16  *buf     = (int16 *)outbuf;
        uint32  n_pairs = samples / 2;
        uint32  nch     = (uint32)dd->ua_NumChannels;
        uint32  off     = (uint32)dd->ua_ChannelOffset;
        uint32  p, c;

        for (p = n_pairs; p > 0; p--)
        {
            int16  L    = buf[(p - 1) * 2];
            int16  R    = buf[(p - 1) * 2 + 1];
            uint32 base = (p - 1) * nch;

            for (c = 0; c < nch; c++)
                buf[base + c] = 0;

            buf[base + off]     = L;
            buf[base + off + 1] = R;
        }
        samples = n_pairs * nch;
    }

    /* In multichannel mode, AHI provides data in order:
     * FL,FR, RL,RR, SL,SR, C,LFE  (4 stereo pairs)
     * USB Audio Class 1.0 expects:
     * FL,FR, C,LFE, BL,BR, SL,SR
     * Remap each sample frame in-place. */
    if ((AudioCtrl->ahiac_Flags & AHIACF_MULTICHANNEL) &&
        dd->ua_NumChannels >= 8)
    {
        int16 *buf = (int16 *)outbuf;
        uint32 nch = (uint32)dd->ua_NumChannels;
        uint32 n_frames = samples / nch;
        uint32 f;

        for (f = 0; f < n_frames; f++)
        {
            uint32 base = f * nch;
            /* AHI order: [0]=FL [1]=FR [2]=RL [3]=RR [4]=SL [5]=SR [6]=C [7]=LFE */
            /* USB order: [0]=FL [1]=FR [2]=C  [3]=LFE [4]=BL [5]=BR [6]=SL [7]=SR */
            int16 FL  = buf[base + 0];
            int16 FR  = buf[base + 1];
            int16 RL  = buf[base + 2];
            int16 RR  = buf[base + 3];
            int16 SL  = buf[base + 4];
            int16 SR  = buf[base + 5];
            int16 C   = buf[base + 6];
            int16 LFE = buf[base + 7];

            buf[base + 0] = FL;
            buf[base + 1] = FR;
            buf[base + 2] = C;
            buf[base + 3] = LFE;
            buf[base + 4] = RL;
            buf[base + 5] = RR;
            buf[base + 6] = SL;
            buf[base + 7] = SR;
        }
    }

    return samples * (uint32)dd->ua_SubframeSize;
}

/*
 * Double-buffered staging.
 *
 * The AHI software mixer (PlayerFunc + MixerFunc, called by mix_to_usb) is by
 * far the most expensive thing this process does, and it runs for a whole
 * ahiac_BuffSamples worth of audio in one uninterruptible burst.  The earlier
 * single-buffer design called it lazily, from inside the IORequest completion
 * handler, at the exact moment the staging buffer ran dry — i.e. with only one
 * IORequest (a few ms) of audio still queued in the USB pipeline.  Under load
 * (dragging windows, heavy CPU) that burst could exceed the remaining runway
 * and starve the pipe, which is what produced the intermittent clicks.
 *
 * With two buffers, one is consumed while the other already holds the next
 * mix.  staging_read() therefore never calls the mixer in normal operation:
 * when the current buffer drains it simply swaps to the spare.  The refill is
 * done by staging_top_up(), called from the main loop *after* the completed
 * IORequest has been re-sent — so the mix runs at the point of maximum
 * remaining runway rather than the minimum.
 */
struct StagingCtx
{
    uint8  *buf[2];       /* the two staging buffers */
    uint32  size[2];      /* valid bytes in each */
    uint32  cur;          /* index currently being consumed */
    uint32  off;          /* read offset into buf[cur] */
    BOOL    spare_ready;  /* buf[cur ^ 1] holds a fresh, unconsumed mix */
};

/*
 * staging_top_up
 *
 * Refills the spare buffer if it is not already loaded.  No-op otherwise, so
 * it is safe (and intended) to call this after every send.
 */
static void staging_top_up(struct AHIAudioCtrlDrv *AudioCtrl,
                           struct StagingCtx *sc)
{
    uint32 spare;

    if (sc->spare_ready)
        return;

    spare = sc->cur ^ 1;
    sc->size[spare] = mix_to_usb(AudioCtrl, sc->buf[spare]);
    sc->spare_ready = TRUE;
}

/*
 * staging_read
 *
 * Copies exactly `need` bytes into `dst`, swapping buffers as they drain.
 *
 * The loop handles any ratio between per-IOReq size and mix-buffer size: if a
 * single IORequest needs more than one whole staging buffer it will swap more
 * than once.  That is also the only case in which the mixer is called from
 * here (via the staging_top_up fallback) — mixing inline is still better than
 * emitting silence.
 */
static void staging_read(struct AHIAudioCtrlDrv *AudioCtrl,
                         struct StagingCtx *sc, uint8 *dst, uint32 need)
{
    uint32 done = 0;

    while (done < need)
    {
        uint32 remaining = sc->size[sc->cur] - sc->off;
        uint32 chunk;

        if (remaining == 0)
        {
            /* Current buffer drained — switch to the spare.  If the top-up
             * did not happen in time, mix now rather than emit silence. */
            staging_top_up(AudioCtrl, sc);

            sc->cur         = sc->cur ^ 1;
            sc->off         = 0;
            sc->spare_ready = FALSE;

            /* Guard against a mixer that returns 0 bytes (avoid an infinite
             * loop): zero-fill the rest and bail. */
            if (sc->size[sc->cur] == 0)
            {
                memset(dst + done, 0, need - done);
                return;
            }
            continue;
        }

        chunk = need - done;
        if (chunk > remaining)
            chunk = remaining;

        memcpy(dst + done, sc->buf[sc->cur] + sc->off, chunk);
        done    += chunk;
        sc->off += chunk;
    }
}

/*
 * setup_ior_sizes
 *
 * Programs the per-sub-frame transfer setups of one IORequest and returns the
 * total byte count.  The fractional accumulator keeps the long-term average
 * sample rate exact (e.g. at 44100 Hz with 1ms frames, 44 samples per frame
 * with a 45th every tenth frame).
 *
 * Shared by the initial launch and the refill path so the two cannot drift.
 */
static uint32 setup_ior_sizes(struct USBSysIFace *IUSBSys, struct USBIOReq *ureq,
                              uint32 subXfers, uint32 baseSamples,
                              uint32 fracNum, uint32 fracDen,
                              uint32 *accumulator, uint32 frameSize,
                              uint32 maxTransferSize)
{
    uint32 y;
    uint32 bytesThisFrame = 0;

    for (y = 0; y < subXfers; y++)
    {
        uint32 n = baseSamples;
        uint32 subLen;

        *accumulator += fracNum;
        if (*accumulator >= fracDen)
        {
            *accumulator -= fracDen;
            n++;
        }

        subLen = n * frameSize;
        if (subLen > maxTransferSize)
            subLen = maxTransferSize;


        IUSBSys->USBSetIsoTransferSetup(ureq, y, bytesThisFrame, subLen);
        bytesThisFrame += subLen;
    }

    return bytesThisFrame;
}

/*
 * hwUSBPlaySlave
 *
 * Entry point for the playback process created in AHIsub_Start.
 * AudioCtrl is passed via tc_UserData.
 *
 * Uses direct async isochronous IO to usbsys.device with multiple
 * pre-queued USBIOReqs for stutter-free playback.
 */
uint32 hwUSBPlaySlave(STRPTR *args UNUSED, int32 arglen UNUSED,
                      struct ExecBase *sysbase)
{
    struct AHIAudioCtrlDrv *AudioCtrl;
    uint32  signals;
    uint32 loop_count = 0;
    uint32 error_count = 0;
    uint32 consecutive_fatal = 0;  /* consecutive fatal USB errors (device removed) */
    uint32 framemissed_count = 0;  /* USBERR_FRAMEMISSED: audio never reached the bus */
    uint32 checkresults_count = 0; /* USBERR_CHECKRESULTS: partial sub-transfer failure */
    uint32 rejected_count = 0;     /* submissions the HCD refused to schedule */
    uint32 bytes_sent = 0;         /* bytes the hardware reports as transferred */
    uint32 last_report = 0;        /* loop_count at last status line */
    uint32 backoff_count = 0;      /* times the circuit breaker had to sleep */

    /* Local USB resources — opened and freed entirely within this process */
    struct MsgPort    *usbPort    = NULL;
    struct IORequest  *baseReq    = NULL;
    struct USBSysIFace *IUSBSys   = NULL;
    struct USBIOReq  **iorTable   = NULL;
    uint32             iorCount   = 0;
    uint32            *iorSendFrame = NULL; /* USB frame number at submission */
    uint32            *iorLength  = NULL;   /* io_Length as submitted, for retries */
    uint8             *iorParked  = NULL;   /* 1 = rejected, awaiting retry */
    BOOL               haveFrameNo = FALSE; /* HCD maintains a frame number */
    uint32             inflight   = 0;      /* requests currently submitted */
    struct UsbEndPoint *ep        = NULL;
    BOOL               deviceOpen = FALSE;

    /* Double-buffered staging area for AHI mix output */
    struct StagingCtx stage = { { NULL, NULL }, { 0, 0 }, 0, 0, FALSE };

    AudioCtrl = (struct AHIAudioCtrlDrv *)sysbase->ThisTask->tc_UserData;

    DPRINTF("[USBAudio] PlaySlave: entry, AudioCtrl=%p\n", AudioCtrl);

    if (AudioCtrl == NULL || AudioCtrl->ahiac_DriverData == NULL)
    {
        DPRINTF("[USBAudio] PlaySlave: AudioCtrl or DriverData is NULL!\n");
        goto quit;
    }

    /* Allocate a signal for the stop request */
    if ((dd->ua_SlaveSignal = IExec->AllocSignal(-1)) == -1)
    {
        DPRINTF("[USBAudio] PlaySlave: AllocSignal failed!\n");
        goto quit;
    }

    DPRINTF("[USBAudio] PlaySlave: EP=0x%02lx MaxPkt=%lu Channels=%lu SubSz=%lu\n",
                       (ULONG)dd->ua_EndpointAddr, (ULONG)dd->ua_MaxPacketSize,
                       (ULONG)dd->ua_NumChannels, (ULONG)dd->ua_SubframeSize);

    /* ------------------------------------------------------------------
     * Step 1: Open usbsys.device and get IUSBSys interface.
     * We need our own MsgPort for async IO completions.
     * ------------------------------------------------------------------ */
    usbPort = IExec->AllocSysObjectTags(ASOT_PORT, TAG_END);
    if (!usbPort)
    {
        DPRINTF("[USBAudio] PlaySlave: AllocSysObject(PORT) failed\n");
        goto quit;
    }

    baseReq = IExec->AllocSysObjectTags(ASOT_IOREQUEST,
                ASOIOR_Size,      sizeof(struct USBIOReq),
                ASOIOR_ReplyPort, (uint32)usbPort,
                TAG_END);
    if (!baseReq)
    {
        DPRINTF("[USBAudio] PlaySlave: AllocSysObject(IOREQUEST) failed\n");
        goto quit;
    }

    if (IExec->OpenDevice("usbsys.device", 0, baseReq, 0))
    {
        DPRINTF("[USBAudio] PlaySlave: OpenDevice(usbsys.device) failed\n");
        goto quit;
    }
    deviceOpen = TRUE;

    IUSBSys = (struct USBSysIFace *)IExec->GetInterface(
                  (struct Library *)baseReq->io_Device, "main", 1, NULL);
    if (!IUSBSys)
    {
        DPRINTF("[USBAudio] PlaySlave: GetInterface(USBSys) failed\n");
        goto quit;
    }

    /* ------------------------------------------------------------------
     * Step 2: Get the UsbEndPoint from libusb's internal data.
     *
     * libusb_device_handle->data->lad_EndPoints[] stores cached
     * UsbEndPoint pointers.  OUT endpoints 0x00-0x0F are at indices
     * 0..15, IN endpoints 0x80-0x8F at 16..31.
     * ------------------------------------------------------------------ */
    {
        uint8  ep_addr = dd->ua_EndpointAddr;
        uint32 ep_idx  = (ep_addr & 0x80) ? ((ep_addr & 0x0F) + 16)
                                           : (ep_addr & 0x0F);

        if (dd->ua_DevHandle && dd->ua_DevHandle->data)
        {
            ep = dd->ua_DevHandle->data->lad_EndPoints[ep_idx];
            if (!ep)
            {
                /* Endpoint not cached yet — look it up via the claimed interface */
                ep = IUSBSys->USBGetEndPoint(NULL,
                         dd->ua_DevHandle->data->lad_Interface,
                         ep_addr);
                if (ep)
                    dd->ua_DevHandle->data->lad_EndPoints[ep_idx] = ep;
            }
        }
    }
    if (!ep)
    {
        DPRINTF("[USBAudio] PlaySlave: could not get UsbEndPoint for EP 0x%02lx\n",
                           (ULONG)dd->ua_EndpointAddr);
        DPRINTF("[USBAudio] PlaySlave: FAILED to resolve UsbEndPoint 0x%02lx "
                "(lad_Interface=%p) - no audio will play\n",
                (ULONG)dd->ua_EndpointAddr,
                dd->ua_DevHandle ? dd->ua_DevHandle->data->lad_Interface : NULL);
        goto quit;
    }

    DPRINTF("[USBAudio] PlaySlave: got UsbEndPoint %p for EP 0x%02lx\n",
                       ep, (ULONG)dd->ua_EndpointAddr);

    /* ------------------------------------------------------------------
     * Step 3: Query isochronous parameters from the endpoint.
     * ------------------------------------------------------------------ */
    {
        uint32 cachedFrames    = 0;
        uint32 maxTransferSize = 0;
        uint32 transfersPerFrame = 0;
        uint32 wantIors;         /* IORequests the schedule budget allows */
        uint32 maxIsoChunkSize;  /* max bytes per IOReq (for allocation) */
        uint32 frameSize;        /* bytes per sample frame: channels * subframeSize */
        uint32 baseSamples;      /* floor(sampleRate / (TPF * 1000)) per sub-frame */
        uint32 fracNum;          /* sampleRate % (TPF * 1000) — fractional part */
        uint32 fracDen;          /* TPF * 1000 — accumulator threshold */
        uint32 accumulator = 0;  /* fractional sample accumulator */
        uint32 subXfersPerIOR;   /* total sub-transfers per IORequest */
        uint32 iorBufSize;       /* DMA buffer size per IORequest */
        uint32 x;

        IUSBSys->USBGetEndPointAttrs(ep,
            USBA_HCD_CachedIsochronousFrames, &cachedFrames,
            USBA_EP_MaxTransferSize, &maxTransferSize,
            USBA_EP_TransfersPerFrame, &transfersPerFrame,
            TAG_END);

        DPRINTF("[USBAudio] PlaySlave: cachedFrames=%lu maxXferSize=%lu xfersPerFrame=%lu\n",
                           (ULONG)cachedFrames, (ULONG)maxTransferSize,
                           (ULONG)transfersPerFrame);

        DPRINTF("[USBAudio] PlaySlave: iso params cachedFrames=%lu "
                "maxXferSize=%lu xfersPerFrame=%lu speed=%lu\n",
                (ULONG)cachedFrames, (ULONG)maxTransferSize,
                (ULONG)transfersPerFrame,
                (ULONG)(dd->ua_DevHandle ? dd->ua_DevHandle->dev->speed : 0));

        if (cachedFrames < 1)
            cachedFrames = 1;

        if (transfersPerFrame == 0)
            transfersPerFrame = 1;

        /* Turn the HCD's frame lookahead requirement into a request count.
         * See the geometry comment at the top of this file: the budget is in
         * frames, and iorCount * FRAMES_PER_IOR must stay inside it or the
         * endpoint's schedule node overflows. */
        {
            uint32 inflightFrames = cachedFrames * SCHEDULE_DEPTH_FACTOR;

            if (inflightFrames < MIN_INFLIGHT_FRAMES)
                inflightFrames = MIN_INFLIGHT_FRAMES;
            if (inflightFrames > MAX_INFLIGHT_FRAMES)
                inflightFrames = MAX_INFLIGHT_FRAMES;

            /* The cap must never starve the HCD: if this controller wants
             * more lookahead than the cap allows, the HCD requirement wins,
             * otherwise the stream underruns by construction. */
            if (inflightFrames < cachedFrames)
                inflightFrames = cachedFrames;

            wantIors = inflightFrames / FRAMES_PER_IOR;
            if (wantIors < 2)
                wantIors = 2;   /* need at least double buffering */

            DPRINTF("[USBAudio] PlaySlave: cachedFrames=%lu -> budget %lu frames "
                    "-> %lu IOReqs x %lu frames (%lu ms buffered, %lu ms deadline)\n",
                    (ULONG)cachedFrames, (ULONG)inflightFrames, (ULONG)wantIors,
                    (ULONG)FRAMES_PER_IOR,
                    (ULONG)(wantIors * FRAMES_PER_IOR),
                    (ULONG)FRAMES_PER_IOR);
        }

        /*
         * For output: use a fractional accumulator to maintain the correct
         * average sample rate.  USB isochronous frames are 1ms each.
         * At 44100 Hz with TPF=1: baseSamples=44, fracNum=100, fracDen=1000.
         * Every 10th frame we send 45 samples instead of 44 to maintain
         * the 44.1 kHz average.  This prevents slow clock drift that
         * causes periodic clicks.
         *
         * Buffer allocation uses maxTransferSize (the endpoint's max
         * packet size) so there is always room for baseSamples+1.
         */
        frameSize   = (uint32)dd->ua_NumChannels * (uint32)dd->ua_SubframeSize;
        fracDen     = transfersPerFrame * 1000;
        baseSamples = AudioCtrl->ahiac_MixFreq / fracDen;
        fracNum     = AudioCtrl->ahiac_MixFreq % fracDen;

        maxIsoChunkSize = maxTransferSize * transfersPerFrame;

        subXfersPerIOR = FRAMES_PER_IOR * transfersPerFrame;
        iorBufSize     = FRAMES_PER_IOR * maxIsoChunkSize;

        DPRINTF("[USBAudio] PlaySlave: frameSize=%lu baseSamples=%lu fracNum=%lu/%lu\n",
                           (ULONG)frameSize, (ULONG)baseSamples,
                           (ULONG)fracNum, (ULONG)fracDen);

        if (maxIsoChunkSize == 0 || baseSamples == 0)
        {
            DPRINTF("[USBAudio] PlaySlave: invalid isochronous parameters!\n");
            DPRINTF("[USBAudio] PlaySlave: INVALID iso params "
                    "(chunk=%lu baseSamples=%lu) - no audio will play\n",
                    (ULONG)maxIsoChunkSize, (ULONG)baseSamples);
            goto quit;
        }

        /* ------------------------------------------------------------------
         * Step 4: Allocate IORequests.
         *
         * Buffers are sized to maxTransferSize (endpoint max packet).
         * Isochronous sub-frame count is set once; actual sub-frame
         * offsets/lengths are updated dynamically before each SendIO.
         * ------------------------------------------------------------------ */
        iorTable = (struct USBIOReq **)IExec->AllocVecTags(
                       sizeof(struct USBIOReq *) * (wantIors + 1),
                       AVT_ClearWithValue, 0,
                       TAG_END);
        if (!iorTable)
        {
            DPRINTF("[USBAudio] PlaySlave: iorTable alloc failed\n");
            goto quit;
        }

        for (x = 0; x < wantIors; x++)
        {
            iorTable[x] = IUSBSys->USBAllocRequest((struct IORequest *)baseReq, TAG_END);
            if (!iorTable[x])
            {
                DPRINTF("[USBAudio] PlaySlave: USBAllocRequest %lu failed\n", (ULONG)x);
                iorCount = x;
                goto quit;
            }

            /* Allocate DMA-safe data buffer — fits FRAMES_PER_IOR frames */
            iorTable[x]->io_Data = IExec->AllocVecTags(iorBufSize,
                                       AVT_Type, MEMF_SHARED,
                                       AVT_ClearWithValue, 0,
                                       TAG_END);
            if (!iorTable[x]->io_Data)
            {
                DPRINTF("[USBAudio] PlaySlave: IOReq buffer %lu alloc failed\n", (ULONG)x);
                IUSBSys->USBFreeRequest(iorTable[x]);
                iorTable[x] = NULL;
                iorCount = x;
                goto quit;
            }

            /* Sub-frame count is fixed; sizes are set per-send */
            IUSBSys->USBSetIsoTransferCount(iorTable[x], subXfersPerIOR);

            /* Set reply port for async completions */
            ((struct IORequest *)iorTable[x])->io_Message.mn_ReplyPort = usbPort;
        }
        iorCount = wantIors;

        DPRINTF("[USBAudio] PlaySlave: allocated %lu IORequests (%lu bytes, %lu subXfers each)\n",
                           (ULONG)iorCount, (ULONG)iorBufSize, (ULONG)subXfersPerIOR);

        /* Per-request bookkeeping for the rejected-submission detector. */
        iorSendFrame = (uint32 *)IExec->AllocVecTags(sizeof(uint32) * iorCount,
                           AVT_ClearWithValue, 0, TAG_END);
        iorLength    = (uint32 *)IExec->AllocVecTags(sizeof(uint32) * iorCount,
                           AVT_ClearWithValue, 0, TAG_END);
        iorParked    = (uint8 *)IExec->AllocVecTags(sizeof(uint8) * iorCount,
                           AVT_ClearWithValue, 0, TAG_END);
        if (!iorSendFrame || !iorLength || !iorParked)
        {
            DPRINTF("[USBAudio] PlaySlave: request state alloc failed\n");
            goto quit;
        }

        /* The rejected-submission detector compares the USB frame number at
         * submission with the one at completion.  That only works if this HCD
         * maintains a frame number; if it does not we fall back to the batch
         * circuit breaker alone. */
        {
            uint32 maintains = 0;

            if (dd->ua_DevHandle && dd->ua_DevHandle->data &&
                dd->ua_DevHandle->data->lad_RawInterface)
            {
                IUSBSys->USBGetRawInterfaceAttrs(
                    dd->ua_DevHandle->data->lad_RawInterface,
                    USBA_HCD_MaintainsFrameNumber, &maintains,
                    TAG_END);
            }
            haveFrameNo = (maintains != 0);


            DPRINTF("[USBAudio] PlaySlave: HCD maintains frame number: %s\n",
                               haveFrameNo ? "yes" : "no (using fallback only)");
        }

        /* ------------------------------------------------------------------
         * Step 5: Set up staging buffer and fill initial IORequests.
         *
         * ua_USBBuffer / ua_USBBuffer2 are the two halves of the staging
         * double buffer.  Both are primed here so the pipeline starts with a
         * full spare and the mixer is never on the critical path.
         * ------------------------------------------------------------------ */
        if (!dd->ua_USBBuffer || !dd->ua_USBBuffer2)
        {
            DPRINTF("[USBAudio] PlaySlave: staging buffers not allocated!\n");
            goto quit;
        }

        stage.buf[0] = dd->ua_USBBuffer;
        stage.buf[1] = dd->ua_USBBuffer2;
        stage.cur    = 0;
        stage.off    = 0;

        /* Prime the buffer we will consume first, then the spare. */
        stage.size[0]     = mix_to_usb(AudioCtrl, stage.buf[0]);
        stage.spare_ready = FALSE;
        staging_top_up(AudioCtrl, &stage);

        DPRINTF("[USBAudio] PlaySlave: primed staging (%lu + %lu bytes)\n",
                           (ULONG)stage.size[0], (ULONG)stage.size[1]);

        /* Fill and launch all IORequests */
        for (x = 0; x < iorCount; x++)
        {
            uint32 bytesThisFrame = setup_ior_sizes(IUSBSys, iorTable[x],
                                        subXfersPerIOR, baseSamples,
                                        fracNum, fracDen, &accumulator,
                                        frameSize, maxTransferSize);

            /* Fill IOReq from staging, swapping buffers as needed */
            staging_read(AudioCtrl, &stage, (uint8 *)iorTable[x]->io_Data,
                         bytesThisFrame);

            iorTable[x]->io_Command  = CMD_WRITE;
            iorTable[x]->io_EndPoint = ep;
            iorTable[x]->io_Length   = bytesThisFrame;
            iorTable[x]->io_Actual   = subXfersPerIOR;
            iorTable[x]->io_Error    = 0;

            iorLength[x]    = bytesThisFrame;
            iorSendFrame[x] = haveFrameNo
                            ? IUSBSys->USBGetFrameNumber(ep, NULL) : 0;
            IExec->SendIO((struct IORequest *)iorTable[x]);
            inflight++;

            /* Restore the spare now that the request is on its way. */
            staging_top_up(AudioCtrl, &stage);
        }

        DPRINTF("[USBAudio] PlaySlave: all %lu IORequests launched\n",
                           (ULONG)iorCount);
        /* Report the payload actually programmed per frame, and say plainly
         * whether this is a clamped diagnostic build.  Without this there is
         * no way to tell from a log which binary produced it. */
        DPRINTF("[USBAudio] PlaySlave: streaming started, %lu IOReqs x %lu frames, "
                "%lu bytes/frame\n",
                (ULONG)iorCount, (ULONG)FRAMES_PER_IOR,
                (ULONG)(baseSamples * frameSize));

        /* Tell master we are alive */
        IExec->Signal((struct Task *)dd->ua_MasterTask, 1L << dd->ua_MasterSignal);

        /* ------------------------------------------------------------------
         * Step 6: Main playback loop — async isochronous.
         *
         * Wait for completed IORequests, refill them with new audio
         * from the AHI mixer, and immediately re-send.  The USB stack
         * always has (iorCount - 1) requests queued ahead, providing
         * continuous audio flow without gaps.
         * ------------------------------------------------------------------ */
        {
            uint32 usbsignal  = 1L << usbPort->mp_SigBit;
            uint32 stopsignal = 1L << dd->ua_SlaveSignal;
            uint32 signalmask = usbsignal | stopsignal | SIGBREAKF_CTRL_C;

            for (;;)
            {
                signals = IExec->Wait(signalmask);

                if (signals & (stopsignal | SIGBREAKF_CTRL_C))
                    break;

                if (signals & usbsignal)
                {
                    struct USBIOReq *ureq;
                    uint32 batch    = 0;      /* completions handled this wakeup */
                    uint32 progress = 0;      /* of which actually transferred */

                    while ((ureq = (struct USBIOReq *)IExec->GetMsg(usbPort)))
                    {
                        uint32 idx;
                        uint32 frameNow;

                        loop_count++;
                        batch++;
                        if (inflight > 0)
                            inflight--;

                        /* Map the completion back to its slot. */
                        for (idx = 0; idx < iorCount; idx++)
                            if (iorTable[idx] == ureq)
                                break;

                        frameNow = haveFrameNo
                                 ? IUSBSys->USBGetFrameNumber(ep, NULL) : 0;

                        /*
                         * Detect a submission the HCD refused to schedule.
                         *
                         * When the endpoint's isochronous schedule is full the
                         * EHCI HCD logs "[EHCI] EndPoint iso schedule full" and
                         * returns *without* an error code: the IORequest comes
                         * straight back with io_Error == USBERR_NOERROR and
                         * nothing transferred.  Its own contract is that the
                         * caller must retry later, once other transfers have
                         * completed.
                         *
                         * Treating that as a successful transfer — which is
                         * what this loop used to do — is what burns the CPU:
                         * the request is refilled with fresh audio (discarding
                         * audio that never reached the bus) and resubmitted
                         * immediately, is refused again immediately, and the
                         * loop spins without ever blocking in Wait().
                         *
                         * A genuine isochronous completion cannot occur in the
                         * same USB frame it was submitted in: the transfer is
                         * scheduled into a future frame and takes
                         * FRAMES_PER_IOR milliseconds.  So an unchanged frame
                         * number means the request was never scheduled.
                         * Comparing for equality (rather than subtracting) is
                         * immune to however the frame counter wraps.
                         */
                        if (idx < iorCount && haveFrameNo &&
                            ureq->io_Error == USBERR_NOERROR &&
                            frameNow == iorSendFrame[idx])
                        {
                            rejected_count++;
                            if (rejected_count <= 5 ||
                                (rejected_count & 0x3FF) == 0)
                            {
                                DPRINTF("[USBAudio] PlaySlave: submission refused "
                                        "#%lu (schedule full, will retry)\n",
                                        (ULONG)rejected_count);
                                /* Frame numbers included: if they are always
                                 * equal because the HCD reports a constant (or
                                 * zero) frame number, this detector is
                                 * misfiring and every transfer is being parked
                                 * for no reason. */
                                DPRINTF("[USBAudio] PlaySlave: submission refused #%lu "
                                        "(sendFrame=%lu nowFrame=%lu)\n",
                                        (ULONG)rejected_count,
                                        (ULONG)iorSendFrame[idx], (ULONG)frameNow);
                            }

                            /* Park it: keep its audio and transfer setups
                             * untouched so the retry sends the same samples,
                             * and do not advance the mixer.  Nothing was
                             * transmitted, so nothing should be discarded. */
                            iorParked[idx] = 1;
                            continue;
                        }

                        progress++;

                        /* Sum what the hardware says it actually put on the
                         * bus.  Transfers that "succeed" while moving zero
                         * bytes look identical to working audio from the
                         * outside, and sound like silence. */
                        {
                            uint32 sx;
                            for (sx = 0; sx < subXfersPerIOR; sx++)
                            {
                                struct USBTransferResult *res =
                                    IUSBSys->USBGetIsoTransferResult(ureq, sx, NULL);
                                if (res)
                                    bytes_sent += res->Actual;
                            }
                        }

                        /* Handle isochronous errors gracefully.
                         *
                         * USBERR_FRAMEMISSED is not benign: it means the
                         * request's target frame had already passed and the
                         * audio in it was never put on the bus — an audible
                         * dropout.  It is not fatal (the stream recovers on
                         * the next request) so playback continues, but it is
                         * counted and reported: a rising count is the direct
                         * signal that the pipeline is not being refilled in
                         * time, and it correlates with the HCD logging
                         * "[EHCI] EndPoint iso schedule full". */
                        if (ureq->io_Error != USBERR_NOERROR)
                        {
                            if (ureq->io_Error == USBERR_FRAMEMISSED)
                            {
                                framemissed_count++;
                                consecutive_fatal = 0;
                                /* Rate-limited: log the first few, then every
                                 * 256th, so a persistent problem stays visible
                                 * without flooding the log. */
                                if (framemissed_count <= 5 ||
                                    (framemissed_count & 0xFF) == 0)
                                    DPRINTF("[USBAudio] PlaySlave: FRAMEMISSED #%lu "
                                            "(audio dropped, pipeline late)\n",
                                            (ULONG)framemissed_count);
                            }
                            else if (ureq->io_Error == USBERR_CHECKRESULTS)
                            {
                                /* Some sub-transfers failed; the rest went
                                 * out.  Partial loss, stream stays valid. */
                                checkresults_count++;
                                consecutive_fatal = 0;
                            }
                            else
                            {
                                error_count++;
                                consecutive_fatal++;
                                if (error_count <= 5)
                                {
                                    DPRINTF("[USBAudio] PlaySlave: io_Error=%ld (fatal #%lu)\n",
                                                       (LONG)ureq->io_Error, (ULONG)consecutive_fatal);
                                    DPRINTF("[USBAudio] PlaySlave: transfer error %ld (#%lu)\n",
                                            (LONG)ureq->io_Error, (ULONG)consecutive_fatal);
                                }
                                /* Device removed or dead — stop playing */
                                if (consecutive_fatal >= 3)
                                {
                                    DPRINTF("[USBAudio] PlaySlave: device gone, exiting\n");
                                    dd->ua_DeviceGone = 1;
                                    goto drain;
                                }
                            }
                            ureq->io_Error = USBERR_NOERROR;
                        }
                        else
                        {
                            consecutive_fatal = 0;
                        }

                        {
                            uint32 bytesThisFrame =
                                setup_ior_sizes(IUSBSys, ureq, subXfersPerIOR,
                                                baseSamples, fracNum, fracDen,
                                                &accumulator, frameSize,
                                                maxTransferSize);

                            /* Fill this IOReq with fresh audio from staging.
                             * This is a pure memcpy — the mixer does not run
                             * here, so the time between completion and
                             * re-send stays short and predictable. */
                            staging_read(AudioCtrl, &stage,
                                         (uint8 *)ureq->io_Data, bytesThisFrame);

                            /* Re-send immediately */
                            ureq->io_Command  = CMD_WRITE;
                            ureq->io_EndPoint = ep;
                            ureq->io_Length   = bytesThisFrame;
                            ureq->io_Actual   = subXfersPerIOR;
                            ureq->io_Error    = 0;

                            if (idx < iorCount)
                            {
                                iorLength[idx]    = bytesThisFrame;
                                iorSendFrame[idx] = frameNow;
                            }
                            IExec->SendIO((struct IORequest *)ureq);
                            inflight++;

                            /* Only now, with the pipeline refilled and a full
                             * IORequest of runway ahead, run the AHI mixer to
                             * restore the spare staging buffer. */
                            staging_top_up(AudioCtrl, &stage);
                        }
                    }

                    /* Periodic status: the single line that says whether
                     * audio data is actually reaching the device. */
                    if (loop_count - last_report >= 500)
                    {
                        last_report = loop_count;
                        DPRINTF("[USBAudio] PlaySlave: status loops=%lu sent=%luKB "
                                "rejected=%lu framemissed=%lu err=%lu backoff=%lu "
                                "inflight=%lu\n",
                                (ULONG)loop_count, (ULONG)(bytes_sent >> 10),
                                (ULONG)rejected_count, (ULONG)framemissed_count,
                                (ULONG)error_count, (ULONG)backoff_count,
                                (ULONG)inflight);
                    }

                    /*
                     * Retry parked requests.
                     *
                     * The HCD frees scheduling slots as transfers complete, so
                     * the natural moment to retry is after a real completion —
                     * which also paces the retries at FRAMES_PER_IOR ms instead
                     * of spinning.  A retry that is refused again simply gets
                     * re-parked and waits for the next completion.
                     */
                    if (iorParked && (progress > 0 || inflight == 0))
                    {
                        /* If nothing is in flight, no completion will ever
                         * wake us — we must not spin.  Sleeping one tick is
                         * coarse for audio, but the stream has already
                         * stalled at this point and the alternative is a busy
                         * loop at full CPU. */
                        if (inflight == 0)
                        {
                            backoff_count++;
                            if (backoff_count <= 5 ||
                                (backoff_count & 0xFF) == 0)
                                DPRINTF("[USBAudio] PlaySlave: pipeline stalled, "
                                        "backing off (#%lu)\n",
                                        (ULONG)backoff_count);
                            IDOS->Delay(1);
                        }

                        for (x = 0; x < iorCount; x++)
                        {
                            if (!iorParked[x])
                                continue;

                            /* Resubmit unchanged: same samples, same setups. */
                            iorTable[x]->io_Command  = CMD_WRITE;
                            iorTable[x]->io_EndPoint = ep;
                            iorTable[x]->io_Length   = iorLength[x];
                            iorTable[x]->io_Actual   = subXfersPerIOR;
                            iorTable[x]->io_Error    = 0;

                            iorSendFrame[x] = haveFrameNo
                                            ? IUSBSys->USBGetFrameNumber(ep, NULL)
                                            : 0;
                            iorParked[x] = 0;
                            IExec->SendIO((struct IORequest *)iorTable[x]);
                            inflight++;
                        }
                    }

                    /*
                     * Circuit breaker.
                     *
                     * Backstop for the case where rejections cannot be
                     * identified individually (an HCD that does not maintain a
                     * frame number, so haveFrameNo is FALSE).  A wakeup should
                     * never yield many more completions than there are
                     * requests; if it does, something is completing without
                     * consuming time and the loop would spin.  Yield rather
                     * than burn the CPU.
                     */
                    if (batch > iorCount * 4)
                    {
                        backoff_count++;
                        if (backoff_count <= 5 || (backoff_count & 0xFF) == 0)
                            DPRINTF("[USBAudio] PlaySlave: %lu completions in one "
                                    "wakeup, throttling (#%lu)\n",
                                    (ULONG)batch, (ULONG)backoff_count);
                        IDOS->Delay(1);
                    }
                }
            }
        }

        /* ------------------------------------------------------------------
         * Step 7: Drain — abort all in-flight IORequests.
         * ------------------------------------------------------------------ */
    drain:
        {
            uint32 x;
            for (x = 0; x < iorCount; x++)
            {
                if (iorTable[x])
                {
                    IExec->AbortIO((struct IORequest *)iorTable[x]);
                    IExec->WaitIO((struct IORequest *)iorTable[x]);
                }
            }
        }
    }

quit:
    DPRINTF("[USBAudio] PlaySlave: exiting (loops=%lu xfer_errors=%lu "
                       "framemissed=%lu checkresults=%lu rejected=%lu backoff=%lu)\n",
                       (ULONG)loop_count, (ULONG)error_count,
                       (ULONG)framemissed_count, (ULONG)checkresults_count,
                       (ULONG)rejected_count, (ULONG)backoff_count);

    /* Free IORequests and their buffers */
    if (iorTable && IUSBSys)
    {
        uint32 x;
        for (x = 0; x < iorCount; x++)
        {
            if (iorTable[x])
            {
                IExec->FreeVec(iorTable[x]->io_Data);
                IUSBSys->USBFreeRequest(iorTable[x]);
            }
        }
        IExec->FreeVec(iorTable);
    }

    IExec->FreeVec(iorSendFrame);
    IExec->FreeVec(iorLength);
    IExec->FreeVec(iorParked);

    /* Release USB system resources */
    if (IUSBSys)
        IExec->DropInterface((struct Interface *)IUSBSys);

    if (deviceOpen)
        IExec->CloseDevice(baseReq);

    if (baseReq)
        IExec->FreeSysObject(ASOT_IOREQUEST, baseReq);

    if (usbPort)
        IExec->FreeSysObject(ASOT_PORT, usbPort);

    /* Only touch driver data / signal the master if we actually have a valid
     * AudioCtrl and DriverData — the early "AudioCtrl == NULL" bailout above
     * jumps here too, and dereferencing dd (which reads ahiac_DriverData)
     * would crash otherwise. */
    if (AudioCtrl != NULL && AudioCtrl->ahiac_DriverData != NULL)
    {
        IExec->Forbid();
        dd->ua_SlaveTask = NULL;
        if (dd->ua_SlaveSignal != -1)
        {
            IExec->FreeSignal(dd->ua_SlaveSignal);
            dd->ua_SlaveSignal = -1;
        }

        /* Tell master we are dying */
        IExec->Signal((struct Task *)dd->ua_MasterTask, 1L << dd->ua_MasterSignal);
    }

    /* Multitasking will resume when we are dead */
    return 0;
}
