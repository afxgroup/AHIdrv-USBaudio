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

/* Minimum number of isochronous IORequests to keep in flight.
 * More requests = larger buffer = more resilient to scheduling jitter.
 * 8 IOReqs at 1 frame each = ~8ms cushion. */
#define MIN_ISO_IOS 8

/* Number of USB frames packed into each isochronous IORequest.
 * More frames per IOReq = larger buffer = more time for refill.
 * 8 frames at 1ms each = 8ms per IOReq; with 2 IOReqs = 16ms total. */
#define FRAMES_PER_IOR 8

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

    /* Local USB resources — opened and freed entirely within this process */
    struct MsgPort    *usbPort    = NULL;
    struct IORequest  *baseReq    = NULL;
    struct USBSysIFace *IUSBSys   = NULL;
    struct USBIOReq  **iorTable   = NULL;
    uint32             iorCount   = 0;
    struct UsbEndPoint *ep        = NULL;
    BOOL               deviceOpen = FALSE;

    /* Staging buffer for AHI mix output */
    uint8  *staging     = NULL;
    uint32  staging_size = 0;   /* bytes produced by last mix_to_usb */
    uint32  staging_off  = 0;   /* current read offset into staging */

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
        uint32 maxIsoChunkSize;  /* max bytes per IOReq (for allocation) */
        uint32 frameSize;        /* bytes per sample frame: channels * subframeSize */
        uint32 baseSamples;      /* floor(sampleRate / (TPF * 1000)) per sub-frame */
        uint32 fracNum;          /* sampleRate % (TPF * 1000) — fractional part */
        uint32 fracDen;          /* TPF * 1000 — accumulator threshold */
        uint32 accumulator = 0;  /* fractional sample accumulator */
        uint32 subXfersPerIOR;   /* total sub-transfers per IORequest */
        uint32 iorBufSize;       /* DMA buffer size per IORequest */
        uint32 x, y;

        IUSBSys->USBGetEndPointAttrs(ep,
            USBA_HCD_CachedIsochronousFrames, &cachedFrames,
            USBA_EP_MaxTransferSize, &maxTransferSize,
            USBA_EP_TransfersPerFrame, &transfersPerFrame,
            TAG_END);

        DPRINTF("[USBAudio] PlaySlave: cachedFrames=%lu maxXferSize=%lu xfersPerFrame=%lu\n",
                           (ULONG)cachedFrames, (ULONG)maxTransferSize,
                           (ULONG)transfersPerFrame);

        /* Use at least 2 IOReqs for double-buffering, but do NOT exceed
         * what the HCD can schedule — exceeding cachedFrames causes
         * "[EHCI] EndPoint iso schedule full" and corrupts the pipe. */
        if (cachedFrames < 2)
            cachedFrames = 2;

        if (transfersPerFrame == 0)
            transfersPerFrame = 1;

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

        /* Pack multiple USB frames per IORequest for timing resilience.
         * With cachedFrames=1, each IOReq of 1 frame gives only ~1ms
         * margin for refill — causing clicks when mix callbacks are slow.
         * Packing 8 frames per IOReq gives ~8ms margin. */
        subXfersPerIOR = FRAMES_PER_IOR * transfersPerFrame;
        iorBufSize     = FRAMES_PER_IOR * maxIsoChunkSize;

        DPRINTF("[USBAudio] PlaySlave: frameSize=%lu baseSamples=%lu fracNum=%lu/%lu framesPerIOR=%lu iorCount=%lu\n",
                           (ULONG)frameSize, (ULONG)baseSamples,
                           (ULONG)fracNum, (ULONG)fracDen,
                           (ULONG)FRAMES_PER_IOR, (ULONG)cachedFrames);

        if (maxIsoChunkSize == 0 || cachedFrames == 0 || baseSamples == 0)
        {
            DPRINTF("[USBAudio] PlaySlave: invalid isochronous parameters!\n");
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
                       sizeof(struct USBIOReq *) * (cachedFrames + 1),
                       AVT_ClearWithValue, 0,
                       TAG_END);
        if (!iorTable)
        {
            DPRINTF("[USBAudio] PlaySlave: iorTable alloc failed\n");
            goto quit;
        }

        for (x = 0; x < cachedFrames; x++)
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
        iorCount = cachedFrames;

        DPRINTF("[USBAudio] PlaySlave: allocated %lu IORequests (%lu bytes, %lu subXfers each)\n",
                           (ULONG)iorCount, (ULONG)iorBufSize, (ULONG)subXfersPerIOR);

        /* ------------------------------------------------------------------
         * Step 5: Set up staging buffer and fill initial IORequests.
         *
         * Use ua_USBBuffer as the staging area for mix_to_usb output.
         * As IOReqs are filled, we consume data from staging and call
         * mix_to_usb again when the staging buffer runs out.
         * ------------------------------------------------------------------ */
        staging = dd->ua_USBBuffer;

        /* First mix to fill the staging buffer */
        staging_size = mix_to_usb(AudioCtrl, staging);
        staging_off  = 0;

        DPRINTF("[USBAudio] PlaySlave: first mix produced %lu bytes\n",
                           (ULONG)staging_size);

        /* Fill and launch all IORequests */
        for (x = 0; x < iorCount; x++)
        {
            /* Compute this IOReq's sub-frame sizes via accumulator */
            uint32 bytesThisFrame = 0;
            for (y = 0; y < subXfersPerIOR; y++)
            {
                uint32 n = baseSamples;
                accumulator += fracNum;
                if (accumulator >= fracDen)
                {
                    accumulator -= fracDen;
                    n++;
                }
                uint32 subLen = n * frameSize;
                if (subLen > maxTransferSize) subLen = maxTransferSize;
                IUSBSys->USBSetIsoTransferSetup(iorTable[x], y,
                    bytesThisFrame, subLen);
                bytesThisFrame += subLen;
            }

            /* Fill IOReq from staging, wrapping to next mix as needed */
            {
                uint32 remaining = staging_size - staging_off;
                if (remaining >= bytesThisFrame)
                {
                    memcpy(iorTable[x]->io_Data, staging + staging_off, bytesThisFrame);
                    staging_off += bytesThisFrame;
                }
                else
                {
                    memcpy(iorTable[x]->io_Data, staging + staging_off, remaining);
                    staging_size = mix_to_usb(AudioCtrl, staging);
                    staging_off  = bytesThisFrame - remaining;
                    memcpy((uint8 *)iorTable[x]->io_Data + remaining,
                           staging, staging_off);
                }
            }

            iorTable[x]->io_Command  = CMD_WRITE;
            iorTable[x]->io_EndPoint = ep;
            iorTable[x]->io_Length   = bytesThisFrame;
            iorTable[x]->io_Actual   = subXfersPerIOR;
            iorTable[x]->io_Error    = 0;

            IExec->SendIO((struct IORequest *)iorTable[x]);
        }

        DPRINTF("[USBAudio] PlaySlave: all %lu IORequests launched\n",
                           (ULONG)iorCount);

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

                    while ((ureq = (struct USBIOReq *)IExec->GetMsg(usbPort)))
                    {
                        loop_count++;

                        /* Handle isochronous errors gracefully */
                        if (ureq->io_Error != USBERR_NOERROR)
                        {
                            if (ureq->io_Error != -37 && ureq->io_Error != -38)
                            {
                                error_count++;
                                consecutive_fatal++;
                                if (error_count <= 5)
                                    DPRINTF("[USBAudio] PlaySlave: io_Error=%ld (fatal #%lu)\n",
                                                       (LONG)ureq->io_Error, (ULONG)consecutive_fatal);
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

                        /* Compute this IOReq's sub-frame sizes via accumulator */
                        {
                            uint32 bytesThisFrame = 0;
                            for (y = 0; y < subXfersPerIOR; y++)
                            {
                                uint32 n = baseSamples;
                                accumulator += fracNum;
                                if (accumulator >= fracDen)
                                {
                                    accumulator -= fracDen;
                                    n++;
                                }
                                uint32 subLen = n * frameSize;
                                if (subLen > maxTransferSize) subLen = maxTransferSize;
                                IUSBSys->USBSetIsoTransferSetup(ureq, y,
                                    bytesThisFrame, subLen);
                                bytesThisFrame += subLen;
                            }

                            /* Fill this IOReq with fresh audio from staging */
                            {
                                uint32 remaining = staging_size - staging_off;
                                if (remaining >= bytesThisFrame)
                                {
                                    memcpy(ureq->io_Data, staging + staging_off, bytesThisFrame);
                                    staging_off += bytesThisFrame;
                                }
                                else
                                {
                                    memcpy(ureq->io_Data, staging + staging_off, remaining);
                                    staging_size = mix_to_usb(AudioCtrl, staging);
                                    staging_off  = bytesThisFrame - remaining;
                                    memcpy((uint8 *)ureq->io_Data + remaining,
                                           staging, staging_off);
                                }
                            }

                            /* Re-send immediately */
                            ureq->io_Command  = CMD_WRITE;
                            ureq->io_EndPoint = ep;
                            ureq->io_Length   = bytesThisFrame;
                            ureq->io_Actual   = subXfersPerIOR;
                            ureq->io_Error    = 0;

                            IExec->SendIO((struct IORequest *)ureq);
                        }
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
    DPRINTF("[USBAudio] PlaySlave: exiting (loops=%lu xfer_errors=%lu)\n",
                       (ULONG)loop_count, (ULONG)error_count);

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

    /* Release USB system resources */
    if (IUSBSys)
        IExec->DropInterface((struct Interface *)IUSBSys);

    if (deviceOpen)
        IExec->CloseDevice(baseReq);

    if (baseReq)
        IExec->FreeSysObject(ASOT_IOREQUEST, baseReq);

    if (usbPort)
        IExec->FreeSysObject(ASOT_PORT, usbPort);

    IExec->Forbid();
    dd->ua_SlaveTask = NULL;
    IExec->FreeSignal(dd->ua_SlaveSignal);
    dd->ua_SlaveSignal = -1;

    /* Tell master we are dying */
    IExec->Signal((struct Task *)dd->ua_MasterTask, 1L << dd->ua_MasterSignal);

    /* Multitasking will resume when we are dead */
    return 0;
}
