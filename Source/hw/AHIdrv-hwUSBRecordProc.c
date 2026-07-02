/*
 * AHIdrv-hwUSBRecordProc.c
 *
 * USB Audio recording slave process for the AHI sub-driver.
 *
 * This process runs as a separate AmigaOS task. It:
 *   1. Reads audio data from the USB audio device via asynchronous
 *      isochronous transfers directly to usbsys.device, with multiple
 *      USBIOReqs pre-queued to eliminate data loss.
 *   2. Converts USB audio format (16-bit LE) to AHI format (16-bit BE)
 *   3. Delivers recorded audio to AHI via SamplerFunc callback
 *
 * Uses the same async SendIO/Wait/GetMsg pattern as the playback slave,
 * modelled after usbaudio2 by Chris Handley.
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

/* NOTE: as in the playback slave, the in-flight IORequest count is driven by
 * the HCD's USBA_HCD_CachedIsochronousFrames (clamped to a minimum of 2),
 * not by a fixed number — exceeding the HCD schedule corrupts the pipe. */

#define dd ((struct USBAudioData *)AudioCtrl->ahiac_DriverData)

/*
 * Byte-swap a 16-bit value (BE <-> LE)
 */
static inline uint16 rec_swap16(uint16 x)
{
    return (uint16)((x >> 8) | (x << 8));
}


/*
 * hwUSBRecordSlave
 *
 * Entry point for the recording process created in AHIsub_Start.
 * AudioCtrl is passed via tc_UserData.
 *
 * Uses direct async isochronous IO to usbsys.device with multiple
 * pre-queued CMD_READ USBIOReqs for gap-free recording.
 */
uint32 hwUSBRecordSlave(STRPTR *args UNUSED, int32 arglen UNUSED,
                        struct ExecBase *sysbase)
{
    struct AHIAudioCtrlDrv *AudioCtrl;
    uint32  signals;
    uint32  loop_count = 0;
    uint32  error_count = 0;
    uint32  consecutive_fatal = 0;

    /* Local USB resources */
    struct MsgPort    *usbPort    = NULL;
    struct IORequest  *baseReq    = NULL;
    struct USBSysIFace *IUSBSys   = NULL;
    struct USBIOReq  **iorTable   = NULL;
    uint32             iorCount   = 0;
    struct UsbEndPoint *ep        = NULL;
    BOOL               deviceOpen = FALSE;

    AudioCtrl = (struct AHIAudioCtrlDrv *)sysbase->ThisTask->tc_UserData;

    DPRINTF("[USBAudio] RecordSlave: entry, AudioCtrl=%p\n", AudioCtrl);

    if (AudioCtrl == NULL || AudioCtrl->ahiac_DriverData == NULL)
    {
        DPRINTF("[USBAudio] RecordSlave: AudioCtrl or DriverData is NULL!\n");
        goto quit;
    }

    /* Allocate a signal for the stop request */
    if ((dd->ua_RecSlaveSignal = IExec->AllocSignal(-1)) == -1)
    {
        DPRINTF("[USBAudio] RecordSlave: AllocSignal failed!\n");
        goto quit;
    }

    DPRINTF("[USBAudio] RecordSlave: EP=0x%02lx MaxPkt=%lu Channels=%lu SubSz=%lu\n",
                       (ULONG)dd->ua_RecEndpointAddr, (ULONG)dd->ua_RecMaxPacketSize,
                       (ULONG)dd->ua_RecNumChannels, (ULONG)dd->ua_RecSubframeSize);

    /* ------------------------------------------------------------------
     * Step 1: Open usbsys.device and get IUSBSys interface.
     * ------------------------------------------------------------------ */
    usbPort = IExec->AllocSysObjectTags(ASOT_PORT, TAG_END);
    if (!usbPort)
    {
        DPRINTF("[USBAudio] RecordSlave: AllocSysObject(PORT) failed\n");
        goto quit;
    }

    baseReq = IExec->AllocSysObjectTags(ASOT_IOREQUEST,
                ASOIOR_Size,      sizeof(struct USBIOReq),
                ASOIOR_ReplyPort, (uint32)usbPort,
                TAG_END);
    if (!baseReq)
    {
        DPRINTF("[USBAudio] RecordSlave: AllocSysObject(IOREQUEST) failed\n");
        goto quit;
    }

    if (IExec->OpenDevice("usbsys.device", 0, baseReq, 0))
    {
        DPRINTF("[USBAudio] RecordSlave: OpenDevice(usbsys.device) failed\n");
        goto quit;
    }
    deviceOpen = TRUE;

    IUSBSys = (struct USBSysIFace *)IExec->GetInterface(
                  (struct Library *)baseReq->io_Device, "main", 1, NULL);
    if (!IUSBSys)
    {
        DPRINTF("[USBAudio] RecordSlave: GetInterface(USBSys) failed\n");
        goto quit;
    }

    /* ------------------------------------------------------------------
     * Step 2: Get the UsbEndPoint for the recording IN endpoint.
     * ------------------------------------------------------------------ */
    {
        uint8  ep_addr = dd->ua_RecEndpointAddr;
        uint32 ep_idx  = (ep_addr & 0x80) ? ((ep_addr & 0x0F) + 16)
                                           : (ep_addr & 0x0F);

        if (dd->ua_DevHandle && dd->ua_DevHandle->data)
        {
            ep = dd->ua_DevHandle->data->lad_EndPoints[ep_idx];
            if (!ep)
            {
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
        DPRINTF("[USBAudio] RecordSlave: could not get UsbEndPoint for EP 0x%02lx\n",
                           (ULONG)dd->ua_RecEndpointAddr);
        goto quit;
    }

    /* Tell master we are alive */
    DPRINTF("[USBAudio] RecordSlave: signaling master, starting record loop\n");
    IExec->Signal((struct Task *)dd->ua_MasterTask, 1L << dd->ua_MasterSignal);

    /* ------------------------------------------------------------------
     * Step 3: Query isochronous parameters and allocate IORequests.
     * ------------------------------------------------------------------ */
    {
        uint32 cachedFrames      = 0;
        uint32 maxTransferSize   = 0;
        uint32 transfersPerFrame = 0;
        uint32 isoChunkSize;
        uint32 x, y;

        IUSBSys->USBGetEndPointAttrs(ep,
            USBA_HCD_CachedIsochronousFrames, &cachedFrames,
            USBA_EP_MaxTransferSize, &maxTransferSize,
            USBA_EP_TransfersPerFrame, &transfersPerFrame,
            TAG_END);

        DPRINTF("[USBAudio] RecordSlave: cachedFrames=%lu maxXferSize=%lu xfersPerFrame=%lu\n",
                           (ULONG)cachedFrames, (ULONG)maxTransferSize,
                           (ULONG)transfersPerFrame);

        /* Use at least 2 IOReqs for double-buffering, but do NOT exceed
         * what the HCD can schedule — exceeding cachedFrames causes
         * "[EHCI] EndPoint iso schedule full" and corrupts the pipe. */
        if (cachedFrames < 2)
            cachedFrames = 2;

        if (transfersPerFrame == 0)
            transfersPerFrame = 1;

        /* For input: use the endpoint's maxTransferSize as-is */
        isoChunkSize = maxTransferSize * transfersPerFrame;

        if (isoChunkSize == 0 || cachedFrames == 0)
        {
            DPRINTF("[USBAudio] RecordSlave: invalid isochronous parameters!\n");
            goto quit;
        }

        /* Allocate IORequests */
        iorTable = (struct USBIOReq **)IExec->AllocVecTags(
                       sizeof(struct USBIOReq *) * (cachedFrames + 1),
                       AVT_ClearWithValue, 0,
                       TAG_END);
        if (!iorTable)
        {
            DPRINTF("[USBAudio] RecordSlave: iorTable alloc failed\n");
            goto quit;
        }

        for (x = 0; x < cachedFrames; x++)
        {
            iorTable[x] = IUSBSys->USBAllocRequest((struct IORequest *)baseReq, TAG_END);
            if (!iorTable[x])
            {
                iorCount = x;
                goto quit;
            }

            iorTable[x]->io_Data = IExec->AllocVecTags(isoChunkSize,
                                       AVT_Type, MEMF_SHARED,
                                       AVT_ClearWithValue, 0,
                                       TAG_END);
            if (!iorTable[x]->io_Data)
            {
                IUSBSys->USBFreeRequest(iorTable[x]);
                iorTable[x] = NULL;
                iorCount = x;
                goto quit;
            }

            IUSBSys->USBSetIsoTransferCount(iorTable[x], transfersPerFrame);
            for (y = 0; y < transfersPerFrame; y++)
            {
                IUSBSys->USBSetIsoTransferSetup(iorTable[x],
                    y, y * maxTransferSize, maxTransferSize);
            }

            ((struct IORequest *)iorTable[x])->io_Message.mn_ReplyPort = usbPort;
        }
        iorCount = cachedFrames;

        DPRINTF("[USBAudio] RecordSlave: allocated %lu IORequests (%lu bytes each)\n",
                           (ULONG)iorCount, (ULONG)isoChunkSize);

        /* ------------------------------------------------------------------
         * Step 4: Launch all CMD_READ IORequests.
         * ------------------------------------------------------------------ */
        for (x = 0; x < iorCount; x++)
        {
            iorTable[x]->io_Command  = CMD_READ;
            iorTable[x]->io_EndPoint = ep;
            iorTable[x]->io_Length   = isoChunkSize;
            iorTable[x]->io_Error    = 0;

            IExec->SendIO((struct IORequest *)iorTable[x]);
        }

        /* ------------------------------------------------------------------
         * Step 5: Main recording loop — async isochronous.
         *
         * AHI delivery buffer for SamplerFunc.
         * We accumulate samples and deliver when we have enough.
         * ------------------------------------------------------------------ */
        {
            uint32 max_samples = USB_AUDIO_REC_SAMPLES;
            int16 *ahi_buf = (int16 *)IExec->AllocVecTags(
                                 max_samples * 2 * sizeof(int16),
                                 AVT_Type, MEMF_SHARED,
                                 AVT_ClearWithValue, 0,
                                 TAG_END);
            if (!ahi_buf)
            {
                DPRINTF("[USBAudio] RecordSlave: cannot alloc AHI buffer!\n");
                goto drain;
            }

            uint32 ahi_buf_samples = 0;

            uint32 usbsignal  = 1L << usbPort->mp_SigBit;
            uint32 stopsignal = 1L << dd->ua_RecSlaveSignal;
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

                        /* Handle errors gracefully */
                        if (ureq->io_Error != USBERR_NOERROR)
                        {
                            if (ureq->io_Error != -37 && ureq->io_Error != -38)
                            {
                                error_count++;
                                consecutive_fatal++;
                                if (error_count <= 5)
                                    DPRINTF("[USBAudio] RecordSlave: io_Error=%ld (fatal #%lu)\n",
                                                       (LONG)ureq->io_Error, (ULONG)consecutive_fatal);
                                if (consecutive_fatal >= 3)
                                {
                                    DPRINTF("[USBAudio] RecordSlave: device gone, exiting\n");
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

                        /* Process each isochronous sub-transfer */
                        {
                            uint8 *dta = (uint8 *)ureq->io_Data;

                            for (x = 0; x < ureq->io_Actual; x++)
                            {
                                struct USBTransferResult *ures =
                                    IUSBSys->USBGetIsoTransferResult(ureq, x, NULL);

                                if (!ures || ures->Actual == 0)
                                    continue;

                                /* Convert received USB audio (LE) to AHI (BE stereo) */
                                uint32 bytes   = ures->Actual;
                                int16  *src    = (int16 *)(dta + x * maxTransferSize);
                                uint32 frames  = bytes / ((uint32)dd->ua_RecSubframeSize *
                                                          (uint32)dd->ua_RecNumChannels);
                                uint32 i;

                                for (i = 0; i < frames; i++)
                                {
                                    if (dd->ua_RecNumChannels >= 2)
                                    {
                                        ahi_buf[ahi_buf_samples * 2]     = (int16)rec_swap16((uint16)src[i * 2]);
                                        ahi_buf[ahi_buf_samples * 2 + 1] = (int16)rec_swap16((uint16)src[i * 2 + 1]);
                                    }
                                    else
                                    {
                                        int16 s = (int16)rec_swap16((uint16)src[i]);
                                        ahi_buf[ahi_buf_samples * 2]     = s;
                                        ahi_buf[ahi_buf_samples * 2 + 1] = s;
                                    }
                                    ahi_buf_samples++;

                                    if (ahi_buf_samples >= max_samples)
                                    {
                                        struct AHIRecordMessage rm;
                                        rm.ahirm_Type   = AHIST_S16S;
                                        rm.ahirm_Buffer = ahi_buf;
                                        rm.ahirm_Length = ahi_buf_samples;

                                        IUtility->CallHookPkt(AudioCtrl->ahiac_SamplerFunc,
                                                               (APTR)AudioCtrl, &rm);
                                        ahi_buf_samples = 0;
                                    }
                                }
                            }
                        }

                        /* Re-send for more data */
                        ureq->io_Command  = CMD_READ;
                        ureq->io_EndPoint = ep;
                        ureq->io_Length   = isoChunkSize;
                        ureq->io_Error    = 0;

                        IExec->SendIO((struct IORequest *)ureq);
                    }
                }
            }

            IExec->FreeVec(ahi_buf);
        }

drain:
        /* Abort all in-flight IORequests */
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
    DPRINTF("[USBAudio] RecordSlave: exiting (loops=%lu xfer_errors=%lu)\n",
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

    if (IUSBSys)
        IExec->DropInterface((struct Interface *)IUSBSys);

    if (deviceOpen)
        IExec->CloseDevice(baseReq);

    if (baseReq)
        IExec->FreeSysObject(ASOT_IOREQUEST, baseReq);

    if (usbPort)
        IExec->FreeSysObject(ASOT_PORT, usbPort);

    /* Only touch driver data / signal the master if AudioCtrl and DriverData
     * are valid — the early "AudioCtrl == NULL" bailout jumps here too, and
     * dereferencing dd (which reads ahiac_DriverData) would crash otherwise. */
    if (AudioCtrl != NULL && AudioCtrl->ahiac_DriverData != NULL)
    {
        IExec->Forbid();
        dd->ua_RecSlaveTask = NULL;
        if (dd->ua_RecSlaveSignal != -1)
        {
            IExec->FreeSignal(dd->ua_RecSlaveSignal);
            dd->ua_RecSlaveSignal = -1;
        }

        /* Tell master we are dying */
        IExec->Signal((struct Task *)dd->ua_MasterTask, 1L << dd->ua_MasterSignal);
    }

    return 0;
}
