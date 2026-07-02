/*
 * AHIdrv-USBaudioInit.c
 *
 * ROMTAG, library initialization and manager interface for usbaudio.audio
 * AHI sub-driver that outputs through USB Audio Class devices via libusb-1.
 */

#include <exec/exec.h>
#include <proto/exec.h>
#include <dos/dos.h>
#include <exec/types.h>
#include <utility/utility.h>
#include <proto/utility.h>
#include <libraries/ahi_sub.h>
#include <proto/usbaudio.h>
#include <stdarg.h>
#include "includes/AHIdrv-USBaudio.h"

/* Version Tag */
#include "includes/AHIdrv-USBaudio.audio_rev.h"

STATIC CONST UBYTE USED verstag[] = VERSTAG;

/*
 * Global interface pointers — opened in libInit.
 */
struct ExecIFace    *IExec    = NULL;
#ifndef __CLIB4__
struct Library      *NewlibBase  = NULL;
struct Interface    *INewlib     = NULL;
#else
struct Library      *Clib4Base   = NULL;
struct Interface    *IClib4      = NULL;
#endif
struct Library      *DOSBase     = NULL;
struct DOSIFace     *IDOS        = NULL;
struct Library      *UtilityBase = NULL;
struct UtilityIFace *IUtility    = NULL;
struct Library      *Libusb1Base = NULL;
struct Libusb1IFace *ILibusb1    = NULL;

/* Global USB audio device info — discovered lazily */
struct USBAudioDeviceInfo g_usb_info = { 0 };

/* Array of all discovered USB audio devices */
struct USBAudioDeviceInfo g_usb_devices[MAX_USB_AUDIO_DEVICES];
int32 g_usb_num_devices = 0;

/* Flattened output/input arrays for AHI UI */
struct USBAudioFlatOutput g_usb_flat_outputs[MAX_TOTAL_OUTPUTS];
int32 g_num_flat_outputs = 0;
struct USBAudioFlatInput  g_usb_flat_inputs[MAX_TOTAL_INPUTS];
int32 g_num_flat_inputs  = 0;

/* User-defined device names from ENVARC: prefs file */
struct USBAudioUserDevice g_user_devices[MAX_USER_DEVICES];
int32 g_num_user_devices = 0;

/* Set to 1 by libExpunge (even if deferred) so the next libOpen rescans.
 * This bridges the usbaudio.usbfd RemLibrary trigger to the scan logic. */
int32 g_force_rescan = 0;

/*
 * _start — library is not directly executable
 */
int32 _start(void);
int32 _start(void)
{
    return RETURN_FAIL;
}

/* Forward declaration — libClose may trigger a deferred expunge */
STATIC APTR libExpunge(struct LibraryManagerInterface *Self);

/*
 * Interface stub functions
 */
struct USBAudioIFace * _usbaudio_Clone(struct USBAudioIFace *Self)
{
    return (struct USBAudioIFace *)0;
}

uint32 _usbaudio_Obtain(struct USBAudioIFace *Self)
{
    return (uint32)0;
}

uint32 _usbaudio_Release(struct USBAudioIFace *Self)
{
    return (uint32)0;
}

/*
 * libOpen — called when AHI opens usbaudio.audio
 *
 * This ALWAYS succeeds (returns the library base) so that
 * AddAudioModes can register the mode file at boot time.
 *
 * NO USB access happens here.  All USB work (opening libusb-1,
 * scanning devices) is deferred to scan_usb_audio_device(),
 * which is called lazily from AHIsub_AllocAudio / AHIsub_GetAttr.
 *
 * This avoids a deadlock when the USBFD triggers AddAudioModes
 * during boot: the USB stack is still initialising, and any
 * OpenLibrary/OpenDevice call that touches usbsys.device will
 * block forever.
 */
STATIC struct USBAudioBase *libOpen(struct LibraryManagerInterface *Self, ULONG version)
{
    struct USBAudioBase *libBase = (struct USBAudioBase *)Self->Data.LibBase;

    DPRINTF("[USBAudio] libOpen: version=%lu, our VERSION=%lu\n", version, (ULONG)VERSION);

    if (version > VERSION)
    {
        DPRINTF("[USBAudio] libOpen: version mismatch, returning NULL\n");
        return NULL;
    }

    libBase->libNode.lib_OpenCnt++;

    /* A pending deferred expunge is cancelled by a new open. */
    libBase->libNode.lib_Flags &= ~LIBF_DELEXP;

    DPRINTF("[USBAudio] libOpen: OK, OpenCnt=%lu\n", (ULONG)libBase->libNode.lib_OpenCnt);
    return libBase;
}

/*
 * libClose
 */
STATIC APTR libClose(struct LibraryManagerInterface *Self)
{
    struct USBAudioBase *libBase = (struct USBAudioBase *)Self->Data.LibBase;

    libBase->libNode.lib_OpenCnt--;

    /* If this was the last user and an expunge was deferred while open,
     * complete it now and return the seglist so the loader can unload us. */
    if (libBase->libNode.lib_OpenCnt == 0 &&
        (libBase->libNode.lib_Flags & LIBF_DELEXP))
    {
        return libExpunge(Self);
    }

    return 0;
}

/*
 * libExpunge — clean up and release resources
 */
STATIC APTR libExpunge(struct LibraryManagerInterface *Self)
{
    struct ExecIFace *IExec = (struct ExecIFace *)(*(struct ExecBase **)4)->MainInterface;
    APTR result = (APTR)0;
    struct USBAudioBase *libBase = (struct USBAudioBase *)Self->Data.LibBase;

    DPRINTF("[USBAudio] libExpunge: OpenCnt=%lu\n", (ULONG)libBase->libNode.lib_OpenCnt);

    if (libBase->libNode.lib_OpenCnt == 0)
    {
        result = (APTR)libBase->segList;

        /* Shut down libusb default context */
        if (ILibusb1)
        {
            ILibusb1->libusb_exit(NULL);
        }

        /* NULL each global after release so the expunge is idempotent and a
         * stray later access hits NULL rather than a freed library base. */
        if (ILibusb1)    { IExec->DropInterface((struct Interface *)ILibusb1); ILibusb1 = NULL; }
        if (Libusb1Base) { IExec->CloseLibrary(Libusb1Base);                   Libusb1Base = NULL; }
#ifndef __CLIB4__
        if (INewlib)     { IExec->DropInterface(INewlib);                      INewlib = NULL; }
        if (NewlibBase)  { IExec->CloseLibrary(NewlibBase);                    NewlibBase = NULL; }
#else
        if (IClib4)      { IExec->DropInterface(IClib4);                       IClib4 = NULL; }
        if (Clib4Base)   { IExec->CloseLibrary(Clib4Base);                     Clib4Base = NULL; }
#endif
        if (IUtility)    { IExec->DropInterface((struct Interface *)IUtility); IUtility = NULL; }
        if (UtilityBase) { IExec->CloseLibrary(UtilityBase);                   UtilityBase = NULL; }
        if (IDOS)        { IExec->DropInterface((struct Interface *)IDOS);     IDOS = NULL; }
        if (DOSBase)     { IExec->CloseLibrary(DOSBase);                       DOSBase = NULL; }

        IExec->Remove((struct Node *)libBase);
        IExec->DeleteLibrary((struct Library *)libBase);
    }
    else
    {
        /* Expunge deferred (OpenCnt > 0).  Mark for rescan so that
         * the next libOpen discovers any newly attached/detached
         * USB Audio devices. */
        g_force_rescan = 1;
        libBase->libNode.lib_Flags |= LIBF_DELEXP;
    }

    return result;
}

/*
 * libInit — ROMTAG init function.
 * Opens only the basic CRT libraries (dos, utility, newlib).
 * libusb-1.library is deferred to libOpen to avoid deadlocking
 * the USB stack when the driver is loaded during USBFD callbacks
 * at boot time (the stack is still initializing).
 */
STATIC struct USBAudioBase *libInit(struct Library *LibraryBase, APTR seglist, struct Interface *exec)
{
    struct USBAudioBase *libBase = (struct USBAudioBase *)LibraryBase;
    IExec = (struct ExecIFace *)exec;

    DPRINTF("[USBAudio] libInit: entry\n");

    libBase->libNode.lib_Node.ln_Type = NT_LIBRARY;
    libBase->libNode.lib_Node.ln_Pri  = 0;
    libBase->libNode.lib_Node.ln_Name = (STRPTR) LIBNAME;
    libBase->libNode.lib_Flags        = LIBF_SUMUSED | LIBF_CHANGED;
    libBase->libNode.lib_Version      = VERSION;
    libBase->libNode.lib_Revision     = REVISION;
    libBase->libNode.lib_IdString     = (STRPTR) VSTRING;

    libBase->segList = (BPTR)seglist;

    /* Open dos.library */
    DOSBase = IExec->OpenLibrary("dos.library", 50L);
    if (!DOSBase) { DPRINTF("[USBAudio] Failed to open dos.library\n"); goto fail; }
    IDOS = (struct DOSIFace *)IExec->GetInterface(DOSBase, "main", 1, NULL);
    if (!IDOS) { DPRINTF("[USBAudio] Failed to get IDOS interface\n"); goto fail; }

    DPRINTF("[USBAudio] dos.library opened OK\n");

    /* Open utility.library */
    UtilityBase = IExec->OpenLibrary("utility.library", 50L);
    if (!UtilityBase) { DPRINTF("[USBAudio] Failed to open utility.library\n"); goto fail; }
    IUtility = (struct UtilityIFace *)IExec->GetInterface(UtilityBase, "main", 1, NULL);
    if (!IUtility) { DPRINTF("[USBAudio] Failed to get IUtility interface\n"); goto fail; }

    DPRINTF("[USBAudio] utility.library opened OK\n");

#ifndef __CLIB4__
    /* Open newlib.library (required for C runtime in shared lib) */
    NewlibBase = IExec->OpenLibrary("newlib.library", 50L);
    if (!NewlibBase) { DPRINTF("[USBAudio] Failed to open newlib.library\n"); goto fail; }
    INewlib = IExec->GetInterface(NewlibBase, "main", 1, NULL);
    if (!INewlib) { DPRINTF("[USBAudio] Failed to get INewlib interface\n"); goto fail; }
    DPRINTF("[USBAudio] newlib.library opened OK\n");
#else
    /* Open clib4.library (required for C runtime in shared lib) */
    Clib4Base = IExec->OpenLibrary("clib4.library", 2L);
    if (!Clib4Base) { DPRINTF("[USBAudio] Failed to open clib4.library\n"); goto fail; }
    IClib4 = IExec->GetInterface(Clib4Base, "main", 1, NULL);
    if (!IClib4) { DPRINTF("[USBAudio] Failed to get IClib4 interface\n"); goto fail; }
    DPRINTF("[USBAudio] clib4.library opened OK\n");
#endif

    /* libusb-1.library is NOT opened here — deferred to libOpen.
     * Opening it here would call _main_Clone → OpenDevice("usbsys.device")
     * which deadlocks when called from a USBFD callback during boot
     * (the USB stack is still initializing). */

    DPRINTF("[USBAudio] libInit: complete\n");
    return libBase;

fail:
    /* Release everything opened so far, in reverse order.  On init failure
     * the OS does NOT call libExpunge, so without this the already-opened
     * libraries/interfaces would leak permanently.  Guarded + NULLed so the
     * cleanup is order-independent. */
#ifndef __CLIB4__
    if (INewlib)    { IExec->DropInterface(INewlib);              INewlib = NULL; }
    if (NewlibBase) { IExec->CloseLibrary(NewlibBase);           NewlibBase = NULL; }
#else
    if (IClib4)     { IExec->DropInterface(IClib4);               IClib4 = NULL; }
    if (Clib4Base)  { IExec->CloseLibrary(Clib4Base);            Clib4Base = NULL; }
#endif
    if (IUtility)   { IExec->DropInterface((struct Interface *)IUtility); IUtility = NULL; }
    if (UtilityBase){ IExec->CloseLibrary(UtilityBase);         UtilityBase = NULL; }
    if (IDOS)       { IExec->DropInterface((struct Interface *)IDOS);     IDOS = NULL; }
    if (DOSBase)    { IExec->CloseLibrary(DOSBase);             DOSBase = NULL; }
    return NULL;
}

/*
 * Manager interface — Obtain/Release with PPC atomic ops
 */
STATIC uint32 _manager_Obtain(struct LibraryManagerInterface *Self)
{
    uint32 res;
    __asm__ __volatile__(
    "1:	lwarx	%0,0,%1\n"
    "addic	%0,%0,1\n"
    "stwcx.	%0,0,%1\n"
    "bne-	1b"
    : "=&r" (res)
    : "r" (&Self->Data.RefCount)
    : "cc", "memory");
    return res;
}

STATIC uint32 _manager_Release(struct LibraryManagerInterface *Self)
{
    uint32 res;
    __asm__ __volatile__(
    "1:	lwarx	%0,0,%1\n"
    "addic	%0,%0,-1\n"
    "stwcx.	%0,0,%1\n"
    "bne-	1b"
    : "=&r" (res)
    : "r" (&Self->Data.RefCount)
    : "cc", "memory");
    return res;
}

STATIC CONST APTR lib_manager_vectors[] =
{
    _manager_Obtain,
    _manager_Release,
    NULL,
    NULL,
    libOpen,
    libClose,
    libExpunge,
    NULL,
    (APTR)-1
};

STATIC CONST struct TagItem lib_managerTags[] =
{
    { MIT_Name,        (Tag)"__library"         },
    { MIT_VectorTable, (Tag)lib_manager_vectors  },
    { MIT_Version,     1                         },
    { TAG_DONE,        0                         }
};

/* Include the main interface vector table */
#include "AHIdrv-USBaudioVectors.c"

STATIC CONST struct TagItem mainTags[] =
{
    { MIT_Name,        (Tag)"main"        },
    { MIT_VectorTable, (Tag)main_vectors  },
    { MIT_Version,     1                  },
    { TAG_DONE,        0                  }
};

STATIC CONST CONST_APTR libInterfaces[] =
{
    lib_managerTags,
    mainTags,
    NULL
};

STATIC CONST struct TagItem libCreateTags[] =
{
    { CLT_DataSize,  sizeof(struct USBAudioBase) },
    { CLT_InitFunc,  (Tag)libInit                },
    { CLT_Interfaces,(Tag)libInterfaces           },
    { TAG_DONE,      0                            }
};

/* ROMTAG */
STATIC CONST struct Resident lib_res USED =
{
    RTC_MATCHWORD,
    (struct Resident *)&lib_res,
    (APTR)(&lib_res + 1),
    RTF_NATIVE | RTF_AUTOINIT,
    VERSION,
    NT_LIBRARY,
    -120,
    LIBNAME,
    VSTRING,
    (APTR)libCreateTags
};
