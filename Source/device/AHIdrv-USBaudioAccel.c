/*
 * AHIdrv-USBaudioAccel.c
 *
 * Stub functions for AHI sub-driver methods that are
 * not applicable to a mixing-mode USB audio driver.
 *
 * SetVol, SetFreq, SetSound — only used by direct/split mode drivers.
 * SetEffect, LoadSound, UnloadSound — not applicable.
 */

#include <exec/exec.h>
#include <proto/exec.h>
#include <dos/dos.h>
#include <exec/types.h>
#include <libraries/ahi_sub.h>
#include <proto/usbaudio.h>
#include <stdarg.h>
#include "includes/AHIdrv-USBaudio.h"


uint32 _usbaudio_AHIsub_SetVol(struct USBAudioIFace    *Self,
                                uint16                   Channel,
                                Fixed                    Volume,
                                sposition                Pan,
                                struct AHIAudioCtrlDrv  *AudioCtrl,
                                uint32                   Flags)
{
    return (uint32)AHIS_UNKNOWN;
}


uint32 _usbaudio_AHIsub_SetFreq(struct USBAudioIFace    *Self,
                                 uint16                   Channel,
                                 uint32                   Freq,
                                 struct AHIAudioCtrlDrv  *AudioCtrl,
                                 uint32                   Flags)
{
    return (uint32)AHIS_UNKNOWN;
}


uint32 _usbaudio_AHIsub_SetSound(struct USBAudioIFace    *Self,
                                  uint16                   Channel,
                                  uint16                   Sound,
                                  uint32                   Offset,
                                  int32                    Lenght,
                                  struct AHIAudioCtrlDrv  *AudioCtrl,
                                  uint32                   Flags)
{
    return (uint32)AHIS_UNKNOWN;
}


uint32 _usbaudio_AHIsub_SetEffect(struct USBAudioIFace    *Self,
                                   APTR                     Effect,
                                   struct AHIAudioCtrlDrv  *AudioCtrl)
{
    return (uint32)AHIS_UNKNOWN;
}


uint32 _usbaudio_AHIsub_LoadSound(struct USBAudioIFace    *Self,
                                   uint16                   Sound,
                                   uint32                   Type,
                                   APTR                     Info,
                                   struct AHIAudioCtrlDrv  *AudioCtrl)
{
    return (uint32)AHIS_UNKNOWN;
}


uint32 _usbaudio_AHIsub_UnloadSound(struct USBAudioIFace  *Self,
                                     uint16                 Sound,
                                     struct AHIAudioCtrlDrv *AudioCtrl)
{
    return (uint32)AHIS_UNKNOWN;
}
