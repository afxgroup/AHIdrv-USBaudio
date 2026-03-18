#ifndef PROTO_USBAUDIO_H
#define PROTO_USBAUDIO_H

/*
 * Proto header for usbaudio.audio AHI sub-driver.
 */

#ifndef EXEC_TYPES_H
#include <exec/types.h>
#endif
#ifndef LIBRARIES_AHI_SUB_H
#include <libraries/ahi_sub.h>
#endif

/****************************************************************************/

#ifndef __NOLIBBASE__
 extern struct Library *USBAudioBase;
#endif /* __NOLIBBASE__ */

/****************************************************************************/

#ifdef __amigaos4__
 #include <interfaces/usbaudio.h>
 #ifndef __NOGLOBALIFACE__
  extern struct USBAudioIFace *IUSBAudio;
 #endif /* __NOGLOBALIFACE__ */
#endif /* __amigaos4__ */

/****************************************************************************/

#endif /* PROTO_USBAUDIO_H */
