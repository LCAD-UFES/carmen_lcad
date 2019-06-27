// File:		mpd.h
// Version:		0.0.1
// Written by:	Alberto F. De Souza (alberto@lcad.inf.ufes.br)
// Date:		03/11/2017
// Description:	This file contains the header file code for the mpd.c file

#ifndef MPD_H
#define MPD_H

#include <jaus.h>
#include <openJaus.h>

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#define MPD_THREAD_DESIRED_RATE_HZ			202

OjCmpt mpdCreate(void);
void mpdDestroy(OjCmpt mpd);

// USER: Insert prototypes for added public function here
JausBoolean mpdGetControllerScStatus(OjCmpt mpd);
JausState mpdGetControllerState(OjCmpt mpd);
ReportCurvatureMessage mpdGetCurvature(OjCmpt mpd);

#endif // MPD_H
