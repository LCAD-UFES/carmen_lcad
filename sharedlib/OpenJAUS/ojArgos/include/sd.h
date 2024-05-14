// File:		sd.h
// Version:		0.0.1
// Written by:	Alberto F. De Souza (alberto@lcad.inf.ufes.br)
// Date:		04/11/2017
// Description:	This file contains the header file code for the mpd.c file

#ifndef SD_H
#define SD_H

#include <jaus.h>
#include <openJaus.h>

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#define SD_THREAD_DESIRED_RATE_HZ			10.0

extern int out_can_sockfd;

OjCmpt sdCreate(void);
void sdDestroy(OjCmpt pd);

// USER: Insert prototypes for added public function here
JausBoolean sdGetControllerScStatus(OjCmpt pd);
JausState sdGetControllerState(OjCmpt pd);
//SetWrenchEffortMessage pdGetWrenchEffort(OjCmpt pd);
//SetDiscreteDevicesMessage pdGetDiscreteDevices(OjCmpt pd);

#endif // SD_H
