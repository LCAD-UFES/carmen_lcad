/*****************************************************************************
 *  Copyright (c) 2009, OpenJAUS.com
 *  All rights reserved.
 *
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD
 *  license.  See the LICENSE file for details.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File:		em.c
// Version:		3.3.0b
// Written by:	Tom Galluzzo (galluzzt@ufl.edu) and Danny Kent (kentd@ufl.edu)
// Date:		09/08/09
// Description:	This file contains the skeleton C code for implementing a JAUS component in a Linux environment
//				This code is designed to work with the node manager and JAUS library software written by CIMAR

#include <jaus.h>			// JAUS message set (USER: JAUS libraries must be installed first)
#include <openJaus.h>	// Node managment functions for sending and receiving JAUS messages (USER: Node Manager must be installed)
#include <stdlib.h>
#include <ncurses.h>
#include <termios.h>
#include <torc.h>
#include <unistd.h>
#include "em.h"	// USER: Implement and rename this header file. Include prototypes for all public functions contained in this file.

extern int error_to_report;


// Private function prototypes
void emReadyState(OjCmpt em);


OjCmpt emCreate(void)
{
	OjCmpt cmpt;
	ReportErrorCountMessage message;
	JausAddress emAddr;

	cmpt = ojCmptCreate("em", JAUS_ERROR_MANAGER, EM_THREAD_DESIRED_RATE_HZ);

	ojCmptAddService(cmpt, JAUS_ERROR_MANAGER);
	ojCmptAddServiceOutputMessage(cmpt, JAUS_ERROR_MANAGER, JAUS_REPORT_ERROR_COUNT, 0xFF);

	ojCmptSetStateCallback(cmpt, JAUS_READY_STATE, emReadyState);
	ojCmptAddSupportedSc(cmpt, JAUS_REPORT_ERROR_COUNT);

	message = reportErrorCountMessageCreate();
	emAddr = ojCmptGetAddress(cmpt);
	jausAddressCopy(message->source, emAddr);
	jausAddressDestroy(emAddr);

	ojCmptSetUserData(cmpt, (void *)message);

	ojCmptSetState(cmpt, JAUS_READY_STATE);

	if(ojCmptRun(cmpt))
	{
		ojCmptDestroy(cmpt);
		return NULL;
	}

	return cmpt;
}

void emDestroy(OjCmpt em)
{
	ReportErrorCountMessage message;

	message = (ReportErrorCountMessage) ojCmptGetUserData(em);

	ojCmptRemoveSupportedSc(em, JAUS_REPORT_ERROR_COUNT);
	ojCmptDestroy(em);

	reportErrorCountMessageDestroy(message);
}


void emReadyState(OjCmpt em)
{
	JausMessage txMessage;
	ServiceConnection scList;
	ServiceConnection sc;

	ReportErrorCountMessage message;

//	static int count = 0;
//	mvprintw(15,0,"Ready %d", count++);
//	refresh();

	message = (ReportErrorCountMessage) ojCmptGetUserData(em);

	if (error_to_report != 0)
	{
		message->numberOfErrors = 1;
		message->error[0] = error_to_report;
	}
	else
		message->numberOfErrors = 0;

	// send message
	if (ojCmptIsOutgoingScActive(em, JAUS_REPORT_ERROR_COUNT))
	{
		scList = ojCmptGetScSendList(em, JAUS_REPORT_ERROR_COUNT);
		sc = scList;
		while (sc)
		{
			jausAddressCopy(message->destination, sc->address);
			message->sequenceNumber = sc->sequenceNumber;
			message->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;

			txMessage = reportErrorCountMessageToJausMessage(message);
			ojCmptSendMessage(em, txMessage);
			jausMessageDestroy(txMessage);

			sc = sc->nextSc;
		}

		ojCmptDestroySendList(scList);
	}
}
