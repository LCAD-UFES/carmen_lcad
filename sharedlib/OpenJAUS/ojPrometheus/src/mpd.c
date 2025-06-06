// File:		mpd.c
// Version:		0.0.1
// Written by:	Alberto F. De Souza (alberto@lcad.inf.ufes.br)
// Date:		03/11/2017

// MODULO COM IMPLEMENTACAO INCOMPLETA!!! Apenas o report de curvatura que foi implementado pois nao usamos o Motion Profile Driver na IARA
#include <jaus.h>
#include <openJaus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
#include <torc.h>
#include "mpd.h"

#define CONTROLLER_STATUS_TIMEOUT_SEC 	1.5
#define CONTROLLER_STATUS_UPDATE_RATE_HZ		5.0
#define CONTROLLER_STATUS_QUEUE_SIZE			1
#define CONTROLLER_STATUS_PRESENCE_VECTOR	0

typedef struct
{
	ReportCurvatureMessage setCurvature;

	ReportCurvatureMessage reportCurvature;
	ReportComponentStatusMessage controllerStatus;
	int controllerSc;
}PdData;

extern double steering_angle;


void mpdProcessMessage(OjCmpt mpd, JausMessage message)
{
	ReportComponentStatusMessage reportComponentStatus;
	JausAddress address;
	PdData *data;

	data = (PdData*)ojCmptGetUserData(mpd);

	// This block of code is intended to reject commands from non-controlling components
	if(ojCmptHasController(mpd) && jausMessageIsRejectableCommand(message) )
	{
		address = ojCmptGetControllerAddress(mpd);
		if(!jausAddressEqual(message->source, address))
		{
			//jausAddressToString(message->source, buf);
			//cError("mpd: Received command message %s from non-controlling component (%s).\n", jausMessageCommandCodeString(message), buf);
			jausAddressDestroy(address);
			return;
		}
		jausAddressDestroy(address);
	}

	switch(message->commandCode) // Switch the processing algorithm according to the JAUS message type
	{
		case JAUS_REPORT_COMPONENT_STATUS:
			reportComponentStatus = reportComponentStatusMessageFromJausMessage(message);
			if(reportComponentStatus)
			{
				address = ojCmptGetControllerAddress(mpd);
				if(jausAddressEqual(reportComponentStatus->source, address))
				{
					reportComponentStatusMessageDestroy(data->controllerStatus);
					data->controllerStatus = reportComponentStatus;
				}
				else
				{
					reportComponentStatusMessageDestroy(reportComponentStatus);
				}
				jausAddressDestroy(address);
			}
			break;

		default:
			ojCmptDefaultMessageProcessor(mpd, message);
			break;
	}
}

void mpdSendReportCurvature(OjCmpt mpd)
{
	PdData *data;
	JausMessage txMessage;
	ServiceConnection scList;
	ServiceConnection sc;

	data = (PdData*)ojCmptGetUserData(mpd);

	scList = ojCmptGetScSendList(mpd, JAUS_REPORT_CURVATURE);
	sc = scList;
	while(sc)
	{
		jausAddressCopy(data->reportCurvature->destination, sc->address);
		data->reportCurvature->sequenceNumber = sc->sequenceNumber;
		data->reportCurvature->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;

		data->reportCurvature->atanOfCurrentCurvature = data->setCurvature->atanOfCurrentCurvature;
		*data->reportCurvature->timeStamp = *data->setCurvature->timeStamp;

		txMessage = reportCurvatureMessageToJausMessage(data->reportCurvature);
		ojCmptSendMessage(mpd, txMessage);
		jausMessageDestroy(txMessage);

		sc = sc->nextSc;
	}

	ojCmptDestroySendList(scList);
}

void mpdStandbyState(OjCmpt mpd)
{
	PdData *data;

//	static int count = 0;
//	mvprintw(15,0,"Standby %d", count++);
//	refresh();

	data = (PdData*)ojCmptGetUserData(mpd);

	mpdSendReportCurvature(mpd);

	if(TRUE)//	vehicleSimGetState() == VEHICLE_SIM_READY_STATE	)
	{
		ojCmptSetState(mpd, JAUS_READY_STATE);
	}
	else
	{
		if(data->controllerSc > -1)
		{
			ojCmptTerminateSc(mpd, data->controllerSc);
			data->controllerSc = -1;
		}
		data->controllerStatus->primaryStatusCode = JAUS_UNKNOWN_STATE;
	}
}

void mpdReadyState(OjCmpt mpd)
{
	PdData *data;
	JausAddress address;

//	static int count = 0;
//	mvprintw(15,0,"Ready %d", count++);
//	refresh();

	data = (PdData*)ojCmptGetUserData(mpd);

	if(FALSE)//	vehicleSimGetState() != VEHICLE_SIM_READY_STATE )
	{
		ojCmptSetState(mpd, JAUS_STANDBY_STATE);
		return;
	}

	if(ojCmptHasController(mpd))
	{
		if(data->controllerSc == -1)
		{
			address = ojCmptGetControllerAddress(mpd);
			data->controllerSc = ojCmptEstablishSc(mpd,
													JAUS_REPORT_COMPONENT_STATUS,
													CONTROLLER_STATUS_PRESENCE_VECTOR,
													address,
													CONTROLLER_STATUS_UPDATE_RATE_HZ,
													CONTROLLER_STATUS_TIMEOUT_SEC,
													CONTROLLER_STATUS_QUEUE_SIZE);
			jausAddressDestroy(address);
		}

		if(ojCmptIsIncomingScActive(mpd, data->controllerSc))
		{
//			if(data->controllerStatus->primaryStatusCode == JAUS_READY_STATE || data->controllerStatus->primaryStatusCode == JAUS_STANDBY_STATE)
//			{
//				if(vehicleSimGetRunPause() == VEHICLE_SIM_RUN)
//				{
//					vehicleSimSetCommand(	data->setWrenchEffort->propulsiveLinearEffortXPercent,
//											data->setWrenchEffort->resistiveLinearEffortXPercent,
//											data->setWrenchEffort->propulsiveRotationalEffortZPercent
//										);
//				}
//				else
//				{
//					vehicleSimSetCommand(0, 80, data->setWrenchEffort->propulsiveRotationalEffortZPercent);
//				}
//			}
//			else
//			{
//				vehicleSimSetCommand(0, 80, 0);
//			}
		}
		else
		{
//			vehicleSimSetCommand(0, 80, 0);
		}
	}
	else
	{
		if(data->controllerSc > -1)
		{
			ojCmptTerminateSc(mpd, data->controllerSc);
			data->controllerSc = -1;
		}

		data->controllerStatus->primaryStatusCode = JAUS_UNKNOWN_STATE;

//		vehicleSimSetCommand(0, 80, 0);
	}

	// Ler estado da IARA e reporta-los aqui
	data->setCurvature->atanOfCurrentCurvature = steering_angle;
//	data->setCurvature->timeStamp = data->setCurvature->timeStamp;
	mpdSendReportCurvature(mpd);
}

OjCmpt mpdCreate(void)
{
	OjCmpt cmpt;
	PdData *data;
	JausAddress mpdAddr;

	cmpt = ojCmptCreate("mpd", JAUS_MOTION_PROFILE_DRIVER, MPD_THREAD_DESIRED_RATE_HZ);

	if (!cmpt)
	{
		printf("Could not create motion profile driver. Is ojNodeManager running?\n");
		return (NULL);
	}

	ojCmptAddService(cmpt, JAUS_MOTION_PROFILE_DRIVER);

	ojCmptAddServiceOutputMessage(cmpt, JAUS_MOTION_PROFILE_DRIVER, JAUS_REPORT_CURVATURE, 0xFF);

	ojCmptAddSupportedSc(cmpt, JAUS_REPORT_CURVATURE);

	ojCmptSetMessageProcessorCallback(cmpt, mpdProcessMessage);
	ojCmptSetStateCallback(cmpt, JAUS_STANDBY_STATE, mpdStandbyState);
	ojCmptSetStateCallback(cmpt, JAUS_READY_STATE, mpdReadyState);
	ojCmptSetState(cmpt, JAUS_READY_STATE);

	data = (PdData*)malloc(sizeof(PdData));
	data->setCurvature = reportCurvatureMessageCreate();
	data->reportCurvature = reportCurvatureMessageCreate();

	data->controllerStatus = reportComponentStatusMessageCreate();
	data->controllerSc = -1;

	mpdAddr = ojCmptGetAddress(cmpt);
	jausAddressCopy(data->reportCurvature->source, mpdAddr);
	jausAddressDestroy(mpdAddr);

	ojCmptSetUserData(cmpt, (void *)data);

	if (ojCmptRun(cmpt))
	{
		ojCmptDestroy(cmpt);
		return NULL;
	}

	return cmpt;
}

void mpdDestroy(OjCmpt mpd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(mpd);

	if(ojCmptIsIncomingScActive(mpd, data->controllerSc))
	{
		ojCmptTerminateSc(mpd, data->controllerSc);
	}
	ojCmptRemoveSupportedSc(mpd, JAUS_REPORT_CURVATURE);
	ojCmptDestroy(mpd);

	reportCurvatureMessageDestroy(data->setCurvature);
	reportCurvatureMessageDestroy(data->reportCurvature);
	reportComponentStatusMessageDestroy(data->controllerStatus);
	free(data);
}

// The series of functions below allow public access to essential component information
// Access:		Public (All)
JausBoolean mpdGetControllerScStatus(OjCmpt mpd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(mpd);

	return ojCmptIsIncomingScActive(mpd, data->controllerSc);
}

JausState mpdGetControllerState(OjCmpt mpd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(mpd);

	return data->controllerStatus->primaryStatusCode;
}

ReportCurvatureMessage mpdGetCurvature(OjCmpt mpd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(mpd);

	return data->reportCurvature;
}
