// File:		sd.c
// Version:		0.0.1
// Written by:	Alberto F. De Souza (alberto@lcad.inf.ufes.br)
// Date:		04/11/2017

#include <jaus.h>
#include <openJaus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
#include <torc.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "sd.h"
#include "can_utils.h"

#define CONTROLLER_STATUS_TIMEOUT_SEC 	1.5
#define CONTROLLER_STATUS_UPDATE_RATE_HZ		5.0
#define CONTROLLER_STATUS_QUEUE_SIZE			1
#define CONTROLLER_STATUS_PRESENCE_VECTOR	0

typedef struct
{
	SetSignalsMessage setSignals;

	ReportSignalsMessage reportSignals;
	ReportComponentStatusMessage controllerStatus;
	int controllerSc;
} PdData;

extern int turn_signal;


void sdProcessMessage(OjCmpt sd, JausMessage message)
{
	ReportComponentStatusMessage reportComponentStatus;
//	SetSignalsMessage setSignals;
	JausAddress address;
	PdData *data;

	data = (PdData*)ojCmptGetUserData(sd);

	// This block of code is intended to reject commands from non-controlling components
	if(ojCmptHasController(sd) && jausMessageIsRejectableCommand(message) )
	{
		address = ojCmptGetControllerAddress(sd);
		if(!jausAddressEqual(message->source, address))
		{
			//jausAddressToString(message->source, buf);
			//cError("sd: Received command message %s from non-controlling component (%s).\n", jausMessageCommandCodeString(message), buf);
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
				address = ojCmptGetControllerAddress(sd);
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

		case JAUS_SET_SIGNALS:
//			setSignals = setSignalsMessageFromJausMessage(message);
//			if(setSignals)
//			{
//				if ((1 << JAUS_SIGNALS_PV_TURN_SIGNAL_BIT) & setSignals->presenceVector)
//				{
//					data->setSignals->turnSignal = setSignals->turnSignal;
//
//					struct can_frame frame;
//					frame.can_id = 0x400;
//					frame.can_dlc = 2; // numero de bytes
//					frame.data[0] = data->setSignals->turnSignal << 5;
//					frame.data[1] = data->setSignals->hornStatus;
//					if (out_can_sockfd != -1)
//						send_frame(out_can_sockfd, &frame);
//				}
//				if ((1 << JAUS_SIGNALS_PV_HORN_BIT) & setSignals->presenceVector)
//				{
//					data->setSignals->hornPeriodOff = setSignals->hornPeriodOff;
//					data->setSignals->hornPeriodOn = setSignals->hornPeriodOn;
//					data->setSignals->hornStatus = setSignals->hornStatus;
//
//					struct can_frame frame;
//					frame.can_id = 0x400;
//					frame.can_dlc = 2; // numero de bytes
//					frame.data[0] = data->setSignals->turnSignal << 5;
//					frame.data[1] = data->setSignals->hornStatus;
//					if (out_can_sockfd != -1)
//						send_frame(out_can_sockfd, &frame);
//				}
//				if ((1 << JAUS_SIGNALS_PV_HEADLIGHTS_BIT) & setSignals->presenceVector)
//				{
//					data->setSignals->headlightsStatus = setSignals->headlightsStatus;
//					data->setSignals->lightsPeriodOff = setSignals->lightsPeriodOff;
//					data->setSignals->lightsPeriodOn = setSignals->lightsPeriodOn;
//					// Mandar comandos para a IARA aqui
//				}
//				setSignalsMessageDestroy(setSignals);
//			}
			break;

		default:
			ojCmptDefaultMessageProcessor(sd, message);
			break;
	}
}

void sdSendReportSignals(OjCmpt sd)
{
	PdData *data;
	JausMessage txMessage;
	ServiceConnection scList;
	ServiceConnection sc;

	data = (PdData*)ojCmptGetUserData(sd);

	scList = ojCmptGetScSendList(sd, JAUS_REPORT_SIGNALS);
	sc = scList;
	while (sc)
	{
		jausAddressCopy(data->reportSignals->destination, sc->address);
		data->reportSignals->presenceVector = sc->presenceVector;
		data->reportSignals->sequenceNumber = sc->sequenceNumber;
		data->reportSignals->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;

		data->reportSignals->headlightsStatus = data->setSignals->headlightsStatus;
		data->reportSignals->hornPeriodOff = data->setSignals->hornPeriodOff;
		data->reportSignals->hornPeriodOn = data->setSignals->hornPeriodOn;
		data->reportSignals->hornStatus = data->setSignals->hornStatus;
		data->reportSignals->lightsPeriodOff = data->setSignals->lightsPeriodOff;
		data->reportSignals->lightsPeriodOn = data->setSignals->lightsPeriodOn;

		data->reportSignals->turnSignal = (turn_signal >> 4) & 0x3;

		txMessage = reportSignalsMessageToJausMessage(data->reportSignals);
		ojCmptSendMessage(sd, txMessage);
		jausMessageDestroy(txMessage);

		sc = sc->nextSc;
	}

	ojCmptDestroySendList(scList);
}

void sdStandbyState(OjCmpt sd)
{
	PdData *data;

//	static int count = 0;
//	mvprintw(15,0,"Standby %d", count++);
//	refresh();

	data = (PdData*)ojCmptGetUserData(sd);

	sdSendReportSignals(sd);

	if(TRUE)//	vehicleSimGetState() == VEHICLE_SIM_READY_STATE	)
	{
		ojCmptSetState(sd, JAUS_READY_STATE);
	}
	else
	{
		if(data->controllerSc > -1)
		{
			ojCmptTerminateSc(sd, data->controllerSc);
			data->controllerSc = -1;
		}
		data->controllerStatus->primaryStatusCode = JAUS_UNKNOWN_STATE;
	}
}

void sdReadyState(OjCmpt sd)
{
	PdData *data;
	JausAddress address;

//	static int count = 0;
//	mvprintw(15,0,"Ready %d", count++);
//	refresh();

	data = (PdData*)ojCmptGetUserData(sd);

	sdSendReportSignals(sd);

	if(FALSE)//	vehicleSimGetState() != VEHICLE_SIM_READY_STATE )
	{
		ojCmptSetState(sd, JAUS_STANDBY_STATE);
		return;
	}

	if(ojCmptHasController(sd))
	{
		if(data->controllerSc == -1)
		{
			address = ojCmptGetControllerAddress(sd);
			data->controllerSc = ojCmptEstablishSc(	sd,
													JAUS_REPORT_COMPONENT_STATUS,
													CONTROLLER_STATUS_PRESENCE_VECTOR,
													address,
													CONTROLLER_STATUS_UPDATE_RATE_HZ,
													CONTROLLER_STATUS_TIMEOUT_SEC,
													CONTROLLER_STATUS_QUEUE_SIZE);
			jausAddressDestroy(address);
		}

		if(ojCmptIsIncomingScActive(sd, data->controllerSc))
		{
//			if(data->controllerStatus->primaryStatusCode == JAUS_READY_STATE || data->controllerStatus->primaryStatusCode == JAUS_STANDBY_STATE)
//			{
//				if(vehicleSimGetRunPause() == VEHICLE_SIM_RUN)
//				{
//					vehicleSimSetCommand(	data->setSignals->propulsiveLinearEffortXPercent,
//											data->setSignals->resistiveLinearEffortXPercent,
//											data->setSignals->propulsiveRotationalEffortZPercent
//										);
//				}
//				else
//				{
//					vehicleSimSetCommand(0, 80, data->setSignals->propulsiveRotationalEffortZPercent);
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
			ojCmptTerminateSc(sd, data->controllerSc);
			data->controllerSc = -1;
		}

		data->controllerStatus->primaryStatusCode = JAUS_UNKNOWN_STATE;

//		vehicleSimSetCommand(0, 80, 0);
	}
}

OjCmpt sdCreate(void)
{
	OjCmpt cmpt;
	PdData *data;
	JausAddress sdAddr;

	cmpt = ojCmptCreate("sd", JAUS_SIGNALS_DRIVER, SD_THREAD_DESIRED_RATE_HZ);

	if (!cmpt)
	{
		printf("Could not create signals driver. Is ojNodeManager running?\n");
		return (NULL);
	}

	ojCmptAddService(cmpt, JAUS_SIGNALS_DRIVER);

	ojCmptAddServiceInputMessage(cmpt, JAUS_SIGNALS_DRIVER, JAUS_SET_SIGNALS, 0xFF);

	ojCmptAddServiceOutputMessage(cmpt, JAUS_SIGNALS_DRIVER, JAUS_REPORT_SIGNALS, 0xFF);

	ojCmptAddSupportedSc(cmpt, JAUS_REPORT_SIGNALS);

	ojCmptSetMessageProcessorCallback(cmpt, sdProcessMessage);
	ojCmptSetStateCallback(cmpt, JAUS_STANDBY_STATE, sdStandbyState);
	ojCmptSetStateCallback(cmpt, JAUS_READY_STATE, sdReadyState);
	ojCmptSetState(cmpt, JAUS_READY_STATE);

	data = (PdData*)malloc(sizeof(PdData));
	data->setSignals = setSignalsMessageCreate();

	data->reportSignals = reportSignalsMessageCreate();

	data->controllerStatus = reportComponentStatusMessageCreate();
	data->controllerSc = -1;

	sdAddr = ojCmptGetAddress(cmpt);
	jausAddressCopy(data->reportSignals->source, sdAddr);
	jausAddressDestroy(sdAddr);

	ojCmptSetUserData(cmpt, (void *)data);

	if (ojCmptRun(cmpt))
	{
		ojCmptDestroy(cmpt);
		return NULL;
	}

	return cmpt;
}

void sdDestroy(OjCmpt sd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(sd);

	if(ojCmptIsIncomingScActive(sd, data->controllerSc))
	{
		ojCmptTerminateSc(sd, data->controllerSc);
	}
	ojCmptRemoveSupportedSc(sd, JAUS_REPORT_SIGNALS);
	ojCmptDestroy(sd);

	setSignalsMessageDestroy(data->setSignals);

	reportSignalsMessageDestroy(data->reportSignals);

	reportComponentStatusMessageDestroy(data->controllerStatus);

	free(data);
}

JausBoolean sdGetControllerScStatus(OjCmpt sd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(sd);

	return ojCmptIsIncomingScActive(sd, data->controllerSc);
}

JausState sdGetControllerState(OjCmpt sd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(sd);

	return data->controllerStatus->primaryStatusCode;
}

SetSignalsMessage sdGetSignals(OjCmpt sd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(sd);

	return data->setSignals;
}
