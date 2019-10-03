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
// File:		pd.c
// Version:		3.3.0b
// Written by:	Tom Galluzzo (galluzzt@ufl.edu) and Danny Kent (kentd@ufl.edu)
// Date:		09/08/09

#include <jaus.h>
#include <openJaus.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
#include <torc.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "pd.h"
#include "can_utils.h"

#define CONTROLLER_STATUS_TIMEOUT_SEC 	1.5
#define CONTROLLER_STATUS_UPDATE_RATE_HZ		5.0
#define CONTROLLER_STATUS_QUEUE_SIZE			1
#define CONTROLLER_STATUS_PRESENCE_VECTOR	0

typedef struct
{
	SetWrenchEffortMessage setWrenchEffort;
	SetDiscreteDevicesMessage setDiscreteDevices;
	ReportWheelsSpeedMessage setWheelsSpeed; // O nome eh set... mas as velocidades sao apenas reportadas. Esta escolha nome nome foi feita para seguir o padrao de outras mensagens do pd

	ReportWrenchEffortMessage reportWrenchEffort;
	ReportDiscreteDevicesMessage reportDiscreteDevices;
	ReportWheelsSpeedMessage reportWheelsSpeed;
	ReportComponentStatusMessage reportComponentStatus;

	ReportComponentStatusMessage controllerStatus;
	int controllerSc;
} PdData;

extern double front_left_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
extern double front_right_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
extern double back_left_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
extern double back_right_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];

// ByWire XGV User Manual 1.5, pg. 62
#define XGV_MANUAL_OVERRIDE_FLAG 16
#define XGV_SAFE_STOP_FLAG 18
#define XGV_DOOR_OPEN_FLAG 24

extern unsigned int manual_override_and_safe_stop;
extern int door_signal;
extern int steering_wheel_zero_torque;

int gear_can_command = 0;

extern double g_break_effort;


double wheel_speed_moving_average(double *wheel_speed);


void send_gear(int gear_can_command)
{
	struct can_frame frame;
	frame.can_id = 0x405;
	frame.can_dlc = 1;
	frame.data[0] = gear_can_command;
	if (out_can_sockfd != -1)
		send_frame(out_can_sockfd, &frame);
}

void send_efforts(double throttle_effort, double breaks_effort, double steering_effort)
{
	struct can_frame frame;
	frame.can_id = 0x480;
	frame.can_dlc = 4;

	// Throttle
	if (throttle_effort > 100.0)
		throttle_effort = 100.0;
	else if (throttle_effort < 0.0)
		throttle_effort = 0.0;
	frame.data[0] = (int) (2.0 * throttle_effort + 0.5); // throttle

	// Breaks
	if (breaks_effort > 100.0)
		breaks_effort = 100.0;
	else if (breaks_effort < 0.0)
		breaks_effort = 0.0;
	frame.data[1] = (int) (2.0 * breaks_effort + 0.5); // breaks

	g_break_effort = breaks_effort;

	// Steering
	if (steering_effort > 100.0)
		steering_effort = 100.0;
	else if (steering_effort < -100.0)
		steering_effort = -100.0;
	int steering_byte0, steering_byte1;
	if (steering_effort > 0.0)
		steering_byte0 = (int) round(27.463 * pow(steering_effort, 0.332)); // Obtido examinando os dados enviados pelo Torc para o can
	else if (steering_effort < 0.0)
		steering_byte0 = (int) round(-27.463 * pow(-steering_effort, 0.332));
	else // effort == 0.0
		steering_byte0 = 0;

	if ((steering_byte0 + steering_wheel_zero_torque) < 0)
	{
		steering_byte0 += steering_wheel_zero_torque + 256;
		steering_byte1 = 0x01;
	}
	else
	{
		steering_byte0 += steering_wheel_zero_torque;
		steering_byte1 = 0x02;
	}

	frame.data[2] = steering_byte0; // Steering
	frame.data[3] = steering_byte1; // Steering

	if (out_can_sockfd != -1)
	{
		printf("sending efforts\n");
		send_frame(out_can_sockfd, &frame);
	}
}

void pdProcessMessage(OjCmpt pd, JausMessage message)
{
	ReportComponentStatusMessage reportComponentStatus;
	ReportPlatformSpecificationsMessage reportPlatformSpecifications;
	SetWrenchEffortMessage setWrenchEffort;
	SetDiscreteDevicesMessage setDiscreteDevices;
	QueryPlatformSpecificationsMessage queryPlatformSpecifications;
	JausAddress address;
	JausMessage txMessage;
	PdData *data;

	data = (PdData*)ojCmptGetUserData(pd);

	// This block of code is intended to reject commands from non-controlling components
	if(ojCmptHasController(pd) && jausMessageIsRejectableCommand(message) )
	{
		address = ojCmptGetControllerAddress(pd);
		if(!jausAddressEqual(message->source, address))
		{
			//jausAddressToString(message->source, buf);
			//cError("pd: Received command message %s from non-controlling component (%s).\n", jausMessageCommandCodeString(message), buf);
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
				address = ojCmptGetControllerAddress(pd);
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

		case JAUS_SET_WRENCH_EFFORT:
			setWrenchEffort = setWrenchEffortMessageFromJausMessage(message);
			if (setWrenchEffort)
			{
				if ((1 << JAUS_WRENCH_PV_PROPULSIVE_LINEAR_X_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->propulsiveLinearEffortXPercent = setWrenchEffort->propulsiveLinearEffortXPercent;
				if ((1 << JAUS_WRENCH_PV_PROPULSIVE_LINEAR_Y_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->propulsiveLinearEffortYPercent = setWrenchEffort->propulsiveLinearEffortYPercent;
				if ((1 << JAUS_WRENCH_PV_PROPULSIVE_LINEAR_Z_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->propulsiveLinearEffortZPercent = setWrenchEffort->propulsiveLinearEffortZPercent;

				if ((1 << JAUS_WRENCH_PV_PROPULSIVE_ROTATIONAL_X_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->propulsiveRotationalEffortXPercent = setWrenchEffort->propulsiveRotationalEffortXPercent;
				if ((1 << JAUS_WRENCH_PV_PROPULSIVE_ROTATIONAL_Y_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->propulsiveRotationalEffortYPercent = setWrenchEffort->propulsiveRotationalEffortYPercent;
				if ((1 << JAUS_WRENCH_PV_PROPULSIVE_ROTATIONAL_Z_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->propulsiveRotationalEffortZPercent = setWrenchEffort->propulsiveRotationalEffortZPercent;

				if ((1 << JAUS_WRENCH_PV_RESISTIVE_LINEAR_X_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->resistiveLinearEffortXPercent = setWrenchEffort->resistiveLinearEffortXPercent;
				if ((1 << JAUS_WRENCH_PV_RESISTIVE_LINEAR_Y_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->resistiveLinearEffortYPercent = setWrenchEffort->resistiveLinearEffortYPercent;
				if ((1 << JAUS_WRENCH_PV_RESISTIVE_LINEAR_Z_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->resistiveLinearEffortZPercent = setWrenchEffort->resistiveLinearEffortZPercent;

				if ((1 << JAUS_WRENCH_PV_RESISTIVE_ROTATIONAL_X_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->resistiveRotationalEffortXPercent = setWrenchEffort->resistiveRotationalEffortXPercent;
				if ((1 << JAUS_WRENCH_PV_RESISTIVE_ROTATIONAL_Y_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->resistiveRotationalEffortYPercent = setWrenchEffort->resistiveRotationalEffortYPercent;
				if ((1 << JAUS_WRENCH_PV_RESISTIVE_ROTATIONAL_Z_BIT) & setWrenchEffort->presenceVector)
					data->setWrenchEffort->resistiveRotationalEffortZPercent = setWrenchEffort->resistiveRotationalEffortZPercent;

				send_efforts(data->setWrenchEffort->propulsiveLinearEffortXPercent, data->setWrenchEffort->resistiveLinearEffortXPercent,
						data->setWrenchEffort->propulsiveRotationalEffortZPercent);

				setWrenchEffortMessageDestroy(setWrenchEffort);
			}
			break;

		case JAUS_SET_DISCRETE_DEVICES:
			setDiscreteDevices = setDiscreteDevicesMessageFromJausMessage(message);
			if(setDiscreteDevices)
			{
				if ((1 << JAUS_DEVICES_PV_PROPULSION_BIT) & setDiscreteDevices->presenceVector)
				{
					// TODO: @@@ Alberto: Ver pagina 66 do manual
					data->setDiscreteDevices->mainPropulsion = setDiscreteDevices->mainPropulsion;
					data->setDiscreteDevices->mainFuelSupply = setDiscreteDevices->mainFuelSupply;
					// Mandar comandos para a IARA aqui
 				}
				if ((1 << JAUS_DEVICES_PV_PARKING_BIT) & setDiscreteDevices->presenceVector)
				{
					// Nao usados pela Torc - Ver pagina 66 do manual. A horn eh controlada pelo Signals Drive (sd.c)
					data->setDiscreteDevices->parkingBrake = setDiscreteDevices->parkingBrake;
					data->setDiscreteDevices->horn = setDiscreteDevices->horn;
 				}
				if ((1 << JAUS_DEVICES_PV_GEAR_BIT) & setDiscreteDevices->presenceVector)
				{
					data->setDiscreteDevices->gear = setDiscreteDevices->gear;

					if (data->setDiscreteDevices->gear == 1) // Low
						gear_can_command = 0x04;
					else if (data->setDiscreteDevices->gear == 2) // Drive
						gear_can_command = 0x03;
					else if (data->setDiscreteDevices->gear == 128) // Neutral
						gear_can_command = 0x02;
					else if (data->setDiscreteDevices->gear == 129) // Reverse
						gear_can_command = 0x01;
 				}
				setDiscreteDevicesMessageDestroy(setDiscreteDevices);
			}
			break;

		case JAUS_QUERY_PLATFORM_SPECIFICATIONS:
			queryPlatformSpecifications = queryPlatformSpecificationsMessageFromJausMessage(message);
			if(queryPlatformSpecifications)
			{
				reportPlatformSpecifications = reportPlatformSpecificationsMessageCreate();

				jausAddressCopy(reportPlatformSpecifications->destination, queryPlatformSpecifications->source);
				jausAddressCopy(reportPlatformSpecifications->source, queryPlatformSpecifications->destination);

				reportPlatformSpecifications->maximumVelocityXMps = 10.0;

				txMessage = reportPlatformSpecificationsMessageToJausMessage(reportPlatformSpecifications);
				ojCmptSendMessage(pd, txMessage);
				jausMessageDestroy(txMessage);

				reportPlatformSpecificationsMessageDestroy(reportPlatformSpecifications);
				queryPlatformSpecificationsMessageDestroy(queryPlatformSpecifications);
			}
			break;

		default:
			ojCmptDefaultMessageProcessor(pd, message);
			break;
	}
}

void pdSendReportWrenchEffort(OjCmpt pd)
{
	PdData *data;
	JausMessage txMessage;
	ServiceConnection scList;
	ServiceConnection sc;

	data = (PdData*)ojCmptGetUserData(pd);

	scList = ojCmptGetScSendList(pd, JAUS_REPORT_WRENCH_EFFORT);
	sc = scList;
	while(sc)
	{
		jausAddressCopy(data->reportWrenchEffort->destination, sc->address);
		data->reportWrenchEffort->presenceVector = sc->presenceVector;
		data->reportWrenchEffort->sequenceNumber = sc->sequenceNumber;
		data->reportWrenchEffort->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;

		data->reportWrenchEffort->propulsiveLinearEffortXPercent = data->setWrenchEffort->propulsiveLinearEffortXPercent;
		data->reportWrenchEffort->propulsiveLinearEffortYPercent = data->setWrenchEffort->propulsiveLinearEffortYPercent;
		data->reportWrenchEffort->propulsiveLinearEffortZPercent = data->setWrenchEffort->propulsiveLinearEffortZPercent;
		data->reportWrenchEffort->propulsiveRotationalEffortXPercent = data->setWrenchEffort->propulsiveRotationalEffortXPercent;
		data->reportWrenchEffort->propulsiveRotationalEffortYPercent = data->setWrenchEffort->propulsiveRotationalEffortYPercent;
		data->reportWrenchEffort->propulsiveRotationalEffortZPercent = data->setWrenchEffort->propulsiveRotationalEffortZPercent;
		data->reportWrenchEffort->resistiveLinearEffortXPercent = data->setWrenchEffort->resistiveLinearEffortXPercent;
		data->reportWrenchEffort->resistiveLinearEffortYPercent = data->setWrenchEffort->resistiveLinearEffortYPercent;
		data->reportWrenchEffort->resistiveLinearEffortZPercent = data->setWrenchEffort->resistiveLinearEffortXPercent;
		data->reportWrenchEffort->resistiveRotationalEffortXPercent = data->setWrenchEffort->resistiveRotationalEffortXPercent;
		data->reportWrenchEffort->resistiveRotationalEffortYPercent = data->setWrenchEffort->resistiveRotationalEffortYPercent;
		data->reportWrenchEffort->resistiveRotationalEffortZPercent = data->setWrenchEffort->resistiveRotationalEffortZPercent;

		txMessage = reportWrenchEffortMessageToJausMessage(data->reportWrenchEffort);
		ojCmptSendMessage(pd, txMessage);
		jausMessageDestroy(txMessage);

		sc = sc->nextSc;
	}

	ojCmptDestroySendList(scList);
}

void pdSendReportDiscreteDevices(OjCmpt pd)
{
	PdData *data;
	JausMessage txMessage;
	ServiceConnection scList;
	ServiceConnection sc;

	data = (PdData*)ojCmptGetUserData(pd);

	scList = ojCmptGetScSendList(pd, JAUS_REPORT_DISCRETE_DEVICES);
	sc = scList;
	while (sc)
	{
		jausAddressCopy(data->reportDiscreteDevices->destination, sc->address);
		data->reportDiscreteDevices->presenceVector = sc->presenceVector;
		data->reportDiscreteDevices->sequenceNumber = sc->sequenceNumber;
		data->reportDiscreteDevices->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;

		data->reportDiscreteDevices->mainPropulsion = data->setDiscreteDevices->mainPropulsion;
		data->reportDiscreteDevices->mainFuelSupply = data->setDiscreteDevices->mainFuelSupply;
		data->reportDiscreteDevices->horn = data->setDiscreteDevices->horn;
		data->reportDiscreteDevices->parkingBrake = data->setDiscreteDevices->parkingBrake;
		data->reportDiscreteDevices->gear = data->setDiscreteDevices->gear;

		txMessage = reportDiscreteDevicesMessageToJausMessage(data->reportDiscreteDevices);
		ojCmptSendMessage(pd, txMessage);
		jausMessageDestroy(txMessage);

		sc = sc->nextSc;
	}

	ojCmptDestroySendList(scList);
}

void pdSendReportWheelsSpeed(OjCmpt pd)
{
	PdData *data;
	JausMessage txMessage;
	ServiceConnection scList;
	ServiceConnection sc;

	data = (PdData*)ojCmptGetUserData(pd);

	scList = ojCmptGetScSendList(pd, JAUS_REPORT_WHEELS_SPEED);
	sc = scList;
	while (sc)
	{
		jausAddressCopy(data->reportWheelsSpeed->destination, sc->address);
		data->reportWheelsSpeed->sequenceNumber = sc->sequenceNumber;
		data->reportWheelsSpeed->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;

		data->reportWheelsSpeed->leftFront = data->setWheelsSpeed->leftFront;
		data->reportWheelsSpeed->leftRear = data->setWheelsSpeed->leftRear;
		data->reportWheelsSpeed->rightFront = data->setWheelsSpeed->rightFront;
		data->reportWheelsSpeed->rightRear = data->setWheelsSpeed->rightRear;
		*data->reportWheelsSpeed->timeStamp = *data->setWheelsSpeed->timeStamp;

		txMessage = reportWheelsSpeedMessageToJausMessage(data->reportWheelsSpeed);
		ojCmptSendMessage(pd, txMessage);
		jausMessageDestroy(txMessage);

		sc = sc->nextSc;
	}

	ojCmptDestroySendList(scList);
}

void pdSendReportComponentStatus(OjCmpt pd)
{
	PdData *data;
	JausMessage txMessage;
	ServiceConnection scList;
	ServiceConnection sc;

	data = (PdData*)ojCmptGetUserData(pd);

	scList = ojCmptGetScSendList(pd, JAUS_REPORT_COMPONENT_STATUS);
	sc = scList;
	while(sc)
	{
		jausAddressCopy(data->reportComponentStatus->destination, sc->address);
		data->reportComponentStatus->sequenceNumber = sc->sequenceNumber;
		data->reportComponentStatus->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;

		data->reportComponentStatus->primaryStatusCode = data->controllerStatus->primaryStatusCode;
		data->reportComponentStatus->secondaryStatusCode = data->controllerStatus->secondaryStatusCode;

		txMessage = reportComponentStatusMessageToJausMessage(data->reportComponentStatus);
		ojCmptSendMessage(pd, txMessage);
		jausMessageDestroy(txMessage);

		sc = sc->nextSc;
	}

	ojCmptDestroySendList(scList);
}

void pdStandbyState(OjCmpt pd)
{
	PdData *data;

//	static int count = 0;
//	mvprintw(15,0,"Standby %d", count++);
//	refresh();

	data = (PdData*)ojCmptGetUserData(pd);

	pdSendReportWrenchEffort(pd);
	pdSendReportDiscreteDevices(pd);

	if(TRUE)//	vehicleSimGetState() == VEHICLE_SIM_READY_STATE	)
	{
		ojCmptSetState(pd, JAUS_READY_STATE);
	}
	else
	{
		if(data->controllerSc > -1)
		{
			ojCmptTerminateSc(pd, data->controllerSc);
			data->controllerSc = -1;
		}
		data->controllerStatus->primaryStatusCode = JAUS_UNKNOWN_STATE;
	}
}

void pdReadyState(OjCmpt pd)
{
	PdData *data;
	JausAddress address;

//	static int count = 0;
//	mvprintw(15,0,"Ready %d", count++);
//	refresh();

	data = (PdData*)ojCmptGetUserData(pd);

	pdSendReportWrenchEffort(pd);
	pdSendReportDiscreteDevices(pd);

	if(FALSE)//	vehicleSimGetState() != VEHICLE_SIM_READY_STATE )
	{
		ojCmptSetState(pd, JAUS_STANDBY_STATE);
		return;
	}

	if(ojCmptHasController(pd))
	{
		if(data->controllerSc == -1)
		{
			address = ojCmptGetControllerAddress(pd);
			data->controllerSc = ojCmptEstablishSc(	pd,
													JAUS_REPORT_COMPONENT_STATUS,
													CONTROLLER_STATUS_PRESENCE_VECTOR,
													address,
													CONTROLLER_STATUS_UPDATE_RATE_HZ,
													CONTROLLER_STATUS_TIMEOUT_SEC,
													CONTROLLER_STATUS_QUEUE_SIZE);
			jausAddressDestroy(address);
		}

		if(ojCmptIsIncomingScActive(pd, data->controllerSc))
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
			ojCmptTerminateSc(pd, data->controllerSc);
			data->controllerSc = -1;
		}

		data->controllerStatus->primaryStatusCode = JAUS_UNKNOWN_STATE;

//		vehicleSimSetCommand(0, 80, 0);
	}

	data->setWheelsSpeed->leftFront = wheel_speed_moving_average(front_left_speed);
	data->setWheelsSpeed->leftRear = wheel_speed_moving_average(back_left_speed);
	data->setWheelsSpeed->rightFront = wheel_speed_moving_average(front_right_speed);
	data->setWheelsSpeed->rightRear = wheel_speed_moving_average(back_right_speed);
//	data->setWheelsSpeed->timeStamp = time_last_odometry;
	pdSendReportWheelsSpeed(pd);

	data->reportComponentStatus->primaryStatusCode = data->controllerStatus->primaryStatusCode;

	unsigned int ssc = (((manual_override_and_safe_stop & 0x02) >> 1) << XGV_MANUAL_OVERRIDE_FLAG) |
					   (((manual_override_and_safe_stop & 0x01)) << XGV_SAFE_STOP_FLAG) |
					   (((door_signal & 0x80) || (door_signal & 0x100) || (door_signal & 0x200) || (door_signal & 0x400)) << XGV_DOOR_OPEN_FLAG);
	data->controllerStatus->secondaryStatusCode = ssc;
	data->reportComponentStatus->secondaryStatusCode = data->controllerStatus->secondaryStatusCode;
	pdSendReportComponentStatus(pd);

	if (gear_can_command)
		send_gear(gear_can_command);
}

OjCmpt pdCreate(void)
{
	OjCmpt cmpt;
	PdData *data;
	JausAddress pdAddr;

	cmpt = ojCmptCreate("pd", JAUS_PRIMITIVE_DRIVER, PD_THREAD_DESIRED_RATE_HZ);

	if (!cmpt)
	{
		printf("Could not create primitive driver. Is ojNodeManager running?\n");
		return (NULL);
	}

	ojCmptAddService(cmpt, JAUS_PRIMITIVE_DRIVER);

	ojCmptAddServiceInputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_WRENCH_EFFORT, 0xFF);
	ojCmptAddServiceInputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DISCRETE_DEVICES, 0xFF);
	ojCmptAddServiceInputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_QUERY_PLATFORM_SPECIFICATIONS, 0xFF);
	ojCmptAddServiceInputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_QUERY_WRENCH_EFFORT, 0xFF);

	ojCmptAddServiceOutputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_PLATFORM_SPECIFICATIONS, 0xFF);
	ojCmptAddServiceOutputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
	ojCmptAddServiceOutputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
	ojCmptAddServiceOutputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WHEELS_SPEED, 0xFF);
	ojCmptAddServiceOutputMessage(cmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_COMPONENT_STATUS, 0xFF);

	ojCmptAddSupportedSc(cmpt, JAUS_REPORT_WRENCH_EFFORT);
	ojCmptAddSupportedSc(cmpt, JAUS_REPORT_DISCRETE_DEVICES);
	ojCmptAddSupportedSc(cmpt, JAUS_REPORT_WHEELS_SPEED);
	ojCmptAddSupportedSc(cmpt, JAUS_REPORT_COMPONENT_STATUS);

	ojCmptSetMessageProcessorCallback(cmpt, pdProcessMessage);
	ojCmptSetStateCallback(cmpt, JAUS_STANDBY_STATE, pdStandbyState);
	ojCmptSetStateCallback(cmpt, JAUS_READY_STATE, pdReadyState);
	ojCmptSetState(cmpt, JAUS_READY_STATE);

	data = (PdData*)malloc(sizeof(PdData));
	data->setWrenchEffort = setWrenchEffortMessageCreate();
	data->setDiscreteDevices = setDiscreteDevicesMessageCreate();
	data->setWheelsSpeed = reportWheelsSpeedMessageCreate();

	data->reportWrenchEffort = reportWrenchEffortMessageCreate();
	data->reportDiscreteDevices = reportDiscreteDevicesMessageCreate();
	data->reportWheelsSpeed = reportWheelsSpeedMessageCreate();
	data->reportComponentStatus = reportComponentStatusMessageCreate();

	data->controllerStatus = reportComponentStatusMessageCreate();
	data->controllerSc = -1;

	pdAddr = ojCmptGetAddress(cmpt);
	jausAddressCopy(data->reportWrenchEffort->source, pdAddr);
	jausAddressCopy(data->reportDiscreteDevices->source, pdAddr);
	jausAddressCopy(data->reportWheelsSpeed->source, pdAddr);
	jausAddressCopy(data->reportComponentStatus->source, pdAddr);
	jausAddressDestroy(pdAddr);

	ojCmptSetUserData(cmpt, (void *)data);

	if (ojCmptRun(cmpt))
	{
		ojCmptDestroy(cmpt);
		return NULL;
	}

	return cmpt;
}

void pdDestroy(OjCmpt pd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(pd);

	if(ojCmptIsIncomingScActive(pd, data->controllerSc))
	{
		ojCmptTerminateSc(pd, data->controllerSc);
	}
	ojCmptRemoveSupportedSc(pd, JAUS_REPORT_WRENCH_EFFORT);
	ojCmptRemoveSupportedSc(pd, JAUS_REPORT_DISCRETE_DEVICES);
	ojCmptRemoveSupportedSc(pd, JAUS_REPORT_WHEELS_SPEED);
	ojCmptRemoveSupportedSc(pd, JAUS_REPORT_COMPONENT_STATUS);
	ojCmptDestroy(pd);

	setWrenchEffortMessageDestroy(data->setWrenchEffort);
	setDiscreteDevicesMessageDestroy(data->setDiscreteDevices);
	reportWheelsSpeedMessageDestroy(data->setWheelsSpeed);

	reportWrenchEffortMessageDestroy(data->reportWrenchEffort);
	reportDiscreteDevicesMessageDestroy(data->reportDiscreteDevices);
	reportWheelsSpeedMessageDestroy(data->reportWheelsSpeed);
	reportComponentStatusMessageDestroy(data->reportComponentStatus);

	reportComponentStatusMessageDestroy(data->controllerStatus);
	free(data);
}

// The series of functions below allow public access to essential component information
// Access:		Public (All)
JausBoolean pdGetControllerScStatus(OjCmpt pd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(pd);

	return ojCmptIsIncomingScActive(pd, data->controllerSc);
}

JausState pdGetControllerState(OjCmpt pd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(pd);

	return data->controllerStatus->primaryStatusCode;
}

SetWrenchEffortMessage pdGetWrenchEffort(OjCmpt pd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(pd);

	return data->setWrenchEffort;
}

SetDiscreteDevicesMessage pdGetDiscreteDevices(OjCmpt pd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(pd);

	return data->setDiscreteDevices;
}

ReportWheelsSpeedMessage pdGetWheelsSpeed(OjCmpt pd)
{
	PdData *data;

	data = (PdData*)ojCmptGetUserData(pd);

	return data->reportWheelsSpeed;
}
