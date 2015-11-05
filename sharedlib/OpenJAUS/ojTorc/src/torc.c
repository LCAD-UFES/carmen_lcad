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
// File:		torc.c
// Version:		0.0.1
// Written by:		Alberto F. Souza (alberto@lcad.inf.ufes.br)
// Date:		02/16/2012

#include <stdlib.h>
#include <jaus.h>
#include <openJaus.h>
#include "torc.h"


void 
velocity_state_message_handler(OjCmpt XGV_CCU, JausMessage velocity_message)
{
	ReportVelocityStateMessage reportVelocityState;
	
	reportVelocityState = reportVelocityStateMessageFromJausMessage(velocity_message);
	if (reportVelocityState)
	{
		if (g_XGV_velocity != reportVelocityState->velocityXMps)
		{
			g_XGV_velocity = reportVelocityState->velocityXMps;
			print_interface();
		}
		
		/*		printf("commandCode = 0x%x  ", reportVelocityState->commandCode);
		 *		printf("destination = %d.%d.%d.%d\r", 	reportVelocityState->destination->subsystem, 
		 *							reportVelocityState->destination->node, 
		 *							reportVelocityState->destination->component, 
		 *							reportVelocityState->destination->instance);
		 */
		reportVelocityStateMessageDestroy(reportVelocityState);
	}
	else
	{
		////cError("vss: Error unpacking %s message.\n", jausMessageCommandCodeString(message));
	}
}


void 
report_curvature_message_handler(OjCmpt XGV_CCU, JausMessage curvature_message)
{
	ReportCurvatureMessage reportCurvature;
	
	reportCurvature = reportCurvatureMessageFromJausMessage(curvature_message);
	if (reportCurvature)
	{
		if (g_XGV_atan_curvature != reportCurvature->atanOfCurrentCurvature)
		{
			g_XGV_atan_curvature = reportCurvature->atanOfCurrentCurvature;
			print_interface();
		}
		
		/*		printf("commandCode = 0x%x  ", reportVelocityState->commandCode);
		 *		printf("destination = %d.%d.%d.%d\r", 	reportVelocityState->destination->subsystem, 
		 *							reportVelocityState->destination->node, 
		 *							reportVelocityState->destination->component, 
		 *							reportVelocityState->destination->instance);
		 */
		reportCurvatureMessageDestroy(reportCurvature);
	}
	else
	{
		////cError("vss: Error unpacking %s message.\n", jausMessageCommandCodeString(message));
	}
}


void 
report_wheels_speed_message_handler(OjCmpt XGV_CCU, JausMessage wheels_speed_message)
{
	ReportWheelsSpeedMessage reportWheelsSpeed;
	
	reportWheelsSpeed = reportWheelsSpeedMessageFromJausMessage(wheels_speed_message);
	if (reportWheelsSpeed)
	{
		if (g_XGV_right_front_wheel_speed != reportWheelsSpeed->rightFront)
		{
			g_XGV_right_front_wheel_speed = reportWheelsSpeed->rightFront;
			print_interface();
		}
		if (g_XGV_left_front_wheel_speed != reportWheelsSpeed->leftFront)
		{
			g_XGV_left_front_wheel_speed = reportWheelsSpeed->leftFront;
			print_interface();
		}
		if (g_XGV_right_rear_wheel_speed != reportWheelsSpeed->rightRear)
		{
			g_XGV_right_rear_wheel_speed = reportWheelsSpeed->rightRear;
			print_interface();
		}
		if (g_XGV_left_rear_wheel_speed != reportWheelsSpeed->leftRear)
		{
			g_XGV_left_rear_wheel_speed = reportWheelsSpeed->leftRear;
			print_interface();
		}
		
		/*		printf("commandCode = 0x%x  ", reportVelocityState->commandCode);
		 *		printf("destination = %d.%d.%d.%d\r", 	reportVelocityState->destination->subsystem, 
		 *							reportVelocityState->destination->node, 
		 *							reportVelocityState->destination->component, 
		 *							reportVelocityState->destination->instance);
		 */
		reportWheelsSpeedMessageDestroy(reportWheelsSpeed);
	}
	else
	{
		////cError("vss: Error unpacking %s message.\n", jausMessageCommandCodeString(message));
	}
}


void 
report_whrench_effort_message_handler(OjCmpt XGV_CCU, JausMessage whrench_effort_message)
{
	ReportWrenchEffortMessage reportWrenchEffort;
	int data_changed = 0;
	
	reportWrenchEffort = reportWrenchEffortMessageFromJausMessage(whrench_effort_message);
	if (reportWrenchEffort)
	{
		if (g_XGV_throttle != reportWrenchEffort->propulsiveLinearEffortXPercent)
		{
			g_XGV_throttle = reportWrenchEffort->propulsiveLinearEffortXPercent;
			data_changed = 1;
		}
		if (g_XGV_steering != reportWrenchEffort->propulsiveRotationalEffortZPercent)
		{
			g_XGV_steering = reportWrenchEffort->propulsiveRotationalEffortZPercent;
			data_changed = 1;
		}
		if (g_XGV_brakes != reportWrenchEffort->resistiveLinearEffortXPercent)
		{
			g_XGV_brakes = reportWrenchEffort->resistiveLinearEffortXPercent;
			data_changed = 1;
		}
		if (data_changed)
			print_interface();
		
		/*		printf("commandCode = 0x%x  ", reportWrenchEffort->commandCode);
		 *		printf("destination = %d.%d.%d.%d\r", 	reportWrenchEffort->destination->subsystem, 
		 *							reportWrenchEffort->destination->node, 
		 *							reportWrenchEffort->destination->component, 
		 *							reportWrenchEffort->destination->instance);
		 */
		reportWrenchEffortMessageDestroy(reportWrenchEffort);
	}
	else
	{
		////cError("vss: Error unpacking %s message.\n", jausMessageCommandCodeString(message));
	}
}


void 
report_discrete_devices_message_handler(OjCmpt XGV_CCU, JausMessage discrete_devices_message)
{
	ReportDiscreteDevicesMessage reportDiscreteDevices;
	int data_changed = 0;
	
	reportDiscreteDevices = reportDiscreteDevicesMessageFromJausMessage(discrete_devices_message);
	if (reportDiscreteDevices)
	{
		if (g_XGV_main_propulsion != reportDiscreteDevices->mainPropulsion)
		{
			g_XGV_main_propulsion = reportDiscreteDevices->mainPropulsion;
			data_changed = 1;
		}
		if (g_XGV_main_fuel_supply != reportDiscreteDevices->mainFuelSupply)
		{
			g_XGV_main_fuel_supply = reportDiscreteDevices->mainFuelSupply;
			data_changed = 1;
		}
		if (g_XGV_parking_brake != reportDiscreteDevices->parkingBrake)
		{
			g_XGV_parking_brake = reportDiscreteDevices->parkingBrake;
			data_changed = 1;
		}
		if (g_XGV_gear != reportDiscreteDevices->gear)
		{
			g_XGV_gear = reportDiscreteDevices->gear;
			data_changed = 1;
		}
		if (data_changed)
			print_interface();
		
		/*		printf("commandCode = 0x%x  ", reportDiscreteDevices->commandCode);
		 *		printf("destination = %d.%d.%d.%d\r", 	reportDiscreteDevices->destination->subsystem, 
		 *							reportDiscreteDevices->destination->node, 
		 *							reportDiscreteDevices->destination->component, 
		 *							reportDiscreteDevices->destination->instance);
		 */
		reportDiscreteDevicesMessageDestroy(reportDiscreteDevices);
	}
	else
	{
		////cError("vss: Error unpacking %s message.\n", jausMessageCommandCodeString(message));
	}
}


void 
report_signals_message_handler(OjCmpt XGV_CCU, JausMessage signals_message)
{
	ReportSignalsMessage reportSignals;
	int data_changed = 0;
	
	reportSignals = reportSignalsMessageFromJausMessage(signals_message);
	if (reportSignals)
	{
		if (g_XGV_turn_signal != reportSignals->turnSignal)
		{
			g_XGV_turn_signal = reportSignals->turnSignal;
			data_changed = 1;
		}
		if (g_XGV_horn_status != reportSignals->hornStatus)
		{
			g_XGV_horn_status = reportSignals->hornStatus;
			data_changed = 1;
		}
		if (g_XGV_headlights_status != reportSignals->headlightsStatus)
		{
			g_XGV_headlights_status = reportSignals->headlightsStatus;
			data_changed = 1;
		}
		if (data_changed)
			print_interface();
		
		/*		printf("commandCode = 0x%x  ", reportSignals->commandCode);
		 *		printf("destination = %d.%d.%d.%d\r", 	reportSignals->destination->subsystem, 
		 *							reportSignals->destination->node, 
		 *							reportSignals->destination->component, 
		 *							reportSignals->destination->instance);
		 */
		reportSignalsMessageDestroy(reportSignals);
	}
	else
	{
		////cError("vss: Error unpacking %s message.\n", jausMessageCommandCodeString(message));
	}
}


void 
report_error_count_message_handler(OjCmpt XGV_CCU, JausMessage error_message)
{
	ReportErrorCountMessage reportErrorCount;
	int i;
	
	reportErrorCount = reportErrorCountMessageFromJausMessage(error_message);
	if (reportErrorCount)
	{
		g_XGV_num_errors = reportErrorCount->numberOfErrors;
		for (i = 0; i < g_XGV_num_errors; i++)
		{
			g_XGV_error[i] = reportErrorCount->error[i];
		}
		reportErrorCountMessageDestroy(reportErrorCount);
	}
	else
	{
		////cError("vss: Error unpacking %s message.\n", jausMessageCommandCodeString(message));
	}
}


void
register_xgv_ccu_messages_callbacks(OjCmpt XGV_CCU)
{
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_VELOCITY_STATE, velocity_state_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_WRENCH_EFFORT, report_whrench_effort_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_DISCRETE_DEVICES, report_discrete_devices_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_CURVATURE, report_curvature_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_WHEELS_SPEED, report_wheels_speed_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_SIGNALS, report_signals_message_handler);
	ojCmptSetMessageCallback(XGV_CCU, JAUS_REPORT_ERROR_COUNT, report_error_count_message_handler);
}
