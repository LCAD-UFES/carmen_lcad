/*****************************************************************************
 *  Copyright (c) 2012, lcad.inf.ufes.br; 2009, OpenJAUS.com
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
// File: torcComm.c
//
// Written By: Alberto F. De Souza
//
// Version: 0.0.1
//
// Date: 02/21/2012
//

#include <stdlib.h>
#include <jaus.h>
#include <openJaus.h>
#include "torc.h"


void
send_set_signals_message(OjCmpt XGV_CCU)
{
	JausAddress addr;
	SetSignalsMessage setSignalsMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_SIGNALS_DRIVER;
	addr->instance = 1;
	
	setSignalsMessage = setSignalsMessageCreate();
	jausAddressCopy(setSignalsMessage->destination, addr);
	
	setSignalsMessage->presenceVector = 7;
	setSignalsMessage->turnSignal = g_turn_signal_command;
	setSignalsMessage->hornStatus = g_horn_status_command | g_windshield_wipers_command << 1;
	setSignalsMessage->headlightsStatus = g_headlights_status_command;
	
	message = setSignalsMessageToJausMessage(setSignalsMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausMessageDestroy(message);
	setSignalsMessageDestroy(setSignalsMessage);
	jausAddressDestroy(addr);
}


void
send_set_discrete_devices_message(OjCmpt XGV_CCU)
{
	JausAddress addr;
	SetDiscreteDevicesMessage setDiscreteDevicesMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_PRIMITIVE_DRIVER;
	addr->instance = 1;
	
	setDiscreteDevicesMessage = setDiscreteDevicesMessageCreate();
	jausAddressCopy(setDiscreteDevicesMessage->destination, addr);
	
	setDiscreteDevicesMessage->presenceVector = 4;
	setDiscreteDevicesMessage->gear = g_gear_command;
	
	message = setDiscreteDevicesMessageToJausMessage(setDiscreteDevicesMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausMessageDestroy(message);
	setDiscreteDevicesMessageDestroy(setDiscreteDevicesMessage);
	jausAddressDestroy(addr);
}


void
send_set_engine_message(OjCmpt XGV_CCU, int set_engine_on)
{
	JausAddress addr;
	SetDiscreteDevicesMessage setDiscreteDevicesMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_PRIMITIVE_DRIVER;
	addr->instance = 1;
	
	setDiscreteDevicesMessage = setDiscreteDevicesMessageCreate();
	jausAddressCopy(setDiscreteDevicesMessage->destination, addr);
	
	setDiscreteDevicesMessage->presenceVector = 1;
	if (set_engine_on)
	{
		setDiscreteDevicesMessage->mainPropulsion = JAUS_TRUE;
		setDiscreteDevicesMessage->mainFuelSupply = JAUS_TRUE;
		setDiscreteDevicesMessage->automaticStart = JAUS_TRUE;
		setDiscreteDevicesMessage->auxFuelSupply = JAUS_FALSE; // The Torc manual does not mention how to set this
	}
	else
	{
		setDiscreteDevicesMessage->mainPropulsion = JAUS_FALSE;
		setDiscreteDevicesMessage->mainFuelSupply = JAUS_FALSE;
		setDiscreteDevicesMessage->automaticStart = JAUS_FALSE;
		setDiscreteDevicesMessage->auxFuelSupply = JAUS_FALSE;
	}
	
	message = setDiscreteDevicesMessageToJausMessage(setDiscreteDevicesMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausMessageDestroy(message);
	setDiscreteDevicesMessageDestroy(setDiscreteDevicesMessage);
	jausAddressDestroy(addr);
}


void
send_set_wrench_efforts_message(OjCmpt XGV_CCU)
{
	JausAddress addr;
	SetWrenchEffortMessage setWrenchEffortMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_PRIMITIVE_DRIVER;
	addr->instance = 1;
	
	setWrenchEffortMessage = setWrenchEffortMessageCreate();
	jausAddressCopy(setWrenchEffortMessage->destination, addr);
	
	setWrenchEffortMessage->propulsiveLinearEffortXPercent = g_throttle_command;
	setWrenchEffortMessage->propulsiveRotationalEffortZPercent = g_steering_command;
	setWrenchEffortMessage->resistiveLinearEffortXPercent = g_brakes_command;
	
	message = setWrenchEffortMessageToJausMessage(setWrenchEffortMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausMessageDestroy(message);
	setWrenchEffortMessageDestroy(setWrenchEffortMessage);
	jausAddressDestroy(addr);
}


void
send_set_motion_profile_message(OjCmpt XGV_CCU)
{
	JausAddress addr;
	SetMotionProfileMessage setMotionProfileMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_MOTION_PROFILE_DRIVER;
	addr->instance = 1;
	
	setMotionProfileMessage = setMotionProfileMessageCreate();
	jausAddressCopy(setMotionProfileMessage->destination, addr);
	
	setMotionProfileMessage->version = 1;
	setMotionProfileMessage->numberOfMotions = 1;
	setMotionProfileMessage->desiredVelocity = g_velocity_command;
	setMotionProfileMessage->maximumAcceleration = g_acceleration_command;
	setMotionProfileMessage->atanOfDesiredCurvature = g_atan_curvature_command;
	setMotionProfileMessage->rateOfChangeOfCurvature = g_rate_change_curvature_command;
	setMotionProfileMessage->timeDuration = g_time_duration_command;
	
	message = setMotionProfileMessageToJausMessage(setMotionProfileMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausMessageDestroy(message);
	setMotionProfileMessageDestroy(setMotionProfileMessage);
	jausAddressDestroy(addr);
}


void
send_request_control_message(OjCmpt XGV_CCU, int component)
{
	JausAddress addr;
	RequestComponentControlMessage requestControlMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = component;
	addr->instance = 1;
	
	requestControlMessage = requestComponentControlMessageCreate();
	jausAddressCopy(requestControlMessage->destination, addr);
	
	requestControlMessage->authorityCode = ojCmptGetAuthority(XGV_CCU);
	message = requestComponentControlMessageToJausMessage(requestControlMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausMessageDestroy(message);
	requestComponentControlMessageDestroy(requestControlMessage);
	jausAddressDestroy(addr);
}


void
send_release_control_message(OjCmpt XGV_CCU, int component)
{
	JausAddress addr;
	ReleaseComponentControlMessage releaseControlMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = component;
	addr->instance = 1;
	
	releaseControlMessage = releaseComponentControlMessageCreate();
	jausAddressCopy(releaseControlMessage->destination, addr);
	
	message = releaseComponentControlMessageToJausMessage(releaseControlMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausMessageDestroy(message);
	releaseComponentControlMessageDestroy(releaseControlMessage);
	jausAddressDestroy(addr);
}


void
send_resume_message(OjCmpt XGV_CCU, int component)
{
	JausAddress addr;
	ResumeMessage resumeMessage;
	JausMessage message;
	
	addr = jausAddressCreate();
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = component;
	addr->instance = 1;
	
	resumeMessage = resumeMessageCreate();
	jausAddressCopy(resumeMessage->destination, addr);
	
	message = resumeMessageToJausMessage(resumeMessage);
	ojCmptSendMessage(XGV_CCU, message);
	
	jausAddressDestroy(addr);
	jausMessageDestroy(message);
	resumeMessageDestroy(resumeMessage);
}


void
terminate_xgv_ccu_service_connections(OjCmpt XGV_CCU, int *xgv_ccu_service_connections)
{
	int i;
	
	for (i = 0; i < xgv_ccu_service_connections[0] - 1; i++)
		ojCmptTerminateSc(XGV_CCU, xgv_ccu_service_connections[i + 1]);
	
	free(xgv_ccu_service_connections);
}


int *
create_xgv_ccu_service_connections(OjCmpt XGV_CCU)
{
	JausAddress addr;
	int *xgv_ccu_service_connections = NULL;
	
	// Initialize the list of service connections
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, sizeof(int));
	xgv_ccu_service_connections[0] = 1;
	
	addr = jausAddressCreate();
	
	// Create velocity service connection. See Torc Manual pages 43, 44, 75.
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_VELOCITY_STATE_SENSOR;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] = 
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_VELOCITY_STATE,	  // Command code of the message being requested
		0x01,			  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		2.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;
	
	// Create curvature service connection. See Torc Manual page 73.
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_MOTION_PROFILE_DRIVER;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] = 
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_CURVATURE,	  // Command code of the message being requested
		0x0,			  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		40.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;
	
	// Create wheels speed service connection. See Torc Manual page 72.
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_PRIMITIVE_DRIVER;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] = 
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_WHEELS_SPEED,	  // Command code of the message being requested
		0x0,			  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		40.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;
	
	// Create whrench effort service connection. See Torc Manual pages 43, 44, 68.
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_PRIMITIVE_DRIVER;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] = 
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_WRENCH_EFFORT,	  // Command code of the message being requested
		0x01 | 0x20 | 0x40,	  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		20.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;
	
	// Create discrete devices service connection. See Torc Manual pages 69, 70.
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_PRIMITIVE_DRIVER;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] = 
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_DISCRETE_DEVICES,	  // Command code of the message being requested
		0x01 | 0x02 | 0x04,	  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		2.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;
	
	// Create signals service connection. See Torc Manual page 73-75
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_SIGNALS_DRIVER;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] = 
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_SIGNALS,	  // Command code of the message being requested
		0x01 | 0x02 | 0x04,	  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		2.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;
	
	// Create component status service connection. See Torc Manual page 73-75
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_PRIMITIVE_DRIVER;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] =
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_COMPONENT_STATUS,	  // Command code of the message being requested
		0x00,	  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		5.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;

	// Create error service connection. See Torc Manual page 76-77
	xgv_ccu_service_connections = (int *) realloc(xgv_ccu_service_connections, (xgv_ccu_service_connections[0] + 1) * sizeof(int));
	addr->subsystem = 1;
	addr->node = 1;
	addr->component = JAUS_ERROR_MANAGER;
	addr->instance = 1;
	xgv_ccu_service_connections[xgv_ccu_service_connections[0]] = 
	ojCmptEstablishSc(
		XGV_CCU, 		  // Reference to the component's OjCmpt data structure
		JAUS_REPORT_ERROR_COUNT,	  // Command code of the message being requested
		0x0,			  // Requested presence vector of the desired message
		addr,		  	  // JausAddress of the desired message source component
		2.0,			  // Repeat rate (Hz) at which the message is requested
		3.0,			  // Maximum allowed time (sec) between receiving these messages
		  1			  // Size of the FIFO queue to store the messages in
	);
	xgv_ccu_service_connections[0] = xgv_ccu_service_connections[0] + 1;
	
	jausAddressDestroy(addr);
	return (xgv_ccu_service_connections);
}


void
add_xgv_ccu_component_service_messages(OjCmpt XGV_CCU, int xgv_ccu_component_id)
{
	ojCmptAddServiceInputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REPORT_VELOCITY_STATE, 0xFF);
	ojCmptAddServiceInputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REPORT_WHEELS_SPEED, 0xFF);
	ojCmptAddServiceInputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
	ojCmptAddServiceInputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
	ojCmptAddServiceInputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REPORT_SIGNALS, 0xFF);
	ojCmptAddServiceInputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REPORT_COMPONENT_STATUS, 0xFF);
	ojCmptAddServiceInputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REPORT_ERROR_COUNT, 0xFF);
	ojCmptAddServiceOutputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_SET_WRENCH_EFFORT, 0xFF);
	ojCmptAddServiceOutputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_RESUME, 0xFF);
	ojCmptAddServiceOutputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_REQUEST_COMPONENT_CONTROL, 0xFF);
	ojCmptAddServiceOutputMessage(XGV_CCU, xgv_ccu_component_id, JAUS_SET_SIGNALS, 0xFF);
}
