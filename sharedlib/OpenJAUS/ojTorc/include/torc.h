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
// File Name: torc.h
//
// Written By: Alberto F. De Souza
//
// Version: 0.0.1
//
// Date: 02/21/2012
//
// Description: This file defines the attributes of the XGV JAUS interface

#include "setMotionProfileMessage.h"
#include "reportCurvatureMessage.h"
#include "reportWheelsSpeedMessage.h"
#include "reportSignalsMessage.h"
#include "setSignalsMessage.h"
#include "reportErrorCountMessage.h"
#include "reportComponentStatusMessage.h"


#define	XGV_CCU_NAME				"XGV_CCU"
#define	XGV_CCU_COMPONENTE_ID			4
#define	XGV_CCU_STATE_MACHINE_UPDATE_RATE	50.0

#define	JAUS_MOTION_PROFILE_DRIVER		10
#define	JAUS_SIGNALS_DRIVER			11
#define	JAUS_ERROR_MANAGER			12

#define	SOFTWARE_PAUSE_MODE			0
#define	WRENCH_EFFORTS_MODE			1
#define	MOTION_PROFILES_MODE			2

#define	MAX_THROTTLE				100.0
#define	MIN_THROTTLE				0.0
#define	MAX_STEERING				100.0
#define	MIN_STEERING				-100.0
#define	MAX_BRAKES				100.0
#define	MIN_BRAKES				0.0

#define	MAX_VELOCITY				65.535
#define	MIN_VELOCITY				-65.535
#define	MAX_ACCELERATION			100.0
#define	MIN_ACCELERATION			0.0
#define	MAX_ARCTAN_DESIRED_CURVATURE		(JAUS_HALF_PI)
#define	MIN_ARCTAN_DESIRED_CURVATURE		(-JAUS_HALF_PI)
#define	MAX_RATE_CHANGE_CURVATURE		10.0
#define	MIN_RATE_CHANGE_CURVATURE		0.0
#define	MOTION_PROFILE_TIME_DURATION		2000

#define XGV_WARNING				0
#define XGV_ERROR				1
#define	MAX_ERRORS				255


// Types
typedef struct
{
	int 	error_code;
	int	error_info;
	char	*error_description;
} ErrorDescription;


// Externs
extern double g_throttle_command;
extern double g_steering_command;
extern double g_brakes_command;
extern double g_velocity_command;
extern double g_acceleration_command;
extern double g_atan_curvature_command;
extern double g_rate_change_curvature_command;
extern JausUnsignedShort g_time_duration_command;
extern JausByte g_turn_signal_command;
extern JausByte g_horn_status_command;
extern JausByte g_headlights_status_command;
extern int g_gear_command;
extern int g_engine_command;
extern int g_windshield_wipers_command;
extern int g_num_errors;
extern int *g_error;

extern double g_XGV_throttle;
extern double g_XGV_steering;
extern double g_XGV_brakes;
extern double g_XGV_velocity;
extern double g_XGV_atan_curvature;
extern double g_XGV_right_front_wheel_speed;
extern double g_XGV_left_front_wheel_speed;
extern double g_XGV_right_rear_wheel_speed;
extern double g_XGV_left_rear_wheel_speed;
extern JausByte g_XGV_turn_signal;
extern JausByte g_XGV_horn_status;
extern JausByte g_XGV_headlights_status;
extern int g_XGV_main_propulsion;
extern int g_XGV_main_fuel_supply;
extern int g_XGV_parking_brake;
extern int g_XGV_gear;
extern JausInteger g_XGV_error[MAX_ERRORS];
extern int g_XGV_num_errors;
extern unsigned int g_XGV_component_status;

extern double g_wheels_speed_update_freq;
extern double g_atan_curvature_update_freq;

// Prototypes
void send_set_signals_message(OjCmpt XGV_CCU);
void send_set_discrete_devices_message(OjCmpt XGV_CCU);
void send_set_engine_message(OjCmpt XGV_CCU, int set_engine_on);
void send_set_wrench_efforts_message(OjCmpt XGV_CCU);
void send_set_motion_profile_message(OjCmpt XGV_CCU);
void send_request_control_message(OjCmpt XGV_CCU, int component);
void send_release_control_message(OjCmpt XGV_CCU, int component);
void send_resume_message(OjCmpt XGV_CCU, int component);
void terminate_xgv_ccu_service_connections(OjCmpt XGV_CCU, int *xgv_ccu_service_connections);
int *create_xgv_ccu_service_connections(OjCmpt XGV_CCU);
void register_xgv_ccu_messages_callbacks(OjCmpt XGV_CCU);
void print_interface();
void add_xgv_ccu_component_service_messages(OjCmpt XGV_CCU, int xgv_ccu_component_id);
