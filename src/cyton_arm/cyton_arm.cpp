#include <carmen/carmen.h>
#include <actinSE/ControlSystem.h>
#include <actinSE/EndEffector.h>
#include <actinSE/CoordinateSystemTransformation.h>
#include <cytonHardwareInterface.h>
#include <cytonTypes.h>
#include <cstring>
#include <iostream>
#include <cstring>

#include <carmen/cyton_arm_messages.h>
#include <carmen/cyton_arm_interface.h>
#include "cyton_arm_core.h"

#define DEBUG_FLAGS
//#define RENDER_DEBUG

using namespace actinSE;

//Global Variables
EcString dataFile;
ControlSystem control;
cyton::hardwareInterface *hardware;
EcReal currentTime;

static int new_point_message, new_joint_message, g_arm_busy; //Flags

//"Buffers"
carmen_cyton_arm_point_command_message point_buffer;
carmen_cyton_arm_joint_command_message joint_buffer;

//Messages that I'm going to execute
carmen_cyton_arm_point_command_message point_command;
carmen_cyton_arm_joint_command_message joint_command;

/*
 ************************************************
 *				  Utilitaries
 ************************************************
*/
int copy_point_message(carmen_cyton_arm_point_command_message *source, carmen_cyton_arm_point_command_message *dest)
{
	dest->x = source->x;
	dest->y = source->y;
	dest->z = source->z;
	dest->gripper = source->gripper;

	dest->timestamp = source->timestamp;
	dest->host = source->host;

	return 0;
}

int copy_joint_message(carmen_cyton_arm_joint_command_message *source, carmen_cyton_arm_joint_command_message *dest)
{
	short int i;
	
	if(dest->num_joints != source->num_joints)
	{
		free(dest->joint_angles);
		dest->num_joints = source->num_joints;
		dest->joint_angles = (double*)malloc(dest->num_joints * sizeof(double));
	}

	for(i = 0; i < dest->num_joints; i++)
	{
		dest->joint_angles[i] = source->joint_angles[i];
	}
	dest->timestamp = source->timestamp;
	dest->host = source->host;

	return 0;
}

/*
 ************************************************
 *					Handlers
 ************************************************
*/

void point_command_handler()
{
	if(!new_point_message)
	{
		//copy_point_message(&point_buffer, &point_command);
		point_command.x = point_buffer.x;
		point_command.y = point_buffer.y;
		point_command.z = point_buffer.z;
		point_command.gripper = point_buffer.gripper;

		point_command.timestamp = point_buffer.timestamp;
		point_command.host = point_buffer.host;

		new_point_message = 1;
	}
}


void joint_command_handler()
{
	if(!new_joint_message)
	{
		copy_joint_message(&joint_buffer, &joint_command);	
		new_joint_message = 1;
	}
}


void point_time_handler(void __attribute__((unused)) *clientData, 
						unsigned long int __attribute__((unused)) thisTime,
						unsigned long int __attribute__((unused)) scheduledTime)
{
	if(new_point_message && !g_arm_busy)
	{
		g_arm_busy = 1;

		if(publish_arm_state(hardware, &control, currentTime, g_arm_busy))
		{
			carmen_die("Failed to retrieve arm state.\n");
		}

		//std::cerr << "not in use, now executing\n";
		if(execute_point_command(hardware, &control, currentTime, point_command))
		{
			std::cerr << "Unable to execute point command.\n";
		}
		new_point_message = 0;

		g_arm_busy = 0;
	}else{

		if(publish_arm_state(hardware, &control, currentTime, g_arm_busy))
		{
			carmen_die("Failed to retrieve arm state.\n");
		}
	}
}

void joint_time_handler(void __attribute__((unused)) *clientData,
						unsigned long int __attribute__((unused)) thisTime,
						unsigned long int __attribute__((unused)) scheduledTime)
{
	if(new_joint_message && !g_arm_busy)
	{
		g_arm_busy = 1;
		
		if(publish_arm_state(hardware, &control, currentTime, g_arm_busy))
		{
			carmen_die("Failed to retrieve arm state.\n");
		}

		//std::cerr << "not in use, now executing\n";
		if(execute_joint_command(hardware, &control, currentTime, joint_command))
		{
			std::cerr << "Unable to execute joint command.\n";
		}
		new_joint_message = 0;

		g_arm_busy = 0;
	}else{

		if(publish_arm_state(hardware, &control, currentTime, g_arm_busy))
		{
			carmen_die("Failed to retrieve arm state.\n");
		}
	}
}


/*
 ************************************************
 *			  Carmen-related functions
 ************************************************
*/

static int define_cyton_arm_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_CYTON_ARM_STATE_NAME, IPC_VARIABLE_LENGTH,
						CARMEN_CYTON_ARM_STATE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_CYTON_ARM_STATE_NAME);

	return 0;
}


static void __attribute__((unused)) read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
		{(char*)"cyton_arm", (char*)"dataFile", CARMEN_PARAM_STRING, &dataFile, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


static void shutdown_module(int x)
{
	if(x == SIGINT)
	{
#ifndef RENDER_DEBUG
		if(!hardware->reset())
		{
			std::cerr << "Problem reseting to the reset position.\n";
		}
		hardware->waitUntilCommandFinished(1000);
#endif

		carmen_ipc_disconnect();
		carmen_die("Shutting down cyton_arm module");
	}
}

/*
 ************************************************
 *						Main
 ************************************************
*/

int main(int argc, char **argv)
{
	DEBUG_FLAGS;
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	//Set the usage flags and new message flags to 0
	new_joint_message = new_point_message = 0;
	currentTime = 0;
	joint_command.num_joints = 0;
	g_arm_busy = 0;

	signal(SIGINT, shutdown_module);
	
	//A copy of these files should be at the CARMEN_HOME/bin folder
	dataFile = "cyton.ecz";
	const EcString pluginName = "cytonAlphaPlugin.ecp";
	const EcString pluginConfig = "cytonAlphaConfig.xml";
	
	//Although we could read the pluginName and Config in the read_parameters as well,
	//It's possible we will never change its usage.
	//read_parameters(argc, argv);
	
	define_cyton_arm_messages();
	
	printf("Reading the control system file...\n");	
	if(!control.loadFromFile(dataFile))
	{
		carmen_die("Problem loading control system file.\n");
	}
	printf("Done.\n");

#ifndef RENDER_DEBUG

	control.setParam<ControlSystem::Rendering>(EcFalse);

	printf("Initializing the hardware...\n");
	hardware = new cyton::hardwareInterface(pluginName, pluginConfig);
	if(!hardware->init())
	{
		carmen_die("Problem initializing cytonHardwareInterface.\n");
	}
	printf("Done.\n");

	//You can set a lower rate if you want to test something.
	hardware->setLowRate(EcTrue);

#else

	control.setParam<ControlSystem::Rendering>(EcTrue);

#endif

	//Now we set the hardware to its initial position
#ifndef RENDER_DEBUG
	if(!hardware->reset())
	{
		std::cerr << "Problem reseting to the reset position.\n";
	}
	hardware->waitUntilCommandFinished(1000);
#endif
	
	printf("Subscribing to commands...\n");
	carmen_cyton_arm_subscribe_point_command_message(&point_buffer,
						(carmen_handler_t) point_command_handler,
						CARMEN_SUBSCRIBE_LATEST);

	carmen_cyton_arm_subscribe_joint_command_message(&joint_buffer,
						(carmen_handler_t) joint_command_handler,
						CARMEN_SUBSCRIBE_LATEST);

	printf("Done.\n");

	carmen_ipc_addPeriodicTimer(0.1, point_time_handler, NULL);
	carmen_ipc_addPeriodicTimer(0.1, joint_time_handler, NULL);
	
	//Loop forever
	carmen_ipc_dispatch();
	return 0;
}

