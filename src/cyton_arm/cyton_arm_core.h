#ifndef CYTON_ARM_CORE_H_
#define CYTON_ARM_CORE_H_

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

using namespace actinSE;

int execute_point_command(cyton::hardwareInterface *hardware, ControlSystem *control, 
							EcReal &currentTime, carmen_cyton_arm_point_command_message message);

int execute_joint_command(cyton::hardwareInterface *hardware, ControlSystem *control,
							EcReal &currentTime,	carmen_cyton_arm_joint_command_message message);

int publish_arm_state(cyton::hardwareInterface *hardware, ControlSystem *control,
						EcReal &currentTime, int arm_busy);


#endif
