#include "cyton_arm_core.h"
//#define RENDER_DEBUG

//extern int g_arm_busy;

int execute_point_command(cyton::hardwareInterface __attribute__((unused)) *hardware, ControlSystem *control, 
							EcReal &currentTime, carmen_cyton_arm_point_command_message message)
{
	EcRealVector jointAngles;
	EcRealVector jointRates;

	//Get the starting position
	CoordinateSystemTransformation initialPose, currentPose;
	EndEffectorVector eeVec;
	control->getParam<ControlSystem::EndEffectors>(eeVec);

	//Use the second to last end-effector.
	if(eeVec.size() <= 2)
	{
		fprintf(stderr, "Unknown configuration for End-Effectors.\n");
		return 1;
	}
	const EcU32 endEffectorId = eeVec.size()-2;

	eeVec[endEffectorId].getParam<EndEffector::ActualPose>(initialPose);
	EcRealVector realVec;

#ifndef	RENDER_DEBUG
	hardware->getJointStates(realVec, cyton::JointAngleInRadiansBiasScale);
	control->setParam<ControlSystem::JointAngles>(realVec);
#else
	control->getParam<ControlSystem::JointAngles>(realVec);
	//control->setParam<ControlSystem::JointAngles>
#endif

	eeVec[endEffectorId].getParam<EndEffector::DesiredVelocity>(realVec);

	Array3 pos = initialPose.translation();
	CoordinateSystemTransformation desiredPose;
	
	//We retreive the desired pose from the point message.
	pos = Array3(message.x, message.y, message.z);

	desiredPose.setTranslation(pos);
	
	//Set the desired final position.
	EcBoolean passed = eeVec[endEffectorId].setParam<EndEffector::DesiredPose>(desiredPose);

	//Now we start the process of moving the end-effector to the desired position

	//Now we define the number and size of time ticks in which we attempt to move
	//the arm to the desired pose. In this case, we define the maximum time spent
	//trying to get there 10 seconds, divided in 500 steps.
	const EcU32 steps = 600;
	const EcReal simRunTime = 30.0;
	const EcReal simTimeStep = simRunTime/steps;

	for(EcU32 ii = 0; ii < steps; ii++)
	{
		//Get the current time
		currentTime += simTimeStep;

		//Calculate new joint values
		passed = control->calculateToNewTime(currentTime);
		passed &= control->getParam<ControlSystem::JointAngles>(jointAngles);
		if(!passed)
		{
			std::cerr << "Unable to calculate new joint angles.\n";
			return 1;
		}

#ifndef RENDER_DEBUG
		//Pass the calculated joint angles to the hardware.
		passed = hardware->setJointCommands(currentTime, jointAngles, cyton::JointAngleInRadiansBiasScale);
		if(!passed)
		{
			std::cerr << "Unable to set joint angles to hardware.\n";
			return 1;
		}
#endif
		eeVec[endEffectorId].getParam<EndEffector::ActualPose>(currentPose);
		if(currentPose.approxEq(desiredPose, 1.0e-3))
		{
			std::cout << "Reached the goal position in " << currentTime << "seconds (" << ii << " steps)\n";
			break;
		}
	}

	
	return 0;
}


int execute_joint_command(cyton::hardwareInterface *hardware, ControlSystem __attribute__((unused)) *control,
							EcReal &currentTime,	carmen_cyton_arm_joint_command_message message)
{
	//There is a 'cyton::JointAngleInDegrees' but we'll use radians.
	cyton::StateType jointModifier = cyton::JointAngleInRadians;

	//As the documentation says, the typical setup speed is .25rad/s.
	//So this time is set up this way because a full "rotation" takes 
	//about ~25 secs, making this wait time safe.
	const EcU32 waitTimeInMS = 1000;
	const size_t numJoints = hardware->numJoints();
	if(!numJoints)
	{
		std::cerr << "Invalid configuration. No joints available.\n";
		return 1;
	}

	//Make sure we just have a modifier parameter.
	//Check the 'cytonTypes.h' in the cyton include folder
	//For more info about Joint Modifiers.
	//jointModifier &= ~cyton::JointBaseMask;

	//These will store the joint angle values (min, max, and such).
	EcRealVector jointAngles(numJoints), minAngles(numJoints), maxAngles(numJoints);

	if(!hardware->getJointStates(minAngles, cyton::MinAngle | jointModifier))
	{
		std::cerr << "Unable to get the minimum angles from getJointStates.\n";
		return 1;
	}

	if(!hardware->getJointStates(maxAngles, cyton::MaxAngle | jointModifier))
	{
		std::cerr << "Unable to get the maximum angles from getJointStates.\n";
		return 1;
	}

	//Now we start the process of moving every joint to the angles we want
	for(EcU32 ii = 0; ii < numJoints; ii++)
	{
		//Retrieve the joint angle we want from the message.
		if(message.joint_angles[ii] < minAngles[ii])
		{
			std::cerr << "Joint value passed to " << ii << " too low. Setting to minAngle.\n";
			jointAngles[ii] = minAngles[ii];
		}else{
			if(message.joint_angles[ii] > maxAngles[ii])
			{
				std::cerr << "Joint value passed to " << ii << "too high. Setting to maxAngle.\n";
				jointAngles[ii] = maxAngles[ii];
			}else{
				jointAngles[ii] = message.joint_angles[ii];
			}
		}		
	}
	
	currentTime += 30;
	if(!hardware->setJointCommands(currentTime, jointAngles, jointModifier))
	{
		std::cerr << "Problem setting the arm joint angles.\n";
		return 1;
	}
	
	control->setParam<ControlSystem::JointAngles>(jointAngles);
	hardware->waitUntilCommandFinished(waitTimeInMS); //Wait until the hardware set the position.
	
	return 0;
}


int publish_arm_state(cyton::hardwareInterface *hardware, ControlSystem *control,
						EcReal __attribute__ ((unused)) &currentTime, int arm_busy)
{
	IPC_RETURN_TYPE err;

	EcRealVector jointStates;

#ifndef RENDER_DEBUG
	EcU32 num_joints = hardware->numJoints();
#else
	EcU32 num_joints = control->param<ControlSystem::NumJoints, EcU32>();
#endif

	EndEffectorVector eeVec;
	Array3 actualPose;
	CoordinateSystemTransformation coord;

	unsigned int aux;

	carmen_cyton_arm_state_message state;

#ifndef RENDER_DEBUG
	//Get the joint states.
	if(!hardware->getJointStates(jointStates, cyton::JointAngleInRadians))
	{
		std::cerr << "Unable to retrieve joint states\n";
		return 1;
	}
#else
	if(!control->getParam<ControlSystem::JointAngles>(jointStates))
	{
		std::cerr << "Unable to retrieve joint states\n";
		return 1;
	}

#endif
	
	//Start getting the Coordinate System (x, y, z) points
	//First we get the End-Effector Vector
	control->getParam<ControlSystem::EndEffectors>(eeVec);
	
	const size_t eeVecSize = eeVec.size();

	if(eeVecSize <= 2)
	{
		std::cerr << "Unable to get the End-Effector Vector size.\n";
		return 1;
	}

	//We will use the second to last.
	EndEffector& ee = eeVec[eeVecSize - 2];
	ee.getParam<EndEffector::ActualPose>(coord);
	actualPose = coord.translation(); //We got the pose! We winners now.

	//Now we beggin to fill the state message.
	state.arm_busy = arm_busy;

	//Get the pose.
	state.x = actualPose[0];
	state.y = actualPose[1];
	state.z = actualPose[2];
	state.gripper = 0; //TODO:: Get the actual gripper.

	state.num_joints = num_joints;
	state.joint_angles = (double*)malloc(num_joints * sizeof(double));

	if(!state.joint_angles)
	{
		std::cerr << "Unable to allocate enough memory to the state message.\n";
		return 1;
	}

	for(aux = 0; aux < num_joints; aux++)
	{
		//std::cerr << jointStates[aux] << "\t";
		state.joint_angles[aux] = jointStates[aux];
	}
	//std::cerr << "\n";

	state.timestamp = carmen_get_time();
	state.host = carmen_get_host();

	err = IPC_publishData(CARMEN_CYTON_ARM_STATE_NAME, &state);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_CYTON_ARM_STATE_NAME);

	free(state.joint_angles);

	return 0;
}
