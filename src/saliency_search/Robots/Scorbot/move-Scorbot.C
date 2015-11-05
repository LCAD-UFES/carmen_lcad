/*!@file src/Robots/Scorobt/move-Scorbot.C Move the robot in varius ways */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Scorbot/move-Scorbot.C $
// $Id: move-Scorbot.C 14059 2010-09-28 02:13:59Z rand $
//

#include "Component/ModelManager.H"
#include "Devices/Scorbot.H"
#include "Util/MathFunctions.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/PrefsWindow.H"
#include "Image/DrawOps.H"
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

nub::soft_ref<Scorbot> scorbot;
//Sholder 3000 - -7000
//ex1: -20000 - -150000
//sholder: 
void terminate(int s)
{
	LINFO("*** INTERRUPT ***");
	scorbot->stopAllMotors();
	scorbot->motorsOff();
	sleep(1);
	exit(0);
}


int main(int argc, char **argv)
{
	// Instantiate a ModelManager:
	ModelManager manager("Move Robot");

	scorbot = nub::soft_ref<Scorbot>(new Scorbot(manager));
	manager.addSubComponent(scorbot);

	// catch signals and redirect them to terminate for clean exit:
	signal(SIGHUP, terminate); signal(SIGINT, terminate);
	signal(SIGQUIT, terminate); signal(SIGTERM, terminate);
	signal(SIGALRM, terminate);

	// Grab the first command line option, and then wipe it out. The manager can't see any 
	// options after this.
	if(argc != 3)
	{
		std::cerr << "TIME POSITION" << std::endl;
		exit(0);
	}
	int time = atoi(argv[1]);
	int pos  = atoi(argv[2]);
	std::cerr << "TIME: " << time << std::endl;
	std::cerr << "POS:  " << pos << std::endl;
	argv[1] = 0;

	// Parse command-line:
	if (manager.parseCommandLine(1, argv, "", 0, 0) == false) return(1);

	// let's get all our ModelComponent instances started:
	manager.start();




	// ######################################################################
	//PID TESTING
	Scorbot::ArmPos armPos;

	scorbot->resetEncoders();
	armPos.base     = 0;
	armPos.sholder  = 0;
	armPos.elbow    = 0;
	armPos.wrist1   = 0;
	armPos.wrist2   = 0;
	armPos.gripper  = 0;
	armPos.ex1      = 0;
	armPos.duration = 1;
	if(!scorbot->setArmPos(armPos))
		LFATAL("Unable to set position");

	usleep(100000);
	scorbot->motorsOn();
	sleep(1);

	scorbot->clearBuffer();

	while(1)
	{
	  scorbot->motorsOn();

		armPos.base     = 0;
		armPos.sholder  = 0;
		armPos.elbow    = 0;
		armPos.wrist1   = 0;
		armPos.wrist2   = 0;
		armPos.gripper  = 0;
		armPos.ex1      = pos;
		armPos.duration = time;
		if(!scorbot->setArmPos(armPos))
			LFATAL("Unable to set position");

		sleep(time/100+2);

		long time, posErr, desiredPos, desiredVel, encoderVal, pwm;
		scorbot->getJointParams(Scorbot::EX1, time, posErr, desiredPos, desiredVel, encoderVal, pwm);
		printf("%li\n", 
				encoderVal);
		sleep(1);

		armPos.base     = 0;
		armPos.sholder  = 0;
		armPos.elbow    = 0;
		armPos.wrist1   = 0;
		armPos.wrist2   = 0;
		armPos.gripper  = 0;
		armPos.ex1      = 0;
		armPos.duration = time;
		if(!scorbot->setArmPos(armPos))
			LFATAL("Unable to set position");

		sleep(time/100+2);

		scorbot->getJointParams(Scorbot::EX1, time, posErr, desiredPos, desiredVel, encoderVal, pwm);
		printf("%li\n", 
				encoderVal);
		sleep(1);

	}



	armPos.base     = 0;
	armPos.sholder  = 0;
	armPos.elbow    = 0;
	armPos.wrist1   = 0;
	armPos.wrist2   = 0;
	armPos.gripper  = 0;
	armPos.ex1      = pos;
	armPos.duration = time;
	if(!scorbot->setArmPos(armPos))
		LFATAL("Unable to set position");

	int i=0;
	while(1)
	{
		long time, posErr, desiredPos, desiredVel, encoderVal, pwm;
		scorbot->getJointParams(Scorbot::EX1, time, posErr, desiredPos, desiredVel, encoderVal, pwm);

		printf("%i %li %li %li %li %li %li\n", 
				i++, time, posErr, desiredPos, desiredVel, encoderVal, pwm);
		usleep(10000);
	}
	// ######################################################################


	//scorbot->motorsOn();
	//sleep(1);
	//scorbot->clearBuffer();
	//Scorbot::ArmPos armPos;
	//while(1)
	//{
	//	LINFO("POS 1");
	//	armPos.base    = 10585 ;
	//	armPos.sholder = -102;
	//	armPos.elbow   = 9456;
	//	armPos.wrist1  = -2566;
	//	armPos.wrist2  = 2260;
	//	armPos.gripper = 0;
	//	armPos.ex1     = -79790;
	//	armPos.duration = 1000;
	//	if(!scorbot->setArmPos(armPos))
	//		LFATAL("Unable to set position");
	//	sleep(5);

	//	LINFO("POS 2");
	//	armPos.base    = 13010;
	//	armPos.sholder = -6955;
	//	armPos.elbow   = 9136;
	//	armPos.wrist1  = -3108;
	//	armPos.wrist2  = 2596;
	//	armPos.gripper = 0;
	//	armPos.ex1     = 19841;
	//	armPos.duration = 1000;
	//	if(!scorbot->setArmPos(armPos))
	//		LFATAL("Unable to set position");
	//	sleep(5);

	//	LINFO("POS 3");
	//	armPos.base    = 10584;
	//	armPos.sholder = 2669;
	//	armPos.elbow   = 1027;
	//	armPos.wrist1  = 678;
	//	armPos.wrist2  = -997;
	//	armPos.gripper = 0;
	//	armPos.ex1     = -82987;
	//	armPos.duration = 1000;
	//	if(!scorbot->setArmPos(armPos))
	//		LFATAL("Unable to set position");
	//	sleep(5);


	//}

	//		for(uint i=0; i<15; i++)
	//		{
	//			//long time, desiredPos, desiredVel, encoderVal, pwm;
	//			//scorbot->getJointParams(RobotArm::EX1, time, desiredPos, desiredVel, encoderVal, pwm);
	//			//printf("%i %li %li %li %li %li\n",i, time, desiredPos, desiredVel, encoderVal, pwm);
	//			Scorbot::ArmPos armPos = scorbot->getArmPos();
	//
	//			printf("%i %i %i %li %li %li %li %li %li %li %li\n", i,
	//					4500, 122500,
	//					armPos.base, armPos.sholder, armPos.elbow, armPos.wrist1, armPos.wrist2,
	//					armPos.gripper, armPos.ex1, armPos.ex2);
	//			usleep(500000);
	//		}
	//
	//		LINFO("Go to position -4500 18000");
	//		armPos.base = -4500;
	//		armPos.sholder = 0;
	//		armPos.elbow = 0;
	//		armPos.wrist1 = 0;
	//		armPos.wrist2 = 0;
	//		armPos.gripper = 0;
	//		armPos.ex1 = 0;
	//		armPos.duration = 1000;
	//		if(!scorbot->setArmPos(armPos))
	//			LFATAL("Unable to set position");
	//
	//		for(uint i=0; i<15; i++)
	//		{
	//			//long time, desiredPos, desiredVel, encoderVal, pwm;
	//			//scorbot->getJointParams(RobotArm::EX1, time, desiredPos, desiredVel, encoderVal, pwm);
	//			//printf("%i %li %li %li %li %li\n",i, time, desiredPos, desiredVel, encoderVal, pwm);
	//			Scorbot::ArmPos armPos = scorbot->getArmPos();
	//
	//			printf("%i %i %i %li %li %li %li %li %li %li %li\n", i,
	//					4500, 122500,
	//					armPos.base, armPos.sholder, armPos.elbow, armPos.wrist1, armPos.wrist2,
	//					armPos.gripper, armPos.ex1, armPos.ex2);
	//			usleep(500000);
	//		}
	//
	//	}

	//for(int pwm=0; pwm>-100; pwm--)
	////for(int pwm=0; pwm<100; pwm++)
	//{
	//	if (scorbot->setMotor(RobotArm::EX1, pwm))
	//	{
	//		for(uint i=0; i<10; i++)
	//		{
	//			Scorbot::ArmPos armPos = scorbot->getArmPos();

	//			LINFO("b:%li s:%li e:%li g:%li e1:%li e2:%li duration: %li",
	//					armPos.base, armPos.sholder, armPos.elbow, armPos.gripper,
	//					armPos.ex1, armPos.ex2, armPos.duration);
	//			printf("%i %ld\n", pwm, armPos.ex1);
	//		}
	//	}
	//}
	//scorbot->setMotor(RobotArm::BASE, 0);


scorbot->motorsOff();
// stop all our ModelComponents
manager.stop();

// all done!
return 0;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
