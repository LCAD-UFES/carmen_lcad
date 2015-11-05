#include "BeoPilot.H"
#include "ros/ros.h"
#include <control_toolbox/pid.h>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
// ######################################################################
BeoPilot::BeoPilot():
	itsSerial(new Serial()),
        itsRCChannels(7, -1),
        itsEmergencyMode(-1)
{
        //Connect to the serial motor board
        //itsSerial->configureSearch ("motorboard", 115200);
        //itsSerial->configure ("/dev/ttyUSB0", 115200);//usb serial
        itsSerial->configure ("/dev/rfcomm1", 115200);//bluetooth serial
	itsPidSpeed.initPid(0.4,0.0,0.0,2.5,0.0);
	itsPidRot.initPid(  1.0,0.5,0.02,2.5,0.0);
}
void BeoPilot::start()
{
	itsSerial->start();
	usleep(1000);
        if(itsSerial->isSerialOk())
                printf("[%s] found on [%s]\n", itsSerial->getDeviceDescriptor().c_str(), itsSerial->getDevName().c_str());
	itsPreviousTime = ros::Time::now();
	itsLastLeftEncoder = 0;
	itsLastRightEncoder = 0;
	itsRightEncoder = 0;
	itsLeftEncoder = 0;

	itsPidSpeed.reset();
	itsPidRot.reset();
}

// ######################################################################
BeoPilot::~BeoPilot()
{
        //Kill the motors on exit
        SetMotors(0,0);
}
void BeoPilot::UpdateRCStatus()
{
        unsigned char cmd = 107;
        itsSerial->write(cmd);
        std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);
        if(frame.size() == 26)
        {
                itsRCChannels[0] = ((0x0FF & frame[ 0]) << 8) | ((0x0FF & frame[ 1]) << 0);
                itsRCChannels[1] = ((0x0FF & frame[ 2]) << 8) | ((0x0FF & frame[ 3]) << 0);
                itsRCChannels[2] = ((0x0FF & frame[ 4]) << 8) | ((0x0FF & frame[ 5]) << 0);
                itsRCChannels[3] = ((0x0FF & frame[ 6]) << 8) | ((0x0FF & frame[ 7]) << 0);
                itsRCChannels[4] = ((0x0FF & frame[ 8]) << 8) | ((0x0FF & frame[ 9]) << 0);
                itsRCChannels[5] = ((0x0FF & frame[10]) << 8) | ((0x0FF & frame[11]) << 0);
                itsRCChannels[6] = ((0x0FF & frame[12]) << 8) | ((0x0FF & frame[13]) << 0);

                itsEmergencyMode = frame[14];
                if(itsRCChannels[2] > 1800)
                        itsEmergencyMode = 255;
                itsRemoteMode    = frame[15];
                itsMotor1Speed   = (frame[16] - 64.0) / 64.0;
                itsMotor2Speed   = (frame[17] - 64.0) / 64.0;

                itsLastLeftEncoder  = itsLeftEncoder;
                itsLastRightEncoder = itsRightEncoder;
                itsLeftEncoder   = ((0x0FF & frame[18]) << 24) |
                                   ((0x0FF & frame[19]) << 16) |
                                   ((0x0FF & frame[20]) <<  8) |
                                   ((0x0FF & frame[21]) <<  0);

		itsRightEncoder  = ((0x0FF & frame[22]) << 24) |
                                   ((0x0FF & frame[23]) << 16) |
                                   ((0x0FF & frame[24]) <<  8) |
                                   ((0x0FF & frame[25]) <<  0);
		printf("RMode[%d],M1[%f]M2[%f],LE[%d],RE[%d]\n",
				itsRemoteMode,
				itsMotor1Speed,
				itsMotor2Speed,
				itsLeftEncoder,
				itsRightEncoder
		);
		UpdatePosition();
		
        }
        else
        {
                //printf("BAD FRAME RECIEVED!");
        }
}
unsigned int BeoPilot::getRCChannel(int channel)
{
        if(channel < 0 || channel > 7)
                return 0;

        unsigned char cmd[2] = {104, channel};

        itsSerial->write(cmd, 2);
        std::vector<unsigned char> frame = itsSerial->readFrame(cmd[0], 255);

        if(frame.size() == 4)
        {
                unsigned int val = ((0x0FF & frame[0]) << 24) |
                                   ((0x0FF & frame[1]) << 16) |
                                   ((0x0FF & frame[2]) <<  8) |
                                   ((0x0FF & frame[3]) <<  0);
                return val;
        }
        else
                printf("Bad Frame Size! (%lu)", frame.size());

        return 0;

}

unsigned char BeoPilot::getRCStatus()
{
        unsigned char cmd = 103;

        itsSerial->write(cmd);
        std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);

        if(frame.size() == 1)
                return frame[0];

        else
                printf("Bad Frame Size! (%lu)", frame.size());
        return 0;

}

unsigned char BeoPilot::getRCEnabled()
{
        unsigned char cmd = 105;

        itsSerial->write(cmd);
        std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);

        if(frame.size() == 1)
                return frame[0];
        else
                printf("Bad Frame Size! (%lu)", frame.size());
        return 0;
}

unsigned int BeoPilot::getRCSpeed()
{
        unsigned char cmd = 106;

        itsSerial->write(cmd);
        std::vector<unsigned char> frame = itsSerial->readFrame(cmd, 255);

        if(frame.size() == 4)
        {
		unsigned int val = ((0x0FF & frame[0]) << 24) |
			((0x0FF & frame[1]) << 16) |
			((0x0FF & frame[2]) <<  8) |
			((0x0FF & frame[3]) <<  0);
                return val;
        }
        else
                printf("Bad Frame Size! (%lu)", frame.size());

        return 0;

}
// ######################################################################
void BeoPilot::SetMotorsPid(float rotationalSpeed, float translationalSpeed)
{

	double currentRot = (itsLeftMotorVelocity - itsRightMotorVelocity)/2;
	double currentTrans = (itsLeftMotorVelocity + itsRightMotorVelocity)/2;
	double rotErr = currentRot - rotationalSpeed;
	double transErr = currentTrans - translationalSpeed;

	double pidRot = itsPidRot.updatePid(rotErr,dt);
	double pidTrans = itsPidSpeed.updatePid(transErr,dt);

	SetMotors(pidRot,pidTrans);
	


}
// ######################################################################
void BeoPilot::SetMotors(float rotationalSpeed, float translationalSpeed)
{
        //Clamp the motor speeds to [-1 .. 1]

	//float vx = itsVelocity.x;
	//float vy = itsVelocity.y;
	//float th = itsVelocity.z;
	




        if(rotationalSpeed > 1.0)
                rotationalSpeed = 1.0;
        if(rotationalSpeed < -1.0)
                rotationalSpeed = -1.0;
        if(translationalSpeed > 1.0)
                translationalSpeed = 1.0;
        if(translationalSpeed < -1.0)
                translationalSpeed = -1.0;

        char rotSpeed  = char(rotationalSpeed    * 100.0);
        char tranSpeed = char(translationalSpeed * 100.0);

		

        //Send the command for the first motor
        unsigned char cmd[3];
        cmd[0] = 100;             //Command: Set Motor 1
        cmd[1] = rotSpeed;                                 //Command: Rotational Speed?
        itsSerial->write(cmd, 2);
        usleep(10000);

        //Send the command for the second motor
        cmd[0] = 101;             //Command: Set Motor 2
        cmd[1] = tranSpeed;       //Command: Translational Speed?
        itsSerial->write(cmd, 2);
        usleep(10000);
}


// We have 10 inch diameter wheel
// Motor Shaft will turn 21 times for one wheel spin
// The encoder have 4000 tick per revolution
// When the wheel have one revolution, it travels 10 * pi inch
//
// 10* pi inch  = 0.797964534 meters
// Which have 1000 * 21 ticks
//  (10 * pi)/84,000 * 0.0254 =  9.49957779 * 10^(-6) = 0.000009.49957779 meter
// Which means for every tick,
// the wheel will move  meter
//
// Width between two wheel W = 510 mm = 0.51m
// To convert odometry data to position, we need do some math again...
// Ot+1 = Ot + (Dr - Dl) / W
// Dt,t+1 = (Dr + Dl) / 2
// Xt+1 = Xt + Dt,t+1 * cos(Ot+1)
// Yt+1 = Yt + Dt,t+1 * sin(Ot+1)
void BeoPilot::UpdatePosition(void)
{
        if(itsLastRightEncoder == 0.0 && itsLastLeftEncoder == 0.0)
		itsPreviousTime = ros::Time::now();
	dt = ros::Time::now() - itsPreviousTime;
	double time = dt.toSec();
        //double time = (double) itsEncoderTimer.getReset()/1000000.0;//sec

//        double meterPerTick = 0.00000949957779 ;
        double ticksPerMeter = 105267.836;
        double dr = (itsRightEncoder - itsLastRightEncoder) / ticksPerMeter;
        double dl = (itsLeftEncoder - itsLastLeftEncoder)  / ticksPerMeter;
        double w = 0.51;
        double ot = itsPosition.z + ((dr - dl) / w );
        double dt = (dr + dl)/2;
        double xt = itsPosition.x + dt * cos(ot);
        double yt = itsPosition.y + dt * sin(ot);


        // compute motor velocity
        if(time != 0.0){
                itsLeftMotorVelocity = -(dl/time);
                itsRightMotorVelocity = -(dr/time);
        }
        printf("Time[%f] Left V %f Right V %f,dl %f,dr %f",time,itsLeftMotorVelocity,
                        itsRightMotorVelocity,dl,dr);

        // the difference in position between the current and previous time step
        itsDiffPosition.x = dt * cos(ot);
        itsDiffPosition.y = dt * sin(ot);
        itsDiffPosition.z = ((dr - dl) / w ); //use z as orientation data

        itsVelocity.x = itsDiffPosition.x/time;
        itsVelocity.y = itsDiffPosition.y/time;
        itsVelocity.z = itsDiffPosition.z/time;
        itsPosition.x = xt;
        itsPosition.y = yt;
        itsPosition.z = ot;//use z as orientation data

}

void BeoPilot::resetEncoder()
{

        unsigned char cmd = 108;
        itsSerial->write(cmd);

        itsPosition.x = 0;
        itsPosition.y = 0;
        itsPosition.z = 0;
        itsDiffPosition.x = 0;
        itsDiffPosition.y = 0;
        itsDiffPosition.z = 0;

        itsVelocity.x = 0;
        itsVelocity.y = 0;
        itsVelocity.z = 0;
	itsPidSpeed.reset();
	itsPidRot.reset();

}
