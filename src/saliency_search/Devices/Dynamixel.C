/*!@file Devices/Dynamixel.C Interface to Dynamixel Serial Servo */

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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Dynamixel.C $
// $Id: Dynamixel.C 8524 2012-11-12 20:37:05Z kai $
//
#include "Devices/Dynamixel.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// Control table address

#define P_TORQUE_ENABLE   24
#define P_LED             25
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_MOVING_SPEED_L  32
#define P_MOVING_SPEED_H  33
#define P_TORQUE_LIMIT_L  34
#define P_TORQUE_LIMIT_H  35
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_PRESENT_SPEED_L 38
#define P_PRESENT_SPEED_H 39
#define P_PRESENT_LOAD_L  40
#define P_PRESENT_LOAD_H  41
#define P_PRESENT_VOLTAGE 42
#define P_PRESENT_TEMP    43

#define P_MOVING		46

// ######################################################################
Dynamixel::Dynamixel(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName)
{


  // Initialize our internals:
  //zero = new rutz::shared_ptr<NModelParam<float> >[NUMSERVOS];
  //posmult = new rutz::shared_ptr<NModelParam<float> >[NUMSERVOS];
  //negmult = new rutz::shared_ptr<NModelParam<float> >[NUMSERVOS];
  //pos = new byte[NUMSERVOS];
}

// ######################################################################
Dynamixel::~Dynamixel()
{
  //delete [] pos;
  //delete [] negmult;
  //delete [] posmult;
  //delete [] zero;
}

// ######################################################################
bool Dynamixel::init(int baudnum,int deviceIndex)
{

	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		LFATAL( "Failed to open USB2Dynamixel!" );
		return false;
	}
	else
		LINFO( "Succeed to open USB2Dynamixel!\n" );
  return true;
}

// ######################################################################
bool Dynamixel::commStatus()
{
  int CommStatus = dxl_get_result();

  if( CommStatus == COMM_RXSUCCESS )
  {
      return true;
  }
  LDEBUG("Comm RX Fail");
  return false;
}
// ######################################################################
bool Dynamixel::move(const int servo, const float deg)
{ 
  float position = deg*1024.0/300.0;
  return moveRaw(servo, (int)position); 

}
// ######################################################################
bool Dynamixel::moveRaw(const int servo, const int rawpos)
{
		// Write goal position
  dxl_write_word( servo, P_GOAL_POSITION_L, rawpos);
  //wait until it's done
  //while(isMoving(servo));
  return false;
}
// ######################################################################
bool Dynamixel::isMoving(const int servo)
{
  //getPosition(servo);
  // Check moving done
  int Moving = dxl_read_byte( servo, P_MOVING );
  commStatus();
  return Moving == 1 ? true : false;
}
//Moving Speed : Join Mode
//0~1023 (0X3FF) can be used, and the unit is about 0.111rpm.
//If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
//If it is 1023, it is about 114rpm.
//For example, if it is set to 300, it is about 33.3 rpm.
// ######################################################################
bool Dynamixel::setSpeedRaw(const int servo, const int speed)
{
  dxl_write_word( servo, P_MOVING_SPEED_L, speed);
  return commStatus();
}
// ######################################################################
//Set speed in RPM
bool Dynamixel::setSpeed(const int servo, const int rpm)
{
  double rawSpeed = rpm/0.111;
  if(rawSpeed < 0 || rawSpeed > 1023) rawSpeed = 0;
  return setSpeedRaw(servo,rawSpeed);
}
// ######################################################################
bool Dynamixel::setTorque(const int servo,bool hold)
{
  dxl_write_word( servo, P_TORQUE_ENABLE, hold?1:0);
  return commStatus();
}
// ######################################################################
bool Dynamixel::setLed(const int servo,bool on)
{
  dxl_write_word( servo, P_LED, on?1:0);
  return commStatus();
}

// ######################################################################
bool Dynamixel::setNeutral(const int servo, const short int pos)
{
      return false;
}

// ######################################################################
bool Dynamixel::setParam(const int servo,
    bool on_off, bool direction, char range)
{
      return false;
}
// ######################################################################
float Dynamixel::getPosition(const int servo) const
{
  float deg = getPositionRaw(servo)*300.0/1024.0;
  return deg;
}
//It is the current position value of Dynamixel.  
//The range of the value is 0~1023 (0x3FF), and the unit is 0.29 degree.
// ######################################################################
int Dynamixel::getPositionRaw(const int servo) const
{
  int pos = dxl_read_word( servo, P_PRESENT_POSITION_L );
  int CommStatus = dxl_get_result();

  if( CommStatus == COMM_RXSUCCESS )
  {
    LDEBUG( "PresentPos %d\n", pos);    
    //PrintErrorCode();
    return pos;
  }
  else
  {
    //PrintCommStatus(CommStatus);
    LDEBUG("Can't not read servo %d",servo);    
  }
  return -1;
}
// ######################################################################
float Dynamixel::getSpeed(const int servo) 
{
  float rpm = getSpeedRaw(servo)*0.111;
  return rpm;
}
// ######################################################################
int Dynamixel::getSpeedRaw(const int servo) 
{
  int speed = dxl_read_word( servo, P_PRESENT_SPEED_L);
  int CommStatus = dxl_get_result();
  if( CommStatus == COMM_RXSUCCESS )
  {
    LDEBUG( "PresentSpeed %d\n", speed);    
    return speed;
  }
  else
  {
    LDEBUG("Can't not read servo %d",servo);    
  }
  return -1;
}
// ######################################################################
float Dynamixel::getVoltage(const int servo) 
{
  float volt = dxl_read_word( servo, P_PRESENT_VOLTAGE)/10.0;
  int CommStatus = dxl_get_result();
  if( CommStatus == COMM_RXSUCCESS )
  {
    LDEBUG( "Present Voltage  %4.2f\n", volt);    
    return volt;
  }
  else
  {
    LDEBUG("Can't not read servo %d",servo);    
  }
  return -1;
}
// ######################################################################
float Dynamixel::getTemperature(const int servo) 
{
  int temp = dxl_read_word( servo, P_PRESENT_TEMP);
  int CommStatus = dxl_get_result();
  if( CommStatus == COMM_RXSUCCESS )
  {
    LDEBUG( "Present Temp %d C\n", temp);    
    return temp;
  }
  else
  {
    LDEBUG("Can't not read servo %d",servo);    
  }
  return -1;
}


// ######################################################################
// Print communication result
void Dynamixel::PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// ######################################################################
// Print error bit of status packet
void Dynamixel::PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
