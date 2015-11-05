/*!@file Devices/Gyro.H Interface to a GyroMouse */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Gyro.H $
// $Id: Gyro.H 4663 2005-06-23 17:47:28Z rjpeters $

#ifndef GYRO_H_DEFINED
#define GYRO_H_DEFINED

#include<pthread.h>
#include<unistd.h>

// This class allows us to use a ps2 gyromouse as an angle sensor
class Gyro
{
public:
  Gyro();
  void go();
  void getAngle( int & , int & );
  void setAngle( int , int );

private:
  int xpos;
  int ypos;
  int xdpos;
  int ydpos;
  pthread_t runner;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
