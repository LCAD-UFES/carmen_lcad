/*!@file Devices/Gyro.C calculates angle of mouse with respect to floor */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Gyro.C $
// $Id: Gyro.C 6003 2005-11-29 17:22:45Z rjpeters $

//note: "g++ Gyro2.C TestGyro2.C -lpthread"
//to compile
#include<iostream>
#include<fstream>
#include<string.h>
#include<errno.h>
#include<stdio.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include"Gyro.H"

//degrees
#define XANGLE_CONVERSION .0432
#define YANGLE_CONVERSION .0405

void* Gyro_run(void *c0);  // will live in a separate thread
void* Gyro_run(void *c0)
{
  //a wrapper so that we call the member function go() on
  //the Gyro1 object passed as argument:
  Gyro *c = (Gyro *)c0;
  c->go(); return NULL;
}

//#####################################################################
//Gyro constructor
Gyro::Gyro()
{
  xpos = 0;
  ypos = 0;
  xdpos = 0;
  ydpos = 0;
  // start thread for run():
  pthread_create(&runner, NULL, &Gyro_run, (void *)this);
}

//#####################################################################
//continuously updates position
void Gyro::go()
{
  int g1, fileno, nbytes = -1, g2, g3;
  fileno = open("/dev/mouse" , O_NONBLOCK);
  while( 1 ) //conditional break?
    {
      g1 = 999;
      g2 = 0;
      g3 = 0;
      nbytes = read(fileno , &g1 , 1);
      //testing
      //std::cout<<"( "<<nbytes<<" )"<<std::endl;

      if(g1 != 999) g1 = g1 % 256;
      if(g1 == 8 || g1 == 24 || g1 == 56 || g1 == 40)
        {
          nbytes = -1;
          while(nbytes == -1)
            {
              nbytes = read(fileno , &g2 , 1);
              if(nbytes == 1)
                {
                  nbytes = -1;
                  while(nbytes == -1)
                    {
                      nbytes = read(fileno , &g3 , 1);
                    }
                }
            }
          g2 = g2 % 256;
          g3 = g3 % 256;
          if(g2 > 128) g2 = g2 - 256;
          if(g3 > 128) g3 = g3 - 256;
          xdpos = xdpos + g2;
          ydpos = ydpos + g3;
        }
      if(nbytes == -1) usleep(3000);
    }
  close(fileno);
  return;
}

//#####################################################################
//returns position
void Gyro::getAngle( int &x , int &y )
{
  xpos += xdpos;
  ypos += ydpos;
  xdpos = 0;
  ydpos = 0;
  x = int(xpos * XANGLE_CONVERSION);
  y = int(ypos * YANGLE_CONVERSION);
}

//#####################################################################
//sets position to x,y
void Gyro::setAngle( int x , int y )
{
  xpos = x;
  ypos = y;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
