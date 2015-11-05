/*!@file Roomba.H Simple roomba class to interact with irobot create  */

//////////////////////////////////////////////////////////////////// //
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
// Primary maintainer for this file: Farhan Baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/IRobot/Roomba.H $
// $Id: Roomba.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef ROOMBA_H_DEFINED
#define ROOMBA_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"

class Serial;

class Roomba : public ModelComponent
{
    public:

        //!Default constructor
    Roomba(OptionManager& mgr, const std::string& descrName ="Roomba",
           const std::string& tagName ="Roomba",
           const char *defdev = "/dev/ttyUSB0");

          //!Destructor
    ~Roomba();


      int getSpeed();
      int getAngle();
      int getRadius();
      int getDist();
      void setSpeed(const int speed);
      void setRadius(const int radius);
      void sendRawCmd(const std::string& data);
      void playSong(int song);
      void sendStart();
      void setMode(const int mode);  //safemode =1, fullmode=2
      void setDemo(const short demo);
      void setLED(const short led, const short color, const short intensity);
      void getDistanceAngle(int& dist, int& ang);
          //!Send the command to drive the robot
      void sendDriveCommand();
          //!Send the command to direct drive the robot
      void sendDirectDriveCommand(float steering);


      protected:

         int itsSpeed;          //!current speed
         int itsRadius;         //!current turning radius set
         int itsAngle;          //!total angle turned since start
         int itsDist;           //!total dist travelled since start
         bool robotStarted;     //!robot start status must be true to send commands
       nub::soft_ref<Serial> itsPort;  //!Serial port to use
        //Serial itsPort;

};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
