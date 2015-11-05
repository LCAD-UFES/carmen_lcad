/*!@file Devices/RangeFinder.H Interface to a home made range-finder */


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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/RangeFinder.C $
// $Id: RangeFinder.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Devices/RangeFinder.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// ######################################################################
RangeFinder::RangeFinder(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr))
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 19200, "8N1", false, false, 1);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);

  itsRangeData.resize(74);
}

// ######################################################################
RangeFinder::~RangeFinder()
{ }


// ######################################################################
void RangeFinder::readRangeData()
{
  unsigned char buff[50];

  int state = 0;
  while(state != 3)
  {
    itsPort->read(buff, 1);

    if (state == 0 && buff[0] == 255) state = 1;
    if (state == 1 && buff[0] == 0) state = 2;
    if (state == 2 && buff[0] == 255) state = 3;

    if (state == 3)
    {
      unsigned char index, val;
      itsPort->read(&index, 1);
      itsPort->read(&val, 1);
      if (index > 0 && index < (unsigned char)itsRangeData.size())
        itsRangeData[index]=val;
    }
  }

}

/*
void RangeFinder::readUTMVersion()
{
  char msg[5];
  char buff[512];
  msg[0] = 'V';
  msg[1] = '\r';

  itsPort->write(msg, 2);
  int i =itsPort->read(buff, 512);
  buff[i]=0;

  LINFO("Version %s", buff);
}*/

void RangeFinder::readUTMRangeData()
{

  char msg[10];
  char buff[1024];
  msg[0] = 'G'; //Distance data acc
  msg[1] = '1'; msg[2] = '2'; msg[3] = '8'; //start point
  msg[4] = '6'; msg[5] = '4'; msg[6] = '0'; //end point
  msg[7] = '0'; msg[8] = '7'; //cluster count
  msg[9] = '\r';

  itsPort->write(msg, 10);
  itsPort->read(buff, 10); //command
  //printf("Command(%i): ", i);
  //for(int ii=0; ii<i; ii++)
  //  printf("%c ", buff[ii]);
  //printf("\n");
  itsPort->read(buff, 8); //status
  //printf("Status(%i): ",j);
  //for(int ii=0; ii<j; ii++)
  //  printf("%i ", buff[ii]);
  //printf("\n");
  int k =itsPort->read(buff, 512); //data
  //printf("Data(%i): ",k);
  //for(int ii=0; ii<k; ii++)
  //  printf("%i ", buff[ii]);
  //printf("\n");

  int index = 0;
  for(int ii=0; ii<k; ii++)
  {
    if (buff[ii] == 48)
    {
      uint range = ((buff[ii+1]-30) << 6) + (buff[ii+2]-30);
      index++;
      if (index > 0 && index < (unsigned char)itsRangeData.size())
        itsRangeData[index]=range;
      //LINFO("Range %i:%i",index, range);
      ii+=2;
    }
  }


}


std::vector<int> RangeFinder::getRangeData()
{
  //readRangeData();
  readUTMRangeData();
  return itsRangeData;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
