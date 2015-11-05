/*!@file Devices/PingSonar.H Interface to a parallax ping sonar */
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
// Primary maintainer for this file: farhan baluch fbaluch@usc.edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/PingSonar.C $
// $Id: PingSonar.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Devices/PingSonar.H"
#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// ######################################################################
PingSonar::PingSonar(OptionManager& mgr, const std::string& descrName,
                     const std::string& tagName, const char *defdev,
                     const int ns) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr))
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 115200, "8N1", false, false, 1);
  // attach our port as a subcomponent:
  addSubComponent(itsPort);
  numSonars = ns;
}

// ######################################################################
PingSonar::~PingSonar()
{ }

// ######################################################################
void PingSonar::setNumSonars(int n)
{
    numSonars = n;
}


// ######################################################################
std::vector<int> PingSonar::getDists()
{
    unsigned char start ={255};
    unsigned char end = {255};

    unsigned int numBytes = numSonars*4;
    int cnt = 0;
    currentDists.clear();


    std::vector<unsigned char> frame = itsPort->readFrame(start,end,
                                                          numBytes,1);

    LINFO("got %" ZU " bytes",frame.size());
    if(frame.size() == numBytes)
    {
        for(int i=0;i<numSonars;i++)
        {
            int tempDist = ((0x0FF & frame[cnt])  << 24) |
            ((0x0FF & frame[cnt + 1])  << 16) |
            ((0x0FF & frame[cnt + 2])  << 8)  |
            ((0x0FF & frame[cnt + 3])  << 0);
            currentDists.push_back(tempDist);
            cnt = cnt+4;
        }

    }
    else
    {
        LFATAL("bad packets");
    }

    usleep(10000);
    return currentDists;


}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
