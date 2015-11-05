 /*!@file AppDevices/appVecHistField.C appVecHistField test a vector field class  */

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
// Primary maintainer for this file: farhan baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/appVecHistField.C $
// $Id: appVecHistField.C 15085 2011-11-21 22:20:09Z kai $
//

#include "Component/ModelManager.H"
#include "VectorHistField.H"
#include "Util/log.H"
#include "Util/Types.H"
#include "Util/sformat.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include  "GUI/XWinManaged.H"
#include "Devices/PingSonar.H"
#include <stdio.h>
#include <cstring>
#include <cstdlib>


// ######################################################################

int main(int argc, const char **argv)
{
  // Instantiate a ModelManager:
  ModelManager manager("Test Vector Field");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  int itsDim = 30;

  // Instantiate our various ModelComponents:
  nub::soft_ref<VectorHistField> vectorHistField(new VectorHistField
                                                 (manager,"VectorHistField",
                                                  "VectorHistField",itsDim,itsDim));

  nub::soft_ref<PingSonar> pingSonar(new PingSonar(manager,"PingSonar",                                           "PingSonar","/dev/ttyUSB0",3));

  manager.addSubComponent(vectorHistField);
  manager.addSubComponent(pingSonar);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv,"", 0, 0) == false) return(1);

  // let's get all our ModelComponent instances started:
  manager.start();

  int cnt=0;

  while(1)
  {
      std::vector<int> dists = pingSonar->getDists();
      std::vector<Point2D<float> > sensor;

      LINFO("getting dists %d,%d,%d",dists.at(0),dists.at(1),dists.at(2));

      sensor.push_back(Point2D<float> ((float)(dists.at(0)*itsDim/3000),150.0));
      sensor.push_back(Point2D<float> ((float)(dists.at(1)*itsDim/3000),90.0));
      sensor.push_back(Point2D<float> ((float)(dists.at(2)*itsDim/3000),30.0));

      Image<PixRGB<byte> >blobs;

      if(cnt++ > 20)
      {
          blobs = vectorHistField->updateField
            (sensor,Point2D<int>(0,0), 
             90.0F,0.0F, Point2D<int> (15,90),
             vectorHistField->obstacleTemplate(20.0F,10.0F));
          
      }

      else
          blobs = vectorHistField->updateField
            (sensor,
             Point2D<int>(0,0),0.0F, 0.0F, Point2D<int> (15,90),
             vectorHistField->obstacleTemplate(20.0F,10.0F));
      
      ofs->writeRGB(vectorHistField->plotGridField(30),sformat("Output%d",cnt));

      if (cnt >21)
        exit(1);


      usleep(1000);
  }


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
