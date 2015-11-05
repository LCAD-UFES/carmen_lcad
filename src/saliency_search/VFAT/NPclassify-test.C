/*!@file VFAT/NPclassify-test.C  Test the non-parametric classifier */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/NPclassify-test.C $
// $Id: NPclassify-test.C 6593 2006-05-16 20:33:52Z rjpeters $
//

// ############################################################
// ############################################################
// ##### ---NPclassify---
// ##### non-parametric classifier:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Vidhya Navalpakkam - navalpak@usc.edu
// ##### partners full name - email
// ############################################################
// ############################################################

//This is the start of the execution path for the NPclassify test alg.
#include "Util/log.H"
#include "Util/readConfig.H"
#include "VFAT/NPclassify.H"
#include "Raster/Raster.H"
#include "Image/All.H"
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>

//! This is the configFile name
char* configFile;
//! This is the configFile object
readConfig configIn(25);


int main(int argc, char* argv[])
{
  LFATAL("this program doesn't compile at the moment");
#if 0
  // start timer
  time_t t1,t2;
  (void) time(&t1);

  // get test image
  Image<byte> input = Raster::ReadGray(PGM,argv[1]);
  Image<float> finput = input;

  // create operating objects
  readConfig.openFile("NPclassify.conf");
  NPclassify(readConfig);
  std::vector<long> feature(2,0);
  std::vector<std::vector<long> > features(50,feature);
  long featureCount = 0;

  // convert test image to vector format
  for(int x = 0; x < finput.getWidth(); x++)
  {
    for(int y = 0; y < finput.getHeight();y++)
    {
      if(finput.getVal() < 128.0F)
      {
        // resize vector if needed
        if(features.size() >= featureCount)
          features.resize((featureCount+50));
        // insert x and y into vector
        features[featureCount][0] = x;
        features[featureCount][1] = y;
      }
    }
  }

  NPclassify.addSpace(features);
  NPclassify.classifySpaceNew();
#endif
}





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
