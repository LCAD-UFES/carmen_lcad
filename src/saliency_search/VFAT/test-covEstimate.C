/*!@file VFAT/test-covEstimate.C quick estimator for coveriance matrix
 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/test-covEstimate.C $
// $Id: test-covEstimate.C 15310 2012-06-01 02:29:24Z itti $
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
//This is the start of the execution path for the test alg.

#define TYPE float

#include "Raster/Raster.H"
#include "Util/Timer.H"
#include "Util/log.H"
#include "Util/readConfig.H"
#include "VFAT/covEstimate.H"
#include <stdlib.h>
#include <sys/types.h>

//! This is the configFile name
char* configFile;
//! This is the configFile object
//readConfig configIn(25);
//! generic pixel
PixRGB<float> pix;
//! number of items if training
int itemNumber;

int main(int argc, char* argv[])
{
  // get test image
  Image<byte> input = Raster::ReadGray(argv[1]);
  Image<float> finput = input;


  // create operating objects
  //configIn.openFile("NPclassify.conf");
  // convert test image to vector format
  LINFO("COUNTING SAMPLES");
  int vCount = 0;
  for(int x = 0; x < finput.getWidth(); x++)
  {
    for(int y = 0; y < finput.getHeight();y++)
    {
      if(finput.getVal(x,y) < 1.0F)
      {
        // find sample size from image
        vCount++;
      }
    }
  }

  std::vector<TYPE> _vinput(vCount,0);
  TYPE t = 0.0F;
  TYPE* tfloat = &t;
  std::vector<TYPE*> _vTinput(vCount,tfloat);
  std::vector<std::vector<TYPE> > vinput(2,_vinput);
  std::vector<std::vector<TYPE*> > vinputP(2,_vTinput);
  std::vector<std::vector<TYPE> > voutput(2,_vinput);

  LINFO("ASSEMBLING SAMPLE VECTOR SIZE %" ZU ,vinput[0].size());
  vCount = 0;
  for(int x = 0; x < finput.getWidth(); x++)
  {
    for(int y = 0; y < finput.getHeight();y++)
    {
      if(finput.getVal(x,y) < 1.0F)
      {
        // insert x and y into vector
        vinput[0][vCount] =  255-x;
        vinputP[0][vCount] = &vinput[0][vCount];
        vinput[1][vCount] =  y;
        vinputP[1][vCount] = &vinput[1][vCount];
        vCount++;
      }
    }
  }

  LINFO("RUNNING COVESTIMATE");
  Timer tim;
  tim.reset();
  int t1,t2;
  int t0 = tim.get();  // to measure display time
  covHolder<TYPE> chold;
  chold.resize(2,vCount,0.0F);

  covEstimate<TYPE> CE(vinputP,chold);
  t1 = tim.get();
  t2 = t1 - t0;
  std::cout << ">\t(0) TIME: " << t2 << "ms\n";
  //CE.printDebug();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cout << ">\tTIME: " << t2 << "ms\n";

  // (1) find mean value (centroid) in matrix
  CE.run();

  t1 = tim.get();
  t2 = t1 - t0;
  std::cout << ">\tTIME: " << t2 << "ms\n";
  Image<float> final = CE.returnCovSlice(0,1,300);
  //Raster::VisuFloat(translate,0,sformat("translate-%s.pgm",argv[1]));
  //Raster::VisuFloat(rotate,0,sformat("rotate-%s.pgm",argv[1]));
  Raster::VisuFloat(final,0,sformat("final-%s.pgm",argv[1]));
};


