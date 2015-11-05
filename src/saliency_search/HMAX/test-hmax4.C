/*!@file HMAX/test-hmax4.C Test Hmax class and compare to original code */

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
// Primary maintainer for this file: Daesu Chung <dchung@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/test-hmax4.C $
// $Id: test-hmax4.C 6191 2006-02-01 23:56:12Z rjpeters $
//

#include "GUI/XWindow.H"
#include "HMAX/Hmax.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Transforms.H"
#include "Raster/Raster.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <iostream>
#include <unistd.h>

// number of orientations to use in Hmax
#define NORI 4

int main(const int argc, const char **argv)
{
  if (argc != 2)
    { std::cerr<<"USAGE: test-hmax <file>"<<std::endl; exit(1); }

  // get an Hmax object:
  std::vector<int> scss(5);
  scss[0] = 0; scss[1] = 2; scss[2] = 5; scss[3] = 8; scss[4] = 12;
  std::vector<int> spss(4);
  spss[0] = 4; spss[1] = 6; spss[2] = 9; spss[3] = 12;
  Hmax hmax(NORI, spss, scss);

  // read the image:
  Image<byte> input = Raster::ReadGray(argv[1], RASFMT_PNM);

  // let's get an Xwindow:
  XWindow xwin(input.getDims());

  // pass image through Hmax model:
  Image<float> inputf = input;   // convert image to floats
  Image<float> c2resp = hmax.getC2(inputf);
  float mi, ma; getMinMax(c2resp, mi, ma);
  LINFO("min=%f max=%f", mi, ma);

  // display C2 response in an X window:
  c2resp = scaleBlock(c2resp, input.getDims());
  inplaceNormalize(c2resp, 0.0F, 255.0F);
  Image<byte> result = c2resp; // convert to byte
  xwin.drawImage(result);

  sleep(10);
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
