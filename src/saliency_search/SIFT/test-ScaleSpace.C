/*! @file SIFT/test-ScaleSpace.C test operation of ScaleSpace class */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/test-ScaleSpace.C $
// $Id: test-ScaleSpace.C 6191 2006-02-01 23:56:12Z rjpeters $
//

#include "Raster/Raster.H"
#include "Image/ShapeOps.H"
#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Util/sformat.H"
#include "SIFT/ScaleSpace.H"
#include "SIFT/VisualObject.H"

#include <fstream>

int main(const int argc, const char **argv)
{
  // check command-line args:
  if (argc < 3 || argc > 4)
    LFATAL("USAGE: test-ScaleSpace <image.png> <result.png> [result.vo]");

  // get input image:
  Image< PixRGB<byte> > colim = Raster::ReadGray(argv[1]);

  // create a VisualObject, which will create a bunch of scalespaces
  // and extract SIFT keypoints:
  VisualObject vo("TestObject", "", colim);

  // compute and save keypoint image:
  Image< PixRGB<byte> > kp = vo.getKeypointImage();
  Raster::WriteRGB(kp, argv[2], RASFMT_PNG);

  if (argc > 3)
    {
      // save the VisualObject:
      std::ofstream outf(argv[3]);
      outf<<vo; outf.close();
    }

  return 0;
}
