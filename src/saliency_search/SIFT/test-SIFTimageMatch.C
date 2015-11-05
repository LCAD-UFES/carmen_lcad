/*! @file SIFT/test-SIFTimageMatch.C test SIFT matching of two images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/test-SIFTimageMatch.C $
// $Id: test-SIFTimageMatch.C 13002 2010-03-11 19:05:16Z irock $
//

#include "Raster/Raster.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"
#include <iostream>

int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_DEBUG;

  // check command-line args:
  if (argc < 5 || argc > 6)
    LFATAL("USAGE: test-SIFTimageMatch <Simple|KDTree|KDBBF> <image1.ppm> "
           "<image2.ppm> <result.ppm> [<fused.ppm>]");

  VisualObjectMatchAlgo voma(VOMA_SIMPLE);
  if (strcmp(argv[1], "KDTree") == 0) voma = VOMA_KDTREE;
  else if (strcmp(argv[1], "KDBBF") == 0) voma = VOMA_KDTREEBBF;
  else if (strcmp(argv[1], "Simple") != 0)
    LFATAL("Unknown matching method %s", argv[0]);

  // get input image:
  Image< PixRGB<byte> > colim1 = Raster::ReadRGB(argv[2]);
  Image< PixRGB<byte> > colim2 = Raster::ReadRGB(argv[3]);

  // create visual objects:
  rutz::shared_ptr<VisualObject> vo1(new VisualObject(argv[2], "", colim1));
  rutz::shared_ptr<VisualObject> vo2(new VisualObject(argv[3], "", colim2));

  // compute the matching keypoints:
  VisualObjectMatch match(vo1, vo2, voma);

  LINFO("Found %u matches between %s and %s", match.size(), argv[2], argv[3]);

  // let's prune the matches:
  uint np = match.prune();
  LINFO("Pruned %u outlier matches.", np);

  // show our final affine transform:
  std::cerr<<match.getSIFTaffine();

  LINFO("getKeypointAvgDist = %f", match.getKeypointAvgDist());
  LINFO("getAffineAvgDist = %f", match.getAffineAvgDist());
  LINFO("getScore = %f", match.getScore());

  if (match.checkSIFTaffine() == false)
    LINFO("### Affine is too weird -- BOGUS MATCH");

  // get an image showing the matches:
  Image< PixRGB<byte> > mimg = match.getMatchImage(1.0F);

  // save the result:
  Raster::WriteRGB(mimg, std::string(argv[4]));

  // do we want a fused image?
  if (argc >= 6)
    {
      Image< PixRGB<byte> > fimg = match.getFusedImage(0.25F);
      Raster::WriteRGB(fimg, std::string(argv[5]));
    }

  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
