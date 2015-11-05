/*!@file Learn/test-SOFM.C test the SOFM
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/test-SOFM.C $
// $Id: test-SOFM.C 12627 2010-01-22 02:00:51Z lior $
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/ColorOps.H"
#include "Image/ShapeOps.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/Transforms.H"
#include "Media/FrameSeries.H"
#include "Raster/Raster.H"
#include "Util/log.H"
#include "Util/MathFunctions.H"
#include "Learn/SOFM.H"
#include "GUI/DebugWin.H"


int main(int argc, char** argv)
{

  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Test SOFM");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // Parse command-line:
  if (manager.parseCommandLine((const int)argc, (const char**)argv, "", 0, 0) == false)
    return(1);

  SOFM sofm("sofm.net", 3, 256, 256);
  sofm.RandomWeights();
//  sofm.ZeroWeights();

  // main loop:
  int ii=0;
  while(1) {

    //generate a random rgb value
    std::vector<double> input(3);
    input[0] = randomUpToIncluding(255);
    input[1] = randomUpToIncluding(255);
    input[2] = randomUpToIncluding(255);

    LINFO("Input is (%f,%f,%f)", input[0], input[1], input[2]);

    Image<PixRGB<byte> > inputImg(32,32,ZEROS);
    inputImg.clear(PixRGB<byte>(input[0], input[1], input[2]));
    ofs->writeRGB(inputImg, "InputImg");

    sofm.setInput(input);
    sofm.Propagate();

    double val;
    Point2D<int> p = sofm.getWinner(val);

    LINFO("Winner at %ix%i value %f", p.i, p.j, val);
    printf("%i %f\n",ii, val);
    fflush(stdout);


    Image<float> sofmOut = sofm.getMap();

    drawCircle(sofmOut, p, 6, 255.0F);

    ofs->writeRGB(sofmOut, "SOFM_act_map");

    Image<PixRGB<byte> > weights = sofm.getWeightsImage();

    drawCircle(weights, p, 6, PixRGB<byte>(0,255,0));

    ofs->writeRGB(weights, "SOFM_weights");


    sofm.SetLearningRate(ii);
    sofm.organize(input);
    //inplaceNormalize(SMap, 0.0F, 255.0F);

    ii++;

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
