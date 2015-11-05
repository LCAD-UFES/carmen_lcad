/*!@file
  Robots/Beobot2/Navigation/FOE_Navigation/test-FOE_PopulationHeadingMap.C
  find the FOE using Lappe & Rauschecker 1993's Population Heading Map
  algorithm  */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: $
// $Id: $


#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/ColorOps.H"

#include "Util/Timer.H"

#include "Robots/Beobot2/Navigation/FOE_Navigation/PopulationHeadingMap.H"

// ######################################################################
// Function declarations

// ######################################################################
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // instantiate a model manager:
  ModelManager manager("Test Optic Flow");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  //nub::ref<PopulationHeadingMap> phm(new PopulationHeadingMap(manager));
  rutz::shared_ptr<PopulationHeadingMap> 
    phm(new PopulationHeadingMap(400.0));

  manager.exportOptions(MC_RECURSE);

  if (manager.parseCommandLine((const int)argc, (const char**)argv,
                               "", 0, 0) == false)
    return(1);

  // get some options
  // if(manager.numExtraArgs() >  0)
  //   {
  //     outFilename = manager.getExtraArg(0);
  //     LINFO("save to: %s", outFilename.c_str());
  //   }

  // let's do it!
  manager.start();

  Timer timer(1000000);
  timer.reset();  // reset the timer
  int frame = 0;
  bool keepGoing = true;

  const FrameState is = ifs->updateNext();
  ifs->updateNext();
  Image<PixRGB<byte> > img1 = ifs->readRGB(); ifs->updateNext();
  Image<PixRGB<byte> > img2 = ifs->readRGB();
  Image<PixRGB<byte> > image;
  Image<float> stim;
  if (is == FRAME_COMPLETE) keepGoing = false;

  float flen = 400;
  while (keepGoing)
    {
      if (ofs->becameVoid())
        {
          LINFO("quitting because output stream was closed or became void");
          break;
        }

      // get the frame 
      const FrameState is = ifs->updateNext();
      if (is == FRAME_COMPLETE) break; // done receiving frames
      image = ifs->readRGB();

      // compute the Focus of Expansion
      phm->setFocalLength(flen); //flen += 5.0;
      phm->getFOE(Image<byte>(luminance(img1)),Image<byte>(luminance(img2)));
      //phm->getFOE(Image<byte>(luminance(image)));

      ofs->writeRGB(image, "Optic Flow");
      const FrameState os = ofs->updateNext();
      Raster::waitForKey();

      //Raster::WriteRGB(currImage, sformat("image%d.ppm",frame));

      if (os == FRAME_FINAL) break;

      frame++;
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
