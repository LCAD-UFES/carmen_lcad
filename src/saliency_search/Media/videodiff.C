/*!@file Media/videodiff.C compute average difference between video frames */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/videodiff.C $
// $Id: videodiff.C 5311 2005-08-12 18:29:27Z rjpeters $
//

#include "Component/ModelManager.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/MathOps.H"  // for absDiff()
#include "Image/Pixels.H"
#include "Media/MPEGStream.H"
#include "Media/MediaOpts.H"
#include "Util/Types.H"

#include <iostream>

int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Video Diff");

  // Instantiate our various ModelComponents:
  nub::soft_ref<InputMPEGStream>
    ims(new InputMPEGStream(manager, "Input MPEG Stream", "InputMPEGStream"));
  manager.addSubComponent(ims);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "<input.mpg>", 1, 1) == false)
    return(1);

  // NOTE: this could now be controlled by a command-line option
  // --preload-mpeg=true
  manager.setOptionValString(&OPT_InputMPEGStreamPreload, "true");

  // do post-command-line configs:
  ims->setFileName(manager.getExtraArg(0));

  // let's get all our ModelComponent instances started:
  manager.start();

  // main loop:
  int frame = 0; Image<byte> prev; double sum = 0.0, sumsq = 0.0;
  while(1)
    {
      if ((frame % 50) == 0) LINFO("Processing frame %d...", frame);

      // load new frame:
      Image< PixRGB<byte> > input = ims->readRGB();
      if (input.initialized() == false) break;  // end of movie

      // compute difference:
      Image<byte> current = luminance(input);
      if (prev.initialized())
        {
          Image<byte> diff = absDiff(prev, current);
          double meandiff = mean(diff);
          sum += meandiff; sumsq += meandiff * meandiff;
        }

      // ready for next frame:
      prev = current; frame ++;
    }

  // compute average and std: we use the standard trick E[(X-E[X])^2]
  // = E[X^2] - E[X]^2, and apply the small-sample correction to the
  // variance (divide by n-1 instead of n):
  double n = double(frame) - 1.0; // number of differences taken
  double avg = sum / n;
  double stdev = sqrt((sumsq - sum * sum / n) / (n - 1.0));

  std::cout<<manager.getExtraArg(0)<<": "<<frame<<" frames, diff = "<<avg
           <<" +/- "<<stdev<<std::endl;

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
