/*!@file VFAT/szvision.C  simplified version of vision.C with feature analysis
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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/szvision.C $
// $Id: szvision.C 14376 2011-01-11 02:44:34Z pez $
//

// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itti itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################

#include "Component/ModelManager.H"
#include "VFAT/featureClusterVision.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEventQueueConfigurator.H"

int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Attention Model");

  // Instantiate our various ModelComponents:
  nub::soft_ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(manager));
  manager.addSubComponent(seqc);

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::soft_ref<StdBrain> brain(new StdBrain(manager));
  manager.addSubComponent(brain);

  // feature analysis part of model
  const std::string name = "featureCluster";
  const std::string tag = "fCV";
  Image< PixRGB<byte> > input;

  std::vector<covHolder<double> > covHolder;
  //unsigned int covHolderSize;

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv,
                               "[0--5 args]", 0, 5) == false)
    return(1);

  nub::soft_ref<featureClusterVision<float> >
    fCV(new featureClusterVision<float>(manager,name,tag,brain,ifs,
                                        manager.getExtraArg(0)));
  manager.addSubComponent(fCV);
  nub::soft_ref<SimEventQueue> seq = seqc->getQ();

  // let's get all our ModelComponent instances started:
  manager.start();
  // main loop:

  while(1) {
    // write outputs or quit?
    bool gotcovert = false;
    if (seq->check<SimEventWTAwinner>(0)) gotcovert = true;
    const FrameState os = ofs->update(seq->now(), gotcovert);

    if (os == FRAME_NEXT || os == FRAME_FINAL)
    {
      SimModuleSaveInfo sinfo(ofs, *seq);
      brain->save(sinfo);
      int foo = ifs->frame();
      std::string Myname;
      std::string a = manager.getExtraArg(0);
      std::string b = ".";
      char c[100];
      if(foo == 1)
        ; //init = false;
      if(foo < 10)
        sprintf(c,"00000%d",foo);
      else if(foo < 100)
        sprintf(c,"0000%d",foo);
      else if(foo < 1000)
        sprintf(c,"000%d",foo);
      else if(foo < 10000)
        sprintf(c,"00%d",foo);
      else if(foo < 100000)
        sprintf(c,"0%d",foo);
      else
        sprintf(c,"%d",foo);
      Myname = a + b + c;
      // NOTE: added '0' at the end here because there was no value
      // matching the final '%d'
      LINFO("RUNNING FRAME %d NTARG %d",foo,0);
      // Upload a frame to the classifier
      //input = rescale(input,input.getWidth()/2,input.getHeight()/2);
      fCV->fCVuploadImage(input,Myname);
      fCV->fCVstandAloneFeatureTest(manager.getExtraArg(0));

      // classify and cluster this image
      /*fCV->fCVsaccadeTest(manager.getExtraArg(1),
                              manager.getExtraArg(2),
                              manager.getExtraArg(3),
                              manager.getExtraArg(4));*/

      // get back image data
    }

    if (os == FRAME_FINAL)
      break;

    // why do we handle the output before the input? That's because
    // both the input and output frame series will switch to the next
    // frame at the exact same time, if input and output framerates
    // are equal. When the input series switches to a new frame, it
    // will reset any drawings that were obtained on the previous
    // frame. So we need to make sure we have saved those results
    // before we read the new frame in.

    // if we displayed a bunch of images, let's pause:
    if (ifs->shouldWait() || ofs->shouldWait())
      Raster::waitForKey();

    // read new image in?
    const FrameState is = ifs->update(seq->now());
    if (is == FRAME_COMPLETE) break; // done
    if (is == FRAME_NEXT || is == FRAME_FINAL) // new frame
    {
      input = ifs->readRGB();

      // empty image signifies end-of-stream
      if (input.initialized())
        {
          rutz::shared_ptr<SimEventInputFrame>
            e(new SimEventInputFrame(brain.get(), GenericFrame(input), 0));
          seq->post(e); // post the image to the brain

          // show memory usage if in debug mode:
          if (MYLOGVERB >= LOG_DEBUG)
            SHOWMEMORY("MEMORY USAGE: frame %d t=%.1fms", ifs->frame(),
                       seq->now().msecs());
        }
    }

    // evolve brain:
    const SimStatus status = seq->evolve();

    if (SIM_BREAK == status) // Brain decided it's time to quit
      break;
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
