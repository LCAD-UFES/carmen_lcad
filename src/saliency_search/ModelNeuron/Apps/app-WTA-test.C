//Test our WTA, SC map etc.
//////////////////////////////////////////////////////////////////////////
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
//////////////////////////////////////////////////////////////////////////
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////



#include "Component/ModelManager.H"
#include "Channels/InputFrame.H"
#include "Image/Image.H"
#include "Media/FrameSeries.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/FrameInfo.H"
#include "Util/Pause.H"
#include "Util/csignals.H"

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work
//#include "ModelNeuron/SupColliculusModule.H"
#include "ModelNeuron/NeuralFieldModule.H"
#include "ModelNeuron/SimStructures.H"

//allocate work queue
nsu::SimulationWorkQueue nsu::SimStructure::itsWorkQueue(4);

int submain(const int argc, const char** argv)
{
  volatile int signum = 0;
  catchsignals(&signum);
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  ModelManager manager("WTA Model Testing Program");

  //hook up our frameseries
  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);
  
  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);
  
  //hook up an SC module  
  //nub::ref<SupColliculusModule> sc(new SupColliculusModule(manager));
  //manager.addSubComponent(sc);

  //hook up an SC module  
  nub::ref<NeuralFieldModule> nf(new NeuralFieldModule(manager));
  manager.addSubComponent(nf);

  //parse command line and start manager
  if (manager.parseCommandLine(argc, argv, "NF or SC", 1, 1) == false)
    return(1);

  if (manager.getExtraArg(0).compare("NF") == 0)
    LINFO("NF module selected");
  //  manager.removeSubComponent(sc);
  //  else if (manager.getExtraArg(0).compare("SC") == 0)
  //manager.removeSubComponent(nf);
  else
    LFATAL("No such type");
  
  manager.start();
  
  ifs->startStream();

  int c = 0;
  PauseWaiter p;

  //simulation time
  SimTime timestep = SimTime::MSECS(1000/ifs->peekFrameSpec().frameRate);
  SimTime time = SimTime::ZERO();

  while (true)
    {
      if (signum != 0)
        {
          LINFO("quitting because %s was caught", signame(signum));
          return -1;
        }

      if (ofs->becameVoid())
        {
          LINFO("quitting because output stream was closed or became void");
          return 0;
        }

      if (p.checkPause())
        continue;

      const FrameState is = ifs->updateNext();
      if (is == FRAME_COMPLETE)
        break;

      GenericFrame input = ifs->readFrame();
      if (!input.initialized())
        break;
      
      //input to model and evolve
      time += timestep;
      if (manager.getExtraArg(0).compare("NF") == 0)
      {
        nf->setInput(input.asFloat(), -1);
        nf->update(time);
      }
      //else
      //{
      //  sc->setInput(input.asFloat(), 0);
      //  sc->update(time);
      //}
      LINFO("Time: %3.2f", time.msecs());

      //update to next frame
      const FrameState os = ofs->updateNext();

      //save output
      ofs->writeFrame(input, "input frame");
      if (manager.getExtraArg(0).compare("NF") == 0)
        ofs->writeRgbLayout(nf->getDisplay(), "NF Simulation");
      //else
      //  ofs->writeRgbLayout(sc->getDisplay(), "SC Simulation");
      
      if (os == FRAME_FINAL)
        break;
      
      LDEBUG("frame %d", c++);
      
      if (ifs->shouldWait() || ofs->shouldWait())
        Raster::waitForKey();
    }

  return 0;  
}
#else
int submain(const int argc, const char** argv) {return 0; }; 
#endif

int main(const int argc, const char **argv)
{
  try
    {
      return submain(argc, argv);
    }
  catch (...)
    {
      REPORT_CURRENT_EXCEPTION;
    }
  return 1;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
