/*! a test for the different neural sim modules */
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
#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/Layout.H"
#include "Image/DrawOps.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Media/FrameSeries.H"
#include "Util/StringConversions.H"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/SimStructureOpts.H"
#include "ModelNeuron/SimUnit.H"
#include "ModelNeuron/PlotBuffer.H"
#include "ModelNeuron/NeuralSimUtils.H"

//modules are registered by just including their header. 
#include "ModelNeuron/LowPass.H"
#include "ModelNeuron/LowpassNeuron.H"
#include "ModelNeuron/Rectify.H"
#include "ModelNeuron/IZNeuron.H"
#include "ModelNeuron/Circuits.H"

//main
// ############################
int submain(const int argc, const char** argv)
{  
  ModelManager manager("SimUnit Test");
  OModelParam<uint> itsDepth(&OPT_SCProbeDepth, &manager);
  OModelParam<bool> itsDisplayOutput(&OPT_SCUseDisplayOutput, &manager);
  OModelParam<SimTime> itsSimTimeStep(&OPT_SCSimTimeStep, &manager);
  
  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);
  
  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);
  
  if (manager.parseCommandLine(argc, argv, 
                               "<excite strength> <inhibit strenth> "
                               "<comp module> <decoder>", 4, 4) == false)
    return 1;
  
  manager.start();
  ifs->startStream();
  
  //setup our simulation time
  GenericFrameSpec gfs(ifs->peekFrameSpec());
  SimTime time(SimTime::ZERO()), timestepstim(SimTime::HERTZ(gfs.frameRate)), timestepmodel(itsSimTimeStep.getVal());
  
  //setup one of our SimUnits
  double excite(0),inhibit(0);
  convertFromString(manager.getExtraArg(0), excite);
  convertFromString(manager.getExtraArg(1), inhibit);
  
  //set timestep in model factory components
  nsu::setParameter(nsu::SimUnit::Factory::instance(), timestepmodel);
  nsu::setParameter(nsu::NeuralDecoder::Factory::instance(), timestepmodel);

  nsu::SimUnit* su = nsu::SimUnit::Factory::instance().create(manager.getExtraArg(2));
  nsu::NeuralDecoder* nd = nsu::NeuralDecoder::Factory::instance().create(manager.getExtraArg(3)); 
  
  su->setDecoderPost(*nd);
 
  nsu::PlotBuffer pb;
  pb.setSamplingRate(timestepstim);

  while (true)
    {      
      //get the input
      const FrameState is = ifs->updateNext();
      if (is == FRAME_COMPLETE)
        break;
      
      GenericFrame input = ifs->readFrame();
      if (!input.initialized())
        break;

      time += timestepstim;
      //LINFO("Time: %3.10f",time.msecs());

      //input to system
      Image<double> img = input.asFloat();
      const double inp = img.getVal(0,0);

      su->input(excite * inp);
      su->input(-1.0 * inhibit * inp);

      su->evolve(time);
      pb.push(*su, inp, 0, itsDepth.getVal(), itsDisplayOutput.getVal());
    }
  
  LINFO("Simulation complete - Drawing");
  
  if (ofs->becameVoid())
    {
      LINFO("quitting because output stream was closed or became void");
      return 0;
    }
  
  ofs->updateNext();
  Layout<PixRGB<byte> > oimage = pb.draw(false, 640, pb.getTotal()*150);
  ofs->writeRgbLayout(oimage, "output");
  
  if (ofs->shouldWait())
    Raster::waitForKey();
  
  delete su;
  delete nd;
  
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
