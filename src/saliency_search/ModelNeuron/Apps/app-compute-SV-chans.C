/*!@file ModelNeuron/Apps/app-compute-SV-chans.C */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Apps/app-compute-SV-chans.C $

#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"

#include "Channels/InputFrame.H"
#include "GUI/XWinManaged.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/ShapeOps.H"
#include "Image/Layout.H"
#include "Image/Normalize.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "SpaceVariant/SCTransformModule.H"
#include "SpaceVariant/SpaceVariantOpts.H"
#include "SpaceVariant/SVChanLevels.H"
#include "Media/FrameSeries.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/FrameInfo.H"
#include "Util/StringUtil.H"
#include "Util/Pause.H"
#include "Util/csignals.H"
#include "Util/Timer.H"
#include "Media/MediaOpts.H"
#include "Media/MgzEncoder.H"

#include <fstream>

extern const ModelOptionDef OPT_SpaceVariantChanTypes = 
{ MODOPT_ARG(std::string), "SpaceVariantChanTypes", &MOC_SPACEVARIANT, OPTEXP_CORE,
  "the channel type when creating a space variant image", 
  "channelsv-types", '\0', "channel string", "ICO" };

extern const ModelOptionDef OPT_SVTakeAbs = 
{ MODOPT_FLAG, "SVTakeAbs", &MOC_SPACEVARIANT, OPTEXP_CORE,
  "Should we take the absolute value of the channel output?", 
  "channelsv-takeabs", '\0', "<bool>", "true" };

extern const ModelOptionDef OPT_MgzFileName = 
{ MODOPT_ARG(std::string), "MgzFileName", &MOC_SPACEVARIANT, OPTEXP_CORE,
  "mgz file output name", 
  "mgz-name", '\0', "<file name>", "" };

int submain(const int argc, const char **argv)
{
  volatile int signum = 0;
  catchsignals(&signum);

  ModelManager manager("compute-SV-chans application");

  OModelParam<std::string> itsChannels(&OPT_SpaceVariantChanTypes, &manager);
  OModelParam<std::string> itsOutName(&OPT_MgzFileName, &manager);
  OModelParam<SVChanLevels> itsLevels(&OPT_SpaceVariantChanScales, &manager);
  OModelParam<bool> itsTakeAbs(&OPT_SVTakeAbs, &manager);

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::soft_ref<SCTransformModule> sc(new SCTransformModule(manager));
  manager.addSubComponent(sc);

  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  manager.start();

  rutz::shared_ptr<MgzEncoder> itsOutput;
  if (itsOutName.getVal().empty() == false)
    itsOutput = rutz::make_shared(new MgzEncoder(itsOutName.getVal(), 9));


  SVChanLevels levels = itsLevels.getVal();
  std::string channels = itsChannels.getVal();
  float const scale = 6.9348F;//standard C-S difference from data

  LINFO("Processing %d levels with channels '%s' at a C-S ratio of %3.2f", levels.numLevels(), channels.c_str(), scale);
  
  //write submap info
  std::string fbase;
  if (itsOutName.getVal().compare("display") == 0)
    fbase = "display";
  else
  {
    std::vector<std::string> tokens;
    split(itsOutName.getVal(), ".", std::back_inserter(tokens));
    fbase = tokens[0];
  }

  const std::string fname(fbase + "-SubMapInfo");
  std::ofstream itsOutFile(fname.c_str());
  if (itsOutFile.is_open() == false)
    LFATAL("Cannot open '%s' for writing", fname.c_str());
  else
  {
    std::string::const_iterator iter(channels.begin()), end(channels.end());
    while (iter != end)
    {
      char c = *iter++;
      switch ( c )
      {
      case 'I':
      {
        for (uint ii = 0; ii < levels.numLevels(); ++ii)
          itsOutFile << "Intensity lev: " << levels.getVariance(ii) << 
            " delta: " << (uint)scale << std::endl;
      }
      break;
        
      case 'C':
      {
        for (uint ii = 0; ii < levels.numLevels(); ++ii)
          itsOutFile << "RGColor lev: " << levels.getVariance(ii) << 
            " delta: " << (uint)scale << std::endl;
        
        for (uint ii = 0; ii < levels.numLevels(); ++ii)
          itsOutFile << "BYColor lev: " << levels.getVariance(ii) << 
            " delta: " << (uint)scale << std::endl;
      }
      break;
      
      case 'O':
      {
        LFATAL("NOT IMPLEMENTED YET");
      }
      break;
      
      default :
        LFATAL("No such channel type");
      }
    }

    itsOutFile.close();
  }
  
  
  //main loop
  ifs->startStream();
  int c = 0;
  PauseWaiter p;

  Timer timer(1000);

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

      //convert it to DKL Munoz
      Image<float> rgimg, byimg, lumimg;
      getDKLM(input.asRgbU8(), rgimg, byimg, lumimg);

      //output for each frame
      ImageSet<float> chanOutput;

      std::string::const_iterator iter(channels.begin()), end(channels.end());
      while (iter != end)
      {
        ImageSet<float> out;//each channels output
        char c = *iter++;
        switch ( c )
        {
        case 'I':
        {
          sc->transformDoGPyramid(lumimg, out, levels);
        }
        break;

        case 'C':
        {
          ImageSet<float> outrg, outby;

          sc->transformDoGPyramid(rgimg, outrg, levels);
          sc->transformDoGPyramid(byimg, outby, levels);

          for (uint ii = 0; ii < outrg.size(); ++ii)
            out.push_back(outrg[ii]);

          for (uint ii = 0; ii < outby.size(); ++ii)
            out.push_back(outby[ii]);
        }
        break;

        case 'O':
        {
          LFATAL("NOT IMPLEMENTED");
        }
        break;
        
        default :
          LFATAL("No such channel type");
        }

        for (uint ii = 0; ii < out.size(); ++ii)
        {
          if (itsTakeAbs.getVal())
            out[ii] = abs(out[ii]);
          
          chanOutput.push_back(out[ii]);
        }
      }
      
      const FrameState os = ofs->updateNext(); 

      if (fbase.compare("display") == 0)
      {
        ofs->writeFrame(input, "input");
      }
      
      for (uint ii = 0; ii < chanOutput.size(); ++ii)
      {
        Image<float> fimg = chanOutput[ii];
        if (fbase.compare("display") == 0)
        {
          ofs->writeFrame(GenericFrame(fimg, FLOAT_NORM_0_255), "Submap"+toStr(ii));
          ofs->writeFrame(GenericFrame(sc->inverseTransform(fimg), FLOAT_NORM_0_255), "Inverse Submap"+toStr(ii));
        }
        else
        {
          itsOutput->writeFrame(GenericFrame(fimg, FLOAT_NORM_PRESERVE));
        }
        
        if (os == FRAME_FINAL)
          break;
      }
      
      LDEBUG("frame %d", c++);
      
      if (ifs->shouldWait() || ofs->shouldWait())
        Raster::waitForKey();    
    }

  if (itsOutput.is_valid()) itsOutput->close();
  
  return 0;
}

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
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
