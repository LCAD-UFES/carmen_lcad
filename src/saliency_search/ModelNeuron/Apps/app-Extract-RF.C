//!extract a small patch of a movie or feature from an RF

//////////////////////////////////////////////////////////////////////////
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
//
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/app-SC-Feature-test.C $

#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/Point2D.H"
#include "Image/CutPaste.H"
#include "Raster/Raster.H"
#include "Util/SimTime.H"
#include "Util/StringUtil.H"
#include "ModelNeuron/FreeViewingData.H"
#include "ModelNeuron/SCFitOpts.H"

#include <iostream>
#include <fstream>

//////////////////////////////////////////////////////////////////////////
// Fitting code params
//////////////////////////////////////////////////////////////////////////
const ModelOptionCateg MOC_RFExtract = { MOC_SORTPRI_3, "RF Extract Options" };

const ModelOptionDef OPT_MGZFiles = { MODOPT_ARG(std::string), "MGZFiles", &MOC_RFExtract, OPTEXP_CORE,
                                      "Name of an mgz file containing features or stimuli.", "mgz-file", '\0', "<fname>", ""};

const ModelOptionDef OPT_OutputDir = { MODOPT_ARG(std::string), "Outputdir", &MOC_RFExtract, OPTEXP_CORE,
                                       "Patch output directory.", "output-dir", '\0', "<path/to/output>", ""};

const ModelOptionDef OPT_SignalFile = { MODOPT_ARG(std::string), "SignalFile", &MOC_RFExtract, OPTEXP_CORE,
                                        "Filename of a file that contains a single 1 on samples where patches should be extracted and zeros elsewhere", "signal-file", '\0', "<filename>", ""};

const ModelOptionDef OPT_RFSampleRate = { MODOPT_ARG(SimTime), "RFSampleRate", &MOC_RFExtract, OPTEXP_CORE,
                                          "The rate at which to sample from the RF.", "rf-sample-rate", '\0', "<SimTime>", "200Hz"};

const ModelOptionDef OPT_RFTimeWindow = { MODOPT_ARG(SimTime), "RFTimeWindow", &MOC_RFExtract, OPTEXP_CORE,
                                          "Collect frames this far back.", "rf-time-window", '\0', "<SimTime>", "100ms"};

const ModelOptionDef OPT_RFRetinal = { MODOPT_FLAG, "RFRetinal", &MOC_RFExtract, OPTEXP_CORE,
                                       "Sample the retinal (otherwise maps)?", "use-retinal", '\0', "<bool>", "true"};

const ModelOptionDef OPT_StimPPD = { MODOPT_ARG(Point2D<float>), "StimPPD", &MOC_RFExtract, OPTEXP_CORE,
                                     "pixels per degree of stimulus", "stimulus-ppd", '\0', "<ppdx,ppdy>", "23.64,20.84"};

//////////////////////////////////////////////////////////////////////////
// main
//////////////////////////////////////////////////////////////////////////
int submain(const int argc, const char** argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  ModelManager manager("SC Feature test");

  //variables for all of our command line paramters
  OModelParam<std::string> itsMGZFile(&OPT_MGZFiles, &manager);
  OModelParam<std::string> itsOutputDir(&OPT_OutputDir, &manager);
  OModelParam<std::string> itsSignalFile(&OPT_SignalFile, &manager);
  OModelParam<SimTime> itsRFSampleRate(&OPT_RFSampleRate, &manager);
  OModelParam<SimTime> itsDataRate(&OPT_FreeDataTime, &manager);
  OModelParam<SimTime> itsRFTimeWindow(&OPT_RFTimeWindow, &manager);
  OModelParam<Point2D<float> > itsProbeDeg(&OPT_StimLocation, &manager);
  OModelParam<Point2D<float> > itsPpd(&OPT_StimPPD, &manager);
  OModelParam<float> itsProbeSize(&OPT_RFSize, &manager);
  OModelParam<bool> itsUseRetinal(&OPT_RFRetinal, &manager);
  OModelParam<Dims> itsRetinalDims(&OPT_RetinalDims, &manager);

  //parse command line and start manager
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  manager.start();
  
  //load up all the Mgz files 
  FreeData freedata;
  getMGZFreeViewingRetinal(freedata, itsMGZFile.getVal());

  //get some info about the channels
  SubMapInfo mapinfo = freedata.info;

  float const fskip = itsDataRate.getVal().hertz() / itsRFSampleRate.getVal().hertz();
  if ((fskip < 0) || (fskip != float((int)fskip)))
    LFATAL("RF sampling must be lower than and divide evenly into the rate of the data");


  uint const maxsize = (uint)(itsRFTimeWindow.getVal().secs() * itsDataRate.getVal().hertz());
  
  //error if we can't open
  std::ifstream signalFile(itsSignalFile.getVal().c_str());
  if (!signalFile.is_open())
    LFATAL("Error: cannot open the file");
  
  //gobble up the file
  std::vector<bool> itsSignal;
  std::string line;
  while (getline(signalFile, line))
    itsSignal.push_back((bool)fromStr<uint>(line));
  
  //probe position
  PixelUnitTransform pixelTransform(itsPpd.getVal().i, 
                                    itsPpd.getVal().j,
                                    itsRetinalDims.getVal());
  
  Point2D<int> itsRfPos = pixelTransform.deg2Pix(itsProbeDeg.getVal());

  int itsRfSize = int(itsProbeSize.getVal() * (pixelTransform.ppdx + pixelTransform.ppdy) / 2.0F);
  Dims itsPatchDims(itsRfSize, itsRfSize);
  
  bool const useretinal = itsUseRetinal.getVal();
  
  if ((freedata.retinal.get() == NULL) && useretinal)
    LFATAL("No retinal mgz found!");
  
  //get the filename base
  std::string fbase = itsMGZFile.getVal();
  std::vector<std::string> toks;
  split(fbase, ".", back_inserter(toks));
  fbase = toks[0];
  split(fbase, "/", back_inserter(toks));
  fbase = toks.back();
  fbase = itsOutputDir.getVal() + "/" + fbase;

  //run through the movie
  bool go = true;
  uint framecount = 0;
  bool hassignal(true), hasmgz(true);
  std::deque<Image<PixRGB<byte> > > itsRetWin(maxsize, Image<PixRGB<byte> >(itsRetinalDims.getVal(), ZEROS));
  uint eventCount = 0;
  while (go)
  {    
    bool collect = false;

    //get signal value
    if (hassignal)
    {
      if (framecount < itsSignal.size())
        collect = itsSignal[framecount];
      else
      {
        hassignal = false;
        collect = false;
        LINFO("signal data exhausted on the %d frame.", framecount);
      }
    }

    //collect the movie images
    if (hasmgz)
    {
      if (useretinal)
      {
        GenericFrame frame = freedata.retinal->readFrame();
        
        if (!frame.initialized())
        {
          hasmgz = false;
          itsRetWin.push_back(Image<PixRGB<byte> >(itsRetinalDims.getVal(), ZEROS));
          itsRetWin.pop_front();
          LINFO("mgz exhausted on the %d frame.", framecount);
        }
        else
        {
          if (frame.getDims() != itsRetinalDims.getVal())
            LFATAL("Retinal dimensions do not match image size!");
          
          Image<PixRGB<byte> > img = frame.asRgb();
          itsRetWin.push_back(img);
          itsRetWin.pop_front();
        } 
      }
      else
      {
        LFATAL("Cannot yet process channels, unimplemented option.");
      }
    }

    //write to disk if signal is high
    if (collect)
    {
      //save the files
      for (uint ii = 0; ii < itsRetWin.size(); ii += fskip)
      {

        //generate the output name
        std::string filename = fbase + "-" + toStr(eventCount) + "-" + toStr(ii) + ".png";

        //get the patch around the rf
        Point2D<int> p = itsRfPos;
        p.i -= itsRfSize / 2;
        p.j -= itsRfSize / 2;
        Image<PixRGB<byte> > img = crop(itsRetWin[ii], p, itsPatchDims, true);
        
        //save it to disk 
        Raster::WriteRGB(img, filename);
        LINFO("Collecting event %d at %d to file %s", eventCount, framecount, filename.c_str());
      }
      ++eventCount;
    }
    
    if (!hassignal && !hasmgz)
      go = false;

    ++framecount;
  }
          
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
/* indent-tabs-mode: nil */
/* End: */
