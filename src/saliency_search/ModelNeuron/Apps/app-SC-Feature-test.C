//!gather statistics on sc model features

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
#include "Image/ImageSet.H"
#include "Image/Point2D.H"
#include "Image/Range.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/ColorOps.H"
#include "Image/ColorMap.H"
#include "Raster/Raster.H"
#include "Image/Layout.H"
#include "Util/StringUtil.H"
#include "Util/StringConversions.H"
#include "Util/SimTime.H"
#include "ModelNeuron/FreeViewingData.H"

#include <iostream>
#include <fstream>

//////////////////////////////////////////////////////////////////////////
// Fitting code params
//////////////////////////////////////////////////////////////////////////
const ModelOptionCateg MOC_SCFeatureTest = { MOC_SORTPRI_3, "SC-Feature Test Options" };

const ModelOptionDef OPT_MGZFiles = { MODOPT_ARG(std::string), "MGZFiles", &MOC_SCFeatureTest, OPTEXP_CORE,
                                      "A comma separated list of mgz files containing features", "mgz-files", '\0', "<name1,name2,...nameN>", ""};

const ModelOptionDef OPT_NormFile = { MODOPT_ARG(std::string), "NormFile", &MOC_SCFeatureTest, OPTEXP_CORE,
                                      "A file to both read and write norm values from.", "norm-file", '\0', "<filename>", ""};

const ModelOptionDef OPT_ShowPlot = { MODOPT_FLAG, "ShowPlot", &MOC_SCFeatureTest, OPTEXP_CORE,
                                      "Show a plot of all the center surround maps", "show-plot", '\0', "<bool>", "false"};

const ModelOptionDef OPT_SampleRate = { MODOPT_ARG(SimTime), "SampleRate", &MOC_SCFeatureTest, OPTEXP_CORE,
                                        "Sample rate of output", "sample-rate", '\0', "<SimTime>", "200Hz"};

const ModelOptionDef OPT_InputRate = { MODOPT_ARG(SimTime), "InputRate", &MOC_SCFeatureTest, OPTEXP_CORE,
                                       "Sample rate of input data", "input-rate", '\0', "<SimTime>", "200Hz"};

const ModelOptionDef OPT_ProbeFilename = { MODOPT_ARG(std::string), "ProbeFilename", &MOC_SCFeatureTest, OPTEXP_CORE,
                                           "Create a file where each column is the time output of each channel at the center pixel", 
                                           "probe-file", '\0', "<filename>", ""};

//////////////////////////////////////////////////////////////////////////
// main
//////////////////////////////////////////////////////////////////////////
int submain(const int argc, const char** argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  ModelManager manager("SC Feature test");

  //variables for all of our command line paramters
  OModelParam<std::string> itsMGZFiles(&OPT_MGZFiles, &manager);
  OModelParam<std::string> itsNormFile(&OPT_NormFile, &manager);
  OModelParam<std::string> itsProbeFilename(&OPT_ProbeFilename, &manager);
  OModelParam<bool> itsPlot(&OPT_ShowPlot, &manager);
  OModelParam<SimTime> itsSampleRate(&OPT_SampleRate, &manager);
  OModelParam<SimTime> itsInputRate(&OPT_InputRate, &manager);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  //parse command line and start manager
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  manager.start();
  
  //load up all the Mgz files 
  std::vector<FreeData> freedata = loadMGZFromList(itsMGZFiles.getVal());

  //set up a probe if desired
  bool const useProbe = (itsProbeFilename.getVal().empty()) ? false : true;
  std::ofstream of;
  if (useProbe)
    of.open(itsProbeFilename.getVal().c_str());

  //get some info about the channels
  SubMapInfo mapinfo = freedata[0].info;
  
  //get some constants
  bool const showPlot = itsPlot.getVal();
  uint const numMaps = freedata[0].info.numMaps();
  float const fskip = itsInputRate.getVal().hertz() / itsSampleRate.getVal().hertz();

  //if (fskip != uint(fskip))
  if (fskip != 1)
    LFATAL("This feature is currently disabled because it because it gives poor results without filtering. Re-run ezvision to create the maps at your desired sampling rate.");

  uint const skip = (uint)fskip;

  Range<float> const outRange(0.0F, 255.0F);

  LINFO("%d MGZ files, each with %d maps", (uint)freedata.size(), numMaps);
  
  //loop through all the frames of each file, display, and calcualte min-max values over all features
  std::vector<FreeData>::iterator data(freedata.begin()), dataend(freedata.end());
  std::vector<Range<float> > itsMapRange(numMaps);

  while (data != dataend)
  {
    LINFO("Processing FreeData: '%s'", data->name.c_str());
    
    bool go = true;
    uint framecount = 0;

    while (go)
    {
      if (framecount % skip == 0)
      {
        //get all maps for one time step
        ImageSet<byte> maps(numMaps);
        for (uint ii = 0; ii < numMaps; ++ii)
        {
          //get the current map for this frame
          GenericFrame frame = data->features->readFrame();
          
          //quit if the frame isn't initialized
          if (!frame.initialized())
          {
            LINFO("features exhausted on the %d frame and %d map.", framecount, ii);
            
            go = false;
            if (ii != 0)
              LFATAL("Error: input frame exhausted but we are not at the beginning of a timestep");    
            
            break;   
          }
          
          //get the frame as a float and update our the overal range
          Image<float> featureMap = frame.asGrayF32();
          Range<float> currRange = rangeOf(featureMap);
          itsMapRange[ii].merge(currRange);

          if (useProbe)
            of << featureMap.getVal(featureMap.getWidth() / 2, featureMap.getHeight()) << " ";
          
          //if we are plotting, remap to the current min/max and store as a byte image
          if (showPlot)
          {
            LINFO("Reading Feature map : %s with size : %dx%d", mapinfo.getMapName(ii).c_str(), featureMap.getWidth(), featureMap.getHeight());
            featureMap = remapRange(featureMap, itsMapRange[ii], outRange);
            Image<byte> bimg = featureMap;
            maps[ii] = bimg;
          }
        }

        if (useProbe)
          of << std::endl;
        
        //create an output for one timestep
        if ((showPlot) && go)
        {
          Layout<PixRGB<byte> > plot;
          Layout<PixRGB<byte> > rows[mapinfo.getMaxChannelGroup()+1];
          
          for (uint ii = 0; ii < maps.size(); ++ii)
          {
            Image<PixRGB<byte> > map = colorize(maps[ii], ColorMap::JET());
            rows[mapinfo.getChannelGroup(ii)] = hcat(rows[mapinfo.getChannelGroup(ii)], map);
            LINFO("%s (%3.5f - %3.5f)", mapinfo.getMapName(ii).c_str(), itsMapRange[ii].min(), itsMapRange[ii].max());
          }
          
          for (uint ii = 0; ii < mapinfo.getMaxChannelGroup()+1; ++ii)
            plot = vcat(plot, rows[ii]);
                    
          //add in the video frame info if any
          if (data->retinal.get() != NULL)
          {
            //get the current map for this frame
            GenericFrame frame = data->retinal->readFrame();
            if (frame.initialized())
            {
              Image<PixRGB<byte> > img = frame.asRgb();
              float fac = (float)img.getWidth() / (float)plot.getDims().w();
              Dims d(img.getWidth() / fac, img.getHeight() / fac); 
              img = rescaleBilinear(img, d);
              plot = vcat(img, plot);
            }
            else
              LINFO("Retinal image exhausted before maps!");
          }

          //update to next frame
          ofs->updateNext();
          
          //save maps
          ofs->writeRgbLayout(plot, "Channels");
          
          //wait if desired
          if (ofs->shouldWait())
            Raster::waitForKey();
        }
      }
      else
      {
        for (uint ii = 0; ii < numMaps; ++ii)
          if (data->features->skipFrame() == false)
          {
            if (ii == 0)
              go = false;
            else
              LFATAL("Maps exhausted without finishing reading all the maps for this frame!");
          }
       
        if (data->retinal.get() != NULL)
          data->retinal->skipFrame();
      }
      LINFO("Frames: %d", framecount++);
    }

    //add an extra empty line after each movie
    if (useProbe)
      of << std::endl;

    ++data;
  }

  of.close();

  //write to a file (will automatically merge with any contents the norm file might have
  writeChanNormFile(itsNormFile.getVal(), freedata[0], itsMapRange);

  //display the final results
  for (uint ii = 0; ii < itsMapRange.size(); ++ii)
    LINFO("%s (%3.5f - %3.5f)", mapinfo.getMapName(ii).c_str(), itsMapRange[ii].min(), itsMapRange[ii].max());
  
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
