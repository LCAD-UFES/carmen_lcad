/*!@file Neuro/SimulationViewerEyeMvtNeuro.C get the saliency values at a
   proble location relative the the current eye position for comparing
   with a neural response */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: David J Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeMvtNeuro.C $

#include "Neuro/SimulationViewerEyeMvtNeuro.H"
#include "Component/ModelComponent.H"
#include "Component/OptionManager.H"
#include "Channels/ChannelOpts.H" // for LevelSpec option
#include "Image/FilterOps.H" // for lowPass3()
#include "Image/ShapeOps.H" // for rescale()
#include "Image/MathOps.H" // for getLcalAvg()
#include "Image/Transforms.H" //for composite()
#include "Image/ColorOps.H" //for toRGB()
#include "Image/DrawOps.H" 
#include "Image/Layout.H"
#include "Image/ImageSetOps.H"
#include "Raster/Raster.H"  //for readGray()
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Psycho/EyeData.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/StringUtil.H" //for split()
#include "Util/StringConversions.H" //for toStr()
#include "rutz/trace.h"

#include <fstream>
#include <iostream>

// ######################################################################
SimulationViewerEyeMvtNeuro::
SimulationViewerEyeMvtNeuro(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName) :
  SimulationViewerAdapter(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsSaveTraj(&OPT_SVsaveTraj, this),
  itsSaveMegaCombo(&OPT_SVEMsaveMegaCombo, this),
  itsDelayCacheSize(&OPT_SVEMdelayCacheSize, this),
  itsEyePatchSize(&OPT_SVpatchSize, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsUseDiagColor(&OPT_SVEMuseDiagColors, this),
  itsMaxComboWidth(&OPT_SVEMmaxComboWidth, this),
  itsShiftInput(&OPT_ShiftInputToEye, this), 
  itsUseSpaceVariantBoundary("UseSpaceVariantBoundary", this, false),
  itsOutFname(&OPT_SVEMNoutFName, this),
  itsProbe(&OPT_SVEMNprobeLocation, this),
  itsRFSize(&OPT_SVEMNrfSize, this),
  itsRFMaskName(&OPT_SVEMNrfMaskName, this),
  itsNeuronFileName(&OPT_SVEMNneuroFileName, this),
  itsDisplayTime(&OPT_SVEMNdisplayTime, this),
  itsDelaySpike(&OPT_SVEMNdelayNeuron, this),
  itsBufferLength(&OPT_SVEMNplotBufLength, this),
  itsSalScale(&OPT_SVEMNSalScale, this),
  itsNeuroScale(&OPT_SVEMNNeuroScale, this),
  itsVisRFName(&OPT_SVEMNNeuroRFVisFile, this),
  itsMotRFName(&OPT_SVEMNNeuroRFMotFile, this),
  itsMotWindow(&OPT_SVEMNMotWindow, this),
  itsVisWindow(&OPT_SVEMNVisWindow, this),
  itsVisOffset(&OPT_SVEMNVisOffset, this),
  itsShowSCMap(&OPT_SVEMNShowSCMap, this),
  itsSCMapDims(&OPT_SVEMNSCMapDims, this),
  itsDelayCache(), itsHeadSM(), itsDrawings(), itsSCMap(), itsRFMask(),
  itsVisRf(), itsVisRfBuf(), itsMotRf(), itsMotSamp(0), 
  itsVisSamp(0), itsVisSampOff(0),
  itsOutFile(0), itsNeuronFile(0), itsRate(SimTime::ZERO()), 
  itsPpdx(0.0F), itsPpdy(0.0F),
  itsProbeP(), itsRawCenter(), itsRFSizeP(0), itsInputDims(0,0), itsOgInputDims(0,0), 
  itsSpikeVals(), itsSpikePos(0), itsSalPb(), itsSpikePb(),
  itsHposPb(), itsVposPb(), itsXlabel(), itsTitle(),
  itsSCTransform()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // disable IOR:
  LINFO("NOTE: disabling IOR and SE, selecting FixedSaccadeController");
  getManager().setOptionValString(&OPT_IORtype, "None");
  getManager().setOptionValString(&OPT_ShapeEstimatorMode, "None");

  // select an eyetrack EyeHeadController:
  getManager().setOptionValString(&OPT_EyeHeadControllerType, "EyeTrack");

  // change default to --display-additive=false; user can still
  // override this on the command-line
  getManager().setOptionValString(&OPT_SVdisplayAdditive, "false");
}

// ######################################################################
SimulationViewerEyeMvtNeuro::~SimulationViewerEyeMvtNeuro()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

 if (itsUseSpaceVariantBoundary.getVal() && !itsShiftInput.getVal())
   LFATAL("You must shift the input (--shift-input) if you are using this simulation viewer "
          "with a space variant transform");

 //!add a delay, if any, to our spike vector
 itsSpikeVals.resize(itsDelaySpike.getVal(), 0.0F);

 //!set the ploting buffer lengths
 itsSalPb.reset(itsBufferLength.getVal(), 0.0F, itsSalScale.getVal(), itsRate);
 itsSpikePb.reset(itsBufferLength.getVal(), 0.0F, itsNeuroScale.getVal(), itsRate);
 itsHposPb.reset(itsBufferLength.getVal(), 0.0F, 0.0F, itsRate);
 itsVposPb.reset(itsBufferLength.getVal(), 0.0F, 0.0F, itsRate);

 //setup delay cache
 if (itsDelayCacheSize.getVal())
   itsDelayCache.setMaxSize(itsDelayCacheSize.getVal());
 
 // open output file:
 if (itsOutFname.getVal().empty() == false)
   {
     if (itsOutFile) delete itsOutFile;
     itsOutFile = new std::ofstream(itsOutFname.getVal().c_str());
     if (itsOutFile->is_open() == false)
       LFATAL("Cannot open '%s' for writing", itsOutFname.getVal().c_str());
   }
 
 // open neuron file
 if (itsNeuronFileName.getVal().empty() == false)
   {
     if (itsNeuronFile) delete itsNeuronFile;
     itsNeuronFile = new std::ifstream(itsNeuronFileName.getVal().c_str());
     if (itsNeuronFile->is_open() == false)
       LFATAL("Cannot open '%s' for reading",
              itsNeuronFileName.getVal().c_str());
     else
       {
         while (! itsNeuronFile->eof())
           {
             float val;
             (*itsNeuronFile) >> val;
             itsSpikeVals.push_back(val);
           }
         itsNeuronFile->close();
       }
   }

 //read visual rf
 if (itsVisRFName.getVal().empty() == false)
   {
     readImageSet(itsVisRf, itsVisRFName.getVal());
     if (itsVisRf.size() > 0)
       LINFO("Loading saved visual RF spike accumulator");
     else
       LINFO("Starting new visual RF spike accumulator");
   }
 
 //read the motor rf
 if (itsMotRFName.getVal().empty() == false)
   {
     if (Raster::fileExists(itsMotRFName.getVal()))
       {
         itsMotRf = Raster::ReadFloat(itsMotRFName.getVal());
         LINFO("Loading saved motor RF spike accumulator");
       }
     else
       LINFO("Starting new motor RF spike accumulator");
   }

 //load a weight mask if one exists
 if (itsRFMaskName.getVal().empty() == false)
   itsRFMask = Raster::ReadFloat(itsRFMaskName.getVal());

 //set our title and xlabel
 itsXlabel = itsBufferLength.getVal().toString() + " window";
 itsTitle = "RF center: " + toStr<Point2D<float> >(itsProbe.getVal()) + 
   ", RF size: " + toStr<float>(itsRFSize.getVal()); 
 
 SimulationViewer::start1();
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::stop1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

 //write out the vis rf
 if (itsVisRFName.getVal().empty() == false)
   {
     LINFO("Saving visual RF set: %s", itsVisRFName.getVal().c_str());
     saveImageSet(itsVisRf, FLOAT_NORM_PRESERVE, itsVisRFName.getVal());
   }

 //write out the motor rf
 if (itsMotRFName.getVal().empty() == false)
   {
     LINFO("Saving motor RF set: %s", itsMotRFName.getVal().c_str());
     Raster::WriteFloat(itsMotRf, FLOAT_NORM_PRESERVE, itsMotRFName.getVal());
   }

 if (itsOutFile) 
   { 
     itsOutFile->close();
     delete itsOutFile; 
     itsOutFile = 0; 
   }
 
 if (itsNeuronFile) 
  {
    delete itsNeuronFile; 
    itsNeuronFile = 0;
  }
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::
doEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // any new input frame from the retina? If so, let's initialize or
  // clear all our drawings:
  itsDrawings.resize(e->frame().getDims(), true);
    
  //check dimensions of retinal output
  if (itsInputDims != e->frame().getDims()) 
    {
      LINFO("Input Image Dimensions have changed");
      itsInputDims =  e->frame().getDims(); //dims of retinal image
      itsOgInputDims = e->getRawInputDims(); //dims of input
      
      itsRawCenter = Point2D<int>(itsInputDims.w()/2, itsInputDims.h()/2);
      
      itsHposPb.reset(itsBufferLength.getVal(), 
                      0.0F, (float)itsOgInputDims.w() / itsPpdx, itsRate);
      itsVposPb.reset(itsBufferLength.getVal(), 
                      0.0F, (float)itsOgInputDims.h() / itsPpdy, itsRate);
    }

  //if we are in shifting mode, we can perform the shift on each new retinal image
  if (itsShiftInput.getVal() && itsShowSCMap.getVal() && (itsPpdx != 0.0))
    itsSCMap = transformToAnatomicalSC(itsInput);
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  // Let's update our sliding caches with the current SM/AGM/etc:
  const Image<float> currsm = getMap(q, false);//automatically inverse transformed if requested

  //if our map dimensions are different then our spike triggered accumulators, reset them
  if ((itsMotRFName.getVal().empty() == false) && (itsMotRf.getDims() != itsOgInputDims))
    itsMotRf.resize(itsOgInputDims, ZEROS);
  
  if ((itsVisRFName.getVal().empty() == false) && (itsVisRf.back().getDims() != currsm.getDims()))
    itsVisRf.clear();

  // if we have a delay cache, we push the SM into it and grab what comes out
  if (itsDelayCacheSize.getVal())
    {
      // let's push the SM into our delay sliding cache:
      itsDelayCache.push_back(currsm);
      
      //grab the latest
      if (int(itsDelayCache.size()) >= itsDelayCacheSize.getVal())
        itsHeadSM = itsDelayCache.front();
      else
        itsHeadSM.resize(currsm.getDims(), true);  // blank map
    }
  else
    itsHeadSM = currsm;  // no delay cache
    
  // any eye-tracker action?
  SeC<SimEventEyeTrackerData> e = q.check<SimEventEyeTrackerData>(this);

  while(e.is_valid()) {
    // parse that event:
    const rutz::shared_ptr<EyeData> trackCurrPos = e->data();
    const int tnum = e->trackerNum();
    setDegtoPixels(e->trackerPpd().ppdx(), e->trackerPpd().ppdy());
    
    //if our tracking rate changed make some updates
    SimTime rate = e->trackerHz();
    if (itsRate != rate)
      {
        itsRate = rate;
        
        itsHposPb.reset(itsBufferLength.getVal(), 
                        0.0F, (float)itsOgInputDims.w() / itsPpdx, itsRate);

        itsVposPb.reset(itsBufferLength.getVal(), 
                        0.0F, (float)itsOgInputDims.h() / itsPpdy, itsRate);

        itsSalPb.reset(itsBufferLength.getVal(), 
                       0.0F, itsSalScale.getVal(), itsRate);
        
        itsSpikePb.reset(itsBufferLength.getVal(), 
                         0.0F, itsNeuroScale.getVal(), itsRate);

        //set motor window in samples
        itsMotSamp = uint(itsMotWindow.getVal().secs() * itsRate.hertz());
        itsVisSamp = uint(itsVisWindow.getVal().secs() * itsRate.hertz());
        itsVisSampOff = uint(itsVisOffset.getVal().secs() * itsRate.hertz());
      }
    
    //lets make sure we only have one tracker
    if (tnum > 0)          
      LFATAL("SimulationEyeMvtNeuro only supports one eye tracker");
    
    // select a drawing color & size:
    PixRGB<byte> col(1,1,1);
    // note: we use 'almost black' (1,1,1) instead of black so that we can later
    // composite our drawings with various images, with pure black
    // then being treated as transparent):
    if (!itsUseDiagColor.getVal())
      {
        col = e->trackerColor();
        if (trackCurrPos->isInSaccade()) col += byte(64);//will clamp/saturate 
        if (trackCurrPos->isInBlink()) col -= byte(128);//will clamp/saturate 
      }
    else //diagnostic colors
      {
        byte r = 1, g = 1, b = 1;
        if (trackCurrPos->isInSaccade()) 
          g = 255;//green for saccade
        else if (trackCurrPos->isInSmoothPursuit()) 
          {r = 255;b = 255;} //magenta for smooth pursuit
        else if (trackCurrPos->isInFixation()) 
          b = 255;// blue for fixation
        if (trackCurrPos->isInCombinedSaccade()) 
          {b = 255;g = 255;}//cyan for combined

        col = PixRGB<byte>(r,g,b);    //default black
        if (trackCurrPos->isInBlink()) 
          col-=byte(128); // let's just make the cursor darker
      }
  
    // convert to retinal coordinates (accounting for any shifting,
    // embedding, etc):
    Point2D<int>  rawTrackerPos = trackCurrPos->position();
    Point2D<int>  trackerPos = (itsShiftInput.getVal()) ? itsRawCenter : rawTrackerPos;    

    //if we are not in shifting mode, 
    if (~itsShiftInput.getVal() && itsShowSCMap.getVal())
    {
      Point2D<int> shiftPos = itsRawCenter - rawTrackerPos;
      itsSCMap = shiftClean(itsInput, shiftPos.i, shiftPos.j);
      itsSCMap = transformToAnatomicalSC(itsSCMap);
    }

    // draw small patch at current eye position:
    if (itsEyePatchSize.getVal())
      drawPatchBB(itsDrawings, trackerPos, itsEyePatchSize.getVal(), 
                  col, PixRGB<byte>(1));
    
    //display simulation time
    if (itsDisplayTime.getVal())
      {
        const SimpleFont sf = SimpleFont::fixedMaxWidth(itsInputDims.w() / 90);
        writeText(itsDrawings, Point2D<int>(0, 0),
                  q.now().toString().c_str(), 
                  PixRGB<byte>(1, 1, 1),PixRGB<byte>(255,255,255),sf);
      }

    double rfsal = -1.0;
    //if we have a valid probe use it, otherwise use an rf mask
    if (itsProbeP != Point2D<int>(-1,-1))
      {
        //get our rf position
        Point2D<int> rfPos = itsProbeP;
        toRetinal(rfPos);
        rfPos += trackerPos;
   
        LINFO("Collecting receptive field sample at (%d,%d) at time: %s", 
              rfPos.i, rfPos.j, q.now().toString().c_str());
        
        //draw a small circle around the receptive field
        if (itsRFSizeP > 0)
          {
            if (itsTransform->validTransforms() && itsInverseRetinal.getVal())
              drawCircle(itsDrawings, rfPos, itsInputDims.w()/20, col, 2);
            else
              drawCircle(itsDrawings, rfPos, itsRFSizeP,col, 2);
          }
        else
          drawPatchBB(itsDrawings, rfPos, itsEyePatchSize.getVal(), 
                      col, PixRGB<byte>(1));
      
        //collect saliency values
        // get the map level to scale things down:
        int sml = itsLevelSpec.getVal().mapLevel();
        Point2D<int> p(rfPos.i >> sml, rfPos.j >> sml);
        
        const int rad = itsRFSizeP >> sml;
        if (itsHeadSM.coordsOk(p)) //is our probe valid
          rfsal = getLocalAvg(itsHeadSM, p, rad);
      }
    else if (itsRFMaskName.getVal().empty() == false) //do we have a rf mask
      {
        if (itsRFMask.getDims() != itsInputDims)
          LFATAL("Receptive field weight mask must be the same size as"
                 "the input.");

        if (itsRFMask.getDims() != itsHeadSM.getDims())
          LFATAL("Output map must be scaled to RF Mask size.");
        
        //if using mask, we must be inverting transforms or not use them
        if (itsTransform->validTransforms() && itsInverseRetinal.getVal())
          LINFO("When using masks, everything must be in stimulus coordinates, so "
                "if you are trying to use a transform, you must inverse it with "
                "--inverse-retinal ");

        LINFO("Collecting receptive field weighted sample at time: %s",
              q.now().toString().c_str());

        rfsal = 0.0;
        Image<float>::const_iterator witer(itsRFMask.begin()),
          wend(itsRFMask.end()), iiter(itsHeadSM.begin());
        while (witer != wend)
          rfsal += (*witer++) * (*iiter++);
      }
    else
      LINFO("Probe is invalid and no RFMask has been supplied, "
            "storing -1.0 as saliency  value");
    
    //add our current sal val to our plot buffer
    itsSalPb.push(q.now(), rfsal); 

    //if we read in neuron data, add it to the plot buffer
    bool didspike = false;
    if (itsNeuronFile)
      {
        itsSpikePb.push(q.now(), itsSpikeVals[itsSpikePos]);

        if (itsSpikeVals[itsSpikePos] > 0)
          didspike = true; 
        ++itsSpikePos;
      }
    
    //for capturing our spike triggered stimulus ensemble
    if (itsVisRFName.getVal().empty() == false)
      {
        itsVisRfBuf.push_back(itsHeadSM);//this will always be the display-type
        while (itsVisRfBuf.size() > (itsVisSamp + itsVisSampOff))
          itsVisRfBuf.pop_front();
      }
    
    //if we spike, grab the stimulus from our queue and add it to the count
    if (didspike && (itsVisRfBuf.size() == (itsVisSamp + itsVisSampOff)))
      { 
        ImageSet<float> temp = itsVisRfBuf.subSet(itsVisSampOff,
                                                  itsVisRfBuf.size());

        if (itsVisRf.isNonEmpty()) itsVisRf += temp; else itsVisRf = temp;
      }

    //if we are at the onset of a saccade, see if there are any spikes
    //and store that count at the saccadic enpoint
    if (didspike && itsMotRf.initialized() && 
        trackCurrPos->hasSaccadeTargetData() && !trackCurrPos->isInBlink())
      {
        //get the endpoint
        Point2D<int> dp = trackCurrPos->saccadeTarget();
        dp.j = itsOgInputDims.h() - dp.j;//flip around
        
        //get the number of spikes that occured in a small window
        //around saccade onset
        uint spos = itsSpikePos - itsMotSamp;
        if (spos < 0) spos = 0;
        uint epos = itsSpikePos + itsMotSamp;
        if (epos >= itsSpikeVals.size()) epos = itsSpikeVals.size() - 1;
        
        for (uint i = spos; i <= epos; ++i)
          itsMotRf[dp] += itsSpikeVals[i];
      }

    //grab our horizontal and vertical eye position for plotting
    Point2D<float> eyeDeg((float)rawTrackerPos.i / itsPpdx, 
                          ((float)itsOgInputDims.h() - (float)rawTrackerPos.j) / itsPpdy);
                          
    itsHposPb.push(q.now(), eyeDeg.i);
    itsVposPb.push(q.now(), eyeDeg.j);
    
    //should we output samples
    if (itsOutFile) 
      (*itsOutFile) << toStr<float>(rfsal) << std::endl;

    // any more events like that in the queue?
    e = q.check<SimEventEyeTrackerData>(this);
  }//end eye tracker data
}

// ######################################################################
Image< PixRGB<byte> > SimulationViewerEyeMvtNeuro::getTraj()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // make a composite of the input + the drawings:
  Image< PixRGB<byte> > comp = composite(itsDrawings, itsInput);
  const int linewidth = comp.getWidth() / 400;
  
  //if we have a mega combo
  Dims ldims;
  Image<PixRGB<byte> > linecomp;
  Layout< PixRGB<byte> > layout;
  if (itsSaveMegaCombo.getVal())
    {
      // let's get the current saliency map (normalized):
      const Dims dims = itsInput.getDims();

      //rescale if necessary
      if (itsHeadSM.initialized()) 
        itsHeadSM = rescaleOpt(itsHeadSM, dims, itsDisplayInterp.getVal());
      else 
        itsHeadSM.resize(dims, true); // blank
      Image< PixRGB<byte> > smc = toRGB(Image<byte>(itsHeadSM));
      
      // make a composite of the instantaneous SM + the drawings:
      Image< PixRGB<byte> > smcomp = composite(itsDrawings, smc);
      
      drawLine(comp, Point2D<int>(dims.w()-1, 0), 
               Point2D<int>(dims.w()-1, dims.h()-1),
               PixRGB<byte>(255, 255, 0), linewidth);
      
      drawLine(smcomp, Point2D<int>(0, 0), 
               Point2D<int>(0, dims.h()-1),
               PixRGB<byte>(255, 255, 0), linewidth);
      
      //input right of salmap 
      layout = hcat(comp, smcomp);
      if (itsShowSCMap.getVal())
        layout = hcat(layout, rescaleBilinear(itsSCMap, itsInput.getDims()));

      //get our layout dims
      ldims = Dims(layout.getWidth(), int(layout.getHeight()/2.5F));

      //if we are doing a combo plot draw a line of saliency values over time
      linecomp = itsSalPb.draw(ldims.w(), 
                               ldims.h(),
                               itsTitle.c_str(), //probe location
                               "Sal", //y label
                               itsXlabel.c_str(),  //window length
                               PixRGB<byte>(1,1,1),//black line
                               0, false);//number of x axis tick marks
    }
  else
    {
      layout = Layout< PixRGB<byte> >(comp);
      if (itsShowSCMap.getVal())
        layout = hcat(layout, rescaleBilinear(itsSCMap, itsInput.getDims()));
      
      //get our layout dims
      ldims = Dims(layout.getWidth(), int(layout.getHeight()/2.5F));
    }

  //create some eye position plots
  Image<PixRGB<byte> > 
    eyeposh = itsHposPb.draw(ldims.w(), 
                             ldims.h(),
                             "Horizontal eye position", //title
                             "Deg", //y label
                             "",  //x label
                             PixRGB<byte>(1,1,1),//green line
                             0, false);//number of x axis tick marks
  
  Image<PixRGB<byte> > 
    eyeposv = itsVposPb.draw(ldims.w(), 
                             ldims.h(),
                             "Vertical eye position", //probe location
                             "Deg", //y label
                             "",  //x label
                             PixRGB<byte>(1,1,1),//green line
                             0, false);//number of x axis tick marks

  //if we have neural data, plot it as well
  if (itsNeuronFile) 
    {
      Image<PixRGB<byte> > 
        linespk = itsSpikePb.draw(ldims.w(), 
                                  ldims.h(),
                                  "", //title
                                  "Sp/S", //ylabel
                                  "", //xlabel
                                  PixRGB<byte>(1,1,255), //blue line
                                  0, true); //number of x axis tick marks

      linecomp = (linecomp.initialized())?
        composite(linecomp,linespk, PixRGB<byte>(255,255,255)):
        linecomp = linespk;
    }

  //combine all our elements and render
  layout = vcat(layout, vcat(vcat(eyeposh, eyeposv),linecomp));
  Image<PixRGB<byte> > ret = layout.render();

  //add a border to seperate movie and line plots
  drawLine(ret, Point2D<int>(0, comp.getHeight() - 1), 
           Point2D<int>(ldims.w()-1, comp.getHeight() - 1),
           PixRGB<byte>(255, 255, 0), linewidth);
  
  drawLine(ret, Point2D<int>(0, comp.getHeight()), 
           Point2D<int>(ldims.w() - 1, comp.getHeight()),
           PixRGB<byte>(255, 255, 0), linewidth);
  
  // make sure image is not unreasonably large:
  const int x = itsMaxComboWidth.getVal();
  const int y = int((float)x / (float)ret.getWidth() * (float)ret.getHeight());
  
  if (ret.getWidth() > itsMaxComboWidth.getVal())
    ret = rescaleBilinear(ret, Dims(x, y));
  
  return ret;
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::save1(const ModelComponentSaveInfo& sinfo)
{
GVX_TRACE(__PRETTY_FUNCTION__);

// update the trajectory:
 Image< PixRGB<byte> > res = getTraj();
 
 // save results?
 if (itsSaveMegaCombo.getVal() || itsSaveTraj.getVal())
   {
     // get the OFS to save to, assuming sinfo is of type
     // SimModuleSaveInfo (will throw a fatal exception otherwise):
     nub::ref<FrameOstream> ofs =
       dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;
     
     ofs->writeRGB(res, "T", 
                   FrameInfo("SimulationViewerEyeMvtNeuro trajectory",SRC_POS));
     
    }
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::setDegtoPixels(const float& ppdx, 
                                                 const float& ppdy)
{
  if ( (ppdx != itsPpdx) && (ppdy != itsPpdy) )
    {
      itsPpdx = ppdx;
      itsPpdy = ppdy;

      if (itsShowSCMap.getVal())
      {
        const float avgppd = (itsPpdx + itsPpdy) / 2;
        LogPolarTransform lpt(itsInputDims, itsSCMapDims.getVal(), 1 / avgppd, 0.2);
        itsSCTransform = lpt;
        LINFO("SC Transform initialized");
        if (itsInput.size() > 0)
          itsSCMap = transformToAnatomicalSC(itsInput);
      }
      
      //get in pixels our probe position and its size
      Point2D<float> p = itsProbe.getVal();
      
      if ((p.i == -1.0F) || (p.j == -1.0F))
        itsProbeP = Point2D<int>(-1,-1);
      else
        itsProbeP = Point2D<int>(int(itsPpdx * p.i),int(-1.0F * itsPpdy * p.j));
      
      itsRFSizeP = int( (itsPpdx + itsPpdy)/2.0F * itsRFSize.getVal());
    }
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::
readImageSet(ImageSet<float>& set, const std::string filename)
{
  std::string base, ext;
  prepFileName(filename, base, ext);

  for (uint i = 0; 1; ++i)
    {
      std::string name = base + toStr<uint>(i) + "." + ext;
      if (Raster::fileExists(name))
        {
          LINFO("Loading image %d in set: %s",i, name.c_str());
          set.push_back(Raster::ReadFloat(name));
        }
      else
        break;
    }
}

// ######################################################################
void SimulationViewerEyeMvtNeuro::
saveImageSet(const ImageSet<float>& set,
             const int flags, std::string fname,const RasterFileFormat ftype)
{
  std::string base, ext;
  prepFileName(fname, base, ext);
  for (uint i = 0; i < set.size(); ++i)
    {
      std::string name = base + toStr<uint>(i) + "." + ext;
      LINFO("Writing image %d in set: %s",i, name.c_str());
      Raster::WriteFloat(set[i], FLOAT_NORM_PRESERVE, name);
    }
}


// ######################################################################
void 
SimulationViewerEyeMvtNeuro::prepFileName(const std::string& name, 
                                          std::string& base, std::string& ext)
{
  std::vector<std::string> tok;
  split(name, ".", std::back_inserter(tok));
  ext = (tok.size() < 2) ? ".pfm" : tok.back();
  base = (tok.size() < 2) ? tok[0] : join(tok.begin(), tok.end()-1,".");
}

// ######################################################################
Image<PixRGB<byte> > SimulationViewerEyeMvtNeuro::transformToAnatomicalSC(const Image<PixRGB<byte> > & img)
{
  return logPolarTransform(itsSCTransform, img, PixRGB<byte>(0,0,0));  
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
