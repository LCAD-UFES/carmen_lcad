/*!@file Neuro/SimulationViewerEyeMvt.C comparison between saliency and
  human eye movements */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeMvt.C $
// $Id: SimulationViewerEyeMvt.C 14887 2011-08-20 06:23:15Z pohetsn $
//

#include "Neuro/SimulationViewerEyeMvt.H"

#include "Channels/ChannelBase.H"
#include "Channels/ChannelOpts.H" // for LevelSpec option
#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Image/ColorOps.H"    // for contrastModulate()
#include "Image/CutPaste.H"    // for concatX()
#include "Image/DrawOps.H"     // for colGreyCombo()
#include "Image/FilterOps.H"   // for lowPass3()
#include "Image/MathOps.H"     // for scramble()
#include "Image/Transforms.H"  // for segmentObjectClean(), contour2D()
#include "Image/ShapeOps.H"    // for rescale()
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SpatialMetrics.H"
#include "Psycho/EyeData.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

#include <fstream>
#include <sstream>
#include <iostream>

// for some reason this is not recognized when it appears in NeuroOpts.C
const ModelOptionDef OPT_SMhistoryQlen =
  { MODOPT_ARG(uint), "SMhistoryQ", &MOC_DISPLAY, OPTEXP_CORE,
    "Keep a historical queue of salieny maps (one per new input frame), "
    "and report the history of saliency values over the entire queue "
    "and at a saccade target location, for each saccade",
    "sm-history-qlen", '\0', "<uint>", "0" };

// ######################################################################
SimulationViewerEyeMvt::
SimulationViewerEyeMvt(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsSaveTraj(&OPT_SVsaveTraj, this),
  itsSaveMegaCombo(&OPT_SVEMsaveMegaCombo, this),
  itsDelayCacheSize(&OPT_SVEMdelayCacheSize, this),
  itsMaxCacheSize(&OPT_SVEMmaxCacheSize, this),
  itsSampleAtStart(&OPT_SVEMsampleAtStart, this),
  itsDisplaySacNum(&OPT_SVEMdisplaySacNum, this),
  itsDisplayPatch(&OPT_SVdisplayPatch, this),
  itsPatchSize(&OPT_SVpatchSize, this),
  itsEraseMarker(&OPT_SVeraseMarker, this),
  itsDisplayFOA(&OPT_SVdisplayFOA, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsOutFname(&OPT_SVEMoutFname, this),
  itsPriorRandomDistro(&OPT_SVEMpriorRandomDistro, this),
  itsViewStrategy(&OPT_SVEMviewingStrategy, this),
  itsViewStrategyDistro(&OPT_SVEMviewingStrategyRandomDistro, this),
  itsViewStrategyTiming(&OPT_SVEMviewingStrategyTiming, this),
  itsUseSaccadeInBlink(&OPT_SVEMuseSaccadeInBlink, this),
  itsUseDiagColor(&OPT_SVEMuseDiagColors, this),
  itsLabelEyePatch(&OPT_SVEMlabelEyePatch, this),
  itsWriteFrameNum(&OPT_SVEMwriteFrameNumber, this),
  itsNumRandomSamples(&OPT_SVEMnumRandomSamples, this),
  itsHistoryNumRandomSamples(&OPT_SVEMhistoryNumRandomSamples, this),
  itsMaxComboWidth(&OPT_SVEMmaxComboWidth, this),
  itsSMhistoryQlen(&OPT_SMhistoryQlen, this),
  itsDelayCache(), itsMaxCache(), itsHeadSM(), 
  itsDrawings(), itsCurrTime(),  itsLastSceneOnsetTime(), itsFrameNumber(-1), 
  itsHeaderCrafted(false), itsOutFields(), itsEyeStyles(), 
  itsTargets(), itsFrame2OnsetTime(), itsSMhistory(), itsOutFile(0), 
  itsRandFile(0),itsVSFile(0), itsRawToRetOffset()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  this->addSubComponent(itsMetrics);

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
SimulationViewerEyeMvt::~SimulationViewerEyeMvt()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SimulationViewerEyeMvt::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsDelayCacheSize.getVal())
    itsDelayCache.setMaxSize(itsDelayCacheSize.getVal());

  if (itsMaxCacheSize.getVal())
    itsMaxCache.setMaxSize(itsMaxCacheSize.getVal());

  // open output file:
  if (itsOutFname.getVal().empty() == false)
    {
      if (itsOutFile) delete itsOutFile;
      itsOutFile = new std::ofstream(itsOutFname.getVal().c_str());
      if (itsOutFile->is_open() == false)
        LFATAL("Cannot open '%s' for writing", itsOutFname.getVal().c_str());
    }

  // open random file if one exists
  if (itsPriorRandomDistro.getVal().empty() == false)
    {
      if (itsRandFile) delete itsRandFile;
      itsRandFile = new std::ifstream(itsPriorRandomDistro.getVal().c_str());
      if (itsRandFile->is_open() == false)
        LFATAL("Cannot open '%s' for reading",
               itsPriorRandomDistro.getVal().c_str());
      else
        {
          while (! itsRandFile->eof()) 
            // FIXME: robust input for incorrect files (i.e with e-1 or decimals) 
            {
              Point2D<int> pr;
              float ii, jj;
              (*itsRandFile) >> ii  >> jj;
              pr = Point2D<int>(ii,jj);
              randPoints.push_back(pr);
            }
          itsRandFile->close();
        }
    }

	// open reset timing file of viewing strategy if one exists
	if (itsViewStrategyTiming.getVal().empty() == false)
		{
			if (itsVSFile) delete itsVSFile;
			itsVSFile = new std::ifstream(itsViewStrategyTiming.getVal().c_str());
      if (itsVSFile->is_open() == false)
        LFATAL("Cannot open '%s' for reading",
               itsViewStrategyTiming.getVal().c_str());
      else
        {
          while (! itsVSFile->eof()) 
            // FIXME: robust input for incorrect files (i.e with e-1 or decimals) 
            {
              uint ii;
              (*itsVSFile) >> ii;
              vsResetFrame.push_back(ii);
            }
          itsVSFile->close();
       }
				nextOnsetIdx = 0; 
		}

	//using viewing strategy as sampling baseline?
	if (itsViewStrategy.getVal()) {
		// get parameters of viewing strategy
		std::stringstream s;
		char c; s<<itsViewStrategyDistro.getVal();
		s>>vsParams[0]>>c>>vsParams[1]>>c>>vsParams[2]>>c>>vsParams[3]>>c>>vsParams[4]
			>>c>>vsParams[5]>>c>>vsParams[6]>>c>>vsParams[7];

		if (s.fail())
			LFATAL("Failed to convert parameters of viewing strategy from string.");
	}

  // set our SM history queue length:
  if (itsSMhistoryQlen.getVal())
    itsSMhistory.setMaxSize(itsSMhistoryQlen.getVal());

  SimulationViewer::start1();
}

// ######################################################################
void SimulationViewerEyeMvt::stop1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

 if (itsOutFile) { delete itsOutFile; itsOutFile = 0; }
 if (itsRandFile) {delete itsRandFile; itsRandFile = 0;}
}

// ######################################################################
void SimulationViewerEyeMvt::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  // get the current time (mostly for SVEyeRegion)
  itsCurrTime = q.now();
  // any new input frame from the retina? If so, let's initialize or
  // clear all our drawings:
  bool gotnewinput = false;
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this)) {
    itsDrawings.resize(e->frame().getDims(), true);
    gotnewinput = true;
    itsFrameNumber++; // used for derived class SVEyeRegion
    // this member is never used in SVEyeMvt

		// monitor whether scene changes
		if (!itsViewStrategyTiming.getVal().empty()){

			if (itsFrameNumber == vsResetFrame[nextOnsetIdx]){
				itsLastSceneOnsetTime = q.now();
				nextOnsetIdx++;

				if (nextOnsetIdx >= (int)vsResetFrame.size()){
					nextOnsetIdx = 0; //reset to the begining to avoid out of range indexing
				}
			}
		}

		// a history of frame timing
		itsFrame2OnsetTime.push_back(q.now().msecs() - itsLastSceneOnsetTime.msecs());
	}

  // Let's update our sliding caches with the current SM/AGM/etc:
  const Image<float> currsm = getMap(q, false);

  // if we have a delay cache, we push the SM into it and push
  // what comes out of it into our max-computation cache;
  // otherwise, we directly push the SM into the max-computation
  // cache:
  if (itsDelayCacheSize.getVal())
    {
      // let's push the SM into our delay sliding cache:
      itsDelayCache.push_back(currsm);

      // transfer oldest delay image to our max computation cache:
      if (int(itsDelayCache.size()) >= itsDelayCacheSize.getVal())
          itsHeadSM = itsDelayCache.front();   
      else
        itsHeadSM.resize(currsm.getDims(), true);  // blank map
    }
  else 
      itsHeadSM = currsm;  // no delay cache

  // if we have a saliency history queue, and a new input, push the
  // current sm into it:
  if (itsSMhistoryQlen.getVal() && gotnewinput)
    itsSMhistory.push_back(itsHeadSM);

  // if we have a max cache, let's push the (possibly delayed) sm into it:
  if (itsMaxCacheSize.getVal()) itsMaxCache.push_back(itsHeadSM);

  // all right, get the latest retina image (whether new or not) as we
  // will need it to do coordinate transforms, coordinate clippings, etc:
  SeC<SimEventRetinaImage> retev = q.check<SimEventRetinaImage>(this, SEQ_ANY);
  if (retev.is_valid() == false)
    LFATAL("I need some SimEventRetinaImage in the queue to function.");
  // set the offset
  itsRawToRetOffset = retev->offset();

  // any eye-tracker action?
  SeC<SimEventEyeTrackerData> e = q.check<SimEventEyeTrackerData>(this);

  while(e.is_valid()) {
    // parse that event:
    const rutz::shared_ptr<EyeData> trackCurrPos = e->data();
    const uint tnum = e->trackerNum();
    const std::string tfn = e->trackerFilename();
    // add entry to style vector when we get a new trackernum
    if (itsEyeStyles.size() <= tnum) {
      itsEyeStyles.resize(tnum+1);
         
      itsEyeStyles[tnum].pSize = itsPatchSize.getVal();
      itsEyeStyles[tnum].col = e->trackerColor();

      // give distinct label, TODO: maybe write a switch for this
      //      itsEyeStyles[tnum].label = convertToString(tnum);
      // huge hack right now, relies on directory tree data/<subject>/ pieces
      itsEyeStyles[tnum].label = tfn.substr(tfn.find("data")+5,2);
    }

    // draw small patch at current eye position:
    if (itsDisplayPatch.getVal()) 
      {
        // erase previous eye markers first
        if (itsEraseMarker.getVal()) itsDrawings.resize(retev->frame().getDims(), true);

        // draw marker at eye position
        drawEye(trackCurrPos, tnum);
      }

    // do we want to do any other sample processing? 
    // (for SVEyeRegion or other derived classes)
    extraSampleProcessing(trackCurrPos);

    bool isUsableTrace = true;
    // if we can't use saccades in blink and it's in blink, don't process the event
    if (itsUseSaccadeInBlink.getVal() == false && trackCurrPos->isInBlink() == true) isUsableTrace = false;
    // NB: what about combined saccades?

    // are we starting a saccade/special region with extra data?
    if (trackCurrPos->hasSaccadeTargetData() && isUsableTrace)
      {
        // extract the saccade destination for display:
        Point2D<int> dp = rawToRet(trackCurrPos->saccadeTarget());

        CLINFO("**** Tracker %03d: Saccade to (%d, %d) at %.1fms. Last scene change time was %.1fms ****",
               tnum, dp.i, dp.j, q.now().msecs(), itsLastSceneOnsetTime.msecs());
        // check that saccade target is okay:
        if (itsDrawings.coordsOk(dp) == false)
          CLINFO("Tracker %03d: Saccade target (%d, %d) out of bounds "
                 "-- IGNORING", tnum, dp.i, dp.j);
        else
          // do some saliency sampling at saccade target, but may
          // have to be delayed to end of a saccade... So for now
          // let's just push the eye data into a temporary storage:
          itsTargets[tnum] = trackCurrPos;
      }

    // is it time to take a bunch of samples at the saccade target of
    // this tracker? Either we want to sample at the start of a
    // saccade and just pushed some saccade data, or we want to sample
    // at end and we are ending the saccade during which we pushed
    // some data:
    std::map<int, rutz::shared_ptr<EyeData> >::iterator
      itr = itsTargets.find(tnum);

    if (itr != itsTargets.end() && isUsableTrace &&
        (itsSampleAtStart.getVal() || trackCurrPos->isInSaccade() == false))
    {
      // extract the saccade target data:
      const rutz::shared_ptr<EyeData> trackEventSample = itr->second;
      itsTargets.erase(itr); // we are now taking care of this one

      // changed from trackCurrPos->...();
      if(!(trackEventSample->hasSaccadeTargetData()) )
         LFATAL("Tracker %03d: Marked event has incomplete targeting data.", tnum);
 
      Point2D<int> trackerTarget = rawToRet(trackEventSample->saccadeTarget());

      LINFO("Tracker %03d [%s]: Taking saccade samples at (%d, %d) at time %.1fms",
            tnum, tfn.c_str(), trackerTarget.i, trackerTarget.j, q.now().msecs());

      // draw circle around target:
      if (itsDisplayFOA.getVal()) drawFOA(trackerTarget, tnum);

      // indicate saccade number?
      if (itsDisplaySacNum.getVal())
        {
          if (tnum == 0) {
            std::string fld("typenum"); //subject to change
            if (trackCurrPos->hasMetaData(fld.c_str()))
              writeText(itsDrawings, Point2D<int>(0, 0),
                        sformat(" %1g ", 
                                trackEventSample->getMetaDataField("typenum")).c_str(),
                        PixRGB<byte>(1, 1, 1));
            else
              LFATAL("--svem-display-sacnum does not have numbered saccades available");
          }
          else
            LFATAL("--svem-display-sacnum only supported with 1 tracker");
        } 

      if (itsOutFile) {
        LINFO("writing to file %s........", itsOutFname.getVal().c_str());
        
        // also creates the header the first time
        std::string statline = craftSVEMOutput(tfn, trackEventSample);

        // write the header line now - only once for each output
        if(!itsHeaderCrafted)
          {
            writeHeader();
            itsHeaderCrafted = true;
          }
        (*itsOutFile) << statline << std::endl;
      }
    }//end sampling at saccade target

    // any more events like that in the queue?
    e = q.check<SimEventEyeTrackerData>(this);
  }
}

// ######################################################################
void SimulationViewerEyeMvt::drawEye(const rutz::shared_ptr<EyeData> rawPos, const uint tN)
{
  // convert to retinal coordinates (accounting for any shifting,
  // embedding, etc):
  Point2D<int> currPos = rawToRet(rawPos->position());

  // note: these functions are tolerant to 
  // coordinates outside the actual image area
  PixRGB<byte> col;
  // select a drawing color:
  // note: we use 'almost black' (1,1,1) instead of black so that we can later
  // composite our drawings with various images, with pure black
  // then being treated as transparent):
  if (!itsUseDiagColor.getVal())
    {
      col = itsEyeStyles[tN].col;
      // dec/inc will clamp/saturate properly
      if (rawPos->isInSaccade()) col += byte(64);
      if (rawPos->isInBlink()) col -= byte(128);  
    }
  else //diagnostic colors
      {
        byte r = 1, g = 1, b = 1;
        if (rawPos->isInSaccade()) g = 255;  //green for saccade
        else if (rawPos->isInSmoothPursuit()) 
          {r = 255;b = 255;} // magenta
        else if (rawPos->isInFixation()) b = 255;                // blue
        if (rawPos->isInCombinedSaccade()) {b = 255;g = 255;}    //cyan

        col = PixRGB<byte>(r,g,b);    //default black
        if (rawPos->isInBlink()) col-=byte(128); // let's just make the cursor darker
      }
  drawPatchBB(itsDrawings, currPos, itsEyeStyles[tN].pSize, 
              col, PixRGB<byte>(1));

  //output the label we got next to the patch
  if(itsLabelEyePatch.getVal()) { 
    writeText(itsDrawings, currPos+itsEyeStyles[tN].pSize/2,
              itsEyeStyles[tN].label.c_str(),
              PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
              SimpleFont::FIXED(10), true);
  }
}

// ######################################################################
void SimulationViewerEyeMvt::drawFOA(const Point2D<int> target, const uint tN)
{
  PixRGB<byte> col = itsEyeStyles[tN].col; col += byte(64);
  // since drawFOA is only called when a saccade is triggered, 
  // the color adjustment for a saccade is used.

  const int radius = itsMetrics->getFoveaRadius();
  drawCircle(itsDrawings, target, radius, col, 2);
}

// ######################################################################
std::string SimulationViewerEyeMvt::craftSVEMOutput(const std::string tfn, 
                        const rutz::shared_ptr<EyeData> data)
{
  // Write some measurements to a string. Here, we just put all the stats
  // into a text string, and we will send that off to be
  // written to disk.
  // The first time we run this, we build a header for the file 
  // to know what stats to read. This is done in the craft submodules.  

  std::string output = 
    sformat("%s ", tfn.c_str());
  if(!itsHeaderCrafted) itsOutFields.push_back("filename");
 
  if(itsWriteFrameNum.getVal()) {
    output += sformat("%u ",itsFrameNumber);
    if(!itsHeaderCrafted) itsOutFields.push_back("framenum");
  }
  
  // add model free output from the original eyetrack-data file 
  output += craftModelFreeOutput(data);

  // construct the saliency map, 
  // if requested from the max of several previous maps 
  Image<float> smap;
  if (itsMaxCacheSize.getVal()) 
    smap = itsMaxCache.getMax();
  else smap = itsHeadSM;

  // add samples of saliency map at the target / in the frame
  output += craftSMSamples(data, smap);

  // add history of saliency map across past frames for key locations
  if (itsSMhistoryQlen.getVal()) output += craftSMHistory(data, smap);

  return output;
}

// ######################################################################
std::string SimulationViewerEyeMvt::
craftModelFreeOutput(const rutz::shared_ptr<EyeData> data)
{
  // legacy behavior - this was the old eyesal output for the model-free numbers
  // retained for reference
  /*
    std::string output =
    sformat("%s %d %d  %d %d  %g %g %g %g", tfn.c_str(), rawdp.i, rawdp.j,
    itsCurrTarget.i, itsCurrTarget.j, d->pupilDiameter(), d->saccadeAmplitude(),
    d->saccadeDuration(), d->saccadeTime());
  */

  Point2D<int> target = rawToRet(data->saccadeTarget());
  std::string output = sformat("%d %d ", target.i, target.j);
  
  // start writing fields for the header
  if(!itsHeaderCrafted) 
    { 
      itsOutFields.push_back("ret_targetx"); 
      itsOutFields.push_back("ret_targety");
    }
  
  // output all the extra (now precomputed) parameters, 
  // default from Eye-markup is saccade target first, 
  // then amplitude and peak velocity, then time onset and offset of saccade.
  // then the interval between this saccade and the next, 
  // and lastly the number of the saccade (discriminates between saccades 
  // in blink, etc.) 
  std::string foo;
  for (ParamMap::key_iterator iter = data->getMetaData()->keys_begin();
       iter != data->getMetaData()->keys_end();
             ++iter)
    {
      // get rid of any leading asterisk
      foo = *iter;
      
      // add this to the queue of arguments
      if(!itsHeaderCrafted) itsOutFields.push_back(foo);
      output += sformat (" %g", data->getMetaDataField(foo));
    }

  return output;
}

// ######################################################################
std::string SimulationViewerEyeMvt::craftSMSamples(const rutz::shared_ptr<EyeData> data, 
                                                   const Image<float> smap)
{
  std::string output;
  // get the map level to scale things down:
  int sml = itsLevelSpec.getVal().mapLevel();

  // get target
  Point2D<int> target = rawToRet(data->saccadeTarget());
  
  // get ready to take samples in the saliency map and in all
  // conspicuity maps, at the scale of the saliency map:
  const int radius = itsMetrics->getFoveaRadius();
  const int rad = radius >> sml;
  Point2D<int> p(target.i >> sml, target.j >> sml);
  
  // just in case the image has weird dims, make sure the sampling
  // location is in it:

  p.clampToDims(smap.getDims());

  // get the min/max/avg and saliency samples:
  float mi, ma, av; getMinMaxAvg(smap, mi, ma, av);

  // get a sample around the saccade target location:

  float val = getLocalMax(smap, p, rad);

  // add necessary headers
  if(!itsHeaderCrafted) 
    {
      itsOutFields.push_back("sal_val");
      itsOutFields.push_back("sal_min");
      itsOutFields.push_back("sal_max");
      itsOutFields.push_back("sal_av");
    }

  output = sformat("  %g %g %g %g ", val, mi, ma, av);

	// compute current time from scene onset
	double timeFromOnset = 0;
	double stdx, stdy;
	int x, y;
	if (!itsViewStrategyTiming.getVal().empty()){
		timeFromOnset = itsCurrTime.msecs() - itsLastSceneOnsetTime.msecs();
		output += sformat(" %g", timeFromOnset);
    if(!itsHeaderCrafted){itsOutFields.push_back("time_scene_change");}
 }

  // now get a bunch of random samples from the saliency map:
  if (itsNumRandomSamples.getVal() < smap.getSize())
    {
      // ok, we want fewer samples than are actually in the
      // saliency map, so choose them randomly:
      for (int k = 0; k < itsNumRandomSamples.getVal(); ++k) 
        {
          if (!itsPriorRandomDistro.getVal().empty()) // take random points from file
            {
               const int randn = randomUpToNotIncluding(randPoints.size());
              Point2D<int> randp(randPoints[randn].i >> sml,
                                 randPoints[randn].j >> sml);

              LINFO("%d %d", randPoints[randn].i, randPoints[randn].j);
              // just in case the image has weird dims, make sure the sampling
              // location is in it:
              randp.clampToDims(smap.getDims());

              output += sformat(" %g %d %d", getLocalMax(smap, randp, rad),
                                randPoints[randn].i, randPoints[randn].j);
           }
					else if (itsViewStrategy.getVal()) // sampling based on viewing strategy
					 {
							// instead of clampToDims, which will show high sampling rate at edges, we redo 
							// the randomization

							// get the standard deviation x and y of the gaussian fit
							stdx = vsParams[0] + vsParams[1]/(1 + exp(-(timeFromOnset+vsParams[2])/vsParams[3])); 
							stdy = vsParams[4] + vsParams[5]/(1 + exp(-(timeFromOnset+vsParams[6])/vsParams[7]));

							do {
								x = (int) randomDoubleFromNormal(stdx) >> sml;
								x = x + smap.getWidth()/2;	
							} while ((x<0) | (x>=smap.getWidth()));
							do {
								y = (int) randomDoubleFromNormal(stdy) >> sml;
								y = y + smap.getHeight()/2;	
							} while ((y<0) | (y>=smap.getHeight()));
							 
							Point2D<int> randp(x, y);
              output += sformat(" %g %d %d", getLocalMax(smap, randp, rad),
                                randp.i, randp.j);
					 }
          else // uniform random
            {
              Point2D<int> randp(randomUpToNotIncluding(smap.getWidth()),
                                 randomUpToNotIncluding(smap.getHeight()));
              output += sformat(" %g %d %d", getLocalMax(smap, randp, rad),
                                randp.i, randp.j);
           }

          if(!itsHeaderCrafted) 
            {
              itsOutFields.push_back("rand_sal");
              itsOutFields.push_back("rand_x");
              itsOutFields.push_back("rand_y");
            }
        }
    }
  else
    {
      // ok, we want more or as many samples as are in the
      // saliency map, so just dump each map value once:
      Point2D<int> rp;
      for (rp.j = 0; rp.j < smap.getHeight(); ++rp.j)
        for (rp.i = 0; rp.i < smap.getWidth(); ++rp.i)
          {
            output += sformat(" %g %d %d", getLocalMax(smap, rp, rad),
                              rp.i, rp.j);
            if(!itsHeaderCrafted) 
              {
                itsOutFields.push_back("rand_sal");
                itsOutFields.push_back("rand_x");
                itsOutFields.push_back("rand_y");
              }
          }
    }
  return output;
}

// ######################################################################
std::string SimulationViewerEyeMvt::craftSMHistory(const rutz::shared_ptr<EyeData> data,
                                                   Image<float> smap)
{
  std::string output;
  // get the map level to scale things down:
  int sml = itsLevelSpec.getVal().mapLevel();
  
  // get target
  Point2D<int> target = rawToRet(data->saccadeTarget());

  // get ready to take samples in the saliency map and in all
  // conspicuity maps, at the scale of the saliency map:
  const int radius = itsMetrics->getFoveaRadius();
  const int rad = radius >> sml;
  Point2D<int> p(target.i >> sml, target.j >> sml);
  p.clampToDims(smap.getDims());

  // save history of saliency values at saccade target?
  output += std::string("   ");
      
  // FIXME: Error thrown on one of these clampToDims for out of range?
  // also get saccade start location, scaled down to SM size:
  Point2D<int> pp(data->position());
  pp.i >>= sml; pp.j >>= sml; pp.clampToDims(smap.getDims());
  
  // note: even though the queue may not be full yet, we
  // will always write itsSMhistoryQlen samples here, to
  // facilitate reading the data into matlab using
  // dlmread(). Saliency values for non-existent maps will
  // be set to 0:
  size_t ql = itsSMhistory.size();
	uint ii = 0;
  for (uint i = 0; i < itsSMhistoryQlen.getVal(); ++ i) 
    {
      if (i < ql)
        {
          // get min, max and avg saliency values over history:
          float mmi, mma, mmav;
          getMinMaxAvg(itsSMhistory[ql - 1 - i], mmi, mma, mmav);
         	output +=
          	  sformat(" %g %g %g %g",
          	          getLocalMax(itsSMhistory[ql - 1 - i], p, rad),
          	          getLocalMax(itsSMhistory[ql - 1 - i], pp, rad),
          	          mma, mmav);
					
					ii = 0;
					while (ii < itsHistoryNumRandomSamples.getVal()){ 
          	// and also value at one random location:
          	Point2D<int> randp;
          	if (!itsPriorRandomDistro.getVal().empty()) // prior from file
          	 {
          	    const int randn =
          	      randomUpToNotIncluding(randPoints.size());
          	    randp.i = randPoints[randn].i >> sml;
          	    randp.j = randPoints[randn].j >> sml;

          	    // just in case the image has weird dims, make sure the sampling
          	    // location is in it:
          	    randp.clampToDims(smap.getDims());
          	  }
						else if (itsViewStrategy.getVal()) // prior from viewing strategy
							{
								double stdx, stdy;
								int x, y;						
	
								// get the standard deviation x and y of the gaussian fit
								stdx = vsParams[0] + vsParams[1]/(1 + exp(-(itsFrame2OnsetTime[itsFrame2OnsetTime.size()-1-i]+vsParams[2])/vsParams[3])); 
								stdy = vsParams[4] + vsParams[5]/(1 + exp(-(itsFrame2OnsetTime[itsFrame2OnsetTime.size()-1-i]+vsParams[6])/vsParams[7]));

								do {
									x = (int) randomDoubleFromNormal(stdx) >> sml;
									x = x + smap.getWidth()/2;	
								} while ((x<0) | (x>=smap.getWidth()));
								do {
									y = (int) randomDoubleFromNormal(stdy) >> sml;
									y = y + smap.getHeight()/2;	
								} while ((y<0) | (y>=smap.getHeight()));

								randp.i = x;
								randp.j = y;
							}
						else // uniform sampling
          	  {
          	    randp.i = randomUpToNotIncluding(smap.getWidth());
          	    randp.j = randomUpToNotIncluding(smap.getHeight());
          	  }
          	 
          	output +=
          	  sformat(" %g",
          	          getLocalMax(itsSMhistory[ql - 1 - i], randp, rad));
						ii++;
					}
        }
      else {
				output += std::string(" 0.0 0.0 0.0 0.0");
				ii = 0;
				while (ii < itsHistoryNumRandomSamples.getVal()) {
					output += std::string(" 0.0");
					ii++;
				}
			}

      if(!itsHeaderCrafted) 
        {
          itsOutFields.push_back("hist_sacendsal");
          itsOutFields.push_back("hist_sacstartsal");
          itsOutFields.push_back("hist_maxsal");
          itsOutFields.push_back("hist_avgsal");
  				ii = 0;
					while (ii < itsHistoryNumRandomSamples.getVal()) {
	        	itsOutFields.push_back("hist_randsal");      
						ii++;
					}
        }
    }
  return output;
}

// ######################################################################
void SimulationViewerEyeMvt::writeHeader()
{
  LINFO("writing header for file %s (%zu fields)", itsOutFname.getVal().c_str(),
        itsOutFields.size());
  (*itsOutFile) << "#" << itsOutFields[0];
  for(uint i = 1; i < itsOutFields.size(); i++) 
    (*itsOutFile) << " " << itsOutFields[i];
  (*itsOutFile) << std::endl;
}

// ######################################################################
void SimulationViewerEyeMvt::extraSampleProcessing(const rutz::shared_ptr<EyeData>) {}
// empty here, written so that EyeRegion can do extra output on each eye position

// ######################################################################
Image< PixRGB<byte> > SimulationViewerEyeMvt::getTraj(SimEventQueue& q)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // get the latest retina image:
  Image< PixRGB<byte> > input;
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this, SEQ_ANY))
    input = e->frame().colorByte();
  else
    LFATAL("Could not find required SimEventRetinaImage");

  // make a composite of the input + the drawings:
  Image< PixRGB<byte> > comp = composite(itsDrawings, input, PixRGB<byte>(0,0,0));
  
  // return a plain traj?
  if (itsSaveTraj.getVal()) return comp;

  // let's get the current saliency map (normalized):
  const Dims dims = input.getDims();
  Image<float> sm = getMap(q, false);
  if (sm.initialized()) sm = rescaleOpt(sm, dims, itsDisplayInterp.getVal());
  else sm.resize(dims, true); // blank
  Image< PixRGB<byte> > smc = toRGB(Image<byte>(sm));

  // make a composite of the instantaneous SM + the drawings:
  Image< PixRGB<byte> > smcomp = composite(itsDrawings, smc);


  // otherwise, return mega combo; we have two formats: if we have a
  // max-cache, it will be a 4-image format, otherwise a 2-image
  // format:
  Image< PixRGB<byte> > ret;
  if (itsMaxCacheSize.getVal())
    {
      // 4-image format
      Image<float> maxsm =
        rescaleOpt(itsMaxCache.getMax(), dims, itsDisplayInterp.getVal());
      Image< PixRGB<byte> > maxsmc = toRGB(Image<byte>(maxsm));

      ret = concatX(concatY(input, smcomp),
                    concatY(comp, composite(itsDrawings, maxsmc)));

      drawGrid(ret, dims.w()-1, dims.h()-1, 3, 3, PixRGB<byte>(128, 128, 128));
    }
  else
    {
      // 2-image format:
      ret = concatX(comp, smcomp);
      drawLine(ret, Point2D<int>(dims.w()-1, 0), Point2D<int>(dims.w()-1, dims.h()-1),
               PixRGB<byte>(255, 255, 0), 1);
      drawLine(ret, Point2D<int>(dims.w(), 0), Point2D<int>(dims.w(), dims.h()-1),
               PixRGB<byte>(255, 255, 0), 1);
    }

  // make sure image is not unreasonably large:
  while (ret.getWidth() > itsMaxComboWidth.getVal())
    ret = decY(lowPass3y(decX(lowPass3x(ret))));

  return ret;
}

// ######################################################################
void SimulationViewerEyeMvt::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerEyeMvt::save1(const ModelComponentSaveInfo& sinfo)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  SimEventQueue *q = dynamic_cast<const SimModuleSaveInfo&>(sinfo).q;

  // update the trajectory:
  Image< PixRGB<byte> > res = getTraj(*q);

  // save results?
  if (itsSaveMegaCombo.getVal() || itsSaveTraj.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

      ofs->writeRGB(res, "T",
                    FrameInfo("SimulationViewerEyeMvt trajectory", SRC_POS));

    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
