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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL:
// $Id:
//

#include "Neuro/SimulationViewerEyeHand.H"

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
#include "Psycho/HandData.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

#include <fstream>
#include <iostream>

// for some reason this is not recognized when it appears in NeuroOpts.C
const ModelOptionDef OPT_SMhistoryQlen =
  { MODOPT_ARG(uint), "SMhistoryQ", &MOC_DISPLAY, OPTEXP_CORE,
    "Keep a historical queue of salieny maps (one per new input frame), "
    "and report the history of saliency values over the entire queue "
    "and at a saccade target location, for each saccade",
    "sm-history-qlen", '\0', "<uint>", "0" };


// ######################################################################
SimulationViewerEyeHand::
SimulationViewerEyeHand(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsSaveTraj(&OPT_SVsaveTraj, this),
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
  itsUseSaccadeInBlink(&OPT_SVEMuseSaccadeInBlink, this),
  itsUseDiagColor(&OPT_SVEMuseDiagColors, this),
  itsLabelEyePatch(&OPT_SVEMlabelEyePatch, this),
  itsNumRandomSamples(&OPT_SVEMnumRandomSamples, this),
  itsMaxComboWidth(&OPT_SVEMmaxComboWidth, this),
  itsSMhistoryQlen(&OPT_SMhistoryQlen, this),
  itsDisplayHand(&OPT_SVHandDisplay, this),
  itsSaveCombo(&OPT_SVEMsaveMegaCombo, this),
  itsDelayCache(), itsMaxCache(), itsHeadSM(), 
  itsDrawings(), itsCurrTime(),  itsFrameNumber(-1), 
  itsHeaderCrafted(false), itsOutFields(), itsEyeStyles(), 
  itsEyeTargets(), itsHandTargets(), itsSMhistory(), itsOutFile(0), 
  itsRandFile(0),itsRawToRetOffset()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  this->addSubComponent(itsMetrics);

  // disable IOR:
  LINFO("NOTE: disabling IOR and SE, selecting FixedSaccadeController");
  getManager().setOptionValString(&OPT_IORtype, "None");
  getManager().setOptionValString(&OPT_ShapeEstimatorMode, "None");

  // select an eyetrack EyeHeadController:
  getManager().setOptionValString(&OPT_EyeHeadControllerType, "EyeTrack");

  // Select HandTrack for the HandControllerType
  getManager().setOptionValString(&OPT_HandControllerType, "HandTrack");

  // change default to --display-additive=false; user can still
  // override this on the command-line
  getManager().setOptionValString(&OPT_SVdisplayAdditive, "false");
}

// ######################################################################
SimulationViewerEyeHand::~SimulationViewerEyeHand()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SimulationViewerEyeHand::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsDelayCacheSize.getVal())
    itsDelayCache.setMaxSize(itsDelayCacheSize.getVal());

  if (itsMaxCacheSize.getVal())
    itsMaxCache.setMaxSize(itsMaxCacheSize.getVal());

  // open output file:
  if (itsOutFname.getVal().empty() == false) {
    if (itsOutFile) delete itsOutFile;
    itsOutFile = new std::ofstream(itsOutFname.getVal().c_str());
    if (itsOutFile->is_open() == false)
      LFATAL("Cannot open '%s' for writing", itsOutFname.getVal().c_str());
  }

  // open random file if one
  if (itsPriorRandomDistro.getVal().empty() == false) {
    if (itsRandFile) delete itsRandFile;
    itsRandFile = new std::ifstream(itsPriorRandomDistro.getVal().c_str());
    if (itsRandFile->is_open() == false)
      LFATAL("Cannot open '%s' for reading",
             itsPriorRandomDistro.getVal().c_str());
    else {
      while (! itsRandFile->eof()) {
        Point2D<int> pr;
        (*itsRandFile) >> pr.i >> pr.j;
        randPoints.push_back(pr);
      }
      itsRandFile->close();
    }
  }
  
  // set our SM history queue length:
  if (itsSMhistoryQlen.getVal())
    itsSMhistory.setMaxSize(itsSMhistoryQlen.getVal());

  SimulationViewer::start1();
}

// ######################################################################
void SimulationViewerEyeHand::stop1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

 if (itsOutFile)  {delete itsOutFile;  itsOutFile  = 0;}
 if (itsRandFile) {delete itsRandFile; itsRandFile = 0;}
}

// ######################################################################
void SimulationViewerEyeHand::
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
    // this member is never used in SVEyeHand
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

  // ## Hand-related stuffs
  //{@
  
  // any hand-tracker action?
  SeC<SimEventHandTrackerData> ee = q.check<SimEventHandTrackerData>(this);
  
  while(ee.is_valid()) {
    // parse that event:
    const rutz::shared_ptr<HandData> trackCurrPos = ee->data();
    const uint tnum = ee->trackerNum();
    const std::string tfn = ee->trackerFilename();
    // add entry to style vector when we get a new trackernum
    if (itsHandStyles.size() <= tnum) {
      itsHandStyles.resize(tnum+1);
      
      itsHandStyles[tnum].pSize = itsPatchSize.getVal();
      itsHandStyles[tnum].col = ee->trackerColor();
      
      // give distinct label, TODO: maybe write a switch for this
      //      itsHandStyles[tnum].label = convertToString(tnum);
      //huge hack right now, relies on directory tree data/<subject>/ pieces
      itsHandStyles[tnum].label = tfn.substr(tfn.find("data")+5,2);
    }
    
    // draw the conditions of current hand conditions:
    if (itsDisplayHand.getVal()) {

      // erase previous hand markers first
      if (itsEraseMarker.getVal()) itsDrawings.resize(retev->frame().getDims(), true);
      
      // draw marker at hand position
      drawHand(trackCurrPos, tnum);
    }
    
    // do we want to do any other sample processing? 
    // (for SVHandRegion or other derived classes)
    extraSampleProcessing(trackCurrPos);
    
    // any more events like that in the queue?
    ee = q.check<SimEventHandTrackerData>(this);
  }
  //@}

  // ## Eye-related Stuffs

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
      //huge hack right now, relies on directory tree data/<subject>/ pieces
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

        CLINFO("**** Tracker %03d: Saccade to (%d, %d) at %.1fms ****",
               tnum, dp.i, dp.j, q.now().msecs());
        // check that saccade target is okay:
        if (itsDrawings.coordsOk(dp) == false)
          CLINFO("Tracker %03d: Saccade target (%d, %d) out of bounds "
                 "-- IGNORING", tnum, dp.i, dp.j);
        else
          // do some saliency sampling at saccade target, but may
          // have to be delayed to end of a saccade... So for now
          // let's just push the eye data into a temporary storage:
          itsEyeTargets[tnum] = trackCurrPos;
      }

    // is it time to take a bunch of samples at the saccade target of
    // this tracker? Either we want to sample at the start of a
    // saccade and just pushed some saccade data, or we want to sample
    // at end and we are ending the saccade during which we pushed
    // some data:
    std::map<int, rutz::shared_ptr<EyeData> >::iterator
      itr = itsEyeTargets.find(tnum);

    if (itr != itsEyeTargets.end() && isUsableTrace &&
        (itsSampleAtStart.getVal() || trackCurrPos->isInSaccade() == false))
    {
      // extract the saccade target data:
      const rutz::shared_ptr<EyeData> trackEventSample = itr->second;
      itsEyeTargets.erase(itr); // we are now taking care of this one

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
        (*itsOutFile) << craftSVEMOutput(tfn, trackEventSample) << std::endl;
      }
    }//end sampling at saccade target

    // any more events like that in the queue?
    e = q.check<SimEventEyeTrackerData>(this);
  }


}

// ######################################################################
void SimulationViewerEyeHand::drawEye(const rutz::shared_ptr<EyeData> rawPos, const uint tN)
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
void SimulationViewerEyeHand::drawHand(const rutz::shared_ptr<HandData> rawPos, const uint tN)
{
  /* The output that we expect on screen:
   * .--------------------.
   * | |                  |
   * | |                  |
   * | Y                  |
   * | | B B B B ...      |
   * |  ------X--------   |
   * '--------------------'
   * X & Y = x & y (left-right & accl-brake) direction respectively
   *         It will show a line for range of movement and a box
   *         indicating the current position
   * B's   = buttons, there will be circles with numbers on it indicating
   *         whether the button is pressed(green) or not(red)
   *         Hopefully the B's on screen is wrapped around incase too many
   *         buttons are avail
   */

  // Our screen dimensions
  const Dims dims = itsDrawings.getDims();
  
  // Default one grid cell based on the screen dimensions
  // Assuming we always have 4:3 tv ratio to get a square grid
  // TODO: make this in the option
  const Dims numgrid (80,60);//Under 640x480, its expected to have 8x8 per grid
  const Dims grid (dims.w()/numgrid.w(),dims.h()/numgrid.h()); 

  PixRGB<byte> colTransp = PixRGB<byte> (  0, 0, 0);
  PixRGB<byte> colRed    = PixRGB<byte> (255, 0, 0);
  PixRGB<byte> colB;


  if (rawPos->isValid()) {
    // Get raw X and Y position
    int rawX = rawPos->getX();
    int rawY = rawPos->getY();

    // Get the dims of X and Y position
    /*! Assuming grid [1,1]'s coord is (8,8) --> [ 0, 0] is (  0,  0)
     *                                       --> [80,60] is (640,480)
     *  We only want the X to be in between grid [2,59] to [79,59]
     *  and Y in [1,1] to [1,58]. We set those instead of [1,59] to prevent
     *  collision incase both meet eachother (rawX=0/255, rawY=0/255)
     *  |----------------|-------[]------|
     *  X1               XC      X_      X2
     */
    Point2D<int> X1 (getGridCoord(grid,             2, numgrid.h()-1));
    Point2D<int> X2 (getGridCoord(grid, numgrid.w()-1, numgrid.h()-1));
    Point2D<int> XC ((127*(X2.i-X1.i)/255)+X1.i,X1.j);
    Point2D<int> XD (0,grid.h()/2);
    Point2D<int> Y1 (getGridCoord(grid,             1,             1));
    Point2D<int> Y2 (getGridCoord(grid,             1, numgrid.h()-2));
    Point2D<int> YC (grid.w(),(127*(Y2.j-Y1.j)/255)+Y1.j);
    Point2D<int> YD (grid.w()/2,0);
    // The patch coords
    Point2D<int> X_ ((rawX*(X2.i-X1.i)/255)+X1.i, dims.h()-grid.h());
    Point2D<int> Y_ (grid.w(), (rawY*(Y2.j-Y1.j)/255)+Y1.j);

    // First draw the white line for the X-Y coords
    const PixRGB<byte> colWhite (255,255,255);
    // The X-line
    drawLine(itsDrawings, X1, X2, colWhite);
    drawLine(itsDrawings, X1-XD, X1+XD, colWhite);
    drawLine(itsDrawings, XC-XD, XC+XD, colWhite);
    drawLine(itsDrawings, X2-XD, X2+XD, colWhite);
    // The Y-line
    drawLine(itsDrawings, Y1, Y2, colWhite);
    drawLine(itsDrawings, Y1-YD, Y1+YD, colWhite);
    drawLine(itsDrawings, YC-YD, YC+YD, colWhite);
    drawLine(itsDrawings, Y2-YD, Y2+YD, colWhite);


    // Draw the box for the X-Y
    drawPatchBB(itsDrawings, X_, itsHandStyles[tN].pSize,   
                colWhite, PixRGB<byte>(1));
    drawPatchBB(itsDrawings, Y_, itsHandStyles[tN].pSize,   
                colWhite, PixRGB<byte>(1));
  
    // For the buttons
    if (rawPos->isButtonEmpty() == false) {  
      for (uint i = 0; i < rawPos->numButton(); i++) {
        // Blue if pressed, red if not
        if (rawPos->isPressed(i))
          colB = colRed;
        else
          colB = colTransp;

        // For buttons position, we want them to be @ bottom of the screen
        // TODO: wrapped around in case the buttons grid over [80,_]

        // Write text of numbers into this thing
        writeText(itsDrawings,
                  Point2D<int>(getGridCoord(grid, 3*i+10, numgrid.h()-5)),
                  sformat("%2d",i+1).c_str(),
                  PixRGB<byte>(255), // Text color
                  colB, // Background color
                  SimpleFont::FIXED(10), // Font
                  false); // BG transparent?
  
        // Draw circle
        //drawCircle(itsDrawings, Point2D<int>(getGridCoord(grid, 3*i+3, 1)),6,colB,1);
      }
    }
  }//if rawPos->isValid()

  // For Mouse
  writeText(itsDrawings,
            Point2D<int>(getGridCoord(grid, 7, numgrid.h()-8)),
            sformat("Mouse: ").c_str(),
            PixRGB<byte>(255), // Text color
            PixRGB<byte>(0), // Background color
            SimpleFont::FIXED(10), // Font
            false); // BG transparent?
  if (rawPos->isMouseValid()) {
    // raw mouse position
    Point2D<int> mousePos;

    // computed mouse position
    if (rawPos->getNativeX() > 0 && rawPos->getNativeY() > 0) // valid dim
      // Absolute relation to the movie
      //mousePos=Point2D<int>(rawPos->getMouseX()*dims.w()/rawPos->getNativeX(),
      //                    rawPos->getMouseY()*dims.h()/rawPos->getNativeY());

      // Relative to the inside, fixed value
      if (rawPos->getNativeX() == 1024)
        mousePos=Point2D<int>((rawPos->getMouseX()*583/rawPos->getNativeX())+37,
                              (rawPos->getMouseY()*438/rawPos->getNativeY())+15);
      else if (rawPos->getNativeX() == 800)
        mousePos=Point2D<int>((rawPos->getMouseX()*602/rawPos->getNativeX())+11,
                              (rawPos->getMouseY()*439/rawPos->getNativeY())+11);
      else
        mousePos=Point2D<int>(rawPos->getMouseX()*dims.w()/rawPos->getNativeX(),
                              rawPos->getMouseY()*dims.h()/rawPos->getNativeY());
    else
      mousePos=Point2D<int>(rawPos->getMouseX(), rawPos->getMouseY());

    // draw the mouse position
    drawPatchBB(itsDrawings, mousePos, itsHandStyles[tN].pSize,
                PixRGB<byte>(127), PixRGB<byte>(1));
    //}
    // drawing the letters: Mouse: L M R
    // the letter LMR will go red if pressed
    colB = rawPos->getMouseBL()?colRed:colTransp;
    writeText(itsDrawings,
              Point2D<int>(getGridCoord(grid, 15, numgrid.h()-8)),
              sformat("L").c_str(),
              PixRGB<byte>(255), // Text color
              colB,
              SimpleFont::FIXED(10), // Font
              false); // BG transparent?
    colB = rawPos->getMouseBM()?colRed:colTransp;
    writeText(itsDrawings,
              Point2D<int>(getGridCoord(grid, 18, numgrid.h()-8)),
              sformat("M").c_str(),
              PixRGB<byte>(255), // Text color
              colB,
              SimpleFont::FIXED(10), // Font
              false); // BG transparent?
    colB = rawPos->getMouseBR()?colRed:colTransp;
    writeText(itsDrawings,
              Point2D<int>(getGridCoord(grid, 21, numgrid.h()-8)),
              sformat("R").c_str(),
              PixRGB<byte>(255), // Text color
              colB,
              SimpleFont::FIXED(10), // Font
              false); // BG transparent?
  }
  else {
    writeText(itsDrawings,
              Point2D<int>(getGridCoord(grid, 15, numgrid.h()-8)),
              sformat("n/a").c_str(),
              PixRGB<byte>(255), // Text color
              PixRGB<byte>(0),
              SimpleFont::FIXED(10), // Font
              false); // BG transparent?
  }

  // For Keyboard
  writeText(itsDrawings,
            Point2D<int>(getGridCoord(grid,25,numgrid.h()-8)),
            sformat("Key:").c_str(),
            PixRGB<byte>(255),
            PixRGB<byte>(0),
            SimpleFont::FIXED(10),
            false);
  if (!rawPos->isKeyboardEmpty()) {
    //LINFO("%s", rawPos->getKeyboard());
    writeText(itsDrawings,
              Point2D<int>(getGridCoord(grid,31,numgrid.h()-8)),
              sformat(" %s ",rawPos->getKeyboard()).c_str(),
              //rawPos->getKeyboard(),
              PixRGB<byte>(255),
              PixRGB<byte>(0),
              SimpleFont::FIXED(10),
              false);
  }
  else {
    writeText(itsDrawings,
              Point2D<int>(getGridCoord(grid,31,numgrid.h()-8)),
              sformat("                         ").c_str(),
              PixRGB<byte>(255),
              PixRGB<byte>(0),
              SimpleFont::FIXED(10),
              false);
  }
}


// ######################################################################
void SimulationViewerEyeHand::drawFOA(const Point2D<int> target, const uint tN)
{
  const PixRGB<byte> col = itsEyeStyles[tN].col;
  const int radius = itsMetrics->getFoveaRadius();
  drawCircle(itsDrawings, target, radius, col, 2);
}

// ######################################################################
std::string SimulationViewerEyeHand::craftSVEMOutput(const std::string tfn, 
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

  // write the header line now - only once for each output 
  if(!itsHeaderCrafted)
    {
      writeHeader();
      itsHeaderCrafted = true;
    }
  return output;
}

// ######################################################################
std::string SimulationViewerEyeHand::
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
std::string SimulationViewerEyeHand::craftSMSamples(const rutz::shared_ptr<EyeData> data, 
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

  // now get a bunch of random samples from the saliency map:
  if (itsNumRandomSamples.getVal() < smap.getSize())
    {
      // ok, we want fewer samples than are actually in the
      // saliency map, so choose them randomly:
      for (int k = 0; k < itsNumRandomSamples.getVal(); ++k) 
        {
          if (itsPriorRandomDistro.getVal().empty()) // uniform random
            {
              Point2D<int> randp(randomUpToNotIncluding(smap.getWidth()),
                                 randomUpToNotIncluding(smap.getHeight()));
              output += sformat(" %g %d %d", getLocalMax(smap, randp, rad),
                                randp.i, randp.j);
            }
          else // take random points from file
            {
              const int randn = randomUpToNotIncluding(randPoints.size());
              Point2D<int> randp(randPoints[randn].i >> sml,
                                 randPoints[randn].j >> sml);
              output += sformat(" %g %d %d", getLocalMax(smap, randp, rad),
                                randPoints[randn].i, randPoints[randn].j);
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
std::string SimulationViewerEyeHand::craftSMHistory(const rutz::shared_ptr<EyeData> data,
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
  
  // save history of saliency values at saccade target?
  output += std::string("   ");
      
  // also get saccade start location, scaled down to SM size:
  Point2D<int> pp(data->position());
  pp.i >>= sml; pp.j >>= sml; pp.clampToDims(smap.getDims());
  
  // note: even though the queue may not be full yet, we
  // will always write itsSMhistoryQlen samples here, to
  // facilitate reading the data into matlab using
  // dlmread(). Saliency values for non-existent maps will
  // be set to 0:
  size_t ql = itsSMhistory.size();
  for (uint i = 0; i < itsSMhistoryQlen.getVal(); ++ i) 
    {
      if (i < ql)
        {
          // get min, max and avg saliency values over history:
          float mmi, mma, mmav;
          getMinMaxAvg(itsSMhistory[ql - 1 - i], mmi, mma, mmav);
          
          // and also value at one random location:
          Point2D<int> randp;
          if (itsPriorRandomDistro.getVal().empty()) // uniform random
            {
              randp.i = randomUpToNotIncluding(smap.getWidth());
              randp.j = randomUpToNotIncluding(smap.getHeight());
            }
          else // prior from file
            {
              const int randn =
                randomUpToNotIncluding(randPoints.size());
              randp.i = randPoints[randn].i >> sml;
              randp.j = randPoints[randn].j >> sml;
            }
          
          output +=
            sformat(" %g %g %g %g %g",
                    getLocalMax(itsSMhistory[ql - 1 - i], p, rad),
                    getLocalMax(itsSMhistory[ql - 1 - i], pp, rad),
                    mma, mmav,
                    getLocalMax(itsSMhistory[ql - 1 - i], randp, rad));
        }
      else output += std::string(" 0.0 0.0 0.0 0.0 0.0");
      if(!itsHeaderCrafted) 
        {
          itsOutFields.push_back("hist_sacendsal");
          itsOutFields.push_back("hist_sacstartsal");
          itsOutFields.push_back("hist_avgsal");
          itsOutFields.push_back("hist_maxsal");
          itsOutFields.push_back("hist_randsal");      
        }
    }
  return output;
}

// ######################################################################
void SimulationViewerEyeHand::writeHeader()
{
  LINFO("writing header for file %s", itsOutFname.getVal().c_str());
  (*itsOutFile) << "#" << itsOutFields[0];
  for(uint i = 1; i < itsOutFields.size(); i++)
    (*itsOutFile) << " " << itsOutFields[i];
  (*itsOutFile) << std::endl;
}

// ######################################################################
void SimulationViewerEyeHand::extraSampleProcessing(const rutz::shared_ptr<EyeData>) {}
// empty here, written so that EyeRegion can do extra output on each eye position

// ######################################################################
void SimulationViewerEyeHand::extraSampleProcessing(const rutz::shared_ptr<HandData>) {}
// empty here, written so that EyeRegion can do extra output on each eye position

// ######################################################################
Image< PixRGB<byte> > SimulationViewerEyeHand::getTraj(SimEventQueue& q)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // get the latest retina image:
  Image< PixRGB<byte> > input;
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this, SEQ_ANY))
    input = e->frame().colorByte();
  else
    LFATAL("Could not find required SimEventRetinaImage");

  // make a composite of the input + the drawings:
  Image< PixRGB<byte> > comp = composite(itsDrawings, input);

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
void SimulationViewerEyeHand::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerEyeHand::save1(const ModelComponentSaveInfo& sinfo)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  SimEventQueue *q = dynamic_cast<const SimModuleSaveInfo&>(sinfo).q;

  // update the trajectory:
  Image< PixRGB<byte> > res = getTraj(*q);

  // save results?
  if (itsSaveCombo.getVal() || itsSaveTraj.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

      ofs->writeRGB(res, "T",
                    FrameInfo("SimulationViewerEyeHand trajectory", SRC_POS));

    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
