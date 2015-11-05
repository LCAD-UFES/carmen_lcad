/*!@file Neuro/SimulationViewerHand.C comparison between saliency and
  human hand movement reactions */

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

#include "Neuro/SimulationViewerHand.H"

//#include "Channels/ChannelBase.H"
//#include "Channels/ChannelOpts.H" // for LevelSpec option
#include "Component/OptionManager.H"
//#include "Component/ModelOptionDef.H"
#include "Image/ColorOps.H"    // for contrastModulate(), toRGB()
//#include "Image/CutPaste.H"    // for concatX()
#include "Image/DrawOps.H"     // for colGreyCombo()
#include "Image/FilterOps.H"   // for lowPass3()
//#include "Image/MathOps.H"     // for scramble()
#include "Image/Transforms.H"  // for segmentObjectClean(), contour2D()
#include "Image/ShapeOps.H"    // for rescale()
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SpatialMetrics.H"
#include "Psycho/HandData.H"
#include "Simulation/SimEventQueue.H"
//#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

#include <fstream>
#include <iostream>

// for some reason this is not recognized when it appears in NeuroOpts.C
/*const ModelOptionDef OPT_SMhistoryQlen =
  { MODOPT_ARG(uint), "SMhistoryQ", &MOC_DISPLAY, OPTEXP_CORE,
    "Keep a historical queue of salieny maps (one per new input frame), "
    "and report the history of saliency values over the entire queue "
    "and at a saccade target location, for each saccade",
    "sm-history-qlen", '\0', "<uint>", "0" };
//*/

// ######################################################################
SimulationViewerHand::
SimulationViewerHand(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsSaveTraj(&OPT_SVsaveTraj, this),
  itsSaveCombo(&OPT_SVHandSaveCombo, this),
  itsDisplayHand(&OPT_SVHandDisplay, this),
  itsPatchSize(&OPT_SVpatchSize, this),
  itsEraseMarker(&OPT_SVeraseMarker, this),
  itsMaxComboWidth(&OPT_SVHandMaxComboWidth, this),
  itsDrawings(), itsCurrTime(),  itsFrameNumber(-1), 
  itsHeaderCrafted(false), itsOutFields(), itsHandStyles(), 
  itsTargets()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  this->addSubComponent(itsMetrics);

  // Select HandTrack for the HandControllerType
  getManager().setOptionValString(&OPT_HandControllerType, "HandTrack");
}

// ######################################################################
SimulationViewerHand::~SimulationViewerHand()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SimulationViewerHand::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  SimulationViewer::start1();
}

// ######################################################################
void SimulationViewerHand::stop1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SimulationViewerHand::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  // get the current time (mostly for SVHandRegion)
  itsCurrTime = q.now();
  // any new input frame from the retina? If so, let's initialize or
  // clear all our drawings:
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this)) {
    itsDrawings.resize(e->frame().getDims(), true);
    itsFrameNumber++; // used for derived class SVHandRegion
    // this member is never used in SVHand
  }
  
  // Let's update our sliding caches with the current SM/AGM/etc:
  const Image<float> currsm = getMap(q, false);

  // all right, get the latest retina image (whether new or not) as we
  // will need it to do coordinate transforms, coordinate clippings, etc:
  SeC<SimEventRetinaImage> retev = q.check<SimEventRetinaImage>(this, SEQ_ANY);
  if (retev.is_valid() == false)
    LFATAL("I need some SimEventRetinaImage in the queue to function.");
 
  // any hand-tracker action?
  SeC<SimEventHandTrackerData> e = q.check<SimEventHandTrackerData>(this);
  
  while(e.is_valid()) {
    // parse that event:
    const rutz::shared_ptr<HandData> trackCurrPos = e->data();
    const uint tnum = e->trackerNum();
    const std::string tfn = e->trackerFilename();
    // add entry to style vector when we get a new trackernum
    if (itsHandStyles.size() <= tnum) {
      itsHandStyles.resize(tnum+1);
      
      itsHandStyles[tnum].pSize = itsPatchSize.getVal();
      itsHandStyles[tnum].col = e->trackerColor();
      
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
    e = q.check<SimEventHandTrackerData>(this);
  }
}



// ######################################################################
void SimulationViewerHand::drawHand(const rutz::shared_ptr<HandData> rawPos, const uint tN)
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
  const Dims numgrid (80,60); // Under 640x480, its expected to have 8x8 per grid
  const Dims grid (dims.w()/numgrid.w(),dims.h()/numgrid.h()); 

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
  Point2D<int> XD (0,grid.h()/2); // for difference
  Point2D<int> Y1 (getGridCoord(grid,             1,             1));
  Point2D<int> Y2 (getGridCoord(grid,             1, numgrid.h()-2));
  Point2D<int> YC (grid.w(),(127*(Y2.j-Y1.j)/255)+Y1.j);
  Point2D<int> YD (grid.w()/2,0); // for difference
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
  
  PixRGB<byte> colB;
  // For the buttons
  if (rawPos->isButtonEmpty() == false) {  
    for (uint i = 0; i < rawPos->numButton(); i++) {
      // Blue if pressed, red if not
      if (rawPos->isPressed(i))
        colB = PixRGB<byte> (255, 0, 0); // Red
      else
        colB = PixRGB<byte> (0, 0, 0); // Transparent

      // For buttons position, we want them to be @ bottom of the screen
      // TODO: wrapped around in case the buttons grid over [80,_]

      // Write text of numbers into this thing
      writeText(itsDrawings, Point2D<int>(getGridCoord(grid, 3*i+10, numgrid.h()-5)),
                sformat("%2d",i+1).c_str(), PixRGB<byte>(255), colB,SimpleFont::FIXED(10),false);
  
      // Draw circle
      //drawCircle(itsDrawings, Point2D<int>(getGridCoord(grid, 3*i+3, 1)),6,colB,1);
    }
  }
}

// ######################################################################
void SimulationViewerHand::extraSampleProcessing(const rutz::shared_ptr<HandData>) {}
// empty here, written so that HandRegion can do extra output on each hand position

// ######################################################################
Image< PixRGB<byte> > SimulationViewerHand::getTraj(SimEventQueue& q)
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

  // otherwise, return mega combo; we have two formats: if we have a
  // max-cache, it will be a 4-image format, otherwise a 2-image
  // format:
  const Dims dims = itsDrawings.getDims();
  Image< PixRGB<byte> > ret;

  if (itsSaveCombo.getVal()) {
    ret = concatX(input, itsDrawings);
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
void SimulationViewerHand::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerHand::save1(const ModelComponentSaveInfo& sinfo)
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
                    FrameInfo("SimulationViewerHand trajectory", SRC_POS));

    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
