/*!@file Media/MediaSimEvents.C SimEvent derivatives for media modules */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MediaSimEvents.C $
// $Id: MediaSimEvents.C 14286 2010-12-01 17:46:34Z sophie $
//

#include "Media/MediaSimEvents.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"

// ######################################################################
SimEventInputFrame::SimEventInputFrame(SimModule* src,
                                       const GenericFrame& fram,
                                       const int framenum) :
  SimEvent(src), itsFrame(fram), itsFrameNum(framenum)
{ }

SimEventInputFrame::~SimEventInputFrame()
{ }

std::string SimEventInputFrame::toString() const
{
  return SimEvent::toString() +
    sformat(", frame=%06d (%dx%d)[%s]", itsFrameNum,
            itsFrame.getDims().w(), itsFrame.getDims().h(),
            itsFrame.nativeTypeName().c_str());
}

const GenericFrame& SimEventInputFrame::frame() const
{ return itsFrame; }

int SimEventInputFrame::frameNum() const
{ return itsFrameNum; }

// ######################################################################
SimEventRequestSaveOutput::SimEventRequestSaveOutput(SimModule* src) :
  SimEvent(src)
{ }

SimEventRequestSaveOutput::~SimEventRequestSaveOutput()
{ }

// ######################################################################
SimEventUserInput::SimEventUserInput(SimModule* src, const char* wname, const Point2D<int> clickLoc, const int key) :
  SimEvent(src),
  itsWname(wname),
  itsClick(clickLoc),
  itsKey(key)
{ }

SimEventUserInput::~SimEventUserInput()
{ }

const char* SimEventUserInput::getWinName() const
{
  return itsWname;
}
Point2D<int> SimEventUserInput::getMouseClick() const
{
  return itsClick;
}
int SimEventUserInput::getKey() const
{
  return itsKey;
}


// ######################################################################
SimEventInputDescription::SimEventInputDescription(SimModule* src, const FrameRange range, const std::string name) :
  SimEvent(src),
  itsFrameRange(range),
  itsName(name)
{ }

SimEventInputDescription::~SimEventInputDescription()
{ }

FrameRange SimEventInputDescription::getFrameRange() const
{
  return itsFrameRange;
}

std::string SimEventInputDescription::getName() const 
{
  return itsName;
}

// ######################################################################
SimEventRequestFrameNum::SimEventRequestFrameNum(SimModule* src, const int frameNum) :
  SimEvent(src),
  itsFrameNum(frameNum)
{ }

SimEventRequestFrameNum::~SimEventRequestFrameNum()
{ }

int SimEventRequestFrameNum::getFrameNum() const
{
  return itsFrameNum;
}

// ######################################################################
SimEventSceneDescription::SimEventSceneDescription(SimModule* src,
      const rutz::shared_ptr<TestImages::SceneData> sceneData):
  SimEvent(src), itsSceneData(sceneData)
{ }

SimEventSceneDescription::~SimEventSceneDescription()
{ }

rutz::shared_ptr<TestImages::SceneData> SimEventSceneDescription::getSceneData() const
{
  return itsSceneData;
}

// ######################################################################
SimEventObjectDescription::SimEventObjectDescription(SimModule* src,
      const rutz::shared_ptr<TestImages::ObjData> objData):
  SimEvent(src), itsObjData(objData)
{ }

SimEventObjectDescription::~SimEventObjectDescription()
{ }

rutz::shared_ptr<TestImages::ObjData> SimEventObjectDescription::getObjData() const
{
  return itsObjData;
}

// ######################################################################
SimEventITOutput::SimEventITOutput(SimModule* src,
      const rutz::shared_ptr<TestImages::ObjData> objData):
  SimEvent(src), itsObjData(objData)
{ }

SimEventITOutput::~SimEventITOutput()
{ }

rutz::shared_ptr<TestImages::ObjData> SimEventITOutput::getObjData() const
{
  return itsObjData;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
