/*!@file Media/MediaSimEvents.H SimEvent derivatives for media modules */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MediaSimEvents.H $
// $Id: MediaSimEvents.H 14286 2010-12-01 17:46:34Z sophie $
//

#ifndef MEDIA_MEDIASIMEVENTS_H_DEFINED
#define MEDIA_MEDIASIMEVENTS_H_DEFINED

#include "Raster/GenericFrame.H"
#include "Simulation/SimEvent.H"
#include "Transport/FrameInfo.H"
#include "Media/TestImages.H"
#include "Media/FrameRange.H"

class FrameOstream;

// ######################################################################
//! A new input frame is available
/*! InputFrameSeries posts this event each time a new input frame has
  become available. */
class SimEventInputFrame : public SimEvent {
public:
  //! Constuctor
  SimEventInputFrame(SimModule* src,
                     const GenericFrame& fram,
                     const int framenum);

  //! Destructor
  virtual ~SimEventInputFrame();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the frame
  const GenericFrame& frame() const;

  //! Get the frame number
  int frameNum() const;

private:
  const GenericFrame itsFrame;
  const int itsFrameNum;
};

// ######################################################################
//! Request that outputs be saved now
/*! Normally, SimOutputFrameSeries will save outputs at a fixed
  rate. However, it can also operate in an event-driven mode if a
  framerate has not been specified on the command line. SimModule
  objects may post SimEventSaveOutput events each time they generate
  events which they think warrant saving the outputs should the output
  frame series operate in event-driven mode. Any SimEventSaveOutput
  will be ignored if the output frame series is operating at a fixed
  framerate. An example is that when WinnerTakeAll has a new covert
  shift of attention, it will post a SimEventSaveOutput. */
class SimEventRequestSaveOutput : public SimEvent {
public:
  //! Constuctor
  SimEventRequestSaveOutput(SimModule* src);

  //! Destructor
  virtual ~SimEventRequestSaveOutput();
};

// ######################################################################
//! Send the user mouse and keyboard events
class SimEventUserInput : public SimEvent {
public:
  //! Constuctor
  SimEventUserInput(SimModule* src, const char* wname, const Point2D<int> clickLoc, const int key);

  //! Destructor
  virtual ~SimEventUserInput();

  //! get the window name that generated the event
  const char* getWinName() const;

  //! Get the mouse event
  Point2D<int> getMouseClick() const;

  //! get the keyboard input
  int getKey() const;

private:
  const char* itsWname;
  const Point2D<int> itsClick;
  const int itsKey;

};

// ######################################################################
//! Get a description of the current input
/*! This message can be sent just after initialization so that downstream
    modules can be informed of the number of frames in the input, and the
    name of the input file. */
class SimEventInputDescription : public SimEvent {
  public:
    //! Constructor
    SimEventInputDescription(SimModule* src, const FrameRange range, const std::string name);

    //! Destructor
    virtual ~SimEventInputDescription();

    //! Get the frame range for the input
    FrameRange getFrameRange() const;

    //! Get a description of the input (a filename, camera source, etc.)
    std::string getName() const;

  private:
    FrameRange itsFrameRange;
    std::string itsName;

};

// ######################################################################
//! Request that the input module seek to a new frame
class SimEventRequestFrameNum : public SimEvent {
  public:
    //! Constructor
    SimEventRequestFrameNum(SimModule* src, int frameNum);

    //! Destructor
    virtual ~SimEventRequestFrameNum();

    //! Get the requested frame number
    int getFrameNum() const;

  private:
    int itsFrameNum;
};


// ######################################################################
//! A Description of the scene
/*! Any module can use this event to post knowledge of the scene. For example gist recognition
  can determine the scene type and post this event. */
class SimEventSceneDescription : public SimEvent {
public:
  //! Constuctor
  SimEventSceneDescription(SimModule* src,
      const rutz::shared_ptr<TestImages::SceneData> sceneData);

  //! Destructor
  virtual ~SimEventSceneDescription();

  rutz::shared_ptr<TestImages::SceneData> getSceneData() const;
private:
  rutz::shared_ptr<TestImages::SceneData> itsSceneData;
};

// ######################################################################
//! A Description of an object
/*! Any module can use this event to post knowledge of an object. For example IT
  can determine the object type and post this event. */
class SimEventObjectDescription : public SimEvent {
public:
  //! Constuctor
  SimEventObjectDescription(SimModule* src,
      const rutz::shared_ptr<TestImages::ObjData> objData);

  //! Destructor
  virtual ~SimEventObjectDescription();

  rutz::shared_ptr<TestImages::ObjData> getObjData() const;
private:
  rutz::shared_ptr<TestImages::ObjData> itsObjData;
};


// ######################################################################
//! The output of the IT cortex, i.e. information about the object
/*! Any module can use this event to post knowledge of an object. For example IT
  can determine the object type and post this event. */
class SimEventITOutput : public SimEvent {
public:
  //! Constuctor
  SimEventITOutput(SimModule* src,
      const rutz::shared_ptr<TestImages::ObjData> objData);

  //! Destructor
  virtual ~SimEventITOutput();

  rutz::shared_ptr<TestImages::ObjData> getObjData() const;
private:
  rutz::shared_ptr<TestImages::ObjData> itsObjData;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MEDIASIMEVENTS_H_DEFINED
