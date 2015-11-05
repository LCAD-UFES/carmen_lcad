/*!@file Media/SimFrameSeries.H a series of frames as SimModule */

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
// Primary maintainer for this file: Laurent Itti <itti@pollux.usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/SimFrameSeries.H $
// $Id: SimFrameSeries.H 13373 2010-05-09 04:28:40Z lior $
//

#ifndef SIMFRAMESERIES_H_DEFINED
#define SIMFRAMESERIES_H_DEFINED

#include "Component/ModelParam.H"
#include "Media/MediaSimEvents.H"
#include "Raster/GenericFrame.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "Util/SimTime.H"
class InputFrameSeries;
class OutputFrameSeries;

// ######################################################################
//! An InputFrameSeries encapsulated into a SimModule
/*! SimInputFrameSeries encapsulates the functionality of
  InputFrameSeries for use by SimModule objects. The interface is a
  simplified version of that of InputFrameSeries, since now most of
  the I/O is handled through the SimEventQueue blackboard rather than
  through explicit function calls and return values. Rather than
  deriving from InputFrameSeries, we hold an InputFrameSeries as a
  data member. This is because much of the InputFrameSeries interface
  is not used here because it is replaced by posting messages to the
  SimEventQueue. */
class SimInputFrameSeries : public SimModule
{
public:
  //! Constructor
  SimInputFrameSeries(OptionManager& mgr,
                      const std::string& descrName = "Sim Input Frame Series",
                      const std::string& tagName = "SimInputFrameSeries");

  //! Destructor
  virtual ~SimInputFrameSeries();

  //! Get specifications of the image frames
  /*! It is okay to call this before the model is started().  If input
      resizing is being done, the returned dimensions will be the
      resized dims.  All in all, this will return the size of whatever
      you will get when calling readFrame(), readRGB(), etc. */
  virtual GenericFrameSpec peekFrameSpec();

  //! Optional call to efficiently prepare for frame streaming
  virtual void startStream();

  //! Set the frame source; same as doing --in=source on the command line
  void setFrameSource(const std::string& source);

  //! Get the current frame number
  int frame() const;

protected:
  //! Callback function triggered at each clock tick
  SIMCALLBACK_DECLARE(SimInputFrameSeries, SimEventClockTick);

private:
  nub::ref<InputFrameSeries> itsIFS;
};



// ######################################################################
//! An OutputFrameSeries encapsulated into a SimModule
/*! SimOutputFrameSeries encapsulates the functionality of
  OutputFrameSeries for use by SimModule objects. The interface is a
  simplified version of that of OutputFrameSeries, since now most of
  the I/O is handled through the SimEventQueue blackboard rather than
  through explicit function calls and return values. Rather than
  deriving from OutputFrameSeries, we hold an OutputFrameSeries as a
  data member. This is because much of the OutputFrameSeries interface
  is not used here because it is replaced by posting messages to the
  SimEventQueue. */
class SimOutputFrameSeries : public SimModule
{
public:
  //! Constructor
  SimOutputFrameSeries(OptionManager& mgr,
                       const std::string& descrName="Sim Output Frame Series",
                       const std::string& tagName = "SimOutputFrameSeries");

  //! Destructor
  virtual ~SimOutputFrameSeries();

  //! Check if we have no output destinations (e.g., user gave --out=none)
  /*! Clients can test isVoid() before they generate potentially
      expensive output images. This is just a performance optimization
      -- even if isVoid() is true, it's still safe to call writeRGB(),
      writeGray(), etc., but those calls will do nothing, and so any
      time spent computing the image will have been wasted. */
  virtual bool isVoid() const;

  //! Add a frame destination; same as doing --out=source on the command line
  void addFrameDest(const std::string& dest);

  //! Get the current frame number
  int frame() const;

protected:
  //! Callback function at triggered at each clock tick
  SIMCALLBACK_DECLARE(SimOutputFrameSeries, SimEventClockTick);

  //! Callback function to check for requests to save outputs
  SIMCALLBACK_DECLARE(SimOutputFrameSeries, SimEventRequestSaveOutput);

  //! update
  void update(SimEventQueue& q, const bool saveRequested);

  //! Simulation time step
  OModelParam<SimTime> itsTimeStep;

private:
  nub::ref<OutputFrameSeries> itsOFS;
  bool itsSaveRequested;
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
