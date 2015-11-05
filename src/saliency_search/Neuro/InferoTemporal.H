/*!@file Neuro/InferoTemporal.H Object recognition module */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporal.H $
// $Id: InferoTemporal.H 10747 2009-02-03 08:24:04Z itti $
//

#ifndef INFEROTEMPORAL_H_DEFINED
#define INFEROTEMPORAL_H_DEFINED

#include "Channels/InputFrame.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Pixels.H"
#include "Image/Image.H"
#include "Neuro/NeuroSimEvents.H"
#include "SIFT/VisualObjectMatchAlgo.H"
#include "Simulation/SimModule.H"

class VisualCortex;
class VisualObjectDB;
class Brain;

namespace nub { template <class T> class ref; }

// ######################################################################
//! Abstract inferotemporal interface
class InferoTemporal : public SimModule
{
public:
  //! Constructor
  InferoTemporal(OptionManager& mgr,
                 const std::string& descrName,
                 const std::string& tagName);

  //! Destructor
  virtual ~InferoTemporal();

protected:
  //! Callback for when a new WTA winner is available
  SIMCALLBACK_DECLARE(InferoTemporal, SimEventWTAwinner);

  //! Object recognition
  virtual void attentionShift(SimEventQueue& q, const Point2D<int>& location) = 0;
};

// ######################################################################
//! Do-nothing stub implementation of InferoTemporal
class InferoTemporalStub : public InferoTemporal
{
public:
  //! Constructor
  InferoTemporalStub(OptionManager& mgr,
                     const std::string& descrName = "Infero Temporal Stub",
                     const std::string& tagName = "InferoTemporalStub");

  //! Destructor
  virtual ~InferoTemporalStub();

protected:
  //! No-op stub
  virtual void attentionShift(SimEventQueue& q, const Point2D<int>& location);
};

// ######################################################################
//! InferoTemporalStd
/*! Model of the Inferior Temporal Cortex utilizing SIFT object
    recognition. */
class InferoTemporalStd : public InferoTemporal
{
public:
  //! Constructor
  InferoTemporalStd(OptionManager& mgr,
                    const std::string& descrName = "Infero Temporal Std",
                    const std::string& tagName = "InferoTemporalStd");

  //! Destructor
  virtual ~InferoTemporalStd();

protected:
  //! Object recognition
  virtual void attentionShift(SimEventQueue& q, const Point2D<int>& location);

protected:
  OModelParam<bool> itsUseAttention;              //!< attention modulated reco
  OModelParam<std::string> itsObjectDatabaseFile; //!< obj knowledgebase file
  OModelParam<bool> itsTrainObjectDB;             //!< train the database
  OModelParam<bool> itsPromptUserTrainDB;         //!< prompt before training
  OModelParam<bool> itsMatchObjects;              //!< match new objects
  OModelParam<int> itsRecogMinMatch;              //!< min matches for recog
  OModelParam<VisualObjectMatchAlgo> itsMatchingAlg; //!< matching algo to use

  rutz::shared_ptr<VisualObjectDB> itsObjectDB; //!< our object database

  InputFrame itsInputFrame;

  //! (re-)initialization - load object database
  virtual void start1();

  //! destruction - save object database
  virtual void stop1();
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
