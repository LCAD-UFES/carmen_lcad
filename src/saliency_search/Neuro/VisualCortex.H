/*!@file Neuro/VisualCortex.H The early visual processing stages */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortex.H $
// $Id: VisualCortex.H 12574 2010-01-15 02:09:17Z itti $
//

#ifndef VISUALCORTEX_H_DEFINED
#define VISUALCORTEX_H_DEFINED

#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEvents.H"
#include "Simulation/SimModule.H"

class RawVisualCortex;
class IntegerRawVisualCortex;
class IntegerMathEngine;
class EnvVisualCortexFloat;

// ######################################################################
//! The Visual Cortex Base Class
class VisualCortex : public SimModule
{
public:
  // ######################################################################
  /*! @name Constructors/Destructor */
  //@{

  //! Constructor
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  VisualCortex(OptionManager& mgr,
               const std::string& descrName = "Visual Cortex",
               const std::string& tagName = "VisualCortex");

  //! Virtual destructor for safe inheritance.
  virtual ~VisualCortex();

  //@}

private:
  VisualCortex(const VisualCortex&); // not allowed
  VisualCortex& operator=(const VisualCortex&); // not allowed

};

// ######################################################################
//! The Standard Visual Cortex
/*! This basically wraps the functionality of RawVisualCortex. */
class VisualCortexStd : public VisualCortex
{
public:
  // ######################################################################
  /*! @name Constructors/Destructor */
  //@{

  //! Constructor
  VisualCortexStd(OptionManager& mgr,
                  const std::string& descrName = "Visual Cortex",
                  const std::string& tagName = "VisualCortex");

  //! Constructor using an existing RawVisualCortex
  VisualCortexStd(OptionManager& mgr,
                  const std::string& descrName,
                  const std::string& tagName,
                  nub::ref<RawVisualCortex> vcx);

  //! Virtual destructor for safe inheritance.
  virtual ~VisualCortexStd();

  //@}

protected:
  //! Callback for when a new input (retina) frame is available
  SIMCALLBACK_DECLARE(VisualCortexStd, SimEventRetinaImage);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(VisualCortexStd, SimEventSaveOutput);

  //! Callback for every time someone wants our features
  SIMREQHANDLER_DECLARE(VisualCortexStd, SimReqVCXfeatures);

  //! Callback for every time someone wants our maps
  SIMREQHANDLER_DECLARE(VisualCortexStd, SimReqVCXmaps);

  //! Callback for every time someone wants to trigger a channel visitor
  SIMREQHANDLER_DECLARE(VisualCortexStd, SimReqVCXchanVis);

private:
  friend class VisualCortexConfigurator; // allow the configurator to configure our VCX
  nub::ref<RawVisualCortex> itsVCX;
};

// ######################################################################
//! An Integer-based VisualCortex
/*! This basically wraps the functionality of
    IntegerRawVisualCortex. This uses fixed-point integer computations
    but remains quite flexible and similar to RawVisualCortex. */
class VisualCortexInt : public VisualCortex
{
public:
  // ######################################################################
  /*! @name Constructors/Destructor */
  //@{

  //! Constructor
  VisualCortexInt(OptionManager& mgr,
                  const std::string& descrName = "Visual Cortex",
                  const std::string& tagName = "VisualCortex");

  //! Constructor using an existing IntegerRawVisualCortex
  VisualCortexInt(OptionManager& mgr,
                  const std::string& descrName,
                  const std::string& tagName,
                  nub::ref<IntegerMathEngine> ime,
                  nub::ref<IntegerRawVisualCortex> vcx);

  //! Virtual destructor for safe inheritance.
  virtual ~VisualCortexInt();

  //@}

protected:
  //! Callback for when a new input (retina) frame is available
  SIMCALLBACK_DECLARE(VisualCortexInt, SimEventRetinaImage);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(VisualCortexInt, SimEventSaveOutput);

  //! Callback for every time someone wants our features
  SIMREQHANDLER_DECLARE(VisualCortexInt, SimReqVCXfeatures);

  //! Callback for every time someone wants our maps
  SIMREQHANDLER_DECLARE(VisualCortexInt, SimReqVCXmaps);

  //! Callback for every time someone wants to trigger a channel visitor
  SIMREQHANDLER_DECLARE(VisualCortexInt, SimReqVCXchanVis);

private:
  friend class VisualCortexConfigurator; // allow the configurator to configure our VCX
  nub::ref<IntegerMathEngine> itsIME;
  nub::ref<IntegerRawVisualCortex> itsVCX;
  PyramidCache<int> itsPcache;
};

// ######################################################################
//! An optimized C-based Integer-based VisualCortex
/*! This used the EnvVisualCortex, it is fast but not flexible in
    terms of which channels are used, etc. */
class VisualCortexEnv : public VisualCortex
{
public:
  // ######################################################################
  /*! @name Constructors/Destructor */
  //@{

  //! Constructor
  VisualCortexEnv(OptionManager& mgr,
                  const std::string& descrName = "Visual Cortex",
                  const std::string& tagName = "VisualCortex");

  //! Virtual destructor for safe inheritance.
  virtual ~VisualCortexEnv();

  //@}

protected:
  //! Callback for when a new input (retina) frame is available
  SIMCALLBACK_DECLARE(VisualCortexEnv, SimEventRetinaImage);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(VisualCortexEnv, SimEventSaveOutput);

  //! Callback for every time someone wants our features
  SIMREQHANDLER_DECLARE(VisualCortexEnv, SimReqVCXfeatures);

  //! Callback for every time someone wants our maps
  SIMREQHANDLER_DECLARE(VisualCortexEnv, SimReqVCXmaps);

  //! Callback for every time someone wants to trigger a channel visitor
  SIMREQHANDLER_DECLARE(VisualCortexEnv, SimReqVCXchanVis);

private:
  friend class VisualCortexConfigurator; // allow the configurator to configure our VCX
  nub::ref<EnvVisualCortexFloat> itsVCX;
};


/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
