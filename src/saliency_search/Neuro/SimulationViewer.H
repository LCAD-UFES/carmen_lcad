/*!@file Neuro/SimulationViewer.H visualize various model simulations */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewer.H $
// $Id: SimulationViewer.H 14518 2011-02-17 00:48:30Z dberg $
//

#ifndef SIMULATIONVIEWER_H_DEFINED
#define SIMULATIONVIEWER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Media/MediaSimEvents.H"
#include "Simulation/SimModule.H"
#include "SpaceVariant/SpaceVariantModule.H"//module for space var
#include "Neuro/WTAwinner.H"//for WTAwinner
#include "Neuro/NeuroSimEvents.H"

template <class T> class Image;

//! This class provides an abstract interface for a simulation viewer
/*! Its typical usage is with a model that generates shifts of
  attention, eye and head movements. Derivatives from this class will
  handle visualizing those events. See SimulationViewerStd.H for an
  example of such derivative. The base class described here just
  specifies the interface. */
class SimulationViewer : public SimModule {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
    /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  SimulationViewer(OptionManager& mgr,
                   const std::string& descrName = "Simulation Viewer",
                   const std::string& tagName = "SimulationViewer");

  //! Destructor
  virtual ~SimulationViewer();

  //@}

protected:
  //! Callback for when a new scene description is available
  SIMCALLBACK_DECLARE(SimulationViewer, SimEventSceneDescription);

  //! Callback for when a new object description is available
  SIMCALLBACK_DECLARE(SimulationViewer, SimEventObjectDescription);

  OModelParam<bool> itsDisplayInterp;  //!< use interpolation to display maps?
  OModelParam<float> itsMapFactor;     //!< factor to use to display maps
  OModelParam<std::string> itsMapType; //!< type of map to use for displays

  //! Helper function to get the latest ''saliency map''
  /*! What is returned here could be SM, AGM, TRM, VCO, etc in
    normalized or un-normalized form, depending upon the values of
    itsMapFactor and itsMapType. If warn is true an error will be
    displayed if no suitable map is found in the event queue */
  Image<float> getMap(SimEventQueue& q, const bool warn = true) const;

  //! the map may be in non-cartesian coords, and an inverse may need to be applied
  virtual Image<float> inverseMap(const Image<float>& map_image) const
  {return map_image; };
};

// ######################################################################
/*! Its simulation viewer that implements some basic catching of
  events and inverse transforms non-cartesian retinal images for
  display if necessary */
// ######################################################################
class SimulationViewerAdapter : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
    /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  SimulationViewerAdapter(OptionManager& mgr,
                          const std::string& descrName = "Simulation Viewer",
                          const std::string& tagName = "SimulationViewerAdapter");

  //! Destructor
  virtual ~SimulationViewerAdapter() { };
  
  //@}
  
protected:
  //! Callback for when a new retina image is available
  SIMCALLBACK_DECLARE(SimulationViewerAdapter, SimEventRetinaImage);
  
  //! Callback for when a new attention shift occurs
  SIMCALLBACK_DECLARE(SimulationViewerAdapter, SimEventWTAwinner);
  
  OModelParam<bool> itsInverseRetinal; //!< should we inverse our transforms for display
  
  Image< PixRGB<byte> > itsInput; // clean input image 
  WTAwinner itsCurrFOA;           // current attention position
  WTAwinner itsPrevFOA;           // previous attention position

  //! Reset SimulationViewerStd
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();
  
  //! so derived classes can process retinal images
  virtual void doEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e) { };

  // so derived classes can process wta winners
  virtual void doEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e) { };

  //!inverse a space variant transformed retinal image
  Image<float> inverseRetinal(const Image<float>& ret_image) const;
  
  //!inverse a space variant transformed map
  virtual Image<float> inverseMap(const Image<float>& map_image) const;

  //!put a point in retinal coords
  void toRetinal(Point2D<int>& point) const;

  //!get a point from retinal coords
  void fromRetinal(Point2D<int>& point) const;

  //!get a point from retinal coords at the map level
  void fromRetinalMap(Point2D<int>& point) const;

  //! hold potential transforms
  nub::ref<SpaceVariantModule> itsTransform;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
