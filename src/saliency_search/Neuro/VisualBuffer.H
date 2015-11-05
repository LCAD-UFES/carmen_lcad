/*!@file Neuro/VisualBuffer.H Grab ausio samples from /dev/dsp */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualBuffer.H $
// $Id: VisualBuffer.H 10779 2009-02-06 01:36:27Z itti $
//

#ifndef VISUALBUFFER_H_DEFINED
#define VISUALBUFFER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Image/fancynorm.H"
#include "Neuro/WTAwinner.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimModule.H"
#include "Util/SimTime.H"
#include "Util/Types.H"

// ######################################################################
//! An integrative visual buffer, base class
// ######################################################################
/*! A world-centered Visual Buffer to accumulate interesting locations
  (each time a covert attention shift occurs), with internal
  competitive dynamics within the buffer. Base class does nothing. */
class VisualBuffer : public SimModule
{
public:
  //! Constructor
  VisualBuffer(OptionManager& mgr,
               const std::string& descrName = "Visual Buffer",
               const std::string& tagName = "VisualBuffer");

  //! Destructor
  virtual ~VisualBuffer();
};

// ######################################################################
//! An integrative visual buffer, stub implementation, does nothing
// ######################################################################
class VisualBufferStub : public VisualBuffer
{
public:
  //! Constructor
  VisualBufferStub(OptionManager& mgr,
               const std::string& descrName = "Visual Buffer Stub",
               const std::string& tagName = "VisualBufferStub");

  //! Destructor
  virtual ~VisualBufferStub();
};

// ######################################################################
//! VisualBuffer configurator
// ######################################################################
/*! This will export the --vb-type=XX command-line option and will
  instantiate a VisualBuffer of the desired type as the option gets assigned a
  value. As this happens, new options may become available in the
  command-line. To see them, use --help AFTER you have chosen the type
  to use. The current VB may be retrieved using getVB(). */
class VisualBufferConfigurator : public ModelComponent
{
public:
  //! Constructor
  VisualBufferConfigurator(OptionManager& mgr,
                          const std::string& descrName = "Visual Buffer Configurator",
                          const std::string& tagName = "VisualBufferConfigurator");

  //! destructor
  virtual ~VisualBufferConfigurator();

  //! Get the chosen VB
  /*! You should call this during start() of the ModelComponent that
      needs the VB. NOTE: this will never return a null pointer (since
      it's a nub::ref rather than a nub::soft_ref), but it may return
      a "stub" object (e.g. VisualBufferStub). */
  nub::ref<VisualBuffer> getVB() const;

protected:
  OModelParam<std::string> itsVBtype; //!< type of buffer

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    buffer of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<VisualBuffer> itsVB; // the buffer
};

// ######################################################################
//! An integrative visual buffer, std implementation
// ######################################################################
/*! A world-centered Visual Buffer to accumulate interesting locations
  (each time a covert attention shift occurs), with internal
  competitive dynamics within the buffer. */
class VisualBufferStd : public VisualBuffer
{
public:
  //! Constructor
  VisualBufferStd(OptionManager& mgr,
               const std::string& descrName = "Visual Buffer",
               const std::string& tagName = "VisualBuffer");

  //! Destructor
  virtual ~VisualBufferStd();

protected:
  //! Callback for every agm
  SIMCALLBACK_DECLARE(VisualBufferStd, SimEventAttentionGuidanceMapOutput);

  //! Callback for every agm
  SIMCALLBACK_DECLARE(VisualBufferStd, SimEventRetinaImage);

  //! New input
  /*! Beware that the inputs here are matched to what Brain naturally
    provides, which is
    @param win the coordinates of an atteention shift at retinal resolution
    @param sm the saliency map at the maplevel resolution
    @param objmask a mask with (if initialized) the shape of the attended
    object at retinal resolution. */
  void input(const WTAwinner& win, const Image<float>& sm, const Image<byte>& objmask);

  //! Get saliency mask
  /*! This is mostly for display purposes. The mask is the one that
    was created at the last input() and has the same size/scale as the
    saliency map and is in retinal coordinates */
  Image<byte> getSaliencyMask() const;

  //! Get current buffer
  Image<float> getBuffer() const;

  //! evolve our internal dynamics
  void evolve(SimEventQueue& q);

  //! inhibit a location, given in retinotopic/retinal-scale coords
  /*! A disk of radius FOAradius will be used for inhibition */
  void inhibit(const Point2D<int>& loc);

  //! inhibit a location, using a mask of same dims/scale as the buffer
  /*! @param mask should have same dims as the buffer, and values
    between 0.0 (no inhibition) and 1.0 (full inhibition). */
  void inhibit(const Image<float>& mask);

  //! Decide on most interesting target location given current eye position
  /*! All coordinates are in world-centered/sm-scale. The algorithm
    used here is inspired from the area activation model; we threshold
    out regions lower than a fraction of the maximum activation, then
    aim for the centroid of the region that is closest to current eye
    position. */
  Point2D<int> findMostInterestingTarget(const Point2D<int>& p);

  //! Decide on most interesting target location given current eye position
  /*! All coordinates are in world-centered/sm-scale. The algorithm
    used here will aim for the local max above a threshold that is
    closest to current eye position. */
  Point2D<int> findMostInterestingTargetLocMax(const Point2D<int>& p);

  //! transform coord from retinotopic/retinal-scale to world-centered/sm-scale
  Point2D<int> retinalToBuffer(const Point2D<int>& p) const;

  //! transform coord from world-centered/sm-scale to retinotopic/retinal-scale
  Point2D<int> bufferToRetinal(const Point2D<int>& p) const;

  //! are we object-based?
  bool isObjectBased() const;

  OModelParam<int> itsFOAradius;        //!< FOA radius
  OModelParam<bool> itsIgnoreBoring;   //!< ignore boring attention shifts
  OModelParam<bool> itsObjectBased;    //!< true if doing object-based
  OModelParam<Dims> itsBufferDims;     //!< dims of our internal buffer
  OModelParam<LevelSpec> itsLevelSpec; //!< levelspec determines buffer scale
  OModelParam<Dims> itsInputDims;      //!< input image dims
  OModelParam<SimTime> itsTimePeriod;   //!< period at which to apply interaction
  OModelParam<float> itsDecayFactor;   //!< temporal decay factor
  OModelParam<MaxNormType> itsNormType;//!< maxnorm type for our internals

  //! get started (see ModelComponent.H)
  void start1();

private:
  // apply one iteration of our internal dynamics
  void internalDynamics();

  Image<float> itsBuffer;
  Dims itsSMdims;
  Image<byte> itsSaliencyMask;
  SimTime itsTime;
  SimTime itsLastInteractTime;
  Point2D<int> itsRetinaOffset;
};


// ######################################################################
//! Helper function to transform from buffer to retinal coordinates
/*! You normally would not use this function directly, but rather
    VisualBufferStd::bufferToRetinal() or
    SimEventVisualBufferOutput::bufferToRetinal(). This function is
    here mostly so that both implementations of bufferToRetinal are
    always consistent. */
inline Point2D<int> visualBufferToRetinal(const Point2D<int>& p, const Point2D<int>& retinaOffset,
                                          const int smlev, const Dims& smdims, const Dims& bufdims)
{
  // re-center out of the larger visual buffer:
  Point2D<int> pp = p - Point2D<int>((bufdims.w() - smdims.w()) / 2, (bufdims.h() - smdims.h()) / 2);

  // scale back up to retinal scale (we do like in WTAwinner.H):
  pp.i <<= smlev; pp.j <<= smlev;

  // then add our current eye position. Caution: make sure this is like rawToRetinal():
  pp += retinaOffset;

  return pp;
}

// ######################################################################
//! Helper function to transform from buffer to retinal coordinates
/*! You normally would not use this function directly. */
inline Point2D<int> retinalToVisualBuffer(const Point2D<int>& p, const Point2D<int>& retinaOffset,
                                          const int smlev, const Dims& smdims, const Dims& bufdims)
{
  // first subtract our current eye position. Caution: Make sure this is the same as retinalToRaw():
  Point2D<int> pp = p - retinaOffset;

  // then downscale to maplevel (we do like in WTAwinner.H, but no random jitter):
  pp.i = int(pp.i / double(1 << smlev) + 0.49);
  pp.j = int(pp.j / double(1 << smlev) + 0.49);

  // then re-center into the larger visual buffer:
  pp += Point2D<int>((bufdims.w() - smdims.w()) / 2, (bufdims.h() - smdims.h()) / 2);

  return pp;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
