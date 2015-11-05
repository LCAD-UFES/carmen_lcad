/*!@file Neuro/Retina.H simulation of a human retina */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/Retina.H $
// $Id: Retina.H 14634 2011-03-24 00:33:16Z dberg $
//

#ifndef RETINA_H_DEFINED
#define RETINA_H_DEFINED

#include "Channels/InputFrame.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Image/LevelSpec.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "SpaceVariant/SpaceVariantModule.H"
#include "SpaceVariant/SVChanLevels.H"
#include "Simulation/SimEvents.H"
#include "Simulation/SimModule.H"
#include "Util/Timer.H"
#include "Util/Types.H"

class ModelManager;
class SpatialMetrics;

// ######################################################################
//! The retina base class
/*! The Retina is the entry point to the Brain and VisualCortex, and this is a base class that mostly specifies the
  interface. Every input image to be processed first passes through the Retina (which is a subcomponent of Brain). The
  Retina may transform the input in various ways, e.g., apply a foveation filter, embed the input into a larger
  background framing image, or shift the image to current eye position. The base class does nothing in order to not
  constrain the interface. Derived classes implement different strategies. */
class Retina : public SimModule
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  Retina(OptionManager& mgr, const std::string& descrName = "Retina",
         const std::string& tagName = "Retina");

  //! Destructor
  virtual ~Retina();
};

// ######################################################################
class RetinaAdapter : public Retina {
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  RetinaAdapter(OptionManager& mgr, const std::string& descrName = "RetinaAdapter",
         const std::string& tagName = "RetinaAdapter");

  //! Destructor
  virtual ~RetinaAdapter();

protected:
  //! Callback for when a new input frame is available
  SIMCALLBACK_DECLARE(RetinaAdapter, SimEventInputFrame);

  //! Callback for when a new eye movement is available
  SIMCALLBACK_DECLARE(RetinaAdapter, SimEventSaccadeStatusEye);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(RetinaAdapter, SimEventSaveOutput);

  //! filename of a clipmask image to load; or empty string for no clipmask
  OModelParam<std::string> itsClipMaskFname;

  //! Border for our raw input rectangle, in pixels
  OModelParam<int> itsRawInpRectBorder;

  //! Initial eye position (to foveate first frame)
  OModelParam<Point2D<int> > itsInitialEyePosition;

  //! Whether to allow pyramid caches in the InputFrame objects that we create
  OModelParam<bool> itsEnablePyramidCaches;

  //! Overridden so we can load our clip mask file, if any
  virtual void start1();

  //! Radius of our fovea
  OModelParam<int> itsFoveaRadius;

  //! Save our raw input?
  OModelParam<bool> itsSaveInput;

  //! Save our output image?
  OModelParam<bool> itsSaveOutput;

  //! Image file to use for input framing
  OModelParam<std::string> itsFramingImageName;

  //! Offset to apply for input framing
  OModelParam<Point2D<int> > itsFramingImagePos;

  //! Pyramid depth to use when foveating the input, or 0 for no foveation
  OModelParam<uint> itsFoveateInputDepth;

  //! shift input to eye position?
  OModelParam<bool> itsShiftInput;

  //! background color to use when shifting input to eye position
  OModelParam< PixRGB<byte> > itsShiftInputBGcol;

  //! crop input to a given FOV?
  OModelParam<Dims> itsInputFOV;

  //! Save our internal pyramid?
  OModelParam<bool> itsSavePyr;

  //! Blank out visual inputs during blinks?
  OModelParam<bool> itsBlankBlink;

  //! filename of a retinal mask image to load; or empty string for no mask
  OModelParam<std::string> itsRetMaskFname;

  //! flip input image horizontally?
  OModelParam<bool> itsFlipHoriz;

  //! flip input image vertically?
  OModelParam<bool> itsFlipVertic;

  //! Compute a new output
  /*! Do not overload this in derived classes, instead overload transform() below. */
  Image<PixRGB<byte> > getOutput(const Image<PixRGB<byte> >& inp, const Point2D<int>& eyepos, const bool inBlink);

  //! post our input frame with a SimEventRetinaImage message.
  //! Derived classes may want to post messages derived from
  //! SimEventRetinaImage
  virtual void postInputFrame(SimEventQueue& q, InputFrame& ifr);

//! Transform the image after it has already been shifted, cropped, foveated, etc
  virtual Image<PixRGB<byte> > transform(const Image<PixRGB<byte> >& image) = 0;

  //! Get a rectangle delineating the raw input image area
  /*! As the raw input may be embedded into a background, shifted,
    cropped by a field of view, etc., this function allows users to
    recover the boundaries of the original frames. Note that the
    rectangle will be possibly cropped to fit within the dims of the
    Retina's output. A border may be specified that will be used to
    shrink (if positive) or enlarge (if negative) the returned
    rectangle (and will be applied before the rectangle is cropped to
    dims). The default implementation in the base class is to just
    return a rectangle at (0,0) with dims of the raw input. */
  virtual Rectangle getRawInputRectangle(const Dims& indims, const Dims& outdims) const;
  
  //! Get raw to retinal offset
  Point2D<int> getRawToRetinalOffset() const;
  
  //! Save our internals
  /*! Depending on our ModelParam settings, we can save our raw input
      frames (with name prefix "RETIN-"), and/or our foveated output
      images (prefix "RETOUT-"), and/or our pyramid frames (with name
      prefix "RET<level>-"). */
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  Image<byte> itsClipMask;  //!< mask to eliminate parts of scene
  Point2D<int> itsEyePos;   //!< current eye position
  bool itsEyeBlinkStatus;   //!< are we in blink?

  Image<PixRGB<byte> > itsRawInput; //!< copy of input in case we want to save it
  Image<PixRGB<byte> > itsOutput; //!< copy of output in case we want to save it

  Image< PixRGB<byte> > itsFramingImage; //!< image used for framing
  Point2D<int> itsRetinalShift;
  Image<byte> itsRetMask;
  ImageSet< PixRGB<byte> > itsMultiRetina;
};


// ######################################################################
//! Retina configurator
// ######################################################################
/*! This will export the --retina-type=XX command-line option and will
  instantiate a Retina of the desired type as the option gets assigned a
  value. As this happens, new options may become available in the
  command-line. To see them, use --help AFTER you have chosen the type
  to use. The current Retina may be retrieved using getRET(). */
class RetinaConfigurator : public ModelComponent
{
public:
  //! Constructor
  RetinaConfigurator(OptionManager& mgr,
                     const std::string& descrName = "Retina Configurator",
                     const std::string& tagName = "RetinaConfigurator");

  //! destructor
  virtual ~RetinaConfigurator();

  //! Get the chosen Retina
  /*! You should call this during start() of the ModelComponent that
      needs the Retina. This is guaranteed to be a non-null object
      (though it may be "stub" type, e.g. RetinaStub). */
  nub::ref<Retina> getRET() const;

protected:
  OModelParam<std::string> itsType; //!< type of retina

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    retina of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<Retina> itsRET; // the retina
};

// ######################################################################
//! "Stub" Retina implementation
// ######################################################################
/*! Does nothing but copy its input to its output. */
class RetinaStub : public Retina
{
public:
  //! Constructor
  RetinaStub(OptionManager& mgr,
             const std::string& descrName = "RetinaStub",
             const std::string& tagName = "RetinaStub");

  //! Destructor
  virtual ~RetinaStub();

protected:
  //! Callback for when a new input frame is available
  SIMCALLBACK_DECLARE(RetinaStub, SimEventInputFrame);
};

// ######################################################################
//! Standard Retina implementation
// ######################################################################
/*! Can do foveation, shifting, framing, etc all inherited from RetinaAdapter. */
class RetinaStd : public RetinaAdapter
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  RetinaStd(OptionManager& mgr, const std::string& descrName = "RetinaStd",
            const std::string& tagName = "RetinaStd");

  //! Destructor
  virtual ~RetinaStd();

protected:
  //! Transform the image after it has already been shifted, cropped, foveated, etc
  /*! In RetinaStd, this is a no-op */
  virtual Image<PixRGB<byte> > transform(const Image<PixRGB<byte> >& image);
};

// ######################################################################
/*! Space variant transform using a foveated model. Could be used as a model
    of spatial input of retinal ganglion cell's, thalamus, colliculus or 
    early visual cortex */
// ######################################################################
/*! Can do foveation, shifting, flipping, framing in log-polar coords. */
class RetinaCT : public RetinaAdapter
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  RetinaCT(OptionManager& mgr,
           const std::string& descrName = "Cortical Transform",
           const std::string& tagName = "RetinaCT");
  
  //this implementation just returns a rectangle at 0,0 with the size of the input for now
  Rectangle getRawInputRectangle(const Dims& indims, const Dims& outdims) const;
  
  //! Destructor
  virtual ~RetinaCT();
  
protected:
  virtual void start1();
  
  //! post a SimEventRetinaSpaceVariantmage
  virtual void postInputFrame(SimEventQueue& q, InputFrame& ifr);
  
  //! Transform the image after it has already been shifted, cropped, foveated, etc
  /*! In RetinaStd, this is a no-op */
  virtual Image<PixRGB<byte> > transform(const Image<PixRGB<byte> >& image);

  OModelParam<float> itsSurrFac;
  OModelParam<SVChanLevels> itsLevels;
  
private:
  nub::ref<SpaceVariantModule> itsTransform;
  rutz::shared_ptr<PyramidCache<PixRGB<float> > > itsRgbCache;
  rutz::shared_ptr<PyramidCache<float> > itsFloatCache;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
