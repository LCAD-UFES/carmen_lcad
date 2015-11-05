/*!@file Channels/RawVisualCortex.H The early visual processing stages */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/RawVisualCortex.H $
// $Id: RawVisualCortex.H 15258 2012-04-18 23:05:58Z dberg $
//

#ifndef CHANNELS_RAWVISUALCORTEX_H_DEFINED
#define CHANNELS_RAWVISUALCORTEX_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Channels/InputFrame.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Image/fancynorm.H" // for MaxNormType

#include "GUI/XWinManaged.H"


// Forward declarations will suffice instead of #include's here, and will
// be more efficient in compile times and in minimizing dependencies.
template <class T> class PixRGB;
class MgzDecoder;
class MgzEncoder;

// ######################################################################
//! The Visual Cortex Class
/*! In brief, RawVisualCortex holds a collection of ChannelBase objects,
    and most of RawVisualCortex's operations are achieved by some kind of
    iteration over that collection. That is, RawVisualCortex now does
    little work by itself, but delegates its operations to the
    channels, accumulating their results if necessary. NOTE:
    RawVisualCortex has a virtual ModelComponent base which is shared
    among its inheritance from ModelComponent via the ComplexChannel
    inheritance path, and its inheritance from ModelComponent via its
    SimModule inheritance path. Just beware! */
class RawVisualCortex : public ComplexChannel
{
public:
  // ######################################################################
  /*! @name Constructors/Destructor */
  //@{

  //! Construct with no channel; channels must then be added with addSubChan().
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  RawVisualCortex(OptionManager& mgr,
                  const std::string& descrName = "Raw Visual Cortex",
                  const std::string& tagName = "RawVisualCortex");

  //! Virtual destructor for safe inheritance.
  virtual ~RawVisualCortex();

  //@}

  //! Save our results
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

  //! Get the output of the vc map from a given image
  const Image<float> getVCOutput(const Image<PixRGB<byte> >& rgbin);

protected:
  //! Implementation of ChannelBase input() functions
  virtual void doInput(const InputFrame& inframe);

  //! Combine the outputs of our subchannels
  /*! We do this in a slightly different way than the ComplexChannel
      base version. */
  virtual Image<float> combineOutputs();

  //! get weighted (but not resized) channel output map
  virtual Image<float> getChannelOutputMap(const uint idx) const;

  //! get the raw unweighted channel output map
  virtual Image<float> getRawChannelOutputMap(const uint idx) const;

  //! post-process raw weighted sum of channel output maps
  /*! Derived classes may overload this to provide custom
    post-processing. For example, VisualCortexSurprise may pass the
    output through a sigmoidal nonlinearity or spatial
    competition. Default implementation is to treat the special case
    of VCXNORM_LANDMARK. */
  virtual Image<float> postProcessOutputMap(const Image<float>& outmap);

  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  OModelParam<std::string> itsType;
  OModelParam<MaxNormType> itsNormType; //!< maxNormalization to use
  OModelParam<bool> itsUseRandom;       //!< add random noise to output
  OModelParam<float> itsOutputFactor;   //!< output range factor
  OModelParam<float> itsNoise;          //!< output noise range

  /* older version: raw CS submaps -> [normalize, spatial competition,
     sum] -> SO -> [spatial competition, sum] -> CO -> [sum, spatial
     competition] -> VCX o/p :

     Since different feature types may have different range of
     responses, their responses are normalized to a fixed range so as
     to treat them equally. Ideally, normalization should occur at the
     visualCortex when it combines the outputs of the different
     feature types. Instead, the older version performs the
     normalization within each feature type, forcing all scales within
     a feature type to be treated as equally important. This is
     undesirable as it artificially magnifies even those scales which
     contain only noise, and we lose information about the relative
     goodness of the different scales.

     new version: raw CS submaps -> [spatial competition, sum] -> SO
     -> [spatial competition, sum] -> CO -> [normalize, sum, spatial
     competition] -> VCX o/p */
  OModelParam<bool> itsUseOlderVersion;

  //! LevelSpec used by our channels, used to compute output dims
  OModelParam<LevelSpec> itsLevelSpec;

  OModelParam<std::string> itsSaveOutTo; //!< save our outputs to MGZ file?
  OModelParam<std::string> itsLoadOutFrom; //!< load outputs from MGZ file?
  OModelParam<std::string> itsSaveRawCSOutTo; //!< save all the CS maps to an MGZ file?
  OModelParam<bool> itsSaveOutput;      //!< save our output?
  OModelParam<bool> itsUseMax;     //!< use max across features instead of sum?
  OModelParam<float> itsWeightThresh;     //!< threshold on channel total weights for last round of maxnorm
  OModelParam<Dims> itsRawCSDims; //!< dimensions at which to save raw cs maps

  //! start1
  virtual void start1();

  //! start2
  virtual void start2();

  //! stop
  virtual void stop2();

private:
  RawVisualCortex(const RawVisualCortex&); // not allowed
  RawVisualCortex& operator=(const RawVisualCortex&); // not allowed

  // NOTE: We use the low-level MgzEncoder and MgzDecoder objects here
  // instead of the MgzInputStream and MgzOutputStream just to avoid
  // having extra ModelComponent objects which may confuse the
  // VisualCortex if we add them as subcomponents:
  rutz::shared_ptr<MgzDecoder> itsOutputMgzIn; // read precomputed outputs
  rutz::shared_ptr<MgzEncoder> itsOutputMgzOut; // write computed outputs
  rutz::shared_ptr<MgzEncoder> itsRawCSMgzOut; // write computed channel outputs

  rutz::shared_ptr<XWinManaged> itsWin;
  uint itsFrame;
};


/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
