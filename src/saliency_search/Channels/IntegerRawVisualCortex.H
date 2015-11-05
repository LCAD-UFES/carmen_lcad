/*!@file Channels/IntegerRawVisualCortex.H */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerRawVisualCortex.H $
// $Id: IntegerRawVisualCortex.H 13993 2010-09-20 04:54:23Z itti $
//

#ifndef CHANNELS_INTEGERRAWVISUALCORTEX_H_DEFINED
#define CHANNELS_INTEGERRAWVISUALCORTEX_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Image/fancynorm.H" // for MaxNormType
#include "Util/Types.H"
#include "rutz/shared_ptr.h"
#include "Channels/ChannelOpts.H"
#include "Channels/IntegerComplexChannel.H"
#include "Channels/InputFrame.H"

#include <string>
#include <utility>
#include <vector>

// Forward declarations will suffice instead of #include's here, and
// will be more efficient in compile times and in minimizing
// dependencies.
class FrameOstream;
class InputFrame; // see Channels/InputFrame.H
class ParamMap;
class Retina;
class SimTime;
class WTAwinner;
template <class T> class Jet;
namespace rutz { template <class T> class shared_ptr; }
template <class T> class PixRGB;
class MgzDecoder;
class MgzEncoder;

// ######################################################################
//! The Visual Cortex Class
/*! In brief, IntegerRawVisualCortex holds a collection of IntegerChannel
    objects, and most of IntegerRawVisualCortex's operations are achieved
    by some kind of iteration over that collection. That is,
    IntegerRawVisualCortex now does little work by itself, but delegates
    its operations to the channels, accumulating their results if
    necessary. */
class IntegerRawVisualCortex : public IntegerComplexChannel
{
public:
  // ######################################################################
  /*! @name Constructors/Destructor */
  //@{

  //! Construct with no channel; channels must then be added with addSubChan().
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  IntegerRawVisualCortex(OptionManager& mgr,
                      nub::ref<IntegerMathEngine> eng,
                      const std::string& descrName = "Integer Visual Cortex",
                      const std::string& tagName = "IntegerRawVisualCortex");

  //! Virtual destructor for safe inheritance.
  virtual ~IntegerRawVisualCortex();

  //@}

  //! Overload to not only convert to float but also apply our output factor
  virtual Image<float> getOutput();

  //! Save our results
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

protected:
  virtual void doInputInt(const IntegerInput& inp,
                          const SimTime& t,
                          PyramidCache<int>* cache,
                          const Image<byte>& clipMask);

  //! Combine the outputs of our subchannels
  /*! We do this in a slightly different way than the ComplexChannel
      base version. */
  virtual Image<int> combineOutputsInt();

  //! get weighted (but not resized) channel output map
  virtual Image<int> getChannelOutputMap(IntegerChannel& chan) const;

  //! post-process raw weighted sum of channel output maps
  /*! Derived classes may overload this to provide custom
    post-processing. For example, IntegerRawVisualCortexSurprise may pass the
    output through a sigmoidal nonlinearity or spatial
    competition. Default implementation is to treat the special case
    of VCXNORM_LANDMARK. */
  virtual Image<int> postProcessOutputMap(const Image<int>& outmap);

  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  OModelParam<std::string> itsType;
  OModelParam<std::string> itsLogFile;  //!< text log file name
  OModelParam<MaxNormType> itsNormType; //!< maxNormalization to use
  OModelParam<bool> itsUseRandom;       //!< add random noise to output
  OModelParam<float> itsOutputFactor;   //!< output range factor

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
  OModelParam<bool> itsSaveOutput;      //!< save our output?
  OModelParam<bool> itsUseMax;     //!< use max across features instead of sum?

  //! start
  virtual void start1();

  //! stop
  virtual void stop2();

private:
  IntegerRawVisualCortex(const IntegerRawVisualCortex&); // not allowed
  IntegerRawVisualCortex& operator=(const IntegerRawVisualCortex&); // not allowed
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERRAWVISUALCORTEX_H_DEFINED
