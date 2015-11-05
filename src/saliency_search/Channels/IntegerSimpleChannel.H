/*!@file Channels/IntegerSimpleChannel.H IntegerSimpleChannel is like SingleChannel, but avoids floating-point arithmetic */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerSimpleChannel.H $
// $Id: IntegerSimpleChannel.H 10983 2009-03-05 07:19:14Z itti $
//

#ifndef CHANNELS_INTEGERSIMPLECHANNEL_H_DEFINED
#define CHANNELS_INTEGERSIMPLECHANNEL_H_DEFINED

#include "Channels/ChannelFacet.H"
#include "Channels/IntegerChannel.H"
#include "Component/ModelParam.H"
#include "Image/ImageSet.H"
#include "Image/LevelSpec.H"
#include "Image/PyrBuilder.H"
#include "Image/fancynorm.H" // for MaxNormType
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"
#include <deque>

// ######################################################################
//! IntegerSimpleChannel is like SingleChannel, but avoids floating-point arithmetic
class IntegerSimpleChannel : public IntegerChannel, public ChannelFacetMap
{
public:
  //! Constructor. See ChannelBase.H
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage
      @param vs The VisualFeature implemented by the channel
      @param pyr The type of pyramid that should be used. */
  IntegerSimpleChannel(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName, const VisualFeature vs,
                       rutz::shared_ptr<PyrBuilder<int> > pyr,
                       nub::ref<IntegerMathEngine> eng);

  //! destructor
  virtual ~IntegerSimpleChannel();

  //! Reset IntegerSimpleChannel
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

  virtual void readFrom(const ParamMap& pmap);

  virtual void writeTo(ParamMap& pmap) const;

  virtual bool outputAvailable() const;

  virtual Dims getMapDims() const;

  virtual uint numSubmaps() const;

  virtual Image<int> getSubmapInt(const uint index) const;

  virtual Image<float> getSubmap(const uint index) const
  { return Image<float>(getSubmapInt(index)); }

  virtual std::string getSubmapName(const uint index) const;

  virtual std::string getSubmapNameShort(const uint index) const;

  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;

  virtual Image<int> getOutputInt();

  //! This is just a caching wrapper around combineSubMaps()
  virtual Image<float> getOutput()
  { return Image<float>(getOutputInt()); }

  //! Save our various maps using an FrameOstream
  /*! Depending on our ModelParam settings, we can save raw pyramid
    levels (with name prefix "SR<tagname>-<level>-"), center-surround
    feature maps ("SF<tagname>-<ctr>-<surr>-"), and output map
    ("SO<tagname>-") */
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

  //! get our the LevelSpec
  LevelSpec getLevelSpec() const;

  //! IntegerSimpleChannel implements this to clear its cached output.
  virtual void killCaches();

  //! get raw CS map; part of getSubmap()
  virtual Image<float> getRawCSmap(const uint idx) const
  { return Image<float>(getRawCSmapInt(idx)); }

  //! get raw CS map; part of getSubmap()
  virtual Image<int> getRawCSmapInt(const uint idx) const;

  void setTakeAbs(bool val) { itsTakeAbs.setVal(val); }

  void setNormalizeOutput(bool val) { itsNormalizeOutput.setVal(val); }

  const ImageSet<int>& intgPyramid() { return itsPyr; }

  int getMinPyrLevel() const
  {
    // If we want to save our raw pyramid maps, then let's compute the
    // pyramid in full; otherwise, we can skip the levels below our
    // LevelSpec's levMin():
    return itsSaveRawMaps.getVal() ? 0 : itsLevelSpec.getVal().levMin();
  }

  int getMaxPyrLevel() const { return itsLevelSpec.getVal().maxDepth(); }

  /// Install a new PyrBuilder object
  void setPyrBuilder(rutz::shared_ptr<PyrBuilder<int> > pbuild);

protected:

  virtual void doInputInt(const IntegerInput& inp,
                          const SimTime& t,
                          PyramidCache<int>* cache,
                          const Image<byte>& clipMask);

  NModelParam<bool> itsTakeAbs;
  NModelParam<bool> itsNormalizeOutput;
  NModelParam<bool> itsScaleNoiseToMax;
  NModelParam<int> itsLowThresh;
  NModelParam<bool> itsRectifyPyramid;
  OModelParam<bool> itsUseRandom;
  OModelParam<LevelSpec> itsLevelSpec;
  OModelParam<MaxNormType> itsNormType;
  OModelParam<int> itsQlen;
  OModelParam<bool> itsUseOlderVersion;

  //! Save our raw pyramid levels?
  OModelParam<bool> itsSaveRawMaps;

  //! Save our center-surround feature maps?
  OModelParam<bool> itsSaveFeatureMaps;

  //! Save our output map?
  OModelParam<bool> itsSaveOutputMap;

  //! Set a fixed range of values for the raw output
  /*! By default, the range is set to [MAXNORMMIN .. MAXNORMMAX] at
    construction. If non-zero values are provided here, in getOutput()
    we will cumulate our various submaps, apply the provided range,
    apply spatial competition for salience, apply our total weight,
    and return the result. If a range [0.0 .. 0.0] is provided here,
    only spatial competition and weights will be applied, but the
    first step of applying the range will be skipped. This must be
    called before start(). */
  OModelParam<int> itsOutputRangeMin;
  OModelParam<int> itsOutputRangeMax;

  /// get us started (see ModelComponent.H)
  /*! If you overload this, make sure you call IntegerSimpleChannel::start1()
    at the beginning of your overload */
  virtual void start1();

  /// get us stopped (see ModelComponent.H)
  /*! If you overload this, make sure you call IntegerSimpleChannel::start1()
    at the beginning of your overload */
  virtual void stop2();

  /// shortcut to the csToIndex function of itsLevelSpec:
  /*! If you overload this, make sure you call IntegerSimpleChannel::stop1()
    at the beginning of your overload */
  uint csToIndex(uint centerlev, uint surroundlev) const;

private:
  IntegerSimpleChannel(const IntegerSimpleChannel&); // not allowed
  IntegerSimpleChannel& operator=(const IntegerSimpleChannel&); // not allowed

  ImageSet<int>                  itsPyr;
  SimTime                        itsT;
  Image<int>                     itsOutputCache;
  Image<int>*                    itsSubmapCache;
  rutz::shared_ptr<PyrBuilder<int> >  itsPyrBuilder;
  ImageSet<int>                  itsClipPyr;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERSIMPLECHANNEL_H_DEFINED
