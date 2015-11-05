/*!@file Channels/SingleChannel.H Channel for a single stream of processing. */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SingleChannel.H $
// $Id: SingleChannel.H 14632 2011-03-23 20:08:44Z dberg $
//

#ifndef SINGLECHANNEL_H_DEFINED
#define SINGLECHANNEL_H_DEFINED

#include "Channels/ChannelBase.H"
#include "Channels/ChannelFacet.H"
#include "Channels/InputHandler.H"
#include "Channels/SubmapAlgorithm.H"
#include "Component/ModelParam.H"
#include "Image/ImageSet.H"
#include "Image/LevelSpec.H"
#include "Image/PyrBuilder.H"
#include "Image/fancynorm.H" // for MaxNormType
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"
#include <deque>

// ######################################################################
//! SingleChannel represents a single stream of processing.
/*! The processing is implemented by an internal PyrBuilder object of some
    kind. */
class SingleChannel : public ChannelBase, public ChannelFacetMap
{
public:
  //! Constructor. See ChannelBase.H
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage
      @param vs The VisualFeature implemented by the channel
      @param pyr The type of pyramid that should be used. */
  SingleChannel(OptionManager& mgr, const std::string& descrName,
                const std::string& tagName, const VisualFeature vs,
                rutz::shared_ptr<PyrBuilder<float> > pyr);

  //! destructor
  virtual ~SingleChannel();

  //! Reset SingleChannel
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

  /// Calls visitSingleChannel() on the ChannelVisitor.
  virtual void accept(ChannelVisitor& v);

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Send an already computed pyramid as input to the channel.
  void inputPyramid(const ImageSet<float>& pyramid, const SimTime& t,
                    const Image<byte>& clipMask = Image<byte>());

  //! Computes the channel's pyramid but doesn't store it
  virtual ImageSet<float> computePyramid(const Image<float>& bwimg,
                                         const rutz::shared_ptr<PyramidCache<float> >& cache);

  virtual void readFrom(const ParamMap& pmap);

  virtual void writeTo(ParamMap& pmap) const;

  //! Set the template that will be applied to the submap
  void setTempl(const uint cntr, const uint surr, Image<float> &templ);

  //! Set the mask that will be applied to the submap
  void setBiasMask(Image<float> &biasMask);

  //! Set the mask that will be applied to the submap
  Image<float> getBiasMask() const;

  //! Get the template that is beeing applied to the submap
  Image<float> getTempl(const uint cntr, const uint surr) const;

  virtual bool outputAvailable() const;

  //! do we have any input pyramids in our queue?
  bool hasPyramid() const;

  //! have we already computed and cached the current output?
  bool hasOutputCache() const;

  virtual Dims getMapDims() const;

  //! Get the image from the given level of the internal pyramid
  virtual const Image<float>& getImage(const uint lev) const;

  //! Get the center/surround image for the given levels
  virtual Image<float> centerSurround(const uint cntr, const uint surr) const;

  //! Get the center/surround, split into positive and negative parts
  virtual void centerSurround(const uint cntr, const uint surr,
                              Image<float>& pos, Image<float>& neg) const;

  virtual uint numSubmaps() const;

  //! This is just a caching wrapper around computeSubmap()
  virtual Image<float> getSubmap(const uint index) const;

  virtual std::string getSubmapName(const uint index) const;

  virtual std::string getSubmapNameShort(const uint index) const;

  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;

  //! save basic stats for single channel
  void saveStats(const Image<float> img, const short idx);

  //! This is just a caching wrapper around combineSubMaps()
  virtual Image<float> getOutput();

  //! Combine all feature maps into a single output map
  virtual Image<float> combineSubMaps();

  //! Save our various maps using an FrameOstream
  /*! Depending on our ModelParam settings, we can save raw pyramid
    levels (with name prefix "SR<tagname>-<level>-"), center-surround
    feature maps ("SF<tagname>-<ctr>-<surr>-"), and output map
    ("SO<tagname>-") */
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

  //! Get the number of pyramids in our queue
  size_t numPyramids() const;

  //! Access to the underlying pyramid is provided for completeness.
  /*! However, it probably shouldn't need to be used except for testing,
      debugging, etc. Instead, the Channel interface should be used or
      extended to fill the specific need.
      @param index The index in the queue (0 = newest, qlen-1 = oldest). */
  const ImageSet<float>& pyramid(const uint index) const;

  //! Access to the underlying pyramid times is provided for completeness.
  /*! However, it probably shouldn't need to be used except for testing,
      debugging, etc. Instead, the Channel interface should be used or
      extended to fill the specific need.
      @param index The index in the queue (0 = newest, qlen-1 = oldest). */
  SimTime pyramidTime(const uint index) const;

  //! Access to the underlying clipPyramid
  const ImageSet<float>& clipPyramid() const;

  //! get our the LevelSpec
  virtual LevelSpec getLevelSpec() const;

  //! get our NormType:
  int getNormType() const;

  //! SingleChannel implements this to clear its cached output.
  virtual void killCaches();

  //! a time-stamped pyramid
  struct TPyr
  {
    TPyr(const ImageSet<float>& pyr_, const SimTime t_)
      : pyr(pyr_), t(t_) {}
    ImageSet<float> pyr; //!< the pyramid
    SimTime t;           //!< the timestamp
  };

  //! Store m as the output map
  void storeOutputCache(const Image<float>& m);

  //! Stores p as the channel's pyramid at time t
  virtual void storePyramid(const ImageSet<float>& p, const SimTime& t);

  //! Store p as the channel's clip pyramid
  void storeClipPyramid(const ImageSet<float>& p);

  //! make the clipping pyramid from the clip mask
  virtual void setClipPyramid(const Image<byte>& clipMask);

  //! Store p as the channel's submap cache
  void storeSubmapCache(const ImageSet<float>& p);

  //! Install an input handler
  void setInputHandler(rutz::shared_ptr<InputHandler> h);

  //! Get a cloned copy of our input handler
  rutz::shared_ptr<InputHandler> cloneInputHandler() const;

  //! get raw CS map; part of getSubmap()
  virtual Image<float> getRawCSmap(const uint idx) const;

  //! rescale and post-process raw CS map; part of default getSubMap()
  Image<float> postProcessMap(const Image<float>& smap,
                              const uint idx) const;

  //! Install a new submap algorithm
  void setSubmapAlgorithm(nub::ref<SubmapAlgorithm> algo);

  void setComputeFullPyramid(bool v)
  { itsComputeFullPyramid.setVal(v); }

  virtual int getMinPyrLevel() const
  {
    // If we want to save our raw pyramid maps, or otherwise need a
    // full pyramid, then let's compute the pyramid in full starting
    // from level 0; otherwise, we can skip the levels below our
    // LevelSpec's levMin():
    return (itsSaveRawMaps.getVal() || itsComputeFullPyramid.getVal() ||
            itsComputeFullPyramidForGist.getVal() )  ? 0 : itsLevelSpec.getVal().levMin();
  }

  virtual int getMaxPyrLevel() const { return itsLevelSpec.getVal().maxDepth(); }

protected:
  //! SingleChannel requires only luminance input
  virtual void doInput(const InputFrame& inframe);

  //! Change to use a different pyramid object.
  void setPyramid(rutz::shared_ptr<PyrBuilder<float> > pyr);

  //! Mutable access to underlying pyramid
  ImageSet<float>& pyrMut(const uint index);

  NModelParam<bool> itsTakeAbs;
  NModelParam<bool> itsNormalizeOutput;
  NModelParam<bool> itsScaleNoiseToMax;
  NModelParam<float> itsLowThresh;
  NModelParam<bool> itsRectifyPyramid;
  NModelParam<bool> itsComputeFullPyramid;

  OModelParam<bool> itsUseRandom;
  OModelParam<bool> itsUseSplitCS;
  OModelParam<LevelSpec> itsLevelSpec;
  OModelParam<MaxNormType> itsNormType;
  OModelParam<int> itsQlen;
  OModelParam<bool> itsUseOlderVersion;

  //! Time decay for the contribution of differences in the pyramid queue.
  /*! This decay will yield a factor fac = exp( (t2 - t) * decay)
    applied to the difference image between t (current time, in
    seconds) and t2 (time of previous image, in seconds): */
  OModelParam<double> itsTimeDecay;

  //! Save our raw pyramid levels?
  OModelParam<bool> itsSaveRawMaps;

  //! save our raw pyramid levels for the gist computation?
  OModelParam<bool> itsComputeFullPyramidForGist;

  //! Save our center-surround feature maps?
  OModelParam<bool> itsSaveFeatureMaps;

  //! Save our output map?
  OModelParam<bool> itsSaveOutputMap;

  //! Type name for our SubmapAlgorithm
  OModelParam<std::string> itsSubmapAlgoType;

  //! Save basic single channel stats after combineSubMaps
  OModelParam<bool> itsGetSingleChannelStats;

  //! If saving stats, should we put each feature in its own file?
  OModelParam<bool> itsSaveStatsPerChannel;

  //! Should we save frequency information per channel?
  OModelParam<bool> itsSaveStatsPerChannelFreq;

  //! File name for single channel stats after combineSubMaps
  OModelParam<std::string> itsGetSingleChannelStatsFile;

  //! Tag name for single channel stats after combineSubMaps
  OModelParam<std::string> itsGetSingleChannelStatsTag;

  //! Set a fixed range of values for the raw output
  /*! By default, the range is set to [MAXNORMMIN .. MAXNORMMAX] at
    construction. If non-zero values are provided here, in getOutput()
    we will cumulate our various submaps, apply the provided range,
    apply spatial competition for salience, apply our total weight,
    and return the result. If a range [0.0 .. 0.0] is provided here,
    only spatial competition and weights will be applied, but the
    first step of applying the range will be skipped. This must be
    called before start(). */
  OModelParam<float> itsOutputRangeMin;
  OModelParam<float> itsOutputRangeMax;

  // get us started (see ModelComponent.H)
  /*! If you overload this, make sure you call SingleChannel::start1()
    at the beginning of your overload */
  virtual void start1();

  // get us stopped (see ModelComponent.H)
  /*! If you overload this, make sure you call SingleChannel::start1()
    at the beginning of your overload */
  virtual void stop2();

  // shortcut to the csToIndex function of itsLevelSpec:
  virtual uint csToIndex(uint centerlev, uint surroundlev) const;

  // shortcut to the indexToCS function of itsLevelSpec:
  virtual void indexToCS(const uint index, uint& centerlev, uint& surroundlev) const;

  // shortcut to maxIndex function in itsLevelSpec
  virtual uint maxIndex() const;

private:
  SingleChannel(const SingleChannel&); // not allowed
  SingleChannel& operator=(const SingleChannel&); // not allowed

  std::deque<TPyr>               itsPq; // temporal queue of pyramids
  Image<float>                   itsBiasMask;   // to bias the channel based on a mask

  uint                           itsFrameIdx; // for logging purposes
  Image<float>                   itsOutputCache;
  Image<float>*                  itsSubmapCache;
  ImageSet<float>                itsTempl; // the templates at each submap level
  rutz::shared_ptr<PyrBuilder<float> >  itsPyrBuilder;
  ImageSet<float>                itsClipPyr;
  rutz::shared_ptr<InputHandler>        itsInputHandler;
  nub::ref<SubmapAlgorithm>      itsSubmapAlgo;

  friend class OrientationChannel; // this one too
  friend class RGBConvolveChannel; // this one messes with itsPyrBuilder
};


/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !SINGLECHANNEL_H_DEFINED
