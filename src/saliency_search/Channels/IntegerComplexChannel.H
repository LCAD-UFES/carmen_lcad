/*!@file Channels/IntegerComplexChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerComplexChannel.H $
// $Id: IntegerComplexChannel.H 12820 2010-02-11 05:44:51Z itti $
//

#ifndef CHANNELS_INTEGERCOMPLEXCHANNEL_H_DEFINED
#define CHANNELS_INTEGERCOMPLEXCHANNEL_H_DEFINED

#include "Channels/ChannelFacet.H"
#include "Channels/IntegerChannel.H"
#include "Image/Image.H"
#include "Image/fancynorm.H" // for MaxNormType

// ######################################################################
//! Like ComplexChannel, but avoids floating-point arithmetic
class IntegerComplexChannel : public IntegerChannel, public ChannelFacetMap
{
public:
  /// Refer to a subchannel by one of: its index, its tagname, its address
  /** Only one of the pieces of information will be valid. Inside the
      IntegerComplexChannel functions that use SubchanKey, we will determine
      which one of the keys is the valid one, and use that one to look
      up the subchannel itself. */
  struct SubchanKey
  {
    /// Initialize with an integer index
    SubchanKey(uint i)          : index(i),        tag(0), addr(0) {}

    /// Initialize with a tag name
    SubchanKey(const char* t)   : index(uint(-1)), tag(t), addr(0) {}

    /// Initialize with a channel reference
    SubchanKey(IntegerChannel& c): index(uint(-1)), tag(0), addr(&c) {}

    uint index;
    const char* tag;
    IntegerChannel* addr;
  };

  //! Constructor.
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage
      @param vs The VisualFeature implemented by the channel */
  IntegerComplexChannel(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName,
                        const VisualFeature vs,
                        nub::ref<IntegerMathEngine> eng);

  //! Virtual destructor.
  virtual ~IntegerComplexChannel();

  //! Is the channel homogeneous?
  /*! A homogeneous channel is one whose subchannels (and their subs)
    have identical VisualFeatures, and those are also identical to our
    VisualFeature. Default implementation returns true. */
  virtual bool isHomogeneous() const;

  //! Get the number of subchannels.
  uint numChans() const;

  //! Look up one of the subchannels by its integer index.
  nub::ref<IntegerChannel> subChan(const uint idx) const;

  //! Look up one of the subchannels by its tagname
  nub::ref<IntegerChannel> subChan(const std::string& tagname) const;

  //! Look up one of the subchannels by a submap number
  /*!@param oldIdx - the submap index (in the IntegerComplexChannel);
     @param newIdx - return value: the number of the submap within
     the subchannel*/
  nub::ref<IntegerChannel> subChanFromSubmapNum(const uint oldIdx,
                                                uint& newIdx) const;

  //! Add a subchannel
  /*! @param ch The channel to be added
      @param name If not null, the channel's tagName will be set to
      this value

      The channel's tagName is used for associative lookup.

      This is just a wrapper to ModelComponent::addSubComponent(),
      with some checking that the subcomponent indeed is a
      IntegerChannel derivative, and some adjustment of our
      homogeneity (if the newly added subchannel has a different
      VisualFeature from ours, then be become heterogeneous). */
  void addSubChan(nub::ref<IntegerChannel> ch, const char* name = 0);

  /// Remove a subchannel
  void removeSubChan(nub::ref<IntegerChannel> ch);

  /// Remove all of our subchannels
  void removeAllSubChans();

  /// Check if we have a channel of given tagname
  bool hasSubChan(const char* name) const;

  /// Install a visitor which will be applied to all newly-added subchannels.
  void setSubchanVisitor(rutz::shared_ptr<ChannelVisitor> v);

  //! Set the total weight allocated to the specified subchannel.
  /*! This weight will be divided amongst any sub-subchannels managed
      by the specifed subchannel. So, for example, if the total weight
      is set to 2.0, and the subchannel has four sub-subchannels, then
      each sub-subchannel will get an effective weight of 0.5. The
      initial weight setting is 1.0. But, If you are handling b/w
      images, for instance, you may set the weight for the color
      channel to 0.0 and save computations. */
  void setSubchanTotalWeight(SubchanKey key, const double wt);

  //! Return the total weight allocated to the specified subchannel.
  double getSubchanTotalWeight(SubchanKey key) const;

  /// Whether to sort subchannels by their number of submaps.
  /** If true, this may help in parallel-processing situations where
      we want to start the most CPU-intensive channels first.

      Must be called before start().
  */
  void sortChannelsByNumSubmaps(bool dosort);

  virtual void readFrom(const ParamMap& pmap);

  virtual void writeTo(ParamMap& pmap) const;

  virtual bool outputAvailable() const;

  virtual Dims getMapDims() const;

  virtual uint numSubmaps() const;

  virtual Image<int> getSubmapInt(const uint index) const;

  virtual Image<float> getSubmap(const uint index) const
  { return Image<float>(getSubmapInt(index)); }

  virtual Image<int> getRawCSmapInt(const uint idx) const;

  virtual Image<float> getRawCSmap(const uint idx) const
  { return Image<float>(getRawCSmap(idx)); }

  virtual std::string getSubmapName(const uint index) const;

  virtual std::string getSubmapNameShort(const uint index) const;


  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;

  virtual Image<int> getOutputInt();

  //! Form output by combining output of subchannels.
  /*! This function just provides a caching wrapper around
      combineOutputsInt(), where the real work is
      done. IntegerComplexChannel provides a default implementation of
      combineOutputsInt() that sums the outputs of the subchannels and
      then optionally maxNormalize()'s the result. */
  virtual Image<float> getOutput()
  { return Image<float>(getOutputInt()); }

  //! Save our various maps using a FrameOstream
  /*! By default, we save our output map (if itsSaveOutputMap is true)
      and propagate the call to our subchannels. Our output
      (conspicuity) map will be prefixed by "CO<tagname>-" */
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

  //! Clears the output cache.
  virtual void killCaches();

protected:
  OModelParam<MaxNormType> itsNormType; //!< Normalization; see fancynorm.H
  OModelParam<bool> itsSaveOutputMap;   //!< Save our output map?
  OModelParam<bool> itsUseOlderVersion; //!< use old version? see Visualcortex.H

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

  //! Combines the outputs of subchannels.
  /*! The default implementation just sums the outputs of the
      subchannels and then maxNormalize()'s if requested by
      itsNormType. */
  virtual Image<int> combineOutputsInt();

  //! get started
  virtual void start1();

  //! get started
  virtual void start2();

  //! stop
  virtual void stop2();

private:
  IntegerComplexChannel(const IntegerComplexChannel&); // not allowed
  IntegerComplexChannel& operator=(const IntegerComplexChannel&); // not allowed

  void lookupSubmap(const uint idx, uint& subchan, uint& subidx) const;

  struct Impl;
  Impl* rep;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERCOMPLEXCHANNEL_H_DEFINED
