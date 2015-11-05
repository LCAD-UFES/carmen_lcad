/*!@file Channels/SingleSvChannel.H Channel for a single stream of processing 
 in space variant mode.  */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/Channels/SingleSvChannel.H $

#ifndef SINGLESVCHANNEL_H_DEFINED
#define SINGLESVCHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"
#include "Component/ModelParam.H"
#include "SpaceVariant/SVChanLevels.H"

// ######################################################################
//! SingleSvChannel represents a single stream of processing in space variant mode
// templates, bias maps etc should be in space variant coordinates. 
// ######################################################################
class SingleSvChannel : public ChannelBase, public ChannelFacetMap
{
public:
  //! Constructor. See ChannelBase.H
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage
      @param vs The VisualFeature implemented by the channel
      @param spacevariantmodule the module to use for the sv channel*/
  SingleSvChannel(OptionManager& mgr, const std::string& descrName,
                  const std::string& tagName, const VisualFeature vs, const SpaceVariantModule& spacevariantmodule);
  
  //! destructor
  virtual ~SingleSvChannel();
  
  //! Computes the channel's pyramid but doesn't store it
  virtual ImageSet<float> computePyramid(const Image<float>& bwimg,
                                         const rutz::shared_ptr<PyramidCache<float> >& cache);
  //!get output map dimensions
  virtual Dims getMapDims() const;

  //! non-op for this class
  virtual Image<float> centerSurround(const uint cntr, const uint surr) const;

  //! non-op for this class
  virtual void centerSurround(const uint cntr, const uint surr,
                              Image<float>& pos, Image<float>& neg) const;

  //!get the submap name
  virtual std::string getSubmapName(const uint index) const;

  //!get the submap short name
  virtual std::string getSubmapNameShort(const uint index) const;

  //get feature values at a location
  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  //get feature values at many location
  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;

  //! non-op for this class
  virtual LevelSpec getLevelSpec() const;

  //! make the clipping pyramid from the clip mask
  virtual void setClipPyramid(const Image<byte>& clipMask);

  //store a precomputed pyramid
  virtual void storePyramid(const ImageSet<float>& p, const SimTime& t)

  //used by getSubmap()
  virtual Image<float> getRawCSmap(const uint idx) const;

  //get the min pyramid level
  virtual int getMinPyrLevel() const;

  //get the max pyramid level
  virtual int getMaxPyrLevel() const;

  //grab a subchannel (level from the pyramid) splitting pos/neg values and possibly taking abs, square etc
  void computeSubChanSplit(const uint idx, Image<float>& pos, Image<float>& neg) const;

  //grab a subchannel (level from the pyramid) splitting pos/neg values and possibly taking abs, square etc
  Image<float> computeSubChan(const uint idx) const;

protected:
  OModelParam<Dims> itsChanDims;//output dims for the channel
  OModelParam<SVChanLevels> itsLevels;//variance of each level
  NModelParam<bool> itsTakeSquare;//should we take squareroot
  NModelParam<bool> isPolarized;//do we have positive and negative values from a subtraction?
  NModelParam<bool> itsUseSpaceVariantBoundary;//we must have this param to allow RetinaCT

  // get us started (see ModelComponent.H)
  /*! If you overload this, make sure you call SingleChannel::start1()
    at the beginning of your overload */
  virtual void start1();

  virtual uint csToIndex(uint centerlev, uint surroundlev) const;

  virtual indexToCS(const uint idx, uint& centerlev, uint& surroundlev) const;

  //!get the maximum depth of the feature pyramid
  virtual uint maxIndex() const;
  
private:
  nub::ref<SpaceVariantModule>   itsTransform;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !SINGLESVCHANNEL_H_DEFINED
