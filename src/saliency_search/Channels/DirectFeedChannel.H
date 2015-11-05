/*!@file Channels/DirectFeedChannel.H the source for DirectFeedChannel and
MultiDirectFeedChannel */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DirectFeedChannel.H $
// $Id: DirectFeedChannel.H 10746 2009-02-03 07:09:00Z itti $
//

#ifndef DIRECTFEEDCHANNEL_H_DEFINED
#define DIRECTFEEDCHANNEL_H_DEFINED

#include "Channels/ChannelBase.H"
#include "Channels/ComplexChannel.H"
#include "Component/ModelParam.H"
#include "Image/ImageSet.H"
#include "Image/fancynorm.H"
#include "Util/SimTime.H"

//! Direct Feed Channel
class DirectFeedChannel : public ChannelBase
{
public:

  //! Constructor
  /*!@param id if given, becomes part of the name of the channel */
  DirectFeedChannel(OptionManager& mgr, int id = -1);

  //! Destructor
  virtual ~DirectFeedChannel();

  //! Reset SingleChannel
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

  //! Read params from the ParamMap.
  virtual void readFrom(const ParamMap& pmap);

  //! Write params to the ParamMap.
  virtual void writeTo(ParamMap& pmap) const;

  //! Clamp the weighting coefficients to the specified range.
  void clampCoeffs(const double cmin, const double cmax);

  //! Return the sum of the absolute values of the weighting coefficients.
  double absSumCoeffs() const;

  //! set coefficient idx to val
  void setCoeff(const uint idx, const double val);

  //! returns coefficient idx
  double getCoeff(const uint idx) const;

  //! Normalize the weighting coefficients by dividing by div.
  void normalizeCoeffs(const double div);

  //! initialize the coefficients to value val
  void initializeCoeffs(const uint numCoeffs, const double val = 1.0);


  //! Query whether the channel is ready to give yield valid output.
  /*! In some cases (e.g. with motion channels), it may take several input
      frames before the channel is fully initialized. */
  virtual bool outputAvailable() const;

  //! Return the dimensions of the output image.
  virtual Dims getMapDims() const;

  //! Return the number of feature maps represented by this channel.
  virtual uint numSubmaps() const;

  //! Get the feature map at the given index.
  /*! This function is intended to be implemented by subclasses so that it
      recurses into any nested channels, mapping nested submaps into a
      linear sequence of indices. */
  virtual Image<float> getSubmap(const uint index) const;

  //! Get the name/description of the feature map at the given index.
  /*! This function is intended to be implemented by subclasses so that it
      recurses into any nested channels, mapping nested submaps into a
      linear sequence of indices. The output of this function is intended
      only to be human-readable (e.g. for labeling image displays), and
      does not have to follow any particular parseable syntax. */
  virtual std::string getSubmapName(const uint index) const;

  //! not implemented
  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  //! not implemented
  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;

  //! The output will be the size of the feature map level in the pyramid.
  /*! (I.e., the size of the output is given by getMapDims(). */
  virtual Image<float> getOutput();

  //! erase all internal caches
  virtual void killCaches();

  //! Send an already computed pyramid as input to the channel.
  /*! Must be called after each call of input.*/
  void inputPyramid(const ImageSet<float>& pyramid, const SimTime& t);

protected:
  virtual void start1();

  //! the actual input routine
  /*! Only colimg or bwimg ar needed to obtain the dimensions.
    After each call of doinput, inputPyramid must be called.*/
  virtual void doInput(const InputFrame& inframe);

  //! computed the output of the channel
  virtual void computeOutput();

  //! applies the max norm to map depending on itsNormType
  virtual Image<float> applyMaxNorm(Image<float>& map);

  //! the level at which the saliency map lives
  OModelParam<uint> itsMapLevel;

  //! the kind of normalization to be applied
  OModelParam<MaxNormType> itsNormType;

  OModelParam<bool> itsUseOlderVersion;
  OModelParam<float> itsOutputRangeMin;
  OModelParam<float> itsOutputRangeMax;

private:
  Dims itsMapDims;
  ImageSet<float> itsPyr;
  SimTime itsPyrTime;
  SimTime itsInputTime;
  std::vector<double> itsCoeff;
  Image<float> itsOutputCache;
};

// ######################################################################
// #### MultiDirectFeedChannel
// ######################################################################

//! A vector of ImageSets - to be used as input to MultiDirectFeedChannel
typedef std::vector< ImageSet<float> > PyrVec;

//! Multi Direct Feed Channel
class MultiDirectFeedChannel : public ComplexChannel
{
public:
  //! Constructor.
  /*! @param mgr our ModelManager (see ModelManager.H)
    @param num number of sub-channels */
  MultiDirectFeedChannel(OptionManager& mgr, uint num = 1);

  //! Virtual destructor.
  virtual ~MultiDirectFeedChannel();

  //! reset the number of sub-channels
  virtual void setNumChans(uint num);

  //! Send an already computed pyramid as input to the sub-channel num.
  void inputPyramid(uint ChanNum,
                            const ImageSet<float>& pyramid,
                            const SimTime& t);

  //! Send a vector of already computed pyramids as inputs to the channels.
  void inputPyramidVector(const PyrVec& pvec,
                                  const SimTime& t);

  //! returns sub-channel number num
  DirectFeedChannel& directFeed(uint num);

protected:
  //! Combines the outputs of subchannels.
  virtual Image<float> combineOutputs();

  //! must be called with either colimg or bwimg to obtain the dims
  /*! After each call of input, inputPyramidVector or inputPyramid
    must be called.*/
  virtual void doInput(const InputFrame& inframe);

};


#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
