/*!@file Channels/ChannelBase.H The base class for all channel classes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelBase.H $
// $Id: ChannelBase.H 12820 2010-02-11 05:44:51Z itti $
//

#ifndef CHANNELBASE_H_DEFINED
#define CHANNELBASE_H_DEFINED

#include "Channels/InputFrame.H"
#include "Channels/VisualFeatures.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/PixelsTypes.H"
#include "Util/Assert.H"

#include <vector>

class ChannelBase;
class ChannelVisitor;
class Dims;
class FrameOstream;
class ParamMap;
template <class T> class Point2D;
class Rectangle;
class SimTime;
template <class T> class PixRGB;

// ######################################################################
//! ChannelBase represents the various computational modules in VisualCortex.
class ChannelBase : virtual public ModelComponent
{
public:
  //! Default constructor. See ModelComponent.H
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage
      @param vs The VisualFeature implemented by the channel */
  ChannelBase(OptionManager& mgr, const std::string& descrName,
              const std::string& tagName, const VisualFeature vs);

  //! Virtual destructor ensures proper destruction of derived classes.
  virtual ~ChannelBase();

  //! Kill our caches
  virtual void reset1();

  /// Default version calls visitChannelBase() on the ChannelVisitor.
  virtual void accept(ChannelVisitor& v);

  // ######################################################################
  /*! @name VisualFeature functions */
  //@{

  //! Get the visual feature
  VisualFeature visualFeature() const;

  //! Is the channel homogeneous?
  /*! A homogeneous channel is one whose subchannels (and their subs)
      have identical VisualFeatures, and those are also identical to
      our VisualFeature. Default implementation returns true. */
  virtual bool isHomogeneous() const;
  //@}

  // ######################################################################
  /*! @name ParamMap functions

      Subclasses should be sure to call ChannelBase's version of these
      functions within their own implementations, so that
      ChannelBase's parameters (such as its weights) are properly
      handled.

      This interface is different from that provided by
      ModelComponent, as here we only save a subset of all of the
      Channel's parameters, that are useful, e.g., for object
      recognition or feature-based attention biasing. Some of the
      values saved here may not even be ModelParam parameters. */
  //@{

  //! Read params from the ParamMap.
  virtual void readFrom(const ParamMap& pmap);

  //! Write params to the ParamMap.
  virtual void writeTo(ParamMap& pmap) const;

  //@}

  // ######################################################################
  /*! @name Input functions

      See documentation on InputFrame for multiple ways in which an
      InputFrame can be constructed. */
  //@{

  //! Send a full-blown InputFrame to the channel
  void input(const InputFrame& inframe);

  //@}

  // ######################################################################
  /*! @name Access functions */
  //@{

  //! Query whether the channel is ready to give yield valid output.
  /*! In some cases (e.g. with motion channels), it may take several
      input frames before the channel is fully initialized. */
  virtual bool outputAvailable() const = 0;

  //! Return the dimensions of the output image.
  virtual Dims getMapDims() const = 0;

  //! Check if we've received any input images yet.
  bool hasInput() const;

  //! Return the dimensions of the most recent input image.
  /*! Will LFATAL() if hasInput() is false. */
  Dims getInputDims() const;

  //! Return the number of feature maps represented by this channel.
  virtual uint numSubmaps() const = 0;

  //! Get the feature map at the given index.
  /*! This function is intended to be implemented by subclasses so
      that it recurses into any nested channels, mapping nested
      submaps into a linear sequence of indices. */
  virtual Image<float> getSubmap(const uint index) const = 0;

  //! get raw CS map; part of getSubmap()
  virtual Image<float> getRawCSmap(const uint idx) const {LFATAL("Not implemented yet."); return Image<float>();}

  //! Get the name/description of the feature map at the given index.
  /*! This function is intended to be implemented by subclasses so
      that it recurses into any nested channels, mapping nested
      submaps into a linear sequence of indices. The output of this
      function is intended only to be human-readable (e.g. for
      labeling image displays), and does not have to follow any
      particular parseable syntax. */
  virtual std::string getSubmapName(const uint index) const = 0;

  //! Get the name/description of the feature map at the given index, short version
  virtual std::string getSubmapNameShort(const uint index) const = 0;


  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const = 0;

  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const = 0;

  //! The output will be the size of the feature map level in the pyramid.
  /*! (I.e., the size of the output is given by getMapDims(). */
  virtual Image<float> getOutput() = 0;

  //! Save our various maps using a FrameOstream
  /*! The default implementation is a no-op; subclasses can override
      to do something interesting */
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

  //! Subclasses override this to clear any caches they may maintain.
  /*! <b>Subclasses must be sure to explicitly call their base class's
      version of this function!</b>. This function will be called by
      ChannelBase when a new input image is received, or when new
      parameters are read from a ParamMap, etc. Subclasses may also
      call this function when they know that caches may have become
      invalid. Calling this explicitly after you have obtained the
      output from a channel may save you some memory. */
  virtual void killCaches();

  //@}

protected:
  //! Subclasses override this implementation of the input() function.
  /*! This will be called by one of the input() methods. For
      efficiency reasons (e.g. network bandwidth in parallel code), it
      is possible that the color or luminance input may be null,
      although the code should be wired up properly so that given
      subclasses do not receive a null input that they in fact
      need. Subclasses should be sure to ASSERT that any used inputs
      are non-null. */
  virtual void doInput(const InputFrame& inframe) = 0;

  // This is used only by SingleOpponentChannel::singleOpponentInput()
  void setInputDims(const Dims& dims);

private:
  ChannelBase(const ChannelBase&); // not allowed
  ChannelBase& operator=(const ChannelBase&); // not allowed

  NModelParam<VisualFeature> itsVisFeature;

  Dims itsInputDims;
};


/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !CHANNELBASE_H_DEFINED
