/*!@file Neuro/NeuroSimEvents.H SimEvent derivatives for neuro modules */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/NeuroSimEvents.H $
// $Id: NeuroSimEvents.H 14762 2011-05-03 01:13:16Z siagian $
//

#ifndef NEURO_NEUROSIMEVENTS_H_DEFINED
#define NEURO_NEUROSIMEVENTS_H_DEFINED

#include "Channels/InputFrame.H"
#include "Neuro/WTAwinner.H"
#include "Neuro/SaccadeBodyPart.H"
#include "Neuro/SaccadeController.H"
#include "Psycho/EyeTrace.H"
#include "Simulation/SimEvent.H"
#include "Simulation/SimReq.H"
#include "Util/TransientStatus.H"
#include "Channels/ChannelVisitor.H"

class SpaceVariantTransform;
class EyeData;
class HandData;
class ParamMap;
class RawVisualCortex;
class IntegerRawVisualCortex;
class EnvVisualCortexFloat;
class ChannelMaps;
class VisualCortexEyeMvt;

// ######################################################################
//! The WinnerTakeAll selected a winner
class SimEventWTAwinner : public SimEvent {
public:
  //! Constuctor
  SimEventWTAwinner(SimModule* src, const WTAwinner w, const uint shiftnum);

  //! Destructor
  virtual ~SimEventWTAwinner();

  //! Get a description for printing out
  /*! Shows regular SimEvent info, plus winner coordinates. */
  virtual std::string toString() const;

  //! Get the winner
  const WTAwinner& winner() const;

  //! Get the attention shift number (0-based)
  uint shiftNum() const;

private:
  const WTAwinner itsWinner;
  const uint itsShiftNum;
};

// ######################################################################
//! The TargetChecker hit a one or more targets
class SimEventTargetsHit : public SimEvent {
public:
  //! Constuctor
  SimEventTargetsHit(SimModule* src, const int numhit);

  //! Destructor
  virtual ~SimEventTargetsHit();

  //! Get a description for printing out
  /*! Shows regular SimEvent info, plus number of targets hit. */
  virtual std::string toString() const;

  //! Get the number of targets hit this time
  int numHits() const;

private:
  const int itsNumHits;
};

// ######################################################################
//! A new image is available from the retina
class SimEventRetinaImage : public SimEvent {
public:
  //! Constuctor
  SimEventRetinaImage(SimModule* src, const InputFrame& ifr,
                      const Rectangle& rawinprect,
                      const Point2D<int> offset);

  //! Constuctor with tranform
  SimEventRetinaImage(SimModule* src, const InputFrame& ifr,
                      const Rectangle& rawinprect,
                      const Point2D<int> offset, 
                      rutz::shared_ptr<SpaceVariantTransform> retinal_transform,
                      rutz::shared_ptr<SpaceVariantTransform> map_transform);

  //! Destructor
  virtual ~SimEventRetinaImage();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the frame
  const InputFrame& frame() const;

  //! Get the raw input rectangle
  const Rectangle& rawInputRectangle() const;

  //! Translate a raw eye position from stimulus coordinates to retinal
  virtual Point2D<int> rawToRetinal(const Point2D<int>& rawpos) const;

  //! Translate a retinal eye position back into raw stimulus coordinates
  virtual Point2D<int> retinalToRaw(const Point2D<int>& retpos) const;

  //! get the raw input dims
  virtual const Dims& getRawInputDims() const;

  //! Get the coordinates of the center of the image
  virtual Point2D<int> center() const;

  //! Get the retinal/raw offset
  /*! Use with caution! Prefer using rawToRetinal() and retinalToRaw() instead. */
  const Point2D<int>& offset() const;

  //! get retinal transform
  rutz::shared_ptr<SpaceVariantTransform> getRetTransform() const;

  //! get map transform
  rutz::shared_ptr<SpaceVariantTransform> getMapTransform() const;

private:
  const InputFrame      itsFrame;
  const Rectangle       itsRawInputRectangle;
  const Point2D<int>    itsOffset;
  rutz::shared_ptr<SpaceVariantTransform> itsRetTransform;
  rutz::shared_ptr<SpaceVariantTransform> itsMapTransform;
};

// ######################################################################
//! A new output is available from the VisualCortex
class SimEventVisualCortexOutput : public SimEvent {
public:
  //! Constuctor
  SimEventVisualCortexOutput(SimModule* src, const Image<float>& vcout);

  //! Destructor
  virtual ~SimEventVisualCortexOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the VisualCortex output
  /*! A factor of 1.0 will return the raw, unnormalized map. A factor
    of 0.0F will normalize the map to a range of [0..255]. Any other
    factor will be used to multiply the map values. */
  const Image<float> vco(const float factor = 1.0F) const;

private:
  const Image<float> itsMap;
};

// ######################################################################
//! A new output is available from the SaliencyMap
class SimEventSaliencyMapOutput : public SimEvent {
public:
  //! Constuctor
  SimEventSaliencyMapOutput(SimModule* src, const Image<float>& smout, const int maplevel);

  //! Destructor
  virtual ~SimEventSaliencyMapOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the SaliencyMap output
  /*! A factor of 1.0 will return the raw, unnormalized map. A factor
    of 0.0F will normalize the map to a range of [0..255]. Any other
    factor will be used to multiply the map values. */
  const Image<float> sm(const float factor = 1.0F) const;

  //! Transform coordinates from saliency map to original image
  Point2D<int> smToOrig(const Point2D<int>& p) const;

  //! Transform coordinates from original image to saliency map
  Point2D<int> origToSm(const Point2D<int>& p) const;

private:
  const Image<float> itsMap;
  const int itsMapLevel;
};

// // ######################################################################
// //! A new output is available from the MT module
// class SimEventMTfeatureMapOutput : public SimEvent {
// public:
//   //! Constuctor
//   SimEventMTfeatureMapOutput
//   (SimModule* src, const std::vector<Image<float> > mtFeat);

//   //! Destructor
//   virtual ~SimEventMTfeatureMapOutput();

//   //! Get a description for printing out
//   virtual std::string toString() const;

//   //! Get the MT feature Map output
//   const std::vector<Image<float> > mtFeatures() const;

// private:
//   const std::vector<Image<float> > itsMTfeatures;
// };

// ######################################################################
//! A new output is available from the GistEstimator
class SimEventGistOutput : public SimEvent {
public:
  //! Constuctor
  SimEventGistOutput(SimModule* src, const Image<float>& gout);

  //! Destructor
  virtual ~SimEventGistOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the gist feature vector
  const Image<float> gv() const;

private:
  const Image<float> itsGistVector;
};

// #####################################################################
//! A new output is available from the TaskRelevanceMap
class SimEventTaskRelevanceMapOutput : public SimEvent {
public:
  //! Constuctor
  SimEventTaskRelevanceMapOutput(SimModule* src, const Image<float>& smout);

  //! Destructor
  virtual ~SimEventTaskRelevanceMapOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the TaskRelevanceMap output
  /*! A factor of 1.0 will return the raw, unnormalized map. A factor
    of 0.0F will normalize the map to a range of [0..255]. Any other
    factor will be used to multiply the map values. */
  const Image<float> trm(const float factor = 1.0F) const;

private:
  const Image<float> itsMap;
};

// ######################################################################
//! A new output(s) is available from the AttentionGuidanceMap
class SimEventAttentionGuidanceMapOutput : public SimEvent {
public:
  //! Constuctor for a single map
  SimEventAttentionGuidanceMapOutput(SimModule* src, const Image<float>& agmout);

  //!constructor for when we have more than 1 map
  SimEventAttentionGuidanceMapOutput(SimModule* src, const ImageSet<float>& agmout);
  
  //! Destructor
  virtual ~SimEventAttentionGuidanceMapOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! get the number of maps
  uint numMaps() const;

  //! Get the AttentionGuidanceMap output
  /*! A factor of 1.0 will return the raw, unnormalized map. A factor
    of 0.0F will normalize the map to a range of [0..255]. Any other
    factor will be used to multiply the map values. */
  const Image<float> agm(const float factor = 1.0F, const uint pos = 0) const;

  //! Get all AttentionGuidanceMap outputs
  /*! A factor of 1.0 will return the raw, unnormalized map. A factor
    of 0.0F will normalize the map to a range of [0..255]. Any other
    factor will be used to multiply the map values. */
  const ImageSet<float> allAgm(const float factor = 1.0F) const;
  
private:
  ImageSet<float> itsMap;
};

// ######################################################################
//! A new output is available from the AttentionGate
class SimEventAttentionGateOutput : public SimEvent {
public:
  //! Constuctor
  SimEventAttentionGateOutput(SimModule* src,
                              const Image<float>& agmout,
                              const Image<float>& lamout,
                              const Image<float>& camout,
                              const unsigned int lamframe);

  //! Destructor
  virtual ~SimEventAttentionGateOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Attention gate map
  const Image<float> ag(const float factor = 1.0F) const;

  //! What finally got through in the last attention map
  const Image<float> lam(const float factor = 1.0F) const;

  //! candidate for what will get through in the current attention map
  const Image<float> cam(const float factor = 1.0F) const;

  //! What was the frame number on our last attention gate;
  const unsigned int lamFrame() const;
private:
  const Image<float> itsMap;
  const Image<float> itsLastAttMap;
  const Image<float> itsCurrAttMap;
  const unsigned int itsLastFrame;
};

// ######################################################################
struct SimEventAttentionGateStageTwoObjects
{
  int                              n;
  Image<int>                       segments;
  Image<float>                     fdistance;
  std::vector<int>                 x;
  std::vector<int>                 y;
  std::vector<int>                 id;
  std::vector<float>               val;
  std::vector<std::vector<float> > features;
};

class SimEventAttentionGateStageTwoSegments : public SimEvent {
public:
  //! Constuctor
  SimEventAttentionGateStageTwoSegments(SimModule* src,
                               const Image<bool>& candidates,
                               const SimEventAttentionGateStageTwoObjects& obj,
                               const int          segnum);

  //! Destructor
  ~SimEventAttentionGateStageTwoSegments();

  //! Which attention gate regions are open from stage one
  const Image<bool> candidates() const;

  //! What are the attention gate regions
  const SimEventAttentionGateStageTwoObjects obj() const;

  //! how many segments were found
  const int segnum() const;

private:
  const Image<bool>                          itsCandidates;
  const SimEventAttentionGateStageTwoObjects itsObjects;
  const int                                  itsSegmentNum;
};

// ######################################################################
//! A new output is available from the ShapeEstimator
class SimEventShapeEstimatorOutput : public SimEvent {
public:
  //! Constuctor
  SimEventShapeEstimatorOutput(SimModule* src,
                               const Image<float>& winmap,
                               const Image<byte>& objmask,
                               const Image<byte>& iormask,
                               const Image<float>& smoothmask,
                               const Image<float>& cumsmoothmask,
                               const std::string& winlabel,
                               const bool isshaped);

  //! Destructor
  virtual ~SimEventShapeEstimatorOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the map where the winner was found
  const Image<float>& winningMap() const;

  //! Get the object mask
  /*! The object mask is a binary mask {0,255} specifiying the extend
    of the object (255 inside the object and 0 outside). Its dims are
    the dims of the map where the object was segmented (depends on
    shape estimator mode used). */
  const Image<byte>& objectMask() const;

  //! Get the IOR mask
  /*! The IOR mask is a byte map [0,255] - 0 everywhere outside the
   object, (winMapNormalized * 255) everywhere inside the object this
   is used for IOR in a graded manner - the stronger the winning
   property is in a certain position, the stronger this position is
   inhibited. */
  const Image<byte>& iorMask() const;

  //! Get the smooth mask
  /*! The smooth mask is a float map [0.0,1.0] in input image
   coordinates created from the object mask by scaling it up and
   smoothing out the edges with some specified smoothing method. */
  const Image<float>& smoothMask() const;

  //! Get the cumulative smooth mask
  /*! The cumulative smooth mask a merger of all smooth masks since
    the last reset(). */
  Image<float> cumSmoothMask() const;

  //! Get a negative of the cumulative smooth mask
  Image<float> negCumSmoothMask() const;

  //! Get description of the feature/conspic/saliency map used for masks
  const std::string& winningLabel() const;

  //! Get object area, in original input pixels
  uint objectArea() const;

  //! Did the shape extraction succeed?
  /*! If this returns true, then the ShapeEstimator was able to find a
    shape to extract. Otherwise it fell back to just using a disk at
    the attended location. The masks always contain something, just
    sometimes that thing may be a disk rather than a nicely segmented
    object. */
  bool isShaped() const;

private:
  const Image<float> itsWinningMap;
  const Image<byte> itsObjMask;
  const Image<byte> itsIORmask;
  const Image<float> itsSmoothMask;
  const Image<float> itsCumSmoothMask;
  const std::string itsWinLabel;
  const bool itsIsShaped;
};

// ######################################################################
//! A SaccadeController may post this at every evolve
/*! Note how you cannot implement an object of this type, since it
  contains a pure virtual function. Use SimEventSaccadeStatusEye or
  SimEventSaccadeStatusHead instead. */
class SimEventSaccadeStatus : public SimEvent {
public:
  //! Constuctor
  SimEventSaccadeStatus(SimModule* src, const Point2D<int>& pos,
                        const SaccadeState state,
                        const SaccadeState prevState,
                        const bool blinkState,
                        const bool prevBlinkState);

  //! Destructor
  virtual ~SimEventSaccadeStatus();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the position
  const Point2D<int>& position() const;

  //! Are we in/out/beginning/ending unknown/junk mode?
  TransientStatus unknownStatus() const;

  //! Are we in/out/beginning/ending fixation?
  TransientStatus fixationStatus() const;

  //! Are we in/out/beginning/ending saccade?
  TransientStatus saccadeStatus() const;

  //! Are we in/out/beginning/ending blink?
  TransientStatus blinkStatus() const;

  //! Are we in/out/beginning/ending smooth pursuit?
  TransientStatus smoothPursuitStatus() const;

  //! Get the body part, used by toString()
  virtual SaccadeBodyPart bodyPart() const = 0;

private:
  const Point2D<int> itsPosition;
  const SaccadeState itsState;
  const SaccadeState itsPrevState;
  const bool itsBlinkState;
  const bool itsPrevBlinkState;
};

// ######################################################################
//! An Eye SaccadeController may post this at every evolve
class SimEventSaccadeStatusEye : public SimEventSaccadeStatus {
public:
  //! Constuctor
  SimEventSaccadeStatusEye(SimModule* src, const Point2D<int>& pos,
                           const SaccadeState state,
                           const SaccadeState prevState,
                           const bool blinkState,
                           const bool prevBlinkState);

  //! Destructor
  virtual ~SimEventSaccadeStatusEye();

  //! Get the body part
  SaccadeBodyPart bodyPart() const;
};

// ######################################################################
//! An Head SaccadeController may post this at every evolve
class SimEventSaccadeStatusHead : public SimEventSaccadeStatus {
public:
  //! Constuctor
  SimEventSaccadeStatusHead(SimModule* src, const Point2D<int>& pos,
                            const SaccadeState state,
                            const SaccadeState prevState,
                            const bool blinkState,
                            const bool prevBlinkState);

  //! Destructor
  virtual ~SimEventSaccadeStatusHead();

  //! Get the body part
  SaccadeBodyPart bodyPart() const;
};

// ######################################################################
//! An EyeTrackerSaccadeController may post this
class SimEventEyeTrackerData : public SimEvent {
public:
  //! Constuctor
  SimEventEyeTrackerData(SimModule* src, rutz::shared_ptr<EyeData> d,
                         const uint trackernum,
                         const std::string& trackerfname,
                         const PixRGB<byte>& trackercolor, 
                         const PixPerDeg& ppd, const SimTime samplerate);

  //! Destructor
  virtual ~SimEventEyeTrackerData();

  //! Get the eye data
  rutz::shared_ptr<EyeData> data() const;

  //! Get the tracker number
  uint trackerNum() const;

  //! Get the tracker filename
  std::string trackerFilename() const;

  //! Get the tracker color
  PixRGB<byte> trackerColor() const;

  //! Get the current pixels per degree
  PixPerDeg trackerPpd() const;

  //!get the sampling rate
  SimTime trackerHz() const;

private:
  rutz::shared_ptr<EyeData> itsData;
  const uint itsTrackerNum;
  const std::string itsTrackerFname;
  const PixRGB<byte> itsTrackerColor;
  const PixPerDeg itsPpd;
  const SimTime itsHz;
};


// ######################################################################
//! An TrackerHandController may post this
class SimEventHandTrackerData : public SimEvent {
public:
  //! Constuctor
  SimEventHandTrackerData(SimModule* src, rutz::shared_ptr<HandData> d,
                          const uint trackernum,
                          const std::string& trackerfname,
                          const PixRGB<byte>& trackercolor, 
                          const SimTime samplerate);
  
  //! Destructor
  virtual ~SimEventHandTrackerData();

  //! Get the eye data
  rutz::shared_ptr<HandData> data() const;

  //! Get the tracker number
  uint trackerNum() const;

  //! Get the tracker filename
  std::string trackerFilename() const;

  //! Get the tracker color
  PixRGB<byte> trackerColor() const;

  //!get the sampling rate
  SimTime trackerHz() const;

private:
  rutz::shared_ptr<HandData> itsData;
  const uint itsTrackerNum;
  const std::string itsTrackerFname;
  const PixRGB<byte> itsTrackerColor;
  const SimTime itsHz;
};


// ######################################################################
//! Trigger a ChannelVisitor on VisualCortex
/*! This request is checked for and caught by VisualCortex. Upon
    receipt, VisualCortex will first call preProcessing(), the accept
    the visitor, and finally call postProcessing(). */
class SimReqVCXchanVis : public SimReq {
public:
  //! Construct from a pre-loaded ParamMap
  SimReqVCXchanVis(SimModule* src, rutz::shared_ptr<ChannelVisitor> vis);

  //! Destructor
  virtual ~SimReqVCXchanVis();

  //! Run some pre-processing before we accept()
  /*! On the base class, this is a no-op. */
  virtual void preProcessing(RawVisualCortex *vcx);

  //! Get our visitor
  rutz::shared_ptr<ChannelVisitor> visitor() const;

  //! Run some post-processing after we accept()
  /*! On the base class, this is a no-op. */
  virtual void postProcessing(RawVisualCortex *vcx);

private:
  rutz::shared_ptr<ChannelVisitor> itsVisitor;
};

// ######################################################################
//! Indicate which object we are biasing for (use for statistics)
class SimEventObjectToBias : public SimEvent {
public:
  SimEventObjectToBias(SimModule* src, const std::string& objName);

  virtual ~SimEventObjectToBias();

  const std::string& name() const;

private:
  const std::string itsObjName;
};

// ######################################################################
//! A new target mask is available, TargetChecker will check for this and use the new mask
class SimEventTargetMask : public SimEvent {
public:
  //! Constuctor
  SimEventTargetMask(SimModule* src, const Image<byte>& tmask);

  //! Destructor
  virtual ~SimEventTargetMask();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the target mask
  const Image<byte> mask() const;

private:
  const Image<byte> itsMask;
};

// ######################################################################
//! Request VCX features at a given location
/*! PROGRAMMER NOTE: This is an example of a read/write SimReq,
    whereby the VCX directly dumps its features into the request and
    posts nothing in return. The object that did a request() of this
    event can then directly use the results in the event as soon as
    request() returns. If features() is empty after the event has been
    requested, then it means that no VisualCortex caught it and there are
    no features available. */
class SimReqVCXfeatures : public SimReq {
public:
  //! Constuctor
  SimReqVCXfeatures(SimModule* src, const Point2D<int>& p);

  //! Destructor
  virtual ~SimReqVCXfeatures();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get access to the location of the features
  const Point2D<int>& loc() const;

  //! Get read/write access to the features
  std::vector<float>& features();

private:
  const Point2D<int> itsLoc;
  std::vector<float> itsFeatures;
};

// ######################################################################
//! Request VCX internal maps
/*! PROGRAMMER NOTE: This is an example of a read/write SimReq,
    whereby the VCX directly dumps its features into the request and
    posts nothing in return. The object that did a request() of this
    event can then directly use the results in the event as soon as
    request() returns. While this object is quite benign as long as
    everything runs on a same machine, beware that it is potentially
    quite costly to transport from one machine to another via
    network. */
class SimReqVCXmaps : public SimReq {
public:
  //! Constuctor
  SimReqVCXmaps(SimModule* src);

  //! Destructor
  virtual ~SimReqVCXmaps();

  //! Get the ChannelMaps object that will contain all the VCX maps
  rutz::shared_ptr<ChannelMaps> channelmaps() const;

private:
  rutz::shared_ptr<ChannelMaps> itsChannelMaps;

  friend class VisualCortexStd;
  friend class VisualCortexInt;
  friend class VisualCortexEnv;
  friend class VisualCortexEyeMvt;
  void populateChannelMaps(RawVisualCortex *vcx);
  void populateChannelMaps(IntegerRawVisualCortex *vcx);
  void populateChannelMaps(EnvVisualCortexFloat *vcx);
  void populateChannelMaps(VisualCortexEyeMvt *vcx);
};

// ######################################################################
//! A new output is available from the VisualBuffer
class SimEventVisualBufferOutput : public SimEvent {
public:
  //! Constuctor
  SimEventVisualBufferOutput(SimModule* src, const Image<float>& buf,
                             const int smlev, const Dims& smdims, const Point2D<int>& retoff);

  //! Destructor
  virtual ~SimEventVisualBufferOutput();

  //! Get a description for printing out
  virtual std::string toString() const;

  //! Get the buffer
  const Image<float>& buffer() const;

  //! transform coord from retinotopic/retinal-scale to world-centered/sm-scale
  Point2D<int> retinalToBuffer(const Point2D<int>& p) const;

  //! transform coord from world-centered/sm-scale to retinotopic/retinal-scale
  Point2D<int> bufferToRetinal(const Point2D<int>& p) const;

  //! SM dims
  const Dims& smdims() const;

  //! SM level
  int smlev() const;

private:
  const Image<float> itsVisualBuffer;
  const int itsSMlev;
  const Dims itsSMdims;
  const Point2D<int> itsRetinaOffset;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_NEUROSIMEVENTS_H_DEFINED
