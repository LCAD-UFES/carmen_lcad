/*!@file Neuro/SimulationViewerCompress.H multi-foveated saliency-based
  compression */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerCompress.H $
// $Id: SimulationViewerCompress.H 11546 2009-07-31 18:19:09Z lzc $
//

#ifndef SIMULATIONVIEWERCOMPRESS_H_DEFINED
#define SIMULATIONVIEWERCOMPRESS_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/ImageCache.H"
#include "Image/ImageSet.H"
#include "Image/LevelSpec.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SimulationViewer.H"
#include "Simulation/SimEvents.H"
#include "Util/SimTime.H"

#include <vector>

class SaccadeController;
class SpatialMetrics;

//! Do multi-foveated saliency-based image compression
/*! This viewer will consider the top N salient locations and apply
  blurring to the image that increases in strength (sigma) as we get
  farther away from any of the top N most salient locations. The
  resulting trajectory image hence will be crisp at the top N
  locations and increasingly blurred away from those. Compressing the
  output of this viewer, e.g., using MPEG, yields smaller file size
  while (hopefully) preserving high quality at the important image
  locations. This viewer relies on a collection of N
  SaccadeControllers to track the N hotspots; you can pick which type
  of controller to use and configure their parameters as usual, via
  the command line. Note that in programs like ezvision that use an
  StdBrain, the StdBrain also contains a SaccadeController; but here
  we will explicitly drop it, as we won;t be using Brain's controller,
  since we will decide on the N hotspots here based onthe raw saliency
  map. We will also drop Brain's ShapeEstimator and force Brain's IOR
  type to none, so that we have a clean saliency map here to work
  with. */
class SimulationViewerCompress : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerCompress(OptionManager& mgr,
                           const std::string& descrName =
                           "Multi-Foveated Compression Simulation Viewer",
                           const std::string& tagName =
                           "SimulationViewerCompress");

  //! Destructor
  virtual ~SimulationViewerCompress();

  //@}

protected:
  //! Callback for when a new retina image is available
  SIMCALLBACK_DECLARE(SimulationViewerCompress, SimEventRetinaImage);

  //! Callback for when the eye moves
  SIMCALLBACK_DECLARE(SimulationViewerCompress, SimEventSaccadeStatusEye);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerCompress, SimEventSaveOutput);

  //! Get the attention/eye/head trajectory image
  Image< PixRGB<byte> > getTraj(SimEventQueue& q);

  OModelParam<int> itsFOAradius;        //!< FOA radius for object trackers
  OModelParam<int> itsNumFoveas;        //!< number of foveas
  OModelParam<bool> itsSaveTraj;        //!< save trajectory?
  OModelParam<bool> itsSaveMegaCombo;   //!< save mega combo?
  OModelParam<bool> itsSaveMask;        //!< save mask?
  OModelParam<bool> itsSaveFoveatedImage;   //!< save foveated image?
  OModelParam<float> itsDistanceFactor;     //!< distance factor to change the fovea size
  OModelParam<bool> itsSaveEyeCombo;    //!< save eye combo?
  OModelParam<bool> itsDisplayPatch;    //!< draw patches
  OModelParam<bool> itsDisplayFOA;      //!< draw objetc outlines
  OModelParam<bool> itsDisplayEye;      //!< draw human eye movements
  NModelParam< PixRGB<byte> > itsColorNormal; //!< patch color
  NModelParam< PixRGB<byte> > itsColorEye;  //!< patch color for human eye mvts
  OModelParam<int> itsHeadRadius;           //!< head radius
  OModelParam<int> itsMultiRetinaDepth;     //!< depth of blur pyramid
  OModelParam<int> itsCacheSize;            //!< size of our mask cache
  OModelParam<bool> itsUseTRMmax;           //!< use TRM to take max in cache
  OModelParam<std::string> itsFoveaSCtype;  //!< type of SC for foveas
  OModelParam<std::string> itsOutFname;     //!< Name of output file
  OModelParam<LevelSpec> itsLevelSpec;      //!< our levelspec
  OModelParam<int> itsNumRandomSamples;     //!< number of random samples
  OModelParam<bool> itsEyeCompare;          //!< do the eye compare with the mask

  //! This parameter is the period (in frames) for foveation mask to change
  /*! The idea here is to see whether keeping the foveation mask
    stable for predicted frames and changing it only for intra-coded
    frames may improve the compression ratio when using MPEG-1
    compression. Have a look at
    http://www.disctronics.co.uk/technology/video/video_mpeg.htm if
    you are not familiar with those various types of frames. In
    particular (quoted from that web site):

    "I-frames (Intra coded frames) use DCT encoding only to compress a
    single frame without reference to any other frame in the sequence.
    [...]
    P-frames (Predicted frames) are coded as differences from the last
    I or P frame. The new P-frame is first predicted by taking the
    last I or P frame and 'predicting' the values of each new
    pixel. P-frames use Motion Prediction and DCT encoding. As a
    result P-frames will give a compression ratio better than I-frames
    but depending on the amount of motion present. The differences
    between the predicted and actual values are encoded. [...]
    B-frames (Bidirectional frames) are coded as differences from the
    last or next I or P frame. B-frames use prediction as for P-frames
    but for each block either the previous I or P frame is used or the
    next I or P frame. [...]"

    So, in our case, changing the foveation mask on a P or B frame may
    yield lots of prediction errors, that can be reduced if we force
    the mask to be only allowed to change on I frames. Here we assume
    that the first frame is an I-frame. */
  OModelParam<int> itsIFramePeriod;

  //! Get started and disable any SC the Brain may want to use
  virtual void start1();

  //! get stopped
  virtual void stop1();

  //! Intercept people changing our number of foveas
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  ImageSet< PixRGB<byte> > itsMultiTraj; // used to foveate traj
  std::vector< nub::soft_ref<SaccadeController> > itsSC; // our SCs
  SimTime itsInputTime;                   // time of last input
  int itsFrame;                          // keep track of frame number

  void buildSCC(); // build our SaccadeControllerConfigurators

  float getSample(const Image<float>& smap, const Point2D<int>& p,
                  const int radius) const;
  float getRandomSample(const Image<float>& smap,
                        const int radius, const int n) const;

  // create mask from SCs
  Image<byte> getMaskSC(const Image<float>& smf, SimEventQueue& q);

  // create mask from SM
  Image<byte> getMaskSM(const Image<float>& smf);

  // keep track of features being tracked by each SC:
  std::vector< std::vector<float> > itsFeatures;

  // use a sliding image cache for our foveation masks:
  ImageCacheAvg<byte> itsMask;

  // ignore SCs that have too low salience:
  std::vector<bool> itsIgnoreSC;

  Image<byte> itsCurrentMask;  // last I-frame mask computed

  // eye movement comparison stuff:
  FILE *itsOutFile;

  Image<byte> itsBlurMask;  // final current blur mask to use
  std::deque<Point2D<int> > itsEyeData;  // queued-up eye movement data over 1 frame

  Image<PixRGB<byte> > itsRawInputCopy; //FIXME
  Rectangle itsRawInputRectangle; //FIXME
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
