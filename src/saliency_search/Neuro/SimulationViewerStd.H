/*!@file Neuro/SimulationViewerStd.H visualize various model simulations */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerStd.H $
// $Id: SimulationViewerStd.H 14468 2011-02-02 08:42:19Z dberg $
//

#ifndef SIMULATIONVIEWERSTD_H_DEFINED
#define SIMULATIONVIEWERSTD_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/ImageSet.H"
#include "Image/LevelSpec.H"
#include "Image/NamedImage.H"
#include "Image/Pixels.H"      // for PixRGB<byte>
#include "Image/SimpleFont.H"
#include "Neuro/SimulationViewer.H"
#include "Simulation/SimEvents.H"
#include "rutz/shared_ptr.h"   // for rutz::shared_ptr

class SpatialMetrics;

//! This class provides a standard set of visualization tools
/*! Its typical usage is with a model that generates shifts of
  attention, eye and head movements. This class will handle
  visualizing those events. There are three ways of customizing it to
  your specific needs: 1) by changing the values of some of its
  internal ModelParam parameters, either via the command-line or via a
  config file for those that do not have command-line options; 2) by
  overloading (in a derived class) the low-level drawing functions in
  the protected section of the class definition; 3) by overloading the
  trigger functions in the public section of the class definition. */
class SimulationViewerStd : public SimulationViewerAdapter {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  SimulationViewerStd(OptionManager& mgr,
                      const std::string& descrName = "Simulation Viewer Std",
                      const std::string& tagName = "SimulationViewerStd");
  //! Destructor
  virtual ~SimulationViewerStd();

  //@}

protected:
  //! Callback for when the eye moves
  SIMCALLBACK_DECLARE(SimulationViewerStd, SimEventSaccadeStatusEye);

  //! Callback for when the head moves
  SIMCALLBACK_DECLARE(SimulationViewerStd, SimEventSaccadeStatusHead);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerStd, SimEventSaveOutput);

  //! Callbacks for when a IT Output is available
  SIMCALLBACK_DECLARE(SimulationViewerStd, SimEventITOutput);

  //! Simulation time step
  OModelParam<SimTime> itsTimeStep;

  //! the model's LevelSpec
  OModelParam<LevelSpec> itsLevelSpec;

  //! metrics that depend on the input size:
  nub::ref<SpatialMetrics> itsMetrics;

  // ########## what to compute and show parameters:
  OModelParam<uint> itsFontSize;
  OModelParam<bool> itsSaveTraj;
  OModelParam<bool> itsSaveXcombo;
  OModelParam<bool> itsSaveYcombo;
  OModelParam<bool> itsSaveTRMXcombo;
  OModelParam<bool> itsSaveTRMYcombo;
  OModelParam<bool> itsSaveTRMmegaCombo;
  OModelParam<bool> itsWarp3D;
  OModelParam<bool> itsMegaCombo;
  OModelParam<uint> itsMegaComboZoom;
  OModelParam<bool> itsMegaComboTopCMapsOnly;
  OModelParam<Dims> itsCropFOA;
  OModelParam<bool> itsFoveateTraj;

  // ########## display selection parameters:
  OModelParam<bool> itsDisplayFOA;
  OModelParam<bool> itsDisplayPatch;
  OModelParam<bool> itsDisplayFOAnum;
  OModelParam<bool> itsDisplayFOALinks;
  OModelParam<bool> itsDisplayEye;
  OModelParam<bool> itsDisplayEyeLinks;
  OModelParam<bool> itsDisplayHead;
  OModelParam<bool> itsDisplayHeadLinks;
  OModelParam<bool> itsDisplayTime;
  OModelParam<bool> itsDisplayAdditive;
  OModelParam<bool> itsDisplayHighlights;
  OModelParam<bool> itsDisplaySMmodulate;
  OModelParam<bool> itsDisplayBoring;
  NModelParam<bool> itsDisplayShapeEstimator;

  // ########## drawing color selection parameters:
  NModelParam< PixRGB<byte> > itsColorNormal;
  NModelParam< PixRGB<byte> > itsColorBoring; //!< subtracted when boring
  NModelParam< PixRGB<byte> > itsColorBlink;
  NModelParam< PixRGB<byte> > itsColorSaccade;
  NModelParam< PixRGB<byte> > itsColorSmoothPursuit;
  NModelParam< PixRGB<byte> > itsColorLink;
  NModelParam< PixRGB<byte> > itsColorText;

  // ########## misc default values parameters:
  NModelParam<int> itsHighlightMax;
  NModelParam<float> itsShapeEstimatorBaseContrast;
  NModelParam<int> itsShapeEstimatorBaseBright;

  NModelParam<float> itsWarp3Dpitch;
  NModelParam<float> itsWarp3Dyaw;
  NModelParam<float> itsWarp3DpitchRate;
  NModelParam<float> itsWarp3DyawRate;
  NModelParam<float> itsWarp3DpitchMax;
  NModelParam<float> itsWarp3DyawMax;

  // ########## general model parameters, shared with other components:
  OModelParam<int> itsHeadRadius;
  OModelParam<int> itsMultiRetinaDepth;

  // ########## parameters for drawings (not exported as command-line opts):
  NModelParam<int> itsFOApsiz;     //!< half-size of (filled square) FOA patch
  NModelParam<int> itsFOAthick;    //!< line thickness for FOA outline
  NModelParam<int> itsFOAlinkThick;//!< line thickness for links bewteen FOAs
  NModelParam<int> itsFOVpsiz;     //!< half-size of (hollow square) FOV patch
  NModelParam<int> itsFOVpthick;   //!< line thickness for FOV patch
  NModelParam<int> itsFOVthick;    //!< line thickness for FOV outline
  NModelParam<int> itsFOVlinkThick;//!< line thickness for links btw FOVs
  NModelParam<int> itsHEDpsiz;     //!< half-size of (hollow-square) head patch
  NModelParam<int> itsHEDpthick;   //!< line thickness for head patch
  NModelParam<int> itsHEDthick;    //!< line thickness for head outline
  NModelParam<int> itsHEDlinkThick;//!< line thickness for links btw heads

  OModelParam<bool> itsUseLargerDrawings;  //!< use larger drawings?

  // ########## obsolete parameters, which we still keep around so
  // ########## that the user can get a helpful error message if they
  // ########## try to request the parameter on the command-line
  OModelParam<bool> itsObsolete1;

  //!process retinal images
  virtual void doEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e);

  //process wta winners
  virtual void doEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e);

  //! Get started:
  virtual void start2();

  //! Reset SimulationViewerStd
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

  //! Save our various results
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Get the attention/eye/head trajectory image
  Image< PixRGB<byte> > getTraj(SimEventQueue& q);

  // ######################################################################
  /*! @name Derived classes should overload those to fit their cosmetic needs*/
  //@{

  //! Draw the focus of covert attention
  void drawFOA();

  //! Link previous focus of attention to current one
  void linkFOAs();

  //! Draw current eye position
  void drawEye();

  //! Link previous eye position to current one
  void linkEyes();

  //! Draw current head position
  void drawHead();

  //! Link previous head position to current one
  void linkHeads();

  //! Draw current time onto given image
  void drawTime(Image< PixRGB<byte> >& image);

  //! Draw highlights around a given position
  void drawHighlights();

  //! Draw mega combo for X display
  Image< PixRGB<byte> > drawMegaCombo(SimEventQueue& q,
                                      const Image< PixRGB<byte> >& in);

  //! Draw motion sixpack
  Image< PixRGB<byte> > drawMotionSixPack(const Image< PixRGB<byte> >& in);

  //! Draw outline of a mask (used by drawFOA, drawEye, etc)
  /*! if the mask is initialized, revert to drawing a circle centered
    at "pos" an of radius "radius" */
  void drawMaskOutline(Image< PixRGB<byte> >& traj, const Image<byte> mask,
                       const PixRGB<byte>& col, const int thick,
                       const Point2D<int>& pos, const int radius);

  //! recurse to get all maps to display in a megacombo
  void getMegaComboMaps(const rutz::shared_ptr<ChannelMaps>& chm,
                        ImageSet< PixRGB<byte> >& cmaps, const uint depth = 0);

  //! Prepare a map for display
  Image< PixRGB<byte> > prepMapForDisplay(const NamedImage<float>& m) const;

  //@}

private:
  friend class SimulationViewerEyeSim;
  Image< PixRGB<byte> > itsTraj;  // all our drawings go here
  uint itsFOAshiftNum;            // Attention shift number (0-based)
  Image<byte> itsCurrFOAmask;     // current FOA shape
  Image<byte> itsCumFOAmask;      // cumulative FOA shape
  Point2D<int> itsCurrEye;             // current eye position
  Image<byte> itsCurrFoveaMask;   // current fovea shape
  Image<byte> itsCumFoveaMask;    // cumulative fovea shape
  Point2D<int> itsCurrHead;            // current head position
  Image<byte> itsCurrHeadMask;    // current head shape
  Image<byte> itsCumHeadMask;     // cumulative head shape

  Point2D<int> itsPrevEye;             // previous eye position
  Point2D<int> itsPrevHead;            // previous head position
  SimTime itsCurrTime;            // time of last evolve()
  bool itsEyeSaccade;             // are we in eye saccade?
  bool itsEyeSmoothPursuit;       // are we in eye smooth pursuit?
  bool itsHeadSaccade;            // are we in head saccade?
  bool itsHeadSmoothPursuit;      // are we in head smooth pursuit?
  bool itsEyeBlink;               // are we in eye blink?
  ImageSet< PixRGB<byte> > itsMultiTraj; // used to foveate traj
  Dims itsDims3D;                 // dims of 3D drawing
  mutable float itsPitch3D, itsYaw3D; // 3D drawing; mutable for testsuite
  bool itsHasNewInput;            // input changed since last getTraj()
  SimpleFont itsFont;             // set from itsFontSize during start2()
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
