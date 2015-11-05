/*!@file Neuro/SimulationViewerNerdCam.H entry interface between INVT and ASAC */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerNerdCam.H $
// $Id: SimulationViewerNerdCam.H 10714 2009-02-01 07:16:36Z itti $
//

#ifndef SIMULATIONVIEWERNERDCAM_H_DEFINED
#define SIMULATIONVIEWERNERDCAM_H_DEFINED

#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Image/ArrayData.H"   // for Dims
#include "Image/Image.H"       // for Image
#include "Image/ImageSet.H"
#include "Image/Pixels.H"      // for PixRGB<byte>
#include "Image/Point2DT.H"    // for Point2DT
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/Brain.H"
#include "Neuro/SimulationViewer.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/WTAwinner.H"   // for WTAwinner
#include "Simulation/SimEvents.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"   // for rutz::shared_ptr
#include <fstream>
#include <fcntl.h> // for open()
#include <cerrno> // for errno
#include <cstdio> // for perror()

class Brain;
class FrameOstream;

//! This class provides interface with vision and NerdCam
/*! This simulation viewer is designed to take brain and VisualCortex
    information and process it for use with the nerd cam web page. For instance
    we can control file locking, motion noticing and file saving. We use
    a SimulationViewer to standardize its workings with ezvision.
*/
class SimulationViewerNerdCam : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage */
  SimulationViewerNerdCam(OptionManager& mgr,
                       const std::string& descrName = "NerdCam Interface",
                       const std::string& tagName   = "NerdCam");
  //! Destructor
  virtual ~SimulationViewerNerdCam();

  //! Reset SimulationViewerNerdCam
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

  //! basic init method
  void init(const ushort baseSizeX,const ushort baseSizeY);

  //@}

protected:
  //! Callback for when a new input image is available
  SIMCALLBACK_DECLARE(SimulationViewerNerdCam, SimEventInputFrame);

  //! Callback for every clock tick
  /*! Note: this is only being used by warp3d right now and should go away. */
  SIMCALLBACK_DECLARE(SimulationViewerNerdCam, SimEventClockTick);

  //! Callback for when a new attention shift occurs
  SIMCALLBACK_DECLARE(SimulationViewerNerdCam, SimEventWTAwinner);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerNerdCam, SimEventSaveOutput);

  //! Save our various results
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Get the attention/eye/head trajectory image
  Image< PixRGB<byte> > getTraj(SimEventQueue& q);

  //! Called by saveOutput()
  void saveResults(const nub::ref<FrameOstream>& ofs, SimEventQueue& q);

  //! Get started:
  virtual void start2();
  //! write a basic status page out
  void writeStatusPage() const;
  //! write a basic status page out
  void writeChannelPage() const;
  //! Draw current time onto given image
  void drawTime(Image< PixRGB<byte> >& image) const;
  //! apply an advisory lock on a file
  void lockFile(const std::string fileName, int &fd, struct flock &fl) const;
  //! remove an advisory lock on a file
  void unlockFile(const std::string fileName, const int fd,
                         struct flock &fl) const;
  //! log data to standard log file
  void nlog(const std::string logData) const;
  //! Draw the date and time onto an image
  void drawDateTime(Image< PixRGB<byte> >& image) const;
  //! Draw the focus of covert attention
  void drawFOA();
  //! Link previous focus of attention to current one
  void linkFOAs();
  //! draw the mega combo
  Image< PixRGB<byte> > drawMegaCombo(SimEventQueue& q) const;
  //!  Draw outline of a mask (used by drawFOA)
  void drawMaskOutline(Image< PixRGB<byte> >& traj,
                       const Image<byte> mask,
                       const PixRGB<byte>& col,
                       const int thick,
                       const Point2D<int>& pos,
                       const int radius) const;

  //! metrics that depend on the input size:
  nub::ref<SpatialMetrics> itsMetrics;

  //! drawing color selection parameter
  NModelParam< PixRGB<byte> > itsColorBoring;
  //! drawing color selection parameter
  NModelParam< PixRGB<byte> > itsColorLink;
  //! drawing color selection parameter
  NModelParam< PixRGB<byte> > itsColorNormal ;
  //! config file to open
  OModelParam<std::string>    itsConfigFile;

  // ########## display selection parameters:
  OModelParam<Dims>  itsCropFOA;
  OModelParam<bool>  itsDisplayAdditive;
  OModelParam<bool>  itsDisplayBoring;
  OModelParam<bool>  itsDisplayFOA;
  OModelParam<bool>  itsDisplayFOALinks;
  OModelParam<bool>  itsDisplayHighlights;
  OModelParam<bool>  itsDisplayPatch;
  OModelParam<bool>  itsDisplaySMmodulate;
  OModelParam<bool>  itsDisplayTime;

  //! half-size of (filled square) FOA patch
  NModelParam<int>   itsFOApsiz;
  //! line thickness for FOA outline
  NModelParam<int>   itsFOAthick;
  //! line thickness for links bewteen FOAs
  NModelParam<int>   itsFOAlinkThick;
  OModelParam<bool>  itsMegaCombo;
  NModelParam<float> itsWarp3Dpitch;
  NModelParam<float> itsWarp3Dyaw;
  NModelParam<float> itsWarp3DpitchRate;
  NModelParam<float> itsWarp3DyawRate;
  NModelParam<float> itsWarp3DpitchMax;
  NModelParam<float> itsWarp3DyawMax;
  //! use larger drawings?
  OModelParam<bool>  itsUseLargerDrawings;

private:
  //! cumulative FOA shape
  Image<byte>              itsCumFOAmask;
  //! current attention position
  WTAwinner                itsCurrFOA;
  //! current FOA shape
  Image<byte>              itsCurrFOAmask;
  //! current time
  SimTime                  itsCurrTime;
  //! dims of 3D drawing
  Dims                     itsDims3D;
  //! used to foveate traj
  ImageSet< PixRGB<byte> > itsMultiTraj;
  // previous attention position
  WTAwinner                itsPrevFOA;
  //! Some drawings go here
  Image< PixRGB<byte> >    itsTraj;
  //! local copy of the input image
  Image< PixRGB<float> >   itsInput;
  //! copy of saliency map
  Image<float>             itsSalMap;
  //! what file to write output motion image file to
  std::string   itsOutputMotionImage;
  //! what file to write output SalMap image file to
  std::string   itsOutputSalMapImage;
  //! what file to write output 3D saliency image
  std::string   itsOutput3DImage;
  //! what file to write output Mega saliency image
  std::string   itsOutputMegaImage;
  //! what file to write a basic trajectory image to
  std::string   itsOutputTrajImage;
  //! where is the base web page?
  std::string   itsWebPageFile;
  //! status web page
  std::string   itsStatusFile;
  //! status web page header
  std::string   itsStatusHeader;
  //! status web page footer
  std::string   itsStatusFooter;
  //! channel web page
  std::string   itsChannelFile;
  //! URL of the web page
  std::string   itsBaseURL;
  //! The name of this web page
  std::string   itsBaseName;
  //! the file to log stuff to
  std::string   itsLogFile;
  //! Time stamp for start up
  std::string   itsStartTime;
  //! how many frames have we processed
  unsigned long itsTotalFrames;
  //! 3D drawing; mutable for testsuite
  mutable float itsPitch3D;
  //! 3D drawing; mutable for testsuite
  mutable float itsYaw3D;
  //! floating point threshold above which we spot moving things
  float         itsMotionThreshold;
  //! the base image size in X
  ushort        itsBaseSizeX;
  //! the base image size in Y
  ushort        itsBaseSizeY;
  //! size of the mega combo image x
  ushort        itsComboSizeX;
    //! size of the mega combo image y
  ushort        itsComboSizeY;
  //! size of the mega combo image x
  ushort        its3DSizeX;
    //! size of the mega combo image y
  ushort        its3DSizeY;
  //! input changed since last getTraj()
  bool          itsHasNewInput;
  //! tells us if we need to initialize ScaleSurpriseControl
  bool          itsInit;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
