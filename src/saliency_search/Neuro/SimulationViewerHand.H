/*!@file Neuro/SimulationViewerHand.H comparison between saliency and
  human hands movement reaction (joystick, mouse, keyboard) */

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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL:
// $Id:
//

#ifndef SIMULATIONVIEWERHAND_H_DEFINED
#define SIMULATIONVIEWERHAND_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/ImageCache.H"
//#include "Image/ImageSet.H"
//#include "Image/LevelSpec.H"
#include "Neuro/SimulationViewer.H"
#include "Simulation/SimEvents.H"

#include <map>
#include <vector>

class HandData;
class SpatialMetrics;
class ofstream;
class SimTime;

typedef Point2D<int> (*Point2DTransform)(Point2D<int> P);

//! Measure salience at human eye positions
class SimulationViewerHand : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerHand(OptionManager& mgr,
                       const std::string& descrName =
                       "Hand Simulation Viewer",
                       const std::string& tagName =
                       "SimulationViewerHand");
  
  //! Destructor
  virtual ~SimulationViewerHand();
  
  //@}

protected:
  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(SimulationViewerHand, SimEventClockTick);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerHand, SimEventSaveOutput);

  //! Save our various results
  void save1(const ModelComponentSaveInfo& sinfo);
  
  //! Get the attention/eye/head trajectory image
  virtual Image< PixRGB<byte> > getTraj(SimEventQueue& q);

  nub::ref<SpatialMetrics> itsMetrics;//!< metrics that depend on input size
  
  OModelParam<bool> itsSaveTraj;      //!< save trajectory?
  OModelParam<bool> itsSaveCombo;     //!< save combo?
  OModelParam<bool> itsDisplayHand;  //!< display Hand position
  OModelParam<int> itsPatchSize;      //!< size of marker at eye position
  OModelParam<bool> itsEraseMarker;   //!< erase marker at each frame
  OModelParam<int> itsMaxComboWidth; //!< max width of getTraj()

  virtual void start1(); //!< get started
  virtual void stop1();  //!< get stopped

  virtual void drawHand(const rutz::shared_ptr<HandData> data, const uint trackerNum);  // overloaded by HandRegion

  // overloaded by HandRegion to do its output
  virtual void extraSampleProcessing(const rutz::shared_ptr<HandData>);

  Image< PixRGB<byte> > itsDrawings;  // our drawings - black is transparent!
  // these drawings include, from SVHand: the eyetrace
  // from SVHandRegion: the names and borders of regions

  SimTime itsCurrTime;
  uint itsFrameNumber;     // this member is never used in SVHand

  bool itsHeaderCrafted;
  std::vector<std::string> itsOutFields;

  struct GHandFormat {
    PixRGB<byte> col;
    std::string label; // depends on OModelParam
    int pSize; // patch size
    // shape? other aspects?
  };

  std::vector<GHandFormat> itsHandStyles;
private:
  // Used in DrawHand for coordinate system
  // Get the grid coordinate of position W-th and H-th of a
  // grid system with src size of block
  inline Dims getGridCoord(Dims src, int W_, int H_) {
    Dims dst (src.w() * W_, src.h() * H_);
    return dst; 
  };
  std::map<int, rutz::shared_ptr<HandData> > itsTargets;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
