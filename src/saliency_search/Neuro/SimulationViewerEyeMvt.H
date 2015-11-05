/*!@file Neuro/SimulationViewerEyeMvt.H comparison between saliency and
  human eye movements */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeMvt.H $
// $Id: SimulationViewerEyeMvt.H 14887 2011-08-20 06:23:15Z pohetsn $
//

#ifndef SIMULATIONVIEWEREYEMVT_H_DEFINED
#define SIMULATIONVIEWEREYEMVT_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/ImageCache.H"
#include "Image/ImageSet.H"
#include "Image/LevelSpec.H"
#include "Neuro/SimulationViewer.H"
#include "Simulation/SimEvents.H"

#include <map>
#include<vector>

class EyeData;
class SpatialMetrics;
class ofstream;
class SimTime;

typedef Point2D<int> (*Point2DTransform)(Point2D<int> P);

//! Measure salience at human eye positions
class SimulationViewerEyeMvt : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerEyeMvt(OptionManager& mgr,
                         const std::string& descrName =
                         "EyeMvt Simulation Viewer",
                         const std::string& tagName =
                         "SimulationViewerEyeMvt");

  //! Destructor
  virtual ~SimulationViewerEyeMvt();

  //@}

protected:
  //! Callback for every clock tick
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvt, SimEventClockTick);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerEyeMvt, SimEventSaveOutput);

  //! Save our various results
  void save1(const ModelComponentSaveInfo& sinfo);

  //! Get the attention/eye/head trajectory image
  virtual Image< PixRGB<byte> > getTraj(SimEventQueue& q);

  nub::ref<SpatialMetrics> itsMetrics;//!< metrics that depend on input size

  OModelParam<bool> itsSaveTraj;      //!< save trajectory?
  OModelParam<bool> itsSaveMegaCombo; //!< save mega combo?
  OModelParam<int> itsDelayCacheSize; //!< size of our delay cache
  OModelParam<int> itsMaxCacheSize;   //!< size of our max cache
  OModelParam<bool> itsSampleAtStart; //!< take samples at start or end of sac
  OModelParam<bool> itsDisplaySacNum; //!< display saccade number?
  OModelParam<bool> itsDisplayPatch;  //!< display eye position
  OModelParam<int> itsPatchSize;      //!< size of marker at eye position
  OModelParam<bool> itsEraseMarker;   //!< erase marker at each frame
  OModelParam<bool> itsDisplayFOA; //!< display saccade targets?
  OModelParam<LevelSpec> itsLevelSpec; //!< our levelspec
  OModelParam<std::string> itsOutFname;  //!< Our results file
  OModelParam<std::string> itsPriorRandomDistro; //!< prior sampling from File
  OModelParam<bool> itsViewStrategy; //!< do viewing strategy to compensate sampling baseline?
  OModelParam<std::string> itsViewStrategyDistro; //!< prior sampling from gaussian distribution (viewing strategy)
  OModelParam<std::string> itsViewStrategyTiming; //!< prior sampling from gaussian reset timing
  OModelParam<bool> itsUseSaccadeInBlink;//!< use saccade during blink?
  OModelParam<bool> itsUseDiagColor;//!< use saccade during blink?
  OModelParam<bool> itsLabelEyePatch; //!< label eyetraces on video
  OModelParam<bool> itsWriteFrameNum; //!< write framenumber
  OModelParam<int> itsNumRandomSamples; //!< number of random samples
  OModelParam<uint> itsHistoryNumRandomSamples; //!< number of random samples
  OModelParam<int> itsMaxComboWidth; //!< max width of getTraj()
  OModelParam<uint> itsSMhistoryQlen; //!< queue len for Sm history

  virtual void start1(); //!< get started
  virtual void stop1(); //!< get stopped

  virtual void drawEye(const rutz::shared_ptr<EyeData> data, const uint trackerNum);  // overloaded by EyeRegion
  virtual void drawFOA(const Point2D<int> target, const uint trackerNum); // overloaded by EyeRegion

  // where the SVEM output is all put together
  virtual std::string craftSVEMOutput(const std::string TrackerNum,
                              const rutz::shared_ptr<EyeData> data);

  // overloaded by EyeRegion to do its output
  virtual void extraSampleProcessing(const rutz::shared_ptr<EyeData>);

  // auxiliaries for file I/O
  std::string craftModelFreeOutput(const rutz::shared_ptr<EyeData> data);
  std::string craftSMSamples(const rutz::shared_ptr<EyeData> data, Image<float> smap);
  std::string craftSMHistory(const rutz::shared_ptr<EyeData> data, Image<float> smap);
  void writeHeader();

  // use a sliding image cache for salience computations:
  ImageCacheMinMax<float> itsDelayCache;
  ImageCacheMinMax<float> itsMaxCache;

  // the saliency map at the head of the delay cache;
  Image<float> itsHeadSM;

  Image< PixRGB<byte> > itsDrawings;  // our drawings - black is transparent!
  // these drawings include, from SVEyeMvt: the eyetrace
  // from SVEyeRegion: the names and borders of regions

  SimTime itsCurrTime;
	SimTime itsLastSceneOnsetTime;
  uint itsFrameNumber;     // this can now be outputted to eyesal file

  bool itsHeaderCrafted;
  std::vector<std::string> itsOutFields;

  // based on SimEventRetinaImage::rawToRetinal()
  // wraps the rawToRetinal transform for inherited functions to use
  Point2D<int> rawToRet(Point2D<int> P) const {return P+itsRawToRetOffset;}

  struct GEyeFormat {
    PixRGB<byte> col;
    std::string label; // depends on OModelParam
    int pSize; // patch size
    // shape? other aspects?
  };

  std::vector<GEyeFormat> itsEyeStyles;
private:
  std::map<int, rutz::shared_ptr<EyeData> > itsTargets;
  std::vector<Point2D<int> > randPoints;
	std::vector<uint> vsResetFrame; //frame number of onset of a new scene
	std::vector<double> itsFrame2OnsetTime; //frame timing reset by onset of a new scene
  ImageCache<float> itsSMhistory;

  std::ofstream *itsOutFile;
  std::ifstream *itsRandFile;
	std::ifstream *itsVSFile;
  Point2D<int> itsRawToRetOffset;
	double vsParams [8];
	int nextOnsetIdx;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
