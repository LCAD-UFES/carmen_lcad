/*!@file Neuro/SimulationViewerEyeRegion.H comparison between region data  from an ObjRec object (from an XML file) and human eye movements */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeRegion.H $
// $Id: SimulationViewerEyeRegion.H 13644 2010-07-06 17:07:43Z jshen $
//

#ifndef SIMULATIONVIEWEREYEREGION_H_DEFINED
#define SIMULATIONVIEWEREYEREGION_H_DEFINED

#include "Component/ModelParam.H"
//#include "Image/ImageCache.H"
//#include "Image/ImageSet.H"
//#include "Image/LevelSpec.H"
#include "Neuro/SimulationViewerEyeMvt.H"
#include "Simulation/SimEvents.H"

// add class definitions for Object Recognition
class TestImages;

typedef TestImages::SceneData Scene;
typedef TestImages::ObjData Object;
//! Measure salience at human eye positions

const uint NULL_OBJ = -1;

class SimulationViewerEyeRegion : public SimulationViewerEyeMvt {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerEyeRegion(OptionManager& mgr,
                         const std::string& descrName =
                         "EyeRegion Simulation Viewer",
                         const std::string& tagName =
                         "SimulationViewerEyeRegion");

  //! Destructor
  virtual ~SimulationViewerEyeRegion();
  //@}

  // returns the XML id of the object
  uint findHitRegion(const Point2D<int> pt, uint frameNum);
  uint findHitRegion(const Point2D<int> pt) 
  {return findHitRegion(pt, itsFrameNumber);}

  std::string listAllRegionsHit(const Point2D<int> pt, uint frameNum); //uses default
  std::string listAllRegionsHit(const Point2D<int> pt)
  {return listAllRegionsHit(pt, itsFrameNumber);}
    
  double rankForeground(Object obj);
protected:
  //! Callback for every clock tick
  //SIMCALLBACK_DECLARE(SimulationViewerEyeRegion, SimEventClockTick);

  //! Callback for every time we should save our outputs
  //SIMCALLBACK_DECLARE(SimulationViewerEyeRegion, SimEventSaveOutput);

  //! Save our various results
  //virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Get the attention/eye/head trajectory image
  Image< PixRGB<byte> > getTraj(SimEventQueue& q);
  void extraSampleProcessing(const rutz::shared_ptr<EyeData> data);
  void writeROIOutput(Point2D<int> pos);
  void drawEye(const rutz::shared_ptr<EyeData> data, const uint trackerNum);   
  void drawFOA(const Point2D<int> target, const uint trackerNum); 
  void drawRegions(Object obj);
  void drawRegions() {drawRegions(getSceneObj(NULL_OBJ));}

  Object getSceneObj(uint objID, uint FrameNum); //uses default
  Object getSceneObj(uint objID) {return getSceneObj(objID,itsFrameNumber);}
  std::string getObjName(uint objID);

  std::string craftSVEMOutput(const std::string tfn, 
                              const rutz::shared_ptr<EyeData> data);
  std::string craftRegionOutput(const rutz::shared_ptr<EyeData> data); 

  void start1(); //!< get started
  void stop1(); //!< get stopped

  OModelParam<std::string> itsRegionOutFname;  //!< Our results file
  OModelParam<std::string> itsXMLFname;
  OModelParam<std::string> itsSelectedObjects;
  OModelParam<std::string> itsObjectDrawMode;
  OModelParam<std::string> itsHighlightMode;
  OModelParam<bool> itsPrependHeader; // Setup for when we reincorporate the two files together
  NModelParam<uint> itsLineThickness;
  NModelParam<double> itsHitTransparency;
  NModelParam<bool> itsRandomColoring;

  rutz::shared_ptr<TestImages> itsObjectsInfo;

  std::ofstream *itsRegionOutFile;  
  Image< PixRGB<byte> > itsRegions;
  Image< PixRGB<byte> > itsTargetsMask; // used for transparency/highlighting
 
  uint itsNumObjects;
  uint itsCurrRegionID;
  rutz::shared_ptr<EyeData> itsObjectEntry;
  SimTime itsObjectOnset;
  uint itsObjectFrameOnset;
  
  std::vector<PixRGB<byte> > itsRandColors;
  std::vector<std::string> itsObjectsNames;
  bool itsRegHeaderWritten;
private:
  
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
