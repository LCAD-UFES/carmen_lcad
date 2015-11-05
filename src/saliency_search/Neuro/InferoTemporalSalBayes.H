/*!@file Neuro/InferoTemporalSalBayes.H Object recognition module with SalBayes */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalSalBayes.H $
// $Id: InferoTemporalSalBayes.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef INFEROTEMPORALSALBAYES_H_DEFINED
#define INFEROTEMPORALSALBAYES_H_DEFINED

#include "Channels/InputFrame.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/LevelSpec.H"
#include "Image/Pixels.H"
#include "Image/Image.H"
#include "Simulation/SimModule.H"
#include "Neuro/InferoTemporal.H"
#include "Learn/Bayes.H"
#include "Media/TestImages.H"
#include "SIFT/VisualObjectMatchAlgo.H"
#include "SIFT/VisualObjectDB.H"

class Brain;

// ######################################################################
//! InferoTemporalSalBayes
/*! Model of the Inferior Temporal Cortex utilizing the SalBayes method
    recognition. */
class InferoTemporalSalBayes : public InferoTemporal
{
public:
  //! Constructor
  InferoTemporalSalBayes(OptionManager& mgr,
                    const std::string& descrName = "Infero Temporal Std",
                    const std::string& tagName = "InferoTemporalSalBayes");

  //! Destructor
  virtual ~InferoTemporalSalBayes();

  std::vector<double> buildRawDV(SimEventQueue& q, const Point2D<int>& foveaLoc);
  std::string getObjNameAtLoc(const std::vector<TestImages::ObjData> &objects, const Point2D<int>& loc);
  void predict(std::vector<double> &fv, SimEventQueue& q);
  void predictWithSift(std::vector<double> &fv,
      rutz::shared_ptr<VisualObject> &vob,
      SimEventQueue& q);

protected:
  OModelParam<LevelSpec> itsLevelSpec;    //! Our levelspec
  OModelParam<Dims> itsFoveaSize;    //! The size of our fovea
  OModelParam<std::string> itsITCMode;
  OModelParam<std::string> itsBayesNetFilename; //!< the file to save/read the gains from
  OModelParam<int> itsUseSift; //!< use sift after basic recognition
  OModelParam<std::string> itsSiftObjectDBFile; //!< obj knowledgebase file
  OModelParam<bool> itsUseMaxNMatches; //!< use the max num of matches


  //! Object recognition at attention point
  virtual void attentionShift(SimEventQueue& q, const Point2D<int>& location);

  //! (re-)initialization - load object database
  virtual void start1();

  //! destruction - save object database
  virtual void stop1();

private:
  bool                             itsHas3Dinfo;
  float                            its3DSalVal;
  std::vector<int>                 its3DSalLocation;
  rutz::shared_ptr<Bayes>          itsBayesNet;
  rutz::shared_ptr<VisualObjectDB> itsSiftObjectDB;



};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
