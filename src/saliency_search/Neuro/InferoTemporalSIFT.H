/*!@file Neuro/InferoTemporalHmax.H Object recognition module */

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
// Primary maintainer for this file:Sophie Marat
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalSIFT.H $
// $Id: InferoTemporalHmax.H 14244 2010-11-17 02:56:14Z sophie $
//

#ifndef INFEROTEMPORALSIFT_H_DEFINED
#define INFEROTEMPORALSIFT_H_DEFINED


#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Neuro/InferoTemporal.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"
#include "Channels/InputFrame.H"
#include "Image/Pixels.H"
#include "Image/Image.H"
#include "Media/TestImages.H"

#include "Channels/RawVisualCortex.H"
#include"Component/RawGistEstimatorStd.H"

#include "Neuro/SVMClassifierModule.H"

class Brain;

namespace nub { template <class T> class ref; }


// ######################################################################
//! Inferotemporal interface using SIFT with feature learning
/*! Model of the Inferior Temporal Cortex utilizing SIFT object recognition */
class InferoTemporalSIFT : public InferoTemporal
{

public:
  //! Constructor
  InferoTemporalSIFT(OptionManager& mgr,
                     const std::string& descrName = "Infero Temporal SIFT",
                     const std:: string& tagName = "InferoTemporalSIFT");



  //! Destructor
  virtual ~InferoTemporalSIFT();

  std::string getObjNameAtLoc(const std::vector<TestImages::ObjData> &objects, const Point2D<int>& loc);
  

 
protected:
  //! Object recognition
  virtual void attentionShift(SimEventQueue& q, const Point2D<int>& location);

protected:
  OModelParam<std::string> itsSIFTStoredDatabase; //!< dir of stored SIFT Database
  OModelParam<std::string> itsITCMode; //!< option for training or testing mode
  OModelParam<std::string> itsPathMatch; //!< Path of the objects to consider for matching
  OModelParam<bool> itsCoarseReco; //!< Use the gist to do a coares pre-recognition

  OModelParam<std::string> itsTrainSVM; //!< file where the training exemple for the SVM will be saved 
  OModelParam<std::string> itsSVMId; //!< The Id of the object to train the SVM
  OModelParam<std::string> itsSVMClass; //!< The Class of the object to train the SVM - Used for completing the ID:Class table
  OModelParam<std::string> itsSVMModel; //!< file where the training exemple for the SVM will be saved The SVM model to use
  OModelParam<std::string> itsSVMRange; //!< file where the training exemple for the SVM will be saved The range to rescale data before SVM
  OModelParam<std::string> itsNameObj; //!< Name on the object that is processed to train SVM ///
  OModelParam<std::string> itsRecoSave; //!< file where the result of the recognition will be saved
  OModelParam<std::string> itsTable; //!< file where the table of the class and their Id is saved


  rutz::shared_ptr<VisualObjectDB> itsObjectDB; //!< our object database
  rutz::shared_ptr<VisualObjectDB> itsnewObjectDB; //!< to adapt the database to select only object to match within the given path
  rutz::shared_ptr<std::map<double,int> > itsPDFGist; //!< proba density of the recognition using gist

  //GIST//
  nub::ref<RawVisualCortex> itsVisualCortex;
  //rutz::shared_ptr<RawVisualCortex> itsVisualCortex;  // raw visual cortex use to compute coarse recognition with gist
  nub::ref<RawGistEstimatorStd> itsGistEstim;


  //nub::ref<SVMClassifierModule> itsClassifier;

  //! (re-)initialization - load object database
  virtual void start1();

  //! destruction - save object database
  virtual void stop1();

  void getObjDBToMatch(const char *dir);

  void computeGist(Image<PixRGB<byte> >);
  //Image<float> computeGist(Image<PixRGB<byte> >);
  
  void gistSelect(std::map<double, int>::reverse_iterator iteratorPDFGist);

};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

