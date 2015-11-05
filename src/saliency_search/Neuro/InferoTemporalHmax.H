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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalHmax.H $
// $Id: InferoTemporalHmax.H 14157 2010-10-22 00:54:14Z dparks $
//

#ifndef INFEROTEMPORALHMAX_H_DEFINED
#define INFEROTEMPORALHMAX_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Neuro/InferoTemporal.H"
#include "HMAX/HmaxFL.H"
#include "Learn/SVMObject.H"
#include "Neuro/SVMClassifierModule.H"
class Brain;

namespace nub { template <class T> class ref; }


// ######################################################################
//! Inferotemporal interface using HMAX with feature learning
/*! Model of the Inferior Temporal Cortex utilizing HMAX object recognition */
class InferoTemporalHmax : public InferoTemporal
{

public:
  //! Constructor
  InferoTemporalHmax(OptionManager& mgr,
                    const std::string& descrName = "Infero Temporal HMAX",
                    const std::string& tagName = "InferoTemporalHmax");

  virtual std::vector<float> calculateFeatureVector(Image<float> img);
  virtual SVMObject determineLabel(Image<float> objImg,int id=-1, std::string name="");
  void writeOutFeatureVector(std::vector<float> featureVector, int id);

  //! Destructor
  virtual ~InferoTemporalHmax();

protected:
  //! Object recognition
  virtual void attentionShift(SimEventQueue& q, const Point2D<int>& location);
  virtual float** _createFeatureVector();
  virtual void _freeFeatureVector(float **c2Res);
  virtual std::vector<float> _convertFeatureVector(float **c2Res);

protected:
  OModelParam<std::string> itsHMAXStoredPatchesDir;   //!< dir of stored patches
  OModelParam<std::string> itsHMAXFeatureVectorFileName;    //!< File to output HMAX feature vectors
  OModelParam<bool>        itsHMAXDisableClassifier; //!< whether to disable support for the internal classifier
  InputFrame itsInputFrame;
  HmaxFL itsHmax;
  rutz::shared_ptr<SVMClassifierModule> itsClassifier;

  //! (re-)initialization - load object database
  virtual void start1();

  //! destruction - save object database
  virtual void stop1();

};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

