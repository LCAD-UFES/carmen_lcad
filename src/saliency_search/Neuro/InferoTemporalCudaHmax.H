/*!@file Neuro/InferoTemporalCudaHmax.H Object recognition module using CUDA */

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
// $HeadURL: svn://dparks@isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalHmax.H $
// $Id: InferoTemporalHmax.H 13498 2010-05-28 02:53:57Z dparks $
//

#ifndef INFEROTEMPORALCUDAHMAX_H_DEFINED
#define INFEROTEMPORALCUDAHMAX_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Neuro/InferoTemporalHmax.H"
#include "HMAX/HmaxFL.H"
#include "Learn/SVMObject.H"
#include "Learn/SVMClassifier.H"
#include "Learn/SVMObjectDB.H"
#include "CUDA/CudaHmaxCBCL.H"

class Brain;

namespace nub { template <class T> class ref; }

// ######################################################################
//! Inferotemporal interface using HMAX with feature learning
/*! Model of the Inferior Temporal Cortex utilizing HMAX object recognition */
class InferoTemporalCudaHmax : public InferoTemporalHmax
{

public:
  //! Constructor
  InferoTemporalCudaHmax(OptionManager& mgr,
                    const std::string& descrName = "Infero Temporal CUDA HMAX",
                    const std::string& tagName = "InferoTemporalCudaHmax");

  //! Destructor
  virtual ~InferoTemporalCudaHmax();

  virtual std::vector<float> calculateFeatureVector(Image<float> objImg);

protected:
  virtual std::vector<float> _convertFeatureVector(float *c2Res, int numC2);
  virtual void _freeFeatureVector(float *c2Res);

  // Override these
  virtual void _freeFeatureVector(float **c2Res);
  std::vector<float> _convertFeatureVector(float **c2Res);
  // Base class variable should not be used
  // OModelParam<std::string> itsHMAXStoredPatchesDir;   //!< dir of stored patches
  // HmaxFL hmax;
  OModelParam<std::string> itsCUDAHMAXStoredC0PatchesFile;   //!< file of stored C0 patches
  OModelParam<std::string> itsCUDAHMAXStoredC1PatchesFile;   //!< file of stored C1 patches
  CudaHmaxCBCL hmax;                                     //!< CBCL CUDA Hmax class

  //! (re-)initialization - load object database
  virtual void start1();


};

#endif
