/*!@file Gist/trainUtils.H training utility functions (not limited to FFN) */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/trainUtils.H $
// $Id: trainUtils.H 12395 2009-12-25 02:34:13Z siagian $
//

#ifndef GIST_TRAINUTILS_H_DEFINED
#define GIST_TRAINUTILS_H_DEFINED

#include "Image/Image.H"
#include <string>

template <class T> class Image;

#define TUTILS_RW_RANGE 1.0    // range of initial random weights [-.5 ... .5]

// ######################################################################
// utilities for training feed forward network (FFN)

// ######################################################################
// to store information from the training file
class FFNtrainInfo
{
public:
  //! Construct a FFN training params info
  //! if blank, need to call reset later on
  FFNtrainInfo(std::string fName = std::string(""));

  //! Destructor
  virtual ~FFNtrainInfo();

  //! reset the Neural Network classifier parameter
  bool reset(std::string fName);

  std::string  trainFolder;      //!< where the training data is
  std::string  testFolder;       //!< where the testing data is
  uint         nOutput;          //!< the number of output dimension
  bool         isPCA;            //!< has a dimension reduction step
  std::string  evecFname;        //!< the dimension reduction
  uint         oriFeatSize;      //!< the original number of features
  uint         redFeatSize;      //!< the reduced number of features
  uint         h1size;           //!< the number of hidden nodes in layer 1
  uint         h2size;           //!< the number of hidden nodes in layer 2
  std::string  h1Name;           //!< the weight file for hidden layer 1
  std::string  h2Name;           //!< the weight file for hidden layer 2
  std::string  oName;            //!< the weight file for output layer
  float        learnRate;        //!< the learning rate
  std::string  trainSampleFile;  //!< the training sample file list
  std::string  testSampleFile;   //!< the testing sample file list
};

// ######################################################################
// utilities for applying PCA/ICA dimension reduction

//! Setup the PCA/ICA unmixing matrix
Image<double> setupPcaIcaMatrix(std::string inW, int oriSize, int redSize);

//! Get the PCA/ICA reduced feature vector
//! in a form of an image
Image<float> getPcaIcaFeatImage(Image<double> res, int w, int h, int s);

#endif // GIST_TRAINUTILS_H_DEFINED
