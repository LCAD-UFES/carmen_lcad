/*!@file Learn/SVMClassifier.H Support Vector Machine Classifier module */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SVMClassifier.H $
// $Id: SVMClassifier.H 14581 2011-03-08 07:18:09Z dparks $
//

#ifndef SVMCLASSIFIER_H_DEFINED
#define SVMCLASSIFIER_H_DEFINED

#include <map>

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"

#include "svm.h"

namespace nub { template <class T> class ref; }

// ######################################################################
//! SVM Classifier Class
class SVMClassifier
{

public:
  //! Constructor
  SVMClassifier(float gamma=0.00078125, int C=32);

  //! Destructor
  ~SVMClassifier();

  //! Train
  void train(std::string outputFileName, int id, std::vector<float> &feature);
  void train(std::string outputFileName, int id, float *&feature, unsigned int fdim);
  void train(std::string outputFilename, int id, float **&feature, unsigned int fdim1, unsigned int fdim2);

  //! Train the SVM using the data in trainingData, with the ground truth data in dataClasses
  //    trainingData is a matrix which contains column vectors of data points, where each row represents
  //      a dimension of your data
  //    dataClasses is a vector which defines the class type of each data point in the trainingData
  //     (Your dataClasses vector should have as many elements as there are columns in your trainingData)
  void train(Image<double> trainingData, std::vector<double> dataClasses);
  void train(std::vector<std::vector<float> > trainingDta, std::vector<float> dataClasses);

  //! Predict the class of a data point
  //    dataPoint should be a column vector with each row representing a dimension of the data
  std::map<int,double> predictPDF(const svm_node* dataPointNodes);
  std::map<int,double> predictPDF(std::vector<float> &feature);

  //! Predict
  double predict(Image<double> dataPoint, double *probability=NULL);
  double predict(std::vector<float> &feature, double *probability=NULL);
  double predict(float * &feature, unsigned int fdim, double *probability=NULL);
  double predict(float **&feature, unsigned int fdim1, unsigned int fdim2, double *probability=NULL);

  void readModel(std::string modelFileName); //! Read model file into memory
  void writeModel(std::string modelFileName); //! Write model file into memory
  void readRange(std::string rangeFileName); //! Read a range file to scale the feature vector, if needed
  float rescaleValue(float value, unsigned int index); //! If range is enabled, rescale the value to

protected:
  int _getBestLabel(std::map<int,double> pdf, double *probability);
  double _predict(struct svm_node *node, unsigned int fdim, double * probability);
  void _train(svm_problem& trainingProblem);

  struct svm_parameter itsSVMParams;
  struct svm_model*    itsSVMModel;
  bool                 itsSVMRangeEnabled;
  std::vector<double>  itsSVMFeatureRangeMax;
  std::vector<double>  itsSVMFeatureRangeMin;
  double               itsSVMFeatureRangeUpper;
  double               itsSVMFeatureRangeLower;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

