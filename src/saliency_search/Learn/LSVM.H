/*!@file Learn/LSVM.H Latent Support Vector Machine Classifier module */

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
// $HeadURL: $
// $Id: $
//

#ifndef LSVM_H_DEFINED
#define LSVM_H_DEFINED

#include <map>

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "FeatureMatching/DPM.H"

// ######################################################################
//! LSVM Classifier Class
class LSVM
{
  struct LabeledData
  {
    int classId;
    std::vector<double> features;
  };

  struct Block
  {
    std::vector<double> weight;
    std::vector<double> features;
  };

  struct LossInfo
  {
    double loss; //the loss values
    double pos; // loss on positive examples
    double neg; // loss on negative examples
    double reg; // regularization term's value
  };
    
public:
  //! Constructor
  LSVM(float gamma=0.00078125, int C=32);

  //! Destructor
  ~LSVM();

  //! Train
  void train(std::string outputFileName, int id, std::vector<float> &feature);

  //! Get the score for a particular data
  double getScore(const LabeledData& data, Model& model, std::vector<double>& weight);
protected:
  //Calculate the weight using gradient decent
  void gradientDescent(double C, double J, data X,
      double **w, double **lb);
  //Calculate the hinge loss
  LossInfo computeLoss(double C, double J, data X, std::vector<double>& weight); 
private:
  long itsNumIter; //Number of iterations
  long itsMinIter;  //Min iterations before stoping
  double itsDeltaStop; // convergence threshold
  long itsStopCount; // number of times in a row the convergence threshold must be reached before stopping

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

