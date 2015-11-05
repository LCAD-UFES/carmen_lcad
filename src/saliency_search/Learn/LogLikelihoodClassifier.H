/*!@file Learn/LogLikelikhoodClassifier.H Log Likelihood Classifier for Histograms module */
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
// Primary maintainer for this file: Dan Parks <danielfp@usc.edu>
// $HeadURL$
// $Id$
//

#ifndef LOGLIKELIHOODCLASSIFIER_H_DEFINED
#define LOGLIKELIHOODCLASSIFIER_H_DEFINED

#include <cstdlib>
#include <deque>
#include <map>



class LogLikelihoodClassifier
{

public:
  LogLikelihoodClassifier(int k=5);

  // Map of model histograms (multiple per model id)
  typedef std::map<int, std::vector<std::vector<float> > > MapModelVector;

  //! Calculate log likelihood probability for this sample and model
  float calcLogLikelihood(const std::vector<float>& sample, const std::vector<float>& model);

  //! Add a model exemplar for this id (ok to have multiple exemplars per class)
  void addModel(std::vector<float> hist, int id);
  
  //! Set the models that will be used for this classifier
  void setModels(MapModelVector models);

  //! Return the models that were used for this classifier
  MapModelVector getModels();

  //! Predict using classifier
  int predict(const std::vector<float>& hist);

  //! Predict using classifier and return the full PDF across classes
  std::map<int,double> predictPDF(const std::vector<float>& hist);

  //! Perform k-nearest neighbor vote that does a short circuited calculation using the best likelihoods to calculate the full PDF
  void nearestNeighborVotePDF(const std::map<int,std::list<float> >& logLikelihood, std::map<int,double>& pdf);
  
  //! Get number of models
  uint getNumModels();

protected:
  //! Store a list of model histograms for each model id
  MapModelVector itsModels;
  uint itsHistLength;
  uint itsK;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

