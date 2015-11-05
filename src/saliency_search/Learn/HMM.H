/*!@file Learn/HMM.H Hidden Markov Models */


// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: $
// $Id: $
//

#ifndef LEARN_HMM_H_DEFINED
#define LEARN_HMM_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/ColorOps.H"
#include "Util/Types.H" // for uint
#include <vector>
#include <string>
#include <map>

template <class T>
class HMM
{
public:

  struct Path
  {
    double prob;
    std::vector<size_t> path;

    Path() :
      prob(0)
    {}
  };

  struct SeqInfo
  {
    std::vector<double>             scale;
    std::vector< std::vector<double> > alpha;
    std::vector< std::vector<double> > beta;
    std::vector< std::vector<double> >        gamma;
    std::vector< std::vector< std::vector<double> > > xi;

    double prob;
  };
  

  HMM() {}

  HMM(const std::vector<T>& states,
      const std::vector<T>& observations,
      const std::string& name = "");

  //! Destructor
  ~HMM();

  //! Set the state transitions
  void setStateTransition(const T fromState, const T toState, double prob);

  //! The state emission probability
  void setStateEmission(const T state, const T emission, double prob);

  //! Set the current prob of which state we are in
  void setCurrentState(const T state, double prob);
 
  //! Find the most likely sequence of hidden states given the observations and
  //! return the probability of this path.
  // This alg is using the Viterbi algorithm to find the most probable path
  // and the forward alg to find the prob of this path
  std::vector<T> getLikelyStates(const std::vector<T> observations, 
      double& maxPathProb = double()); 

  //! Iterate through the path given one observation
  void iteratePath(const T observation);

  //! Forward alg ie. compute P(Obj | currentHMMModel)
  double forward(const std::vector<T> observations);

  //! Backward alg
  double backward(const std::vector<T> observations);
  

  //! Train the model by changing the state transitions and 
  //! state emittions for the given observation
  void train(const std::vector< std::vector<T> > observations, size_t numIterations);

  //! Train the model by changing the state transitions and 
  //! state emittions for the given observation
  //! batch update per Fundamentals of Speech Recognition -
  //! by Lawrence Rabiner , Biing-Hwang Juang
  void train(const std::vector<T> observations, size_t numIterations);
 
  void computeXi(const std::vector<T> observations);
  void computeGamma(const std::vector<T> observations);
  

  //! Get the max path so far
  std::vector<T> getMaxPath(double& maxPathProb);

  //! Show the internal states and observations of the HMM
  void show();

  //! Get the hmm name
  std::string getName() { return itsName; }
  
 
private:
  std::string  itsName;

  std::vector<T> itsStates;
  std::vector<T> itsObservations;

  Image<double> itsStateTransitions;
  Image<double> itsStateEmissions;

  //Map to find the index in the matrix
  std::map<T,size_t> itsStatesMap;
  std::map<T,size_t> itsObservationsMap;

  std::vector<Path>               itsCurrentPath;
  
  SeqInfo       itsSeqInfo;
  
  
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_BAYES_H_DEFINED
