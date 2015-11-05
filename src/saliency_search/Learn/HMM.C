/*!@file Learn/HMM.C Hidden Markov Models */

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

#include "Learn/HMM.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>

#include <cstdio>

// ######################################################################
template <class T>
HMM<T>::HMM(const std::vector<T>& states, const std::vector<T>& observations, const std::string& name) :
  itsName(name),
  itsStates(states),
  itsObservations(observations)
{
  itsStateTransitions.resize(states.size(), states.size());
  itsStateTransitions.clear(0);

  itsStateEmissions.resize(states.size(), observations.size());
  itsStateEmissions.clear(1.0F/(float)observations.size()); //default to 1/numObservations

  for(uint i=0; i<itsStates.size(); i++)
    itsStatesMap[itsStates[i]] = i;

  for(uint i=0; i<itsObservations.size(); i++)
    itsObservationsMap[itsObservations[i]] = i;

  itsCurrentPath.resize(states.size());


}

// ######################################################################
template <class T>
HMM<T>::~HMM()
{
}


// ######################################################################
template <class T>
void HMM<T>::setStateTransition(const T fromState, const T toState, double prob)
{
  size_t fromStateIdx = itsStatesMap[fromState];
  size_t toStateIdx = itsStatesMap[toState];

  itsStateTransitions.setVal(fromStateIdx, toStateIdx, prob);

}

// ######################################################################
template <class T>
void  HMM<T>::setStateEmission(const T state, const T emission, double prob)
{
  size_t stateIdx = itsStatesMap[state];
  size_t observationIdx = itsObservationsMap[emission];

  itsStateEmissions.setVal(stateIdx, observationIdx, prob);

}

// ######################################################################
template <class T>
void HMM<T>::setCurrentState(const T state, double prob)
{
  size_t stateIdx = itsStatesMap[state];

  itsCurrentPath[stateIdx].prob = prob;
  itsCurrentPath[stateIdx].path.clear();
  itsCurrentPath[stateIdx].path.push_back(stateIdx);
}

// ######################################################################
template <class T>
void HMM<T>::show()
{
  printf("HMM: %s", itsName.c_str());

  printf("Current State\n");
  for(size_t i=0; i<itsCurrentPath.size(); i++)
    printf("%zu\t\t", i);
  printf("\n");

  for(size_t i=0; i<itsCurrentPath.size(); i++)
    printf("%f\t", itsCurrentPath[i].prob);
  printf("\n\n");

  printf("State Transitions\n");
  printf("\t");
  for(int i=0; i<itsStateTransitions.getWidth(); i++)
    printf("%i\t\t", i);
  printf("\n");

  for(int i=0; i<itsStateTransitions.getWidth(); i++)
  {
    printf("%i\t", i);
    for(int j=0; j<itsStateTransitions.getHeight(); j++)
      printf("%f\t", itsStateTransitions.getVal(i,j));
    printf("\n");
  }
  printf("\n\n");

  printf("State Emissions\n");
  printf("\t");
  for(int i=0; i<itsStateEmissions.getWidth(); i++)
    printf("%i\t\t", i);
  printf("\n");

  for(int j=0; j<itsStateEmissions.getHeight(); j++)
  {
    printf("%i\t", j);
    for(int i=0; i<itsStateEmissions.getWidth(); i++)
      printf("%f\t", itsStateEmissions.getVal(i,j));
    printf("\n");
  }


}


// ######################################################################
template <class T>
std::vector<T> HMM<T>::getLikelyStates(const std::vector<T> observations,
    double& maxPathProb)
{
  if (observations.size() < 1)
    return std::vector<T>();

  for(uint i=0; i<observations.size(); i++)
    iteratePath(observations[i]);


  return getMaxPath(maxPathProb);
}


// ######################################################################
template <class T>
std::vector<T> HMM<T>::getMaxPath(double& maxPathProb)
{
  //Final sum/max
  double maxProb = 0;
  std::vector<size_t> maxPath;
  for(uint i=0; i<itsStates.size(); i++)
    if (itsCurrentPath[i].prob > maxProb)
    {
      maxProb = itsCurrentPath[i].prob;
      maxPath = itsCurrentPath[i].path;
    }

  std::vector<T> maxPathType; //the path in the vector type
  for(uint i=0; i<maxPath.size(); i++)
    maxPathType.push_back(itsStates[maxPath[i]]);

  maxPathProb = maxProb;

  return maxPathType;
}

// ######################################################################
template <class T>
void HMM<T>::iteratePath(const T observation)
{
  std::vector<Path> U(itsStates.size());

  for(uint nextState=0; nextState<itsStates.size(); nextState++)
  {
    Path nextPath;
    nextPath.prob = 0;

    //Find the maxProb for each state given the observation
    double maxProb = 0;
    for(uint sourceState=0; sourceState<itsStates.size(); sourceState++)
    {
      size_t observationIdx = itsObservationsMap[observation];

      double p = itsStateEmissions.getVal(sourceState,observationIdx) *
        itsStateTransitions.getVal(sourceState,nextState);

      double prob = itsCurrentPath[sourceState].prob * p;

      if (prob > maxProb)
      {
        nextPath.path = itsCurrentPath[sourceState].path;
        nextPath.path.push_back(nextState);
        nextPath.prob = prob;
        maxProb = prob;
      }
    }
    U[nextState] = nextPath;
  }
  itsCurrentPath = U; //Assign the max path so far and forget the reset

}

// ######################################################################
template <class T>
double HMM<T>::forward(const std::vector<T> observations)
{
  if (observations.size() < 1)
    return 0;

  double logProb = 0; 

  double sum;  // partial sum

  //restart forward
  itsSeqInfo.alpha.clear();

  // space needed for this forward iteration
  itsSeqInfo.alpha.resize(observations.size());
  itsSeqInfo.scale.resize(observations.size());

  for(size_t t = 0; t<observations.size(); t++)
  {
    size_t observationIdx = itsObservationsMap[observations[t] ];

    //Init
    if(t == 0){

      logProb = 0.0;
      itsSeqInfo.scale[0] = 0.0;
      itsSeqInfo.alpha[0].resize(itsStates.size());


      for(size_t state = 0; state < itsStates.size(); state++) {
        itsSeqInfo.alpha[0][state] = itsCurrentPath[state].prob * itsStateEmissions.getVal(state, observationIdx);
        itsSeqInfo.scale[0] += itsSeqInfo.alpha[0][state];
      }
      for(size_t state = 0; state < itsStates.size(); state++)
        itsSeqInfo.alpha[0][state] /= itsSeqInfo.scale[0];

    }else{
      itsSeqInfo.scale[t] = 0.0;
      itsSeqInfo.alpha[t].resize(itsStates.size());
      for(size_t j = 0; j < itsStates.size(); j++) {
        sum = 0.0;
        for(size_t i = 0; i < itsStates.size(); i++)
          sum += itsSeqInfo.alpha[t-1][i] *itsStateTransitions.getVal(i,j);

        itsSeqInfo.alpha[t][j] = sum * itsStateEmissions.getVal(j, observationIdx);
        itsSeqInfo.scale[t] += itsSeqInfo.alpha[t][j];
      }
      for(size_t j = 0; j < itsStates.size(); j++)
        itsSeqInfo.alpha[t][j] /= itsSeqInfo.scale[t];
    }
  }

  /*
     Compute sequence probability
     */
  for(size_t t = 0; t<observations.size(); t++)
    logProb += log(itsSeqInfo.scale[t]);


  return logProb;
}

template <class T>
double HMM<T>::backward(const std::vector<T> observations)
{
  if (observations.size() < 1)
    return 0;

  /*
     1. Initialization
     forward algorithm must finished before now
     */
  for(size_t i = 0; i < itsStates.size(); i++)
    itsSeqInfo.beta[observations.size()-1][i] = 1.0/itsSeqInfo.scale[observations.size()-1];

  /*
     2. Induction
     */
  for(size_t t = observations.size() - 2; /*t>=0 cannot be used for size_t*/; t--)
  {
    for(size_t i = 0; i < itsStates.size(); i++) {
      double sum = 0.0;
      for(size_t j = 0; j < itsStates.size(); j++)
      {
        size_t observationIdx = itsObservationsMap[observations[t+1] ];
        sum += itsStateTransitions.getVal(i,j) 
          * itsStateEmissions.getVal(j,observationIdx)
          * itsSeqInfo.beta[t+1][j];
      }
      itsSeqInfo.beta[t][i] = sum/itsSeqInfo.scale[t];
    }
    if(t==0)
      break;
  }
  
  return 0;
}

// ######################################################################
template <class T>
void HMM<T>::train(const std::vector<T> observations, size_t numIterations)
{
  if (observations.size() < 1)
    return;


  size_t loopCount = 0;

  double numeratorA, denominatorA;
  double numeratorB, denominatorB;

  //probability of sequence to hmm at previous estimation
  double logProb_prev;
  //difference of prob between iteration
  double delta;

  //allocate memory space, alpha and scale will be allocated in Forward()
  itsSeqInfo.beta.resize(observations.size());
  itsSeqInfo.gamma.resize(observations.size());
  itsSeqInfo.xi.resize(observations.size());

  for(size_t t=0; t<observations.size(); t++){
    itsSeqInfo.beta[t].resize(itsStates.size());
    itsSeqInfo.gamma[t].resize(itsStates.size());
    itsSeqInfo.xi[t].resize(itsStates.size());
    for(size_t i=0; i<itsStates.size(); i++)
      itsSeqInfo.xi[t][i].resize(itsStates.size());
  }


  //compute probs first time
  double logProb = forward(observations);
  backward(observations);
  computeGamma(observations);
  computeXi(observations);

  logProb_prev = logProb;

  do{
    // reestimate probility of state i in time t=0
    //for(size_t i = 0; i < itsStates.size(); i++)
    //  itsCurrentPath[i].prob = 0.0001 + 0.9999*itsSeqInfo.gamma[1][i];

    // reestimate transition matrix and prob of symbols to states
    for(size_t i = 0; i < itsStates.size(); i++) {
      denominatorA = 0.0;
      for(size_t t = 0; t < observations.size() - 1; t++)
        denominatorA += itsSeqInfo.gamma[t][i];

      for(size_t j = 0; j < itsStates.size(); j++) {
        numeratorA = 0.0;
        for(size_t t = 0; t < observations.size() - 1; t++)
          numeratorA += itsSeqInfo.xi[t][i][j];
        itsStateTransitions.setVal(i, j, 0.0001 + 0.9999*numeratorA/denominatorA);
      }

      denominatorB = denominatorA + itsSeqInfo.gamma[observations.size()-1][i];
      for(size_t k = 0; k < itsObservations.size(); k++) {
        numeratorB = 0.0;
        for(size_t t = 0; t < observations.size(); t++) {
          size_t observationIdx = itsObservationsMap[observations[t] ];
          if(observationIdx == k)
            numeratorB += itsSeqInfo.gamma[t][i];
        }
        itsStateEmissions.setVal(i,k, 0.0001 + 0.9999*numeratorB/denominatorB);
      }
    }

    // compute probs by new model
    logProb = forward(observations);
    backward(observations);
    computeGamma(observations);
    computeXi(observations);

    // delta prob between old and estimated model
    delta = logProb - logProb_prev;
    logProb_prev = logProb;
    loopCount++;
  } while(loopCount < numIterations);//loop utill log probability converged.

  //return loopCount;
}

// ######################################################################
template <class T>
void HMM<T>::train(const std::vector< std::vector<T> > observations, size_t numIterations)
{

  if (observations.size() < 1)
    return;

  //Run the single ibservation trainer
  if (observations.size() == 1)
  {
    train(observations[0], numIterations);
    return;
  }

  size_t loopCount = 0;

  //FIXME (observations[0].size() may not equale observations[1].size()
  itsSeqInfo.beta.resize(observations[0].size());
  itsSeqInfo.gamma.resize(observations[0].size());
  itsSeqInfo.xi.resize(observations[0].size());

  for(size_t t=0; t<observations[0].size(); t++){
    itsSeqInfo.beta[t].resize(itsStates.size());
    itsSeqInfo.gamma[t].resize(itsStates.size());
    itsSeqInfo.xi[t].resize(itsStates.size());
    for(size_t i=0; i<itsStates.size(); i++)
      itsSeqInfo.xi[t][i].resize(itsStates.size());
  }

  double numeratorA, numeratorA_partial, denominatorA, denominatorA_partial;
  double numeratorB, numeratorB_partial, denominatorB, denominatorB_partial;

  double logProbSum_prev = 0.0;
  double logProbSum = 0.0;
  double delta; //difference of prob between iteration

  /*
     Initialization
     */
  std::vector<SeqInfo> seqsInfo(observations.size());

  for(size_t seq = 0; seq < observations.size(); seq++)
  {
    double logProb = forward(observations[seq]);
    backward(observations[seq]);
    computeGamma(observations[seq]);
    computeXi(observations[seq]);
    logProbSum_prev += logProb;

    seqsInfo[seq].prob = logProb;
    seqsInfo[seq].gamma = itsSeqInfo.gamma;
    seqsInfo[seq].xi = itsSeqInfo.xi;

  }

  //Iteration
  do{
    // reestimate probility of state i in time t=0
    //for(i = 0; i < hmm.N; i++)
    //    hmm.pi[i] = 0.001 + 0.999*curSeq->gamma[1][i];

    // reestimate transition matrix and prob of symbols to states
    for(size_t i = 0; i < itsStates.size(); i++) {
      denominatorA = 0.0;
      denominatorB = 0.0;

      for(size_t seq = 0; seq < observations.size(); seq++)
      {
        double logProb = seqsInfo[seq].prob;
        std::vector< std::vector<double> >& gamma = seqsInfo[seq].gamma;
        denominatorA_partial = 0.0;
        for(size_t t = 0; t < observations[seq].size() - 1; t++)
          denominatorA_partial += gamma[t][i];
        denominatorB_partial = denominatorA_partial + gamma[observations[seq].size()-1][i];
        denominatorA += denominatorA_partial/exp(logProb);
        denominatorB += denominatorB_partial/exp(logProb);
      }

      for(size_t j = 0; j < itsStates.size(); j++) {
        numeratorA = 0.0;
        for(size_t seq = 0; seq < observations.size(); seq++)
        {
          std::vector< std::vector< std::vector<double> > >& xi = seqsInfo[seq].xi;
          numeratorA_partial = 0.0;
          for(size_t t = 0; t < observations[seq].size() - 1; t++)
            numeratorA_partial += xi[t][i][j];
          numeratorA += numeratorA_partial/exp(seqsInfo[seq].prob);
        }
        itsStateTransitions.setVal(i, j, 0.0001 + 0.9999*numeratorA/denominatorA);
      }

      for(size_t k = 0; k < itsObservations.size(); k++) {
        numeratorB = 0.0;
        for(size_t seq = 0; seq < observations.size(); seq++)
        {
          std::vector< std::vector<double> >& gamma = seqsInfo[seq].gamma;
          numeratorB_partial = 0.0;
          for(size_t t = 0; t < observations[seq].size(); t++)
          {
            size_t observationIdx = itsObservationsMap[observations[seq][t] ];
            if(observationIdx == k)
              numeratorB_partial += gamma[t][i];
          }
          numeratorB += numeratorB_partial/exp(seqsInfo[seq].prob);
        }
        itsStateEmissions.setVal(i,k, 0.0001 + 0.9999*numeratorB/denominatorB);
      }
    }

    // compute probs by new model
    for(size_t seq = 0; seq < observations.size(); seq++)
    {
      double logProb = forward(observations[seq]);
      backward(observations[seq]);
      computeGamma(observations[seq]);
      computeXi(observations[seq]);
      logProbSum += logProb;

      seqsInfo[seq].prob = logProb;
      seqsInfo[seq].gamma = itsSeqInfo.gamma;
      seqsInfo[seq].xi = itsSeqInfo.xi;
    }

    // delta prob between old and estimated model
    delta = logProbSum - logProbSum_prev;
    logProbSum_prev = logProbSum;
    loopCount++;
  }while(loopCount < numIterations);//loop utill log probability converged.

}

// ######################################################################
template <class T>
void HMM<T>::computeGamma(const std::vector<T> observations)
{

  for(size_t t = 0; t < observations.size(); t++) {
    double denominator = 0.0;
    for(size_t j = 0; j < itsStates.size(); j++) {
      itsSeqInfo.gamma[t][j] = itsSeqInfo.alpha[t][j] * itsSeqInfo.beta[t][j];
      denominator += itsSeqInfo.gamma[t][j];
    }

    for(size_t i = 0; i < itsStates.size(); i++)
      itsSeqInfo.gamma[t][i] = itsSeqInfo.gamma[t][i]/denominator;
  }

}

// ######################################################################
template <class T>
void HMM<T>::computeXi(const std::vector<T> observations)
{
  for(size_t t = 0; t < observations.size()-1; t++) {
    double denominator = 0.0;
    for(size_t i = 0; i < itsStates.size(); i++)
      for(size_t j = 0; j < itsStates.size(); j++) {
        size_t observationIdx = itsObservationsMap[observations[t+1] ];
        itsSeqInfo.xi[t][i][j] = itsSeqInfo.alpha[t][i] * itsSeqInfo.beta[t+1][j] *
          itsStateTransitions.getVal(i,j) * itsStateEmissions.getVal(j, observationIdx);
        denominator += itsSeqInfo.xi[t][i][j];
      }

    for(size_t i = 0; i < itsStates.size(); i++)
      for(size_t j = 0; j < itsStates.size(); j++)
        itsSeqInfo.xi[t][i][j] /= denominator;
  }
  
}



template class HMM<std::string>;
template class HMM<uint>;

  
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
