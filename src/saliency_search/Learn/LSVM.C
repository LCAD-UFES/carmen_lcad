/*!@file Learn/LSVM.C Latent Support Vector Machine Classifier module */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/LSVM.C $
// $Id: LSVM.C 14581 2011-03-08 07:18:09Z dparks $
//

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <map>

#include "svm.h"
#include "LSVM.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"


LSVM::LSVM(float gamma, int C)
{
}

LSVM::~LSVM()
{
}

void LSVM::train(const std::vector<LabeledData>& examples)
{
  seed_rand();
  int count;
  DPMModel model;

  LINFO("Sorting Examples");
  std::vector<LabeledData> sortedExamples = std::sort(examples);

  // find unique examples

  // collapse examples
  // merge examples with identical labels
  //collapse(&X, sorted, num_unique);

  // initial model

  // lower bounds

  // train
  LINFO("Training");
  gradientDescent(C, J, X, w, lb, logdir, logtag);

  // score examples
  LINFO("Scoring\n");
  std::vector<double> score;
  for(size_t i=0; i<examples.size(); i++)
    score.push_back(getScore(model, w, examples[i]));
    
  // compute loss and write it to a file
  LossInfo lossInfo = computeLoss(C, J, model, w);
}



double LSVM::getScore(const LabeledData& data, Model& model, std::vector<double>& weight) {

  double val = 0.0;
  for(uint i=0; i<data.features.size(); i++)
    val += weight[i]*data.features[i];

  return val;
}

LSVM::LossInfo LSVM::computeLoss(double C, double J, data X, std::vector<double>& weight) {

  LossInfo lossInfo;

  double loss = 0;
  if (itsFullL2) // compute ||w||^2
  {
    for (int j = 0; j < X.numblocks; j++) {
      for (int k = 0; k < X.blocksizes[j]; k++) {
        loss += w[j][k] * w[j][k] * X.regmult[j];
      }
    }
  } else {
    // compute max norm^2 component
    for (int c = 0; c < X.numcomponents; c++) {
      double val = 0;
      for (int i = 0; i < X.componentsizes[c]; i++) {
        int b = X.componentblocks[c][i];
        double blockval = 0;
        for (int k = 0; k < X.blocksizes[b]; k++)
          blockval += w[b][k] * w[b][k] * X.regmult[b];
        val += blockval;
      }
      if (val > loss)
        loss = val;
    }
  }
  loss *= 0.5;

  // record the regularization term
  lossInfo.reg = loss;

  // compute loss from the training data
  for (int l = 0; l < 2; l++) {
    // which label subset to look at: -1 or 1
    int subset = (l*2)-1;
    double subsetloss = 0.0;
    for (int i = 0; i < X.num; i++) {
      collapsed x = X.x[i];

      // only consider examples in the target subset
      char *ptr = x.seq[0];
      if (LABEL(ptr) != subset)
        continue;

      // compute max over latent placements
      int M = -1;
      double V = -INFINITY;
      for (int m = 0; m < x.num; m++) {
        double val = ex_score(x.seq[m], X, w);
        if (val > V) {
          M = m;
          V = val;
        }
      }

      // compute loss on max
      ptr = x.seq[M];
      int label = LABEL(ptr);
      double mult = C * (label == 1 ? J : 1);
      subsetloss += mult * max(0.0, 1.0-label*V);
    }
    loss += subsetloss;
    if (l==0)
      lossInfo.pos = subsetloss;
    else
      lossInfo.neg = subsetloss;
  }

  lossInfo.loss = loss;

}

void LSVM::gradientDescent(double C, double J, data X,
    double **w, double **lb) {

  int num = X.num;
  
  // state for random permutations

  // state for small cache
  double prev_loss = 1E9;

  bool converged = false;
  int stop_count = 0;
  int t = 0;
  while (t < itsNumIter && !converged) {
    // pick random permutation
    for (int i = 0; i < num; i++)
      perm[i] = i;
    for (int swapi = 0; swapi < num; swapi++) {
      int swapj = (int)(drand48()*(num-swapi)) + swapi;
      int tmp = perm[swapi];
      perm[swapi] = perm[swapj];
      perm[swapj] = tmp;
    }

    // count number of examples in the small cache
    int cnum = 0;
    for (int i = 0; i < num; i++)
      if (W[i] <= INCACHE)
        cnum++;

    int numupdated = 0;
    for (int swapi = 0; swapi < num; swapi++) {
      // select example
      int i = perm[swapi];

      // skip if example is not in small cache
      if (W[i] > INCACHE) {
        W[i]--;
        continue;
      }

      collapsed x = X.x[i];

      // learning rate
      double T = min(itsNumIterations/2.0, t + 10000.0);
      double rateX = cnum * C / T;

      t++;
      //Evey 100000 itr show loss/stats and determin if we need to stop
      if (t % 100000 == 0) {
        //Compute the hinge loss
        LossInfo lossInfo = compute_loss(C, J, X, w);
        double delta = 1.0 - (fabs(prev_loss - loss) / loss);

        LINFO("t=%i loss=%f delta=%f", t, lossInfo.loss, delta);

        //Do we need to stop
        if (delta >= itsDeltaStop && t >= itsMinIter) {
          stop_count++;
          if (stop_count > itsStopCount)
            converged = true;
        } else if (stop_count > 0) {
          stop_count = 0;
        }
        prev_loss = lossInfo.loss;
        LINFO("%7.2f%% of max # iterations "
            "(delta = %.5f; stop count = %d)", 
            100*double(t)/double(itsNumIter),
            max(delta, 0.0), 
            itsStopCount - stop_count + 1);
        if (converged)
          break;
      }
      
      // compute max over latent placements
      int M = -1;
      double V = -INFINITY;
      for (int m = 0; m < x.num; m++) {
        double val = ex_score(x.seq[m], X, w);
        if (val > V) {
          M = m;
          V = val;
        }
      }
      
      //Compute the weights
      char *ptr = x.seq[M];
      int label = LABEL(ptr);
      if (label * V < 1.0) {
        numupdated++;
        W[i] = 0;
        float *data = EX_DATA(ptr);
        int blocks = NUM_NONZERO(ptr);
        for (int j = 0; j < blocks; j++) {
          int b = BLOCK_IDX(data);
          double mult = (label > 0 ? J : -1) * rateX * X.learnmult[b];      
          data++;
          for (int k = 0; k < X.blocksizes[b]; k++)
            w[b][k] += mult * data[k];
          data += X.blocksizes[b];
        }
      } else {
        if (W[i] == INCACHE)
          W[i] = MINWAIT + (int)(drand48()*50);
        else
          W[i]++;
      }

      // periodically regularize the model
      if (t % REGFREQ == 0) {
        // apply lowerbounds
        for (int j = 0; j < X.numblocks; j++)
          for (int k = 0; k < X.blocksizes[j]; k++)
            w[j][k] = max(w[j][k], lb[j][k]);

        double rateR = 1.0 / T;

        if (itsFullL2)
        {
          // update model
          for (int j = 0; j < X.numblocks; j++) {
            double mult = rateR * X.regmult[j] * X.learnmult[j];
            mult = pow((1-mult), REGFREQ);
            for (int k = 0; k < X.blocksizes[j]; k++) {
              w[j][k] = mult * w[j][k];
            }
          }
        } else {
          // assume simple mixture model
          int maxc = 0;
          double bestval = 0;
          for (int c = 0; c < X.numcomponents; c++) {
            double val = 0;
            for (int i = 0; i < X.componentsizes[c]; i++) {
              int b = X.componentblocks[c][i];
              double blockval = 0;
              for (int k = 0; k < X.blocksizes[b]; k++)
                blockval += w[b][k] * w[b][k] * X.regmult[b];
              val += blockval;
            }
            if (val > bestval) {
              maxc = c;
              bestval = val;
            }
          }
          for (int i = 0; i < X.componentsizes[maxc]; i++) {
            int b = X.componentblocks[maxc][i];
            double mult = rateR * X.regmult[b] * X.learnmult[b];        
            mult = pow((1-mult), REGFREQ);
            for (int k = 0; k < X.blocksizes[b]; k++)
              w[b][k] = mult * w[b][k];
          }
        }
      }
    }
  }

  if (converged)
    LINFO("Termination criteria reached after %d iterations.\n", t);
  else
    LINFO("Max iteration count reached.\n", t);
}

