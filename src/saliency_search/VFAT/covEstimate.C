/*!@file VFAT/covEstimate.C quick estimator for coveriance matrix
 */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/covEstimate.C $
// $Id: covEstimate.C 10794 2009-02-08 06:21:09Z itti $
//
// ############################################################
// ############################################################
// ##### --- VFAT ---
// ##### Vision Feature Analysis Tool:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Laurent Itt itti@pollux.usc.edu
// #####
// ############################################################
// ############################################################
// Create object and copy pointer to data matrix

#include "VFAT/covEstimate.H"

#include "Util/Assert.H"
#include "Image/Point2D.H"

#include <fstream>

template <class T>
covEstimate<T>::covEstimate(std::vector<std::vector<T> > &_inputSpace,
                            covHolder<T> &_ch)
{
  cov_currentStep = 0.0F;
  // call resize space
  std::cerr << "Inputing space\n";  setNew(_inputSpace,true,_ch,0);
}

/*********************************************************************/

template <class T>
covEstimate<T>::covEstimate(std::vector<std::vector<T*> > &_inputSpace,
                            covHolder<T> &_ch)
{
  cov_currentStep = 0.0F;
  // call resize space
  std::cerr << "Inputing space\n";  setNew(_inputSpace,true,_ch,0);
  cov_samples = 0;
}

/*********************************************************************/
template <class T>
covEstimate<T>::covEstimate()
{
  cov_currentStep = 0.0F;
}

/*********************************************************************/
template <class T>
covEstimate<T>::~covEstimate()
{}

// Find the mean values in the data set

/*********************************************************************/

template <class T>
void covEstimate<T>::findMean()
{
  T grandMean = 0;
  if(cov_usePtr == false)
  {
    for(unsigned int i = 0; i < cov_dim; i++)
    {
      T total = 0;
      T TSS   = 0;
      for(unsigned int k = 0; k < cov_samples; k++)
      {
        total += cov_inputSpace->at(i)[k];
        TSS   += pow(cov_inputSpace->at(i)[k],2);
      }
      cov_covHolder->mean[i] = total/cov_samples;
      cov_covHolder->STD[i]  = sqrt((TSS/cov_samples)
                                   - pow(cov_covHolder->mean[i],2));
      grandMean += cov_covHolder->mean[i];
    }
  }
  else
  {
    if(cov_useFloatSpecial == false)
    {
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        T total = 0;
        T TSS   = 0;
        for(unsigned int k = 0; k < cov_samples; k++)
        {
          total += *cov_inputSpacePtr->at(i)[k];
          TSS   += pow(*cov_inputSpacePtr->at(i)[k],2);
        }
        cov_covHolder->mean[i] = total/cov_samples;

        T P = (TSS/cov_samples) - pow(cov_covHolder->mean[i],2);

        // rounding errors in floting points can make this less than 0
        if(P > 0)
          cov_covHolder->STD[i] = sqrt(P);
        else
          cov_covHolder->STD[i] = 0.0F;

        grandMean += cov_covHolder->mean[i];
      }
    }
    else
    {
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        float total = 0;
        double TSS  = 0;
        for(unsigned int k = 0; k < cov_samples; k++)
        {
          total += *cov_FinputSpacePtr->at(i)[k];
          TSS   += (double)pow(*cov_FinputSpacePtr->at(i)[k],2);
        }
        cov_covHolder->mean[i] = total/cov_samples;

        T P = (TSS/cov_samples) - pow(cov_covHolder->mean[i],2);

        // rounding errors in floting points can make this less than 0
        if(P > 0)
          cov_covHolder->STD[i] = sqrt(P);
        else
          cov_covHolder->STD[i] = 0.0F;

        grandMean += cov_covHolder->mean[i];
      }
    }
  }
  cov_covHolder->grandMean = grandMean/cov_dim;
  cov_currentStep = 1.0F;
}

/*********************************************************************/

template <class T>
void covEstimate<T>::findMinMax()
{
  T *minHold;
  T *maxHold;
  if(cov_usePtr == false)
  {
    for(unsigned int i = 0; i < cov_dim; i++)
    {
      minHold = &cov_covHolder->mean[i];
      maxHold = &cov_covHolder->mean[i];
      for(unsigned int k = 0; k < cov_samples; k++)
      {
        if(cov_inputSpace->at(i)[k] > *maxHold)
          maxHold = &cov_inputSpace->at(i)[k];
        if(cov_inputSpace->at(i)[k] < *minHold)
          minHold = &cov_inputSpace->at(i)[k];
      }
      cov_covHolder->min[i] = *minHold;
      cov_covHolder->max[i] = *maxHold;
    }
  }
  else
  {
    if(cov_useFloatSpecial == false)
    {
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        minHold = &cov_covHolder->mean[i];
        maxHold = &cov_covHolder->mean[i];
        for(unsigned int k = 0; k < cov_samples; k++)
        {
          if(*cov_inputSpacePtr->at(i)[k] > *maxHold)
            maxHold = cov_inputSpacePtr->at(i)[k];
          if(*cov_inputSpacePtr->at(i)[k] < *minHold)
            minHold = cov_inputSpacePtr->at(i)[k];
        }
        cov_covHolder->min[i] = *minHold;
        cov_covHolder->max[i] = *maxHold;
      }
    }
    else
    {
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        float FminHold = cov_covHolder->mean[i];
        float FmaxHold = cov_covHolder->mean[i];
        for(unsigned int k = 0; k < cov_samples; k++)
        {
          if(*cov_FinputSpacePtr->at(i)[k] > FmaxHold)
            FmaxHold = *cov_FinputSpacePtr->at(i)[k];
          if(*cov_FinputSpacePtr->at(i)[k] < FminHold)
            FminHold = *cov_FinputSpacePtr->at(i)[k];
        }
        cov_covHolder->min[i] = FminHold;
        cov_covHolder->max[i] = FmaxHold;
      }
    }
  }
}

/*********************************************************************/
// translate space so that the mean is centered on the origen

template <class T>
void covEstimate<T>::translateSpace()
{
  if(cov_usePtr == true)
  {
    if(cov_useFloatSpecial == false)
    {
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        for(unsigned int k = 0; k < cov_samples; k++)
        {
          cov_transSpace[i][k] = *cov_inputSpacePtr->at(i)[k] -
            cov_covHolder->mean[i];
        }
      }
    }
    else
    {
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        for(unsigned int k = 0; k < cov_samples; k++)
        {
          cov_transSpace[i][k] = (T)*cov_FinputSpacePtr->at(i)[k] -
            cov_covHolder->mean[i];
        }
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < cov_dim; i++)
    {
      for(unsigned int k = 0; k < cov_samples; k++)
      {
        cov_transSpace[i][k] = cov_inputSpace->at(i)[k] -
          cov_covHolder->mean[i];
      }
    }
  }

  cov_currentStep = 2.0F;
}

/*********************************************************************/
// find the eigen vectors, then rotate to (approx) 0 covariance
// NOTE: to increase accuracy call this method several consecutive times
// with doSimple equal to false. Or use a combination of simple and
// following several non-simple rotations (???????)

template <class T>
void covEstimate<T>::findRotatios(bool doSimple = true)
{
  cov_doSimple = doSimple;
  cov_listSize = 0;

  // For each feature
  for(unsigned int i = 0; i < cov_dim; i++)
  {
    // for each feature (upper triangular)
    for(unsigned int j = i + 1; j < cov_dim; j++,cov_listSize++)
    {
      T meanRatio = 0;
      T eucDist   = 0;
      T eucSum    = 0;

      // for each sample
      for(unsigned int k = 0; k < cov_samples; k++)
      {
        T *trans_ik = &cov_transSpace[i][k];
        T *trans_jk = &cov_transSpace[j][k];

        /* find ratio between x,y and the distance to coord */
        eucDist = sqrt(pow(*trans_ik,2) +
                       pow(*trans_jk,2));

        /* sum and find average ratios weighted for distance */
        if(*trans_ik != 0)
        {
          meanRatio += (*trans_jk/(*trans_ik))
            * eucDist;
        }
        eucSum += eucDist;
      }

      //alter to weighted average by distance
      if(eucSum != 0)
        cov_covHolder->meanRatio[i][j] = meanRatio/eucSum;
      else
        cov_covHolder->meanRatio[i][j] = 0;


      cov_covHolder->meanTheta[i][j] = atan(cov_covHolder->meanRatio[i][j]);

      T *meanTheta = &cov_covHolder->meanTheta[i][j];

      for(unsigned int k = 0; k < cov_samples; k++)
      {
        //LINFO("C1 %d %d %d",i,j,k);
        T *trans_ik = &cov_transSpace[i][k];
        T *trans_jk = &cov_transSpace[j][k];
        T *rot_ik   = &cov_rotSpace[i][k];
        T *rot_jk   = &cov_rotSpace[j][k];

        int inv1, inv2;
        // invert the sin from the last axis to the first only

        inv1 = 1;
        inv2 = -1;

        T X = *trans_ik;
        T Y = *trans_jk;

        // X' = [Xcos(theta)*Y(inv1)sin(theta)]
        *rot_ik = (X * cos(*meanTheta))
          + (Y * (inv1*sin(*meanTheta)));

        // Y' = [X(inv2)sin(theta)*Ycos(theta)]
        *rot_jk = (X * (inv2*(sin(*meanTheta))))
          + (Y * cos(*meanTheta));
      }

      T SS = 0;
      for(unsigned int k = 0; k < cov_samples; k++)
      {
        SS += pow(cov_rotSpace[i][k],2)/cov_samples;
      }
      if(cov_sumSquare[i] == 0)
      {
        cov_sumSquare[i] = SS;
      }
      cov_sumSquare[i] += pow(sqrt(SS) * cos(-1*(*meanTheta)),2);

      SS = 0;
      for(unsigned int k = 0; k < cov_samples; k++)
      {
        SS += pow(cov_rotSpace[j][k],2)/cov_samples;
      }
      if(cov_sumSquare[j] == 0)
      {
        cov_sumSquare[j] = SS;
      }
      cov_sumSquare[j] += pow(sqrt(SS) * cos(-1*(*meanTheta)),2);
    }
  }
  cov_currentStep = 3.0F;
}

/*********************************************************************/

template <class T>
void covEstimate<T>::findVariance()
{
   for(unsigned int i = 0; i < cov_dim; i++)
  {
    if(cov_sumSquare[i] > 0)
      cov_covHolder->eigenVal[i] = sqrt(cov_sumSquare[i]);
    else
      cov_covHolder->eigenVal[i] = 0;
  }
  cov_currentStep = 4.0F;
}

/*********************************************************************/

template <class T>
T covEstimate<T>::getP(std::vector<T> sample, covHolder<T> &ch, int stop)
{
  int stopper = ch.dim-stop;

  ASSERT(sample.size() == ch.dim);

  typename std::vector<T>::iterator iSample;
  typename std::vector<T>::iterator iTrans;
  typename std::vector<T>::iterator iMean;
  typename std::vector<T>::iterator iBias;

  iSample = ch.mean.begin();
  iMean   = sample.begin();
  iTrans  = cov_meanTranslate.begin();
  iBias   = ch.bias.begin();

  while(iSample != ch.mean.end())
  {
    *iTrans = *iMean - *iSample;
    ++iTrans; ++iMean; ++iSample;
  }

  // more stuff
  for(unsigned int i = 0; i < sample.size(); i++)
  {
    for(unsigned int j = i + 1; j < sample.size(); j++)
    {
      //alter to weighted average by distance

      T *meanTheta = &ch.meanTheta[i][j];

      int inv1, inv2;
      // invert the sin from the last axis to the first only

      inv1 = 1;
      inv2 = -1;

      T X = cov_meanTranslate[i];
      T Y = cov_meanTranslate[j];
      T rot_ik,rot_jk;

      // X' = [Xcos(theta)*Y(inv1)sin(theta)]
      rot_ik = (X * cos(*meanTheta))
        + (Y * (inv1*sin(*meanTheta)));

      // Y' = [X(inv2)sin(theta)*Ycos(theta)]
      rot_jk = (X * (inv2*(sin(*meanTheta))))
        + (Y * cos(*meanTheta));

      if(cov_sumSquare[i] == 0)
      {
        cov_sumSquare[i] = rot_ik;
      }
      cov_sumSquare[i] += pow(rot_ik * cos(-1*(*meanTheta)),2);

      if(cov_sumSquare[j] == 0)
      {
        cov_sumSquare[j] = rot_jk;
      }
      cov_sumSquare[j] += pow(rot_jk * cos(-1*(*meanTheta)),2);
    }
  }

  for(unsigned int i = 0; i < sample.size(); i++)
  {
    if(cov_sumSquare[i] > 0)
      cov_sumSquare[i] = sqrt(cov_sumSquare[i]);
    else
      cov_sumSquare[i] = 0;
  }

  iTrans = cov_sumSquare.begin();
  int i = 0;
  T P = 0;
  T TP = 0;
  while(iTrans != cov_sumSquare.end())
  {
    if(ch.eigenVal[i] != 0)
      TP = gauss(*iTrans,0,ch.eigenVal[i])*(*iBias);

    if(i == 0)
      P = TP;
    else
    {
      if(i < stopper)
        if(TP != 0)
        {
          P *= TP;
        }
    }
    i++;
    ++iTrans; ++iBias;
  }
  P = P*ch.samples;

  return P;
}

/*********************************************************************/

template <class T>
T covEstimate<T>::getD(std::vector<T> sample, covHolder<T> &ch, int stop)
{
  int stopper = ch.dim-stop;
  T SS        = 0.0F;

  ASSERT(sample.size() == ch.dim);

  typename std::vector<T>::iterator iSample;
  typename std::vector<T>::iterator iMean;
  typename std::vector<T>::iterator iBias;

  iSample = ch.mean.begin();
  iMean   = sample.begin();
  iBias   = ch.bias.begin();

  for(int i = 0; i < stopper; i++)
  {
    SS += pow((*iMean - *iSample)/(*iBias),2);
    ++iMean; ++iSample; ++iBias;
  }
  return sqrt(SS);
}

/*********************************************************************/

template <class T>
T covEstimate<T>::getD(std::vector<T> *point1,std::vector<T> *point2,
                       std::vector<T> *biasSet, bool useGause)
{
  typename std::vector<T>::iterator p1 = point1->begin();
  typename std::vector<T>::iterator p2 = point2->begin();
  typename std::vector<T>::iterator b  = biasSet->begin();

  double D = 0.0F;

  if(useGause == false)
  {
    while(p1 != point1->end())
    {
      D += pow(((*p1 - *p2)),2);
      ++p1; ++p2; ++b;
    }
    D = 1/sqrt(D);
  }
  else
  {
    while(p1 != point1->end())
    {
      D += gauss(*p1,*p2,(*b/8));
      ++p1; ++p2; ++b;
    }
    D = 1/D;
  }
  return D;
}

/*********************************************************************/

#if 0
// FIXME this function doesn't compile cleanly
template <class T>
void covEstimate<T>::matchPmean(std::vector<covHolder<T> > *chNew,
                                long sizeNew,
                                std::vector<covHolder<T> > *chOld,
                                long sizeOld, long minSize)
{
  typename std::vector<covHolder<T> >::iterator chNewi;
  typename std::vector<covHolder<T> >::iterator chOldi;
  std::vector<bool> reserveTable;
  reserveTable.resize(sizeNew,false);
  chNewi = chNew->begin();
  for(long ii = 0; ii < sizeNew; ii++, ++chNewi)
  {
    chNewi->isMatched = false;
  }
  chOldi = chOld->begin();
  for(long  jj = 0; jj < sizeOld; jj++, ++chOldi)
  {
    chOldi->isMatched = false;
  }


  for(int i = 0; i < sizeNew; i++)
  {
    chNewi = chNew->begin();
    double bestD = 0.0F;
    unsigned long bestClassO = 0;
    unsigned long bestClassN = 0;
    bool setThis = false;
    for(int ii = 0; ii < sizeNew; ii++, ++chNewi)
    {
      if(chNewi->isMatched == false)
      {
        if(chNewi->isLarge == true)
        {
          chOldi = chOld->begin();
          for(int jj = 0; jj < sizeOld; jj++, ++chOldi)
          {
            if(chOldi->isMatched == false)
            {
              if(chOldi->isLarge == true)
              {
                double D = getD(&(chNewi->mean),&(chOldi->mean),
                                &(chOldi->bias),false);
                LINFO("D %f for %d and %d",D,ii,jj);
                if(D > bestD)
                {
                  bestD = D;
                  bestClassN = ii;
                  bestClassO = jj;
                  setThis = true;
                  LINFO("SET BEST D");
                  for(unsigned int m = 0; m < chNewi->dim; m++)
                  {
                    LINFO("MATCH : %f to %f",chNewi->mean[m],chOldi->mean[m]);
                  }
                }
              }
            }
          }
        }
      }
    }
    if(setThis == true)
    {
      //LINFO("FINAL %d MATCHED TO %d",bestClassN,bestClassO);
      //LINFO("NEW matchID %d"
      //            ,chOld->at(bestClassO).matchID);
      //LINFO("SAMPLES NEW %d OLD %d",chNew->at(bestClassN).samples,
      //            chOld->at(bestClassO).samples);
      //for(unsigned int m = 0; m < chNew->at(bestClassN).dim; m++)
      //{
      //        LINFO("FINAL MATCH : %f to %f",chNew->at(bestClassN).mean[m]
      //              ,chOld->at(bestClassO).mean[m]);
      //}
      // how long has this object been alive for?
      if(chNew->at(bestClassN).matchID == chOld->at(bestClassO).matchID)
      {
        chNew->at(bestClassN).lifeSpan = chOld->at(bestClassO).lifeSpan + 1;
        // compute 1st and 2nd order partial derivatives for this objects
        // features as well as the partial first order integral
        for(unsigned int m = 0; m < chNew->at(bestClassN).dim; m++)
        {
          chNew->at(bestClassN).speed[m] = chOld->at(bestClassO).mean[m] -
            chNew->at(bestClassN).mean[m];

          chNew->at(bestClassN).avgspeed[m] =
            (chOld->at(bestClassO).avgspeed[m] +
             chNew->at(bestClassN).speed[m])/2;

          chNew->at(bestClassN).accel[m] = chOld->at(bestClassO).speed[m] -
            chNew->at(bestClassN).speed[m];

          chNew->at(bestClassN).avgaccel[m] =
            (chOld->at(bestClassO).avgaccel[m] +
             chNew->at(bestClassN).accel[m])/2;

          chNew->at(bestClassN).distance[m] += chNew->at(bestClassN).speed[m];
        }
      }
      else
      {
        if(chNew->at(bestClassN).lifeSpan != 0)
        {
          chNew->at(bestClassN).lifeSpan = 0;
          // reset derivatives and integrals if we are new
          for(unsigned int m = 0; m < chNew->at(bestClassN).dim; m++)
          {
            chNew->at(bestClassN).speed = 0;
            chNew->at(bestClassN).accel = 0;
            chNew->at(bestClassN).distance = 0;
          }
        }
      }

      chNew->at(bestClassN).matchID = chOld->at(bestClassO).matchID;
      chNew->at(bestClassN).isMatched = true;
      chOld->at(bestClassO).isMatched = true;
      if(chNew->at(bestClassN).matchID < reserveTable.size())
        reserveTable[chNew->at(bestClassN).matchID] = true;
    }
  }

  // if some classes are left over, assign them their own class

  if(sizeNew > sizeOld)
  {
    typename std::vector<bool>::iterator rTable;
    unsigned long last = 0;
    chNewi = chNew->begin();
    for(int ii = 0; ii < sizeNew; ii++, ++chNewi)
    {
      if(chNewi->isMatched == false)
      {
        unsigned long cc = last;
        for(rTable = reserveTable.begin() + last; rTable != reserveTable.end();
            ++rTable, cc++)
        {
          if(*rTable == false)
          {
            chNewi->isMatched = true;
            chNewi->matchID = cc;
            last = cc;
            //LINFO("%d SET BUT NOT MATCHED AS %d",ii,cc);
            *rTable = true;
            break;
          }
        }
      }
    }
  }
}
#endif

/*********************************************************************/

template <class T>
void covEstimate<T>::matchPmeanAccum(std::vector<covHolder<T> > *ch,
                                     unsigned int *sizeCH,
                                     std::vector<covHolder<T> > *accum,
                                     unsigned int *sizeAccum,
                                     long minSize)
{

  // For storage, see covHolder.H to see where all this stuff is going

  typename std::vector<covHolder<T> >::iterator chi;
  typename std::vector<covHolder<T> >::iterator accumi;

  chi = ch->begin();
  for(long  jj = 0; jj < (signed)*sizeCH; jj++, ++chi)
  {
    chi->isMatched = false;
  }

  accumi = accum->begin();
  for(long  jj = 0; jj < (signed)*sizeAccum; jj++, ++accumi)
  {
    accumi->isMatched = false;
  }

  // Do a stupid N^2 match of every class to every class to find the best
  // matches. Match the first best (closest pair) then remove them. Then
  // match the second best until matching cannot be further completed.
  // This is sub optimal in complexity, but the numbers are so small it
  // is not worth my time to optimize this part of the code (total number
  // of classes < 20 most of the time)

  for(int i = 0; i < (signed)*sizeAccum; i++)
  {
    accumi = accum->begin();
    double bestD = 0.0F;
    unsigned long bestClassO = 0;
    unsigned long bestClassN = 0;
    bool setThis = false;
    for(int ii = 0; ii < (signed)*sizeAccum; ii++, ++accumi)
    {
      if(accumi->isMatched == false)
      {
        if(accumi->isLarge == true)
        {
          chi = ch->begin();
          for(int jj = 0; jj < (signed)*sizeCH; jj++, ++chi)
          {
            if(chi->isMatched == false)
            {
              if(chi->isLarge == true)
              {
                // compute distance between classes
                double D = getD(&(accumi->mean),&(chi->mean),
                                &(chi->bias),false);
                if(D > bestD)
                {
                  bestD = D;
                  bestClassN = ii;
                  bestClassO = jj;
                  setThis = true;
                }
              }
            }
          }
        }
      }
    }

    // We have found a good match for this class from the last iteration
    // copy over some properties, compute running averages etc. incrememnt
    // life span.

    if(setThis == true)
    {
      // how long has this object been alive for?
      accum->at(bestClassN).lifeSpan++;
      // compute first and second order derivatives over features
      // as well as averages and
      for(unsigned int m = 0; m < accumi->dim; m++)
      {
        ch->at(bestClassO).speed[m] = ch->at(bestClassO).mean[m] -
          accum->at(bestClassN).mean[m];

        ch->at(bestClassO).accel[m] = ch->at(bestClassO).speed[m] -
            accum->at(bestClassN).speed[m];

        ch->at(bestClassO).distance[m] = ch->at(bestClassO).speed[m]
          + accum->at(bestClassN).distance[m];

        ch->at(bestClassO).avgspeed[m] =
          (ch->at(bestClassO).speed[m] +
           accum->at(bestClassN).avgspeed[m])/2;

        ch->at(bestClassO).avgaccel[m] =
          ((ch->at(bestClassO).avgspeed[m] -
            accum->at(bestClassN).avgspeed[m])
           + accum->at(bestClassN).avgaccel[m])/2;
      }
      ch->at(bestClassO).matchID = accum->at(bestClassN).matchID;
      ch->at(bestClassO).isMatched = true;
      ch->at(bestClassO).lifeSpan = accum->at(bestClassN).lifeSpan;
      accum->at(bestClassN) = ch->at(bestClassO);
    }
  }

  // reset any unmatched classes
  for(int i = 0; i < (signed)*sizeAccum; i++)
  {
    // no match was found for this class, therefor we reset some parts
    // we only decay some of the running averages. We hold onto some
    // values incase it turns out that it matches next iteration

    if(accum->at(i).isMatched == false)
    {
      accum->at(i).lifeSpan = 0;
      // reset absolute speed and accel, but decay averages
      for(unsigned int m = 0; m < accum->at(i).dim; m++)
      {
        accum->at(i).speed[m] = 0;
        accum->at(i).accel[m] = 0;
        accum->at(i).distance[m] = 0;
        accum->at(i).avgspeed[m] = accum->at(i).avgspeed[m]/2;
        accum->at(i).avgaccel[m] = ((accum->at(i).avgspeed[m] -
                                accum->at(i).avgspeed[m])
                               + accum->at(i).avgaccel[m])/2;
      }
    }
  }

  // if some classes are left over, assign them their own class
  // As such, this iteration we may find more classes then in the last
  // iteration. As such, we cannot throw them away, but insted assign them
  // their own class

  if(*sizeCH > *sizeAccum)
  {
    //unsigned long last = *sizeAccum;
    chi = ch->begin();
    for(int ii = 0; ii < (signed)*sizeCH; ii++, ++chi)
    {
      if(chi->isMatched == false)
      {
        if(chi->isLarge == true)
        {
          chi->isMatched = true;
          chi->matchID = *sizeAccum;
          accum->at(*sizeAccum) = *chi;
          *sizeAccum = *sizeAccum + 1;
          //LINFO("%d SET BUT NOT MATCHED AS %d",ii,*sizeAccum);
        }
      }
    }
  }
}

/*********************************************************************/

template <class T>
T covEstimate<T>::gauss(T X, T mu, T std)
{
  return (1/(sqrt(2*3.14159*pow(std,2))))
  *exp(-1*(pow((X - mu),2)/(2*pow(std,2))));
}

/*********************************************************************/

template <class T>
void covEstimate<T>::run()
{
  //LINFO("FIND MEAN");
  findMean();
  //LINFO("FIND MIN and MAX");
  findMinMax();
  //LINFO("TRANSLATE SPACE");
  translateSpace();
  //LINFO("FIND ROTATIOS");
  findRotatios(true);
  //LINFO("FIND VARAINCE");
  findVariance();
}

/*********************************************************************/
template <class T>
void covEstimate<T>::setNew(std::vector<std::vector<T> > &_inputSpace,
                            bool doResize, covHolder<T> &_ch, T initVal)
{
  cov_covHolder       = &_ch;
  cov_usePtr          = false;
  cov_useFloatSpecial = false;

  cov_inputSpace      = &(_inputSpace);
  cov_dim             = cov_inputSpace->size();
  cov_samples         = cov_inputSpace->at(0).size();

  if((doResize == true) || (_inputSpace.size() != cov_transSpace.size()))
  {
    resize(_inputSpace.size(),_inputSpace[0].size(),initVal);
    cov_covHolder->samples = _inputSpace[0].size();
  }
}

/*********************************************************************/

template <class T>
void covEstimate<T>::setNew(std::vector<std::vector<T*> > &_inputSpace,
                            bool doResize, covHolder<T> &_ch, T initVal)
{
  cov_covHolder       = &_ch;
  cov_usePtr          = true;
  cov_useFloatSpecial = false;

  cov_inputSpacePtr   = &(_inputSpace);
  cov_dim             = cov_inputSpace->size();
  cov_samples         = cov_inputSpace->at(0).size();

  if((doResize == true) || (_inputSpace.size() != cov_transSpace.size()))
  {
    resize(_inputSpace.size(),_inputSpace[0].size(),initVal);
    cov_covHolder->samples = _inputSpace[0].size();
  }
}

/*********************************************************************/

template <class T>
void covEstimate<T>::setNew(std::vector<std::vector<T> > &_inputSpace,
                            T initVal, int samples, int dim, covHolder<T> &_ch,
                            bool _resize = false)
{
  cov_covHolder          = &_ch;
  cov_usePtr             = false;
  cov_useFloatSpecial    = false;

  cov_inputSpace         = &(_inputSpace);
  cov_dim                = dim;
  cov_samples            = samples;
  cov_covHolder->samples = samples;

  if(_resize == true)
    resize(dim,samples,initVal);

}

/*********************************************************************/

template <class T>
void covEstimate<T>::setNew(std::vector<std::vector<T*> > &_inputSpace,
                            T initVal, int samples, int dim, covHolder<T> &_ch,
                            bool _resize = false)
{
  cov_covHolder          = &_ch;
  cov_usePtr             = true;
  cov_useFloatSpecial    = false;

  cov_inputSpacePtr      = &(_inputSpace);
  cov_dim                = dim;
  cov_samples            = samples;
  cov_covHolder->samples = samples;

  if(_resize == true)
    resize(dim,samples,initVal);

  cov_covHolder->samples = samples;

}

/*********************************************************************/

template <class T>
void covEstimate<T>::setNewF(std::vector<std::vector<float*> > &_inputSpace,
                            T initVal, int samples, int dim, covHolder<T> &_ch,
                            bool _resize = false)
{
  cov_covHolder          = &_ch;
  cov_usePtr             = true;
  cov_useFloatSpecial    = true;

  cov_FinputSpacePtr     = &(_inputSpace);
  cov_dim                = dim;
  cov_samples            = samples;
  cov_covHolder->samples = samples;

  if(_resize == true)
    resize(dim,samples,initVal);

  cov_covHolder->samples = samples;

}

/*********************************************************************/

template <class T>
void covEstimate<T>::resize(unsigned int _dimensions, unsigned int _samples,
                            T zero)
{
  unsigned int dimensions = _dimensions;
  unsigned int samples = _samples;
  cov_tempTS.resize(samples,zero);
  cov_transSpace.resize(dimensions,cov_tempTS);
  cov_rotSpace.resize(dimensions,cov_tempTS);
  cov_meanTranslate.resize(dimensions,zero);
  cov_meanRot.resize(dimensions,zero);
  cov_sumSquare.resize(dimensions,zero);
  cov_list_i.resize(((int)((pow((float)dimensions,2)/2)+dimensions)),0);
  cov_list_j.resize(((int)((pow((float)dimensions,2)/2)+dimensions)),0);
  cov_listSize = 0;
  cov_currentStep = 0.5F;
}

/*********************************************************************/

template <class T>
void covEstimate<T>::printDebug()
{
  int doThis = (int)cov_currentStep*10;
  switch(doThis)
  {
  case 5 : std::cerr << "SPACE RESIZED D:" << cov_transSpace.size()
                         << " S:" << cov_transSpace[1].size()
                         << " List Size:" << cov_list_i.size()
                         << " Data Zero:" << cov_transSpace[1][1]
                         << "\n"; break;
  case 10 : std::cerr << "Mean values: \n";
    if(cov_usePtr)
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        std::cerr << i << " : " << cov_covHolder->mean[i] << "\n";
      }
    else
      for(unsigned int i = 0; i < cov_dim; i++)
      {
        std::cerr << i << " : " << cov_covHolder->mean[i] << "\n";
      }
    break;
  case 20 : std::cerr << "Space translated\n"; break;
  case 30 : std::cerr << "Space Ratios\n";
      for(unsigned int i = 0; i < cov_transSpace.size(); i++)
      {
        for(unsigned int j = 0; j < cov_transSpace.size(); j++)
        {
          std::cerr << cov_covHolder->meanRatio[i][j] << "\t";
        }
        std::cerr << "\n";
      }
      std::cerr << "\n";
      std::cerr << "Space Angles\n";
      for(unsigned int i = 0; i < cov_transSpace.size(); i++)
      {
        for(unsigned int j = 0; j < cov_transSpace.size(); j++)
        {
          std::cerr << cov_covHolder->meanTheta[i][j] << "\t";
        }
        std::cerr << "\n";
      }
      std::cerr << "\n";
      break;
  case 40 :  std::cerr << "Cov Eigen\n";
    for(unsigned int i = 0; i < cov_dim; i++)
    {
      std::cerr << cov_covHolder->eigenVal[i] << "\t";
    }
    std::cerr << "\n";
    break;
  default: std::cerr << "No debug info on state\n" ; break;
  }
}

/*********************************************************************/

template <class T>
void covEstimate<T>::printEigenVals()
{
  std::cout << "EIGEN VALUES\n";
  for(unsigned int i = 0; i < cov_dim; i++)
  {
    std::cout << i << ":\t" << cov_covHolder->eigenVal[i] << "\n";
  }
  std::cout << "RATIOS\n";
  for(unsigned int i = 0; i < cov_dim; i++)
  {
    for(unsigned int j = 0; j < cov_dim; j++)
    {
      std::cout << cov_covHolder->meanRatio[i][j] << "\t";
    }
    std::cout << "\n";
  }
}

/*********************************************************************/

template <class T>Image<float> covEstimate<T>::returnMatrixSlice(int dim1,
                                                                 int dim2,
                                                                 int spaceSize)
{
  Image<float> output;
  output.resize(spaceSize,spaceSize);
  T temp1 = 0;
  T temp2 = 0;

  if(cov_currentStep >= 2)
  {
    temp1 = cov_covHolder->mean[dim1];
    temp2 = cov_covHolder->mean[dim2];
    drawLine(output, Point2D<int>((spaceSize-(int)temp1),0),
             Point2D<int>((spaceSize-(int)temp1),spaceSize),128.0F,1);
    drawLine(output, Point2D<int>(0,(int)temp2),
             Point2D<int>(spaceSize,(int)temp2),128.0F,1);

  }

  for(unsigned int i = 0; i < cov_samples; i++)
  {
    output.setVal(spaceSize-((int)cov_transSpace[dim1][i] + (int)temp1)
                    ,(int)cov_transSpace[dim2][i] + (int)temp2,255.0F);
  }
  return output;
}


/*********************************************************************/

template <class T>
Image<float> covEstimate<T>::returnCovSlice(int dim1, int dim2,
                                            int spaceSize,
                                            bool inorm)
{
  Image<float> output;
  output.resize(spaceSize,spaceSize);
  return returnCovSlice(dim1,dim2,output,inorm);
}

/*********************************************************************/

template <class T>
Image<float> covEstimate<T>::returnCovSlice(int dim1, int dim2,
                                            Image<float> &output,
                                            bool inorm)
{
  LINFO("Creating cov Slice Image");
  ASSERT(output.getWidth() == output.getHeight());
  int spaceSize = output.getWidth();

  T norm = 1;
  T Xtrans = 0;
  T Ytrans = 0;
  if(cov_usePtr == true)
  {
    LINFO("DIMS %d %d", dim1,dim2);

    LINFO("IMAGE SIZE %d %d",output.getWidth(),output.getHeight());
    if(inorm == true)
    {
      norm = spaceSize;
      T maxX = 0;
      T minX = *cov_inputSpacePtr->at(dim1)[0];
      T maxY = 0;
      T minY = *cov_inputSpacePtr->at(dim2)[0];
      for(unsigned int i = 0; i < cov_samples; i++)
      {
        T temp1 = *cov_inputSpacePtr->at(dim1)[i];
        if(temp1 < minX)
          minX = temp1;
        if(temp1 > maxX)
          maxX = temp1;

        T temp2 = *cov_inputSpacePtr->at(dim2)[i];
        if(temp2 < minY)
          minY = temp2;
        if(temp2 > maxY)
          maxY = temp2;
      }
      T tx = maxX - minX;
      T ty = maxY - minY;
      if(tx > ty)
        norm = ((spaceSize-1)/tx)*0.9F;
      else
        norm = ((spaceSize-1)/ty)*0.9F;
      T h1 = 0; T h2 = 0;
      for(unsigned int i = 0; i < cov_samples; i++)
      {
        T temp1 = norm * (*cov_inputSpacePtr->at(dim1)[i]);
        T temp2 = norm * (*cov_inputSpacePtr->at(dim2)[i]);
        if(temp1 < h1)
          h1 = temp1;
        if(temp2 < h2)
          h2 = temp2;
      }
      Xtrans = fabs(h1);
      Ytrans = fabs(h2);
    }
    LINFO("NORMALIZER %f",norm);
    LINFO("Xtrans %f Ytrans %f",Xtrans,Ytrans);
    for(unsigned int i = 0; i < cov_samples; i++)
    {
      LINFO("SETTING %d %d",
            (spaceSize-1)-((int)(Xtrans+
                                 (norm*(*cov_inputSpacePtr->at(dim1)[i])))),
            ((int)(Ytrans+(norm*(*cov_inputSpacePtr->at(dim2)[i])))));

      drawPoint(output, (spaceSize-1)
                -((int)(Xtrans+(norm*(*cov_inputSpacePtr->at(dim1)[i]))))
                ,((int)(Ytrans+(norm*(*cov_inputSpacePtr->at(dim2)[i])))),128.0F);
    }
  }
  else
  {
    for(unsigned int i = 0; i < cov_samples; i++)
    {
      drawPoint(output, spaceSize-((int)cov_inputSpace->at(dim1)[i])
                ,(int)(cov_inputSpace->at(dim2)[i]),128.0F);
    }
  }

  // ************************** now draw eigen vector and values

  T ortho, Rise1, Run1, Rise2, Run2;

  if(cov_covHolder->meanRatio[dim1][dim2] != 0)
    ortho = -1/cov_covHolder->meanRatio[dim1][dim2];
  else
  {
    ortho =-1/0.0000001;
    std::cout << "Cov matrix 1 is 0\n";
  }

  if(cov_covHolder->eigenVal[dim1] != 0)
    Rise1 = 1/sqrt((1+pow(cov_covHolder->meanRatio[dim1][dim2],2))
                   /pow(cov_covHolder->eigenVal[dim1],2));
  else
  {
    Rise1 = 1/sqrt((1+pow(cov_covHolder->meanRatio[dim1][dim2],2))
                   /pow(0.0000001,2));
    std::cout << "Cov matrix 2 is 0\n";
  }
  Run1 = (pow(cov_covHolder->eigenVal[dim1],2)
                - pow(Rise1,2));
  if(Run1 > 0)
    Run1 = sqrt(Run1);
  else
  {
    Run1 = 0;
    std::cout << "Cov matrix 3 is 0\n";
  }
  if(cov_covHolder->eigenVal[dim2] != 0)
    Rise2 = 1/sqrt((1+pow(ortho,2))
                   /pow(cov_covHolder->eigenVal[dim2],2));
  else
  {
    Rise2 = 1/sqrt((1+pow(ortho,2))
                   /pow(0.0000001,2));
    std::cout << "Cov matrix 4 is 0\n";
  }
  T temp = pow(cov_covHolder->eigenVal[dim2],2) - pow(Rise2,2);
  if(temp > 0)
    Run2 = sqrt(temp);
  else
    Run2 = 0;

  LINFO("Ytrans %f Xtrans %f norm %f",Ytrans,Xtrans,norm);
  std::cerr << Run1 << " " << Rise1 << " " << Run2 << " " << Rise2 << "\n";

  int P1a = (int)(Ytrans+norm*(cov_covHolder->mean[dim2] + Run1));
  int P1b = (int)(spaceSize - (Xtrans+norm*((cov_covHolder->mean[dim1])
                                            + Rise1)));
  int P2a = (int)(Ytrans+norm*(cov_covHolder->mean[dim2] - Run1));
  int P2b = (int)(spaceSize - (Xtrans+norm*((cov_covHolder->mean[dim1])
                                            - Rise1)));

  int P3a = (int)(Ytrans+norm*(cov_covHolder->mean[dim2] - Run2));
  int P3b = (int)(spaceSize - (Xtrans+norm*((cov_covHolder->mean[dim1])
                                            - Rise2)));
  int P4a = (int)(Ytrans+norm*(cov_covHolder->mean[dim2] + Run2));
  int P4b = (int)(spaceSize - (Xtrans+norm*((cov_covHolder->mean[dim1])
                                            + Rise2)));

  LINFO("######-> %d,%d to %d %d", P1b,P1a,P2b,P2a);
  LINFO("######-> %d,%d to %d %d", P3a,P3b,P4a,P4b);
  char size[20];
  sprintf(size,"%f",norm);
  writeText(output, Point2D<int>(1,1),size,0.0f);

  drawLine(output, Point2D<int>((int)P1b,(int)P1a),
           Point2D<int>((int)P2b,(int)P2a),255.0f,1);
  drawLine(output, Point2D<int>((int)P3b,(int)P4a),
           Point2D<int>((int)P4b,(int)P3a),255.0f,1);
  LINFO("DONE");
  return output;
}

/*********************************************************************/

template <class T>
void covEstimate<T>::dumpMatrix(std::string fileName,int index, std::string ID)
{
  std::string eigFile, rotFile, meanFile;
  eigFile  = fileName + ".EIG.out.txt";
  rotFile  = fileName + ".ROT.out.txt";
  meanFile = fileName + ".MEAN.out.txt";
  std::ofstream eig(eigFile.c_str(),std::ios::app);
  std::ofstream rot(rotFile.c_str(),std::ios::app);
  std::ofstream mean(meanFile.c_str(),std::ios::app);
  mean  << ID << "\t" << index << "\t";
  eig   << ID << "\t" << index << "\t";
  mean  << cov_covHolder->samples << "\t";
  rot   << ID << "\t" << index << "\n";
  for(unsigned int i = 0; i < cov_covHolder->dim; i++)
  {
    mean << cov_covHolder->mean[i] << "\t";
    eig << cov_covHolder->eigenVal[i] << "\t";
    for(unsigned int j = 0; j < cov_covHolder->dim; j++)
    {
      rot << cov_covHolder->meanRatio[i][j] << "\t";
    }
    rot << "\n";
  }
  mean << "\n";
  eig << "\n";
  rot.close();
  mean.close();
  eig.close();

}


/*********************************************************************/

template <class T>
void covEstimate<T>::dumpMatrix(std::string fileName, covHolder<T> &_ch, int index,
                                std::string ID)
{
  cov_covHolder = &_ch;
  dumpMatrix(fileName,index,ID);
}

/*********************************************************************/

template <class T>
unsigned long covEstimate<T>::getSampleSize()
{
  return cov_samples;
}


template class covEstimate<float>;
template class covEstimate<double>;
