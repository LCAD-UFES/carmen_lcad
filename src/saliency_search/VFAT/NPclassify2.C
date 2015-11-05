/*!@file VFAT/NPclassify2.C  Test the nonparametric classifier
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/NPclassify2.C $
// $Id: NPclassify2.C 9412 2008-03-10 23:10:15Z farhan $
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

/* This is a non-parametric unsupervised  classifier for use in the
   future with kubists. It takes as an input data points in n dimentions.
   classification is done in a clustering meathod by finding for any point
   whats its local density is then linking that point to the next closest
   point that has a higher density rating. Class boundry is created by
   "snipping" links that are for instance too long. This meathod has the
   advantage of being non-parametric. It makes no assumptions about data
   distribution (per se) or how many classes may exist. By taking the
   nearest point of highest density it can also distinguish between two
   incidentally connected classes e.g. two classes that are clustered close
   together.
   Complexity is D*n^2
*/

#include "VFAT/NPclassify2.H"

#include "Util/Assert.H"
#include "Util/Timer.H"
#include "Util/log.H"
#include <fstream>
#include <iostream>
#include <math.h>

/************************************************************************/
// *****************************************
// PUBLIC METHODS
// *****************************************
/************************************************************************/
template <class FLOAT>
NPclassify2<FLOAT>::NPclassify2(readConfig &settings, readConfig &polySet,
                       bool commandLineSettings)
{
  NPsetup(settings,polySet,commandLineSettings);
}

/************************************************************************/

template <class FLOAT>
NPclassify2<FLOAT>::NPclassify2()
{}

/************************************************************************/

template <class FLOAT>
NPclassify2<FLOAT>::~NPclassify2()
{}

/***********************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPsetup(readConfig &settings, readConfig &polySet,
                                 bool commandLineSettings)
{
  NP_spaceSize = 0;
  NP_uspaceSize = 0;
  NP_stems = 0;
  NP_roots = 0;
  NP_lowDensCount = 0;
  // set up variables from config file
  NP_CLS = commandLineSettings;
  LINFO("SETTING NPclassify variables from readConfig");
  NP_defaultSize = (int)settings.getItemValueF("defaultSize");
  NP_Con1 = settings.getItemValueF("Con1");
  NP_Con2 = settings.getItemValueF("Con2");
  NP_Con3 = settings.getItemValueF("Con3");
  NP_hardClassSize = (int)settings.getItemValueF("hardClassSize");
  NP_hardLinkSize = (int)settings.getItemValueF("hardLinkSize");
  NP_DWeight1 = settings.getItemValueF("DWeight1");
  NP_DWeight2 = settings.getItemValueF("DWeight2");
  NP_CWeight1 = settings.getItemValueF("CWeight1");
  NP_CWeight2 = settings.getItemValueF("CWeight2");

  NP_IDWeight1 = settings.getItemValueF("IDWeight1");
  NP_IDWeight2 = settings.getItemValueF("IDWeight2");
  NP_ICWeight1 = settings.getItemValueF("ICWeight1");
  NP_ICWeight2 = settings.getItemValueF("ICWeight2");
  NP_DenWeight1 = settings.getItemValueF("DenWeight1");
  NP_DenWeight2 = settings.getItemValueF("DenWeight2");
  NP_preDenWeight1 = settings.getItemValueF("preDenWeight1");
  NP_preDenWeight2 = settings.getItemValueF("preDenWeight2");
  NP_enthardClassSize = settings.getItemValueF("enthardClassSize");
  NP_enthardLinkSize = settings.getItemValueF("enthardLinkSize");
  NP_entDWeight1 = settings.getItemValueF("entDWeight1");
  NP_entDWeight2 = settings.getItemValueF("entDWeight2");
  NP_entCWeight1 = settings.getItemValueF("entCWeight1");
  NP_entCWeight2 = settings.getItemValueF("entCWeight2");
  NP_entIDWeight1 = settings.getItemValueF("entIDWeight1");
  NP_entIDWeight2 = settings.getItemValueF("entIDWeight2");
  NP_entICWeight1 = settings.getItemValueF("entICWeight1");
  NP_entICWeight2 = settings.getItemValueF("entICWeight2");
  NP_entDenWeight1 = settings.getItemValueF("entDenWeight1");
  NP_entDenWeight2 = settings.getItemValueF("entDenWeight2");
  NP_entpreDenWeight1 = settings.getItemValueF("entpreDenWeight1");
  NP_entpreDenWeight2 = settings.getItemValueF("entpreDenWeight2");

  NP_trainChildWeight = settings.getItemValueF("trainChildWeight");
  NP_doLinkMap = (int)settings.getItemValueF("doLinkMap");
  NP_doDensityMap = (int)settings.getItemValueF("doDensityMap");
  NP_doClassMap = (int)settings.getItemValueF("doClassMap");
  NP_usePolySet = (int)settings.getItemValueF("usePolySet");
  LINFO("SETTING kernel poly variables from polySet");
  NP_polyDensObjectCut1 = polySet.getItemValueF("polyDensObjectCut1");
  NP_polyDensObjectCut2 = polySet.getItemValueF("polyDensObjectCut2");
  NP_polyDensObjectCut3 = polySet.getItemValueF("polyDensObjectCut3");
  NP_polySpaceChildCut1 = polySet.getItemValueF("polySpaceChildCut1");
  NP_polySpaceChildCut2 = polySet.getItemValueF("polySpaceChildCut2");
  NP_polySpaceChildCut3 = polySet.getItemValueF("polySpaceChildCut3");
  LINFO("DONE");
  // set up vectors to a reasonable initial size
  //NPresizeSpace();
  // zero some initial counters
  //NPresetSpace();
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPinputCommandLineSettings(FLOAT inVals[31])
{
  NP_DWeight1           = inVals[0]; /*used*/
  NP_DWeight2           = inVals[1]; /*used*/
  NP_CWeight1           = inVals[2]; /*used*/
  NP_CWeight2           = inVals[3]; /*used*/
  NP_hardClassSize      = (int)inVals[4]; /*used*/
  NP_hardLinkSize       = (int)inVals[5]; /*used*/
  NP_IDWeight1          =  inVals[6]; /*used*/
  NP_IDWeight2          = inVals[7]; /*used*/
  NP_ICWeight1          = inVals[8]; /*used*/
  NP_ICWeight2          = inVals[9]; /*NOT used*/
  NP_DenWeight1         = inVals[10]; /*used*/
  NP_DenWeight2         = inVals[11]; /*NOT used*/
  NP_preDenWeight1      = inVals[12]; /*NOT used*/
  NP_preDenWeight2      = inVals[13]; /*NOT used*/
  NP_polyDensObjectCut1 = inVals[14]; /*used*/
  NP_polyDensObjectCut2 = inVals[15]; /*used*/
  NP_polyDensObjectCut3 = inVals[16]; /*used*/

  NP_entDWeight1        = inVals[17]; /*used*/
  NP_entDWeight2        = inVals[18]; /*used*/
  NP_entCWeight1        = inVals[19]; /*used*/
  NP_entCWeight2        = inVals[20]; /*used*/
  NP_enthardClassSize   = inVals[21]; /*used*/
  NP_enthardLinkSize    = inVals[22]; /*used*/
  NP_entIDWeight1       = inVals[23]; /*used*/
  NP_entIDWeight2       = inVals[24]; /*used*/
  NP_entICWeight1       = inVals[25]; /*used*/
  NP_entICWeight2       = inVals[26]; /*NOT used*/
  NP_entDenWeight1      = inVals[27]; /*used*/
  NP_entDenWeight2      = inVals[28]; /*used*/
  NP_entpreDenWeight1   = inVals[29];  /*NOT used*/
  NP_entpreDenWeight2   = inVals[30];  /*NOT used*/
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPaddPoint(std::vector<int> point)
{
  //FOO
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPaddSpace(typename
                                    std::vector<std::vector<FLOAT> > &space)
{
  NP_Space = &space;
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPechoSpace()
{
  for(int i = 0; i < NP_spaceSize; i++)
  {
    std::cerr << i << " ";
    for(unsigned int j = 0; j < NP_Space->at(i).size(); j++)
      std::cerr << NP_Space->at(i)[j] << " ";
    std::cerr << "\n";
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPresetSpace(int features, int dims)
{
  std::cerr << "SPACE RESET new size " << features << " x " << dims << "\n";
  NP_stems        = 0;
  NP_roots        = 0;
  NP_lowDensCount = 0;
  NP_useBias      = false;
  for(int i = 0; i < NP_spaceSize; i++)
  {
    NP_classSize[i] = 0;
    NP_decCount[i]  = 0;
    NP_children[i]  = 0;
    NP_distance[i]  = 0.0F;
    NP_density[i]   = 0.0F;
    NP_entropy[i]   = 0.0F;
    NP_predCount[i] = 0;
    NP_master[i]    = &NP_ZERO;
    NP_masterIndexCount[i] = 0;
    NP_subDecCount[i]      = 0;

  }
  NP_spaceSize = 0;  NP_uspaceSize = 0;
  NPparamSpace(features,dims);
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPresetSpace()
{
  std::cerr << "SPACE RESET\n";
  NP_stems = 0;
  NP_roots = 0;
  NP_lowDensCount = 0;
  NP_useBias = false;
  for(int i = 0; i < NP_spaceSize; i++)
  {
    NP_classSize[i] = 0;
    NP_decCount[i]  = 0;
    NP_children[i] = 0;
    NP_density[i] = 0.0F;
    NP_entropy[i] = 0.0F;
    NP_distance[i] = 0.0F;
    NP_predCount[i] = 0;
    NP_master[i] = &NP_ZERO;
    NP_masterIndexCount[i] = 0;
    NP_subDecCount[i] = 0;
  }
}

/************************************************************************/
template <class FLOAT>
void NPclassify2<FLOAT>::NPsetConvolveBias(
                            std::vector<covHolder<double> > *ch,
                            int listSize,
                            FLOAT bias)
{
  NP_useBias       = true;
  NP_biasWeight    = bias;
  NP_covHolder     = ch;
  NP_covHolderSize = listSize;
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPclassifySpaceNew(bool doBias = true)
{
  std::cerr << "SPACE SIZE = " << NP_spaceSize << "\n";
  Timer tim;
  tim.reset();
  int t1,t2;
  int t0 = tim.get();  // to measure display time
  std::cerr << "\tTIME: " << t0 << "ms\n";
  std::cerr << "CONVOLVE SPACE\n";
  NPconvolveSpace();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: " << t2 << "ms\n";
  if((NP_useBias == true) && (doBias == true))
  {
    std::cerr << "BIAS SPACE\n";
    NPbiasConvolveSpace();
    t1 = tim.get();
    t2 = t1 - t0;
    std::cerr << "\tTIME: " << t2 << "ms\n";
  }
  std::cerr << "LINK SPACE\n";
  NPlinkSpace();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: "<< t2 << "ms\n";
  std::cerr << "MAP SPACE\n";
  NPmapSpace();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: "<< t2 << "ms\n";
  std::cerr << "ANALYZE SPACE\n";
  NPanalizeSpace();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: "<< t2 << "ms\n";
  std::cerr << "EVOLVE SPACE\n";
  NPevolveSpace();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: "<< t2 << "ms\n";
  std::cerr << "ANALYZE INTER SPACE\n";
  NPanalizeInterSpace();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: "<< t2 << "ms\n";
  std::cerr << "EVOLVE INTER SPACE\n";
  NPevolveInterSpace();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: "<< t2 << "ms\n";
  std::cerr << "ANALYZE CLASS DENSITY\n";
  NPanalizeClassDensity();
  t1 = tim.get();
  t2 = t1 - t0;
  std::cerr << "\tTIME: "<< t2 << "ms\n";
}
 /************************************************************************/

template <class FLOAT>
FLOAT NPclassify2<FLOAT>::NPclassifySpaceKmeans(unsigned int *K,
                                                FLOAT *minDiff,
                                                unsigned int *stopIter)
{
  // set this for consistancy with other methods
  NP_stems = *K;
  // counter for iterations
  int iter = 0;
  // holds Kmean values
  typename std::vector<FLOAT> tempVec(NP_dimSize,0.0F);
  NP_Kmean.resize(*K,tempVec);

  LINFO("KMEAN - Space resized");
  //first assign everyone to a random class
  FLOAT temp;
  temp = *K;
  LINFO("classes %f",temp);
  for(int i = 0; i < NP_spaceSize; i++)
  {
    int tempClass  = (int)((temp*rand())/(RAND_MAX+1.0F));
    int *classsize = &NP_classSize[tempClass];
    NP_Class[tempClass][*classsize] = &NP_keyVal[i];
    *classsize     = *classsize+1;
    NP_master[i]   = &NP_keyVal[tempClass];
  }
  LINFO("KMEAN - Random classes assigned");
  //iterate until convergance
  bool doThis = true;
  FLOAT error = -1.0F;

  while(doThis)
  {
    //compute K mean
    LINFO("...KMEAN - Iteration %d",iter);
    for(int i = 0; i < (signed)*K; i++)
    {
      int *classsize = &NP_classSize[i];
      if(*classsize > 0)
      {
        for(int d = 0; d < NP_dimSize; d++)
        {
          FLOAT num = 0;
          for(int j = 0; j < *classsize; j++)
          {
            num += NP_Space->at(*NP_Class[i][j])[d];
          }
          NP_Kmean[i][d] = num/(*classsize);
        }
      }
    }
    LINFO("...KMEAN - Means Calculated");
    std::vector<int> tempClassSize(*K,0);
    FLOAT tempError = 0;
    // compute distance from points to each K mean
    for(int ix = 0; ix < NP_spaceSize; ix++)
    {
      //iterate over every point to the means
      for(int iy = 0; iy < (signed)*K; iy++)
      {
        FLOAT *npdis = &NP_Dis[ix][iy];
        *npdis = 0;
        // get distance between point and k-mean
        for(int px = 0; px < NP_dimSize; px++)
        {
          *npdis = *npdis +
            pow((NP_Space->at(ix)[px] - NP_Kmean[iy][px]),2);
        }
        *npdis = sqrt(*npdis);
      }

      // find error (the sum of all distances from any point to it's
      // class mean
      tempError += NP_Dis[ix][*NP_master[ix]];
    }
    LINFO("...KMEAN - Distance to Means Checked ERROR %f",tempError);
    // check error v. previous error
    // run if error is still high or iterations have not been surpassed
    // and if it is the first iteration, always run.
    if(((error < 0) || ((error - tempError) > *minDiff)) &&
       (iter < (signed)*stopIter))
    {
      for(int ix = 0; ix < NP_spaceSize; ix++)
      {
        int *np_master = NP_master[ix];
        // re-assign me to a new k-mean class
        for(int iy = 0; iy < (signed)*K; iy++)
        {
          NP_classSize[*K] = 0;
          if(NP_Dis[ix][iy] < NP_Dis[ix][*np_master])
          {
            np_master = &NP_keyVal[iy];
          }
        }
        //re-build class list
        NP_Class[*np_master][tempClassSize[*np_master]] = &NP_keyVal[ix];
        NP_classIndex[ix] = np_master;
        tempClassSize[*np_master]++;
      }
      // resize the classes for all the new points
      for(int iy = 0; iy < (signed)*K; iy++)
      {
        NP_classSize[iy] = tempClassSize[iy];
      }
      iter++;
    }
    else
    {
      doThis = false;
    }
  }
  return error;
}


/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPclassifySpacePartial()
{}

/************************************************************************/

template <class FLOAT>
int NPclassify2<FLOAT>::NPgetStemNumber()
{
  return NP_stems;
}

/************************************************************************/

template <class FLOAT>
int NPclassify2<FLOAT>::NPgetStemNumberEdit()
{
  int min = 0;
  for(int i = 0; i < NPgetStemNumber(); i++)
  {
    if(NPgetClassSize(i) > NPgetMinClassSize())
    {
      min++;
    }
  }
  return min;
}

/************************************************************************/

template <class FLOAT>
FLOAT NPclassify2<FLOAT>::NPgetMaxDensity()
{
  return NP_maxDensity;
}

/************************************************************************/

template <class FLOAT>
FLOAT NPclassify2<FLOAT>::NPgetMeanDensity()
{
  return NP_meanDensity;
}

/************************************************************************/

template <class FLOAT>
FLOAT NPclassify2<FLOAT>::NPgetStdDensity()
{
  return NP_stdDensity;
}

/************************************************************************/

template <class FLOAT>
typename std::vector<FLOAT>* NPclassify2<FLOAT>::NPgetMeanClassDensity()
{
  return &NP_meanClassDensity;
}

/************************************************************************/

template <class FLOAT>
typename std::vector<FLOAT>* NPclassify2<FLOAT>::NPgetStdClassDensity()
{
  return &NP_stdClassDensity;
}

/************************************************************************/

template <class FLOAT>
FLOAT NPclassify2<FLOAT>::NPgetEntropy()
{
  return NP_totalEntropy;
}

/************************************************************************/

template <class FLOAT>
bool NPclassify2<FLOAT>::NPisLowDensity(int item)
{
  return NP_lowDensity[item];
}

/************************************************************************/

template <class FLOAT>
bool NPclassify2<FLOAT>::NPisStem(int item)
{
  return NP_revStem[item];
}

/************************************************************************/

template <class FLOAT>
std::vector<FLOAT> NPclassify2<FLOAT>::NPgetDensity()
{
  return NP_density;
}

/************************************************************************/

template <class FLOAT>
std::vector<FLOAT>* NPclassify2<FLOAT>::NPgetDensityPtr()
{
  return &NP_density;
}

/************************************************************************/

template <class FLOAT>
std::vector<int*> NPclassify2<FLOAT>::NPgetStems()
{
  return NP_stem;
}

/************************************************************************/

template <class FLOAT>
int NPclassify2<FLOAT>::NPgetClassSize(int _Class)
{
  ASSERT((_Class <= (signed)NP_classSize.size()) && (_Class >= 0));
  return NP_classSize[_Class];
}

/************************************************************************/

template <class FLOAT>
int NPclassify2<FLOAT>::NPgetMinClassSize()
{
  return NP_hardClassSize;
}

/************************************************************************/

template <class FLOAT>
std::vector<std::vector<int*> >* NPclassify2<FLOAT>::NPgetClass()
{
  return &NP_Class;
}

/************************************************************************/

template <class FLOAT>
std::vector<int*> NPclassify2<FLOAT>::NPgetClass(int _Class)
{
  ASSERT((_Class <= (signed)NP_Class.size()) && ((_Class >= 0)));
  return NP_Class[_Class];
}

/************************************************************************/

template <class FLOAT>
int NPclassify2<FLOAT>::NPgetClass(int _Class, int item)
{
  ASSERT((_Class <= (signed)NP_Class.size()) && ((_Class >= 0)));
  ASSERT((item <= (signed)NP_Class[item].size()) && ((item >= 0)));
  return *NP_Class[_Class][item];
}

/************************************************************************/

template <class FLOAT>
FLOAT NPclassify2<FLOAT>::NPgetFeature(int m_feature_index, int n_vector_index)
{
  ASSERT((m_feature_index <= (signed)NP_Space->size()) && (m_feature_index >= 0));
  ASSERT((n_vector_index <= (signed)NP_Space->at(m_feature_index).size())
         && (n_vector_index >= 0));
  return NP_Space->at(m_feature_index)[n_vector_index];
}

/************************************************************************/

template <class FLOAT>
std::vector<int*> NPclassify2<FLOAT>::NPgetParents()
{
  return NP_parent;
}

/************************************************************************/

template <class FLOAT>
std::vector<std::vector<int*> > NPclassify2<FLOAT>::NPgetChildren()
{
  return NP_childMap;
}

/************************************************************************/

template <class FLOAT>
std::vector<std::vector<int> > NPclassify2<FLOAT>::NPgetBoundingBoxes(
bool ignore)
{
  std::vector<int> box(4,0);
  std::vector<std::vector<int> > boundingBox(NP_stems,box);


  for(int ix = 0; ix < NP_spaceSize; ix++)
  {
    if((NP_revStem[ix] == false) || (ignore))
    {
      if((NP_lowDensity[ix] == false) || (ignore))
        //&& (classSize[classIndexIndex[ix]] > hardClassSize))
      {
        //std::cerr << *NP_classIndex[ix] << " " << ix << "\n";
        if(boundingBox[*NP_classIndex[ix]][0] == 0)
          boundingBox[*NP_classIndex[ix]][0] = (int)NP_Space->at(ix)[0];
        else
          if(boundingBox[*NP_classIndex[ix]][0] < NP_Space->at(ix)[0])
            boundingBox[*NP_classIndex[ix]][0] = (int)NP_Space->at(ix)[0];
        if(boundingBox[*NP_classIndex[ix]][1] == 0)
          boundingBox[*NP_classIndex[ix]][1] = (int)NP_Space->at(ix)[1];
        else
          if(boundingBox[*NP_classIndex[ix]][1] < NP_Space->at(ix)[1])
            boundingBox[*NP_classIndex[ix]][1] = (int)NP_Space->at(ix)[1];
        if(boundingBox[*NP_classIndex[ix]][2] == 0)
          boundingBox[*NP_classIndex[ix]][2] = (int)NP_Space->at(ix)[0];
        else
          if(boundingBox[*NP_classIndex[ix]][2] > NP_Space->at(ix)[0])
            boundingBox[*NP_classIndex[ix]][2] = (int)NP_Space->at(ix)[0];
        if(boundingBox[*NP_classIndex[ix]][3] == 0)
          boundingBox[*NP_classIndex[ix]][3] = (int)NP_Space->at(ix)[1];
        else
          if(boundingBox[*NP_classIndex[ix]][3] > NP_Space->at(ix)[1])
            boundingBox[*NP_classIndex[ix]][3] = (int)NP_Space->at(ix)[1];
      }
    }
  }
  return boundingBox;
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPdrawBoundingBoxes(
                                      std::vector<std::vector<int> > *BB,
                                      Image<PixRGB<FLOAT> > *ibox,
                                      int offsetX, int offsetY,
                                      PixRGB<FLOAT> pix, bool ignorMin)
{
  LINFO("DRAWING %d STEMS",NPgetStemNumber());
  for(int i = 0; i < NPgetStemNumber(); i++)
  {
    if((NPgetClassSize(i) > NPgetMinClassSize()) || (ignorMin))
    {
      drawLine(*ibox, Point2D<int>(BB->at(i)[0]+offsetX,BB->at(i)[1]+offsetY),
               Point2D<int>(BB->at(i)[2]+offsetX,BB->at(i)[1]
                       +offsetY),pix,1);
      drawLine(*ibox, Point2D<int>(BB->at(i)[0]+offsetX,BB->at(i)[3]+offsetY),
               Point2D<int>(BB->at(i)[2]+offsetX,BB->at(i)[3]
                       +offsetY),pix,1);

      drawLine(*ibox, Point2D<int>(BB->at(i)[0]+offsetX,BB->at(i)[1]+offsetY),
               Point2D<int>(BB->at(i)[0]+offsetX,BB->at(i)[3]
                       +offsetY),pix,1);
      drawLine(*ibox, Point2D<int>(BB->at(i)[2]+offsetX,BB->at(i)[1]+offsetY),
               Point2D<int>(BB->at(i)[2]+offsetX,BB->at(i)[3]
                       +offsetY),pix,1);
      LINFO("DRAWING BOX for class %d size %d",i,NPgetClassSize(i));
    }
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPmetaClassify(int objects)
{
  // find the first n links that meet that training criteria
  // find their statistics
  // open file to save tab delim data to
  std::ofstream outfile("train_set.dat",std::ios::app);

  // create statistics

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++)
  {
    NP_trainMeasure[ix] = NP_distance[ix]*(NP_decCount[ix]/NP_trainChildWeight);
    NP_selected[ix] = false;
  }

  // find the n best candidates
  for(int n = 0; n < objects; n++)
  {
    int init = 0;
    for(unsigned int ix = 0; ix < NP_uspaceSize; ix++)
    {
      if(NP_selected[ix] == false)
      {
        if(init == 0)
        {
          NP_idealLinks[n] = ix;
          init = 1;
        }
        else
        {
          if(NP_trainMeasure[ix] > NP_trainMeasure[NP_idealLinks[n]])
          {
            NP_idealLinks[n] = ix;
          }
        }
      }
    }
    NP_selected[NP_idealLinks[n]] = true;
  }

  // find there stats as they apply to the current scene
  // find min child and distance for nodes
  int init = 0;
  for(int n = 0; n < objects; n++)
  {
    if(init == 0)
    {
      NP_minDist = NP_idealLinks[n];
      NP_minChild = NP_idealLinks[n];
      init = 1;
    }
    else
    {
      if(NP_distance[NP_idealLinks[n]] < NP_distance[NP_minDist])
      {
        NP_minDist = NP_idealLinks[n];
      }
      if(NP_decCount[NP_idealLinks[n]] <  NP_decCount[NP_minChild])
      {
        NP_minChild = NP_idealLinks[n];
      }
    }
  }
  int returnStems = NP_stems;
  for(int i = 0; i < NP_stems; i++)
  {
    if(NP_classSize[i] <= NP_hardClassSize)
    {
      --returnStems;
    }
  }

  NP_distanceCut = (NP_distance[NP_minDist] - NP_meanDistance)/NP_stdDistance;
  NP_childCut    = NP_decCount[NP_minChild]/NP_meanDecCount;

  outfile << NP_spaceSize << "\t" << NP_meanDensity << "\t"
          << objects << "\t" << NP_distanceCut << "\t"
          << NP_childCut << "\t" << returnStems << "\n";

  outfile.close();
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPdumpLinkInfo(std::string fileName)
{
  // each sample carries these pieces of information as well as
  // larger maps which are not used here
  typename std::vector<FLOAT>::iterator idistance    = NP_distance.begin();
  typename std::vector<FLOAT>::iterator idensity     = NP_density.begin();
  typename std::vector<FLOAT>::iterator ientropy     = NP_entropy.begin();
  std::vector<int*>::iterator   iparent              = NP_parent.begin();
  std::vector<int*>::iterator   irevStemVal          = NP_revStemVal.begin();
  std::vector<int*>::iterator   iclassIndex          = NP_classIndex.begin();
  std::vector<bool>::iterator   irevStem             = NP_revStem.begin();
  std::vector<bool>::iterator   ilowDensity          = NP_lowDensity.begin();
  std::vector<int>::iterator    ipredCount           = NP_predCount.begin();
  std::vector<int>::iterator    ichildren            = NP_children.begin();
  std::vector<int>::iterator    idecCount            = NP_decCount.begin();
  std::string myName = "NPfeatures." + fileName + ".txt";
  std::ofstream outfile(myName.c_str(),std::ios::out);
  outfile << "BASE STATS:\n";
  outfile << "SpaceSize\tDimSize\tStems\tRoots\tMinDistance\t"
          << "MinChild\tLowDensityCount\tMaxDensity\tMeanDistance\t"
          << "StdDistance\tMeanDensity\tStdDensity\tMeanChild\t"
          << "StdChild\tMeanDecCount\tStdDecCount\n";
  outfile << NP_spaceSize    << "\t" << NP_dimSize      << "\t"
          << NP_stems        << "\t" << NP_roots        << "\t"
          << NP_minDist      << "\t" << NP_minChild     << "\t"
          << NP_lowDensCount << "\t" << NP_maxDensity   << "\t"
          << NP_meanDistance << "\t" << NP_stdDistance  << "\t"
          << NP_meanDensity  << "\t" << NP_stdDensity   << "\t"
          << NP_meanChild    << "\t" << NP_stdChildren  << "\t"
          << NP_meanDecCount << "\t" << NP_stdDecCount  << "\n";

  outfile << "SAMPLE STATS:\n";
  outfile << "Distance\tDensity\tEntropy\tParent\tRevStemVal\t"
          << "ClassIndex\tRevStem\tLowDensity\tPredCount\tChildren\t"
          << "DecCount\n";
  for(int ix = 0; ix < NP_spaceSize; ix++, ++idistance, ++idensity,
        ++ientropy, ++iparent, ++irevStemVal, ++irevStem, ++ilowDensity,
        ++ipredCount, ++ichildren, ++idecCount, ++iclassIndex)
  {
    outfile << *idistance    << "\t" << *idensity     << "\t"
            << *ientropy     << "\t" << **iparent     << "\t"
            << **irevStemVal << "\t" << **iclassIndex << "\t"
            << *irevStem     << "\t" << *ilowDensity  << "\t"
            << *ipredCount   << "\t" << *ichildren    << "\t"
            << *idecCount    << "\n";
  }
  outfile.close();
}


/************************************************************************/
// *****************************************
// PRIVATE METHODS
// *****************************************
/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPconvolveSpace()
{
  // find distance between all points as an n^2 alg. Sum these for all
  // points to determine a points density measure
  std::vector<int>::iterator  ichildren = NP_children.begin();
  std::vector<bool>::iterator irevStem  = NP_revStem.begin();
  typename std::vector<std::vector<FLOAT>  >::iterator iDis = NP_Dis.begin();
  typename std::vector<std::vector<FLOAT*> >::iterator irevDis
    = NP_reverseDisMap.begin();
  typename std::vector<std::vector<FLOAT> >::iterator iSpaceX
    = NP_Space->begin();
  typename std::vector<std::vector<FLOAT> >::iterator iSpaceY;
  typename std::vector<FLOAT>::iterator  idensity = NP_density.begin();
  typename std::vector<FLOAT>::iterator  iiSpaceX;
  typename std::vector<FLOAT>::iterator  iiSpaceY;
  typename std::vector<FLOAT>::iterator  iiDis;
  typename std::vector<FLOAT*>::iterator iirevDis;

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++,
        ++ichildren, ++irevStem, ++iDis,
        ++idensity, ++iSpaceX, ++irevDis)
  {
    *ichildren = 0;
    *irevStem = false;
    //iterate over every tupple, find distance, copy to symetric tupple
    iiDis    = iDis->begin()     + ix + 1;
    iirevDis = irevDis->begin()  + ix + 1;
    iSpaceY  = NP_Space->begin() + ix + 1;
    for(unsigned int iy = ix+1; iy < NP_uspaceSize; iy++, ++iiDis, ++iSpaceY,
        ++iirevDis)
    {
      *iiDis   = 0;
      iiSpaceX = iSpaceX->begin();
      iiSpaceY = iSpaceY->begin();
      // get distance between these two points
      for(unsigned int px = 0; px < NP_udimSize; px++, ++iiSpaceX, ++iiSpaceY)
      {
        *iiDis = *iiDis + pow((*iiSpaceX - *iiSpaceY),2);
      }
      **iirevDis = *iiDis;
    }

    // find distance as sqrt(sum(sqared distances))
    // take the inverse distance 1/D
    iiDis = iDis->begin();
    for(unsigned int iy = 0; iy < NP_uspaceSize; iy++, ++iiDis)
    {
      if(*iiDis != 0)
        *idensity = (1/sqrt(*iiDis))+(*idensity);
    }
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPconvolveSpace(long item)
{
  // find distance between all points as an n alg. Sum these for all
  // points to determine a points density measure

  NP_children[item] = 0;
  NP_revStem[item]  = false;

  //iterate over every tupple, find distance, copy to symetric tupple
  for(int iy = 0; iy < NP_spaceSize; iy++)
  {
    NP_Dis[item][iy] = 0;
    FLOAT *npdis     = &NP_Dis[item][iy];
    // get distance between these two points
    for(int px = 0; px < NP_dimSize; px++)
    {
      *npdis = *npdis +
        pow((NP_Space->at(item)[px] - NP_Space->at(iy)[px]),2);
    }
    NP_Dis[iy][item] = *npdis;
  }

  // find distance as sqrt(sum(sqared distances))
  // take the inverse distance 1/D
  for(int iy = 0; iy < NP_spaceSize; iy++)
  {
    if(NP_Dis[item][iy] != 0)
    {
      NP_density[item] += 1/sqrt(NP_Dis[item][iy]);
      NP_density[iy]   += 1/sqrt(NP_Dis[item][iy]);
    }
  }
}


/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPbiasConvolveSpace()
{
  for(int ix = 0; ix < NP_spaceSize; ix++)
  {
    FLOAT distance = 0.0F;
    //iterate over every tupple, find distance, copy to symetric tupple
    for(int iy = 0; iy < NP_covHolderSize; iy++)
    {
      // get distance between these two points
      for(int px = 0; px < NP_dimSize; px++)
      {
        distance +=
          pow((NP_Space->at(ix)[px] - NP_covHolder->at(iy).mean[px]),2);
      }

      // find distance as sqrt(sum(sqared distances))
      // take the inverse distance 1/D
      if(distance != 0.0F)
        NP_density[ix] += (1/sqrt(distance))
          * NP_covHolder->at(iy).samples * NP_biasWeight;
    }
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPbasicMeanDensity()
{
  // find mean density
  FLOAT sumSquares = 0;
  typename std::vector<FLOAT>::iterator idensity = NP_density.begin();
  NP_sumDensity = 0;
  for(int ix = 0; ix < NP_spaceSize; ix++, ++idensity)
  {
    NP_sumDensity = *idensity + NP_sumDensity;
    sumSquares    += pow((*idensity),2)/(NP_spaceSize);
    if(*idensity > NP_maxDensity)
      NP_maxDensity = *idensity;
  }
  NP_meanDensity = NP_sumDensity/(NP_spaceSize);
  NP_stdDensity  = sqrt(sumSquares - pow(NP_meanDensity,2));
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPcomputeEntropy()
{
  NP_totalEntropy = 0.0F;
  typename std::vector<FLOAT>::iterator idensity = NP_density.begin();
  typename std::vector<FLOAT>::iterator ientropy = NP_entropy.begin();
  if(NP_sumDensity != 0)
  {
    for(int ix = 0; ix < NP_spaceSize; ix++, ++idensity, ++ientropy)
    {
      *ientropy = (*idensity/NP_sumDensity)*log(*idensity/NP_sumDensity);
      NP_totalEntropy += (*ientropy);
    }
  }
  NP_totalEntropy = -1*NP_totalEntropy;
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPlinkSpace()
{
  // link points together. Take the closest point that is higher in
  // density
  NPbasicMeanDensity();
  NPcomputeEntropy();

  FLOAT thresh = (NP_preDenWeight1*NP_meanDensity)
    + (NP_preDenWeight2*pow(NP_meanDensity,2));
  typename std::vector<FLOAT>::iterator idensity = NP_density.begin();
  std::vector<bool>::iterator ilowDensity        = NP_lowDensity.begin();
  for(int ix = 0; ix < NP_spaceSize; ix++, ++idensity, ++ilowDensity)
  {
    if(*idensity < thresh)
    {
      *ilowDensity = true;
      NP_lowDensCount++;
    }
    else
      *ilowDensity = false;
  }
  NP_newSampleSize = NP_spaceSize - NP_lowDensCount;

  /* link all nodes to the node which is closest but which has
     a higher density. Do this by comparing every node to every other
     node, This is O(n^2). Do not add any nodes to the tree which
     have a low density.
  */

  NPdoLinkSpace();
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPlinkSpaceIncr()
{
  /*
  // link points together. Take the closest point that is higher in
  // density
  NPbasicMeanDensity();
  NP_lowDensCount = 0;
  //std::cout << "meanDensity " << meanDensity << "\n";
  //std::cout << "stdDensity " << stdDensity << "\n";
  FLOAT thresh = (NP_preDenWeight1*NP_meanDensity)
    + (NP_preDenWeight2*pow(NP_meanDensity,2));
  for(int ix = 0; ix < NP_spaceSize; ix++)
  {
    if(NP_density[ix] < thresh)
    {
      NP_lowDensity[ix] = true;
      NP_lowDensCount++;
    }
    else
      NP_lowDensity[ix] = false;
  }
  NP_newSampleSize = NP_spaceSize - NP_lowDensCount;

  //std::cerr << "new sample size " <<  NP_newSampleSize << "\n";
  int temp1 = -1;
  FLOAT temp2 = -1.0F;
  int *best = &temp1;
  FLOAT *bestVal = &temp2;
  // link all nodes to the node which is closest but which has
  // a higher density. Do this by comparing every node to every other
  // node, This is O(n^2). Do not add any nodes to the tree which
  // have a low density.


  // two things must happen here
  //  (a) my parent now has a lower density than me, so I must totally
  //       update my links for this node and find a new parent
  //  (b) my parent is still better, I must check all nodes
  //       that had a lower density than me before hand to see if they
  //       now have a high density and are closer than my parent

  // My parent has a lower density than me, relink the whole list
  for(int ix = 0; ix < NP_spaceSize; ix++)
  {
    // (A) Check My parent
    if(NP_density[ix] > NP_density[NP_parent[ix]])
    {
      // relink
      NPdoLinkSpace(ix);
      // shrink list
      NPeditLessDensityList(ix);
      break;
    }
    // (B) check most of the guys who have a lower density
    tempCount = NP_lessDensityCount[ix];
    NP_lessDensityCount[ix] = 0;
    std::vector<int*>::iterator NP_lessDensityIter;
    NP_lessDensityIter = NP_lessDensity.at(ix);
    for(int in = 0; in < tempCount; in++)
    {
      if(NP_density[in] > NP_density[ix])
      {
        if((NP_Dis[ix][in] < *bestVal) || (*bestVal == -1))
        {
          bestVal = &NP_Dis[ix][in];
          best = &NP_keyVal[in];
        }
      }
      else
      {
        if((NP_Dis[ix][in] < *bestVal) || (*bestVal == -1))
        {
          NP_lessDensityIter** = &NP_keyVal[in];
          ++NP_lessDensityIter; NP_lessDensityCount[ix]++;
        }
      }
    }
    if(*best != -1)
    {
      // A change has been found, I now have a new parent
      //std::cerr << "Set non-stem best " << *best << "\n";
      // RESET my parents list
      long newCount = 0;
      for(int cm = 0; cm < NP_children[NP_parent[ix]]; cm++)
      {
        if(NP_childMap[NP_parent[ix]][cm] != NP_keyVal[ix])
        {
          NP_childMap[NP_parent[ix]][newCount]
            = NP_childMap[NP_parent[ix]][cm];
          newCount++;
        }
      }
      NP_children[NP_parent[ix]] = &NP_keyVal[newCount];

      // JOIN my new parents list
      NP_parent[ix] = &NP_keyVal[*best];
      NP_distance[ix] = NP_Dis[ix][*best];
      // add myself to my parents list of children
      // remove me from my old parent
      NP_childMap[*best][NP_children[*best]] = &NP_keyVal[ix];
      NP_children[*best]++;
      //std::cerr << "LINKING "
      //          << "This " << NP_keyVal[ix] << " to " << *NP_parent[ix]
      //          << " Childmap [" << *best << "][" << NP_children[*best]
      //          << "] distance " << NP_distance[ix] << "\n";
    }
  }
  */
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPdoLinkSpace()
{
  int temp1 = -1;
  FLOAT temp2 = -1.0F;
  int *best = &temp1;
  FLOAT *bestVal = &temp2;
  NP_stems = 0;
  /* link all nodes to the node which is closest but which has
     a higher density. Do this by comparing every node to every other
     node, This is O(n^2). Do not add any nodes to the tree which
     have a low density.
  */
  typename std::vector<FLOAT>::iterator  idensityX    = NP_density.begin();
  std::vector<bool>::iterator            ilowDensityX = NP_lowDensity.begin();
  typename std::vector<FLOAT>::iterator  idensityY;
  std::vector<bool>::iterator            ilowDensityY;
  typename std::vector<FLOAT>::iterator  iiDis;
  typename std::vector<FLOAT>::iterator  idistance    = NP_distance.begin();
  typename std::vector<std::vector<FLOAT> >::iterator iDis = NP_Dis.begin();
  std::vector<int>::iterator     ikeyValX     = NP_keyVal.begin();
  std::vector<int>::iterator     ikeyValY;
  std::vector<int*>::iterator    iparent      = NP_parent.begin();
  std::vector<int*>::iterator    irevStemVal  = NP_revStemVal.begin();
  std::vector<bool>::iterator    irevStem     = NP_revStem.begin();
  for(int ix = 0; ix < NP_spaceSize; ix++, ++idensityX,++ilowDensityX, ++iDis,
        ++iparent, ++idistance, ++irevStemVal, ++ikeyValX)
  {
    if(*ilowDensityX == false)
    {
      best         = &temp1;
      bestVal      = &temp2;
      idensityY    = NP_density.begin();
      ilowDensityY = NP_lowDensity.begin();
      iiDis        = iDis->begin();
      ikeyValY     = NP_keyVal.begin();
      for(int iy = 0; iy < ix; iy++, ++idensityY, ++ilowDensityY, ++iiDis,
            ++ikeyValY)
      {
        if(*ilowDensityY == false)
        {
          if(*idensityY > *idensityX)
          {
            if((*iiDis < *bestVal) || (*bestVal == -1))
            {
              bestVal = &*iiDis;
              best    = &*ikeyValY;
            }
          }
        }
      }
      int offset   = ix + 1;
      idensityY    = NP_density.begin() + offset;
      ilowDensityY = NP_lowDensity.begin() + offset;
      iiDis        = iDis->begin() + offset;
      ikeyValY     = NP_keyVal.begin() + offset;
      for(int iy = ix+1; iy < NP_spaceSize; iy++, ++idensityY, ++ilowDensityY,
            ++iiDis, ++ikeyValY)
      {
        if(*ilowDensityY == false)
        {
          if(*idensityY > *idensityX)
          {
            if((*iiDis < *bestVal) || (*bestVal == -1))
            {
              bestVal = &*iiDis;
              best    = &*ikeyValY;
            }
          }
        }
      }

      if(*best != -1)
      {
        *iparent   = &NP_keyVal[*best];
        *idistance = iDis->at(*best);
        // add myself to my parents list of children
        NP_childMap[*best][NP_children[*best]] = &*ikeyValX;
        NP_children[*best]++;
      }
      else
      {
        NP_stem[NP_stems] = &*ikeyValX;
        *irevStemVal      = &NP_keyVal[NP_stems];
        *iparent          = &*ikeyValX;
        *irevStem         = true;
        NP_stems++;
      }
    }
  }
}

/************************************************************************/
/*
template <class FLOAT>
void NPclassify2<FLOAT>::NPdoLinkSpaceIter(int item)
{

  int temp1 = -1;
  FLOAT temp2 = -1.0F;
  int *best = &temp1;
  FLOAT *bestVal = &temp2;
  NP_stems = 0;
     link all nodes to the node which is closest but which has
     a higher density. Do this by comparing every node to every other
     node, This is O(n^2). Do not add any nodes to the tree which
     have a low density.

  std::vector<std::vector<int*> >::iterator NP_lessDensityIter;
  std::vector<int*>::iterator NP_lessDensityIterIter;
  std::vector<int>::iterator NP_lessDensityCountIter;

  NP_lessDensityIter       = NP_lessDensity.at(item);
  NP_lessDensityCountIter  = NP_lessDensityCount.at(item);
  *NP_lessDensityCountIter = 0;
  NP_lessDensityIterIter   = NP_lessDensityIter->begin();

  for(int iy = 0; iy < item; iy++)
  {
    if(NP_density[iy] > NP_density[item])
    {
      if((NP_Dis[item][iy] < *bestVal) || (*bestVal == -1))
      {
        bestVal = &NP_Dis[item][iy];
        best = &NP_keyVal[iy];
      }
    }
    else
    {
      if(NP_Dis[item][iy] < *bestVal) || (*bestVal == -1))
      {
        **NP_lessDensityIterIter = &NP_keyVal[iy];
        ++NP_lessDensityIterIter; NP_lessDensityCountIter* += 1;
      }
    }
  }
  for(int iy = item+1; iy < NP_spaceSize; iy++)
  {
    if(NP_density[iy] > NP_density[item])
    {
      if((NP_Dis[item][iy] < *bestVal) || (*bestVal == -1))
      {
        bestVal = &NP_Dis[item][iy];
        best = &NP_keyVal[iy];
      }
    }
    else
    {
      if(NP_Dis[item][iy] < *bestVal) || (*bestVal == -1))
      {
        **NP_lessDensityIterIter = &NP_keyVal[iy];
        ++NP_lessDensityIterIter; NP_lessDensityCountIter* += 1;
      }
    }
  }

  if(*best != -1)
  {
    NP_parent[item] = &NP_keyVal[*best];
    NP_distance[item] = NP_Dis[item][*best];
    // add myself to my parents list of children
    NP_childMap[*best][NP_children[*best]] = &NP_keyVal[item];
    NP_children[*best]++;
  }
  else
  {
    //LINFO("STEM SET - as default");
    NP_stem[NP_stems] = &NP_keyVal[item];
    NP_revStemVal[item] = &NP_keyVal[NP_stems];
    NP_parent[item] = &NP_keyVal[item];
    NP_revStem[item] = true;
    NP_stems++;
  }
  //}
  //
*/


/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPmapSpace()
{
  /* This will map all decendants to their parent nodes such that
     a list is built which contains each decentant for every node.
     that is, line 43 would have all decendants listed for node 43.
     this is done in n(log(n)) time by having a node place it's self
     in every predecessor(parent) above it. In essence by exploring
     the tree upwards (bottom up).
     Also: build a list of all my predecessors at the same time
  */
  std::vector<bool>::iterator ilowDensity = NP_lowDensity.begin();
  std::vector<int>::iterator  ikeyVal     = NP_keyVal.begin();
  std::vector<int>::iterator  ipredCount  = NP_predCount.begin();
  std::vector<std::vector<int*> >::iterator ipred = NP_pred.begin();

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++, ++ilowDensity,
        ++ikeyVal,++ipredCount, ++ipred)
  {
    if(*ilowDensity == false)
    {
      bool stop = false;
      int *currentParent = NP_parent[ix];
      while(stop == false)
      {
        for(int sc = 0; sc < NP_stems; sc++)
        {
          if(currentParent == NP_stem[sc])
          {
            stop = true;
            break;
          }
        }
        // who am I a decendant to, add me to their list
        NP_decend[*currentParent]
          [NP_decCount[*currentParent]] = &*ikeyVal;
        NP_decCount[*currentParent]++;
        // who are my predicessors, add them to my list
        ipred->at(*ipredCount) = &NP_keyVal[*currentParent];
        *ipredCount            = *ipredCount + 1;
        currentParent          = NP_parent[*currentParent];
      }
    }
  }
}

/************************************************************************/
template <class FLOAT>
void NPclassify2<FLOAT>::NPanalizeSpace()
{
  FLOAT sumD = 0; double sumSquaresD = 0;
  FLOAT sumC = 0; double sumSquaresC = 0;
  FLOAT sumO = 0; double sumSquaresO = 0;
  typename std::vector<FLOAT>::iterator idistance   = NP_distance.begin();
  std::vector<int>::iterator    ichildren           = NP_children.begin();
  std::vector<int>::iterator    idecCount           = NP_decCount.begin();
  std::vector<bool>::iterator   ilowDensity         = NP_lowDensity.begin();
  // find mean distance between linked nodes

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++, ++idistance, ++ichildren,
        ++idecCount, ++ilowDensity)
  {
    if(*ilowDensity == false)
    {
      // find the mean distances of links
      sumD        += *idistance;
      sumSquaresD += pow((*idistance),2)/NP_newSampleSize;

      // find the mean number of children
      sumC        += *ichildren;
      sumSquaresC += pow(((FLOAT)*ichildren),2)/NP_newSampleSize;

      // find mean number of decendants
      sumO        += *idecCount;  // previously childMapTot, childMap*
      sumSquaresO += pow(((FLOAT)*idecCount),2)/NP_newSampleSize;
    }
  }
  if(NP_newSampleSize == 0)
    NP_newSampleSize = 1;

  NP_meanDistance = sumD/NP_newSampleSize;
  NP_stdDistance  = sqrt(sumSquaresD - pow(NP_meanDistance,2));

  NP_meanChild    = sumC/NP_newSampleSize;
  NP_stdChildren  = sqrt(sumSquaresC - pow(NP_meanChild,2));

  NP_meanDecCount = sumO/NP_newSampleSize;
  NP_stdDecCount  = sqrt(sumSquaresO - pow(NP_meanDecCount,2));

}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPevolveSpace()
{

  //initial threshold is a function of standard deviation and a weight
  //and number of children and a weight for that
  std::vector<bool>::iterator   ilowDensity = NP_lowDensity.begin();
  std::vector<bool>::iterator   irevStem    = NP_revStem.begin();
  std::vector<int>::iterator    ikeyVal     = NP_keyVal.begin();
  std::vector<int>::iterator    idecCount   = NP_decCount.begin();
  std::vector<int*>::iterator   irevStemVal = NP_revStemVal.begin();
  typename std::vector<FLOAT>::iterator idistance   = NP_distance.begin();
  typename std::vector<FLOAT>::iterator idensity    = NP_density.begin();


  // use polynomial optimized (linear regression) thresholds
  FLOAT threshDensity = (NP_polyDensObjectCut1*NP_meanDensity+
               NP_polyDensObjectCut2*pow(NP_meanDensity,(FLOAT)2.0)+
               NP_polyDensObjectCut3*pow(NP_meanDensity,(FLOAT)3.0)) /
    (NP_stdDensity+1);

  FLOAT threshDecend = (((NP_entCWeight1*NP_totalEntropy)+NP_CWeight1)
                         *NP_meanDecCount) -
    (((NP_entCWeight2*NP_totalEntropy)+NP_CWeight2)
     *pow(NP_meanDecCount,2));

  FLOAT threshDensity2 = (((NP_entDenWeight1*NP_totalEntropy)+NP_DenWeight1)
                           *NP_stdDensity)
    + (((NP_entDenWeight2*NP_totalEntropy)+NP_DenWeight2)
       *pow(NP_stdDensity,2));

  FLOAT threshDistance = (((NP_entDWeight1*NP_totalEntropy)+NP_DWeight1)
                           *NP_stdDistance)
    + (((NP_entDWeight2*NP_totalEntropy)+NP_DWeight2)
       *pow(NP_stdDistance,2));


  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++, ++ilowDensity, ++irevStem,
        ++ikeyVal, ++idecCount, ++irevStemVal, ++idistance, ++idensity)
  {
    if(*ilowDensity == false)
    {
      FLOAT measureDistance = *idistance      - NP_meanDistance;
      FLOAT measureDensity  = *idensity       - NP_meanDensity;
      FLOAT measureDensity2 = NP_meanDensity  - *idensity;

      if((measureDensity < 0) && (measureDensity < threshDensity2))
        *ilowDensity = true;

      // How many std are you from the mean link distance. Use a weight that
      // signifies how far off you can be
      // look at both intre and inter class variance

      if((((measureDistance > threshDistance) ||
           (measureDensity2 > threshDensity))
          &&
          (*idecCount > threshDecend)) ||
         (*idistance > ( NP_hardLinkSize
                         + ( NP_enthardLinkSize*NP_totalEntropy ))))
      {
        if(*ilowDensity == false)
        {
          if(*irevStem == false)
          {
            // how many grandchildren do you have compaired with the mean
            // make sure you have lots. Remove outliers
            NP_stem[NP_stems] = &*ikeyVal;
            *irevStemVal      = &NP_keyVal[NP_stems];
            NP_stems++;
            *irevStem         = true;
          }
        }
      }
    }
  }
  //std::cout << "FIRST ROUND STEMS: " << NP_stems << "\n";
  NPcomputeMasters();
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPanalizeInterSpace()
{
  // find masters, the class clusters
  // compute interclass variance and mean
  for(int i = 0; i < NP_stems; i++)
  {
    NP_meanInterDistance[i] = 0;
    NP_stdInterDistance[i]  = 0;
    NP_meanInterChild[i]    = 0;
    NP_stdInterChild[i]     = 0;
    NP_interEntropy[i]      = 0;

    if(NP_classSize[i] == 0)
      NP_classSize[i] = 1;
  }


  // find mean for interclass statistics
  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++)
  {
    //int *index = NP_classIndex[*NP_master[ix]];
    int *index = NP_classIndex[ix];
    // count how many decendants I have inside my class
    for(int d = 0; d < NP_decCount[ix]; d++)
    {
      if(NP_master[ix] == NP_master[*NP_decend[ix][d]])
      {
        NP_subDecCount[ix]++;
      }
    }

    if((NP_revStem[ix] == false) && (NP_lowDensity[ix] == false))
    {
      NP_meanInterDistance[*index] += NP_distance[ix];
      NP_meanInterDensity[*index]  += NP_density[ix];
      NP_meanInterChild[*index]    += NP_subDecCount[ix];
    }
  }

  for(int i = 0; i < NP_stems; i++)
  {
    // copy over before computing full mean
    NP_sumInterDensity[i]   = NP_meanInterDensity[i];
    NP_meanInterDistance[i] = NP_meanInterDistance[i]/NP_classSize[i];
    NP_meanInterDensity[i]  = NP_meanInterDensity[i] /NP_classSize[i];
    NP_meanInterChild[i]    = NP_meanInterChild[i]   /NP_classSize[i];
  }

  // compute entropy per inter class
  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++)
  {
    int *index = NP_classIndex[ix];

    if((NP_revStem[ix] == false) && (NP_lowDensity[ix] == false))
    {
      NP_interEntropy[*index] +=
        (NP_density[ix]/NP_sumInterDensity[*index])
        *log(NP_density[ix]/NP_sumInterDensity[*index]);
    }
  }

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++)
  {
    //int *index = NP_classIndex[*NP_master[ix]];
    int *index = NP_classIndex[ix];
    if((NP_revStem[ix] == false) && (NP_lowDensity[ix] == false))
    {
      NP_stdInterDistance[*index]
        += pow((NP_meanInterDistance[*index] - NP_distance[ix]),2);
      NP_stdInterDensity[*index]
        += pow((NP_meanInterDensity[*index]  - NP_density[ix]),2);
      NP_stdInterChild[*index]
        += pow((NP_meanInterChild[*index]    - NP_subDecCount[ix]),2);
    }
  }

  for(int i = 0; i < NP_stems; i++)
  {
    NP_stdInterDistance[i] = sqrt(NP_stdInterDistance[i]/NP_classSize[i]);
    NP_stdInterDensity[i]  = sqrt(NP_stdInterDensity[i] /NP_classSize[i]);
    NP_stdInterChild[i]    = sqrt(NP_stdInterChild[i]   /NP_classSize[i]);
    NP_interEntropy[i]     = -1.0F*NP_interEntropy[i];
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPevolveInterSpace()
{
  //initial threshold is a function of standard deviation and a weight
  //and number of children and a weight for that

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++)
  {
    int *index = NP_classIndex[*NP_master[ix]];
    if(NP_revStem[ix] == false)
    {
      FLOAT threshDistance = (((NP_entIDWeight1*NP_interEntropy[*index])
                               +NP_IDWeight1)
                              *NP_stdInterDistance[*index])
        + (((NP_entIDWeight2*NP_interEntropy[*index])
            + NP_IDWeight2)
           *pow(NP_stdInterDistance[*index],2))
        *((NP_stdInterDensity[*index]+1)*.2);

      // How many std are you from the mean link distance. Use a weight that
      // signifies how far off you can be
      // look at both intre and inter class variance
      FLOAT measureDistance = NP_distance[ix] -
        NP_meanInterDistance[*index];

      if((measureDistance > threshDistance)
         &&
         (NP_subDecCount[ix] > ((NP_enthardClassSize*NP_interEntropy[*index])
          +NP_hardClassSize))
         &&
         (NP_subDecCount[ix]  >
          (((NP_entICWeight1*NP_interEntropy[*index]) +
           NP_ICWeight1)*NP_meanInterDensity[*index])))
      {
        // check to make sure I'm not already a stem
        // how many grandchildren do you have compaired with the mean
        // make sure you have lots. Remove outliers
        NP_stem[NP_stems] = &NP_keyVal[ix];
        NP_revStemVal[ix] = &NP_keyVal[NP_stems];
        NP_stems++;
        NP_revStem[ix]    = true;
      }
    }
  }
  NPcomputeMasters();
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPanalizeClassDensity()
{
  typename std::vector<FLOAT>::iterator imeanClassDensity
    = NP_meanClassDensity.begin();
  typename std::vector<FLOAT>::iterator istdClassDensity
    = NP_stdClassDensity.begin();
  typename std::vector<FLOAT>::iterator idensity = NP_density.begin();
  std::vector<bool>::iterator irevStem           = NP_revStem.begin();
  std::vector<bool>::iterator ilowDensity        = NP_lowDensity.begin();
  std::vector<int>::iterator iclassSize          = NP_classSize.begin();
  std::vector<int*>::iterator imaster            = NP_master.begin();

  // find masters, the class clusters
  // compute interclass variance and mean
  for(int i = 0; i < NP_stems; i++, ++imeanClassDensity,
        ++istdClassDensity)
  {
    *imeanClassDensity = 0.0F;
    *istdClassDensity  = 0.0F;
  }

  // find mean for interclass statistics
  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++, ++irevStem, ++ilowDensity,
        ++idensity, ++imaster)
  {
    int *index = NP_classIndex[ix];
    if((*irevStem == false) && (*ilowDensity == false))
    {
      NP_meanClassDensity[*index] += *idensity;
    }
  }
  imeanClassDensity = NP_meanClassDensity.begin();

  for(int i = 0; i < NP_stems; i++, ++imeanClassDensity, ++iclassSize)
  {
    if(*iclassSize != 0)
    {
      *imeanClassDensity = *imeanClassDensity/(*iclassSize);
    }
    else
    {
      *imeanClassDensity = 0;
    }
  }

  irevStem          = NP_revStem.begin();
  ilowDensity       = NP_lowDensity.begin();
  idensity          = NP_density.begin();
  imeanClassDensity = NP_meanClassDensity.begin();
  imaster           = NP_master.begin();

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++, ++irevStem, ++ilowDensity,
        ++imeanClassDensity, ++idensity, ++imaster)
  {
    int *index = NP_classIndex[ix];
    if((*irevStem == false) && (*ilowDensity == false))
    {
      NP_stdClassDensity[*index]
        += pow((NP_meanClassDensity[*index] - *idensity),2);
    }
  }

  istdClassDensity = NP_stdClassDensity.begin();
  iclassSize = NP_classSize.begin();

  for(int i = 0; i < NP_stems; i++, ++iclassSize, ++istdClassDensity)
  {
    if(*iclassSize != 0)
    {
      *istdClassDensity = sqrt(*istdClassDensity/(*iclassSize));
    }
    else
    {
      *istdClassDensity = 0;
    }
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPcomputeMasters()
{
  // find my masters, form discrete classes
  /* This is done by taking the first predecesor that is a stem
     and treating it as my master. I then add myself to its list
     of nodes (NP_masterIndex) of slaves
  */

  for(int st = 0; st < NP_stems; st++)
  {
    NP_classSize[st]                  = 0;
    NP_masterIndexCount[*NP_stem[st]] = 0;
  }

  for(unsigned int ix = 0; ix < NP_uspaceSize; ix++)
  {
    bool stop = false;
    bool finish = false;

    for(int pr = 0; pr < NP_predCount[ix]; pr++)
    {
      for(int st = 0; st < NP_stems; st++)
      {
        // check to see if I am a stem/master
        if(NP_revStem[ix] && (finish == false))
        {
          if(st == *NP_revStemVal[ix])
          {
            // I am in my own master node class
            NP_Class[st][NP_classSize[st]] = &NP_keyVal[ix];
            // Increment class size
            NP_classSize[st]++;
            // index from me to my class
            NP_classIndex[ix]              = &NP_keyVal[st];
            finish                         = true;
          }
        }
        // climb up the ladder of predecessor to find the closest master
        if(NP_stem[st] == NP_pred[ix][pr])
        {
          // which node is my master
          NP_master[ix] = NP_stem[st];
          if(NP_revStem[ix] == false)
          {
            // what class am I in, store this for output only
            // (never used again)
            NP_Class[st][NP_classSize[st]] = &NP_keyVal[ix];
            // Increment class size
            NP_classSize[st]++;
            // index from me to my class
            NP_classIndex[ix] = &NP_keyVal[st];
          }
          // index from my masters list to me
          NP_masterIndex[*NP_stem[st]][NP_masterIndexCount[*NP_stem[st]]]
            = &NP_keyVal[ix];
          // increment my masters list counter
          NP_masterIndexCount[*NP_stem[st]]++;
          stop = true;
          break;
        }
      }
      if(stop == true)
        break;
    }
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPparamSpace(int samples, int dim)
{
  NP_spaceSize  = samples;
  NP_uspaceSize = (unsigned)samples;
  NP_dimSize    = dim;
  NP_udimSize   = (unsigned)dim;
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPreverseMap()
{
  typename std::vector<FLOAT*> tempVec(NP_spaceSize,&NP_fZERO);
  NP_reverseDisMap.resize(NP_spaceSize,tempVec);

  for(unsigned int x = 0; x < NP_Dis.size(); x++)
  {
    for(unsigned int y = 0; y < NP_Dis.size(); y++)
    {
      NP_reverseDisMap[x][y] = &NP_Dis[y][x];

    }
  }
}

/************************************************************************/

template <class FLOAT>
void NPclassify2<FLOAT>::NPresizeSpace(int samples, int dim)
{
  // resize vectors to default size initially
  // we suck memory because we care!!
  NP_fZERO = 0;
  NPparamSpace(samples,dim);

  typename std::vector<FLOAT> tempf(NP_spaceSize,0.0F);
  // The distance between any two nodes in space
  NP_Dis.resize(NP_spaceSize,tempf);
  // create reverse mapping for NP_Dis
  NPreverseMap();
  // holds the measure from metaclassify for a train
  NP_trainMeasure.resize(NP_spaceSize,0.0F);
  // Holds the distance from a node to its parent node
  NP_distance.resize(NP_spaceSize,0.0F);
  // the density of this node
  NP_density.resize(NP_spaceSize,0.0F);
  // entropy measure holder
  NP_entropy.resize(NP_spaceSize,0.0F);
  // this is the mean distance within a class
  NP_meanInterDistance.resize(NP_spaceSize,0.0F);
  // The standard deviation for meanInterDistance
  NP_stdInterDistance.resize(NP_spaceSize,0.0F);
  // this is the mean distance within a class
  NP_meanClassDensity.resize(NP_spaceSize,0.0F);
  // this is the sum density within a class used for entropy comp.
  NP_sumInterDensity.resize(NP_spaceSize,0.0F);
  // The standard deviation for meanInterDistance
  NP_stdClassDensity.resize(NP_spaceSize,0.0F);
  // Mean number of decendants per node inter class
  NP_meanInterChild.resize(NP_spaceSize,0.0F);
  // the standard deviation for meanInterClass
  NP_stdInterChild.resize(NP_spaceSize,0.0F);
  // Mean density for nodes inter class
  NP_meanInterDensity.resize(NP_spaceSize,0.0F);
  // Standard deviation for meanInterDensity
  NP_stdInterDensity.resize(NP_spaceSize,0.0F);
  // Entropy measure for intercalss
  NP_interEntropy.resize(NP_spaceSize,0.0F);

  NP_ZERO = 0;
  std::vector<int*> temp(NP_spaceSize,&NP_ZERO);
  // Holds class membership for nodes
  NP_Class.resize(NP_spaceSize,temp);
  // Holds a list of all children for a node
  NP_childMap.resize(NP_spaceSize,temp);
  // Holds a list of all of nodes decendants
  NP_decend.resize(NP_spaceSize,temp);
  // Holds a list of all my predecessors
  NP_pred.resize(NP_spaceSize,temp);
  // List of all nodes I am master of
  NP_masterIndex.resize(NP_spaceSize,temp);


  // holds the ID of each stem for each class
  NP_stem.resize(NP_spaceSize,&NP_ZERO);
  // holds parent for a sample in cluster space
  NP_parent.resize(NP_spaceSize,&NP_ZERO);
  // lists who a nodes master is
  NP_master.resize(NP_spaceSize,&NP_ZERO);
  // lists who a nodes master is
  NP_classIndex.resize(NP_spaceSize,&NP_ZERO);
  // reverse index to the stem for which I am a master
  NP_revStemVal.resize(NP_spaceSize,&NP_ZERO);


  // Returns the number of samples in each class
  NP_classSize.resize(NP_spaceSize,0);
  // Holds the total number of decendants for a node
  NP_decCount.resize(NP_spaceSize,0);
  // Tells which links would have been ideal in the meta-classifier
  NP_idealLinks.resize(NP_spaceSize,0);
  // number of children for a given node
  NP_children.resize(NP_spaceSize,0);
  // tells how many predecessors I have
  NP_predCount.resize(NP_spaceSize,0);
  // counts how many nodes I am master of
  NP_masterIndexCount.resize(NP_spaceSize,0);
  //! Holds the total number of decendants for a node in this class
  NP_subDecCount.resize(NP_spaceSize,0);


  // Returns if the sample has a low density in the map
  NP_lowDensity.resize(NP_spaceSize,false);
  // tells if this node has a stem attached to it
  NP_revStem.resize(NP_spaceSize,false);
  // Tells if this node has been selected or not
  NP_selected.resize(NP_spaceSize,false);

  // lists the key number for any given node
  NP_keyVal.resize(NP_spaceSize,0);
  for(int k = 0; k < NP_spaceSize; k++)
  {
    NP_keyVal[k] = k;
  }


}

/************************************************************************/

template <class FLOAT> inline
FLOAT NPclassify2<FLOAT>::NPgaussKernel(FLOAT x, FLOAT mu, FLOAT sig) const
{
  return (FLOAT)((1/sqrt(2*3.14159F*pow(sig,2.0F))) *
         (pow(2.71828F,-1*pow(x - mu,2.0F)/(2 * pow(sig,2.0F)))));
}

/************************************************************************/

template <class FLOAT> inline
FLOAT NPclassify2<FLOAT>::NPgaussKernel(FLOAT z) const
{
  return (1/sqrt(2 * 3.14159F))*(pow(2.71828F,-1*(pow(z,2.0F)/2)));
}

/************************************************************************/
template <class FLOAT> inline
FLOAT NPclassify2<FLOAT>::NPsigmoidKernel(FLOAT beta, FLOAT v) const
{
  return (1.0f / (1.0f + pow(2.71828f, (-2.0f * (beta * v)))));
}

template class NPclassify2<float>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
