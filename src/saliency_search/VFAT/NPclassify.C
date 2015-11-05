/*!@file VFAT/NPclassify.C  Test the nonparametric classifier
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/NPclassify.C $
// $Id: NPclassify.C 6003 2005-11-29 17:22:45Z rjpeters $
//

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

#include "VFAT/NPclassify.H"

#include "Util/Assert.H"
#include "Util/Timer.H"
#include "Util/log.H"
#include <fstream>
#include <iostream>
#include <math.h>

// *****************************************
// PUBLIC METHODS
// *****************************************

NPclassify::NPclassify(readConfig &settings, readConfig &polySet,
                       bool commandLineSettings)
{
  // set up variables from config file
  CLS = commandLineSettings;
  defaultSize = (int)settings.getItemValueF("defaultSize");
  Con1 = settings.getItemValueF("Con1");
  Con2 = settings.getItemValueF("Con2");
  Con3 = settings.getItemValueF("Con3");
  hardClassSize = (long)settings.getItemValueF("hardClassSize");
  hardLinkSize = (long)settings.getItemValueF("hardLinkSize");
  DWeight1 = settings.getItemValueF("DWeight1");
  DWeight2 = settings.getItemValueF("DWeight2");
  CWeight1 = settings.getItemValueF("CWeight1");
  CWeight2 = settings.getItemValueF("CWeight2");
  IDWeight1 = settings.getItemValueF("IDWeight1");
  IDWeight2 = settings.getItemValueF("IDWeight2");
  ICWeight1 = settings.getItemValueF("ICWeight1");
  ICWeight2 = settings.getItemValueF("ICWeight2");
  DenWeight1 = settings.getItemValueF("DenWeight1");
  DenWeight2 = settings.getItemValueF("DenWeight2");
  preDenWeight1 = settings.getItemValueF("preDenWeight1");
  preDenWeight2 = settings.getItemValueF("preDenWeight2");
  trainChildWeight = settings.getItemValueF("trainChildWeight");
  doLinkMap = (int)settings.getItemValueF("doLinkMap");
  doDensityMap = (int)settings.getItemValueF("doDensityMap");
  doClassMap = (int)settings.getItemValueF("doClassMap");
  usePolySet = (int)settings.getItemValueF("usePolySet");

  polyDensObjectCut1 = polySet.getItemValueF("polyDensObjectCut1");
  polyDensObjectCut2 = polySet.getItemValueF("polyDensObjectCut2");
  polyDensObjectCut3 = polySet.getItemValueF("polyDensObjectCut3");
  polySpaceChildCut1 = polySet.getItemValueF("polySpaceChildCut1");
  polySpaceChildCut2 = polySet.getItemValueF("polySpaceChildCut2");
  polySpaceChildCut3 = polySet.getItemValueF("polySpaceChildCut3");
  // zero some initial counters
  resetSpace();
  // set up vectors to a reasonable initial size
  resizeSpace();
}

NPclassify::~NPclassify()
{}

void NPclassify::inputCommandLineSettings(double _distance, double _children,
                               double Idistance, double Ichildren,
                               long _hardClassSize, long _hardLinkSize,
                               double _polyDensObjectCut1,
                               double _polyDensObjectCut2,
                               double _polyDensObjectCut3)
{
  DWeight1 = _distance;
  CWeight1 = _children;
  IDWeight1 = Idistance;
  ICWeight1 = Ichildren;
  hardClassSize = _hardClassSize;
  hardLinkSize = _hardLinkSize;
  polyDensObjectCut1 = _polyDensObjectCut1;
  polyDensObjectCut2 = _polyDensObjectCut2;
  polyDensObjectCut3 = _polyDensObjectCut3;
}

void NPclassify::addPoint(std::vector<long> point)
{
  //FOO
}

void NPclassify::addSpace(std::vector<std::vector<double> > &space,long sSize)
{
  // make sure space stays
  if(sSize == 0)
    sSize = space.size();
  long oldSpaceSize = spaceSize;
  spaceSize += sSize;
  if((unsigned)(spaceSize) >= Space.size())
    resizeSpace();
  for(int i = 0; i < sSize; i++)
  {
    Space[i+oldSpaceSize] = space[i];
  }
  //std::cerr << "SPACE SIZE " << spaceSize << "\n";
}

void NPclassify::echoSpace()
{
  for(int i = 0; i < spaceSize; i++)
  {
    std::cerr << i << " ";
    for(unsigned int j = 0; j < Space[i].size(); j++)
      std::cerr << Space[i][j] << " ";
    std::cerr << "\n";
  }
}

void NPclassify::resetSpace()
{
  //std::cerr << "SPACE RESET\n";
  spaceSize = 0;
  stems = 0;
  roots = 0;
}

void NPclassify::classifySpaceNew()
{
  Timer tim;
  tim.reset();
  //int t1,t2;
  //int t0 = tim.get();  // to measure display time
  //std::cout << "CONVOLVE SPACE\n";
  convolveSpace2();
  //t1 = tim.get();
  //t2 = t1 - t0;
  //std::cout << "\tTIME: " << t2 << "ms\n";
  //std::cout << "LINK SPACE\n";
  linkSpace();
  //t1 = tim.get();
  //t2 = t1 - t0;
  //std::cout << "\tTIME: "<< t2 << "ms\n";
  //std::cout << "MAP SPACE\n";
  mapSpace();
  //t1 = tim.get();
  //t2 = t1 - t0;
  //std::cout << "\tTIME: "<< t2 << "ms\n";
  //std::cout << "ANALYZE SPACE\n";
  analizeSpace();
  //t1 = tim.get();
  //t2 = t1 - t0;
  //std::cout << "\tTIME: "<< t2 << "ms\n";
  //std::cout << "EVOLVE SPACE\n";
  evolveSpace();
  //t1 = tim.get();
  //t2 = t1 - t0;
  //std::cout << "\tTIME: "<< t2 << "ms\n";
  //std::cout << "ANALYZE INTER SPACE\n";
  analizeInterSpace();
  //t1 = tim.get();
  //t2 = t1 - t0;
  //std::cout << "\tTIME: "<< t2 << "ms\n";
  //std::cout << "EVOLVE INTER SPACE\n";
  evolveInterSpace();
  //t1 = tim.get();
  //t2 = t1 - t0;
  //std::cout << "\tTIME: "<< t2 << "ms\n";
}

void NPclassify::classifySpacePartial()
{}

long NPclassify::getStemNumber()
{
  return stems;
}

double NPclassify::getMaxDensity()
{
  return maxDensity;
}

bool NPclassify::isLowDensity(long item)
{
  return lowDensity[item];
}

bool NPclassify::isStem(long item)
{
  return revStem[item];
}

std::vector<double> NPclassify::getDensity()
{
  return density;
}

std::vector<long> NPclassify::getStems()
{
  return stem;
}

long NPclassify::getClassSize(long _Class)
{
  ASSERT((_Class <= (signed)classSize.size()) && (_Class >= 0));
  return classSize[_Class];
}

long NPclassify::getMinClassSize()
{
  return hardClassSize;
}

std::vector<long> NPclassify::getClass(long _Class)
{
  ASSERT((_Class <= (signed)Class.size()) && ((_Class >= 0)));
  return Class[_Class];
}

long NPclassify::getClass(long _Class, long item)
{
  ASSERT((_Class <= (signed)Class.size()) && ((_Class >= 0)));
  ASSERT((item <= (signed)Class[item].size()) && ((item >= 0)));
  return Class[_Class][item];
}

double NPclassify::getFeature(long m_feature_index, long n_vector_index)
{
  ASSERT((m_feature_index <= (signed)Space.size()) && (m_feature_index >= 0));
  ASSERT((n_vector_index <= (signed)Space[m_feature_index].size())
         && (n_vector_index >= 0));
  return Space[m_feature_index][n_vector_index];
}

std::vector<long> NPclassify::getParents()
{
  return parent;
}

std::vector<std::vector<long> > NPclassify::getChildren()
{
  return childMap;
}

std::vector<std::vector<long> > NPclassify::getBoundingBoxes()
{
  std::vector<long> box(4,0);
  std::vector<std::vector<long> > boundingBox(stems,box);

  for(int ix = 0; ix < spaceSize; ix++)
  {
    if(revStem[ix] == false)
    {
      if(lowDensity[ix] == false)
        //&& (classSize[masterIndex[ix]] > hardClassSize))
      {
        if(boundingBox[masterIndex[ix]][0] == 0)
          boundingBox[masterIndex[ix]][0] = (long)Space[ix][0];
        else
          if(boundingBox[masterIndex[ix]][0] < Space[ix][0])
            boundingBox[masterIndex[ix]][0] = (long)Space[ix][0];
        if(boundingBox[masterIndex[ix]][1] == 0)
          boundingBox[masterIndex[ix]][1] = (long)Space[ix][1];
        else
          if(boundingBox[masterIndex[ix]][1] < Space[ix][1])
            boundingBox[masterIndex[ix]][1] = (long)Space[ix][1];
        if(boundingBox[masterIndex[ix]][2] == 0)
          boundingBox[masterIndex[ix]][2] = (long)Space[ix][0];
        else
          if(boundingBox[masterIndex[ix]][2] > Space[ix][0])
            boundingBox[masterIndex[ix]][2] = (long)Space[ix][0];
        if(boundingBox[masterIndex[ix]][3] == 0)
          boundingBox[masterIndex[ix]][3] = (long)Space[ix][1];
        else
          if(boundingBox[masterIndex[ix]][3] > Space[ix][1])
            boundingBox[masterIndex[ix]][3] = (long)Space[ix][1];
      }
    }
  }
  return boundingBox;
}

void NPclassify::metaClassify(int objects)
{
  // find the first n links that meet that training criteria
  // find their statistics
  // open file to save tab delim data to
  std::ofstream outfile("train_set.dat",std::ios::app);

  // create statistics

  for(int ix = 0; ix < spaceSize; ix++)
  {
    trainMeasure[ix] = distance[ix]*(childMapTot[ix]/trainChildWeight);
    selected[ix] = false;
  }

  // find the n best candidates
  for(int n = 0; n < objects; n++)
  {
    int init = 0;
    for(int ix = 0; ix < spaceSize; ix++)
    {
      if(selected[ix] == false)
      {
        if(init == 0)
        {
          idealLinks[n] = ix;
          init = 1;
        }
        else
        {
          if(trainMeasure[ix] > trainMeasure[idealLinks[n]])
          {
            idealLinks[n] = ix;
          }
        }
      }
    }
    //std::cout << "FOUND " << idealLinks[n] << " " <<  distance[ idealLinks[n]]
    //              << " " << childMapTot[ idealLinks[n]] << "\n";
    selected[idealLinks[n]] = true;
  }

  // find there stats as they apply to the current scene
  // find min child and distance for nodes
  int init = 0;
  for(int n = 0; n < objects; n++)
  {
    if(init == 0)
    {
      minDist = idealLinks[n];
      minChild = idealLinks[n];
      init = 1;
    }
    else
    {
      if(distance[idealLinks[n]] < distance[minDist])
      {
        minDist = idealLinks[n];
      }
      if(childMapTot[idealLinks[n]] <  childMapTot[minChild])
      {
        minChild = idealLinks[n];
      }
    }
  }
  long returnStems = stems;
  for(int i = 0; i < stems; i++)
  {
    if(classSize[i] <= hardClassSize)
    {
      --returnStems;
    }
  }

  //std::cout << "CALC " << distance[minDist] << " - " <<  meanDistance
  //            << " / " << stdDistance << "\n";

  distanceCut = (distance[minDist] - meanDistance)/stdDistance;
  childCut = childMapTot[minChild]/meanChildMap;
  //std::cout << "distance " << distanceCut
  //            << " children " << childCut << "\n";
  outfile << spaceSize << "\t" << meanDensity << "\t"
          << objects << "\t" << distanceCut << "\t"
          << childCut << "\t" << returnStems << "\n";

  outfile.close();
}

// *****************************************
// PRIVATE METHODS
// *****************************************

void NPclassify::convolveSpace()
{
  // find distance between all points as an n^2 alg. Sum these for all
  // points to determine a points density measure

  for(int ix = 0; ix < spaceSize; ix++)
  {
    children[ix] = 0;
    revStem[ix] = false;
    for(int iy = 0; iy < spaceSize; iy++)
    {
      if(ix != iy)
      {
        Dis[ix][iy] = 0;
        // get distance between these two points
        for(unsigned int px = 0; px < Space[ix].size(); px++)
        {
            Dis[ix][iy] = Dis[ix][iy] +
              pow((Space[ix][px] - Space[iy][px]),2);
        }
        // find distance as sqrt(sum(sqared distances))
        // take the inverse distance 1/D
        // Dis[ix][iy] = sqrt(D[ix][iy]);
        if(Dis[ix][iy] != 0)
          D[ix][iy] = 1/sqrt(Dis[ix][iy]);
        // find polynomial distance

        D[ix][iy] = Con1*D[ix][iy] + pow((Con2*D[ix][iy]),2) +
          pow((Con3*D[ix][iy]),3);
      }

      density[ix] += D[ix][iy];
    }
    //std::cerr << "Density " << ix << " is " << density[ix] << "\n";
    //std::cerr << "PDensity " << ix << " is " << polyDensity[ix] << "\n";
  }
}


void NPclassify::convolveSpace2()
{
  // find distance between all points as an n^2 alg. Sum these for all
  // points to determine a points density measure

  for(int ix = 0; ix < spaceSize; ix++)
  {
    children[ix] = 0;
    revStem[ix] = false;
    for(int iy = 0; iy < ix; iy++)
    {
        Dis[ix][iy] = 0;
        // get distance between these two points
        for(unsigned int px = 0; px < Space[ix].size(); px++)
        {
            Dis[ix][iy] = Dis[ix][iy] +
              pow((Space[ix][px] - Space[iy][px]),2);
        }
        // find distance as sqrt(sum(sqared distances))
        // take the inverse distance 1/D
        if(Dis[ix][iy] != 0)
          density[ix] += 1/sqrt(Dis[ix][iy]);
    }
    for(int iy = ix+1; iy < spaceSize; iy++)
    {
        Dis[ix][iy] = 0;
        // get distance between these two points
        for(unsigned int px = 0; px < Space[ix].size(); px++)
        {
            Dis[ix][iy] = Dis[ix][iy] +
              pow((Space[ix][px] - Space[iy][px]),2);
        }
        // find distance as sqrt(sum(sqared distances))
        // take the inverse distance 1/D
        if(Dis[ix][iy] != 0)
          density[ix] += 1/sqrt(Dis[ix][iy]);
    }
    //std::cerr << "Density " << ix << " is " << density[ix] << "\n";
    //std::cerr << "PDensity " << ix << " is " << polyDensity[ix] << "\n";
  }
}

void NPclassify::linkSpace()
{
  // link points together. Take the closest point that is higher in
  // density

  // find mean density
  sum = 0; sumSquares = 0;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sum += density[ix];
  }
  meanDensity = sum/spaceSize;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sumSquares += pow((meanDensity - density[ix]),2);
  }
  stdDensity = sqrt(sumSquares/spaceSize);
  //std::cout << "meanDensity " << meanDensity << "\n";
  //std::cout << "stdDensity " << stdDensity << "\n";
  thresh3 = (preDenWeight1*meanDensity) + (preDenWeight2*pow(meanDensity,2));
  for(int ix = 0; ix < spaceSize; ix++)
  {
    if(density[ix] < thresh3)
      lowDensity[ix] = true;
    else
      lowDensity[ix] = false;
  }

  stems = 0;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    children[ix] = 0;
  }

  for(int ix = 0; ix < spaceSize; ix++)
  {
    distance[ix] = 0;
    parent[ix] = -1;
    if(lowDensity[ix] == false)
    {
      for(int iy = 0; iy < spaceSize; iy++)
      {
        if(lowDensity[iy] == false)
        {
          if(ix != iy)
          {
            if(density[iy] > density[ix])
            {
              if(distance[ix] == 0)
              {
                distance[ix] = Dis[ix][iy];
                parent[ix] = iy;
              }
              else
              {
                if(Dis[ix][iy] < distance[ix])
                {
                  distance[ix] = Dis[ix][iy];
                  parent[ix] = iy;
                }
              }
            }
          }
        }
      }

      // this guy has no parent. Label him as a stem
      if(parent[ix] == -1)
      {
        stem[stems] = ix;
        stems++;
        revStem[ix] = true;
        maxDensity = density[ix];
      }
      else
      {
        child[parent[ix]][children[parent[ix]]] = ix;
        children[parent[ix]]++;
      }
    }
    ///    else  // Compiler says: value computed is not used
    ///{
    ///  parent[ix] == -2;
    ///}
    //std::cerr << "LINKED " << ix << " TO " << parent[ix] << "\n";
  }

  // quick check for children. Your a root if you have none.
  roots = 0;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    if(children[ix] == 0)
    {
      root[roots] = ix;
      roots++;
    }
  }
}

void NPclassify::mapSpace()
{
  for(int ix = 0; ix < spaceSize; ix++)
  {
    for(int C = 0; C < children[ix]; C++)
    {
      childMap[ix][C] = child[ix][C];
      childMapTot[ix]++;
    }
  }
  //std::cerr << "CREATED initial map\n";

  notDone = true;

  // iterate linking parents and children until no more linking can be done
  // >retarded linking
  iteration = 0;
  while(notDone == true)
  {
    notDone = false;
    for(int ix = 0; ix < spaceSize; ix++)
    {
      // don't bother with this node if done
      if(childMapDone[ix] == false)
      {
        // my children
        for(int C = 0; C < childMapTot[ix]; C++)
        {
          // my childrens children
          for(int C2 = 0; C2 < childMapTot[childMap[ix][C]]; C2++)
          {
            // check to make sure we don't add twice
            bool found = false;
            //std::cerr << "TRYING " << childMap[childMap[ix][C]][C2]
            //              << " AGAINST " << ix << "\n";
            for(int CH = 0; CH < childMapTot[ix]; CH++)
            {
              if(childMap[ix][CH] == childMap[childMap[ix][C]][C2])
              {
                found = true;
              }
            }
            if(found == false)
            {
              childMap[ix][childMapTot[ix]] =
                childMap[childMap[ix][C]][C2];
              childMapTot[ix]++;
            }
          }
        }
      }
      // if I added anyone since the last iteration. Do another iteration.
      // stop if no one was added this iteration
      if(iteration > 0)
      {
        if(childMapTot[ix] != childMapTotLast[ix])
        {
          //std::cerr << "NOT DONE\n";
          notDone = true;
        }
        else
        {
          //std::cerr << "DONE\n";
          childMapDone[ix] = true;
        }
      }
      childMapTotLast[ix] = childMapTot[ix];
    }
    if(iteration == 0)
    {
      //std::cerr << "NOT DONE (i1)\n";
      notDone = true;
    }
    iteration++;
  }
}

void NPclassify::analizeSpace()
{

  // find mean distance
  sum = 0; sumSquares = 0;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sum += distance[ix];
  }
  meanDistance = sum/spaceSize;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sumSquares += pow((meanDistance - distance[ix]),2);
  }
  stdDistance = sqrt(sumSquares/spaceSize);
  //std::cout << "Mean Dist " << meanDistance << " std " << stdDistance << "\n";
  // find the mean number of children
  sum = 0; sumSquares = 0;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sum += children[ix];
  }
  meanChildren = sum/spaceSize;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sumSquares += pow((meanChildren - children[ix]),2);
  }
  stdChildren = sqrt(sumSquares/spaceSize);
  //std::cout << "Mean Children " << meanChildren
  //            <<" std " << stdChildren << "\n";
  // find the mean number of all subordinates (grandchildren etc.)
  //            << " std " << stdChildren << "\n";
  sum = 0; sumSquares = 0;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sum += childMapTot[ix];
  }
  meanChildMap = sum/spaceSize;
  for(int ix = 0; ix < spaceSize; ix++)
  {
    sumSquares += pow((meanChildMap - childMapTot[ix]),2);
  }
  stdChildMap = sqrt(sumSquares/spaceSize);
  //std::cout << "Mean Children Tot " << meanChildMap
  //            << " std " << stdChildMap << "\n";

}

void NPclassify::evolveSpace()
{
  //initial threshold is a function of standard deviation and a weight
  //and number of children and a weight for that



  // use polynomial optimized (linear regression) thresholds
  thresh1 = (polyDensObjectCut1*meanDensity+
               polyDensObjectCut2*pow(meanDensity,(double)2.0)+
               polyDensObjectCut3*pow(meanDensity,(double)3.0)) /
    (stdDensity+1);

    //>>rev A
    //thresh2 = (polySpaceChildCut1*spaceSize+
  //               polySpaceChildCut2*pow((double)spaceSize,(double)2.0)+
  //               polySpaceChildCut3*pow((double)spaceSize,(double)3.0));// *
      //(stdDensity+1);
    //>>rev A

  thresh2 = (CWeight1*meanChildMap) - (CWeight2*pow(meanChildMap,2));
  //std::cout << "THRESH1 " << thresh1 << " THRESH2 " << thresh2 << "\n";
  thresh3 = (DenWeight1*stdDensity) + (DenWeight2*pow(stdDensity,2));
  thresh4 = (DWeight1*stdDistance) + (DWeight2*pow(stdDistance,2));
  for(int ix = 0; ix < spaceSize; ix++)
  {
    measure1 = density[ix] - meanDensity;
    if((measure1 < 0) && (measure1 < thresh3))
      lowDensity[ix] = true;
    // How many std are you from the mean link distance. Use a weight that
    // signifies how far off you can be
    // look at both intre and inter class variance
    measure1 = distance[ix] - meanDistance;
    measure2 = meanDensity - density[ix];
    //std::cout << measure1 << " > " << thresh4 << "\t"
    //          << measure2 << " > " << thresh1 << "\t"
    //          << childMapTot[ix] << " > " << thresh2 << "\t"
    //          << distance[ix] << " > " << hardLinkSize << "\n";

    if((((measure1 > thresh4) || (measure2 > thresh1))
        && (childMapTot[ix] > thresh2)) || (distance[ix] > hardLinkSize))
    {
      if(lowDensity[ix] == false)
      {
        // how many grandchildren do you have compaired with the mean
        // make sure you have lots. Remove outliers
        stem[stems] = ix;
        //std::cerr << "STEM " << ix << "\n";
        stems++;
        revStem[ix] = true;
      }
    }
  }
  //std::cout << "FIRST ROUND STEMS: " << stems << "\n";
  computeMasters();
}

void NPclassify::analizeInterSpace()
{
  // find masters, the class clusters
  // compute interclass variance and mean
  for(int i = 0; i < stems; i++)
  {
    meanInterDistance[i] = 0;
    stdInterDistance[i] = 0;
    meanInterChild[i] = 0;
    stdInterChild[i] = 0;
    if(classSize[i] == 0)
      classSize[i] = 1;
  }
  //first refine how many children are inside my class
  for(int ix = 0; ix < spaceSize; ix++)
  {
    childInterCount[ix] = 0;
    if(lowDensity[ix] == false)
    {
      for(int C = 0; C < childMapTot[ix]; C++)
      {
        // me and my decendant have the same master
        if(masterIndex[ix] == masterIndex[childMap[ix][C]])
        {
          childInterMap[ix][childInterCount[ix]] = childMap[ix][C];
          childInterCount[ix]++;
        }
      }
    }
  }

  // find mean for interclass statistics
  for(int ix = 0; ix < spaceSize; ix++)
  {
    if((revStem[ix] == false) && (lowDensity[ix] == false))
    {
      meanInterDistance[masterIndex[ix]] += distance[ix];
      meanInterDensity[masterIndex[ix]] += density[ix];
      meanInterChild[masterIndex[ix]] += childInterCount[ix];
    }
  }

  for(int i = 0; i < stems; i++)
  {
    //std::cout << "Class Size " << classSize[i] << "\n";
    meanInterDistance[i] = meanInterDistance[i]/classSize[i];
    meanInterDensity[i] = meanInterDensity[i]/classSize[i];
    meanInterChild[i] = meanInterChild[i]/classSize[i];
    //std::cout << "meanInterDistance " << meanInterDistance[i] << "\n";
    //std::cout << "meanInterDensity " << meanInterDensity[i] << "\n";
    //std::cout << "meanInterChild " << meanInterChild[i] << "\n";
  }

  for(int ix = 0; ix < spaceSize; ix++)
  {
    if((revStem[ix] == false) && (lowDensity[ix] == false))
    {
      stdInterDistance[masterIndex[ix]]
        += pow((meanInterDistance[ix] - distance[ix]),2);
      stdInterDensity[masterIndex[ix]]
        += pow((meanInterDensity[ix] - density[ix]),2);
      stdInterChild[masterIndex[ix]]
        += pow((meanInterChild[ix] - childInterCount[ix]),2);
    }
  }
  for(int i = 0; i < stems; i++)
  {
    stdInterDistance[i] = sqrt(stdInterDistance[i]/classSize[i]);

    stdInterDensity[i] = sqrt(stdInterDensity[i]/classSize[i]);

    stdInterChild[i] = sqrt(stdInterChild[i]/classSize[i]);
    //std::cout << "stdInterDistance " << stdInterDistance[i] << "\n";
    //std::cout << "stdInterDensity " << stdInterDensity[i] << "\n";
    //std::cout << "stdInterChild " << stdInterChild[i] << "\n";
  }
}

void NPclassify::evolveInterSpace()
{
  //initial threshold is a function of standard deviation and a weight
  //and number of children and a weight for that
  for(int ix = 0; ix < spaceSize; ix++)
  {
    thresh3 = ((IDWeight1*stdInterDistance[masterIndex[ix]])
               + (IDWeight2*pow(stdInterDistance[masterIndex[ix]],2)))
      *((stdInterDensity[masterIndex[ix]]+1)*.2);
      //(stdDensity+1);
    thresh2 = ((ICWeight1*(1+meanInterChild[masterIndex[ix]]))
               - (ICWeight2*pow((1+meanInterChild[masterIndex[ix]]),2)))
      *(meanInterDensity[masterIndex[ix]]+1);
      //(stdDensity+1);
    // thresh2 = thresh2 * density[ix];
    // How many std are you from the mean link distance. Use a weight that
    // signifies how far off you can be
    // look at both intre and inter class variance
    measure1 = distance[ix] - meanInterDistance[masterIndex[ix]];
    //if((measure1 > thresh3) && (childInterCount[ix] > thresh2))
    //std::cout << measure1 << " > " <<  thresh3 << "\t"
    //          << childInterCount[ix] << " > " << hardClassSize << "\t"
    //          << childInterCount[ix] << " > "
    //          << (ICWeight1*meanInterDensity[masterIndex[ix]]) << "\n";
    if((measure1 > thresh3) && (childInterCount[ix] > hardClassSize) &&
       (childInterCount[ix]  > (ICWeight1*meanInterDensity[masterIndex[ix]])))
    {
      // how many grandchildren do you have compaired with the mean
      // make sure you have lots. Remove outliers
      stem[stems] = ix;
      stems++;
      revStem[ix] = true;
    }
  }
  //std::cout << "NEXT ROUND STEMS: " << stems << "\n";
  computeMasters();
}

void NPclassify::computeMasters()
{
  // find my masters, form discrete classes

  for(int ix = 0; ix < spaceSize; ix++)
  {
    master[ix] = 0;
    classSize[ix] = 1;
  }
  for(int i = 0; i < stems; i++)
  {
    for(int j = 0; j < childMapTot[stem[i]]; j++)
    {
      if(master[childMap[stem[i]][j]] == 0)
      {
        master[childMap[stem[i]][j]] = stem[i];
        masterIndex[childMap[stem[i]][j]] = i;
      }
      else
      {
        for(int k = 0; k < childMapTot[master[childMap[stem[i]][j]]]; k++)
        {
          if(childMap[master[childMap[stem[i]][j]]][k] == stem[i])
          {
            master[childMap[stem[i]][j]] = stem[i];
            masterIndex[childMap[stem[i]][j]] = i;
          }
        }
      }
    }
  }
  // structure class list
  for(int ix = 0; ix < spaceSize; ix++)
  {
    Class[masterIndex[ix]][classSize[masterIndex[ix]]] = ix;
    classSize[masterIndex[ix]]++;
  }
}

void NPclassify::resizeSpace()
{
  // resize vectors to default size initially
  // we suck memory because we care!!
  //std::cerr << "RESIZING ALL VECTORS TO: " << (defaultSize+spaceSize) << "\n";
  children.resize(defaultSize+spaceSize,0);
  parent.resize(defaultSize+spaceSize);
  childMapTot.resize(defaultSize+spaceSize);
  childMapTotLast.resize(defaultSize+spaceSize);
  stem.resize(defaultSize+spaceSize);
  root.resize(defaultSize+spaceSize);
  master.resize(defaultSize+spaceSize);
  masterIndex.resize(defaultSize+spaceSize);
  classSize.resize(defaultSize+spaceSize);
  childInterCount.resize(defaultSize+spaceSize);
  idealLinks.resize(defaultSize+spaceSize);
  density.resize(defaultSize+spaceSize);
  distance.resize(defaultSize+spaceSize);
  meanInterDistance.resize(defaultSize+spaceSize);
  meanInterDensity.resize(defaultSize+spaceSize);
  meanInterChild.resize(defaultSize+spaceSize);
  stdInterDistance.resize(defaultSize+spaceSize);
  stdInterDensity.resize(defaultSize+spaceSize);
  stdInterChild.resize(defaultSize+spaceSize);
  trainMeasure.resize(defaultSize+spaceSize);
  revStem.resize(defaultSize+spaceSize,false);
  childMapDone.resize(defaultSize+spaceSize);
  lowDensity.resize(defaultSize+spaceSize);
  selected.resize(defaultSize+spaceSize);

  std::vector<long> tempLong(defaultSize,0);
  std::vector<double> tempDouble(defaultSize,0);
  child.resize(defaultSize+spaceSize,tempLong);
  childMap.resize(defaultSize+spaceSize,tempLong);
  childInterMap.resize(defaultSize+spaceSize,tempLong);
  Class.resize(defaultSize+spaceSize,tempLong);
  Dis.resize(defaultSize+spaceSize,tempDouble);
  D.resize(defaultSize+spaceSize,tempDouble);
  Space.resize(defaultSize+spaceSize,tempDouble);
  //std::cerr << "DONE\n";
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
