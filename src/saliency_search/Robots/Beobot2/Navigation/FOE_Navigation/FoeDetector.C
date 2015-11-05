/*!@file Robots2/Beobot2/Navigation/FOE_Navigation/FoeDetector.C */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/FoeDetector.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>

#include "Robots/Beobot2/Navigation/FOE_Navigation/FoeDetector.H"

#include  <cstdio>
#include "Image/Kernels.H"
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/ShapeOps.H"
#include "Image/SimpleFont.H"

#include "Util/Timer.H"

#define FOE_STEP                 4
#define INTEGRATION_WINDOW       5

// ######################################################################
// ######################################################################
FoeDetector::FoeDetector(OptionManager& mgr,
                         const std::string& descrName,
                         const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName)
{ 
  // set observer rotation motion to be 0
  itsCurrentRotMotionSpeed = 0.0;
  itsCurrentRotMotionDir   = 0;
  itsCurrentRotMotionDi    = 0.0; 
  itsCurrentRotMotionDj    = 0.0;
  
  // For temporal integration of FOE computation
  itsCurrentFoeMapIndex = -1;
  itsRecentFoeMaps.resize(INTEGRATION_WINDOW);

  // storage for MT features
  itsMT.reset(new MiddleTemporal());

  itsWin.reset();
}

// ######################################################################
void FoeDetector::reset
(uint numPyrLevels, uint numDirs, uint numSpeeds)
{
  itsNumPyrLevels  = numPyrLevels;
  itsNumDirs       = numDirs;
  itsNumSpeeds     = numSpeeds;

  itsSpatioTemporalPyrBuilders.clear();
  itsSpatioTemporalPyrBuilders.resize(itsNumDirs);
  for(uint i = 0; i < itsNumDirs; i++)
    itsSpatioTemporalPyrBuilders[i].resize(itsNumSpeeds);

  itsRawSpatioTemporalEnergy.clear();
  itsRawSpatioTemporalEnergy.resize(itsNumDirs);
  for(uint i = 0; i < itsNumDirs; i++)
    itsRawSpatioTemporalEnergy[i].resize(itsNumSpeeds);

  itsSpatioTemporalEnergy.resize(itsNumDirs);
  //itsSpatioTemporalEnergyOptimalShift.resize(itsNumDirs);
  //for(uint i = 0; i < itsNumDirs; i++)
  //  itsSpatioTemporalEnergyOptimalShift[i].reset(itsNumPyrLevels);

  itsMTfeatures.resize(itsNumDirs);
  itsMToptimalShift.resize(itsNumDirs);

  LINFO("Using %d directions spanning [0..360]deg", itsNumDirs);
  for (uint i = 0; i < itsNumDirs; i++)
    {
      for (uint j = 0; j < itsNumSpeeds; j++)
        {
          float speed = pow(2.0, j);
          
          itsSpatioTemporalPyrBuilders[i][j].reset
            (new SpatioTemporalEnergyPyrBuilder<float>
             (Oriented5, 
              360.0 * double(i)/double(itsNumDirs),
              speed,
              itsNumPyrLevels));
        }

      // FIXXX: add the 1/2, 1/2, 1/4 case as well
    }
}

// ######################################################################
FoeDetector::~FoeDetector()
{ }

// ######################################################################
Point2D<int> FoeDetector::getFoe
(Image<byte> lum, uint method, bool tempFilter)
{
  // compute all Visual Cortex features
  uint width  = itsCurrentImage.getWidth();
  uint height = itsCurrentImage.getHeight(); 

  uint lwidth  = lum.getWidth();
  uint lheight = lum.getHeight(); 

  if(width != lwidth || height != lheight)
    {
      width  = lum.getWidth();
      height = lum.getHeight(); 
      itsFoeMap = Image<float>
        (width/MAX_NEIGHBORHOOD, height/MAX_NEIGHBORHOOD, ZEROS);

      // pre-compute direction weights for FOE detection
      resetDirectionWeights(width, height);
    }

  itsCurrentImage = lum;

  Timer tim(1000000);
    
  // compute Visual Cortex features 
  tim.reset();
  computeV1features();
  //uint64 t1 = tim.get();
  //LINFO("time V1feat : %f", t1/1000.0);

  Point2D<int> maxpt;
  if(itsRawSpatioTemporalEnergy[0][0].size() > 0) 
    {
      // compute Medial Temporal features 
      itsMT->computeMTfeatures(itsRawSpatioTemporalEnergy);
      itsMTfeatures = itsMT->getMTfeatures();
      itsMToptimalShift = itsMT->getMToptimalShift();
      
      // DEBUG
      //printItsSpatioTemporalEnergy();
      
      //uint64 t2 = tim.get();
      //LINFO("time MTfeat : %f", (t2 - t1)/1000.0);
      
      // detect yaw-pitch plane rotation (planar motion)
      // and correct the FOE templates accordingly
      //detectObserverRotation();
      //correctForObserverRotation();
      
      // compute FOE     
      if(method == FOE_METHOD_TEMPLATE)
        maxpt = computeFoeTemplate();
      else if(method == FOE_METHOD_AVERAGE)
        maxpt = computeFoeAverage();
      else LFATAL("Unknown Foe Method");
      //uint64 t3 = tim.get();  
      //LINFO("time MSTfeat: %f", (t3 - t2)/1000.0);
    }

  // if we want to filter the FOE locations temporally
  if(tempFilter)
    itsFoe = temporalFilterFoeComputation();
  else
    itsFoe = maxpt;

  return itsFoe;
}

// ######################################################################
Image<float> FoeDetector::getFoeMap
(Image<byte> lum, uint method, bool tempFilter)
{
  Point2D<int> pt =  getFoe(lum, method, tempFilter);
  LDEBUG("pt: %d %d", pt.i, pt.j);
  
  return itsFoeMap;
}

// ######################################################################
Point2D<int> FoeDetector::getFoe
( rutz::shared_ptr<OpticalFlow> flow, uint method, bool tempFilter)
{
  Timer tim(1000000); tim.reset();
      
  // detect yaw-pitch plane rotation (planar motion)
  // and correct the FOE templates accordingly
  detectObserverRotation();
  correctForObserverRotation();

  // NOTE: can create MT features from the flow as well

  // compute FOE     
  Point2D<int> maxpt;
  if(method == FOE_METHOD_TEMPLATE)
    maxpt = computeFoeTemplate(flow);
  else if(method == FOE_METHOD_AVERAGE)
    maxpt = computeFoeAverage(flow);
  else LFATAL("Unknown Foe Method");
  //uint64 t3 = tim.get();  
  //LINFO("time MSTfeat: %f", t3/1000.0);

  // if we want to filter the FOE locations temporally
  if(tempFilter)
    itsFoe = temporalFilterFoeComputation();
  else
    itsFoe = maxpt;

  return itsFoe;
}

// ######################################################################
Image<float> FoeDetector::getFoeMap
( rutz::shared_ptr<OpticalFlow> flow, uint method, bool tempFilter)
{
  Point2D<int> pt = getFoe(flow, method, tempFilter);
  LDEBUG("pt: %d %d", pt.i, pt.j);

  return itsFoeMap;
}
// ######################################################################
Point2D<int> FoeDetector::getFoe
( std::vector <Image<float> > mtFeatures,
  std::vector <Image<float> > mtOptimalShift,
  uint method, bool tempFilter)
{
  itsNumDirs        = mtFeatures.size();
  itsMTfeatures     = mtFeatures;
  itsMToptimalShift = mtOptimalShift;
      
  //uint64 t2 = tim.get();
  //LINFO("time MTfeat : %f", (t2 - t1)/1000.0);

  // check whether observer is stationary
  float activityVal = detectObserverStationarity();

  // detect yaw-pitch plane rotation (planar motion)
  // and correct the FOE templates accordingly  
  detectObserverRotation();
  //correctForObserverRotation();
  
  // compute FOE
  Point2D<int> maxpt;
  if(method == FOE_METHOD_TEMPLATE)
    maxpt = computeFoeTemplate();
  else if(method == FOE_METHOD_AVERAGE)
    maxpt = computeFoeAverage();
  else LFATAL("Unknown Foe Method");

  // if we want to filter the FOE locations temporally
  if(tempFilter)
    itsFoe = temporalFilterFoeComputation();
  else
    itsFoe = maxpt;

  // check if the we observe stationary or radial motion
  // by the confidence value of the maximum location
  LINFO("maxpt: %d %d", maxpt.i, maxpt.j);
  float val = itsFoeMap.getVal(maxpt/MAX_NEIGHBORHOOD);
  if(val < .1)
    { 
      LINFO("max val too low: "
            "must be radial motion (%f) or stationary (%f)",
            val, activityVal);
      itsFoeMap.clear(0.0);
      itsFoe = Point2D<int>(-1,-1);
    }

  return itsFoe;
}

// ######################################################################
Point2D<int> FoeDetector::temporalFilterFoeComputation()
{
  Point2D<int> foe;
  uint width  = itsFoeMap.getWidth();
  uint height = itsFoeMap.getHeight();

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(width,height), 
  //                                width+10, 0, "FOE Detector"));
  // itsWin->setDims(Dims(width*MAX_NEIGHBORHOOD, height*MAX_NEIGHBORHOOD));
  // itsWin->drawImage(zoomXY(itsFoeMap,MAX_NEIGHBORHOOD),0,0);

  // Raster::waitForKey();


  // put the current FOE map to the history
  itsCurrentFoeMapIndex++;
  uint index = (itsCurrentFoeMapIndex)%INTEGRATION_WINDOW;
  itsRecentFoeMaps[index] = itsFoeMap;

  uint numMaps = INTEGRATION_WINDOW;
  if (itsCurrentFoeMapIndex < INTEGRATION_WINDOW)
    numMaps = itsCurrentFoeMapIndex+1;

  LINFO("start: %d",itsCurrentFoeMapIndex);
  Image<float> image =  itsRecentFoeMaps[0];
  for(uint i = 1; i < numMaps; i++)
    image = image + itsRecentFoeMaps[i];
  itsFoeMap = image/numMaps;

  float max = itsFoeMap.getVal(0,0);
  for(uint i = 0; i < width; i++) 
    for(uint j = 0; j < height; j++)
      { 
        float val = itsFoeMap.getVal(i,j);
        if(max < val) 
          {
            foe = Point2D<int>(i*MAX_NEIGHBORHOOD,j*MAX_NEIGHBORHOOD);
            max = val;
          }
      }

  // LINFO("After Integrated FOE: (%3d %3d): %15.5f", foe.i, foe.j, max);
  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(width,height), 
  //                                width+10, 0, "FOE Detector"));
  // itsWin->setDims(Dims(width*MAX_NEIGHBORHOOD, height*MAX_NEIGHBORHOOD));
  // itsWin->drawImage(zoomXY(itsFoeMap,MAX_NEIGHBORHOOD),0,0);

  // Raster::waitForKey();


      // float midsum = 0.0;
      // for(uint i = width/4+1; i < 3*width/4-1; i++)
      //   for(uint j = height/4+1; j < 3*height/4-1; j++)
      //     midsum += itsFoeMap.getVal(i,j);
      // float total = sum(itsFoeMap);

      // LINFO("mid: %f sum: %f", midsum, total);

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(width,height), 28
  //                                width+10, 0, "FOE Detector"));
  // itsWin->setDims(Dims(width*MAX_NEIGHBORHOOD, height*MAX_NEIGHBORHOOD));
  // itsWin->drawImage(zoomXY(itsFoeMap,MAX_NEIGHBORHOOD),0,0);
  
  // Raster::waitForKey();

  return foe;
}

// ######################################################################
Image<float> FoeDetector::getFoeMap
( std::vector <Image<float> > mtFeatures,
  std::vector <Image<float> > mtOptimalShift,
  uint method, bool tempFilter)
{
  Point2D<int> pt = getFoe(mtFeatures, mtOptimalShift, 
                           method, tempFilter);
  LDEBUG("pt: %d %d", pt.i, pt.j);

  return itsFoeMap;
}

// ######################################################################
Point2D<int> FoeDetector::getFoe()
{
  return itsFoe;
}

// ######################################################################
Image<float> FoeDetector::getFoeMap()
{
  return itsFoeMap;
}

// ######################################################################
void FoeDetector::resetDirectionWeights(uint width, uint height)
{
  float length = 1.0;

  itsDirWeights.clear();
  itsDirWeights.resize(width/MAX_NEIGHBORHOOD/FOE_STEP);
  for(uint i = 0; i < width/MAX_NEIGHBORHOOD/FOE_STEP; i++)
    itsDirWeights[i].resize(width/MAX_NEIGHBORHOOD/FOE_STEP);

  for(uint i = 0; i < width/MAX_NEIGHBORHOOD/FOE_STEP; i++)
    for(uint j = 0; j < height/MAX_NEIGHBORHOOD/FOE_STEP; j++)
      itsDirWeights[i][j].resize(itsNumDirs);

  // for each point in the FOE map that we will fill in
  for(uint i = 0; i < width/MAX_NEIGHBORHOOD/FOE_STEP; i++)
    for(uint j = 0; j < height/MAX_NEIGHBORHOOD/FOE_STEP; j++)
      for(uint k = 0; k < itsNumDirs; k++)
        {
          float mangle = (k*360.0)/itsNumDirs;
          
          itsDirWeights[i][j][k] = 
            Image<float>(width/MAX_NEIGHBORHOOD, 
                         height/MAX_NEIGHBORHOOD, NO_INIT);

          // for each point in the MT features map
          for(uint ii = 0; ii < width/MAX_NEIGHBORHOOD; ii++)
            for(uint jj = 0; jj < height/MAX_NEIGHBORHOOD; jj++)
              {
                float val = getDirectionWeight
                  (Point2D<float>(ii,jj), 
                   Point2D<float>(i*FOE_STEP, j*FOE_STEP), length, mangle);

                  itsDirWeights[i][j][k].setVal(ii,jj, val);
              }
        }
}

// ######################################################################
void FoeDetector::computeV1features()
{
  // have 3 (1,2, and 4 pix/fr) spatial shifts
  // have 3 (1/2, 1/3, and 1/4 pix/fr) temporal shifs
  //    --> too slow or fast a movement cannot be detected by human

  Timer tim(1000000);

  // compute the motion energy for each direction and speeds
  for (uint i = 0; i < itsNumDirs; i++)
    {
      tim.reset();
      for (uint j = 0; j < itsNumSpeeds; j++)
        {
          itsRawSpatioTemporalEnergy[i][j] = 
            itsSpatioTemporalPyrBuilders[i][j]->build(itsCurrentImage);
        }

      uint64 t1 = tim.get();
      LINFO("[%3d] ste: %f", i, t1/1000.0);
    }
}

// ######################################################################
float FoeDetector::detectObserverStationarity()
{
  if(itsMTfeatures.size() == 0 ||
     itsMTfeatures[0].getSize() == 0) return -1.0;

  double max = mean(itsMTfeatures[0]);  uint maxInd = 0;
  std::vector<double> maxs(itsNumDirs); double total = 0.0;
  LDEBUG("%3d: %10.3f  max[%3d]: %10.3f ", 0, max, maxInd, max);
  for(uint i = 1; i < itsNumDirs; i++)
    {
      maxs[i] = mean(itsMTfeatures[i]);
      if(maxs[i] > max)
        {
          max = maxs[i];
          maxInd = i;
        }
      total += maxs[i];
      LDEBUG("%3d: %10.3f  max[%3d]: %10.3f ", i, maxs[i], maxInd, max);
    }
  float avg = total/itsNumDirs;
  LINFO("avg activity: %f", avg);
  return avg;
}

// ######################################################################
void FoeDetector::detectObserverRotation()
{
  if(itsMTfeatures.size() == 0 ||
     itsMTfeatures[0].getSize() == 0) return;

  // we do not worry about identical speed throughout the field
  //  to gauge for speed we look for parts that have dominant preference
  double max = maxMean(itsMTfeatures[0]);  uint maxInd = 0;
  std::vector<double> maxs(itsNumDirs); double total = 0.0;
  LDEBUG("%3d: %10.3f  max[%3d]: %10.3f ", 0, max, maxInd, max);
  for(uint i = 1; i < itsNumDirs; i++)
    {
      maxs[i] = maxMean(itsMTfeatures[i]);
      if(maxs[i] > max)
        {
          max = maxs[i];
          maxInd = i;
        }
      total += maxs[i];
      LDEBUG("%3d: %10.3f  max[%3d]: %10.3f ", i, maxs[i], maxInd, max);
    }

  // check ratio of maximum to total 
  double rat = max/total;
  if(rat > 3.0/itsNumDirs) LINFO("Planar motion : %f", rat);

  computeFoeAverage();

  // MAYBE THE TOTAL NEEDS TO GO ABOVE A CERTAIN THRESHOLD: .020 
  // to go above stationary.

  // motion from far away areas has little speed but mostly rotational
  // motion from closer areas has more speed but mostly translational
  //   and also more varied speed.
  // not FOE

  // compute motion opponency ? already computed


  // try SIFT first for correspondences

  //itsCurrentRotMotionDir   = maxInd;
  //itsCurrentRotMotionSpeed = 0.0;
  //LINFO("max[%3d]: %10.3f Speed: %f", maxInd, max, itsCurrentRotMotionSpeed);

  // LINFO("estimating ego-motion");
  //Raster::waitForKey();
}


// ######################################################################
float FoeDetector::maxMean(Image<float> image)
{
  uint width  = image.getWidth();
  uint height = image.getHeight();

  float max = 0.0;  Rectangle rMax;
  for(uint i = 0; i < 5; i++)
    {
      uint sI = uint(i* width/8.0);
      uint eI = uint((i+4)* width/8.0 -1);

      for(uint j = 0; j < 5; j++)
        {
          uint sJ = uint(j* height/8.0);
          uint eJ = uint((j+4)* height/8.0 - 1);
      
          Rectangle r = Rectangle::tlbrI(sJ, sI, eJ, eI);
          LDEBUG("[%3d %3d %3d %3d]", 
                r.top(), r.left(), r.bottomI(), r.rightI());
          Image<float> result = crop(image, r);
          float val = mean(result); 
          LDEBUG("val: %f", val);

          // get the mean magnitude
          if(max < val){ max = val; rMax = r; }
        }
    }
  LDEBUG("[%3d %3d %3d %3d]: %10.3f", 
         rMax.top(), rMax.left(), rMax.bottomI(), rMax.rightI(), max);

  return max;
}

// ######################################################################
void FoeDetector::setObserverRotation(uint dir, float speed)
{
  itsCurrentRotMotionDir   = dir;
  itsCurrentRotMotionSpeed = speed;

  itsCurrentRotMotionDi = 
    itsCurrentRotMotionSpeed *
    cos((2.0*M_PI * itsCurrentRotMotionDir)/itsNumDirs); 
  itsCurrentRotMotionDj =
    itsCurrentRotMotionSpeed *
    sin((2.0*M_PI * itsCurrentRotMotionDir)/itsNumDirs);
  
  LINFO("di,j: %f %f", itsCurrentRotMotionDi, itsCurrentRotMotionDj);
}

// ######################################################################
void FoeDetector::setObserverRotation(float di, float dj)
{
  itsCurrentRotMotionDi = di;
  itsCurrentRotMotionDj = dj;

  float angle = atan2(dj,di) / M_PI * 180.0;              
  float nangle = fmod(angle+360.0, 360.0);
  
  float max = 360.0; uint dir = 0;
  for(uint i = 0; i <itsNumDirs; i++)
    {
      float mangle = (i*360.0)/itsNumDirs;

      float diff1 = fabs(nangle - mangle);
      float diff2 = 360.0 - diff1;
      float diff = diff1; if(diff1 > diff2) diff = diff2;

      if(diff < max) { max = diff; dir = i; }
    }

  itsCurrentRotMotionDir   = dir;
  itsCurrentRotMotionSpeed = sqrt(di*di + dj*dj);

  LINFO("di,j: %f %f", itsCurrentRotMotionDi, itsCurrentRotMotionDj);
}

// ######################################################################
void FoeDetector::correctForObserverRotation()
{
  // vector addition 
  // between rotational change and FOE templates


//   itsCurrentRotMotionDirection;
//   itsCurrentRotMotionSpeed;
  
//   float dX = 
//     itsCurrentRotMotionSpeed *
//     cos((2.0*M_PI * itsCurrentRotMotionDirection)/itsNumDirs); 
//   float dY = 
//     itsCurrentRotMotionSpeed *
//     sin((2.0*M_PI * itsCurrentRotMotionDirection)/itsNumDirs); 



  // NOTE: maybe try idealized rotational motion first
  //    moving to the left
  //    moving at an angle
  //    combine moving and FOE
}

// ######################################################################
Point2D<int> FoeDetector::computeFoeTemplate()
{
  Point2D<int> foeLoc(-1, -1);
  if(itsMTfeatures[0].size() == 0) return foeLoc;

  int width  = itsMTfeatures[0].getWidth();
  int height = itsMTfeatures[0].getHeight();
  itsFoeMap = Image<float>(width,height, ZEROS);

  int foeStep = FOE_STEP;
  int border = 2*foeStep;
  Timer tim(1000000);

  LDEBUG("dir: %d sp: %f", itsCurrentRotMotionDir, itsCurrentRotMotionSpeed);

  // compute FOE value for all desired locations

  // good result is about 3.6% 
  // max out at about 5%, will multiply by 20.0 
  // to make it 100% or bigger in a few cases
  // we divide with width * height to normalize on size of image as well
  float normalizer = 20.0/width/height;
  float max = 0.0; float min = 10000.0;
  for(int i = border; i < (width - border)-6*foeStep; i+=foeStep)
    {
      for(int j = border+ 2*foeStep; j < (height - border)-6*foeStep; j+=foeStep)
        {
          tim.reset();
          float val = computeFoeTemplateValue(i,j) * normalizer;

          // NOTE FIX: maybe have 0 flow threshold to say that there is no FOE
          if(val > max) 
            {
              foeLoc = Point2D<int>(i*MAX_NEIGHBORHOOD,j*MAX_NEIGHBORHOOD);
              max = val;
            }

          if(val < min) min = val;
          itsFoeMap.setVal(i,j,val);
          drawFilledRect
            (itsFoeMap,Rectangle::tlbrI(j,i,j+foeStep-1,i+foeStep-1), val);

          //LINFO("(%13.3f) [%3d %3d]: %15.5f", tim.get()/1000.0, i,j, val);
        }
    }
  LINFO("Current FOE: (%3d %3d): %15.5f", foeLoc.i, foeLoc.j, max);

  for(int i = 0; i < width; i+=foeStep)
    {
      for(int j = 0; j < height; j+=foeStep)
        {
          if((i < border) || 
             (j < border + 2*foeStep) ||
             (i >= (width - border)-6*foeStep) || 
             (j >= (height - border)-6*foeStep)) 
            {
              drawFilledRect
                (itsFoeMap,Rectangle::tlbrI(j,i,j+foeStep-1,i+foeStep-1), min);
              //LINFO("[%3d %3d]: %15.5f", i,j, min);
            }
        }
    }

 // if(itsWin.is_invalid())
 //   itsWin.reset(new XWinManaged(Dims(width,height), 
 //                                width+10, 0, "FOE Template: FOE Detector"));
 // itsWin->setDims(Dims(width*MAX_NEIGHBORHOOD, height*MAX_NEIGHBORHOOD));
 // itsWin->drawImage(zoomXY(itsFoeMap,MAX_NEIGHBORHOOD),0,0);
  
 // FILE *fp; 
 // LINFO("FOE%d %d", foeLoc.i, foeLoc.j);
 // if((fp = fopen("ST_PS_Germany.txt","at")) == NULL) LFATAL("not found");
 // fputs(sformat("%d %d \n", foeLoc.i, foeLoc.j).c_str(), fp);
 // fclose (fp);  

 // Raster::waitForKey();

  return foeLoc;
}

// ######################################################################
float FoeDetector::computeFoeTemplateValue(uint foeI, uint foeJ)
{
  float result = 0.0;

  int width  = itsFoeMap.getWidth();
  int height = itsFoeMap.getHeight(); 
  float diag = sqrt(width*width + height*height);
  //float range = 360.0/double(itsNumDirs)/2.0;

  Image<float>::iterator mtT[itsNumDirs];
  for (unsigned int i=0; i < itsNumDirs; i++)
    mtT[i] = itsMTfeatures[i].beginw();

  Image<float>::iterator mtosT[itsNumDirs];
  for (unsigned int i=0; i < itsNumDirs; i++)
    mtosT[i] = itsMToptimalShift[i].beginw();

  // go through each direction
  for(uint dir = 0; dir < itsNumDirs; dir++)
    {
//       Image<float> distWeight(width, height, ZEROS);
//       Image<float> dirWeight (width, height, ZEROS);
//       Image<float>::iterator tmtT = distWeight.beginw();
//       Image<float>::iterator tmtT = dirWeight.beginw();

      float mangle = (dir*360.0)/itsNumDirs;

      // go through each point
      for(int j = 0; j < height; j++)
        {
          for(int i = 0; i < width; i++)
            {
              // the vector distance from this FOE
              float di = float(i) - float(foeI);
              float dj = float(j) - float(foeJ);
              float dist = sqrt(pow(di, 2.0) + pow(dj, 2.0)); 

              // check the quadrant 
//              uint quad = 0; 
//               if(nangle <= range || nangle >= (360.0 - range))
//                 {
//                   quad = 0;
//                 }
//               else
//                 {
//                   for(uint k = 1; k < itsNumDirs; k++)
//                     {
//                       float lang = k * 360.0/double(itsNumDirs) - range;
//                       float rang = k * 360.0/double(itsNumDirs) + range;
//                       if(nangle >= lang && nangle <= rang)
//                         {
//                           quad = k; k = itsNumDirs;
//                         }
//                     }
//                 }

              // weigh contribution with direction 
              float length = *mtosT[dir]++;              
              float wDir = getDirectionWeight
                (Point2D<float>(i,j), Point2D<float>(foeI, foeJ), length, mangle);

              // NOTE: USING PRE-COMPUTED WEIGHTS
              //float wDir = itsDirWeights[foeI/FOE_STEP][foeJ/FOE_STEP][dir].getVal(i,j);
              

              //LINFO("(%d %d)(%d %d -> %d %d) %f %f", 
              //      i,j, foeI,foeJ,foeI/FOE_STEP, foeJ/FOE_STEP, wDir, wDir2);              

             // weight contribution with distance:
              //   the closer the location to FOE 
              //   the less reliable it is
              float wLoc = 1.0; 
              if(dist/diag < 1.0/8.0) wLoc = .1;
              
              //distWeight.setVal(i,j, wLoc);

              float val = *mtT[dir]++;
              //result += wDir*val;
              result += wDir*wLoc*val;

              
            }
        }
    }

  // NOTE: FIXXX: find more constraints for FOE!!!

  // 6DOF observers: 2trans, 1heading, 3 rot (pitch, yaw, roll) 
  //   Roll is minimal
  // speed is not recoverable from flow field
  
  // Deal with rotation in yaw --> 4 rotation presets

  // take care of noise in the max
  return result;
}

// ######################################################################
float FoeDetector::getDirectionWeight
  (Point2D<float> pt, Point2D<float> foe, float length, float mangle)
{  
  if(length == 0.0) return 0.0;
  //if(mangle == 0.0 || mangle == 180.0) return 0.0;
  //if(length*3.0 < itsCurrentRotMotionSpeed) return 0.0; 

  // observer rotational motion
  float mI = itsCurrentRotMotionDi;
  float mJ = itsCurrentRotMotionDj;

  // delta in the middle
  float odi = (float(pt.i) - float(foe.i));
  float odj = (float(pt.j) - float(foe.j));
  float dist = sqrt(odi*odi + odj*odj); 
  if(dist == 0.0) dist = 1.0;

  float di = length*(float(pt.i) - float(foe.i))/dist + mI;
  float dj = length*(float(pt.j) - float(foe.j))/dist + mJ;

  float angle = atan2(dj,di) / M_PI * 180.0;              
  float nangle = fmod(angle +360.0, 360.0);

  // find difference in direction
  float diff1 = fabs(nangle - mangle);
  float diff2 = 360.0 - diff1;
  float diff = diff1; if(diff1 > diff2) diff = diff2;

//   if((foe.i == 20 && foe.j == 16) &&
//      (odi == odj) && (odi > 12 && fmod(odi,4.0) == 0.0) && 
//      length != 0.0)
//     {
//       float oangle = atan2(odj,odi) / M_PI * 180.0;              
//       float onangle = fmod(oangle +360.0, 360.0);
      
//       float odiff1 = fabs(onangle - mangle);
//       float odiff2 = 360.0 - odiff1;
//       float odiff = odiff1; if(odiff1 > odiff2) odiff = odiff2;

//       LINFO("(%3d %3d), FOE(%3d %3d): Motion[[%8.3f %8.3f]] "
//             "mangle: %8.3f, length = %8.3f " 
//             "angle %8.3f -> %8.3f diff: %8.3f -> %8.3f ", 
//             pt.i, pt.j, foe.i, foe.j, 
//             mI, mJ, 
//             mangle, length,
//             onangle, nangle, odiff, diff);
//     }



//   float width  = itsFOEmap.getWidth();
//   float height = itsFOEmap.getHeight();

//   //===================================================
//   // top left delta
//   float tli = pt.i - 1.0F; if(tli < 0) tli = 0.0F;
//   float tlj = pt.j - 1.0F; if(tlj < 0) tlj = 0.0F; 

//   float tldi = tli - float(foe.i);
//   float tldj = tlj - float(foe.j);

//   float tlAngle = atan2(tldj,tldi) / M_PI * 180.0;              
//   float tlNangle = fmod(tlAngle +360.0, 360.0);

//   // angle is circular
//   float tlDiff1 = fabs(tlNangle - mangle);
//   float tlDiff2 = 360.0 - tlDiff1;
//   float tlDiff = tlDiff1; if(tlDiff1 > tlDiff2) tlDiff = tlDiff2;

//   //===================================================    
//   // top right delta
//   float tri = pt.i + 1.0F; if(tri > width-1) tri = width-1;
//   float trj = pt.j - 1.0F; if(trj < 0) trj = 0.0F; 

//   float trdi = tri - float(foe.i);
//   float trdj = trj - float(foe.j);

//   float trAngle = atan2(trdj,trdi) / M_PI * 180.0;              
//   float trNangle = fmod(trAngle +360.0, 360.0);

//   // angle is circular
//   float trDiff1 = fabs(trNangle - mangle);
//   float trDiff2 = 360.0 - trDiff1;
//   float trDiff = trDiff1; if(trDiff1 > trDiff2) trDiff = trDiff2;

//   //===================================================
//   // bottom left delta
//   float bli = pt.i - 1.0F; if(bli < 0) bli = 0.0F;
//   float blj = pt.j + 1.0F; if(blj > height-1) blj = height-1; 

//   float bldi = bli - float(foe.i);
//   float bldj = blj - float(foe.j);

//   float blAngle = atan2(bldj,bldi) / M_PI * 180.0;              
//   float blNangle = fmod(blAngle +360.0, 360.0);

//   // angle is circular
//   float blDiff1 = fabs(blNangle - mangle);
//   float blDiff2 = 360.0 - blDiff1;
//   float blDiff  = blDiff1; if(blDiff1 > blDiff2) blDiff = blDiff2;

//   //===================================================
//   // bottom right delta
//   float bri = pt.i - 1.0F; if(bri > width-1)  bri = width-1;
//   float brj = pt.j - 1.0F; if(brj > height-1) brj = height-1; 

//   float brdi = bri - float(foe.i);
//   float brdj = brj - float(foe.j);

//   float brAngle = atan2(brdj,brdi) / M_PI * 180.0;
//   float brNangle = fmod(brAngle +360.0, 360.0);

//   // angle is circular
//   float brDiff1 = fabs(brNangle - mangle);
//   float brDiff2 = 360.0 - brDiff1;
//   float brDiff  = blDiff1; if(brDiff1 > brDiff2) brDiff = brDiff2;


//   // find the smallest angles  
//   diff = tlDiff;
//   if(diff > trDiff) diff = trDiff;
//   if(diff > blDiff) diff = blDiff;
//   if(diff > brDiff) diff = brDiff;


  float val = 0.0;
  //float stdang = 30.0;

  // we will use a half cosine rectified weights
  if(diff <= 90.0) val = cos(diff/180.0*M_PI)*.4;

  // LINFO("cos[90]: %f cos(45): %f cos[0]: %f ",
  //       cos(90.0/180.0*M_PI),
  //       cos(45.0/180.0*M_PI),
  //       cos( 0.0/180.0*M_PI));
  // Raster::waitForKey();

  //  // difference between 0 and 30 degrees
  //  if(diff >= 0 && diff < stdang)
  //    val = (stdang - diff)/stdang *    0.1 + 0.3;
  //    //  val = .4;

  //  // difference between 30 and 60 degrees
  //  else if(diff >= stdang && diff < 2*stdang)
  //    val = (2*stdang - diff)/stdang *  0.2 + 0.1;

  //  // difference between 60 and 90 degrees
  //  else if(diff >= 2*stdang && diff < 3*stdang)
  //    val = (3*stdang - diff)/stdang *  0.1;

  // // difference between 150 and 180 degrees
  // else
  //   val = (6*stdang - diff)/stdang * 0.1 + -0.4;

  // // OR JUST:
  // // difference between 0 and 90 degrees
  // // NOTE: for aperture problem: doesn't work well, however
  // //if(diff >= 0 && diff < 3*stdang) val = .4;

  // // difference between 90 and 120 degrees
  // else if(diff >= 3*stdang && diff < 4*stdang)
  //   val = (4*stdang - diff)/stdang * 0.1 + -0.1;

  // // difference between 120 and 150 degrees
  // else if(diff >= 4*stdang && diff < 5*stdang)
  //   val = (5*stdang - diff)/stdang * 0.2 + -0.3;

  // // difference between 150 and 180 degrees
  // else
  //   val = (6*stdang - diff)/stdang * 0.1 + -0.4;

  return val;
}

// ######################################################################
float FoeDetector::getDirectionWeight2(uint quad, uint dir)
{
  if(quad == dir) return 1.00;
  else            return 0.25;

  // need more fancy computation

  // using distance from quadrants: min(x, x-NUM_DIR/2, x+NUM_DIR/2)
}

// ######################################################################
Point2D<int> FoeDetector::computeFoeTemplate
(rutz::shared_ptr<OpticalFlow> flow)
{
  Point2D<int> foeLoc(-1,-1);

  Dims imSize = flow->getImageDims();
  int width  = imSize.w()/MAX_NEIGHBORHOOD;
  int height = imSize.h()/MAX_NEIGHBORHOOD;
  itsFoeMap = Image<float>(width,height, ZEROS);

  int foeStep = FOE_STEP;
  int border  = 2*foeStep;
  Timer tim(1000000);

  LDEBUG("dir: %d sp: %f", itsCurrentRotMotionDir, itsCurrentRotMotionSpeed);

  // compute FOE value

  // good result is about 3.6%_FIXXXX
  // max out at about 5%, will multiply by 20.0_FIXXXX 
  // to make it 100% or bigger in a few cases
  // we divide with width * height to normalize on size of image as well
  float normalizer = 20.0/width/height;
  float max = 0.0; float min = 10000.0;
  for(int i = border; i < (width - border); i+=foeStep)
    {
      for(int j = border; j < (height - border); j+=foeStep)
        {
          tim.reset();
          float val = computeFoeTemplateValue
            (i*MAX_NEIGHBORHOOD,j*MAX_NEIGHBORHOOD, flow) * normalizer;

          // NOTE FIX: have 0 flow threshold to say that there is no FOE    
          if(val > max) 
            {
              foeLoc = Point2D<int>(i*MAX_NEIGHBORHOOD,j*MAX_NEIGHBORHOOD);
              max = val;
            }

          if(val < min) min = val;

          itsFoeMap.setVal(i,j,val);
          drawFilledRect
            (itsFoeMap,Rectangle::tlbrI(j,i,j+foeStep-1,i+foeStep-1), val);
          //LINFO("(%13.3f) [%3d %3d]: %15.5f", tim.get()/1000.0, i,j, val);
        }
    }
  LINFO("Current FOE: (%3d %3d): %15.5f", foeLoc.i, foeLoc.j, max);

  for(int i = 0; i < width; i+=foeStep)
    {
      for(int j = 0; j < height; j+=foeStep)
        {
          if((i < border) || 
             (j < border) ||
             (i >= (width - border)) || 
             (j >= (height - border))) 
            {
              drawFilledRect
                (itsFoeMap,Rectangle::tlbrI(j,i,j+foeStep-1,i+foeStep-1), min);
              //LINFO("[%3d %3d]: %15.5f", i,j, min);
            }
        }
    }

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(width,height), 
  //                                width+10, 0, "FOE Template: FOE Detector"));
  // itsWin->setDims(Dims(width*MAX_NEIGHBORHOOD, height*MAX_NEIGHBORHOOD));
  // itsWin->drawImage(zoomXY(itsFoeMap,MAX_NEIGHBORHOOD),0,0);
  // //Raster::waitForKey();

  return foeLoc;
}

// ######################################################################
float FoeDetector::computeFoeTemplateValue
(uint foeI, uint foeJ, rutz::shared_ptr<OpticalFlow> flow)
{
  float result = 0.0;

  Dims imSize = flow->getImageDims();
  int width  = imSize.w();
  int height = imSize.h(); 
  float diag = sqrt(width*width + height*height);
  //float range = 360.0/double(itsNumDirs)/2.0;

  std::vector<rutz::shared_ptr<FlowVector> > flowVectors =
    flow->getFlowVectors();

  // go through each point
  for(uint i = 0; i < flowVectors.size(); i++)
    {
      // FIXXX: to speed things up compute these motion vectors outside!!!
      // get the corresponding points
      Point2D<float> pt1 = flowVectors[i]->p1;
      //Point2D<float> pt2 = flowVectors[i]->p2;
      float mangle       = flowVectors[i]->angle;
      float length       = flowVectors[i]->mag;
      //float val = flowVectors[i].val; // always 1 in LucasKanade

      //LINFO("[%3d] flow: 1[%13.4f %13.4f] 2[%13.4f %13.4f] ang: %f mag: %f val: %f",
      //     i, pt1.i, pt1.j, pt2.i, pt2.j, mangle, length, val);

      // the vector distance from this FOE
      float di = float(pt1.i) - float(foeI);
      float dj = float(pt1.j) - float(foeJ);
      float dist = sqrt(pow(di, 2.0) + pow(dj, 2.0)); 

      // weigh contribution with direction 
      float wDir = getDirectionWeight
        (pt1, Point2D<float>(foeI, foeJ), length, mangle);

      // NOTE: USING PRE-COMPUTED WEIGHTS
      //float wDir = itsDirWeights[foeI/FOE_STEP][foeJ/FOE_STEP][dir].getVal(i,j);
              
      //LINFO("(%d %d)(%d %d -> %d %d) %f %f", 
      //      i,j, foeI,foeJ,foeI/FOE_STEP, foeJ/FOE_STEP, wDir, wDir2);              

      // weight contribution with distance:
      //   the closer the location to FOE 
      //   the less reliable it is
      float wLoc = 1.0; 
      if(dist/diag < 1.0/8.0) wLoc = .1;
              
      //distWeight.setVal(i,j, wLoc);

      // assume all motion detection strength to be equal
      //result += wDir;
      result += wDir*wLoc;//*val;
    }

  // NOTE: FIXXX: find more constraints for FOE!!!

  // 6DOF observers: 2trans, 1heading, 3 rot (pitch, yaw, roll) 
  //   Roll is minimal
  // speed is not recoverable from flow field
  
  // Deal with rotation in yaw --> 4 rotation presets

  // take care of noise in the max
  return result;
}

// ######################################################################
Point2D<int> FoeDetector::computeFoeAverage()
{
  Point2D<int> foeLoc(-1, -1);
  if(itsMTfeatures[0].size() == 0) return foeLoc;

  float maxShift = 16.0;

  uint width  = itsMTfeatures[0].getWidth();
  uint height = itsMTfeatures[0].getHeight();

  Image<float> tempH(width, height, ZEROS);
  Image<float> tempV(width, height, ZEROS);
  std::vector<Point2D<float> > hVals; std::vector<float> hW;
  std::vector<Point2D<float> > vVals; std::vector<float> vW;

  // get the horizontal projection of the y-component of each point 
  // and the vertical   projection of the x-component of each point 
  for(uint d = 0; d < itsNumDirs; d++)
    {
      float sinAng = sin((2.0*M_PI * d)/itsNumDirs); 
      float cosAng = cos((2.0*M_PI * d)/itsNumDirs); 
      //LINFO("dir[%d]: c:%10.3f s:%10.3f", d, cosAng, sinAng);

      for(uint i = 0; i < width; i++)
        for(uint j = 0; j < height; j++)
          {
            float val = itsMTfeatures[d].getVal(i,j);
            //LINFO("[%3d %3d]: %10.3f", i,j,val);

            float len = itsMToptimalShift[d].getVal(i,j);
            float dy = len * sinAng;
            float dx = len * cosAng;
            
            uint lH = uint(width/2  - dy/maxShift*width/2);
            uint lV = uint(height/2 - dx/maxShift*height/2);
            if(lH == width) lH--; if(lV == height) lV--;
            
            //                 LINFO("[%3d, %3d]:l %10.3f:(%10.3f %10.3f): tHV; %3d %13d ",
            //                       i,j, len, dy, dx, lH, lV);
            
            float tH = tempH.getVal(lH, j);
            float tV = tempV.getVal(i, lV);

            if(val > .20)
              {                
                tempH.setVal(lH, j, val + tH);
                tempV.setVal(i, lV, val + tV);

                hVals.push_back(Point2D<float>(j,dy)); hW.push_back(val);
                vVals.push_back(Point2D<float>(i,dx)); vW.push_back(val);            
              }
          }
    }

  // weighted linear regression to find zero crossings
  double sumXY = 0.0; double sumX  = 0.0; double sumY  = 0.0; double sumX2 = 0.0;
  double wsum = 0.0;
  for(uint i = 0; i < hVals.size(); i++)
    {
      wsum  += hW[i];

      // sum(xy)
      sumXY += hW[i] * (hVals[i].i * hVals[i].j);
      
      // sum(x)
      sumX  += hW[i] * hVals[i].i;
      
      // sum(y)
      sumY  += hW[i] * hVals[i].j;
      
      // sum(x^2)
      sumX2 += hW[i] * hVals[i].i*hVals[i].i;
    }

  double n = wsum;
  double mH = (n*sumXY - sumX*sumY)/(n*sumX2 - sumX*sumX);
  double bH = (sumY - mH*sumX)/n; 
  foeLoc.j = -bH/mH * MAX_NEIGHBORHOOD;
  LINFO("x = %10.5fx + %10.5f: %10.5f", mH, bH,-bH/mH);

  sumXY = 0.0; sumX  = 0.0; sumY  = 0.0; sumX2 = 0.0;
  wsum = 0.0;
  for(uint i = 0; i < vVals.size(); i++)
    {
      wsum  += vW[i];

      // sum(xy)
      sumXY += vW[i] * (vVals[i].i * vVals[i].j);
      
      // sum(x)
      sumX  += vW[i] * vVals[i].i;
      
      // sum(y)
      sumY  += vW[i] * vVals[i].j;
      
      // sum(x^2)
      sumX2 += vW[i] * vVals[i].i*vVals[i].i;
    }

  n = wsum;
  float mV = (n*sumXY - sumX*sumY)/(n*sumX2 - sumX*sumX);
  float bV = (sumY - mV*sumX)/n; 
  foeLoc.i = -bV/mV * MAX_NEIGHBORHOOD;
  LINFO("y = %10.5fx + %10.5f: %10.5f", mV, bV,-bV/mV);

  LINFO("FOE: (%3d %3d)", foeLoc.i, foeLoc.j);

  tempH = zoomXY(tempH, MAX_NEIGHBORHOOD);
  tempV = zoomXY(tempV, MAX_NEIGHBORHOOD);
  uint dispW = width*MAX_NEIGHBORHOOD;
  uint dispH = height*MAX_NEIGHBORHOOD;

  float spH = mH*0         + bH;
  float epH = mH*(height-1) + bH;

  Point2D<int> sH((width/2 - spH/8.0*width/2)*MAX_NEIGHBORHOOD,        0);
  Point2D<int> eH((width/2 - epH/8.0*width/2)*MAX_NEIGHBORHOOD,  dispH-1);

  float spV = mV*0         + bV;
  float epV = mV*(width-1) + bV;
  Point2D<int> sV(0,       (height/2 - spV/8.0*height/2)*MAX_NEIGHBORHOOD);
  Point2D<int> eV(dispW-1, (height/2 - epV/8.0*height/2)*MAX_NEIGHBORHOOD);


  float mn, mx; 
  getMinMax(tempH, mn,mx);
  drawLine(tempH, sH, eH, mx, 1);
  drawLine(tempH, Point2D<int>(dispW/2,0),Point2D<int>(dispW/2,dispH-1),mx);
  drawLine(tempH, Point2D<int>(0,dispH/2),Point2D<int>(dispW-1,dispH/2),mx);
  drawLine(tempH, Point2D<int>(dispW-1,0),Point2D<int>(dispW-1,dispH-1),mx);
  drawDisk(tempH, Point2D<int>(dispW/2, foeLoc.j), 4, mx);

  getMinMax(tempV, mn,mx);
  drawLine(tempV, sV, eV, mx, 1);
  drawLine(tempV, Point2D<int>(dispW/2,0),Point2D<int>(dispW/2,dispH-1),mx);
  drawLine(tempV, Point2D<int>(0,dispH/2),Point2D<int>(dispW-1,dispH/2),mx);
  drawDisk(tempV, Point2D<int>(foeLoc.i, dispH/2), 4, mx);

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged
  //                (Dims(2*dispW, dispH), dispW+10, 0, "FOE Detector"));
  // Image<float> disp(2*dispW, dispH, ZEROS);
  // inplacePaste(disp, tempH, Point2D<int>(0,0));
  // inplacePaste(disp, tempV, Point2D<int>(dispW,0));
  // itsWin->setDims(Dims(2*dispW, dispH));
  // itsWin->drawImage(disp,0,0);
  // LINFO("Foe Estimation: AVERAGE");
  // Raster::waitForKey();



  // FoeMap:
  //   just draw a gaussian blob centered at the found FOE
  itsFoeMap = gaussianBlob<float>(Dims(width, height),
                                  Point2D<int>(-bV/mV, -bH/mH), 7, 7);  
  inplaceNormalize(itsFoeMap, 0.0F, 1.0F);


  return foeLoc;

//   itsWin->setDims(itsFoeMap.getDims());
//   itsWin->drawImage(itsFoeMap,0,0);
//   getMinMax(itsFoeMap,mn,mx);
//   LINFO("Foe Map: %f %f",mn,mx);
//   Raster::waitForKey();
}

// ######################################################################
Point2D<int> FoeDetector::computeFoeAverage
(rutz::shared_ptr<OpticalFlow> flow)
{
  Point2D<int> foeLoc(-1, -1);

  float maxShift = 16.0;

  Dims imSize = flow->getImageDims();
  uint width  = imSize.w()/MAX_NEIGHBORHOOD;
  uint height = imSize.h()/MAX_NEIGHBORHOOD;

  Image<float> tempH(width, height, ZEROS);
  Image<float> tempV(width, height, ZEROS);
  std::vector<Point2D<float> > hVals; std::vector<float> hW;
  std::vector<Point2D<float> > vVals; std::vector<float> vW;

  std::vector<rutz::shared_ptr<FlowVector> > flowVectors =
    flow->getFlowVectors();

  // go through each point
  // get the horizontal projection of the y-component of each point 
  // and the vertical   projection of the x-component of each point 
  for(uint i = 0; i < flowVectors.size(); i++)
    {
      // get the corresponding points
      Point2D<float> pt1 = flowVectors[i]->p1;
      //Point2D<float> pt2 = flowVectors[i]->p2;
      float angle        = flowVectors[i]->angle;
      float length       = flowVectors[i]->mag;
      float val          = 1;//flowVectors[i].val; // always 1 in LucasKanade

      float sinAng = sin(angle/180.0*M_PI); 
      float cosAng = cos(angle/180.0*M_PI); 
      //LINFO("[%f %f] [%f %f] c:%10.3f s:%10.3f", 
      //      pt1.i, pt1.j, pt2.i, pt2.j, cosAng, sinAng);

      float dy = length * sinAng; 
      if(dy >  maxShift) dy = maxShift;
      if(dy < -maxShift) dy = -maxShift;

      float dx = length * cosAng; 
      if(dx >  maxShift) dx = maxShift;
      if(dx < -maxShift) dx = -maxShift;

      uint lH = uint(width/2  - dy/maxShift*width/2);
      uint lV = uint(height/2 - dx/maxShift*height/2);
      if(lH == width) lH--; if(lV == height) lV--;
      if(lH == width) lH--; if(lV == height) lV--;

      int pi = int(pt1.i/MAX_NEIGHBORHOOD);
      int pj = int(pt1.j/MAX_NEIGHBORHOOD);

      //LINFO("{%f %f}[%f %f]  %d %d - %d %d", 
      //      length, angle, dy, dx, lH, pj, pi, lV);

      float tH = tempH.getVal(lH, pj);
      float tV = tempV.getVal(pi, lV);

      if(val > .20)
        {                
          tempH.setVal(lH, pj, val + tH);
          tempV.setVal(pi, lV, val + tV);
          
          hVals.push_back(Point2D<float>(pj,dy)); hW.push_back(val);
          vVals.push_back(Point2D<float>(pi,dx)); vW.push_back(val);            
        }
    }

  //===================================================================

  // weighted linear regression to find zero crossings
  double sumXY = 0.0; double sumX  = 0.0; 
  double sumY  = 0.0; double sumX2 = 0.0;
  double wsum  = 0.0;
  for(uint i = 0; i < hVals.size(); i++)
    {
      wsum  += hW[i];

      // sum(xy)
      sumXY += hW[i] * (hVals[i].i * hVals[i].j);
      
      // sum(x)
      sumX  += hW[i] * hVals[i].i;
      
      // sum(y)
      sumY  += hW[i] * hVals[i].j;
      
      // sum(x^2)
      sumX2 += hW[i] * hVals[i].i*hVals[i].i;
    }

  double n = wsum;
  double mH = (n*sumXY - sumX*sumY)/(n*sumX2 - sumX*sumX);
  double bH = (sumY - mH*sumX)/n; 
  foeLoc.j = -bH/mH * MAX_NEIGHBORHOOD;
  LINFO("x = %10.5fx + %10.5f: %10.5f", mH, bH,-bH/mH);

  sumXY = 0.0; sumX  = 0.0; sumY  = 0.0; sumX2 = 0.0;
  wsum = 0.0;
  for(uint i = 0; i < vVals.size(); i++)
    {
      wsum  += vW[i];

      // sum(xy)
      sumXY += vW[i] * (vVals[i].i * vVals[i].j);
      
      // sum(x)
      sumX  += vW[i] * vVals[i].i;
      
      // sum(y)
      sumY  += vW[i] * vVals[i].j;
      
      // sum(x^2)
      sumX2 += vW[i] * vVals[i].i*vVals[i].i;
    }

  n = wsum;
  float mV = (n*sumXY - sumX*sumY)/(n*sumX2 - sumX*sumX);
  float bV = (sumY - mV*sumX)/n; 
  foeLoc.i = -bV/mV * MAX_NEIGHBORHOOD;
  LINFO("y = %10.5fx + %10.5f: %10.5f", mV, bV,-bV/mV);

  LINFO("FOE: (%3d %3d)", foeLoc.i, foeLoc.j);

  // FIXXXX: need to compute the quality of the fit
  //for(uint i = 0; i < hVals.size(); i++)
  //  {
  //   // compute the error between the point
  //   // and the predicted location by the equation
  //   
  //  // accumulate 
  //  }
  // divide total with number of points
  // normalize error and flip (1 - err)
  
  // do the same for the vertical points

  // average the two dimensions

  // display the result
  tempH = zoomXY(tempH, MAX_NEIGHBORHOOD);
  tempV = zoomXY(tempV, MAX_NEIGHBORHOOD);
  uint dispW = width*MAX_NEIGHBORHOOD;
  uint dispH = height*MAX_NEIGHBORHOOD;

  float spH = mH*0         + bH;
  float epH = mH*(height-1) + bH;

  Point2D<int> sH((width/2 - spH/8.0*width/2)*MAX_NEIGHBORHOOD,        0);
  Point2D<int> eH((width/2 - epH/8.0*width/2)*MAX_NEIGHBORHOOD,  dispH-1);

  float spV = mV*0         + bV;
  float epV = mV*(width-1) + bV;
  Point2D<int> sV(0,       (height/2 - spV/8.0*height/2)*MAX_NEIGHBORHOOD);
  Point2D<int> eV(dispW-1, (height/2 - epV/8.0*height/2)*MAX_NEIGHBORHOOD);


  float mn, mx; 
  getMinMax(tempH, mn,mx);
  drawLine(tempH, sH, eH, mx, 1);
  drawLine(tempH, Point2D<int>(dispW/2,0),Point2D<int>(dispW/2,dispH-1),mx);
  drawLine(tempH, Point2D<int>(0,dispH/2),Point2D<int>(dispW-1,dispH/2),mx);
  drawLine(tempH, Point2D<int>(dispW-1,0),Point2D<int>(dispW-1,dispH-1),mx);
  drawDisk(tempH, Point2D<int>(dispW/2, foeLoc.j), 4, mx);

  getMinMax(tempV, mn,mx);
  drawLine(tempV, sV, eV, mx, 1);
  drawLine(tempV, Point2D<int>(dispW/2,0),Point2D<int>(dispW/2,dispH-1),mx);
  drawLine(tempV, Point2D<int>(0,dispH/2),Point2D<int>(dispW-1,dispH/2),mx);
  drawDisk(tempV, Point2D<int>(foeLoc.i, dispH/2), 4, mx);

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged
  //                (Dims(2*dispW, dispH), dispW+10, 0, "FOE Detector: AVG"));
  // Image<float> disp(2*dispW, dispH, ZEROS);
  // inplacePaste(disp, tempH, Point2D<int>(0,0));
  // inplacePaste(disp, tempV, Point2D<int>(dispW,0));
  // itsWin->setDims(Dims(2*dispW, dispH));
  // itsWin->drawImage(disp,0,0);
  //Raster::waitForKey();
 
  // FoeMap:
  //   just draw a gaussian blob centered at the found FOE
  itsFoeMap = gaussianBlob<float>(Dims(width, height),
                                  Point2D<int>(-bV/mV, -bH/mH), 7, 7);  
  inplaceNormalize(itsFoeMap, 0.0F, 1.0F);

  // itsWin->setDims(itsFoeMap.getDims());
  // itsWin->drawImage(itsFoeMap,0,0);
  // Raster::waitForKey();

  return foeLoc;
}

// ######################################################################
Layout<byte> FoeDetector::getMTfeaturesDisplay(Image<byte> image)
{
  return itsMT->getMTfeaturesDisplay(image);
}

// ######################################################################
void FoeDetector::print
(Image<float> img, 
 uint si, uint ei, uint sj, uint ej, bool stop)
{
  for(uint y = sj; y < ej; y++)
    {
      for(uint x = si; x < ei; x++)
        printf("%10.3f ", img.getVal(x,y));
      printf("\n");
    }
  if(stop) Raster::waitForKey();
}

// ######################################################################
void FoeDetector::display(Image<float> img, std::string info)
{
  LINFO("%s", info.c_str());
  uint w = img.getWidth();
  uint h = img.getHeight();
  itsWin->setDims(Dims(w,h));
  itsWin->drawImage(img,0,0);
  Raster::waitForKey();
}

// ######################################################################
void FoeDetector::display(ImageSet<float> imgs, std::string info)
{
  LINFO("%s", info.c_str());
  uint w = imgs[0].getWidth();
  uint h = imgs[0].getHeight();
  itsWin->setDims(Dims(w,h));

  for(uint i = 0; i < imgs.size(); i++)
    {
      uint scale = uint(pow(2.0, i));
      itsWin->drawImage(zoomXY(imgs[i], scale),0,0);
      Raster::waitForKey();
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
