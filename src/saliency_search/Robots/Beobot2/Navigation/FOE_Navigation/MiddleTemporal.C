/*!@file Robots2/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.C */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>

#include "Robots/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.H"

#include  <cstdio>
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/ShapeOps.H"
#include "Image/SimpleFont.H"

#include "Util/Timer.H"

// ######################################################################
// ######################################################################

// ######################################################################
MiddleTemporal::MiddleTemporal()
{
  itsWin.reset();
}

// ######################################################################
MiddleTemporal::~MiddleTemporal()
{ }

// ######################################################################
std::vector<Image<float> > MiddleTemporal::getMTfeatures()
{
  return itsMTfeatures;
}

// ######################################################################
std::vector<Image<float> > MiddleTemporal::getMToptimalShift()
{
  return itsMToptimalShift;
}

// ######################################################################
void MiddleTemporal::computeMTfeatures
(std::vector<std::vector<ImageSet<float> > > rawSpatioTemporalEnergy)
{
  if(rawSpatioTemporalEnergy[0][0].size() == 0) return;
  itsRawSpatioTemporalEnergy = rawSpatioTemporalEnergy;
  reset();

 // find optimal speed for each direction
  //Timer tim(1000000);
  for (uint i = 0; i < itsNumDirs; i++)
    {
      //tim.reset();
      computeMaxSpeed(i);
      //uint64 t1 = tim.get();
      //LINFO("[%3d] max speed: %f", i, t1/1000.0);
    }

  //displayItsSpatioTemporalEnergy();
  //printItsSpatioTemporalEnergy(140, 158, 45, 85, true, 100.0);

  // compute max in 4x4 top level regions
  computeSteMaxVals();

  // DEBUG
//   LINFO("Max Vals 0");
//   print(itsSteMaxVals[0], 170/4, 188/4, 45/4, 75/4, true);
//   LINFO("MaxVals 1");
//   print(itsSteMaxVals[1], 170/4, 188/4, 45/4, 75/4, true);
  
  // normalize with the max values regardless of direction
  normalizeOnMaxVal();

  // DEBUG
//   LINFO("After Normalize on Max");
//   printItsSpatioTemporalEnergy(170, 188, 45, 75, true);  
 
  // opponencies: NOTE: from opposite direction of motion
  // this will subdue locations that do not have maximum values
  computeDirSteMaxVals();

  // DEBUG
//   LINFO("0 Max Vals 0");
//   print(itsDirSteMaxVals[0][0], 170/4, 188/4, 45/4, 75/4, true);
//   LINFO("0 MaxVals 1");
//   print(itsDirSteMaxVals[0][1], 170/4, 188/4, 45/4, 75/4, true);
  
//   LINFO("4 Max Vals 0");
//   print(itsDirSteMaxVals[4][0], 170/4, 188/4, 45/4, 75/4, true);
//   LINFO("4 MaxVals 1");
//    print(itsDirSteMaxVals[4][1], 170/4, 188/4, 45/4, 75/4, true);

  computeOpponencies();

  // DEBUG
//   LINFO("after opponencies");
//   printItsSpatioTemporalEnergy(170, 188, 45, 75, true);
//   //print(motion, 165/div, 175/div, 45/div, 75/div, true);
//   //print(motion, 175/div, 185/div, 45/div, 75/div, true);
//   //print(motion, 185/div, 195/div, 45/div, 75/div, true);


  // normalize the MT features with the percentage of dominance
  weighMTfeaturesForDominance();

  // find max values for each direction among all the scales
  findMaxMotionVals();

  // DEBUG
  //printItsSpatioTemporalEnergy(170, 188, 45, 75, true, 100.0);

  // DEBUG
  // for ACB: 5,15,30,50
  //printItsMTfeatures(35, 50, 5, 25, true);
  //printItsMToptimalShift(35, 50, 5, 25, true);
  //displayItsMTfeatures();
  //displayItsMToptimalShift();
}

// ######################################################################
void MiddleTemporal::reset()
{
  itsNumDirs       = itsRawSpatioTemporalEnergy.size();
  itsNumSpeeds     = itsRawSpatioTemporalEnergy[0].size();
  itsNumPyrLevels  = itsRawSpatioTemporalEnergy[0][0].size();

  //itsSpatioTemporalPyrs.clear();
  //itsSpatioTemporalPyrs.resize(itsNumDirs);
  //for(uint i = 0; i < itsNumDirs; i++)
  //  itsSpatioTemporalPyrs[i].resize(itsNumSpeeds);

  //itsRawSpatioTemporalEnergy.clear();
  //itsRawSpatioTemporalEnergy.resize(itsNumDirs);
  //for(uint i = 0; i < itsNumDirs; i++)
  //  itsRawSpatioTemporalEnergy[i].resize(itsNumSpeeds);

  itsSpatioTemporalEnergy.resize(itsNumDirs);
  itsSpatioTemporalEnergyOptimalShift.resize(itsNumDirs);
  for(uint i = 0; i < itsNumDirs; i++)
    itsSpatioTemporalEnergyOptimalShift[i].reset(itsNumPyrLevels);

  itsMTfeatures.resize(itsNumDirs);
  itsMToptimalShift.resize(itsNumDirs);
}


// ######################################################################
void MiddleTemporal::computeMaxSpeed(uint index)
{
  int depth = itsRawSpatioTemporalEnergy[index][0].size();
  if(depth == 0) return;
  ImageSet<float> result(depth);

  // compute the motion detection at each location
  for (int scale = 0; scale < depth; scale++)
    result[scale] = computeMaxSpeed(index, scale);

  itsSpatioTemporalEnergy[index] = result;
}

// ######################################################################
Image<float> MiddleTemporal::computeMaxSpeed(uint index, int scale)
{
  // find max among all the spatial and temporal shifts
  // --> basically max among all the shifting values

  uint width  = itsRawSpatioTemporalEnergy[index][0][scale].getWidth();
  uint height = itsRawSpatioTemporalEnergy[index][0][scale].getHeight();
  Image<float> result(width, height, ZEROS);

  itsSpatioTemporalEnergyOptimalShift[index][scale] = 
    Image<float>(width, height, ZEROS);

  Image<float>::iterator resultT = result.beginw();

  Image<float>::iterator 
    shiftT = itsSpatioTemporalEnergyOptimalShift[index][scale].beginw(); 

  Image<float>::iterator resT[itsNumSpeeds];
  for (unsigned int i = 0; i < itsNumSpeeds; i++)
    resT[i] = itsRawSpatioTemporalEnergy[index][i][scale].beginw();
  
  for(uint i = 0; i < width; i++)
    {
      for(uint j = 0; j < height; j++)
        {
          float max = 0.0; float optShift = 0.0;          
          for(uint r = 0; r < itsNumSpeeds; r++)
            {
              float val = *resT[r]++; 
              if(max < val) 
                {
                  max = val;
                  optShift = pow(2.0,r); //FIXXX add case for 1/2, 1/3, 1/4
                }
            }
          *resultT++ = max;
          *shiftT++  = optShift;
        }
    }

  return result;
}

// ######################################################################
void MiddleTemporal::computeSteMaxVals()
{
  itsSteMaxVals.reset(itsNumPyrLevels);
  //itsDirSteMaxVals.resize(itsNumDirs);
  //for(uint i = 0; i < itsNumDirs; i++)
  //  itsDirSteMaxVals[i].reset(itsNumPyrLevels);

  uint width  = itsSpatioTemporalEnergy[0][0].getWidth();
  uint height = itsSpatioTemporalEnergy[0][0].getHeight();  

  // got through each level
  for(uint l = 0; l < itsNumPyrLevels; l++)
    {
      uint pxSize  = uint(pow(2.0, l));
      uint lwidth  = width/pxSize;
      uint lheight = height/pxSize;

      // create a local max map
      // NOTE: is global max biologically plausible?
      uint steps = 1; 
      if(pxSize < MAX_NEIGHBORHOOD) 
        steps = MAX_NEIGHBORHOOD/pxSize;

      // fill in max values for each level
      itsSteMaxVals[l] = Image<float>(lwidth/steps, lheight/steps, NO_INIT);
      //for(uint dir = 0; dir < itsNumDirs; dir++)
      //  itsDirSteMaxVals[dir][l] = 
      //    Image<float>(lwidth/steps, lheight/steps, NO_INIT);

      //LINFO("[%3d] lw/h: %d %d -> %d step: %d", 
      //      l, lwidth, lheight, pxSize, steps);
      
      for(uint i = 0; i < lwidth; i+= steps)
        {
          for(uint j = 0; j < lheight; j+= steps)
            {
              // we find max for all directions
              float max = 0.0;
              for(uint d = 0; d < itsNumDirs; d++)
                {
                  //float dirMax = 0.0;
                  for(uint di = 0; di < steps; di++)
                    {
                      for(uint dj = 0; dj < steps; dj++)
                        {
                          float val = 
                            itsSpatioTemporalEnergy[d][l].getVal(i+di,j+dj);
                          if(val > max) max = val;
                          //if(val > dirMax) dirMax = val;


//                       // DEBUG:
//                       if(i >= 148 && i <= 151 &&
//                          j >=  60 && j <=  63   ) 
//                         {
//                           LINFO("[%d](%3d %3d) d(%3d %3d) -> %3d %3d: %f %f",
//                                 d,i,j, di,dj, i+di,j+dj, val, max); 
//                         }




                        }
                    }
                  //itsDirSteMaxVals[d][l].setVal(i/steps, j/steps, dirMax);
                }
              itsSteMaxVals[l].setVal(i/steps, j/steps, max);
            }
        }      
    }
}

// ######################################################################
void MiddleTemporal::computeDirSteMaxVals()
{
  //itsSteMaxVals.reset(itsNumPyrLevels);
  itsDirSteMaxVals.resize(itsNumDirs);
  for(uint i = 0; i < itsNumDirs; i++)
    itsDirSteMaxVals[i].reset(itsNumPyrLevels);

  uint width  = itsSpatioTemporalEnergy[0][0].getWidth();
  uint height = itsSpatioTemporalEnergy[0][0].getHeight();  

  // got through each level
  for(uint l = 0; l < itsNumPyrLevels; l++)
    {
      uint pxSize  = uint(pow(2.0, l));
      uint lwidth  = width/pxSize;
      uint lheight = height/pxSize;

      // create a local max map
      // NOTE: is global max biologically plausible?
      uint steps = 1; 
      if(pxSize < MAX_NEIGHBORHOOD) 
        steps = MAX_NEIGHBORHOOD/pxSize;

      // fill in max values for each level
      //itsSteMaxVals[l] = Image<float>(lwidth/steps, lheight/steps, NO_INIT);
      for(uint dir = 0; dir < itsNumDirs; dir++)
        itsDirSteMaxVals[dir][l] = 
          Image<float>(lwidth/steps, lheight/steps, NO_INIT);

      //LINFO("[%3d] lw/h: %d %d -> %d step: %d", 
      //      l, lwidth, lheight, pxSize, steps);
      
      for(uint i = 0; i < lwidth; i+= steps)
        {
          for(uint j = 0; j < lheight; j+= steps)
            {
              // we find max for all directions
              //float max = 0.0;
              for(uint d = 0; d < itsNumDirs; d++)
                {
                  float dirMax = 0.0;
                  for(uint di = 0; di < steps; di++)
                    {
                      for(uint dj = 0; dj < steps; dj++)
                        {
                          float val = 
                            itsSpatioTemporalEnergy[d][l].getVal(i+di,j+dj);
                          //if(val > max) max = val;
                          if(val > dirMax) dirMax = val;


//                       // DEBUG:
//                       if(i >= 148 && i <= 151 &&
//                          j >=  60 && j <=  63   ) 
//                         {
//                           LINFO("[%d](%3d %3d) d(%3d %3d) -> %3d %3d: %f %f",
//                                 d,i,j, di,dj, i+di,j+dj, val, max); 
//                         }




                        }
                    }
                  itsDirSteMaxVals[d][l].setVal(i/steps, j/steps, dirMax);
                }
              //itsSteMaxVals[l].setVal(i/steps, j/steps, max);
            }
        }      
    }
}

// ######################################################################
// normalize on the max value regardless of direction
void MiddleTemporal::normalizeOnMaxVal()
{
  uint width  = itsSpatioTemporalEnergy[0][0].getWidth();
  uint height = itsSpatioTemporalEnergy[0][0].getHeight();  

  // got through each level
  for(uint l = 0; l < itsNumPyrLevels; l++)
    {
      uint pxSize  = uint(pow(2.0, l));
      uint lwidth  = width/pxSize;
      uint lheight = height/pxSize;

      uint steps = 1; 
      if(pxSize < MAX_NEIGHBORHOOD) steps = MAX_NEIGHBORHOOD/pxSize;
      for(uint i = 0; i < lwidth; i+= steps)
        {
          for(uint j = 0; j < lheight; j+= steps)
            {
              uint mi = i/steps;
              uint mj = j/steps;
              
              uint lm =  0;              if(mi > 2)              lm = mi - 3; 
              uint rm = lwidth/steps-1;  if(mi < lwidth/steps-4) rm = mi + 3;
              uint tm =  0;              if(mj > 2)              tm = mj - 3; 
              uint bm = lheight/steps-1; if(mj < lheight/steps-4)bm = mj + 3;

              // FIXXX: need to interpolate for levels 3 and higher

//               // DEBUG:
//               if(i >=  148 && i <= 151 &&
//                  j >=  60 && j <=  63   ) 
//                 {
//                   LINFO("(%3d %3d %3d %3d) -> %3d %3d",
//                         lm,rm,tm,bm, mi,mj); 
//                 }


              // get max vals among a neighborhood
              float max = 0.0;
              for(uint bi = lm; bi <= rm; bi++)
                {
                  for(uint bj = tm; bj <= bm; bj++)
                    {
                      float val = itsSteMaxVals[l].getVal(bi, bj);
                      if(max < val) max = val;


//                       // DEBUG:
//                       if(i >= 148 && i <= 151 &&
//                          j >=  60 && j <=  63   ) 
//                         {
//                           LINFO("(%3d %3d) val %10.3f -> max %10.3f",
//                                 bi, bj, val, max); 
//                         }




                    }
                }

              // normalize each point
              for(uint d = 0; d < itsNumDirs; d++)
                {
                  for(uint di = 0; di < steps; di++)
                    {
                      for(uint dj = 0; dj < steps; dj++)
                        {
                          float val = itsSpatioTemporalEnergy[d][l].getVal(i+di,j+dj);

                          if(max == 0.0) 
                            itsSpatioTemporalEnergy[d][l].setVal(i+di,j+dj, 0.0);
                          else
                            itsSpatioTemporalEnergy[d][l].setVal(i+di,j+dj, val/max);



//                           if(val > 30000.0)
//                                LINFO("[%d](%3d %3d) d(%3d %3d) -> %3d %3d: %f/%f = %f",
//                                      d,i,j, di,dj, i+di,j+dj, val, max, val/max); 
                          
//                           // DEBUG:
//                           if(i+di >= 148 && i+di <= 151 &&
//                              j+dj >=  60 && j+dj <=  63   ) 
//                             {
//                               LINFO("[%d](%3d %3d) d(%3d %3d) -> %3d %3d: %f/%f = %f",
//                                     d,i,j, di,dj, i+di,j+dj, val, max, val/max); 
//                             }




                        }
                    }
                }
            }
        }
    }
}

// ######################################################################
void MiddleTemporal::computeOpponencies()
{
  // get lateral inhibition values on each location
  std::vector <ImageSet<float> > osInhibitMap(itsNumDirs);
  std::vector <ImageSet<float> >  nInhibitMap(itsNumDirs);
  for (uint i = 0; i < itsNumDirs; i++)
    {
      osInhibitMap[i].reset(itsSpatioTemporalEnergy[i].size());
      //nInhibitMap[i].reset(itsSpatioTemporalEnergy[i].size());
      for(uint j = 0; j < itsSpatioTemporalEnergy[i].size(); j++)
        {
          osInhibitMap[i][j] = getOnSpotInhibitSpatioTemporalEnergy(i,j);   
          //nInhibitMap[i][j]  = getNeighborInhibitSpatioTemporalEnergy(i,j);
        }
    }

  // perform lateral inhibition on each location
  for (uint i = 0; i < itsNumDirs; i++)
    {
      for(uint j = 0; j < itsSpatioTemporalEnergy[i].size(); j++)
        {         
          itsSpatioTemporalEnergy[i][j] -= osInhibitMap[i][j]; 

//           for (uint k = 0; k < itsNumDirs; k++)
//             {
//               // NOTE FIX maybe add weights 
//               //          according to how disagreeing the direction is
//               if(k != i)
//                 itsSpatioTemporalEnergy[i][j] -= nInhibitMap[i][j]; 
//             }

          // we are going to clamp the result to 0.0;
          float mn,mx; getMinMax(itsSpatioTemporalEnergy[i][j],mn,mx);
          inplaceClamp(itsSpatioTemporalEnergy[i][j], 0.0f, mx);
        }
    }

  // NOTE: ---> how about end-stop neurons for end of bars

  // at this point everything should have been normalized to 0 to 1.0 max
}

// ######################################################################
void MiddleTemporal::weighMTfeaturesForDominance()
{

}

// ######################################################################
Image<float> MiddleTemporal::getOnSpotInhibitSpatioTemporalEnergy
(uint dir, uint scale)
{
  uint oppDir = (dir + itsNumDirs/2)%itsNumDirs;

  uint width  = itsSpatioTemporalEnergy[dir][scale].getWidth();
  uint height = itsSpatioTemporalEnergy[dir][scale].getHeight();  
  uint pxSize = uint(pow(2.0, scale));
  uint steps   = 1; 
  if(pxSize < MAX_NEIGHBORHOOD) steps = MAX_NEIGHBORHOOD/pxSize;
  uint nwidth  = width/steps;
  uint nheight = height/steps;


  Image<float> result(width,height,NO_INIT); 
  Image<float>::iterator resultT = result.beginw();
  
  /*Image<float>::iterator stT[itsNumDirs];
  for (unsigned int i=0; i < itsNumDirs; i++)
    stT[i] = itsSpatioTemporalEnergy[i][scale].beginw();*/

  // NOTE: (other dir average then subtract) have more artifacts
  //       but keep all other possible evidences 
  //       may help in aperture problem 
  //       DO NOT just get the max (although it is cleaner)
  for(uint j = 0; j < height; j++)
    {
      for(uint i = 0; i < width; i++)
        {
          float sum = 0.0; 
          float opp = 0.0;
          for (uint k = 0; k < itsNumDirs; k++)
            {
              // NOTE: use this line if don't want center surround
              //float tval = *stT[k]++;

              // NOTE: to find 4x4 neighborhood max values of opponencies
              //       better than just a single location 
              // get the max vals around this location
              uint mi = i/steps;
              uint mj = j/steps;
              
              uint lm = 0;         if(mi > 0)        lm = mi - 1; 
              uint rm = nwidth-1;  if(mi < nwidth-2) rm = mi + 1;
              uint tm = 0;         if(mj > 0)        tm = mj - 1; 
              uint bm = nheight-1; if(mj < nheight-2)bm = mj + 1;

//               // DEBUG:
//              if(i == 180 && j == 60)
//               if(i >=  148 && i <= 151 && j >=  60 && j <=  63   ) 
//                 {
//                   LINFO("[%3d %3d] (%3d %3d %3d %3d) -> %3d %3d", i,j, lm,rm,tm,bm, mi,mj); 
//                 }

              float nmax = 0.0;
              for(uint bi = lm; bi <= rm; bi++)
                {
                  for(uint bj = tm; bj <= bm; bj++)
                    {
                      float nval = itsDirSteMaxVals[k][scale].getVal(bi, bj);
                      if(nmax < nval) nmax = nval;
 
                      //if(i == 180 && j == 60)
                        //               if(i >=  148 && i <= 151 && j >=  60 && j <=  63   ) 
                        //{
                        //  LINFO("[%3d %3d] -> %10.3f %10.3f", bi,bj, nval, nmax); 
                        //}



                   }
                }

              float tval = nmax;


              // accumulate opponencies only for other directions
              if(k != dir) sum += tval;
              //if(k != dir && k != oppDir) sum += tval;
             
              // also find
              if(k == oppDir) opp = tval;
              //if(max < tval) max =  tval;

            }
          sum /= (itsNumDirs-2.0);

          
          // NOTE: experiment with both average and just the opposite direction
          //       -->on stationary stimuli (or stimuli that's too slow/fast) 
          //          opposite direction gives equal firings.  
          *resultT++ = opp; //(sum+opp);         
        }
    }

  return result;
}

// ######################################################################
Image<float> MiddleTemporal::getNeighborInhibitSpatioTemporalEnergy
(uint dir, uint scale)
{  
  uint width  = itsSpatioTemporalEnergy[dir][scale].getWidth();
  uint height = itsSpatioTemporalEnergy[dir][scale].getHeight();  
  Image<float> result(width,height,ZEROS); 

  float cosAng = cos((2.0*M_PI * dir)/itsNumDirs); 
  float sinAng = sin((2.0*M_PI * dir)/itsNumDirs); 
  
  Image<float>::iterator stT[itsNumDirs];
  for (unsigned int i=0; i < itsNumDirs; i++)
    stT[i] = itsSpatioTemporalEnergy[i][scale].beginw();

  // NOTE: FIX maybe add .5 to make sure the diagonal is taken care
  //LINFO("c,s(%10.3f, %10.3f) [%3d %3d]", cosAng, sinAng, width, height);

  // NOTE: inhibit neighbor motion if they're not in agreement with yours

  std::vector<float> fweights(4);
  fweights[0] = .15;
  fweights[1] = .07;
  fweights[2] = .02;
  fweights[3] = .01;

  std::vector<float> bweights(3);
  bweights[0] = .10;
  bweights[1] = .05;
  bweights[2] = .02;

  for(uint j = 0; j < height; j++)
    {
      for(uint i = 0; i < width; i++)
        {
          float val = *stT[dir]++;          
          //float val  = itsSpatioTemporalEnergy[dir][scale].getVal(i,j);
          for(uint k = 0; k < fweights.size(); k++)
            {
              int ii = (i + (k+1.0)*cosAng);
              int jj = (j + (k+1.0)*sinAng);
 
              if(ii > 0 && ii < int(width) &&
                 jj > 0 && jj < int(height)   )
                {
                  float rval = result.getVal(ii,jj);                  
                  result.setVal(ii,jj, rval + (fweights[k] * val));
                }
           }
        }

      // add the backward inhibitory as well if needed
    }
  return result;
}

// ######################################################################
void MiddleTemporal::findMaxMotionVals()
{
  // SCALE INTEGRATION
  // normalizing to 1.0 is working if we expand the local max normalizer
  // we only use 2 levels now to go from 1 pix - 8pix displacement
  // all that it needed really

  uint width  = itsSpatioTemporalEnergy[0][0].getWidth();
  uint height = itsSpatioTemporalEnergy[0][0].getHeight();

  uint mWidth  = width/MAX_NEIGHBORHOOD;
  uint mHeight = height/MAX_NEIGHBORHOOD;
  //LINFO("MTfeatures size: %d x %d", mWidth, mHeight);

  // got through each point in each direction
  for(uint d = 0; d < itsNumDirs; d++)
    {
      Image<float> tMTfeatures (mWidth, mHeight, NO_INIT);
      Image<float>::iterator tmtT = tMTfeatures.beginw();

      Image<float> tMToptimalShift (mWidth, mHeight, NO_INIT);
      Image<float>::iterator tmtosT = tMToptimalShift.beginw();

      for(uint j = 0; j < mHeight; j++)
        {
          for(uint i = 0; i < mWidth; i++)
            {
              // find max among all scales
              float max = 0.0; float optShift = 0.0;
              for(uint l = 0; l < itsNumPyrLevels; l++)
                {
                  uint pxSize  = uint(pow(2.0, l));
                  uint lwidth  = width/pxSize;
                  uint lheight = height/pxSize;

                  uint steps = 1; 
                  if(pxSize < MAX_NEIGHBORHOOD) 
                    steps = MAX_NEIGHBORHOOD/pxSize;

                  // translated coordinate
                  uint ti = i * steps;
                  uint tj = j * steps;

                  // take care of boundary conditions                  
                  uint lstep = ti; if(ti         >= steps) lstep = ti - steps;
                  uint rstep = ti; if(ti+steps <   lwidth) rstep = ti + steps;
                  uint tstep = tj; if(tj         >= steps) tstep = tj - steps;
                  uint bstep = tj; if(tj+steps <  lheight) bstep = tj + steps;

                  // this needs interpolation
                  if(pxSize > MAX_NEIGHBORHOOD)
                    {
                      // NOTE: ADD LATER
                      uint div = pxSize/MAX_NEIGHBORHOOD; 
                      //LINFO("interp: (%3d %3d) t[%3d %3d] {%3d %3d}: %3d %3d", 
                      //      i,j, ti, tj, d, l, i/div, j/div);
                      float val =
                        itsSpatioTemporalEnergy[d][l].getVal(i/div,j/div);
                      float shift =
                        itsSpatioTemporalEnergyOptimalShift[d][l].getVal(i/div,j/div);
                      if(max < val)
                        {
                          max = val;
                          optShift = shift;
                        }
                    }
                  else
                    {
                      for(uint di = lstep; di < rstep; di++)
                        {
                          for(uint dj = tstep; dj < bstep; dj++)
                            {
                              //LINFO("(%3d %3d) t[%3d %3d] {%3d %3d}[%3d %3d %3d %3d] d: %3d %3d", 
                              //      i,j, ti, tj, d, l, lstep, rstep, tstep, bstep, di, dj);
                              float val = 
                                itsSpatioTemporalEnergy[d][l].getVal(di, dj);
                              float shift = 
                                itsSpatioTemporalEnergyOptimalShift[d][l].getVal(di, dj) * pxSize;
                              if(max < val) 
                                {
                                  max = val;
                                  optShift = shift;
                                }
                            }
                        }
                    }
                }              

              //tMTfeatures.setVal(i,j, max);
              //LINFO("%f %f %f",tMTfeatures.getVal(i,j), max, *tmtT); tmtT++;
              *tmtT++ = max; 
              *tmtosT++ = optShift;
            }
        }
      itsMTfeatures[d]     = tMTfeatures;
      itsMToptimalShift[d] = tMToptimalShift;
    }
}

// ######################################################################
void MiddleTemporal::printItsSpatioTemporalEnergy
(uint si, uint ei, uint sj, uint ej, bool stop, float div)
{
  if(itsSpatioTemporalEnergy[0].size() == 0) return;
    
  //for(uint k = 0; k < 1; k++)
  for(uint k = 0; k < itsNumPyrLevels; k++)
    {
      uint step = uint(pow(2.0,k));
      for(uint i = 0; i < itsNumDirs; i++)
        {
          LINFO("itsSpatioTemporalEnergy features: dir: %d lev: %d", i, k);
          
          for(uint y = sj/step; y < ej/step; y++)
            {
              for(uint x = si/step; x < ei/step; x++)
                printf("%8.3f ", itsSpatioTemporalEnergy[i][k].getVal(x,y)/div);
              printf("\n");
            }
          LINFO("\n");
          
          //i+= 3; // < just to see dir: 0 and 4 or motion left and right
          
          if(stop) Raster::waitForKey();
        }
    }
}

// ######################################################################
void MiddleTemporal::displayItsSpatioTemporalEnergy()
{
  if(itsSpatioTemporalEnergy[0].size() == 0) return;
  uint w = itsSpatioTemporalEnergy[0][0].getWidth();
  uint h = itsSpatioTemporalEnergy[0][0].getHeight();
  Image<float> disp(w*4, h*2,NO_INIT);

  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(w*4,h*2), w+10, 0, "FOE Detector"));
  else itsWin->setDims(Dims(w*4,h*2));
  
  for(uint i = 0; i < itsNumPyrLevels; i++)
  //for(uint i = 0; i < 1; i++)
    {
      LINFO("level: %d", i);

      uint scale = uint(pow(2.0, i)); 
      inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[0][i], scale), 
                   Point2D<int>(0,   0));
      inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[1][i], scale), 
                   Point2D<int>(w,   0));
      inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[2][i], scale), 
                   Point2D<int>(2*w, 0));
      inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[3][i], scale), 
                   Point2D<int>(3*w, 0));

      if(itsNumDirs > 4)
        {
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[4][i], scale), 
                       Point2D<int>(0,   h));
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[5][i], scale), 
                       Point2D<int>(w,   h));
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[6][i], scale), 
                       Point2D<int>(2*w, h));
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[7][i], scale), 
                       Point2D<int>(3*w, h));
        }

      if(itsNumDirs > 8)
        {
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[8][i],  scale), 
                       Point2D<int>(0,     h));
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[9][i],  scale), 
                       Point2D<int>(w/2,   h));
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[10][i], scale), 
                       Point2D<int>(w,     h));
          inplacePaste(disp, zoomXY(itsSpatioTemporalEnergy[11][i], scale), 
                       Point2D<int>(3*w/2, h));
        } 
      itsWin->drawImage(disp,0,0);
      Raster::waitForKey();
    }
}

// ######################################################################
void MiddleTemporal::printItsMTfeatures
(uint si, uint ei, uint sj, uint ej, bool stop)
{
  if(itsSpatioTemporalEnergy[0].size() == 0) return;

  
  LINFO("MT features ");
  for(uint i = 0; i < itsNumDirs; i++)
    {
      for(uint y = sj; y < ej; y++)
        {
          for(uint x = si; x < ei; x++)
            printf("%7.3f ", itsMTfeatures[i].getVal(x,y));
          printf("\n");
        }
      LINFO("MT features: %d", i);
      if(stop)Raster::waitForKey();
    }
}

// ######################################################################
void MiddleTemporal::printItsMToptimalShift
(uint si, uint ei, uint sj, uint ej, bool stop)
{
  if(itsSpatioTemporalEnergy[0].size() == 0) return;

  
  LINFO("MT features Optimal Shift");
  for(uint i = 0; i < itsNumDirs; i++)
    {
      for(uint y = sj; y < ej; y++)
        {
          for(uint x = si; x < ei; x++)
            printf("%7.3f ", itsMToptimalShift[i].getVal(x,y));
          printf("\n");
        }
      LINFO("MT features: %d", i);
      if(stop)Raster::waitForKey();
    }
}

// ######################################################################
void MiddleTemporal::displayItsMToptimalShift()
{
  if(itsSpatioTemporalEnergy[0].size() == 0) return; 
  uint w = itsSpatioTemporalEnergy[0][0].getWidth();
  uint h = itsSpatioTemporalEnergy[0][0].getHeight();
  
  uint wDisp = w*2; uint hDisp = h/2;
  if(itsNumDirs == 8)      { wDisp = w*2; hDisp = h;     }
  else if(itsNumDirs == 8) { wDisp = w*2; hDisp = 3*h/2; }

  Image<float> disp(wDisp,hDisp,NO_INIT);
  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(wDisp,hDisp), w+10, 0, "FOE Detector"));
  else itsWin->setDims(Dims(wDisp, hDisp));

  LINFO("display MT optimal shift");

  inplacePaste(disp, zoomXY(itsMToptimalShift[0], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(0,     0));
  inplacePaste(disp, zoomXY(itsMToptimalShift[1], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(w/2,   0));
  inplacePaste(disp, zoomXY(itsMToptimalShift[2], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(w,     0));
  inplacePaste(disp, zoomXY(itsMToptimalShift[3], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(3*w/2, 0));

  if(itsNumDirs > 4)
    {
      inplacePaste(disp, zoomXY(itsMToptimalShift[4], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(0,    h/2));
      inplacePaste(disp, zoomXY(itsMToptimalShift[5], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w/2,  h/2));
      inplacePaste(disp, zoomXY(itsMToptimalShift[6], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w,    h/2));
      inplacePaste(disp, zoomXY(itsMToptimalShift[7], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(3*w/2,h/2));
    }
  
  if(itsNumDirs > 8)
    {
      inplacePaste(disp, zoomXY(itsMToptimalShift[8],  MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(0,     h));
      inplacePaste(disp, zoomXY(itsMToptimalShift[9],  MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w/2,   h));
      inplacePaste(disp, zoomXY(itsMToptimalShift[10], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w,     h));
      inplacePaste(disp, zoomXY(itsMToptimalShift[11], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(3*w/2, h));
    } 

  float mn,mx; getMinMax(disp,mn,mx);      
  drawLine(disp, Point2D<int>(0    , h/2), Point2D<int>(w*2  , h/2), mx, 1);
  drawLine(disp, Point2D<int>(0    , h  ), Point2D<int>(w*2  , h  ), mx, 1);
  drawLine(disp, Point2D<int>(w/2  , 0  ), Point2D<int>(w/2  , h  ), mx, 1);
  drawLine(disp, Point2D<int>(w    , 0  ), Point2D<int>(w    , h  ), mx, 1);
  drawLine(disp, Point2D<int>(3*w/2, 0  ), Point2D<int>(3*w/2, h  ), mx, 1);
  itsWin->drawImage(disp,0,0);
  Raster::waitForKey();  
}

// ######################################################################
void MiddleTemporal::displayItsMTfeatures()
{
  if(itsSpatioTemporalEnergy[0].size() == 0) return; 
  uint w = itsSpatioTemporalEnergy[0][0].getWidth();
  uint h = itsSpatioTemporalEnergy[0][0].getHeight();
  
  uint wDisp = w*2; uint hDisp = h/2;
  if(itsNumDirs == 8)      { wDisp = w*2; hDisp = h;     }
  else if(itsNumDirs == 8) { wDisp = w*2; hDisp = 3*h/2; }

  Image<float> disp(wDisp,hDisp,NO_INIT);
  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(wDisp,hDisp), w+10, 0, "FOE Detector"));
  else itsWin->setDims(Dims(wDisp, hDisp));

  LINFO("display MT features");

  inplacePaste(disp, zoomXY(itsMTfeatures[0], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(0,     0));
  inplacePaste(disp, zoomXY(itsMTfeatures[1], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(w/2,   0));
  inplacePaste(disp, zoomXY(itsMTfeatures[2], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(w,     0));
  inplacePaste(disp, zoomXY(itsMTfeatures[3], MAX_NEIGHBORHOOD/2), 
               Point2D<int>(3*w/2, 0));

  if(itsNumDirs > 4)
    {
      inplacePaste(disp, zoomXY(itsMTfeatures[4], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(0,    h/2));
      inplacePaste(disp, zoomXY(itsMTfeatures[5], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w/2,  h/2));
      inplacePaste(disp, zoomXY(itsMTfeatures[6], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w,    h/2));
      inplacePaste(disp, zoomXY(itsMTfeatures[7], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(3*w/2,h/2));
    }
  
  if(itsNumDirs > 8)
    {
      inplacePaste(disp, zoomXY(itsMTfeatures[8],  MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(0,     h));
      inplacePaste(disp, zoomXY(itsMTfeatures[9],  MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w/2,   h));
      inplacePaste(disp, zoomXY(itsMTfeatures[10], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(w,     h));
      inplacePaste(disp, zoomXY(itsMTfeatures[11], MAX_NEIGHBORHOOD/2), 
                   Point2D<int>(3*w/2, h));
    } 

  float mn,mx; getMinMax(disp,mn,mx);      
  drawLine(disp, Point2D<int>(0    , h/2), Point2D<int>(w*2  , h/2), mx, 1);
  drawLine(disp, Point2D<int>(0    , h  ), Point2D<int>(w*2  , h  ), mx, 1);
  drawLine(disp, Point2D<int>(w/2  , 0  ), Point2D<int>(w/2  , h  ), mx, 1);
  drawLine(disp, Point2D<int>(w    , 0  ), Point2D<int>(w    , h  ), mx, 1);
  drawLine(disp, Point2D<int>(3*w/2, 0  ), Point2D<int>(3*w/2, h  ), mx, 1);
  itsWin->drawImage(disp,0,0);
  Raster::waitForKey();  
}

// ######################################################################
Layout<byte> MiddleTemporal::getMTfeaturesDisplay(Image<byte> image)
{
  if(itsRawSpatioTemporalEnergy.size() == 0 || 
     itsRawSpatioTemporalEnergy[0][0].size() == 0)
    return Layout<byte>(image);

  uint width  = image.getWidth();
  uint height = image.getHeight();   

  uint scale = 1;
  Image<float> lum    = image;
  Image<float> motion(width* scale, height * scale, ZEROS);

  uint mtWidth  = itsMTfeatures[0].getWidth();
  uint mtHeight = itsMTfeatures[0].getHeight();

  Image<float>::iterator mtT[itsNumDirs];
  for (unsigned int i=0; i < itsNumDirs; i++)
    mtT[i] = itsMTfeatures[i].beginw();

  Image<float>::iterator mtosT[itsNumDirs];
  for (unsigned int i=0; i < itsNumDirs; i++)
    mtosT[i] = itsMToptimalShift[i].beginw();

  float boom = 0;
  // go through each MT locations
  for(uint dir = 0; dir < itsNumDirs; dir++)
    {
      float angle = (dir*2.0*M_PI)/itsNumDirs; 
      if(dir%2 == 1) 
        { 
          //LINFO("[%d> ang: %f", dir, angle);
          angle = (dir*2.0*M_PI)/itsNumDirs + M_PI/2.0;
          //LINFO("<%d] ang: %f", dir, angle);
        }
      // draw an arrow for each location
      for(uint j = 0; j < mtHeight; j++)
        {
          for(uint i = 0; i < mtWidth; i++)
            {
              float val   = *mtT[dir]++;
              float shift = *mtosT[dir]++;
              if(val > .20)// && dir == 3 && boom != 10 && shift == 8.0)//(dir != 0 && dir != 4))
                {
                  boom++;
                  //LINFO("val %f, ang: %f (%d %d)", val, angle, i,j);
                  drawLine(motion, 
                           Point2D<int>(i*MAX_NEIGHBORHOOD*scale,
                                        j*MAX_NEIGHBORHOOD*scale), 
                           //angle, val*MAX_NEIGHBORHOOD*scale, 1.0F);
                           angle, shift*4*scale, val);
  
                  //drawDisk(motion, 
                  //          Point2D<int>(i*MAX_NEIGHBORHOOD*scale,
                  //                       j*MAX_NEIGHBORHOOD*scale), 2, val);
               }
            }
        }
    }

  Image<float> res = motion;
  inplaceNormalize(res,    0.0F, 255.0F);
  inplaceNormalize(lum,    0.0F,  64.0F);
  inplaceNormalize(motion, 0.0F, 255.0F);

  // draw the location of the FOE
  //drawCross(res, itsFoe, 1.0f, 10, 1);
  
  Image<byte> vimg = res;
  //writeText(vimg, Point2D<int>(1,1), "img", byte(0), byte(255),
  //          SimpleFont::fixedMaxWidth(8));
  
  Image<byte> himg = res;
  //writeText(himg, Point2D<int>(1,1), "motion", byte(0), byte(255),
  //          SimpleFont::fixedMaxWidth(8));
  
//   Layout<byte> disp = 
//     vcat(hcat(lum, Image<byte>(quad)), hcat(himg, vimg));

  Layout<byte> disp(res+lum);

  return disp;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
