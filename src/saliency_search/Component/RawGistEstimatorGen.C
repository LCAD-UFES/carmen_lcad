/*!@file Component/RawGistEstimatorGen.C extract estimated gist
         using available features of the image                          */
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/RawGistEstimatorGen.C $
// $Id: RawGistEstimatorGen.C $
//

// ######################################################################
/*! Extract gist of image                                               */

#include "Component/RawGistEstimatorGen.H"
#include "Component/ModelManager.H"
#include "Channels/ChannelMaps.H"
//#include "Channels/BlueYellowChannel.H"
//#include "Channels/ColorChannel.H"
//#include "Channels/GaborChannel.H"
//#include "Channels/IntensityChannel.H"
//#include "Channels/OrientationChannel.H"
//#include "Channels/RedGreenChannel.H"
#include "GUI/XWinManaged.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Neuro/gistParams.H"
#include "Raster/Raster.H"

#include "Simulation/SimEventQueue.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEvents.H"
#include "Channels/ChannelOpts.H"
#include "Channels/SingleChannel.H"

#include "Util/Timer.H"
#include "Util/StringUtil.H"
#include "Neuro/NeuroOpts.H"
#define PYR_LEVEL 5 // if without center surround then use 5 levels pyramid



// ######################################################################
RawGistEstimatorGen::RawGistEstimatorGen(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
	itsLevelSpec(&OPT_LevelSpec, this),
  itsUseCS(&OPT_GistCenterSurroundFlag,this)
{
  itsGistVector.resize(1,NUM_GIST_FEAT * NUM_GIST_CHAN, NO_INIT);
}

// ######################################################################
RawGistEstimatorGen::~RawGistEstimatorGen()
{ }

// ######################################################################
Image<float> RawGistEstimatorGen::compute(rutz::shared_ptr<ChannelMaps> chmp)
{
  computeFeatureVector(chmp);
  return itsGistVector;
}

// ######################################################################
std::vector<float> RawGistEstimatorGen::computeOnPolygon(rutz::shared_ptr<ChannelMaps> chanMaps, std::vector<Point2D<float> > polygonFS)
{
  //create a gist vector
  std::vector<float> gistPolyVector;
    
  //rescale the polygon to be at the same level than the subMaps
  std::vector<Point2D<int> > polygon(polygonFS.size());
  Point2D<int> Ppoly;
  for(uint k=0; k<polygonFS.size(); k++)
    {
      Ppoly.i = (int) floor(polygonFS[k].i/16);
      Ppoly.j = (int) floor(polygonFS[k].j/16);
      polygon[k] = Ppoly;
    }
  //test wheter the polygon contain at least one pixel
  double areaPoly = area(polygon);
  if (areaPoly<=0)
    {
      LINFO("The given polygon is too small for the gist computation!");
      //find a bounding box with at leat one pixel
      int minx, miny, maxx, maxy;
      minx = miny =100000000;
      maxx = maxy =-100000000;
      for (uint k=0; k<polygonFS.size(); k++)
        {
          if(minx > (int) polygonFS[k].i) minx = (int) polygonFS[k].i;       
          if(miny > (int) polygonFS[k].j) miny = (int) polygonFS[k].j;
          if(maxx < (int) polygonFS[k].i) maxx = (int) polygonFS[k].i;
          if(maxy < (int) polygonFS[k].j) maxy = (int) polygonFS[k].j;
        }
      int minxR, minyR, maxxR, maxyR;
      minxR = floor(minx/16.0);
      minyR = floor(miny/16.0);
      maxxR = ceil(maxx/16.0);
      maxyR = ceil(maxy/16.0);
     
      polygon.clear();
      polygon.resize(4);
      polygon[0] = Point2D<int> (minxR, minyR);
      polygon[1] = Point2D<int> (maxxR, minyR);
      polygon[2] = Point2D<int> (maxxR, maxyR);
      polygon[3] = Point2D<int> (minxR, maxyR);
    }

  rutz::shared_ptr<ChannelMaps> orientationMaps;
  rutz::shared_ptr<ChannelMaps> colorMaps;
  rutz::shared_ptr<ChannelMaps> intensityMaps;
  
  for(uint i=0; i < chanMaps->numSubchans(); i++)
    {
      //Grab the current channel
      rutz::shared_ptr<ChannelMaps> currChan = chanMaps->subChanMaps(i);
      
      //Determine the name of the channel
      std::vector<std::string> chanNameVec;
      split(currChan->getMap().name(), ":", std::back_inserter(chanNameVec));
      std::string chanName = chanNameVec[1];
      
      //Assign the channel maps based on their name
      if(chanName == "orientation")
        orientationMaps = currChan;
      else if(chanName == "color")
        colorMaps = currChan;
      else if(chanName == "intensity")
        intensityMaps = currChan;
    }
  
  LDEBUG("Orientation Subchans: %d", orientationMaps->numSubchans());
  LDEBUG("Color Subchans: %d", colorMaps->numSubchans());
  LDEBUG("Intensity Subchans: %d", intensityMaps->numSubchans());
  
  //For now, we are expecting the default number of features in the channel maps, as per Siagian_Itti07pami.
  //TODO:This should be generalized later when things are more stable
  ASSERT(orientationMaps->numSubchans() == 4);
  ASSERT(colorMaps->numSubchans() == 2);
  ASSERT(intensityMaps->numSubchans() == 0);

  //Grab the individual orientation channels
  rutz::shared_ptr<ChannelMaps> orientation_0   = orientationMaps->subChanMaps(0);
  rutz::shared_ptr<ChannelMaps> orientation_45  = orientationMaps->subChanMaps(1);
  rutz::shared_ptr<ChannelMaps> orientation_90  = orientationMaps->subChanMaps(2);
  rutz::shared_ptr<ChannelMaps> orientation_135 = orientationMaps->subChanMaps(3);

  //Grab the different color channels
  rutz::shared_ptr<ChannelMaps> rg;
  rutz::shared_ptr<ChannelMaps> by;
  for(uint i=0; i<colorMaps->numSubchans(); i++)
    {
      std::vector<std::string> chanNameVec;
      split(colorMaps->subChanMaps(i)->getMap().name(), ":", std::back_inserter(chanNameVec));
      std::string chanName = chanNameVec[2];
      
      if(chanName == "rg")
        rg = colorMaps->subChanMaps(i);
      else if(chanName == "by")
        by = colorMaps->subChanMaps(i);
    }
  
  ASSERT(rg.is_valid());
  ASSERT(by.is_valid());

  //For the Gist computed on the Polygon, no more subsum (=sum on grid) are done, the polygon is comnsidered as one cell, the mean is tacken on the whole polygon
  
  // orientation channel 0 degree
  int count = 0;
  for(int i = 0; i < NUM_GIST_LEV; i++)
    {
      float meanPoly = getMeanPoly(orientation_0->getSubmap(i), polygon);   
      gistPolyVector.push_back(meanPoly);
      count += 1;
    }
  // orientation channel 45 degree
  for(int i = 0; i < NUM_GIST_LEV; i++)
    {
      float meanPoly = getMeanPoly(orientation_45->getSubmap(i), polygon);  
      gistPolyVector.push_back(meanPoly);
      count += 1;;
    }
  // orientation channel 90 degree
  for(int i = 0; i < NUM_GIST_LEV; i++)
    {
      float meanPoly = getMeanPoly(orientation_90->getSubmap(i), polygon);   
      gistPolyVector.push_back(meanPoly);
      count += 1;
    }
  // orientation channel 135 degree
  for(int i = 0; i < NUM_GIST_LEV; i++)
    {
      float meanPoly = getMeanPoly(orientation_135->getSubmap(i), polygon); 
      gistPolyVector.push_back(meanPoly);
      count += 1;
    }
  
  //For the Red/Green and Blue/Yellow Channels,
  //Christian's model uses levels:  (2,5), (2,6), (3,6), (3,7), (4,7), (4,8)
  //However, this method will use:  (2,5), (3,6), (4,7), (2,6), (3,7), (4,8)
  //Same channels, just a different order... does this matter?
  
  // red / green opponency
  for(uint i=0; i<rg->numSubmaps(); i++)
    {
      float meanPoly = getMeanPoly(rg->getSubmap(i), polygon);   
      gistPolyVector.push_back(meanPoly);
      count += 1;    
    }
  // blue / yellow opponency
  for(uint i=0; i<by->numSubmaps(); i++)
    {
      float meanPoly = getMeanPoly(by->getSubmap(i), polygon);  
      gistPolyVector.push_back(meanPoly);
      count += 1;   
    }
  //intensity
  for(uint i=0; i<intensityMaps->numSubmaps(); i++)
    {
      float meanPoly = getMeanPoly(intensityMaps->getSubmap(i), polygon);  
      gistPolyVector.push_back(meanPoly);
      count += 1;
    }
  
  return gistPolyVector;
}

// ######################################################################
void RawGistEstimatorGen::computeFeatureVector(rutz::shared_ptr<ChannelMaps> chanMaps)
{
  //! first get the gist feature size and allocate the gist vector size
  int sz = 0, sz_cs=0, sz_nocs = 0;
  if(itsUseCS.getVal() == 1 || itsUseCS.getVal() == 2)
    sz_cs += chanMaps->numSubmaps();

  // sz_nocs is the number of how many raw pyramid types
  if(itsUseCS.getVal() == 0 || itsUseCS.getVal() == 2)
    for(uint i=0; i < chanMaps->numSubchans(); i++)
      {
        rutz::shared_ptr<ChannelMaps> currChan = chanMaps->subChanMaps(i);
        if(currChan->numSubchans() == 0)
          sz_nocs++;
        else
          sz_nocs += currChan->numSubchans();
      }
  sz_nocs *= PYR_LEVEL;

  sz = sz_cs + sz_nocs;
  LINFO("there are in total %4d gist feature chans", sz);
  itsGistVector.resize(1,NUM_GIST_FEAT * sz, NO_INIT);

  int count = 0;

  //! get the center-surround feature values
  if(itsUseCS.getVal() == 1 || itsUseCS.getVal() == 2)
    for(int i = 0; i<sz_cs; i++)
      {
        inplacePaste(itsGistVector,getSubSum(chanMaps->getRawCSmap(i)),
                     Point2D<int>(0, count*NUM_GIST_FEAT));
        count++;
      }

  //! get the non center-surround feature values
  if(itsUseCS.getVal() == 0 || itsUseCS.getVal() == 2)
    for(uint i=0; i<chanMaps->numSubchans(); i++)
      {
        rutz::shared_ptr<ChannelMaps> currChan = chanMaps->subChanMaps(i);
        if(currChan->numSubchans() == 0)
          {
            ASSERT(currChan->hasPyramid());
            for(uint j=0; j<PYR_LEVEL; j++)
              {
                inplacePaste(itsGistVector,getSubSum
                             (currChan->getPyramid().getImage(j)),
                             Point2D<int>(0,count*NUM_GIST_FEAT));
                count++;
              }
          }
        else
          {
            for(uint i=0; i<currChan->numSubchans(); i++)
              {
                rutz::shared_ptr<ChannelMaps> currSubChan = currChan->subChanMaps(i);
                ASSERT(currSubChan->hasPyramid());
                for(uint j=0; j<PYR_LEVEL; j++)
                  {
                    inplacePaste(itsGistVector,getSubSum
                                 (currSubChan->getPyramid().getImage(j)),
                                 Point2D<int>(0,count*NUM_GIST_FEAT));
                    count++;
                  }
              }
          }
      }
  ASSERT(count == sz);
  //itsGistSize = sz;

}

// ######################################################################
// get gist histogram to visualize the data
Image<float> RawGistEstimatorGen::getGistImage(int sqSize,
    float minO, float maxO,
    float minC, float maxC,
    float minI, float maxI)
{
  // square size
  int s = sqSize;
  Image<float> img(NUM_GIST_COL * s, NUM_GIST_FEAT * s, ZEROS);
  float range;
  
  // setup range for orientation channel if necessary
  if(maxO == minO)
    {
      minO = itsGistVector.getVal(0);
      maxO = itsGistVector.getVal(0);
      for(int i = 0; i < 16; i++)
        for(int j = 0; j < NUM_GIST_FEAT; j++)
          {
            float val = itsGistVector.getVal(i*NUM_GIST_FEAT+j);
            if(val < minO)
              minO = val;
            else if(val > maxO)
              maxO = val;
          }
      LDEBUG("Orientation Channel Min: %f, max: %f", minO, maxO);
    }
  range = maxO - minO;
  
  // orientation channel
  for(int a = 0; a < 4; a++)
    for(int b = 0; b < 4; b++)
      for(int j = 0; j < NUM_GIST_FEAT; j++)
        {
          int i  = b*4 + a;
          int ii = a*4 + b;
          float val = itsGistVector.getVal(ii*NUM_GIST_FEAT+j);
          drawPatch(img, Point2D<int>(i*s+s/2,j*s+s/2),s/2,(val-minO)/range);
        }
  
  // setup range for color channel if necessary
  if(maxC == minC)
    {
      minC = itsGistVector.getVal(16*NUM_GIST_FEAT);
      maxC = itsGistVector.getVal(16*NUM_GIST_FEAT);
      for(int i = 16; i < 28; i++)
        for(int j = 0; j < NUM_GIST_FEAT; j++)
          {
            float val = itsGistVector.getVal(i*NUM_GIST_FEAT+j);
            if(val < minC)
              minC = val;
            else if(val > maxC)
              maxC = val;
          }
      LDEBUG("Color Channel Min: %f, max: %f", minC, maxC);
    }
  range = maxC - minC;
  
  // color channel
  for(int i = 16; i < 28; i++)
    for(int j = 0; j < NUM_GIST_FEAT; j++)
      {
        float val = itsGistVector.getVal(i*NUM_GIST_FEAT+j);
        drawPatch(img, Point2D<int>(i*s+s/2,j*s+s/2),s/2,(val-minC)/range);
      }
  
  // setup range for intensity channel if necessary
  if(maxI == minI)
    {
      minI = itsGistVector.getVal(28*NUM_GIST_FEAT);
      maxI = itsGistVector.getVal(28*NUM_GIST_FEAT);
      for(int i = 28; i < 34; i++)
        for(int j = 0; j < NUM_GIST_FEAT; j++)
          {
            float val = itsGistVector.getVal(i*NUM_GIST_FEAT+j);
            if(val < minI)
              minI = val;
            else if(val > maxI)
              maxI = val;
          }
      LDEBUG("Intensity Channel Min: %f, max: %f", minI, maxI);
    }
  range = maxI - minI;
  
  // intensity channel
  for(int i = 28; i < NUM_GIST_COL; i++)
    for(int j = 0; j < NUM_GIST_FEAT; j++)
      {
        float val = itsGistVector.getVal(i*NUM_GIST_FEAT+j);
        drawPatch(img, Point2D<int>(i*s+s/2,j*s+s/2),s/2,(val-minI)/range);
      }
  
  // draw the delineation
  // spatially
  float max = 1.0f;
  drawLine(img, Point2D<int>(0,  1*s), Point2D<int>(NUM_GIST_COL*s,  1*s), max,   1);
  drawLine(img, Point2D<int>(0,  5*s), Point2D<int>(NUM_GIST_COL*s,  5*s), max,   1);
  drawLine(img, Point2D<int>(0,  9*s), Point2D<int>(NUM_GIST_COL*s,  9*s), max/2, 1);
  drawLine(img, Point2D<int>(0, 13*s), Point2D<int>(NUM_GIST_COL*s, 13*s), max/2, 1);
  drawLine(img, Point2D<int>(0, 17*s), Point2D<int>(NUM_GIST_COL*s, 17*s), max/2, 1);
  
  // channelwise
  drawLine(img, Point2D<int>( 4*s, 0), Point2D<int>( 4*s, NUM_GIST_FEAT*s), max, 1);
  drawLine(img, Point2D<int>( 8*s, 0), Point2D<int>( 8*s, NUM_GIST_FEAT*s), max, 1);
  drawLine(img, Point2D<int>(12*s, 0), Point2D<int>(12*s, NUM_GIST_FEAT*s), max, 1);
  drawLine(img, Point2D<int>(16*s, 0), Point2D<int>(16*s, NUM_GIST_FEAT*s), max, 1);
  drawLine(img, Point2D<int>(22*s, 0), Point2D<int>(22*s, NUM_GIST_FEAT*s), max, 1);
  drawLine(img, Point2D<int>(28*s, 0), Point2D<int>(28*s, NUM_GIST_FEAT*s), max, 1);

  return img;
}

// ######################################################################
// get gist difference: Jeffrey Divergence
Image<double> RawGistEstimatorGen::diffGist(Image<double> in)
{
  LFATAL("fix");
  double total = 0.0;
  double a,b,c,d;
  
  for(int i = 0; i < NUM_GIST_COL; i++)
    {
      for(int j = 0; j < NUM_GIST_FEAT; j++)
        {
          a = itsGistVector.getVal(i*NUM_GIST_FEAT +j) /
            itsGistVector.getVal(i*NUM_GIST_FEAT);
          b = in.getVal(i*NUM_GIST_FEAT +j) / in.getVal(i*NUM_GIST_FEAT);
          c = itsGistVector.getVal(i*NUM_GIST_FEAT) +
            in.getVal(i*NUM_GIST_FEAT);
          d = (a - b) * c/2;
          LINFO("%6.3f - %6.3f = %f",a,b,fabs(d));
          
          if((j-5)%4 == 3) LINFO("-:-:-");
          total += sqrt((d*d));
        }
      LINFO("  ");
    }
  LINFO("Diff = %f -> %f\n",total,total/NUM_GIST_COL/NUM_GIST_FEAT);
  Raster::waitForKey();
  
  return Image<double>();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
