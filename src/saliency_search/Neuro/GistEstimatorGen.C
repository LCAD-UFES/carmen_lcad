/*!@file Neuro/GistEstimatorGen.C extract estimated gist
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
// Primary maintainer for this file: Zhicheng Li <zhicheng@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorGen.C $
// $Id: GistEstimatorGen.C 15244 2012-03-30 18:20:48Z kai $
//

// ######################################################################
/*! Extract gist of image                                              */


#include "Neuro/GistEstimatorGen.H"
#include "Component/ModelManager.H"
#include "Channels/ChannelMaps.H"
#include "Channels/BlueYellowChannel.H"
#include "Channels/ColorChannel.H"
#include "Channels/GaborChannel.H"
#include "Channels/IntensityChannel.H"
#include "Channels/OrientationChannel.H"
#include "Channels/RedGreenChannel.H"
#include "Channels/Hue1Channel.H"
#include "Channels/Hue2Channel.H"
#include "Channels/HueChannel.H"
#include "Channels/H2SVChannel.H"
#include "Channels/CIELabChannel.H"
#include "Channels/JunctionChannel.H"
#include "Channels/EndPointChannel.H"
#include "Channels/TJunctionChannel.H"
#include "Channels/LJunctionChannel.H"
#include "Channels/XJunctionChannel.H"
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
GistEstimatorGen::GistEstimatorGen(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  GistEstimatorAdapter(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventVisualCortexOutput),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsUseCS(&OPT_GistCenterSurroundFlag,this)
{
  itsGistSize = 0;
}

// ######################################################################
GistEstimatorGen::~GistEstimatorGen()
{ }

// ######################################################################
void GistEstimatorGen::
onSimEventVisualCortexOutput(SimEventQueue& q, rutz::shared_ptr<SimEventVisualCortexOutput>& e)
{
  //Grab the channel maps from the visual cortex
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  //Compute the full size gist feature vector
  getFeatureVector(chm);

  // post an event so that anyone interested in gist can grab it:
  rutz::shared_ptr<SimEventGistOutput>
    ew(new SimEventGistOutput(this, itsGistVector));
  q.post(ew);
}

// ######################################################################
Image<double> GistEstimatorGen::getGist()
{
  return itsGistVector;
}

// ######################################################################
void GistEstimatorGen::start1()
{
  getManager().setOptionValString(&OPT_SingleChannelComputeFullPyramidForGist,"true");
  GistEstimatorAdapter::start1();
}

// ######################################################################
void GistEstimatorGen::getFeatureVector(rutz::shared_ptr<ChannelMaps> chanMaps)
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
        inplacePaste(itsGistVector,getSubSumGen(chanMaps->getRawCSmap(i)),
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
                inplacePaste(itsGistVector,getSubSumGen
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
                    inplacePaste(itsGistVector,getSubSumGen
                                 (currSubChan->getPyramid().getImage(j)),
                                 Point2D<int>(0,count*NUM_GIST_FEAT));
                    count++;
                  }
              }
          }
      }
  ASSERT(count == sz);
  itsGistSize = sz;
}

// ######################################################################
// get gist histogram to visualize the data
Image<float> GistEstimatorGen::getGistImage(int sqSize,
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
          //float val = log(itsGistVector.getVal(i*NUM_GIST_FEAT+j)+1);
          //val = val * val;
          drawPatch(img, Point2D<int>(i*s+s/2,j*s+s/2),s/2,(val-minO)/range);
          //LINFO("val[%d]: %f",j,val);
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
        //float val = log(itsGistVector.getVal(i*NUM_GIST_FEAT+j)+1);
        //val = val * val;
        drawPatch(img, Point2D<int>(i*s+s/2,j*s+s/2),s/2,(val-minC)/range);
        //LINFO("val[%d]: %f",j,val);
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
        //float val = log(itsGistVector.getVal(i*NUM_GIST_FEAT+j)+1);
        //val = val * val;
        drawPatch(img, Point2D<int>(i*s+s/2,j*s+s/2),s/2,(val-minI)/range);
        //LINFO("val[%d]: %f",j,val);
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
Image<float> GistEstimatorGen::diffGist(Image<float> in)
{
  LFATAL("fix");
  float total = 0.0;
  float a,b,c,d;

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

  return Image<float>();
}


// ######################################################################
// get the gist size at the begining to allocate the size for itsGistVector
int GistEstimatorGen::getGistSize()
{
  return itsGistSize;
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
