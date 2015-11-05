/*!@file Channels/DirectionOpticalFlowChannel.C */
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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DirectionOpticalFlowChannel.C $
// $Id:$
//

#ifndef DIRECTION_OPTICALFLOW_CHANNEL_C_DEFINED
#define DIRECTION_OPTICALFLOW_CHANNEL_C_DEFINED

#include "Channels/DirectionOpticalFlowChannel.H"

#include "rutz/trace.h"
#include "Image/Image.H"
#include "Raster/Raster.H"

#define OF_WINDOW_SIZE   4
#define OF_VALUE_RADIUS  1

// ######################################################################
// DirectionOpticalFlowChannel member definitions:
// ######################################################################

// ######################################################################
DirectionOpticalFlowChannel::DirectionOpticalFlowChannel
(OptionManager& mgr, 
 const uint dirIndex, const double direction, 
 const OpticalFlowType type):
  SingleChannel(mgr, "", "", MOTIONSPATIOTEMPORAL,
                rutz::make_shared(new GaussianPyrBuilder<float>(1))), // no actual pyrbuilder
  itsDirIndex("DirectionChannelDirectionIndex", this, dirIndex),
  itsDirection("DirectionChannelDirection", this, direction),
  itsOpticalFlowType(type)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  setDescriptiveName(sformat("DirectionSpeed(%d)", int(direction)));
  setTagName(sformat("dir_%d", dirIndex));
}

// ######################################################################
void DirectionOpticalFlowChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 itsLevelSpec.setVal(LevelSpec(0,0,0,0,4) );  // only 1 level
 SingleChannel::start1();
}

// ######################################################################
void DirectionOpticalFlowChannel::start2()
{
GVX_TRACE(__PRETTY_FUNCTION__);

}

// ######################################################################
DirectionOpticalFlowChannel::~DirectionOpticalFlowChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
bool DirectionOpticalFlowChannel::outputAvailable() const
{ 
  return itsDirectionalOpticalFlow.initialized();
}

// ######################################################################
  //! different ways to input different optical flow algorithms
void DirectionOpticalFlowChannel::setLucasKanadeOpticalFlow
(rutz::shared_ptr<OpticalFlow> flow)
{
  ASSERT(itsOpticalFlowType == LucasKanade);
  itsOpticalFlow = flow;
}

// ######################################################################
void DirectionOpticalFlowChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!this->started())
    CLFATAL("must be start()-ed before using receiving any input");

  // so that the visual cortex think we can produce a conspicuity map
  // FIXXX: really stupid hack
  SingleChannel::doInput(inframe); // will store the new pyramid

  // check if the optical flow input is already provided
  switch(itsOpticalFlowType)
    {
    case LucasKanade:
      if(itsOpticalFlow.is_invalid()) 
        LFATAL("need to provide optical flow. "
               "Use setLucasKanadeOpticalFlow(flow)");
      computeDirectionalOpticalFlow();
      break;
    case HornSchunck:
      break;
    default: LERROR("Unknown optical flow mode");
    }
}

// ######################################################################
void DirectionOpticalFlowChannel::computeDirectionalOpticalFlow()
{
  Dims d = itsOpticalFlow->getImageDims();
  itsDirectionalOpticalFlow = 
    Image<float>(d.w()/ OF_WINDOW_SIZE, d.h()/ OF_WINDOW_SIZE, ZEROS);

  // go through each flowVector using std::vector
  // FIXXX: for now. We know that Lucas Kanade produces sparse vectors. 
 
  std::vector<rutz::shared_ptr<FlowVector> > flow =
    itsOpticalFlow->getFlowVectors();

  for(uint i = 0; i < flow.size(); i++)
    {
      Point2D<int> pt1((int)flow[i]->p1.i, (int)flow[i]->p1.j); 
      Point2D<int> pt2((int)flow[i]->p2.i, (int)flow[i]->p2.j); 

      float ang  = flow[i]->angle;
      //float mag  = flow[i]->mag;
      float fval = flow[i]->val;
 
      // get the response value of the
      float rv = getResponseValue(ang);            
      computeDirectionalOpticalFlow(pt1.i, pt1.j, rv*fval);

      //LINFO("[%3d %3d]: ang: %f mag: %f fval: %f ---> %f", 
      //      pt1.i, pt1.j, ang, mag, fval, rv*fval);
    }
}

// ######################################################################
float DirectionOpticalFlowChannel::getResponseValue(float angle)
{
  float pangle = itsDirection.getVal();
  
  //  map to to 0 - 360 deg
  float nangle = fmod(angle +360.0, 360.0);

  // find difference in direction
  float diff1 = fabs(nangle - pangle);
  float diff2 = 360.0 - diff1;
  float diff = diff1; if(diff1 > diff2) diff = diff2;

  // we are going to focus to between +/- 30 deg diff
  float val = 0.0;
  float stdang = 15.0;

  // difference between 0 and 15 degrees
  if(diff >= 0 && diff < stdang)
    val = (stdang - diff)/stdang *    0.2 + 0.8;

   // difference between 15 and 30 degrees
   else if(diff >= stdang && diff < 2*stdang)
     val = (2*stdang - diff)/stdang *  0.8 + 0.0;

  // difference farther than 30 degrees
  else
    val = 0.0;

   return val;
}

// ######################################################################
void DirectionOpticalFlowChannel::computeDirectionalOpticalFlow
(uint i, uint j, float val)
{
  // scale down to the actual directional flow map
  uint ii = i/OF_WINDOW_SIZE;
  uint jj = j/OF_WINDOW_SIZE;


  //itsDirectionalOpticalFlow.setVal(ii, jj, val);

  uint ri = OF_VALUE_RADIUS;
  uint rj = OF_VALUE_RADIUS;

  uint width  = itsDirectionalOpticalFlow.getWidth();
  uint height = itsDirectionalOpticalFlow.getHeight();
  
  uint lm =  0;        if(ii > ri-1)        lm = ii - ri; 
  uint rm =  width-1;  if(ii < width-1-ri)  rm = ii + ri;
  uint tm =  0;        if(jj > rj-1)        tm = jj - rj; 
  uint bm =  height-1; if(jj < height-1-ri) bm = jj + rj;

  // set the values
  float max = double (ri*2);
  for(uint bi = lm; bi <= rm; bi++)
    {
      for(uint bj = tm; bj <= bm; bj++)
        {
          // FIX: pre-compute the weights later 
          float di = float(bi) - float(ii);
          float dj = float(bj) - float(jj);
          float w = 1.0 - (sqrt(di*di + dj*dj))/max;
          float ov = itsDirectionalOpticalFlow.getVal(bi, bj);
          itsDirectionalOpticalFlow.setVal(bi, bj, ov+w*val);
          //LINFO("%d %d: %f", bi, bj, w*val);
        }
    }
}

// ######################################################################
Image<float> DirectionOpticalFlowChannel::getDirectionalOpticalFlow()
{
  return itsDirectionalOpticalFlow;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DIRECTION_OPTICALFLOW_CHANNEL_C_DEFINED
