/*!@file Channels/DisparityChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DisparityChannel.C $
// $Id: DisparityChannel.C 12074 2009-11-24 07:51:51Z itti $
//

#ifndef DISPARITYCHANNEL_C_DEFINED
#define DISPARITYCHANNEL_C_DEFINED

#include "Channels/DisparityChannel.H"

#include "Image/CutPaste.H"
#include "Image/MathOps.H"
#include "Image/PyramidOps.H"
#include "Util/sformat.H"

// ######################################################################
// DisparityChannel member definitions:
// ######################################################################

// ######################################################################
DisparityChannel::DisparityChannel(OptionManager& mgr, const uint dIndex,
                                   const double ang, const double pha) :
  SingleChannel(mgr, "", "", DISPARITY, rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsIndex("DisparityChannelIndex", this, dIndex),
  itsOrientation("DisparityChannelOrientation", this, ang),
  itsPhase("DisparityChannelPhase", this, pha)
{
  //  itsNormalizeOutput.setVal(true);

  setDescriptiveName(sformat("Disparity(%d,%d)", int(ang), int(pha)));
  setTagName(sformat("d_%d", dIndex));
}

// ######################################################################
DisparityChannel::~DisparityChannel()
{ }

// ######################################################################
void DisparityChannel::start1()
{
  SingleChannel::start1();

  // initilialize the size of the disparity map
  depth = itsLevelSpec.getVal().maxDepth();
  dispMap.reset(depth);

}

// ######################################################################
double DisparityChannel::angle() const
{
  return itsOrientation.getVal();
}

// ######################################################################
double DisparityChannel::phase() const
{
  return itsPhase.getVal();
}

// ######################################################################
void DisparityChannel::setRawFilteredImages(Image<float> ***fImgLE,
                                            Image<float> ***fImgRI)
{
  fImgL = fImgLE;
  fImgR = fImgRI;
}

// ######################################################################
void DisparityChannel::getRawFilteredImages(Image<float> ****fImgLE,
                                            Image<float> ****fImgRI)
{
  *fImgLE = fImgL;
  *fImgRI = fImgR;
}

// ######################################################################
void DisparityChannel::getDispMap(ImageSet<float> *dMap)
{
  *dMap = dispMap;
}

// ######################################################################
void DisparityChannel::normalizeDispMap(ImageSet<float> tDispMap, int nChan)
{
  float min,max;
  for(uint d = 0; d < depth; d++)
    {
      dispMap[d] /= tDispMap[d];
      getMinMax(dispMap[d], min,max);
      LINFO("MIN:%f, MAX:%f\n\n\n",min,max);

      // FIXME: get the numer of channels from the
      dispMap[d] -= (float)(1.0F/(float)(nChan));
      getMinMax(dispMap[d], min,max);
      LINFO("-- MIN:%f, MAX:%f\n\n\n",min,max);
      inplaceRectify(dispMap[d]);
      getMinMax(dispMap[d], min,max);
      LINFO("RR MIN:%f, MAX:%f\n\n\n",min,max);
    }
}

// ######################################################################
void DisparityChannel::setNumPhase(int n)
{
  nPhase = n;
}

// ######################################################################
void DisparityChannel::setNumTheta(int t)
{
  nTheta = t;
}

// ######################################################################
void DisparityChannel::doInput(const InputFrame& inframe)
{
  // for now only evaluate the horizontal disparity
  // and make sure the phase is normalized to 0 to 360
  //int t  = 0; //(int)(round((itsOrientation.getVal()/180.0)*nTheta));
  //int dp = (int)(round((itsPhase.getVal()/360.0)*nPhase));
  float dt = 1.2;//itsPhase.getVal();
  if(itsPhase.getVal() > 180.0) dt = -1.2;//dt -= 360.0;
  if(itsPhase.getVal() == 0.0) dt = 0.0;
  float  dx = dt;
  //float dx = period * dt/360.0;
  //printf("dx = %f \n",dx);

  int width  = fImgL[0][0][0].getWidth();
  int height = fImgL[0][0][0].getHeight();

  Image<float> sumL(width,height,ZEROS);
  Image<float> sumR(width,height,ZEROS);

  for(uint d = 0; d < 1; d++)
  {
    dispMap[d].resize(width,height,true);
    /*
    // add the phase shift portion
    for(int p = 0; p < nPhase; p++)
      dispMap[d] +=
       fImgL[d][0][p] * fImgR[d][0][(p+dp)%nPhase];
    */

    //int t = 0;
    // add the position shift portion
    // the equation is already derived
    for(uint t = 0; t < nTheta; t++)
      for(uint p = 0; p < nPhase; p++)
      {
        Image<float> tmp =
          fImgL[d][t][p] * shiftImage(fImgR[d][t][p],dx,0);
        inplaceRectify(tmp);
        dispMap[d] += squared(tmp);
     }

    if(width  > 1) width  /= 2;
    if(height > 1) height /= 2;

  }

  // build the feature map pyramid and store it
  dispMap = buildPyrGaussian((dispMap[0]),0,depth,5);
  SingleChannel::storePyramid(dispMap,inframe.time());
}

// store the disparity map as feature map
void DisparityChannel::storePyramid(const SimTime& t)
{
  SingleChannel::storePyramid(dispMap, t);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DISPARITYCHANNEL_C_DEFINED
