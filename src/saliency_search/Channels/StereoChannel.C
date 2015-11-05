/*!@file Channels/StereoChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/StereoChannel.C $
// $Id: StereoChannel.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef STEREOCHANNEL_C_DEFINED
#define STEREOCHANNEL_C_DEFINED

#include "Channels/StereoChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/DisparityChannel.H"
#include "Component/OptionManager.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "Image/Kernels.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"

// ######################################################################
// StereoChannel member definitions:
// ######################################################################

// instantiations for our static members:
const uint  StereoChannel::nPhase;
const uint  StereoChannel::nTheta;
const float StereoChannel::dPhase;
const float StereoChannel::dTheta;
const float StereoChannel::stddev;
const float StereoChannel::period;


// ######################################################################
StereoChannel::StereoChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "Stereo", "stereo", STEREO),
  itsPyrType("StereoChannelPyramidType", this, Gaussian3),
  itsNumTheta(&OPT_NumTheta, this),
  itsNumPhase("NumPhase", this, nPhase/2 + 1)
{
  fImgL = NULL; fImgR = NULL;

  // let's create our subchannels (may be reconfigured later if our
  // number of orientations changes):
  buildSubChans();
}

// ######################################################################
StereoChannel::~StereoChannel()
{  }

// ######################################################################
void StereoChannel::start1()
{

  // get the depth of the feature maps
  itsDepth = dispChan(0,0).getLevelSpec().maxDepth();

  // create the phased gabor pyramids
  createPhasedGaborPyramids();
}

// ######################################################################
DisparityChannel& StereoChannel::dispChan(const uint idO,
                                          const uint idP) const
{
  uint idx= idO * itsNumPhase.getVal() + idP;
  return *(dynCast<DisparityChannel>(this->subChan(idx)));
}

// ######################################################################
void StereoChannel::buildSubChans()
{
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our subchannels now that we know how many we
  // want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d orientations spanning [0..180]deg and %d phase spanning [0..360]deg",
    itsNumTheta.getVal(), itsNumPhase.getVal());

  double theta;
  double phase;

  for (uint i = 0; i < itsNumTheta.getVal(); i ++)
    for (uint j = 0; j < itsNumPhase.getVal(); j ++)
    {
      theta = 180.0 * double(i) / double(itsNumTheta.getVal());
      // the phase range is -PI/2 to PI/2 but -PI/2 to 0 is normalized.
      // the number of thetas is always odd
      // first half is 0 to PI/2
      if(j <= itsNumPhase.getVal()/2)
        phase = 90.0 * double(j)   / double(itsNumPhase.getVal()/2);
      // the second half is -PI/2 to 0
      else
        phase = 90.0 * double(j-1) / double(itsNumPhase.getVal()/2) +
                180.0;

      addSubChan(makeSharedComp
                 (new DisparityChannel(getManager(),
                                       i*itsNumPhase.getVal()
                                       +j,theta,phase )));

      dispChan(i,j).setNumTheta(nTheta);
      dispChan(i,j).setNumPhase(nPhase);

    }

  // Here, we just use our subchannels' default tag names

  // if we have been requested to export some options, let's ask it
  // again on our newly built channels:
  ModelComponent::exportOptions(MC_RECURSE);
}

// ######################################################################
void StereoChannel::createPhasedGaborPyramids()
{
  // create the phased gabor pyramids
  if(fImgL != NULL || fImgR != NULL)
  {
    for(uint i = 0; i < itsDepth; i++)
    {
      for(uint j = 0; j < nTheta; j++)
      {
        delete(fImgL[i][j]);
        delete(fImgR[i][j]);
      }
    }

    for(uint i = 0; i < itsDepth; i++)
    {
      delete(fImgL[i]);
      delete(fImgR[i]);
    }

    delete(fImgL);
    delete(fImgR);
  }

  fImgL = new Image<float>**[itsDepth];
  fImgR = new Image<float>**[itsDepth];

  for(uint i = 0; i < itsDepth; i++)
  {
    fImgL[i] = new Image<float>*[nTheta];
    fImgR[i] = new Image<float>*[nTheta];
  }

  for(uint i = 0; i < itsDepth; i++)
  {
    for(uint j = 0; j < nTheta; j++)
    {
      fImgL[i][j] = new Image<float>[nPhase];
      fImgR[i][j] = new Image<float>[nPhase];
    }
  }
}

// ######################################################################
void StereoChannel::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our  number of channel and it has
  // become different from our number of orientations * number of phase ,
  // let's reconfigure:
  if (param == &itsNumTheta &&
      //      param == &itsNumPhase &&
      numChans() != itsNumPhase.getVal() * itsNumTheta.getVal())
    buildSubChans();
}

// ######################################################################
void StereoChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());

  imgL = inframe.grayFloat();
  // IMPORTANT
  // at this point the secondary view image should already
  // be set by Stereo Vision.

  // create an image pyramid for both images
  pyrL = buildPyrGaussian(imgL, 0, itsDepth, 5);
  pyrR = buildPyrGaussian(imgR, 0, itsDepth, 5);

  // apply the gabor filter at various angle and phase on both eyes
  applyGabor();

  // for normalization
  ImageSet<float> tDispMap(itsDepth);
  ImageSet<float> tempDM;

  for(uint d = 0; d < itsDepth; d++)
  {
    tDispMap[d].resize(pyrL[d].getWidth(),pyrL[d].getHeight(),true);
    //ispMap[d] += 1.0F;
  }

  // pass the response array to each subchannel
  // so they can calculate each disparity correspondence
  for (uint i = 0; i < itsNumTheta.getVal(); i ++)
    for (uint j = 0; j < itsNumPhase.getVal(); j ++)
    {
      dispChan(i,j).setRawFilteredImages(fImgL,fImgR);
      dispChan(i,j).doInput(inframe);

      //dispChan(i,j).getDispMap(&tempDM);

      // add the results for normalizing the output
      //for (uint d = 0; d< itsDepth; d++)
      //  tDispMap[d] += tempDM[d];

    }

  // normalize the values to [0,  1] - confidence level
  // and store the pyramid for the interaction
  /*  for (uint i = 0; i < itsNumTheta.getVal(); i ++)
    for (uint j = 0; j < itsNumPhase.getVal(); j ++)
    {
      dispChan(i,j).normalizeDispMap(tDispMap,
        itsNumTheta.getVal()*itsNumPhase.getVal());
      dispChan(i,j).storePyramid(t);
    }
  */


}

// ######################################################################
void StereoChannel::setSecondImage(const Image<float>* bwimg)
{
  imgR = *bwimg;
}

// ######################################################################
void StereoChannel::getRawFilteredImages(Image<float> ****fImgLE,
                                         Image<float> ****fImgRI)
{
  *fImgLE = fImgL;
  *fImgRI = fImgR;
}

// ######################################################################
void StereoChannel::applyGabor()
{
  Image<float> gaborF;Image<float> gF;
  Point2D<int> a(3,3); Dims b(5,5);
  for(uint d = 0; d < itsDepth; d++)
  {
    for(uint t = 0; t < nTheta; t++)
    {
      for(uint p = 0; p < nPhase; p++)
      {
        gaborF = gaborFilter<float>(stddev,period,p*dPhase,t*dTheta);
        ASSERT(gaborF.getWidth() == 11 && gaborF.getHeight() == 11);
        gaborF = crop(gaborF,a,b,false);

        fImgL[d][t][p] = convolve(pyrL[d],gaborF,CONV_BOUNDARY_ZERO);
        //fImgL[d][t][p].rectify();
        fImgR[d][t][p] = convolve(pyrR[d],gaborF,CONV_BOUNDARY_ZERO);
        //fImgR[d][t][p].rectify();
      }
    }
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // STEREOCHANNEL_C_DEFINED
