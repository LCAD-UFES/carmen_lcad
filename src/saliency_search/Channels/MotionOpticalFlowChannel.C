/*!@file Channels/MotionOpticalFlowChannel.C */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MotionOpticalFlowChannel.C $
// $Id: $
//

#ifndef MOTION_OPTICALFLOW_CHANNEL_C_DEFINED
#define MOTION_OPTICALFLOW_CHANNEL_C_DEFINED

#include "Channels/MotionOpticalFlowChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"

#include "Image/ShapeOps.H"

#include "rutz/trace.h"
#include "Util/Timer.H"

#define OF_SELF_MOTION_WEIGHT     1.0F
#define OF_OBJECT_MOTION_WEIGHT   1.0F
#define OF_MAX_FIRING_RATE        100.0F

// Used by: MotionOpticalFlowChannel
const ModelOptionDef OPT_OpticalFlowType =
  { MODOPT_ARG(OpticalFlowType), "OpticalFlowType", &MOC_CHANNEL, OPTEXP_CORE,
    "Type of computation used to compute the optical flow",
    "optical-flow-type", '\0', "<LucasKanade|HornSchunck>",
    "LucasKanade" };

// ######################################################################
// MotionOpticalFlowChannel member definitions:
// ######################################################################

// ######################################################################
MotionOpticalFlowChannel::MotionOpticalFlowChannel(OptionManager& mgr) :
  ComplexChannel(mgr, 
                 "MotionOpticalFlow", 
                 "motionOpticalFlow", 
                 MOTIONOPTICALFLOW),
  itsOpticalFlowType(&OPT_OpticalFlowType, this), // see Channels/ChannelOpts.{H,C}
  itsNumDirs(&OPT_NumOpticalFlowDirections, this), // see Channels/ChannelOpts.{H,C}
  itsFoeDetector(new FoeDetector(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // let's create our subchannels (may be reconfigured later if our
  // number of directions changes):
  buildSubChans();

  itsWin.reset();
}

// ######################################################################
MotionOpticalFlowChannel::~MotionOpticalFlowChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
DirectionOpticalFlowChannel& MotionOpticalFlowChannel::dirChan
(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *(dynCast<DirectionOpticalFlowChannel>(subChan(idx)));
}

// ######################################################################
void MotionOpticalFlowChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our subchannels now that we know how many we
  // want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d directions spanning [0..360]deg", itsNumDirs.getVal());

  itsDirectionOpticalFlowChannels.clear();

  // go through the different directions
  for (uint i = 0; i < itsNumDirs.getVal(); i++)
    {
      nub::ref<DirectionOpticalFlowChannel> chan =
        makeSharedComp(new DirectionOpticalFlowChannel
                       (getManager(), i,
                        360.0 * double(i)/double(itsNumDirs.getVal()),
                        itsOpticalFlowType.getVal()));
      
      itsDirectionOpticalFlowChannels.push_back(chan);
      
      this->addSubChan(chan);      
      chan->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void MotionOpticalFlowChannel::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumDirs &&
      numChans() != itsNumDirs.getVal())
    buildSubChans();
}

// ######################################################################
void MotionOpticalFlowChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.grayFloat().initialized());

  Image<byte> image(inframe.grayFloat());
  if(!itsCurrentImage.initialized())
    {
      itsCurrentImage = image; return;
    }
  else
    itsPreviousImage = itsCurrentImage;
  itsCurrentImage = image;

  // compute optical flow
  itsOpticalFlow =
    getLucasKanadeOpticFlow(itsPreviousImage, itsCurrentImage);  

  // compute directional optical flow
  // into several directions
  for (uint i = 0; i < numChans(); i++)
    {
      itsDirectionOpticalFlowChannels[i]
        ->setLucasKanadeOpticalFlow(itsOpticalFlow);
      subChan(i)->input(inframe);

      LINFO("Motion Optical flow pyramid (%d/%d) ok.", i+1, numChans());
    }

  // compute motion conspicuity map
  computeConspicuityMap();
}

// ######################################################################
void MotionOpticalFlowChannel::computeConspicuityMap()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  uint cmWidth  = subChan(0)->getMapDims().w();
  uint cmHeight = subChan(0)->getMapDims().h();
  Image<float> result(cmWidth, cmHeight, ZEROS);

  Dims imDims = itsOpticalFlow->getImageDims();
  uint mtWidth  = imDims.w()/4; 
  uint mtHeight = imDims.h()/4;

  for (uint i = 0; i < itsNumDirs.getVal(); i++)
    {
      Image<float> tmap = itsDirectionOpticalFlowChannels[i]
        ->getDirectionalOpticalFlow();

      Image<float> submap = downSizeClean(tmap, Dims(cmWidth, cmHeight));

      Image<float> psubmap;
      if (itsUseOlderVersion.getVal())
        {
          LDEBUG("%s[%d]: applying %s(%f .. %f)", 
                 tagName().c_str(), i, 
                 maxNormTypeName(itsNormType.getVal()), MAXNORMMIN, MAXNORMMAX);
          psubmap = maxNormalize(submap, MAXNORMMIN, MAXNORMMAX,
                                 itsNormType.getVal());
        }
      else
        {
          LDEBUG("%s[%d]: applying %s(0.0 .. 0.0)", tagName().c_str(), i, 
                 maxNormTypeName(itsNormType.getVal()));
          psubmap = maxNormalize(submap, 0.0f, 0.0f, itsNormType.getVal());
        }

      result += psubmap;
     }
  Image<float> tsubmap = maxNormalize(result, MAXNORMMIN, MAXNORMMAX,
                                      itsNormType.getVal());
  result = tsubmap * numChans();
  Image<float> tres = result;
  result = rescale(tres, Dims(mtWidth, mtHeight));

  // May add a map that comes from higher level Motion areas
  //  : MST: FOE, planar motion  
  //  : STS: Biological motion  

  // NOTE:  FOE_METHOD_TEMPLATE and AVERAGE 
  //        is still fooled by Planar movement!!! 
  Image<float> foeMap = 
    itsFoeDetector->getFoeMap
    (itsOpticalFlow, FOE_METHOD_AVERAGE, false);  //FOE_METHOD_TEMPLATE


  // LINFO("ORIGINAL MAP");
  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(mtWidth*4, mtHeight*4), 
  //                                10, 0, "MotSpch: conspicuity map"));
  // else itsWin->setDims(Dims(mtWidth*4, mtHeight*4));
  // itsWin->drawImage(zoomXY(foeMap,4),0,0); Raster::waitForKey();

  // float orgMax = mx;
  // float firingRate = 0.0;
  // if(orgMax > .5) 
  //   firingRate = MAX_FIRING_RATE;  
  // else if(orgMax >= .1 && orgMax <= .5)
  //   firingRate = (orgMax - .1)/.4 * MAX_FIRING_RATE;  

  // crazy normalizer
  float mn, mx; getMinMax(foeMap, mn,mx);
  //LINFO("org   MSTd : %f %f",mn,mx);
  inplaceNormalize(foeMap, 0.0F, 1.0F);
  foeMap = toPower(foeMap, 40.0F);
  foeMap *= mx;

  // weight the firing rate to the maximum possible firing rate
  foeMap *= (OF_SELF_MOTION_WEIGHT *  OF_MAX_FIRING_RATE * numChans());

  // getMinMax(result,mn,mx);
  // LINFO("FINAL MSTv : %f %f",mn,mx);

  // getMinMax(foeMap,mn,mx);
  // LINFO("FINAL MSTd : %f %f",mn,mx);

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(mtWidth*4, mtHeight*4), 
  //                                10, 0, "MotSpch: conspicuity map"));
  // else itsWin->setDims(Dims(mtWidth*4, mtHeight*4));

  // itsWin->drawImage(zoomXY(result,4),0,0); Raster::waitForKey();
  // itsWin->drawImage(zoomXY(foeMap,4),0,0); Raster::waitForKey();

  result += foeMap;

  // itsWin->drawImage(zoomXY(result,4),0,0); Raster::waitForKey();

  // resize submap to fixed scale if necessary:
  getMinMax(result,mn,mx);
  if (mtWidth > cmWidth)
    result = downSize(result, Dims(cmWidth, cmHeight));
  else if (mtWidth < cmWidth)
    result = rescale(result, Dims(cmWidth, cmHeight));
  inplaceNormalize(result,0.0F,mx);
  LINFO("Final cmap mn: %f mx: %f", mn, mx);

  itsConspicuityMap = result;
}

// ######################################################################
Image<float> MotionOpticalFlowChannel::getOutput()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 return itsConspicuityMap;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MOTION_OPTICALFLOW_CHANNEL_C_DEFINED
