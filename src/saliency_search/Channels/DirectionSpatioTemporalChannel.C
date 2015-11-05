/*!@file Channels/DirectionSpatioTemporalChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DirectionSpatioTemporalChannel.C $
// $Id:$
//

#ifndef DIRECTIONSPATIOTEMPORALCHANNEL_C_DEFINED
#define DIRECTIONSPATIOTEMPORALCHANNEL_C_DEFINED

#include "Channels/DirectionSpatioTemporalChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ImageSetOps.H"
#include "Image/MathOps.H"
#include "Util/sformat.H"
#include "rutz/compat_cmath.h" // for M_PI
#include "rutz/trace.h"
#include "Raster/Raster.H"
#include "Image/ShapeOps.H"


// ######################################################################
// DirectionSpatioTemporalChannel member definitions:
// ######################################################################

// ######################################################################
DirectionSpatioTemporalChannel::DirectionSpatioTemporalChannel
(OptionManager& mgr, 
 const uint dirIndex, 
 const uint speedIndex,
 const double direction, 
 const double speed,
 const PyramidType type):
  SingleChannel(mgr, "", "", MOTIONSPATIOTEMPORAL,
                rutz::make_shared
                (new SpatioTemporalEnergyPyrBuilder<float>
                 (Oriented5, direction, speed))), //itsNumPyrLevels
  itsDirIndex("DirectionChannelDirectionIndex", this, dirIndex),
  itsSpeedIndex("DirectionChannelSpeedIndex", this, speedIndex),
  itsDirection("DirectionChannelDirection", this, direction),
  itsSpeed("DirectionChannelSpeed", this, direction)
{
  // FIXXX: how to switch to new SpatioTemporalEnergyPyrBuilder<byte>
  // FIXXX: what do we do with the level specs

GVX_TRACE(__PRETTY_FUNCTION__);
  
  // this is because we only have 2 levels of center surround maps
  //mgr.setOptionValString(&OPT_LevelSpec, "0,1,0,0,4");
  //mgr.setOptionValString(&OPT_LevelSpec, "0,2,0,0,4"); // for depth of 3
  
  itsSpatioTemporalPyrBuilder.reset
    (new SpatioTemporalEnergyPyrBuilder<float>
     (Oriented5, direction, speed));
  
  setDescriptiveName(sformat("DirectionSpeed(%d,%f)", int(direction), speed));
  setTagName(sformat("dir_%d_sp_%d", dirIndex, speedIndex));
}

// ######################################################################
void DirectionSpatioTemporalChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 itsLevelSpec.setVal(LevelSpec(0,1,0,0,4) );  //.levMin();
 SingleChannel::start1();
}

// ######################################################################
void DirectionSpatioTemporalChannel::start2()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
DirectionSpatioTemporalChannel::~DirectionSpatioTemporalChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
ImageSet<float> DirectionSpatioTemporalChannel::
computePyramid(const Image<float>& bwimg,
               const rutz::shared_ptr<PyramidCache<float> >& cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // get the pyramid as usual:
  itsSpatioTemporalEnergy = 
    itsSpatioTemporalPyrBuilder->build(bwimg);
  ImageSet<float> py = itsSpatioTemporalEnergy;
  
  return py;
}

// ######################################################################
void DirectionSpatioTemporalChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!this->started())
    CLFATAL("must be start()-ed before using receiving any input");

  ASSERT(inframe.grayFloat().initialized());  

  // V1: compute spatiotemporal motion detection
  Image<byte> image(inframe.grayFloat());

  setClipPyramid(inframe.clipMask());
  
  itsSpatioTemporalEnergy = 
    itsSpatioTemporalPyrBuilder->build(image);

  if(itsSpatioTemporalEnergy.size() != 0)
    storePyramid(itsSpatioTemporalEnergy, inframe.time());
}

// ######################################################################
void DirectionSpatioTemporalChannel::setMTfeatureMap(Image<float> mtFeat)
{
  itsMTfeatureMap = mtFeat;
}

// ######################################################################
Image<float> DirectionSpatioTemporalChannel::getRawCSmap(const uint idx) const
{
  //Image<float> res = decXY(itsMTfeatureMap);
  Image<float> res = downSize(itsMTfeatureMap, getMapDims());
  Image<float> tres = maxNormalize(res, MAXNORMMIN, MAXNORMMAX,
                                      itsNormType.getVal());


  
  //return tres;
  //return res;
  return itsMTfeatureMap;
}

// ######################################################################
ImageSet<float> DirectionSpatioTemporalChannel::getSpatioTemporalEnergy()
{
  return itsSpatioTemporalPyrBuilder->getSpatioTemporalEnergy();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DIRECTIONSPATIOTEMPORALCHANNEL_C_DEFINED
