/*!@file SpaceVariant/FovealTransformModule.C space variant transformation module */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/FovealTransformModule.C $

#include "SpaceVariant/FovealTransformModule.H"
#include "SpaceVariant/SpaceVariantTransforms.H"
#include "SpaceVariant/SpaceVariantOpts.H"
#include "Channels/ChannelOpts.H"
#include "Neuro/NeuroOpts.H"
#include "Image/CutPaste.H"

// ######################################################################
// ######################################################################
// ########## FovealTransformModule implementation
// ######################################################################
// ######################################################################
FovealTransformModule::FovealTransformModule(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  SpaceVariantModule(mgr, descrName, tagName),
  itsSvScale(&OPT_SpaceVariantScale, this),
  itsSvFovea(&OPT_SpaceVariantFoveaSize, this),
  itsSvSfac(&OPT_SpaceVariantSfac, this),
  itsSvBeta(&OPT_SpaceVariantBeta, this),
  itsSvGain(&OPT_SpaceVariantGain, this),
  itsSvExponent(&OPT_SpaceVariantExponent, this),
  itsSvOffset(&OPT_SpaceVariantOffset, this),
  itsFoveaCutoff(&OPT_SpaceVariantFovCut, this), 
  itsSvDims(&OPT_SpaceVariantDims, this),
  itsPPD(&OPT_PixelsPerDegree, this),
  itsLevelSpec(&OPT_LevelSpec, this)
{ }

// ######################################################################
FovealTransformModule::~FovealTransformModule()
{ }

// ######################################################################
void FovealTransformModule::clear(const Dims& inp_dims)
{
  //one to transform retinal images
  FovealTransform* t = new FovealTransform();
  t->setup(inp_dims.w(), inp_dims.h(),
           itsSvDims.getVal().w(), itsSvDims.getVal().h(),
           itsSvFovea.getVal(), itsSvBeta.getVal(), 
           itsSvGain.getVal(), itsSvExponent.getVal(), itsSvOffset.getVal(),
           itsPPD.getVal().ppdx(), itsPPD.getVal().ppdy(), 
           FovealTransform::toScaleType(itsSvScale.getVal()), 
           itsSvSfac.getVal(), itsFoveaCutoff.getVal());
  itsTransform.reset(t);
  
  //this one gets used downstream to invert map level images
  const int w = inp_dims.w() >> itsLevelSpec.getVal().mapLevel();
  const int h = inp_dims.h() >> itsLevelSpec.getVal().mapLevel();
  const int svw = itsSvDims.getVal().w()*2 >> itsLevelSpec.getVal().mapLevel();
  const int svh = itsSvDims.getVal().h()/2 >> itsLevelSpec.getVal().mapLevel();
  
  FovealTransform* t1 = new FovealTransform();
  t1->setup(w, h, svw/2, svh*2,
            itsSvFovea.getVal(), itsSvBeta.getVal(), 
            itsSvGain.getVal(), itsSvExponent.getVal(), itsSvOffset.getVal(),
            itsPPD.getVal().ppdx(), itsPPD.getVal().ppdy(),
            FovealTransform::toScaleType(itsSvScale.getVal()), itsSvSfac.getVal(), 0.0);
  itsMapTransform.reset(t1);
  
  if ((w % 2 != 0) || (h % 2 != 0))
    LDEBUG("After pyramid operations, the input image is ideally divisible by two for "
           "space variant image processing. Try increasing the input image size or "
           "altering the pyramid depth or output level.");
  
  if (svh % 2 != 0)
    LDEBUG("After pyramid operations, the transformed is ideally divisible by two for "
           "space variant image processing. Try increasing the space variant height (angles) or "
           "altering the pyramid depth or output level.");
  
  LINFO("FovealTransform transform configured.");
}

// ######################################################################
template <class T_or_RGB>
void FovealTransformModule::getFoveaPeriphery(const Image<T_or_RGB>& ret_image, 
                                              Image<T_or_RGB>& fovea, Image<T_or_RGB>& periphery)
{
  ::getFoveaPeriphery(static_cast<FovealTransform&>(*itsTransform), ret_image, fovea, periphery);
}

// ######################################################################
Dims FovealTransformModule::getSVDims() const
{
  return itsSvDims.getVal();
}

#define INST_CLASS FovealTransformModule::
#include "inst/SpaceVariant/FovealTransformModule.I"
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
