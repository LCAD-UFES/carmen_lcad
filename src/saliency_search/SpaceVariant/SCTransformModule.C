/*!@file SpaceVariant/SCTransformModule.C space variant transformation module */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SCTransformModule.C $

#include "SpaceVariant/SCTransformModule.H"
#include "SpaceVariant/SpaceVariantTransforms.H"
#include "SpaceVariant/SpaceVariantOpts.H"
#include "Image/ImageSet.H"
#include "Neuro/NeuroOpts.H"

// ######################################################################
// ######################################################################
// ########## SCTransformModule implementation
// ######################################################################
// ######################################################################
SCTransformModule::SCTransformModule(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
    SpaceVariantModule(mgr, descrName, tagName),
    itsSvDims(&OPT_SpaceVariantDims, this),
    itsPixPerDeg(&OPT_PixelsPerDegree, this),
    itsMaxDeg(&OPT_SpaceVariantMaxVisualAngle, this), 
    itsSvDoGCenter("SpaceVariantDogCenter", this, true),
    itsSvDoGSize("SpaceVariantDogSize", this, 6.7), 
    itsS1("SpaceVariantS1", this, 0.02974),
    itsS2("SpaceVariantS2", this, 1.3923),
    itsBeta("SpaceVariantBeta", this, 0.81685),
    itsAlpha("SpaceVariantAlpha", this, 0.32226),
    itsRfSlope("SpaceVariantRfSlope", this, 0.1062),
    itsRfExp("SpaceVariantRfExp", this, 1.0),
    itsRfOffset("SpaceVariantRfOffset", this, 0.75),
    itsPPD()
{ }

// ######################################################################
SCTransformModule::~SCTransformModule()
{ }

// ######################################################################
void SCTransformModule::start1()
{ 
  itsPPD = itsPixPerDeg.getVal();
}

// ######################################################################
Point2D<int> SCTransformModule::degToSvCoords(Point2D<float> const & point) const
{
  if (itsTransform.is_valid())
  {
    Point2D<int> p = (Point2D<int>)SCTransform::deg2Pix(point.i, point.j, itsPPD.ppdx(), itsPPD.ppdy(), itsTransform->getCTDims().w(), itsTransform->getCTDims().h());
    toSvCoords(p);
    return p;
  }

  return Point2D<int>(-1,-1);
}

// ######################################################################
void SCTransformModule::setPPD(PixPerDeg const & ppd)
{
  itsPPD = ppd;
}

// ######################################################################
PixPerDeg const & SCTransformModule::getPPD() const
{
  return itsPPD;
}

// ######################################################################
void SCTransformModule::clear(const Dims& inp_dims)
{
  //one to transform retinal images
  SCTransform* t = new SCTransform();
  t->setup(inp_dims.w(), inp_dims.h(),
           itsSvDims.getVal().w(), itsSvDims.getVal().h(),
           itsPPD.ppdx(), itsPPD.ppdy(), 
           itsMaxDeg.getVal(), itsS1.getVal(), itsS2.getVal(), itsBeta.getVal(), itsAlpha.getVal(), 
           itsRfSlope.getVal(), itsRfExp.getVal(), itsRfOffset.getVal(), itsSvDoGSize.getVal());
  
  SCTransform* t1 = new SCTransform();
  t->setup(inp_dims.w(), inp_dims.h(),
           itsSvDims.getVal().w(), itsSvDims.getVal().h(),
           itsPPD.ppdx(), itsPPD.ppdy(), 
           itsMaxDeg.getVal(), itsS1.getVal(), itsS2.getVal(), itsBeta.getVal(), itsAlpha.getVal(),
           itsRfSlope.getVal(), itsRfExp.getVal(), itsRfOffset.getVal(), itsSvDoGSize.getVal());

  itsTransform.reset(t);
  itsMapTransform.reset(t1);//bogus, but we won't use with this module

  LINFO("SC Transform transform configured.");
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
SCTransformModule::transformDoG(const Image<T_or_RGB>& ret_image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
{ 
  Image<T_or_RGB> output; 
  if (ret_image.initialized())
    {
      //re-initialzed if our input dims change
      if (!validTransforms())
        clear(ret_image.getDims());
      else if (ret_image.getDims() != itsTransform->getCTDims())
        clear(ret_image.getDims());
      
      //transform image
      output = transformToDoG(*itsTransform, ret_image, itsSvDoGSize.getVal(), itsSvDoGCenter.getVal(), pyr_cache);
    }
  return output;
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
SCTransformModule::transformDivG(const Image<T_or_RGB>& ret_image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
{ 
  Image<T_or_RGB> output; 
  if (ret_image.initialized())
    {
      //re-initialzed if our input dims change
      if (!validTransforms())
        clear(ret_image.getDims());
      else if (ret_image.getDims() != itsTransform->getCTDims())
        clear(ret_image.getDims());
      
      //transform image
      output = transformToDivG(*itsTransform, ret_image, itsSvDoGSize.getVal(), pyr_cache);
    }
  return output;
}

// ######################################################################
template <class T_or_RGB>
void SCTransformModule::transformDoGPyramid(const Image<T_or_RGB>& ret_image, ImageSet<typename promote_trait<T_or_RGB, float>::TP>& pyramid, const SVChanLevels& levels, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
{
  if (ret_image.initialized())
  {
    //re-initialzed if our input dims change
    if (!validTransforms())
      clear(ret_image.getDims());
    else if (ret_image.getDims() != itsTransform->getCTDims())
      clear(ret_image.getDims());
    
    pyramid = transformToDoGPyramid(*itsTransform, ret_image, itsSvDoGSize.getVal(), itsSvDoGCenter.getVal(), levels, pyr_cache);
  }
}

#define INST_CLASS SCTransformModule::
#include "inst/SpaceVariant/SCTransformModule.I"
/// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
