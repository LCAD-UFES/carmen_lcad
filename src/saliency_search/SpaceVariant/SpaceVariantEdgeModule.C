/*!@file SpaceVariant/SpaceVariantEdgeModule.C space variant transformation module */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SpaceVariantEdgeModule.C $

#include "SpaceVariant/SpaceVariantEdgeModule.H"
#include "SpaceVariant/SpaceVariantTransforms.H"
#include "SpaceVariant/SpaceVariantOpts.H"
#include "SpaceVariant/SVChanLevels.H"
#include "Image/ImageSet.H"

// ######################################################################
// implementation for SpaceVariantEdgeModule
// ######################################################################
SpaceVariantEdgeModule::SpaceVariantEdgeModule(OptionManager& mgr, 
                                               const std::string& descrName, const std::string& tagName) :
  FovealTransformModule(mgr, descrName, tagName),
  itsSvDoGSize(&OPT_SpaceVariantDogSize, this),
  itsSvEdgeOrient(&OPT_SpaceVariantEdgeOrient, this),
  itsSvEdgeOLength(&OPT_SpaceVariantEdgeLength, this),
  itsSvEdgeODensity(&OPT_SpaceVariantEdgeDensity, this),
  itsEdges(), useDog(true)
{ }

// ######################################################################
SpaceVariantEdgeModule::~SpaceVariantEdgeModule()
{ }

// ######################################################################
void SpaceVariantEdgeModule::start1()
{ 
  useDog =  (itsSvDoGSize.getVal() < 1.0F) ? false : true;
}

// ######################################################################
void SpaceVariantEdgeModule::clear(const Dims& dims)
{ 
  FovealTransformModule::clear(dims);

  //setup the orientations
  itsEdges =  LocalEdgeMap(*itsTransform, 
                           itsSvDoGSize.getVal(), 
                           itsSvEdgeOrient.getVal(), 
                           itsSvEdgeOLength.getVal(),
                           itsSvEdgeODensity.getVal());
}

// ######################################################################
Image<PixRGB<float> > SpaceVariantEdgeModule::transformRGB(const Image<PixRGB<byte> >& image, 
                                                           const ImageSet<PixRGB<float> >* const pyr_cache)
{
  return this->transformEdge(image, pyr_cache);
}

// ######################################################################
void SpaceVariantEdgeModule::transformRgbPyramid(const Image<PixRGB<byte> >& image, ImageSet<PixRGB<float> >& pyramid, const SVChanLevels& levels, const ImageSet<PixRGB<float> >* const pyr_cache)
{
  return this->transformEdgePyramid(image, pyramid, levels, pyr_cache);
}

// ######################################################################
Image<float> SpaceVariantEdgeModule::transformFloat(const Image<float>& image, const ImageSet<float>* const pyr_cache)
{
  return this->transformEdge(image, pyr_cache);
}

// ######################################################################
void SpaceVariantEdgeModule::transformFloatPyramid(const Image<float>& image, ImageSet<float>& pyramid, const SVChanLevels& levels, const ImageSet<float>* const pyr_cache)
{
  return this->transformEdgePyramid(image, pyramid, levels, pyr_cache);
}
// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
SpaceVariantEdgeModule::transformEdge(const Image<T_or_RGB>& ret_image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
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
      output = (!useDog) ? 
        transformToEdge(*itsTransform, ret_image, itsEdges, pyr_cache)
      : transformToEdge(*itsTransform, ret_image, itsSvDoGSize.getVal(), itsEdges, pyr_cache);
    }
  return output;
}

// ######################################################################
template <class T_or_RGB>
void SpaceVariantEdgeModule::transformEdgePyramid(const Image<T_or_RGB>& ret_image, ImageSet<typename promote_trait<T_or_RGB, float>::TP>& pyramid, const SVChanLevels& levels, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
{
  if (ret_image.initialized())
    {
      //re-initialzed if our input dims change
      if (!validTransforms())
        clear(ret_image.getDims());
      else if (ret_image.getDims() != itsTransform->getCTDims())
        clear(ret_image.getDims());
      
      pyramid = ImageSet<typename promote_trait<T_or_RGB, float>::TP>(levels.numLevels());
      for (uint ii = 0; ii < levels.numLevels(); ++ii)
        pyramid[ii] = (!useDog) ? 
          transformToEdge(*itsTransform, ret_image, itsEdges, pyr_cache,levels.getVariance(ii))
          : transformToEdge(*itsTransform, ret_image, itsSvDoGSize.getVal(),itsEdges,pyr_cache,levels.getVariance(ii));
    }
}

#define INST_CLASS SpaceVariantEdgeModule::
#include "inst/SpaceVariant/SpaceVariantEdgeModule.I"
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
