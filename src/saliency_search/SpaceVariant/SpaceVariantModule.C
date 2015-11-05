/*!@file SpaceVariant/SpaceVariantModule.C space variant transformation module */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SpaceVariantModule.C $

#include "SpaceVariant/SpaceVariantModule.H"
#include "SpaceVariant/SVChanLevels.H"
#include "Image/CutPaste.H"
#include "Image/ImageSet.H"

// ######################################################################
// ######################################################################
// ########## SpaceVariantModule implementation
// ######################################################################
// ######################################################################
SpaceVariantModule::SpaceVariantModule(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsTransform(),
  itsMapTransform() 
{ }

// ######################################################################
SpaceVariantModule::~SpaceVariantModule()
{ }

// ######################################################################
void SpaceVariantModule::clear(const Dims& input_dims)
{ }

// ######################################################################
void SpaceVariantModule::clear()
{
  itsTransform.reset();
  itsMapTransform.reset();
}

// ######################################################################
Dims SpaceVariantModule::getSVImageDims() const
{
  return itsTransform->getSVDims();
}

// ######################################################################
void SpaceVariantModule::reset1()
{
  clear();
}

// ######################################################################
Image<PixRGB<float> > SpaceVariantModule::transformRbg(const Image<PixRGB<byte> >& image, 
                                                       const ImageSet<PixRGB<float> >* const pyr_cache)
{
  return this->transform(image, pyr_cache);
}

// ######################################################################
void SpaceVariantModule::transformRgbPyramid(const Image<PixRGB<byte> >& image, ImageSet<PixRGB<float> >& pyramid, const SVChanLevels& levels, const ImageSet<PixRGB<float> >* const pyr_cache)
{
  return this->transformPyramid(image, pyramid, levels, pyr_cache);
}

// ######################################################################
Image<float> SpaceVariantModule::transformFloat(const Image<float>& image, const ImageSet<float>* const pyr_cache)
{
  return this->transform(image, pyr_cache);
}

// ######################################################################
void SpaceVariantModule::transformFloatPyramid(const Image<float>& image, ImageSet<float>& pyramid, const SVChanLevels& levels, const ImageSet<float>* const pyr_cache)
{
  return this->transformPyramid(image, pyramid, levels, pyr_cache);
}

// ######################################################################
template <class T_or_RGB>
ImageSet<T_or_RGB> SpaceVariantModule::getScaleSpace(const Image<T_or_RGB>& inp, const float& maxrf)
{ 
  ImageSet<T_or_RGB> set;
  if (inp.initialized())
    {
      //re-initialzed if our input dims change
      if (!validTransforms())
        clear(inp.getDims());
      else if (inp.getDims() != itsTransform->getCTDims())
        clear(inp.getDims());
      
      set = (maxrf < 0.0) 
        ? ::getScaleSpace(inp, itsTransform->getMaxRFSize()) 
        : ::getScaleSpace(inp, maxrf);
    }
  return set;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> SpaceVariantModule::transform(const Image<T_or_RGB>& ret_image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
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
      output = transformTo(*itsTransform, ret_image, pyr_cache);
    }
  return output;
}

// ######################################################################
template <class T_or_RGB>
void SpaceVariantModule::transformPyramid(const Image<T_or_RGB>& ret_image, ImageSet<typename promote_trait<T_or_RGB, float>::TP>& pyramid, const SVChanLevels& levels, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
{
  if (ret_image.initialized())
    {
      //re-initialzed if our input dims change
      if (!validTransforms())
        clear(ret_image.getDims());
      else if (ret_image.getDims() != itsTransform->getCTDims())
        clear(ret_image.getDims());
      
      pyramid = transformToPyramid(*itsTransform, ret_image, levels, pyr_cache);

    }
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> SpaceVariantModule::transform(const Image<T_or_RGB>& ret_image, const Point2D<int>& fovea, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
{
  const int i = ret_image.getWidth() / 2 - fovea.i;
  const int j = ret_image.getHeight() / 2 - fovea.j;
  const Image<T_or_RGB> shifted_image = shiftClean(ret_image, i, j);

  return this->transform(shifted_image, pyr_cache);
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> SpaceVariantModule::cropTransform(const Image<T_or_RGB>& ret_image, const Point2D<int>& fovea, const Dims& dims, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache)
{
  if ((dims.w() < ret_image.getWidth()) && (dims.h() < ret_image.getHeight()))
    {
      const int i = ret_image.getWidth() / 2 - fovea.i;
      const int j = ret_image.getHeight() / 2 - fovea.j;
      Point2D<int> c((ret_image.getWidth() - dims.w())/2, (ret_image.getHeight() - dims.h())/2);
      const Image<T_or_RGB> shifted_image = shiftClean(ret_image, i, j);
      return this->transform(crop(shifted_image, c, dims, false), pyr_cache);
    }
  else
    return this->transform(ret_image, fovea, pyr_cache);
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> SpaceVariantModule::inverseTransform(const Image<T_or_RGB>& ret_image) const
{
  Image<T_or_RGB> ret;

  if (itsTransform.is_valid() && (ret_image.getDims() == itsTransform->getSVDims()))
    ret = transformFrom(*itsTransform, ret_image);      

  return ret;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> SpaceVariantModule::inverseMap(const Image<T_or_RGB>& map_image) const
{
  Image<T_or_RGB> ret;
  if (itsMapTransform.is_valid() && (map_image.getDims() == itsMapTransform->getSVDims()))
    ret = transformFrom(*itsMapTransform, map_image);
  return ret;
}
      
// ######################################################################
void SpaceVariantModule::toSvCoords(Point2D<int>& point) const
{
  if (itsTransform.is_valid())
    itsTransform->to(point.i, point.j, point.i, point.j);      
}

// ######################################################################
void SpaceVariantModule::fromSvCoords(Point2D<int>& point) const
{
  if (itsTransform.is_valid())
    itsTransform->from(point.i, point.j, point.i, point.j);      
}

// ######################################################################
void SpaceVariantModule::toSvCoordsMap(Point2D<int>& point) const
{
  if (itsMapTransform.is_valid())
    itsMapTransform->to(point.i, point.j, point.i, point.j);      
}

// ######################################################################
void SpaceVariantModule::fromSvCoordsMap(Point2D<int>& point) const
{
  if (itsMapTransform.is_valid())
    itsMapTransform->from(point.i, point.j, point.i, point.j);      
}

// ######################################################################
bool SpaceVariantModule::validTransforms() const
{
  return (itsTransform.is_valid() && itsMapTransform.is_valid());
}

// ######################################################################
rutz::shared_ptr<SpaceVariantTransform> SpaceVariantModule::getTransform()
{
  return itsTransform;
}

// ######################################################################
rutz::shared_ptr<SpaceVariantTransform> SpaceVariantModule::getMapTransform()
{
  return itsMapTransform;
}

// ######################################################################
void SpaceVariantModule::setTransform(rutz::shared_ptr<SpaceVariantTransform> transform)
{
  itsTransform = transform;
}

// ######################################################################
void SpaceVariantModule::setMapTransform(rutz::shared_ptr<SpaceVariantTransform> map_transform)
{
  itsMapTransform = map_transform;
}

// ######################################################################
float SpaceVariantModule::getMaxRf() const
{
  return itsTransform->getMaxRFSize();
}

// ######################################################################
float SpaceVariantModule::getMaxRf(const float& offset, const float& surround_factor)
{
  return (itsTransform->getMaxRFSize() + offset) * surround_factor;
}

#define INST_CLASS SpaceVariantModule::
#include "inst/SpaceVariant/SpaceVariantModule.I"
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
