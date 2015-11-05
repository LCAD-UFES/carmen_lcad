/*!@file SpaceVariant/SpaceVariantModule.H a model component that embodies different
    space variant image transforms (e.g. foveation). */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SpaceVariantModule.H $

#ifndef SPACEVARIANTMODULE_H_DEFINED
#define SPACEVARIANTMODULE_H_DEFINED


#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "SpaceVariant/SpaceVariantTransforms.H"
#include "Image/Pixels.H"
#include "Image/LevelSpec.H"

class SVChanLevels;
class ModelManager;
// ###################################################################### 
// a Model Component that embodies a space variant transform. Derive 
// from this class and implement clear() to create your own transform.
// See Image/SpaceVariantTransforms.H for details on the computation. 
// The tempalte parameter should be the type of space variant transform
// the user desires. 
// ######################################################################
class SpaceVariantModule : public ModelComponent
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  SpaceVariantModule(OptionManager& mgr, const std::string& descrName = "SpaceVariantModule",
                     const std::string& tagName = "SpaceVariantModule");

  //! Destructor
  virtual ~SpaceVariantModule();
  
  //! reset transforms to the desired input image size
  virtual void clear(const Dims& inp_dims);

  //! clear the transforms
  void clear();

  //!get the sv image dims
  Dims getSVImageDims() const;

  /*!Here, for polymoprhism, we provide virtual functions for commonly used types. 
    However, for flexibility we also provide generic template functions. */

  //! transform a cartesian image to a space variant one
  virtual Image<PixRGB<float> > transformRbg(const Image<PixRGB<byte> >& image, const ImageSet<PixRGB<float> >* const pyr_cache = NULL);
  
  //! transform a cartesian image to a space variant pyramid
  virtual void transformRgbPyramid(const Image<PixRGB<byte> >& image, ImageSet<PixRGB<float> >& pyramid, const SVChanLevels& levels, const ImageSet<PixRGB<float> >* const pyr_cache);
  
  //! transform a cartesian image to a space variant one
  virtual Image<float> transformFloat(const Image<float>& image, const ImageSet<float>* const pyr_cache = NULL);
  
  //! transform a cartesian image to a space variant pyramid
  virtual void transformFloatPyramid(const Image<float>& image, ImageSet<float>& pyramid, const SVChanLevels& levels, const ImageSet<float>* const pyr_cache);
  
  //!Compute just the scale space sometimes we may want to just get the scale space
  template <class T_or_RGB>
  ImageSet<T_or_RGB> getScaleSpace(const Image<T_or_RGB>& inp, const float& maxrf = -1.0F);
  
  //! transform a cartesian image to a space variant one
  template <class T_or_RGB>
  Image<T_or_RGB> transform(const Image<T_or_RGB>& image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);

  //! transform a cartesian image to a space variant pyramid
  template <class T_or_RGB>
  void transformPyramid(const Image<T_or_RGB>& image, ImageSet<typename promote_trait<T_or_RGB, float>::TP>& pyramid, const SVChanLevels& levels, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);
  
  /*! transform a cartesian image to a space variant one, 
    but first center the image to the desired point. */
  template <class T_or_RGB>
  Image<T_or_RGB> transform(const Image<T_or_RGB>& image, const Point2D<int>& fovea, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);

  /*! transform a cartesian image to a space variant one, 
    but first center the image to the desired point and crop. */
  template <class T_or_RGB>
  Image<T_or_RGB> cropTransform(const Image<T_or_RGB>& image, const Point2D<int>& fovea, const Dims& dims, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);
  
  //! inverse transform an already space variant transformed image back to a cartesian one
  template <class T_or_RGB>
  Image<T_or_RGB> inverseTransform(const Image<T_or_RGB>& ret_image) const;
  
  /*! We may perform pyramid operations on space variant images and this function allows for images
    at the output map scale (see LevelSpec.H) to be inverse transformed back to the input image dimensions. */
  template <class T_or_RGB>
  Image<T_or_RGB> inverseMap(const Image<T_or_RGB>& map_image) const;

  //!transform a point from cartesian to space variant coords
  void toSvCoords(Point2D<int>& point) const;

  //!transform a point from space variant to cartesian coords
  void fromSvCoords(Point2D<int>& point) const;

  //!transform a point to space variant coords at the map level
  void toSvCoordsMap(Point2D<int>& point) const;

  //!transform a point from space variant coords at the map level
  void fromSvCoordsMap(Point2D<int>& point) const;

  //!returnt true if we have valid transforms
  bool validTransforms() const;

  //!return the transform
  rutz::shared_ptr<SpaceVariantTransform> getTransform();

  //!return the map level transform
  rutz::shared_ptr<SpaceVariantTransform> getMapTransform();

  //!set the transform
  void setTransform(rutz::shared_ptr<SpaceVariantTransform> transform);
  
  //!set the map level transform
  void setMapTransform(rutz::shared_ptr<SpaceVariantTransform> map_transform);

  //!get the max rf size in sigmas
  float getMaxRf() const;

  //!get the max rf size in sigmas given an offset and surround factor
  float getMaxRf(const float& offset, const float& surround_factor);

protected:
  //reset our internals
  void reset1();

  //the transforms
  rutz::shared_ptr<SpaceVariantTransform> itsTransform;
  rutz::shared_ptr<SpaceVariantTransform> itsMapTransform;
};

#endif //SPACEVARIANTMODULE
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
