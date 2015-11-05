/*!@file SpaceVariant/SpaceVarinatEdgeModule.H a model component that embodies a 
  space variant edge detector */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SpaceVarinatEdgeModule.H $

#ifndef SPACEVARIANTEDGEMODULE_H_DEFINED
#define SPACEVARIANTEDGEMODULE_H_DEFINED

#include "SpaceVariant/FovealTransformModule.H"

// ###################################################################### 
// an edge version of the space variant transform See SpaceVariantTransforms.H 
// and ScaleSpaceOps.H for details on the computation. 
// ######################################################################
class SpaceVariantEdgeModule : public FovealTransformModule
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  SpaceVariantEdgeModule(OptionManager& mgr, const std::string& descrName = "SpaceVariantEdgeModule",
                         const std::string& tagName = "SpaceVarianEdgetModule");
  
  //! Destructor
  virtual ~SpaceVariantEdgeModule();

  //!override clear  
  void clear(const Dims& dims);
  
  //!override transfromRGB
  Image<PixRGB<float> > transformRGB(const Image<PixRGB<byte> >& image, 
                                     const ImageSet<PixRGB<float> >* const pyr_cache = NULL);

  //! transform a cartesian image to a space variant pyramid
  virtual void transformRgbPyramid(const Image<PixRGB<byte> >& image, ImageSet<PixRGB<float> >& pyramid, const SVChanLevels& levels, const ImageSet<PixRGB<float> >* const pyr_cache);
  
  //!override transformFloat
  Image<float> transformFloat(const Image<float>& image, const ImageSet<float>* const pyr_cache = NULL);

  //! transform a cartesian image to a space variant pyramid
  virtual void transformFloatPyramid(const Image<float>& image, ImageSet<float>& pyramid, const SVChanLevels& levels, const ImageSet<float>* const pyr_cache);
  
  /*!transform a cartesian image to a space variant difference of Gaussian image. Notice, 
  // that apposed to transform() where the same time that is input is returned
  // (by clamping and converting), a float image (or PixRGB<float>) is returned here as the DoG
  // may result in negative numbers */
  template <class T_or_RGB>
  Image<typename promote_trait<T_or_RGB, float>::TP> transformEdge(const Image<T_or_RGB>& ret_image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);

  //! transform a cartesian image to a space variant pyramid
  template <class T_or_RGB>
  void transformEdgePyramid(const Image<T_or_RGB>& image, ImageSet<typename promote_trait<T_or_RGB, float>::TP>& pyramid, const SVChanLevels& levels, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);

  
protected:
  void start1();
private:
  OModelParam<float> itsSvDoGSize;  
  OModelParam<float> itsSvEdgeOrient;  
  OModelParam<uint> itsSvEdgeOLength;  
  OModelParam<uint> itsSvEdgeODensity;  

  Image<LocalEdge> itsEdges;
  bool useDog;
};

#endif //SPACEVARIANTEDGEMODULE
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
