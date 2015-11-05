/*!@file SpaceVariant/SCTransformModule.H  */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SCTransformModule.H $

#ifndef SCTRANSFORMMODULE_H_DEFINED
#define SCTRANSFORMMODULE_H_DEFINED

#include "SpaceVariant/SpaceVariantModule.H"
#include "Psycho/PixPerDeg.H"

// ###################################################################### 
// A module which embodies a superior colliculus transform
// ######################################################################
class SCTransformModule : public SpaceVariantModule
{
  public:
    //! Constructor
    /*! See ModelComponent.H for details */
    SCTransformModule(OptionManager& mgr, const std::string& descrName = "SCTransform Module",
                      const std::string& tagName = "SCTransformModule");
    
    //! Destructor
    virtual ~SCTransformModule();
    
    //! reset transforms to the desired input image size
    virtual void clear(const Dims& inp_dims);    

  /*!transform a cartesian image to a space variant difference of Gaussian image. Notice, 
  // that apposed to transform() where the same time that is input is returned
  // (by clamping and converting), a float image (or PixRGB<float>) is returned here as the DoG
  // may result in negative numbers */
  template <class T_or_RGB>
  Image<typename promote_trait<T_or_RGB, float>::TP> transformDoG(const Image<T_or_RGB>& ret_image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);

  /*!transform a cartesian image to a space variant division of Gaussian image. Notice, 
  // that apposed to transform() where the same time that is input is returned
  // (by clamping and converting), a float image (or PixRGB<float>) is returned here as the DivG
  // may result in negative numbers */
  template <class T_or_RGB>
  Image<typename promote_trait<T_or_RGB, float>::TP> transformDivG(const Image<T_or_RGB>& ret_image, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);

  //! transform a cartesian image to a space variant pyramid
  template <class T_or_RGB>
  void transformDoGPyramid(const Image<T_or_RGB>& image, ImageSet<typename promote_trait<T_or_RGB, float>::TP>& pyramid, const SVChanLevels& levels, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const pyr_cache = NULL);

  //!transform a point from cartesian to space variant coords
  Point2D<int> degToSvCoords(Point2D<float> const & point) const;

  //!get the ppd
  PixPerDeg const & getPPD() const;

    //!set the ppd
  void setPPD(PixPerDeg const & ppd);
    
  protected:
    void start1();

    OModelParam<Dims> itsSvDims; //dims of transform
    OModelParam<PixPerDeg> itsPixPerDeg;//pixels per degree
    OModelParam<float> itsMaxDeg;//max visual angle

    NModelParam<bool> itsSvDoGCenter;  
    NModelParam<float> itsSvDoGSize;  

    NModelParam<float> itsS1;
    NModelParam<float> itsS2;
    NModelParam<float> itsBeta;
    NModelParam<float> itsAlpha;
    NModelParam<float> itsRfSlope;
    NModelParam<float> itsRfExp;
    NModelParam<float> itsRfOffset;

    PixPerDeg itsPPD;
};

#endif //SCTRANSFORMMODULE
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
