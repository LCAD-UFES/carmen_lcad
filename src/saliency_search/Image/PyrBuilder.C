/*!@file Image/PyrBuilder.C Classes for building dyadic pyramids */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PyrBuilder.C $
// $Id: PyrBuilder.C 14473 2011-02-03 20:36:39Z dberg $
//

#include "Image/PyrBuilder.H"

#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/Pixels.H"
#include "Image/PyramidCache.H"
#include "Image/PyramidOps.H"
#include "rutz/trace.h"

// ######################################################################
// ##### PyrBuilder functions:
// ######################################################################

// ######################################################################
template <class T>
PyrBuilder<T>::PyrBuilder()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
template <class T>
PyrBuilder<T>::~PyrBuilder()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
template <class T>
void PyrBuilder<T>::reset()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
// ##### GaussianPyrBuilder Functions:
// ######################################################################

template <class T>
GaussianPyrBuilder<T>::GaussianPyrBuilder(int filter_size) :
  PyrBuilder<T>(), itsFiltSize(filter_size)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> GaussianPyrBuilder<T>::build(const Image<T>& img,
                                         const int firstlevel,
                                         const int depth,
                                         PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const ImageSet<T>* const cached =
    (cache != 0 && itsFiltSize == 5)
    ? cache->gaussian5.get(img) // may be null if there is no cached pyramid
    : 0;

  return (cached != 0)
    ? *cached
    : buildPyrGaussian(img, firstlevel, depth, itsFiltSize);
}

template <class T> inline
GaussianPyrBuilder<T>* GaussianPyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new GaussianPyrBuilder<T>(*this);
}

// ######################################################################
// ##### GaussianRadialPyrBuilder Functions:
// ######################################################################

template <class T>
GaussianRadialPyrBuilder<T>::GaussianRadialPyrBuilder() :
  PyrBuilder<T>()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> GaussianRadialPyrBuilder<T>::build(const Image<T>& img,
                                         const int firstlevel,
                                         const int depth,
                                         PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);

 const ImageSet<T>* const cached = (cache != 0)
   ? cache->gaussian5.get(img) // may be null if there is no cached pyramid
   : 0;

 return (cached != 0)
   ? *cached
   : buildRadialPyrGaussian(img, firstlevel, depth);
}

template <class T> inline
GaussianRadialPyrBuilder<T>* GaussianRadialPyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new GaussianRadialPyrBuilder<T>(*this);
}

// ######################################################################
// ##### ConvolvePyrBuilder Functions:
// ######################################################################

template <class T>
ConvolvePyrBuilder<T>::
ConvolvePyrBuilder(const Image<float>& filt,
                   ConvolutionBoundaryStrategy boundary) :
  PyrBuilder<T>(), itsFilt(filt), itsBoundary(boundary)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> ConvolvePyrBuilder<T>::build(const Image<T>& img,
                                         const int firstlevel,
                                         const int depth,
                                         PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return buildPyrConvolve(img, firstlevel, depth, itsFilt, itsBoundary);
}

template <class T>
ConvolvePyrBuilder<T>* ConvolvePyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new ConvolvePyrBuilder<T>(*this);
}

// ######################################################################
// ##### RGBConvolvePyrBuilder Functions:
// ######################################################################

template <class T>
RGBConvolvePyrBuilder<T>::
RGBConvolvePyrBuilder(const Image<float>& rfilt,
                      const Image<float>& gfilt,
                      const Image<float>& bfilt,
                      ConvolutionBoundaryStrategy boundary) :
  PyrBuilder<T>(), itsRFilt(rfilt), itsGFilt(rfilt), itsBFilt(rfilt),
  itsBoundary(boundary)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> RGBConvolvePyrBuilder<T>::
build(const Image< PixRGB<T> >& img,
      const int firstlevel,
      const int depth,
      PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> r, g, b; getComponents(img, r, g, b);
  ImageSet<T> rset = buildPyrConvolve(r, firstlevel, depth, itsRFilt, itsBoundary);
  ImageSet<T> gset = buildPyrConvolve(g, firstlevel, depth, itsGFilt, itsBoundary);
  ImageSet<T> bset = buildPyrConvolve(b, firstlevel, depth, itsBFilt, itsBoundary);
  ImageSet<T> result(depth);

  for (int lev = firstlevel; lev < depth; ++lev)
    result[lev] = (rset[lev] + gset[lev] + bset[lev]) / 3;

  return result;
}

template <class T>
ImageSet<T> RGBConvolvePyrBuilder<T>::
build(const Image<T>& img,
      const int firstlevel,
      const int depth,
      PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  LFATAL("Cannot process greyscale images");
  return ImageSet<T>();
}

template <class T>
ImageSet< PixRGB<T> > RGBConvolvePyrBuilder<T>::
build2(const Image< PixRGB<T> >& img,
       const int firstlevel,
       const int depth,
       PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> r, g, b; getComponents(img, r, g, b);
  ImageSet<T> rset = buildPyrConvolve(r, firstlevel, depth, itsRFilt, itsBoundary);
  ImageSet<T> gset = buildPyrConvolve(g, firstlevel, depth, itsGFilt, itsBoundary);
  ImageSet<T> bset = buildPyrConvolve(b, firstlevel, depth, itsBFilt, itsBoundary);
  ImageSet< PixRGB<T> > result(depth);

  for (int lev = firstlevel; lev < depth; ++lev)
    result[lev] = makeRGB (rset[lev], gset[lev], bset[lev]);

  return result;
}

template <class T>
RGBConvolvePyrBuilder<T>* RGBConvolvePyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new RGBConvolvePyrBuilder<T>(*this);
}

// ######################################################################
// ##### LaplacianPyrBuilder Functions:
// ######################################################################

template <class T>
LaplacianPyrBuilder<T>::LaplacianPyrBuilder(const int filter_size) :
  PyrBuilder<T>(), itsFiltSize(filter_size)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> LaplacianPyrBuilder<T>::build(const Image<T>& img,
                                          const int firstlevel,
                                          const int depth,
                                          PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return buildPyrLaplacian(img, firstlevel, depth, itsFiltSize);
}

template <class T> inline
LaplacianPyrBuilder<T>* LaplacianPyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new LaplacianPyrBuilder<T>(*this);
}

// ######################################################################
// ##### OrientedPyrBuilder Functions:
// ######################################################################

template <class T>
OrientedPyrBuilder<T>::OrientedPyrBuilder(const int filter_size,
                                          const float theta,
                                          const float intens,
                                          const bool usetab) :
  PyrBuilder<T>(), itsFiltSize(filter_size), itsAngle(theta),
  itsGaborIntens(intens), itsUseTab(usetab)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> OrientedPyrBuilder<T>::build(const Image<T>& img,
                                         const int firstlevel,
                                         const int depth,
                                         PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const ImageSet<T>* const lplc =
    (cache != 0 && itsFiltSize == 9)
    ? cache->laplacian9.get(img) // may be null if there is no cached pyramid
    : 0;

  return lplc != 0
    ? buildPyrOrientedFromLaplacian<T>(*lplc,
                                       itsFiltSize, itsAngle, itsGaborIntens,
                                       itsUseTab)
    : buildPyrOriented(img, firstlevel, depth,
                       itsFiltSize, itsAngle, itsGaborIntens, itsUseTab);
}

template <class T>
OrientedPyrBuilder<T>* OrientedPyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new OrientedPyrBuilder<T>(*this);
}


// ######################################################################
// ##### GenericPyrBuilder Functions:
// ######################################################################

template <class T>
GenericPyrBuilder<T>::GenericPyrBuilder(const PyramidType typ,
                                        const float gabor_theta,
                                        const float intens) :
  PyrBuilder<T>(), itsPtype(typ), itsGaborAngle(gabor_theta),
  itsGaborIntens(intens)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> GenericPyrBuilder<T>::build(const Image<T>& image,
                                        const int firstlevel,
                                        const int depth,
                                        PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return buildPyrGeneric(image, firstlevel, depth, itsPtype,
                         itsGaborAngle, itsGaborIntens);
}

template <class T>
GenericPyrBuilder<T>* GenericPyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new GenericPyrBuilder<T>(*this);
}

// ######################################################################
// ##### ReichardtPyrBuilder Functions:
// ######################################################################
template <class T>
ReichardtPyrBuilder<T>::ReichardtPyrBuilder(const float dx,
                                            const float dy,
                                            const PyramidType typ,
                                            const float gabor_theta,
                                            const float intens) :
  PyrBuilder<T>(), itsDX(dx), itsDY(dy), itsPtype(typ),
  itsGaborAngle(gabor_theta), itsGaborIntens(intens)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

template <class T>
ImageSet<T> ReichardtPyrBuilder<T>::build(const Image<T>& image,
                                          const int firstlevel,
                                          const int depth,
                                          PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const ImageSet<T>* const cached =
    (cache != 0 && itsPtype == Gaussian5)
    ? cache->gaussian5.get(image) // may be null if there is no cached pyramid
    : 0;

  // create a pyramid with the input image
  ImageSet<T> upyr =
    cached != 0
    ? *cached
    : buildPyrGeneric(image, firstlevel, depth, itsPtype,
                      itsGaborAngle, itsGaborIntens);
  // create an empty pyramid
  ImageSet<T> spyr(depth);

  // fill the empty pyramid with the shifted version
  for (int i = firstlevel; i < depth; ++i)
    spyr[i] = shiftImage(upyr[i], itsDX, itsDY);

  // store both pyramids in the deques
  unshifted.push_back(upyr);
  shifted.push_back(spyr);

  ImageSet<T> result(depth);

  // so, it's our first time? Pretend the pyramid before this was
  // the same as the current one ...
  if (unshifted.size() == 1)
    {
      unshifted.push_back(upyr);
      shifted.push_back(spyr);
    }

  // need to pop off old pyramid?
  if (unshifted.size() == 3)
    {
      unshifted.pop_front();
      shifted.pop_front();
    }

  // compute the Reichardt maps
  for (int i = firstlevel; i < depth; ++i)
    {
      result[i] =
        (unshifted.back()[i] * shifted.front()[i]) -
        (unshifted.front()[i] * shifted.back()[i]);
    }

  return result;
}

// ######################################################################
template <class T>
ReichardtPyrBuilder<T>* ReichardtPyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new ReichardtPyrBuilder<T>(*this);
}

// ######################################################################
template <class T>
void ReichardtPyrBuilder<T>::reset()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  shifted.clear();
  unshifted.clear();
}

// ######################################################################
// ##### TemplateMatchPyrBuilder Functions:
// ######################################################################
TemplateMatchPyrBuilder::TemplateMatchPyrBuilder(const Image<float>& templ) :
  PyrBuilder<float>(), itsFilt(templ)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
TemplateMatchPyrBuilder* TemplateMatchPyrBuilder::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new TemplateMatchPyrBuilder(*this);
}

// ######################################################################
ImageSet<float> TemplateMatchPyrBuilder::build(const Image<float>& image,
                                               const int firstlevel,
                                               const int depth,
                                               PyramidCache<float>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ImageSet<float> result(depth);
  if (0 >= firstlevel)
    result[0] = templateMatch(image);
  Image<float> prev_scale = image;

  for (int lev = 1; lev < depth; ++lev)
    {
      Image<float> cur_scale = prev_scale;

      cur_scale = decX(lowPass5x(cur_scale));
      cur_scale = decY(lowPass5y(cur_scale));

      // Save the unconvolved image at this scale to be used as the
      // starting point for the next scale
      prev_scale = cur_scale;

      if (lev >= firstlevel)
        result[lev] = templateMatch(cur_scale);
    }

  return result;
}

// ######################################################################
Image<float> TemplateMatchPyrBuilder::templateMatch(const Image<float>& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> conv = convolve(img, itsFilt,
                               CONV_BOUNDARY_ZERO);
  Image<float> conv2 = convolve(binaryReverse(img, 255.0f), itsFilt,
                                CONV_BOUNDARY_ZERO);
  return takeMax(conv, conv2);
}

// ######################################################################
// ##### GaborPyrBuilder Functions:
// ######################################################################
template <class T>
GaborPyrBuilder<T>::GaborPyrBuilder(double angle,
                                    double filter_period,
                                    double elongation,
                                    int size,
                                    int buildFlags) :
  PyrBuilder<T>(),
  itsAngle(angle),
  itsPeriod(filter_period),
  itsElongation(elongation),
  itsSize(size),
  itsBuildFlags(buildFlags)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
template <class T>
GaborPyrBuilder<T>* GaborPyrBuilder<T>::clone() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return new GaborPyrBuilder<T>(*this);
}

// ######################################################################
template <class T>
ImageSet<T> GaborPyrBuilder<T>::build(const Image<T>& img,
                                      const int firstlevel,
                                      const int depth,
                                      PyramidCache<T>* cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return buildPyrGabor(img, firstlevel, depth,
                       itsAngle, itsPeriod, itsElongation,
                       itsSize, itsBuildFlags);
}

// ######################################################################
template <class T>
ImageSet<T> GaborPyrBuilder<T>::input(const ImageSet<T>& pyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return buildPyrGabor(pyr, itsAngle, itsPeriod, itsElongation,
                       itsSize, itsBuildFlags);
}

// ######################################################################
// ##### Instantiations
// ######################################################################
template class GaborPyrBuilder<float>;
template class RGBConvolvePyrBuilder<float>;

#define INSTANTIATE(T) \
template class PyrBuilder< T >; \
template class GenericPyrBuilder< T >; \
template class GaussianPyrBuilder< T >; \
template class GaussianRadialPyrBuilder< T >; \
template class ConvolvePyrBuilder< T >; \
template class LaplacianPyrBuilder< T >; \
template class OrientedPyrBuilder< T >; \
template class ReichardtPyrBuilder< T >; \

template class PyrBuilder<int>;

#ifdef INVT_INST_BYTE
INSTANTIATE(byte);
INSTANTIATE(PixRGB<byte>);
#endif
#ifdef INVT_INST_INT16
INSTANTIATE(int16);
INSTANTIATE(PixRGB<int16>);
#endif
#ifdef INVT_INST_INT32
INSTANTIATE(int32);
INSTANTIATE(PixRGB<int32>);
#endif
#ifdef INVT_INST_FLOAT
INSTANTIATE(float);
INSTANTIATE(PixRGB<float>);
#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
