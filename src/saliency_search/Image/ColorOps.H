/*!@file Image/ColorOps.H Color operations on Image
 */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ColorOps.H $
// $Id: ColorOps.H 15059 2011-11-11 00:55:58Z dberg $
//

// The fromXXX() code here is somewhat inspired from the
// grab_gray_image.c example code provided with the libdc1394
// distribution by Gord Peters, and from the Coriander source code.

#ifndef IMAGE_COLOROPS_H_DEFINED
#define IMAGE_COLOROPS_H_DEFINED

#include "Image/Image.H"
#include "Util/Assert.H"
#include "Util/Promotions.H"

class Dims;
template <class T> class Image;
template <class T> class PixRGB;
class ColorMap;

//! Set the red, green and blue components from 3 monochromatic images
template <class T>
Image<PixRGB<T> > makeRGB(const Image<T>& red,
                          const Image<T>& green,
                          const Image<T>& blue);

//! Colorize a greyscale byte image using a colormap
/*! Note that this will throw a fatal error if the colormap does not
  have exactly 256 entries. */
Image< PixRGB<byte> > colorize(const Image<byte>& img, const ColorMap& cmap);

//! Add color speckle noise to array; faster version, draws 'num' dots
void inplaceColorSpeckleNoise(Image< PixRGB<byte> >& dest, const int num);

//! Get the red, green and blue components as 3 monochromatic images
template <class T>
void getComponents(const Image<PixRGB<T> >& src,
                   Image<T>& red, Image<T>& green, Image<T>& blue);

//! Split out a single pixel component image from a multispectral pixel input image
template <class PixT>
inline Image<typename PixT::ScalarType>
getPixelComponentImage(const Image<PixT>& in, const size_t i)
{
  typedef typename PixT::ScalarType T;

  Image<T> result(in.getDims(), NO_INIT);

  typename Image<PixT>::const_iterator sptr = in.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  ASSERT(i < PixT::myDim);

  while (dptr != stop)
    {
      *dptr = sptr->p[i];
      ++dptr; ++sptr;
    }

  return result;
}

/*! This method will normalize an image based upon the maximum value
  provided to it. This is either max or the absolute value of min.
  It then creates an image where all negative values are assigned a red
  pixel value while all postive numbers are assigned a green pixel value
  @param max The maximum pixel value
  @param min The minimum pixel value
  @param image The image of values (positive and negative) to be processed
*/
Image<PixRGB<float> > normalizeRGPolar(const Image<float>& src,
                                       const float max,
                                       const float min);
/*! This method will normalize an image based upon the maximum value
  provided to it. This is either max or the absolute value of min.
  It then creates an image where all negative values are assigned a red
  pixel value while all postive numbers are assigned a green pixel value.
  This version will autmatically determine min and max and then normalize
  between 0 and 255 for red and green
  @param image The image of values (positive and negative) to be processed
*/
Image<PixRGB<float> > normalizeRGPolarAuto(const Image<float>& src);
//! Normalize a grey scale image, with some idea for scale
/*! nromalize a grey scale image, but to give an idea of the actual scale,
    we leave one channel as the orignal value and set the other the new
    value. This way, the intensity is still normalized over once channel
    so that you can make out the image features, but the hue change lets
    you know that the real intensity was very high.

    Note: the output baseColor and normColor should be 1,2 or 3. They should
    also not be equal. These are which channel is set as the base and
    normalized channel representations.

    Also Note: The base color will be clamped to between 0 and clamp so
    that it can still be displayed

    @param src The orignal grey scale image
    @param min The low bound on the normalization, for instance 0
    @param max The high bound on the normalization, for instance 255
    @param clamp clamp the baseColor to no more than this value
    @param baseColor, 1,2,3 ; R,G,B : which channel to not normalize out
    @param normColor, 1,2,3 ; R,G,B : which channel to normalize out
*/
Image<PixRGB<float> > normalizeWithScale(const Image<float>& src,
                                         const float min,
                                         const float max,
                                         const float clamp,
                                         const char baseColor,
                                         const char normColor);

//! Normalize a grey scale image, with some idea for scale
/*! Here we normalize the image using translation between RGB and HSV
    what we do is set the intensity as the basic normalized value
    of the original gray scale image. However, we use hue as the
    absolute scale. Saturation is constant at 100 (full).
*/
Image<PixRGB<float> > normalizeScaleRainbow(const Image<float>& src,
                                            const float min,
                                            const float max);
//! Create a color image by staining a monochrome image.
/*! Each pixel in the result is given by the stain color multiplied by
    the corresponding pixel in the monochrome source image. This is
    entirely equivalent to, but more efficient than, creating a color
    image from the monochrome image, then multiplying the color image
    by the stain color.*/
template <class T>
Image<PixRGB<T> > stain(const Image<T>& src, PixRGB<float> color);

//! Create a color image from src by tinting positive and negative values
/*! For each pixel x in the input image, we compute ratio=x/maxval;
    then if the ratio is positive, the output value is ratio*pos_stain
    + (1-ratio)*background, and if the ratio is negative, then the
    output value is (-ratio)*neg_stain + (1-(-ratio))*background. */
Image<PixRGB<float> > stainPosNeg(const Image<float>& src,
                                  const float maxval,
                                  const PixRGB<float>& background,
                                  const PixRGB<float>& pos_stain,
                                  const PixRGB<float>& neg_stain);

//! Overlay an image over another, however, stain the top image
/*! This method works almost exactly like overlay except that the
    bottom image is stained (colored) red green or blue depending on
    whether char channel is set to r,g, or b */
Image<PixRGB<float> > overlayStain(const Image<float>& top,
                                   const Image<float>& bottom,
                                   const float trans, const char channel);

//! Get min and max of the components
template <class T>
void getMinMaxC(const Image<PixRGB<T> >& src, T& mi, T& ma);

//! Get the mean RGB of an Image
template <class T>
PixRGB<float> meanRGB(const Image<PixRGB<T> >& src);

//! Normalize values between nmin and nmax
template <class T, class T2>
inline void normalizeC(Image<PixRGB<T> >& src, const T2 nmin, const T2 nmax)
{
  ASSERT(src.initialized());
  T mi, ma; getMinMaxC(src, mi, ma);
  const float mif = float(mi);
  const float maf = float(ma);
  const float scale = maf - mif;
  if (scale > -1.0e-20 && scale < 1.0e-20) return;  // no need to rescale...
  const float nscale = (float(nmax) - float(nmin)) / scale;

  typename Image<PixRGB<T> >::iterator aptr = src.beginw();
  typename Image<PixRGB<T> >::iterator stop = src.endw();

  while (aptr != stop)
    {
      aptr->setRed  (nmin+(T2)(((float(aptr->red()))-mif)*nscale));
      aptr->setGreen(nmin+(T2)(((float(aptr->green()))-mif)*nscale));
      aptr->setBlue (nmin+(T2)(((float(aptr->blue()))-mif)*nscale));
      ++aptr;
    }
}

//! Compute luminance of a color image
template <class T>
Image<T> luminance(const Image<PixRGB<T> >& src);

//! Compute luminance of a greyscale image (no-op)
/*! This no-op overload is provided so that luminance() can be applied
    to template images where it is not known whether the template
    argument is scalar or PixRGB. */
template <class T>
Image<T> luminance(const Image<T>& src);

//! Compute luminance in NTSC coordinates
/*! This version performs the same function as rgb2gray() in matlab */
template <class T>
Image<T> luminanceNTSC(const Image<PixRGB<T> >& src);

//! Convert grayscale image to RGB
template <class T>
Image< PixRGB<T> > toRGB(const Image<T>& src);

//! Convert RGB image to RGB (no-op)
/*! This no-op function is provided so that toRGB() can be applied to
  template images where it is not known whether the template argument
  is scalar or PixRGB. */
template <class T>
Image< PixRGB<T> > toRGB(const Image< PixRGB<T> >& src);

//! Compute color information measure based in infoFFT
template <class T>
Image<float> infoMeasure(const Image<PixRGB<T> >& src,
                         const float eps, const int size);

//! Get YIQ color components
template <class T>
void getYIQ(const Image<PixRGB<T> >& src,
            Image<T>& y, Image<T>& i, Image<T>& q);

//! Get YUV color components from an RGB image
template <class T>
void getJpegYUV(const Image<PixRGB<T> >& src,
                Image<T>& y, Image<T>& u, Image<T>& v);

//! Normalize my values by those in the lum image
template <class T>
Image<PixRGB<T> > luminanceNormalize(const Image<PixRGB<T> >& src,
                                     const T thresh);

//! Compute R/G and B/Y channels from a luminanceNormalized color image
/*! This version does automatic promotion on the results, so that
    negative values can be represented even if the input is of type
    PixRGB<byte> */
template <class T>
void getRGBY(const Image<PixRGB<T> >& src,
             Image<typename promote_trait<T, float>::TP>& rg,
             Image<typename promote_trait<T, float>::TP>& by,
             const typename promote_trait<T, float>::TP thresh);

//! Compute R/G and B/Y channels in a simpler way
/*! Params are as in getRGBY. The computation for the RG and BY maps
    is: RG = (R-G)/max(R,G,B); BY = (B-min(R,G))/max(R,G,B).*/
template <class T>
void getRGBYsimple(const Image<PixRGB<T> >& src,
                   Image<typename promote_trait<T, float>::TP>& rg,
                   Image<typename promote_trait<T, float>::TP>& by,
                   const typename promote_trait<T, float>::TP thresh);

//! Compute R, G, B, Y channels from a luminanceNormalized color image
/*! This version keeps the four channels separate so that correct
    results can be obtained without requiring promotion even if the
    input is of type PixRGB<byte> */
template <class T>
void getRGBY(const Image<PixRGB<T> >& src, Image<T>& r, Image<T>& g,
             Image<T>& b, Image<T>& y, const T thresh);

//! Compute R/G and B/Y via computation of H2SV2 color space
/*! In addition to returning R/G and B/Y this basically returns the
    full H2SV color space. the original RGB can be obtained later
    via conversion of rg,by, sat and val using a PixH2SV2 to RGB
*/
template <class T>
void getRGBY(const Image<PixRGB<T> >& src, Image<T>& rg,  Image<T>& by,
             Image<T>& sat, Image<T>& val, const ushort H2SVtype = 2);

//! Compute D, K, L color components from an RGB image
/*! The DKL color space is motivated by the neurobiology of early
    vision in primates. D is roughly like luminance (with some neutral
    grey (RGB=[161 159 154]) at zero, white around 1.0, and black
    around -1.0), K roughly like Red-Green, and L roughly like
    Blue-Yellow. The images returned here have signed float values
    roughly in [-1,1] (but they sometimes are slightly larger that exactly
    that, so be sure to clamp if you ar egoing to convert). */
template <class T>
void getDKL(const Image<PixRGB<T> >& src,
            Image<typename promote_trait<T, float>::TP>& dimg,
            Image<typename promote_trait<T, float>::TP>& kimg,
            Image<typename promote_trait<T, float>::TP>& limg);

//! Compute DKL components (R/G, B/Y, Lumiance) from an RGB image
/*! The DKL color space is motivated by the neurobiology of early vision in
    primates. This version is specific to the Sony Bravia monitor used in the
    Doug Munoz laboratory at Queen's University in Kingston, ON, CA. The first
    component is rougly Green-Red, the second is roughly Yellow-Blue and the
    third is roughly luminance. The images returned here have signed float
    values where the two chromatic channels range from [-1 1] and the luminance
    channel from [0 1]. */
template <class T>
void getDKLM(const Image<PixRGB<T> >& src,
             Image<typename promote_trait<T, float>::TP>& RGimg,
             Image<typename promote_trait<T, float>::TP>& BYimg,
             Image<typename promote_trait<T, float>::TP>& LUMimg);

//! Compute L, A, B color components from an RGB image
/*! Derived from By Mark Ruzon from C code by Yossi Rubner, 23 September 1997 
 * A Lab color space is a color-opponent space with dimension L for lightness and
 * a and b for the color-opponent dimensions, based on nonlinearly compressed CIE
 * XYZ color space coordinates. Lab color is designed to approximate human vision.
 * It aspires to perceptual uniformity, and its L  component closely matches human
 * perception of lightness. It can thus be used to make accurate color balance corrections
 * by modifying output curves in the a and b components, or to adjust the lightness
 * contrast using the L  component. In RGB or CMYK spaces, which model the output of
 * physical devices rather than human visual perception, these transformations can
 * only be done with the help of appropriate blend modes in the editing application.
 */
template <class T>
void getLAB(const Image<PixRGB<T> >& src,
            Image<typename promote_trait<T, float>::TP>& limg,
            Image<typename promote_trait<T, float>::TP>& aimg,
            Image<typename promote_trait<T, float>::TP>& bimg);

//! normalize the LAB color space to lie in [0.0, 1.0]
//! this is more complex than just calling inplaceNormalize
//! NOTE: we assume that this is called right getLAB 
//!       separate calls usually are done if user wants both
//!       normalized and unnormalized value of the LAB color space
void normalizeLAB(Image<float>& limg,
                  Image<float>& aimg,
                  Image<float>& bimg);

//! just call getLAB and normalizeLAB
template <class T>
void getNormalizedLAB(const Image<PixRGB<T> >& src,
                      Image<typename promote_trait<T, float>::TP>& limg,
                      Image<typename promote_trait<T, float>::TP>& aimg,
                      Image<typename promote_trait<T, float>::TP>& bimg);

//! contrast-modulate an RGB image with a mask
Image< PixRGB<byte> > contrastModulate(const Image< PixRGB<byte> >& img,
                                       const Image<float>& mask,
                                       float baseContrast = 0.05,
                                       byte baseBright = 255);

//! Compute peak signal-to-noise ratio between two color images
/*! For color images, pSNR computed from the average
    mean-squared-error from the red, green and blue channels */
template <class T>
double pSNRcolor(const Image< PixRGB<T> >& img1,
                 const Image< PixRGB<T> >& img2);

//! Compute weighted peak signal-to-noise ratio between two color images
/*! This is like the other pSNRcolor() except that it uses the
    weighted version of distance() internally */
template <class T>
double pSNRcolor(const Image< PixRGB<T> >& img1,
                 const Image< PixRGB<T> >& img2,
                 const Image<float>& weight);

//! Normalize each component of the RGB image separately
/*! Normalize the red component with min.red() and max.red() etc.*/
template <class T>
Image< PixRGB<T> > normalizeRGB(const Image< PixRGB<T> >& img,
                                PixRGB<T> min,
                                PixRGB<T> max);

//! Return the pixel-wise maximum of the r, g and b component
template <class T>
Image<T> maxRGB(const Image< PixRGB<T> >& img);

//! Stain a black and white image with a color
/*! @param src the black and white image to be stained
    @param min This value will be converted to 0 (black)
    @param max This value will be converted to color
    @param color The color used for staining */
template <class T>
Image< PixRGB<T> > colorStain (const Image<T>& src,
                               const T& min, const T& max,
                               const PixRGB<T>& color);

//! convert RGB color values into C.I.E. coordinates
/*! @param rgbColor the color to convert from
    @param cr returns the R/I value
    @param cg returns the G/I value
    @param intens returns the intensity */
void RGBtoCIE(const PixRGB<byte>& rgbColor,
              float& cr, float& cg, float& intens);

//! distance between hue of image pixels and a given hue
/*! @param img The color image to compare
    @param muR the R/I coordinate of the target hue in the C.I.E. color space
    @param muG the G/I coordinate in C.I.E.
    @param sigR the standard deviation of a 2D Gaussian in R/I direction
    @param sigG the standard deviation of a 2D Gaussian in G/I direction
    @param rho the correlation between the R/I and G/I directions
    @return a float image coding for how close the hue in img is
    to the target hue - 1 for right on the mark, 0 for very different */
Image<float> hueDistance(const Image< PixRGB<byte> >& img,
                         float muR, float muG,
                         float sigR, float sigG, float rho);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_COLOROPS_H_DEFINED
