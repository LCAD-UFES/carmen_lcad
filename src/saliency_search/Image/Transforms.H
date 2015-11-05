/*!@file Image/Transforms.H Transformations on Image
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Transforms.H $
// $Id: Transforms.H 14592 2011-03-11 23:19:12Z jshen $
//

#ifndef IMAGE_TRANSFORMS_H_DEFINED
#define IMAGE_TRANSFORMS_H_DEFINED

#include "Util/Promotions.H"

template <class T> class Point2D;
class Dims;
class Rectangle;
template <class T> class Image;
template <class T> class ImageSet;

//! Segment object using an adaptive thresholding & region growing method.
/*! Returns an image that masks the object (see Itti et al, J Neuroimaging,
    2001) */
template <class T>
Image<byte> segmentObject(const Image<T>& src, const Point2D<int>& center);

//! Segment landmark using an adaptive thresholding & region growing method.
/*! Returns an image that masks the landmark
  @param activity is a measure of mean activity in the landmark
  @param standout is a measure of how much the landmark stands out from
  the rest of the image
  @param area is the percentage area of landmark
*/
template <class T>
int segmentLandmark(const Image<T>& src,
                    const Point2D<int>& seed, Image<byte>& target,
                    double& activity, double& standout,
                    float& area_percentage);

//! Compute 2D 4-neighbor contours of binary objects in image
/*! Points on the contour will be given value onval, and points not on the contour will have offval. */
template <class T>
Image<byte> contour2D(const Image<T>& src, const byte onval = 255, const byte offval = 0);

//! 3/4-chamfer distance transform; object = non 0
template <class T>
Image<T> chamfer34(const Image<T>& src, const T dmax = 255);

//! 3/4-chamfer saliency distance transform; object = non 0
//The distance transform (DT) labels each pixel with the distance to the closest
//feature pixel. We have extended this with the salience distance transform
//(Salience DT) by incorporating additional information reflecting the salience
//of the feature pixels. Full details are given in:
//
//    P.L. Rosin & G.A.W. West,
//    "Salience distance transforms",
//    CVGIP: Graphical Models and Image Processing,
//    vol 57, no. 6, pp. 483-521, 1995.

template <class T>
Image<T> saliencyChamfer34(const Image<T>& src, const T dmax = 255);

//! Computes 2D discrete cosine transform
/*! of square image patch size*size at offset (offx, offy) */
template <class T>
Image<typename promote_trait<T, float>::TP> dct(const Image<T>& src,
                                                const int offx,
                                                const int offy,
                                                const int size);

//! FFT-based information measure
template <class T>
float infoFFT(const Image<T>& src, const float eps, const Rectangle& rect);

//! FFT-based information measure
template <class T>
Image<float> infoFFT(const Image<T>& src, const float eps);

//! Examine local maxes in/out of targets and compute a learning weight
/*! Valid ranges for in_thresh and out_thresh are 0..255, plus: use -1
  for in_thresh to learn from the background only, or use -1 for
  out_thresh to learn from the target only. */
double learningCoeff(const Image<float>& featureMap,
                     const Image<byte>& targetDMap,
                     const bool softmask = false,
                     const int in_thresh = 20,
                     const int out_thresh = 255);

//! scale up an image using blocks
template <class T_or_RGB>
Image<T_or_RGB> scaleBlock(const Image<T_or_RGB>& src, const Dims newDims);

//! Floods object >= thresh with val from seed and returns area
/*! a clean version of Image::flood -
  problems solved: correct boundary checking, allow for thresh = 0.
  @param dest if dest is not initialized, it will be initialized to ZEROS.
  ATTENTION: If you want to have the old behavior of dest being initialized
  to src, you have to this explicitely before calling floodClean*/
template <class T>
int floodClean(const Image<T>& src, Image<T>& dest, const Point2D<int>& seed,
               const T thresh, const T val, int numdirs = 8);


//! same as floodClean, only finds the bounding box of the region in addition
template <class T>
int floodCleanBB(const Image<T>& src, Image<T>& dest, const Point2D<int>& seed,
                 const T thresh, const T val, Rectangle& bbox);

//! same as segmentObject, only with floodClean and different parameters
template <class T>
Image<byte> segmentObjectClean(const Image<T>& src,
                               const Point2D<int>& seed,
                               int numdirs = 8);

//! degrades a binary image mask s.t. it decays away from the FOA
template <class T>
Image<float> distDegrade(const Image<T>& src, const Point2D<int>& foa,
                         const float area);

//! set all values >= thresh to val in src and return the result
template <class T>
Image<T> highThresh(const Image<T>& src, const T thresh, const T val);

//! replace all values "from" with "to" and all other values with "other"
template <class T_or_RGB>
Image<T_or_RGB> replaceVals(const Image<T_or_RGB>& src,
                            const T_or_RGB from,
                            const T_or_RGB to,
                            const T_or_RGB other);

//! replace all values "from" with "to" and leave all other values in place
template <class T_or_RGB>
Image<T_or_RGB> replaceVals(const Image<T_or_RGB>& src,
                            const T_or_RGB from,
                            const T_or_RGB to);

//! Composite of the non-transparent pixels of fg on top of bg
template <class T_or_RGB>
Image<T_or_RGB> composite(const Image<T_or_RGB>& fg,
                          const Image<T_or_RGB>& bg,
                          const T_or_RGB transparent = T_or_RGB());

//! Create a mask of an object and transparent pixels in desired area
template <class T_or_RGB>
Image<T_or_RGB> createMask(const Image<T_or_RGB>& fg,
                           const Image<bool> mask,
                           const T_or_RGB transparent = T_or_RGB());

//! Mosaic mapping of transparent pixels of fg into multiple bg's
//! Equivalent to several simultaneous composite operations
template <class T_or_RGB>
Image<T_or_RGB> mosaic(const Image<T_or_RGB>& fg,
                       const Image<T_or_RGB>* bg,
                       const T_or_RGB* transvalues,
                       const uint Nimages);

//! apply a translucent layer (also called alpha blending)
template <class T_or_RGB>
Image<T_or_RGB> alphaBlend(const Image<T_or_RGB>& fg,
                           const Image<T_or_RGB>& bg,
                           const double alpha = 0.8,
                           const T_or_RGB transparent = T_or_RGB());

//! make a binary version of the source image
/*!@param src source image
  @param threshold the threshold used for binarization
  @param lowVal value for all points <= threshold
  @param highVal value for all points > threshold*/
template <class T>
Image<byte> makeBinary(const Image<T>& src,
                       const T& threshold,
                       const byte lowVal = byte(0),
                       const byte highVal = byte(255));

//! make a binary version of the source image
/*!@param src source image
  @param highThresh the threshold used for binarization
  @param lowThresh the threshold used for binarization
  @param lowVal value for all points <= threshold
  @param highVal value for all points > threshold*/
template <class T>
Image<byte> makeBinary2(const Image<T>& src,
                       const T& lowThresh,
                       const T& highThresh,
                       const byte lowVal = byte(0),
                       const byte highVal = byte(255));

//! paste foreground into background image at 
//! location as the upper-left corner in the background frame
/*!@param transparent pixels with this value are not replaced in background*/
template <class T_or_RGB>
void pasteImage(Image<T_or_RGB>& background,
                const Image<T_or_RGB>& foreground,
                const T_or_RGB& transparent,
                const Point2D<int> location,
                float opacity = 1.0F);

//! Paste a gabor into an image area. Account for overlap
/*! this is a simple extention to paste, but it tests for gabor
  overlap and attempts rectify similar to solution for MakeSnake
  by taking the pixel value furthest from the background value
  @param background This is the color of the image background
*/
template <class T>
void inplacePasteGabor(Image<T>& dst,
                       const Image<T>& gabor,
                       const Point2D<int>& pos,const T background);

//! Floods object >= thresh with val from seed and returns area
template <class T>
int flood(const Image<T>& src,
          Image<T>& dest, const Point2D<int>& seed,
          const T thresh, const T val);

//! How many particles >= thresh ?
template <class T>
int countParticles(const Image<T>& src, const T thresh);

//! Add background noise to image
/*! Range should be max value for any image
  of the type to which we add noise */
template <class T>
void inplaceAddBGnoise(Image<T>& dst, const float range);

//! Add background noise to image
/*! Range should be max value for any image
  of the type to which we add noise
  similar to addBGnoise, but eliminates BNG.. constant*/
template <class T>
void inplaceAddBGnoise2(Image<T>& dst, const float range);

// ######################################################################
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_TRANSFORMS_H_DEFINED
