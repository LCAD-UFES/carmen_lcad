/*!@file Image/CutPaste.H Cut+paste operations from/to Image subregions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/CutPaste.H $
// $Id: CutPaste.H 14376 2011-01-11 02:44:34Z pez $
//

#ifndef IMAGE_CUTPASTE_H_DEFINED
#define IMAGE_CUTPASTE_H_DEFINED

#include <vector>

class Dims;
template <class T> class Point2D;
class Rectangle;
template <class T> class Image;

//! Horizontally concatenate two arrays of same height
template <class T_or_RGB>
Image<T_or_RGB> concatX(const Image<T_or_RGB>& left,
                        const Image<T_or_RGB>& right);

//! Vertically concatenate two arrays of same width
template <class T_or_RGB>
Image<T_or_RGB> concatY(const Image<T_or_RGB>& top,
                        const Image<T_or_RGB>& bottom);

//! Vertically concatenate topImg and bottomImg
/*! Like concatY(), but doesn't require the two images to have the
    same width. */
template <class T_or_RGB>
Image<T_or_RGB> concatLooseY(const Image<T_or_RGB>& topImg,
                             const Image<T_or_RGB>& bottomImg,
                             const T_or_RGB& bgColor = T_or_RGB());

//! Horizontally concatenate leftImg and rightImg
/*! Like concatX(), but doesn't require the two images to have the
    same height. */
template <class T_or_RGB>
Image<T_or_RGB> concatLooseX(const Image<T_or_RGB>& leftImg,
                             const Image<T_or_RGB>& rightImg,
                             const T_or_RGB& bgColor = T_or_RGB());

//! Extract a subimage from the input image.
/*! The upper left corner of the subimage is given by a Point2D<int>, and the
    size of the subimage is given by the Dims. If zerofill is false, then
    the rectangle given by Point2D<int>+Dims must be wholly contained within the
    bounds of the input image. If zerofill is true, then any output region
    that lies outside the bounds of the input will be zero-filled (where
    zero means the default value for type T). */
template <class T_or_RGB>
Image<T_or_RGB> crop(const Image<T_or_RGB>& src,
                     const Point2D<int>& upperLeft, const Dims& size,
                     const bool zerofill = false);

//! Overload of crop() using a Rectangle instead of a Point2D<int>+Dims.
template <class T_or_RGB>
Image<T_or_RGB> crop(const Image<T_or_RGB>& src,
                     const Rectangle& rect, const bool zerofill = false);

//! Shift an image by (int dx, int dy), with horizontal wraparound
template <class T_or_RGB>
Image<T_or_RGB> shift(const Image<T_or_RGB>& srcImg,
                      const int dx, const int dy);

//! Shift an image by (float dx, float dy)
template <class T_or_RGB>
Image<T_or_RGB> shiftImage(const Image<T_or_RGB>& srcImg,
                           const float dx, const float dy);

//! Shift an image by (int dx, int dy), without wraparound
template <class T_or_RGB>
Image<T_or_RGB> shiftClean(const Image<T_or_RGB>& srcImg,
                           const int dx, const int dy,
                           const T_or_RGB bgval = T_or_RGB());

//! Paste image into dst at given position
/*! Image must fit within dst and no resizing/interpolation is done.
    See embed() for a more sophisticated version with interpolation
    and resizing. */
template <class T_or_RGB>
void inplacePaste(Image<T_or_RGB>& dst,
                  const Image<T_or_RGB>& img, const Point2D<int>& pos);

//! Set all pixels within a given region to the specified value
/*! Does nothing if the rectangle is invalid. */
template <class T_or_RGB>
void inplaceClearRegion(Image<T_or_RGB>& dst,
                        const Rectangle& region, const T_or_RGB& val);

//! Embed image into dst with position & size given in rectangle
template <class T_or_RGB>
void inplaceEmbed(Image<T_or_RGB>& embed,
                  const Image<T_or_RGB>& img, const Rectangle& r,
                  const T_or_RGB background,
                  const bool keep_aspect = true);

//! Find the smallest bounding rectangle around pixels greater than the threshold
template <class T>
Rectangle findBoundingRect(const Image<T>& src, const T threshold);


//! Find the smallest bounding rectangle around the polygon
Rectangle findBoundingRect(const std::vector<Point2D<int> >& poly, const Dims imgDims);

#endif // !IMAGE_CUTPASTE_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
