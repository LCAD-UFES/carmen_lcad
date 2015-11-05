/*!@file Video/RgbConversion.H Raw conversion between video formats and RGB images */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/RgbConversion.H $
// $Id: RgbConversion.H 15147 2012-02-06 23:22:31Z kai $
//

#ifndef VIDEO_RGBCONVERSION_H_DEFINED
#define VIDEO_RGBCONVERSION_H_DEFINED

#include "Util/Types.H" // for byte
#include "Raster/DeBayer.H"

#include <cstdlib> // for size_t

class Dims;
template <class T> class Image;
template <class T> class PixRGB;

//! Build an RGB image from an RGB C array, with optional byte-swapping
/*! If byte-swapping is off, this will just perform a deep copy of the
    data; this function is somewhat redundant with the Image
    constructor and is provided for completeness with respect to the
    fromXXX methods only. Note that data should have
    3*dims.w()*dims.h() elements, and it is the responsibility of the
    caller to free that data later. */
template <class T>
Image<PixRGB<T> > fromRGB(const T* data, const size_t length,
                          const Dims& dims, const bool byteswap);

//! Build an RGB image from a alpha+RGB C array, with optional byte-swapping
template <class T>
Image<PixRGB<T> > fromARGB(const T* data, const size_t length,
                           const Dims& dims, const bool byteswap);

//! Build an RGB image from an RGB555 C array, with optional byte-swapping
Image<PixRGB<byte> > fromRGB555(const byte* data, const size_t length,
                                const Dims& dims, const bool byteswap);

//! Build an RGB image from an RGB565 C array, with optional byte-swapping
Image<PixRGB<byte> > fromRGB565(const byte* data, const size_t length,
                                const Dims& dims, const bool byteswap);

//! Build an RGB image from a YUV24 C array, with optional byte-swapping
template <class T>
Image<PixRGB<T> > fromVideoYUV24(const T* data, const size_t length,
                                 const Dims& dims, const bool byteswap);

//! Build an RGB image from a YUV444 C array, with optional byte-swapping
template <class T>
Image<PixRGB<T> > fromVideoYUV444(const T* data, const size_t length,
                                  const Dims& dims, const bool byteswap);

//! Build an RGB image from a YUV422 C array, with optional byte-swapping
template <class T>
Image<PixRGB<T> > fromVideoYUV422(const T* data, const size_t length,
                                  const Dims& dims, const bool byteswap);

//! Build an RGB image from a YUV411 C array, with optional byte-swapping
template <class T>
Image<PixRGB<T> > fromVideoYUV411(const T* data, const size_t length,
                                  const Dims& dims, const bool byteswap);

//! Build an RGB image from a YUV422-planar C array
template <class T>
Image<PixRGB<T> > fromVideoYUV444P(const T* data, const size_t length,
                                   const Dims& dims);

//! Build an RGB image from a YUV422-planar C array
template <class T>
Image<PixRGB<T> > fromVideoYUV422P(const T* data, const size_t length,
                                   const Dims& dims);

//! Build an RGB image from a YUV411-planar C array
template <class T>
Image<PixRGB<T> > fromVideoYUV411P(const T* data, const size_t length,
                                   const Dims& dims);

//! Build an RGB image from a YUV420-planar C array
template <class T>
Image<PixRGB<T> > fromVideoYUV420P(const T* data, const size_t length,
                                   const Dims& dims);

//! Build an RGB image from a HM12-planar C array
template <class T>
Image<PixRGB<T> > fromVideoHM12(const T* data, const size_t length,
                                   const Dims& dims);

//! Build an RGB image from a motion compressed jpeg
template <class T>
Image<PixRGB<T> > fromVideoMJPEG(const T* data, const size_t length,
                                   const Dims& dims, const bool byteswap);

//! Build an RGB image from a YUV410-planar C array
template <class T>
Image<PixRGB<T> > fromVideoYUV410P(const T* data, const size_t length,
                                   const Dims& dims);

//! Build an RGB image from a grayscale C array of size dims.w()*dims.h()
template <class T>
Image<PixRGB<T> > fromMono(const T* data, const size_t length,
                           const Dims& dims);

//! Build an RGB image from a bayer C array
Image<PixRGB<byte> > fromBayer(const byte* data, const size_t length,
                              const Dims& dims, BayerFormat ft);

Image<PixRGB<uint16> > fromBayerU16(const uint16* data, const size_t length,
                              const Dims& dims, BayerFormat ft);

//! Fill pre-allocated YUV422 C arrays with values from an image
void toVideoYUV422(const Image<PixRGB<byte> > img,
                   byte* y, byte* u, byte* v,
                   const int ystride = 0,
                   const int ustride = 0,
                   const int vstride = 0);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_RGBCONVERSION_H_DEFINED
