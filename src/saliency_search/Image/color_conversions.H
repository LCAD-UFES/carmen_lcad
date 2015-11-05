/*!@file Image/color_conversions.H low-level routines for colorspace conversions */

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
// Primary maintainer for this file: Rob Peters <rjpeters@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/color_conversions.H $
// $Id: color_conversions.H 8279 2007-04-20 21:44:59Z rjpeters $
//

#ifndef IMAGE_COLOR_CONVERSIONS_H_DEFINED
#define IMAGE_COLOR_CONVERSIONS_H_DEFINED

#include "Util/Types.H"

template <class T> class Image;
template <class T> class PixRGB;

void yv12_to_rgb24_c(unsigned char* dst,
                     int dst_stride,
                     const unsigned char* y_src,
                     const unsigned char* u_src,
                     const unsigned char* v_src,
                     int y_stride,
                     int uv_stride,
                     int width,
                     int height);

void rgb24_to_yv12_c(const Image<PixRGB<byte> >& img,
                     byte* const y_out,
                     byte* u_out,
                     byte* v_out);

void yuv422p_to_rgb24_c(byte* dst,
                        const int w, const int h,
                        const byte* yptr,
                        const byte* uptr,
                        const byte* vptr);

void yuv422_to_rgb24_c(byte* dst,
                       const int w, const int h,
                       const byte* yuv422ptr,
                       const bool byteswap);

#endif // !IMAGE_COLOR_CONVERSIONS_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
