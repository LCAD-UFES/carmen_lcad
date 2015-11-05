/*!@file Image/integer_math.h */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/integer_math.h $
// $Id: integer_math.h 7408 2006-11-04 00:49:56Z rjpeters $
//

#ifndef IMAGE_INTEGER_MATH_H_DEFINED
#define IMAGE_INTEGER_MATH_H_DEFINED

struct integer_math
{

  unsigned int nbits;

  void (*low_pass_5_x_dec_x_manybits)(const int* src,
                                       const int w, const int h,
                                       int* dst,
                                       const int w2);

  void (*low_pass_5_y_dec_y_manybits)(const int* src,
                                       const int w, const int h,
                                       int* dst,
                                       const int h2);

  void (*low_pass_5_x_dec_x_fewbits)(const int* src,
                                       const int w, const int h,
                                       int* dst,
                                       const int w2);

  void (*low_pass_5_y_dec_y_fewbits)(const int* src,
                                       const int w, const int h,
                                       int* dst,
                                       const int h2);

  void (*low_pass_9_x_manybits)(const int* src,
                                 const int w, const int h,
                                 int* dst);

  void (*low_pass_9_y_manybits)(const int* src,
                                 const int w, const int h,
                                 int* dst);

  void (*low_pass_9_x_fewbits)(const int* src,
                                 const int w, const int h,
                                 int* dst);

  void (*low_pass_9_y_fewbits)(const int* src,
                                 const int w, const int h,
                                 int* dst);

  void (*x_filter_clean_manybits)(const int* src,
                                   const int w, const int h,
                                   const int* hf_flipped, const int hfs,
                                   const int shiftbits,
                                   int* dst);

  void (*x_filter_clean_fewbits)(const int* src,
                                   const int w, const int h,
                                   const int* hf_flipped, const int hfs,
                                   const int shiftbits,
                                   int* dst);

  void (*x_filter_clean_small_manybits)(const int* src,
                                         const int w, const int h,
                                         const int* hf_flipped, const int hfs,
                                         const int shiftbits,
                                         int* dst);

  void (*x_filter_clean_small_fewbits)(const int* src,
                                         const int w, const int h,
                                         const int* hf_flipped, const int hfs,
                                         const int shiftbits,
                                         int* dst);

  void (*y_filter_clean_manybits)(const int* src,
                                   const int w, const int h,
                                   const int* vf_flipped, const int vfs,
                                   const int shiftbits,
                                   int* dst);

  void (*y_filter_clean_fewbits)(const int* src,
                                   const int w, const int h,
                                   const int* vf_flipped, const int vfs,
                                   const int shiftbits,
                                   int* dst);

  void (*y_filter_clean_small_manybits)(const int* src,
                                         const int w, const int h,
                                         const int* vf_flipped, const int vfs,
                                         const int shiftbits,
                                         int* dst);

  void (*y_filter_clean_small_fewbits)(const int* src,
                                         const int w, const int h,
                                         const int* vf_flipped, const int vfs,
                                         const int shiftbits,
                                         int* dst);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_INTEGER_MATH_H_DEFINED
