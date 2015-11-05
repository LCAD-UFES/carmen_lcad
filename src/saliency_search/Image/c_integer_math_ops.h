/*!@file Image/c_integer_math_ops.h Fixed-point integer math versions of some of our floating-point Image functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/c_integer_math_ops.h $
// $Id: c_integer_math_ops.h 7408 2006-11-04 00:49:56Z rjpeters $
//

#ifndef IMAGE_C_INTEGER_MATH_OPS_H_DEFINED
#define IMAGE_C_INTEGER_MATH_OPS_H_DEFINED

#ifdef __cplusplus
extern "C"
{
#endif

  void c_intg_low_pass_5_x_dec_x_manybits(const int* src,
                                           const int w, const int h,
                                           int* dst,
                                           const int w2);

  void c_intg_low_pass_5_y_dec_y_manybits(const int* src,
                                           const int w, const int h,
                                           int* dst,
                                           const int h2);

  void c_intg_low_pass_5_x_dec_x_fewbits(const int* src,
                                           const int w, const int h,
                                           int* dst,
                                           const int w2);

  void c_intg_low_pass_5_y_dec_y_fewbits(const int* src,
                                           const int w, const int h,
                                           int* dst,
                                           const int h2);

  void c_intg_low_pass_5_x_dec_x_fewbits_optim(const int* src,
                                           const int w, const int h,
                                           int* dst,
                                           const int w2);

  void c_intg_low_pass_5_y_dec_y_fewbits_optim(const int* src,
                                           const int w, const int h,
                                           int* dst,
                                           const int h2);

  /// Convolve in the x direction with a 9-point binomial kernel
  /** The kernel is {1, 8, 28, 56, 70, 56, 28, 8, 1}, summing to 256

      @param src pointer to the source image, w*h row-major
      @param w width of the source and destination images
      @param h height of the source and destination images
      @param dst pointer to the destination image, w*h row-major

      Internally, this algorithm divides each of the source values by
      the sum of the filter coefficients, then multiplies by the
      coefficient and accumulates. Thus it is appropriate when the
      input has many precision bits and would overflow if multiplied
      by the fitler coefficient without dividing first, but is not
      appropriate when the input has few bits and would underflow as a
      result of the early division. For a 32-bit int, a reasonable
      cutoff point is 18 bits -- if the input has 18 bits or more, use
      this algorithm, otherwise use the fewbits functions (see
      below).
   */
  void c_intg_low_pass_9_x_manybits(const int* src,
                                     const int w, const int h,
                                     int* dst);

  /// Convolve in the y direction with a 9-point binomial kernel
  /** Like c_intg_low_pass_9_y_manybits(), but convolves in the y
      direction instead of the x direction. */
  void c_intg_low_pass_9_y_manybits(const int* src,
                                     const int w, const int h,
                                     int* dst);

  /// Convolve in the x direction with a 9-point binomial kernel
  /** The kernel is {1, 8, 28, 56, 70, 56, 28, 8, 1}, summing to 256

      @param src pointer to the source image, w*h row-major
      @param w width of the source and destination images
      @param h height of the source and destination images
      @param dst pointer to the destination image, w*h row-major

      Internally, this algorithm multiplies each input pixel by the
      corresponding filter coefficient, accumulates across the 9
      filter points, and then divides by the sum of the filter
      coefficients. Thus it is appropriate when the input relatively
      few precision bits giving room for doing the multiply/accumulate
      prior to the divide, but not when the input has many precision
      bits and would require an early division to avoid overflow. For
      a 32-bit int, a reasonable cutoff point is 18 bits -- if the
      input has fewer than 18 bits, use this algorithm, otherwise use
      the manybits functions (see above).
   */
  void c_intg_low_pass_9_x_fewbits(const int* src,
                                     const int w, const int h,
                                     int* dst);

  /// Convolve in the y direction with a 9-point binomial kernel
  /** Like c_intg_low_pass_9_y_fewbits(), but convolves in the y
      direction instead of the x direction. */
  void c_intg_low_pass_9_y_fewbits(const int* src,
                                     const int w, const int h,
                                     int* dst);

  /// Like c_intg_low_pass_9_x_fewbits() but uses optimized filter coefficients
  void c_intg_low_pass_9_x_fewbits_optim(const int* src,
                                         const int w, const int h,
                                         int* dst);

  /// Like c_intg_low_pass_9_y_fewbits() but uses optimized filter coefficients
  void c_intg_low_pass_9_y_fewbits_optim(const int* src,
                                         const int w, const int h,
                                         int* dst);

  /// function for reasonably large images
  void c_intg_x_filter_clean_manybits(const int* src,
                                       const int w, const int h,
                                       const int* hf_flipped, const int hfs,
                                       const int shiftbits,
                                       int* dst);

  /// function for reasonably large images
  void c_intg_x_filter_clean_fewbits(const int* src,
                                       const int w, const int h,
                                       const int* hf_flipped, const int hfs,
                                       const int shiftbits,
                                       int* dst);

  /// special function for very small images
  void c_intg_x_filter_clean_small_manybits(const int* src,
                                             const int w, const int h,
                                             const int* hf_flipped, const int hfs,
                                             const int shiftbits,
                                             int* dst);

  /// special function for very small images
  void c_intg_x_filter_clean_small_fewbits(const int* src,
                                             const int w, const int h,
                                             const int* hf_flipped, const int hfs,
                                             const int shiftbits,
                                             int* dst);

  /// function for reasonably large images
  void c_intg_y_filter_clean_manybits(const int* src,
                                       const int w, const int h,
                                       const int* vf_flipped, const int vfs,
                                       const int shiftbits,
                                       int* dst);

  /// function for reasonably large images
  void c_intg_y_filter_clean_fewbits(const int* src,
                                       const int w, const int h,
                                       const int* vf_flipped, const int vfs,
                                       const int shiftbits,
                                       int* dst);

  /// special function for very small images
  void c_intg_y_filter_clean_small_manybits(const int* src,
                                             const int w, const int h,
                                             const int* vf_flipped, const int vfs,
                                             const int shiftbits,
                                             int* dst);

  /// special function for very small images
  void c_intg_y_filter_clean_small_fewbits(const int* src,
                                             const int w, const int h,
                                             const int* vf_flipped, const int vfs,
                                             const int shiftbits,
                                             int* dst);

#ifdef __cplusplus
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_C_INTEGER_MATH_OPS_H_DEFINED
