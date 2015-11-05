/*!@file Image/PixelsInst.H Macros for instantiating classes with pixel types */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PixelsInst.H $
// $Id: PixelsInst.H 6010 2005-11-30 23:36:40Z rjpeters $
//

#ifndef PIXELSINST_H_DEFINED
#define PIXELSINST_H_DEFINED

/* NOTE:

   WHY IS THIS STUFF NOT IN PixelsBase.H or Pixels.H? Because only a
   few files actually need the PIX_INST macro, so by putting the macro
   here, we avoid having to recompile the world if the macro changes
   (e.g. to add/remove a pixel type from the macro).

*/

// ###################################################################
//! Instantiate classes with pixel types
/*! PIX_INST only includes the "common" pixel types. To also
  instantiate a class for less-common pixel types, use the
  PIX_INST_ALL macro instead (see below). See bottom of Image.C,
  ImageCache.C and ImageSet.C for usage.

  NOTE: if you move a pixel type from PIX_INST to PIX_INST_ALL, or
  vice versa, you'll probably need to make a corresponding change in
  the extract_templates.tcl script.
 */

#define PIX_INST(CLASS, TYPE)                   \
template class CLASS< TYPE >;                   \
template class CLASS< PixRGB<TYPE> >;


// ###################################################################
//! Instantiate classes with ALL known pixel types
/*! To instantiate a class with only "common" pixel types, use
  PIX_INST (see above) instead of PIX_INST_ALL.

  NOTE: if you move a pixel type from PIX_INST to PIX_INST_ALL, or
  vice versa, you'll probably need to make a corresponding change in
  the extract_templates.tcl script.
*/

#define PIX_INST_ALL(CLASS, TYPE)               \
template class CLASS< PixH2SV1<TYPE> >;         \
template class CLASS< PixH2SV2<TYPE> >;         \
template class CLASS< PixYIQ<TYPE> >;           \
template class CLASS< PixJpegYUV<TYPE> >;       \
template class CLASS< PixVideoYUV<TYPE> >;      \
template class CLASS< PixHSV<TYPE> >;           \
template class CLASS< PixH2SV3<TYPE> >;         \
template class CLASS< PixHyper<TYPE,3> >;       \
template class CLASS< PixHyper<TYPE,4> >;


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !PIXELSINST_H_DEFINED
