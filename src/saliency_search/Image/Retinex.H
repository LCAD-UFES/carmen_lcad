/*!@file Image/Retinex.H Retinex color-correction algorithm */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Retinex.H $
// $Id: Retinex.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_RETINEX_H_DEFINED
#define IMAGE_RETINEX_H_DEFINED

#include "Image/Image.H"

template <class T> class ImageSet;

/// Get the recommended pyramid depth for the given input dims
size_t retinexDepth(const Dims& dims);

/// Build a retinex pyramid of the given depth using niter iterations
/** The input is expected to be in log scale, and the output will also
    be in log scale.

    This implementation is based on "McCann99" algorithm described in

    Brian Funt, Florian Ciurea, and John McCann "Retinex in Matlab,"
    Proceedings of the IS&T/SID Eighth Color Imaging Conference: Color
    Science, Systems and Applications, 2000, pp 112-121

    The paper and matlab code are published here:

    http://www.cs.sfu.ca/~colour/publications/IST-2000/
    http://www.cs.sfu.ca/~colour/publications/IST-2000/retinex_mccann99.m
 */
template <class T>
ImageSet<T> buildPyrRetinexLog(const Image<T>& L,
                               const size_t depth, const int niter,
                               const Rectangle& outrect);

/// Like above but with the output rectangle set to the full size of L
template <class T>
ImageSet<T> buildPyrRetinexLog(const Image<T>& L,
                               const size_t depth, const int niter)
{
  return buildPyrRetinexLog(L, depth, niter,
                            Rectangle(Point2D<int>(0,0), L.getDims()));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_RETINEX_H_DEFINED
