/*!@file Image/IO.H I/O operations for Image
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/IO.H $
// $Id: IO.H 7811 2007-01-26 23:38:43Z rjpeters $
//

#ifndef IMAGE_IO_H_DEFINED
#define IMAGE_IO_H_DEFINED

#include "Util/Types.H"

#include <iosfwd>

template <class T> class Image;

//! Write the pixel values of the Image in a nice human-readable format.
/*! <i>Caveat emptor</i>: it may only be "nice" if the img has a fairly
    small number of elements, so that one row of the image fits in one row
    of text. This format is <b>not</b> intended to generate a
    representation that can be re-parsed by the computer to recreate the
    original image. */
template <class T_or_RGB>
std::ostream& operator<<(std::ostream& os, const Image<T_or_RGB>& img);

//! Write img to to "os", one line of the image into one line of text
/*! This is only useful for rather small images!*/
template <class T>
void writeImageToStream(std::ostream& os, const Image<T>& img);

//! Read img from "is" which has been written with writeImageToStream before
template <class T>
void readImageFromStream(std::istream& is, Image<T>& img);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_IO_H_DEFINED
