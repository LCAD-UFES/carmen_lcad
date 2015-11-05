/*!@file QtUtil/ImageConvert.H Functions to convert our images to QImage or QPixmap
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/QtUtil/ImageConvert4.H $
// $Id: ImageConvert4.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef IMAGE_CONVERT_QT_H_DEFINED
#define IMAGE_CONVERT_QT_H_DEFINED

#ifdef INVT_HAVE_QT4

#include "Util/Types.H" // for byte

#include <Qt/qpixmap.h>

template <class T> class Image;
template <class T> class PixRGB;

//! convert Image<PixRGB<byte> > to QPixmap
/*! NOTE that it's not practical to do a conversion to QImage, because
    QImage doesn't own its memory so we don't have a safe place to put
    the temporary storage -- Note: Rand thinks this is not so in Qt4 */
QPixmap convertToQPixmap4(const Image<PixRGB<byte> >& img);

//! convert Image<byte> to QPixmap
/*! NOTE that it's not practical to do a conversion to QImage, because
    QImage doesn't own its memory so we don't have a safe place to put
    the temporary storage -- Note: Rand thinks this is not so in Qt4 */
QPixmap convertToQPixmap4(const Image<byte>& img);

//! convert Image<PixRGB<byte> > to QImage
QImage convertToQImage4(const Image<PixRGB<byte> >& img);

//! convert QPixmap to Image<PixRGB<byte> >
/*! Warning: this is untested in Qt4 */
Image<PixRGB<byte> > convertToImage4(const QPixmap& qpixm);

#endif // INVT_HAVE_QT4

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif //IMAGE_CONVERT_QT_H_DEFINED
