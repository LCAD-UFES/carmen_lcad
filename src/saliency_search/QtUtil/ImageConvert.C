/*!@file QtUtil/ImageConvert.C Functions to convert our images to QImage or QPixmap
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/QtUtil/ImageConvert.C $
// $Id: ImageConvert.C 12424 2009-12-29 02:28:32Z rand $
//

#ifndef IMAGE_CONVERT_QT_C_DEFINED
#define IMAGE_CONVERT_QT_C_DEFINED

#ifdef INVT_HAVE_QT3

#include "QtUtil/ImageConvert.H"

#include "Image/Image.H"
#include "Image/Pixels.H"

#include <qimage.h>
#include <vector>

namespace
{
  bool isLittleEndian()
  {
    union
    {
      int i;
      byte b[sizeof(int)];
    } u;
    memset(&u, 0, sizeof(u));
    u.b[0] = 1;
    if (u.i == 1)
      return true;

    // else ...
    return false;
  }
}

// ######################################################################
QPixmap convertToQPixmap(const Image<PixRGB<byte> >& img)
{
  // allocate 32-bit buffer
  std::vector<uchar> buf(img.getSize()*4);
  uchar* dataptr = &buf[0];

  // copy data to buffer, max alpha
  Image<PixRGB<byte> >::const_iterator sptr = img.begin();
  Image<PixRGB<byte> >::const_iterator stop = img.end();

  if (isLittleEndian())
    while (sptr != stop)
      {
        *dataptr++ = (*sptr).p[2];
        *dataptr++ = (*sptr).p[1];
        *dataptr++ = (*sptr).p[0];
        *dataptr++ = 255;
        ++sptr;
      }
  else
    while (sptr != stop)
      {
        *dataptr++ = 255;
        *dataptr++ = (*sptr).p[0];
        *dataptr++ = (*sptr).p[1];
        *dataptr++ = (*sptr).p[2];
        ++sptr;
      }

  // create and return QImage
  QImage qimg(&buf[0], img.getWidth(), img.getHeight(), 32, 0, 0,
              QImage::IgnoreEndian);

  // NOTE! We can't just return the QImage directly here, like we used
  // to do -- http://doc.trolltech.com/3.3/qimage.html#QImage-7 states
  // that "The buffer must remain valid throughout the life of the
  // QImage". Therefore, if we return the QImage directly, its memory
  // becomes invalid because our temporary 'data' array goes out of
  // scope. This won't necessarily cause a crash, but it will mean
  // that some of our image data will get trashed as the stack
  // contents are overwritten by subsequent calls. So, instead we
  // return a QPixmap which contains its own private copy of the data.

  return QPixmap(qimg);
}

// ######################################################################
QPixmap convertToQPixmap(const Image<byte>& img)
{
  // allocate 32-bit buffer (can it be done with less bits per pixel??)
  std::vector<uchar> buf(img.getSize()*4);
  uchar* pdata = &buf[0];

  // copy data to buffer, max alpha
  Image<byte>::const_iterator sptr = img.begin();
  Image<byte>::const_iterator stop = img.end();

  if (isLittleEndian())
    while (sptr != stop)
      {
        *pdata++ = *sptr;
        *pdata++ = *sptr;
        *pdata++ = *sptr;
        *pdata++ = 255;
        ++sptr;
      }
  else
    while (sptr != stop)
      {
        *pdata++ = 255;
        *pdata++ = *sptr;
        *pdata++ = *sptr;
        *pdata++ = *sptr;
        ++sptr;
      }

  // create and return QImage
  QImage qimg(&buf[0], img.getWidth(), img.getHeight(), 32, 0, 0,
              QImage::IgnoreEndian);

  // NOTE! We can't just return the QImage directly here, like we used
  // to do -- http://doc.trolltech.com/3.3/qimage.html#QImage-7 states
  // that "The buffer must remain valid throughout the life of the
  // QImage". Therefore, if we return the QImage directly, its memory
  // becomes invalid because our temporary 'data' array goes out of
  // scope. This won't necessarily cause a crash, but it will mean
  // that some of our image data will get trashed as the stack
  // contents are overwritten by subsequent calls. So, instead we
  // return a QPixmap which contains its own private copy of the data.

  return QPixmap(qimg);
}

// ######################################################################
Image<PixRGB<byte> > convertToImage(const QPixmap& qpixm)
{
  QImage qimg;
  qimg = qpixm;

  Image<PixRGB<byte> > result(qimg.width(), qimg.height(), NO_INIT);

  Image<PixRGB<byte> >::iterator dptr = result.beginw();
  Image<PixRGB<byte> >::iterator stop = result.endw();

  for (int y = 0; y < qimg.height(); ++y)
    for (int x = 0; x < qimg.width(); ++x)
      {
        QRgb pix = qimg.pixel(x, y);
        *dptr++ = PixRGB<byte>((pix & 0x00ff0000) >> 16,
                               (pix & 0x0000ff00) >> 8,
                               (pix & 0x000000ff));
      }

  ASSERT(dptr == stop);

  return result;
}
#endif //IMAGE_CONVERT_QT_C_DEFINED
#endif //INVT_HAVE_QT3

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
