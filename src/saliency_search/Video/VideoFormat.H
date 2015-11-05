/*!@file Video/VideoFormat.H definitions of possible image grabbing modes */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/VideoFormat.H $
// $Id: VideoFormat.H 12660 2010-01-26 02:14:54Z beobot $
//

#ifndef VIDEO_VIDEOFORMAT_H_DEFINED
#define VIDEO_VIDEOFORMAT_H_DEFINED

#include <string>

class Dims;

//! VideoFrame formats. Not all may be supported by various hardware.
/*! See VideoFrame.H for functions using those modes. If you add modes
    here, be sure to update convertToString() and convertFromString()
    in VideoFormat.C.

    See http://www.fourcc.org/ for useful descriptions of various YUV
    encoding formats.
*/
enum VideoFormat
  {
    VIDFMT_GREY    = 0,  // format: [ grey(8) ]
    VIDFMT_RAW     = 1,
    VIDFMT_RGB555  = 2,  // format: [ (1) r(5) g(5) b(5) ]
    VIDFMT_RGB565  = 3,  // format: [ r(5) g(6) b(5) ]
    VIDFMT_RGB24   = 4,  // format: [ r(8) g(8) b(8) ]
    VIDFMT_RGB32   = 5,  // format: [ r(8) g(8) b(8) ]
    VIDFMT_YUV24   = 6,  // formt:  [ Y(8) U(8) V(8) ] layout-compatible with PixVideoYUV<byte>
    VIDFMT_YUYV    = 7,  // format: [ Y0(8) U0(8) Y1(8) V0(8) ]
    VIDFMT_UYVY    = 8,  // format: [ U0(8) Y0(8) V0(8) Y1(8) ]
    VIDFMT_YUV444  = 9,  // format: [ U0(8) Y0(8) V0(8) U1(8) Y1(8) V1(8) ] (a.k.a. IYU2)
    VIDFMT_YUV422  = 10,  // format: same as VIDFMT_UYVY
    VIDFMT_YUV411  = 11, // format: [ U(8) Y0(8) Y1(8) V(8) Y2(8) Y3(8) ]
    VIDFMT_YUV420  = 12, // does this exist?
    VIDFMT_YUV410  = 13, // does this exist?
    VIDFMT_YUV444P = 14,
    VIDFMT_YUV422P = 15,
    VIDFMT_YUV411P = 16,
    VIDFMT_YUV420P = 17,
    VIDFMT_YUV410P = 18,
    VIDFMT_HM12    = 19,

    // format: bayer, last to letter represents the bayer format
    // for example, "GB" means the 1st row and first two componets are
    // green and blue value
    VIDFMT_BAYER_GB   = 20,
    VIDFMT_BAYER_BG   = 21,
    VIDFMT_BAYER_GR   = 22,
    VIDFMT_BAYER_RG   = 23,

    // 12 bits depth bayer mode
    VIDFMT_BAYER_GB12 = 24,
    VIDFMT_BAYER_BG12 = 25,
    VIDFMT_BAYER_GR12 = 26,
    VIDFMT_BAYER_RG12 = 27,
    VIDFMT_MJPEG      = 28,

    // KEEP THIS ITEM LAST:
    VIDFMT_AUTO    = 29  // auto selection of best mode
  };

//! Conversion from VideoFormat to string
/*! Use it, for example, if you got a grabmode as text from the
  command line and need to convert that to a VideoFormat value for the
  constructor. Text strings should match the symbolic names of the
  VideoFormat enum, minus the "VIDFMT_" prefix. */
std::string convertToString(const VideoFormat val);
//! Conversion from string to VideoFormat
/*! The names used in the definition of the enum VideoFormat
    (minus the "VIDFMT_" prefix) will be returned.

    The string is checked in a case-insensitive manner, so
    e.g. "YUV420P", "yuv420p", and "YuV420p" will all translate to
    VIDMFT_YUV420P.
*/
void convertFromString(const std::string& str, VideoFormat& val);

//! Check if the given mode is a simple packed pixel mode.
/*! If false, the mode might be planar, or might be a "complex" packed
  mode that has values represented by less than a full byte
  (e.g. VIDFMT_RGB555 or VIDFMT_RGB565). */
bool isSimplePackedMode(const VideoFormat vidformat);

//! Get the width of a scanline in the given video format.
/*! This returns some multiple of the image width for packed formats,
    and returns -1 for planar formats (indicating that the video frame
    storage is not just a series of scanlines for that format). */
int getScanlineWidth(const VideoFormat vidformat,
                     const int imgwidth);

//! Get the bytes-per-pixel for the given mode as a rational number (numer/denom)
void getBytesPerPixelForMode(const VideoFormat vidformat,
                             unsigned int* numer,
                             unsigned int* denom);

//! Get the size (in bytes) of a frame in the given video format.
/*! This will return a valid result for any video format, whether it
    is planar or packed. */
unsigned int getFrameSize(const VideoFormat vidformat,
                          const Dims& imgdims);

#endif // VIDEO_VIDEOFORMAT_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
