/*!@file Video/FfmpegFrame.H Conversions between ffmpeg's AVFrame and our VideoFrame */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/FfmpegFrame.H $
// $Id: FfmpegFrame.H 12785 2010-02-06 02:24:05Z irock $
//

#ifndef VIDEO_FFMPEGFRAME_H_DEFINED
#define VIDEO_FFMPEGFRAME_H_DEFINED

#ifdef INVT_HAVE_AVCODEC

#include "Util/Types.H" // for byte
#include "Video/VideoFormat.H"

#define attribute_deprecated
extern "C"
{
#ifdef HAVE_LIBAVCODEC_AVCODEC_H
#include <libavcodec/avcodec.h>
#else
#include <ffmpeg/avcodec.h>
#endif

}

class Dims;
class VideoFrame;
template <class T> class Image;
template <class T> class PixRGB;

/// Get the VideoFormat enum that corresponds to ffmpeg's PixelFormat enum
VideoFormat convertAVPixelFormatToVideoFormat(const PixelFormat fmt);

/// Make a VideoFrame from an AVFrame from ffmpeg.
/** The returned VideoFrame object will contain a private copy of the
    data from the AVFrame, so it is safe to reuse or discard the
    AVFrame object after calling this function. */
VideoFrame convertAVFrameToVideoFrame(const AVFrame* pic,
                                      const PixelFormat fmt,
                                      const Dims& dims);

/// Make a color Image from an AVFrame from ffmpeg.
Image<PixRGB<byte> > convertAVFrameToRGB(const AVFrame* pic,
                                         const PixelFormat fmt,
                                         const Dims& dims);

/// Fill an ffmpeg AVFrame from a VideoFrame
/** Returns true if there was a successful conversion, false otherwise
    -- conversion can fail if vidframe.getMode() doesn't match the
    requested PixelFormat; in that case your best bet is to just
    convert the VideoFrame to RGB with vidframe.toRgb() and then do
    your work in RGB mode instead. */
bool convertVideoFrameToAVFrame(const VideoFrame& vidframe,
                                const PixelFormat fmt,
                                AVFrame* pic);

//! Conversion from ffmpeg's PixelFormat enum to string
std::string convertToString(const PixelFormat fmt);

#endif // HAVE_FFMPEG_AVCODEC_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_FFMPEGFRAME_H_DEFINED
