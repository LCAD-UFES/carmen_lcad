/*!@file Media/FfmpegPacketDecoder.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FfmpegPacketDecoder.H $
// $Id: FfmpegPacketDecoder.H 12374 2009-12-22 16:38:48Z rand $
//

#ifndef MEDIA_FFMPEGPACKETDECODER_H_DEFINED
#define MEDIA_FFMPEGPACKETDECODER_H_DEFINED

#ifdef INVT_HAVE_AVCODEC

#include "Media/MovieDecoder.H"

#include <deque>
#define attribute_deprecated
extern "C"
{
#if defined (HAVE_FFMPEG_AVCODEC_H)
#include <ffmpeg/avcodec.h>
#elif defined(HAVE_LIBAVCODEC_AVCODEC_H)
#include <libavcodec/avcodec.h>
#endif

#if defined (HAVE_FFMPEG_AVFORMAT_H)
#include <ffmpeg/avformat.h>
#elif defined(HAVE_LIBAVFORMAT_AVFORMAT_H)
#include <libavformat/avformat.h>
#endif
}
#include <string>
#include <vector>

/// Decode a movie using ffmpeg's libavformat packet streaming API
class FfmpegPacketDecoder : public MovieDecoder
{
public:

  /// Construct from a given filename
  FfmpegPacketDecoder(const char* fname, const bool preload);

  /// Virtual destructor for safe inheritance
  ~FfmpegPacketDecoder();

  /// Get the "apparent" frame number
  /** This function takes into account whether or not the next frame
      will be a push-back frame */
  virtual int apparentFrameNumber() const;

  /// Get the specifications of the movie frames
  virtual GenericFrameSpec peekFrameSpec();

  /// Get the next raw VideoFrame from the stream
  virtual VideoFrame readVideoFrame();

  /// Read an RGB image from the stream
  virtual Image<PixRGB<byte> > readRGB();

  /// Read a frame from the stream and discard it
  /** If you know you are going to discard the frame (e.g. to skip
      ahead to a certain frame number, or to count the frame), then it
      is more efficient to call readAndDiscardFrame() than to call
      readVideoFrame() or readRGB() but ignore the result. */
  virtual bool readAndDiscardFrame();

private:
  /// Try to read another AVPacket and add it to our queue; return true if successful
  bool getNextPacket();

  /// Read a raw frame from the stream
  /** NOTE: This function used to be public, but now we have
      readVideoFrame() to serve that purpose instead (returns a raw
      YUV frame, but with additional contextual information). If it
      seems that you need to make readRawFrame() public again, that
      wouldn't be the end of the world, but first check if you could
      achieve what you need with an appropriate extension to the
      VideoFrame class. */
  AVFrame* readRawFrame();

  std::string itsFilename;

  AVFormatContext* itsFormatContext;
  AVCodecContext* itsCodecContext;

  int itsStreamID;

  AVFrame itsPicture;

  int itsFrameNumber;
  bool itsDimsValid;
  bool itsNextFramePushback;
  bool itsPacketsExhausted;

  std::deque<AVPacket> itsPacketQ;
  std::vector<AVPacket> itsPacketTrashQ;
};

#endif // INVT_HAVE_AVCODEC

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_FFMPEGPACKETDECODER_H_DEFINED
