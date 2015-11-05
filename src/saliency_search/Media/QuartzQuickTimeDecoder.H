/*!@file Media/QuartzQuickTimeDecoder.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/QuartzQuickTimeDecoder.H $
// $Id: QuartzQuickTimeDecoder.H 7959 2007-02-20 23:37:25Z rjpeters $
//

#ifndef MEDIA_QUARTZQUICKTIMEDECODER_H_DEFINED
#define MEDIA_QUARTZQUICKTIMEDECODER_H_DEFINED

#ifdef HAVE_QUICKTIME_QUICKTIME_H

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/MovieDecoder.H"
#include "Raster/GenericFrame.H"
#include "Util/Janitor.H"

#include <QuickTime/QuickTime.h>
#include <string>

/// Decode movie streams with QuickTime and using Quartz 2D for offscreen rendering
/** The older, now-deprecated, approach was to use QuickDraw (GWorld,
    etc.) for offscreen rendering, which can be found in QuickTimeDecoder */
class QuartzQuickTimeDecoder : public MovieDecoder
{
public:
  /// Construct a quartz quicktime movie from a filename, codec name, etc.
  QuartzQuickTimeDecoder(const char* fname);

  /// Destructor
  ~QuartzQuickTimeDecoder();

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

  GenericFrame readFrame();

private:
  QuartzQuickTimeDecoder(const QuartzQuickTimeDecoder&);
  QuartzQuickTimeDecoder& operator=(const QuartzQuickTimeDecoder&);

  static void imageAvailableCallback(QTVisualContextRef visualContext,
                                     const CVTimeStamp* timeStamp, void* refCon);

  Janitor<Movie>               itsMovie;
  Janitor<QTVisualContextRef>  itsVisualContext;
  Dims                         itsDims;
  GenericFrame                 itsFrame;
  int                          itsFrameNumber;
  TimeValue                    itsNextTime;
  bool                         itsNextFramePushback; // whether to re-use the current frame
  bool                         itsFirstFrame; // if we've read frame #1, then dims are valid
  std::string                  itsCallbackError;
};

#endif // HAVE_QUICKTIME_QUICKTIME_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_QUARTZQUICKTIMEDECODER_H_DEFINED
