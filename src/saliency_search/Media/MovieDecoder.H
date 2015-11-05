/*!@file Media/MovieDecoder.H Abstract base class for low-level movie decoders */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MovieDecoder.H $
// $Id: MovieDecoder.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef MEDIA_MOVIEDECODER_H_DEFINED
#define MEDIA_MOVIEDECODER_H_DEFINED

#include "Util/Types.H" // for byte

class Dims;
class GenericFrameSpec;
class VideoFrame;
template <class T> class Image;
template <class T> class PixRGB;

/// Abstract base class for low-level movie decoders
class MovieDecoder
{
public:
  /// Virtual destructor for safe inheritance
  virtual ~MovieDecoder();

  /// Seek to the given frame number
  /** The new frame number must be greater than or equal to the
      current frame number (i.e., we can't seek backwards). Default
      implementation just calls readAndDiscardFrame() until
      apparentFrameNumber() == n.

      @return Whether the frame number was succesfully set to the
      requested value. The function may fail if the requested number
      is greater than the number of frames in the movie, for example.
  */
  virtual bool setFrameNumber(int n);

  /// Get the "apparent" frame number
  /** This function takes into account whether or not the next frame
      will be a push-back frame */
  virtual int apparentFrameNumber() const = 0;

  /// Get the specifications of the movie frames
  virtual GenericFrameSpec peekFrameSpec() = 0;

  /// Get the next raw VideoFrame from the stream
  virtual VideoFrame readVideoFrame() = 0;

  /// Read an RGB image from the stream
  virtual Image<PixRGB<byte> > readRGB() = 0;

  /// Read a frame from the stream and discard it
  /** If you know you are going to discard the frame (e.g. to skip
      ahead to a certain frame number, or to count the frame), then it
      is more efficient to call readAndDiscardFrame() than to call
      readVideoFrame() or readRGB() but ignore the result. */
  virtual bool readAndDiscardFrame() = 0;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MOVIEDECODER_H_DEFINED
