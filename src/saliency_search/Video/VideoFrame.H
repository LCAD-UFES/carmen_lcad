/*!@file Video/VideoFrame.H Handle generic video frames in a multitude of formats */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/VideoFrame.H $
// $Id: VideoFrame.H 10348 2008-10-15 22:45:29Z ilab24 $
//

#ifndef VIDEO_VIDEOFRAME_H_DEFINED
#define VIDEO_VIDEOFRAME_H_DEFINED

#include "Image/ArrayData.H"
#include "Image/Dims.H"
#include "Util/Types.H"
#include "Video/VideoFormat.H"
#include "rutz/scopedptr.h"

#include <iosfwd>
#include <string>

template <class T> class Image;
template <class T> class PixRGB;
template <class T> class PixVideoYUV;

/// Simple class that encpasulates information about a video frame.
/** This includes information about the format in which the frame is
    encoded, its size, and of course the raw framebuffer data
    itself.

    NOTE: The purpose of this class is to take the place of just
    passing raw buffer pointers around; instead, we wrap the buffer
    pointer along with associated descriptive information. However,
    BEWARE that we have no strong safeguards to ensure that the
    internal buffer pointer stays "live" for very long. If you want to
    keep a VideoFrame object around for a "long" time (say, holding it
    for longer than the function call in which you received it in the
    first place), then you should call deepCopyOf() on it to get a
    safe VideoFrame; that way the VideoFrame will make an internal
    private copy of the frame buffer which can live as long as the
    object lives without becoming corrupted.
*/
class VideoFrame
{
public:
  /// Default constructor; be sure to initialize before use, though!
  VideoFrame();

  /// Construct by borrowing 'data' buffer.
  /** The caller is responsible for ensuring that 'data' points to
      valid memory for as long as the VideoFrame object lives, and for
      freeing/deleting 'data' afterwards. If you need to hold on to
      the frame for a long time, then call deepCopyOf() to get a
      VideoFrame that can be safely held. */
  VideoFrame(const byte* data, const size_t length, const Dims& dims,
             const VideoFormat mode, const bool byteswap,
             const bool strictLength);

  /// Construct with all information needed to convert to RGB, save to disk, etc.
  VideoFrame(const ArrayHandle<byte>& hdl, const Dims& dims,
             const VideoFormat mode, const bool byteswap);

  /// Construct from a grayscale image (using VIDFMT_GREY)
  explicit VideoFrame(const Image<byte>& gray);

  /// Construct from an RGB image (using VIDFMT_RGB24)
  explicit VideoFrame(const Image<PixRGB<byte> >& rgb);

  /// Construct from a VideoYUV image (using VIDFMT_YUV24)
  explicit VideoFrame(const Image<PixVideoYUV<byte> >& yuv);

  /// Destructor
  ~VideoFrame();

  /// Copy constructor
  VideoFrame(const VideoFrame& that);

  /// Assignment operator
  VideoFrame& operator=(const VideoFrame& that);

  /// Pseudo-constructor that builds a VideoFrame from a raw file
  /** @param strictLength if true, then we will throw an error if the
      data length of the file is larger the length that would be
      expected based on the given Dims and VideoFormat; if the file
      length is too small, we will unconditionally throw an error
      regardless of whether strictLength is true */
  static VideoFrame fromFile(const char* fname, const Dims& dims,
                             const VideoFormat mode, const bool byteswap,
                             const bool strictLength = false);

  /// Pseudo-constructor that builds a VideoFrame from an input stream
  static VideoFrame fromStream(std::istream& strm, const Dims& dims,
                               const VideoFormat mode, const bool byteswap,
                               const bool fail_is_fatal = true);

  /// Make a new VideoFrame object with an internal private copy of the frame data
  /** This allows the returned VideoFrame object to be long-lived (on
      the other hand, the original VideoFrame's buffer is prone to
      being eventually overwritten by the frame grabber or movie
      stream that originally produced it).
  */
  static VideoFrame deepCopyOf(const VideoFrame& original);

  /// Check if we have any data
  bool initialized() const { return itsDims.isNonEmpty(); }

  /// Get the raw buffer pointer
  const byte* getBuffer() const { return itsData; }

  /// Get the length of the buffer
  size_t getBufSize() const;

  /// Get the dimensions of the frame
  const Dims& getDims() const { return itsDims; }

  /// Get the video mode
  VideoFormat getMode() const { return itsMode; }

  /// Get the byteswap state
  bool getByteSwap() const { return itsByteSwap; }

  /// Deinterlace the frame using the "bob" method
  /** @param in_bottom_field determines whether to return the rescaled
      top field or rescaled bottom field */
  VideoFrame makeBobDeinterlaced(int in_bottom_field) const;

  /// Flip the frame in the horizontal direction
  /** For now, this only works with VIDFMT_YUV420P frames (which are
      typically read from ffmpeg and displayed in SDL). */
  VideoFrame getFlippedHoriz() const;

  /// Dump the frame to a disk file using mmap()
  /** A filename extension appropriate to the frame's VideoFormat
      will be added to the given filename stem.

      Returns the full filename of the file that was actually written.

      @param flush if true, then do an fsync() on the file before returning
  */
  std::string diskDumpMmap(const char* fstem, bool flush = false) const;

  /// Dump the frame to a disk file using posix io functions
  /** A filename extension appropriate to the frame's VideoFormat
      will be added to the given filename stem.

      Returns the full filename of the file that was actually written.

      @param flush if true, then do an fsync() on the file before returning
  */
  std::string diskDumpStdio(const char* fstem, bool flush = false) const;

  /// Convert the buffer to RGB, according to its video mode
  Image<PixRGB<byte> > toRgb() const;

  // Conver the 16 bit format type buffer to RGB, according to its video mode
  Image<PixRGB<uint16> > toRgbU16() const;

  /// Extract individual Y/U/V component images from the buffer, if possible
  /** This function will LFATAL() if the VideoFrame data is in a
      non-YUV mode (such as RGB or grayscale), so you need to check
      the data mode is appropriate before trying to call this
      function.

      Depending on the data mode, the individual component images may
      not be all the same size; in particular, the y component is
      likely to be at the full image size, while the u and v
      components may be downscaled by a factor of 2 or 4. So, you will
      need to check the Dims of the result images before doing further
      processing with them.
  */
  void toYuvComponents(Image<byte>& y,
                       Image<byte>& u,
                       Image<byte>& v) const;

  /// Abstract base class for various storage back-ends
  class Storage
  {
  public:
    /// Virtual destructor for safe inheritance
    virtual ~Storage();

    virtual Storage* clone() const = 0;
  };

private:
  rutz::scoped_ptr<Storage> itsStorage;
  const byte*       itsData;
  size_t            itsDataLength;
  Dims              itsDims;
  VideoFormat       itsMode;
  bool              itsByteSwap;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_VIDEOFRAME_H_DEFINED
