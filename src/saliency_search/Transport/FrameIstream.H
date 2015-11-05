/*!@file Transport/FrameIstream.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameIstream.H $
// $Id: FrameIstream.H 13273 2010-04-21 22:08:00Z rand $
//

#ifndef TRANSPORT_FRAMEISTREAM_H_DEFINED
#define TRANSPORT_FRAMEISTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Util/Types.H" // for byte

class Dims;
class GenericFrame;
class GenericFrameSpec;
class SimTime;
class FrameRange;
template <class T> class Image;
template <class T> class PixRGB;

// ######################################################################
/// Listener class that can get called each time a FrameIstream grabs a frame
/** Different FrameIstream subclasses may use their FrameListener
    differently, or possibly even ignore the FrameListener. */
class FrameListener
{
public:
  virtual ~FrameListener();

  virtual void onRawFrame(const GenericFrame& frame) = 0;
};

//! Abstract interface class representing a source of Image frames
/*! Concrete classes might implement this interface so that the real
    source is a series of bitmap files (RasterInputSeries), or a movie
    file (InputMPEGSeries), or an external device (V4Lgrabber,
    IEEE1394grabber).

    See also FrameOstream for the analogous output interface. */
class FrameIstream : public ModelComponent
{
public:
  //! Constructor
  FrameIstream(OptionManager& mgr,
               const std::string& descrName,
               const std::string& tagName);

  //! Virtual destructor
  virtual ~FrameIstream();

  //! Configure the FrameIstream object in a type-dependent manner
  /*! The string is expected to be some piece of user input (e.g. from
      the command-line) specifying key information for the
      FrameIstream (e.g., the filename for an input movie).

      The default implementation does nothing; derived classes can
      override if they need to receive such user input.
  */
  virtual void setConfigInfo(const std::string& cfg);

  /// Install a FrameListener; default implementation does nothing
  /** Subclasses may override this method if they want to actually
      pass some information to the listener. */
  virtual void setListener(rutz::shared_ptr<FrameListener> listener);

  //! Advise the FrameIstream object of the current frame number
  /*! NOTE: the default implementation does nothing -- it just ignores
      the frame number. This is allowed for subclasses, too, since for
      certain input formats (e.g., framegrabbers) it doesn't make any
      sense to specify a frame number, since the format doesn't
      support random-access reading. On the other hand, certain
      subclasses require a frame number to function properly (e.g.,
      for writing a series of consecutively-numbered raster
      files). So, the bottom line is: clients of FrameIstream must be
      sure to call setFrameNumber(), but should make no assumptions
      about what it will actually do.

      @return Whether the frame number was succesfully set as
      requested. The function may fail, for instance, in the case of
      an input source like a movie file that has only a limited number
      of frames.
  */
  virtual bool setFrameNumber(int n);

  //! Return the specifications of the next frame
  /*! Subclasses may have to load the next frame in order to peek at
      the dimensions, then cache that frame and return it the next
      time readRGB(), readGray(), or readFloat() is called. */
  virtual GenericFrameSpec peekFrameSpec() = 0;

  //! Get the natural inter-frame time for this frame source
  /*! A return value of 0 means that there is no particular natural
      inter-frame time; the default implementation returns 0 so
      subclasses should override if they need to specify a different
      inter-frame time.

      This information should be treated as advisory only; for
      instance, callers might use this information to try to display
      frames to the user at the "natural" framerate.
  */
  virtual SimTime getNaturalFrameTime() const;

  //! Convenience function to get the frame dims from peekFrameSpec()
  Dims peekDims();

  //! Convenience function to get the frame width from peekFrameSpec()
  int getWidth();

  //! Convenience function to get the frame height from peekFrameSpec()
  int getHeight();

  //! Optional call to efficiently prepare for frame streaming
  /*! Some FrameIstream subclasses may require some preparatory work
      before they can start returning image frames, and this function
      lets that work be done outside of the application's main
      frame-reading loop.

      A good example of this is streaming-mode frame-grabbers, such as
      the video4linux driver. To use that driver in streaming mode,
      each of the internal frame buffers must be started, but they
      shouldn't be started "too soon" either; so, it's best to call
      startStream() just before the main loops starts calling
      readFrame() (or readRGB(), etc.).

      Subclass authors should design their classes so that readFrame()
      always "just works", regardless of whether the user has called
      startStream() or not, typically by keeping a flag to mark
      whether startStream() has been called, and then calling
      startStream() from within the first readFrame() call, if
      needed.

      The default implementation of startStream() is a no-op.
  */
  virtual void startStream();

  //! Read a frame from the input source
  /*! The actual input source could be a raster file, a movie file, or
      some external device, etc., depending on the concrete subclass. */
  virtual GenericFrame readFrame() = 0;

  //! Read an image from the input source
  /*! The actual input source could be a raster file, a movie file, or
      some external device, etc., depending on the concrete subclass.

      The default implementation just returns readFrame().asRgb(). */
  virtual Image<PixRGB<byte> > readRGB();

  //! the specific 12 bit depth (actually any uint16 data type) implemtation
  //! return from readFrame().asRgb16().
  virtual Image<PixRGB<uint16> > readRGBU16();

  //! Read an image from the input source
  /*! The actual input source could be a raster file, a movie file, or
      some external device, etc., depending on the concrete subclass.

      The default implementation just returns readFrame().asGray(). */
  virtual Image<byte> readGray();

  //! the specific 12 bit depth (actually any uint16 data type) implemtation
  //! return from readFrame().asRgb16().
  virtual Image<uint16> readGrayU16();

  //! Read an image from the input source
  /*! The actual input source could be a raster file, a movie file, or
      some external device, etc., depending on the concrete subclass.

      The default implementation just returns readFrame().asFloat(). */
  virtual Image<float> readFloat();

  //! Read a frame from the stream and discard it
  /*! Subclasses may be able to implement this function more
      efficiently than the other functions that actually return an
      Image object (since they might avoid converting from some raw
      representation to Image). So, if you know you are going to
      discard the frame (e.g. to skip ahead to a certain frame number,
      or to count the frame), then it is more efficient to call
      readAndDiscardFrame() than to call readVideoFrame() or readRGB()
      but ignore the result.

      The return value will be true if a frame was actually read, or
      false if no frame was found (e.g., end-of-stream was reached).

      The default implementation just calls readRGB() but ignores the
      result.
  */
  virtual bool readAndDiscardFrame();

  //! Return whether or not this stream supports seek operations
  /*! This function should be reimplemented in subclasses to let
      users know whether or not random frame access is supported.
      The default implementation returns false.
  */
  virtual bool supportsSeek();

  //! Return the frame range for this particular input stream. 
  /*! This is particuarly applicable for FrameIstreams dealing with video files
      and image directories, in which case it may be easier to get this value
      from the underlying FrameIstream, rather than the command line. The default
      behavior is to just return the default empty FrameRange, but should be overloaded
      for stream sources which can return sensical ranges.
   */
  virtual FrameRange getFrameRange();
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_FRAMEISTREAM_H_DEFINED
