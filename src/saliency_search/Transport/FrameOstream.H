/*!@file Transport/FrameOstream.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameOstream.H $
// $Id: FrameOstream.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef FRAMEOSTREAM_H_DEFINED
#define FRAMEOSTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Image/Normalize.H" // for float normalization flags
#include "Util/Types.H" // for byte

template <class T> class Image;
template <class T> class Layout;
template <class T> class PixRGB;

class FrameInfo;
class GenericFrame;

//! Abstract interface class representing a destination for Image frames
/*! Concrete classes might implement this interface so that the real
    final destination is either a series of bitmap files
    (e.g. RasterOutputSeries), or an mpeg-encoded movie stream
    (OutputMPEGStream), or an on-screen window (ImageDisplayStream),
    or a composite of destinations (OutputFrameSeries).

    See also FrameIstream for the analogous input interface. */
class FrameOstream : public ModelComponent
{
public:
  //! Constructor
  FrameOstream(OptionManager& mgr,
               const std::string& descrName,
               const std::string& tagName);

  //! Virtual destructor
  virtual ~FrameOstream();

  //! Configure the FrameOstream object in a type-dependent manner
  /*! The string is expected to be some piece of user input (e.g. from
      the command-line) specifying key information for the
      FrameOstream (e.g., the filename stem for a RasterOutputSeries).

      The default implementation does nothing; derived classes can
      override if they need to receive such user input.
  */
  virtual void setConfigInfo(const std::string& cfg);

  //! Advise the FrameOstream object of the current frame number
  /*! NOTE: the default implementation does nothing -- it just ignores
      the frame number. This is allowed for subclasses, too, since for
      certain output formats (e.g., onscreen windows) it doesn't make
      any sense to specify a frame number. On the other hand, certain
      subclasses require a frame number to function properly (e.g.,
      for writing a series of consecutively-numbered raster
      files). So, the bottom line is: clients of FrameOstream must be
      sure to call setFrameNumber(), but should make no assumptions
      about what it will actually do.

      @return Whether the frame number was successfully set as
      requested.
  */
  virtual bool setFrameNumber(int n);

  //! Write a frame to the output destination
  /*! Destination could be a raster file, frame in an mpeg-encoded
      movie, an on-screen window, etc., depending on the concrete
      subclass.

      @param image the image to be written, displayed, etc.

      @param shortname a brief string (with no spaces) describing the
      image; it is expected that this shortname should be usable as
      part of a filename if the particular FrameOstream subclass
      involves writing files to disk

      @param auxinfo extra optional information describing the image;
      FrameOstream subclasses make use of this information in a
      subclass-dependent manner, e.g. they could display this
      information in an onscreen window, or embed it into a comments
      in an image or movie file, etc.
  */
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo = defaultInfo) = 0;

  //! Write a frame to the output destination
  /*! Just calls writeFrame() with GenericFrame(image).

      Derived classes must not override this function; instead they
      must implement writeFrame().
  */
  void writeRGB(const Image< PixRGB<byte> >& image,
                const std::string& shortname,
                const FrameInfo& auxinfo = defaultInfo);

  //! Write a frame to the output destination
  /*! Just calls writeFrame() with GenericFrame(image).

      Derived classes must not override this function; instead they
      must implement writeFrame().
  */
  void writeGray(const Image<byte>& image,
                 const std::string& shortname,
                 const FrameInfo& auxinfo = defaultInfo);

  //! Write a frame to the output destination
  /*! Just calls writeFrame() with GenericFrame(image, flags).

      Derived classes must not override this function; instead they
      must implement writeFrame().
  */
  void writeFloat(const Image<float>& image,
                  const int flags,
                  const std::string& shortname,
                  const FrameInfo& auxinfo = defaultInfo);

  //! Write an image layout to the output destination
  /*! This constructs a GenericFrame that retains the layout
      information and passes that to writeFrame(). Some subclasses may
      be able to handle a Layout in an optimized way, while others may
      simple call render() on it and handle it as an ordinary image.
   */
  void writeRgbLayout(const Layout<PixRGB<byte> >& layout,
                      const std::string& shortname,
                      const FrameInfo& auxinfo = defaultInfo);

  //! Write an image layout to the output destination
  /*! This constructs a GenericFrame that retains the layout
      information and passes that to writeFrame(). Some subclasses may
      be able to handle a Layout in an optimized way, while others may
      simple call render() on it and handle it as an ordinary image.
   */
  void writeGrayLayout(const Layout<byte>& layout,
                       const std::string& shortname,
                       const FrameInfo& auxinfo = defaultInfo);

  //! Check if we have no output destinations (e.g., user gave --out=none)
  /*! Clients can test isVoid() before they generate potentially
      expensive output images. This is just a performance optimization
      -- even if isVoid() is true, it's still safe to call writeRGB(),
      writeGray(), etc., but those calls will do nothing, and so any
      time spent computing the image will have been wasted.

      The default implementation returns false; subclasses need
      override only if they want to (sometimes) return true.
  */
  virtual bool isVoid() const;

  //! Close off the underlying destination corresponding to shortname
  /*! The semantics of this operation are subclass-dependent, but the
      intention is for subclasses to follow these guidelines when
      implementing closeStream():

      - if the FrameOstream subclass deals with files, then the file
        corresponding to shortname should be closed, or if the
        FrameOstream deals with onscreen windows, then the
        corresponding window should be closed

      - calling closeStream() for a particular shortname should not
        prevent a stream later being reopened for that shortname if
        writeFrame() is later called again with that shortname.

      - calling closeStream() with a shortname for which there is no
        associated destination, or for a shortname which has already
        been closed, should not be an error but should just be
        silently ignored
  */
  virtual void closeStream(const std::string& shortname) = 0;

  //! Default FrameInfo object
  /*! The purpose of this object is so that the default value for the
      write functions can be 'defaultInfo' rather than 'FrameInfo()',
      since the former does not require #include
      "Transport/FrameInfo.H", while the latter does require it. */
  static const FrameInfo defaultInfo;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // FRAMEOSTREAM_H_DEFINED
