/*!@file Raster/GenericFrame.H Discriminated union of rgb, grayscale, floating-point, and video-yuv images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/GenericFrame.H $
// $Id: GenericFrame.H 14290 2010-12-01 21:44:03Z itti $
//

#ifndef RASTER_GENERICFRAME_H_DEFINED
#define RASTER_GENERICFRAME_H_DEFINED

#include "Image/Image.H"
#include "Image/Layout.H"
#include "Image/Pixels.H"
#include "Video/VideoFrame.H"
#include "rutz/shared_ptr.h"

#include <string>
#include <map>

struct GenericFrameSpec;

/// Discriminated union of rgb, grayscale, floating-point, and video-yuv images
/** This class allows frame sources (raster files, movie clips,
    frame-grabbers, etc.) to return frames in their optimal native
    representation, while deferring to the caller the ability to
    subsequently convert the frame to any convenient type. Similarly,
    GenericFrame objects can be passed to frame destinations (raster
    files, movie files, onscreen windows) in their native format,
    incurring as few conversions as possible. */
class GenericFrame
{
public:
  enum NativeType
    {
      NONE     = 0, ///< no image
      RGB_U8   = 1, ///< rgb color, 8 bits unsigned int per component
      RGB_F32  = 5, ///< rgb color, 32 bits floating point per component
      GRAY_U8  = 2, ///< grayscale, 8 bits unsigned int per pixel
      GRAY_F32 = 3, ///< grayscale, 32 bits floating point per pixel
      VIDEO    = 4,
      GRAY_U16 = 6, ///< grayscale, 16 bits unsigned uint per pixel
      RGB_U16  = 7, ///< rgb color, 16 bits unsigned int per componet
      RGBD     = 8  ///< 8-bit rgb color plus 16-bit depth image
    };

  /// MetaData for storing frame spacific information
  class MetaData
  {
  public:
    MetaData();

    virtual ~MetaData();
  };

  /// Construct an empty frame
  GenericFrame();

  /// Construct an 8-bit integer rgb-native frame
  explicit GenericFrame(const Image<PixRGB<byte> >& rgbimg);

  /// Construct an 8-bit integer rgb-native frame plus 16-bit depth image
  explicit GenericFrame(const Image<PixRGB<byte> >& rgbimg, const Image<uint16>& dimg);

  /// Construct an 16-bit integer rgb-native frame
  explicit GenericFrame(const Image<PixRGB<uint16> >& rgbimg);

  /// Construct an 8-bit integer rgb-native frame from a layout
  explicit GenericFrame(const Layout<PixRGB<byte> >& rgbimg);

  /// Construct a 32-bit floating point rgb-native frame
  /** @param flags a bitwise-or'ed combination of FLOAT_NORM_0_255,
      FLOAT_NORM_WITH_SCALE, and FLOAT_NORM_PRESERVE (see
      Image/Normalize.H). */
  explicit GenericFrame(const Image<PixRGB<float> >& rgbimg,
                        const int flags);

  /// Construct an 8-bit integer grayscale-native frame
  explicit GenericFrame(const Image<byte>& grayimg);

  /// Construct an 16-bit integer grayscale-native frame
  explicit GenericFrame(const Image<uint16>& grayimg);

  /// Construct an 8-bit integer grayscale-native frame from a layout
  explicit GenericFrame(const Layout<byte>& grayimg);

  /// Construct a 32-bit floating point grayscale-native frame
  /** @param flags a bitwise-or'ed combination of FLOAT_NORM_0_255,
      FLOAT_NORM_WITH_SCALE, and FLOAT_NORM_PRESERVE (see
      Image/Normalize.H). */
  explicit GenericFrame(const Image<float>& floatimg,
                        const int flags);

  /// Construct an video-native frame
  explicit GenericFrame(const VideoFrame& vidframe);

  // default destruct, assign, copy operators OK

  /// Check if we have a valid non-empty image, regardless of format
  bool initialized() const;

  /// Make a new GenericFrame object with an internal private copy of any transient data
  static GenericFrame deepCopyOf(const GenericFrame& f);

  /// Get the frame's specifications
  GenericFrameSpec frameSpec() const;

  /// Get the frame's native image type
  NativeType nativeType() const { return itsNativeType; }

  /// Get the frame's native image typename
  std::string nativeTypeName() const;

  /// Get the dimensions of the native image
  Dims getDims() const;

  /// Get the width of the native image
  int getWidth() const { return this->getDims().w(); }

  /// Get the height of the native image
  int getHeight() const { return this->getDims().h(); }

  /// Get the float-normalization flags
  int floatFlags() const { return itsFloatFlags; }

  /// Change the float-normalization flags
  void setFloatFlags(int v);

  /// Get an 8-bit integer rgb version of the frame, converting as necessary
  Image<PixRGB<byte> > asRgbU8() const { return this->asRgbU8Layout().render(); }
  Image<PixRGB<byte> > asRgb() const { return this->asRgbU8Layout().render(); }

  // Get a 16-bit interger rgb version of the frame, converting as necessary
  Image<PixRGB<uint16> > asRgbU16() const;

  /// Get an 8-bit integer rgb layout version of the frame, converting as necessary
  const Layout<PixRGB<byte> >& asRgbU8Layout() const;

  /// Get a 32-bit float rgb version of the frame, converting as necessary
  Image<PixRGB<float> > asRgbF32() const;

  /// Get an 8-bit integer grayscale version of the frame, converting as necessary
  Image<byte> asGrayU8() const { return this->asGrayU8Layout().render(); }
  Image<byte> asGray() const { return this->asGrayU8Layout().render(); }

  // Get a 16-bit interger grayscale version of the frame, converting as necessary
  Image<uint16> asGrayU16() const;

  /// Get an 8-bit integer grayscale layout version of the frame, converting as necessary
  const Layout<byte>& asGrayU8Layout() const;

  /// Get an 8-bit integer grayscale NTSC layout version of the frame, converting as necessary
  const Layout<byte>& asGrayU8NTSCLayout() const;

  /// Get a 32-bit float grayscale version of the frame, converting as necessary
  Image<float> asGrayF32() const;
  Image<float> asFloat() const { return this->asGrayF32(); }

  /// Convert the image from rgb to luminance in NTSC coordinates
  Image<byte> asGrayNTSC() const { return this->asGrayU8NTSCLayout().render();}
  /// Get a video version of the frame, converting as necessary
  VideoFrame asVideo() const;

  //Metadata support functions

  /// Check whether we have a given piece of metadata
  bool hasMetaData(const std::string& tag) const;

  /// Get a given piece of metadata or throw an exception if it doesn't exist
  rutz::shared_ptr<MetaData> getMetaData(const std::string& tag);

  /// Insert a given piece of metadata, overwriting any previous metadata associated with that tag
  void addMetaData(const std::string& tag, rutz::shared_ptr<MetaData> d);

private:
  // NOTE: this implementation has sizeof(GenericFrame)==108 on a
  // 32-bit machine; it may be possible to optimize this if necessary
  // (at a potentially slight cost in runtime efficiency), by storing
  // pointers to dynamically-allocated objects for each of the image
  // types instead of storing them by value here

  NativeType itsNativeType;
  mutable Layout<PixRGB<byte> > itsRgbU8;
  mutable Image<PixRGB<uint16> > itsRgbU16;
  mutable Image<PixRGB<float> > itsRgbF32;
  mutable Layout<byte> itsGrayU8;
  mutable Image<uint16> itsGrayU16;
  mutable Image<float> itsGrayF32;
  mutable VideoFrame itsVideo;
  int itsFloatFlags;

  typedef std::map<std::string, rutz::shared_ptr<MetaData> > MetaDataMap;
  rutz::shared_ptr<MetaDataMap> itsMetaDataMap;

};

/// Gives specification of a GenericFrame
struct GenericFrameSpec
{
  GenericFrameSpec();

  GenericFrame::NativeType nativeType;
  VideoFormat videoFormat; // applies only if nativeType == VIDEO
  bool videoByteSwap; // applies only if nativeType == VIDEO
  Dims dims;
  int floatFlags;
  float frameRate ;

  bool operator==(const GenericFrameSpec& that) const;

  bool operator!=(const GenericFrameSpec& that) const
  { return !(this->operator==(that)); }

  /// Return a human-readable description of this frame spec
  std::string getDescription() const;

  /// Get the frame's native image typename
  std::string nativeTypeName() const;

  /// Get the VideoFormat that would result from applying asVideo() to a corresponding GenericFrame
  VideoFormat getActualVideoFormat() const;
};

/// Check equality of two GenericFrame objects, return true if they are identical
/** If the objects are of the same native type (and same video mode,
    if the native type is video), then we check equality of the native
    images; otherwise, we convert both images to RGB_F32 and compare
    the resulting images. */
bool operator==(const GenericFrame& f1, const GenericFrame& f2);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_GENERICFRAME_H_DEFINED
