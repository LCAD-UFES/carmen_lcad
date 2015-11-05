/*!@file Raster/Raster.H Writes a raster image to disk / reads it / displays it */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/Raster.H $
// $Id: Raster.H 10547 2008-12-18 21:37:57Z dparks $
//

#ifndef RASTER_H_DEFINED
#define RASTER_H_DEFINED

#include "Image/ArrayData.H"
#include "Image/Normalize.H" // for float normalization flags
#include "Raster/RasterFileFormat.H"
#include "Util/Types.H"
#include "Util/sformat.H"

#include <string>

class GenericFrame;
class GenericFrameSpec;
class VideoFrame;
template <class T> class Image;
template <class T> class PixRGB;

// ######################################################################
//! For reading, writing and displaying images.
/*! Raster works with PGM files (for grayscale images) and PPM files
    (for RGB images). Selection between PGM and PPM given the
    RASFMT_PNM RasterFileFormat is automatic based on whether the
    image is greyscale or color. In addition, it offers PNG read
    capability (grayscale, RGB, or color-palette, at any bit-depth),
    and write capability (8-bit grayscale and RGB). Similarly, it
    offers write capability for RASFMT_RAW_IMAGE data (straight pixel
    data with no header information), but no such read
    capability. Both the raw and ASCII variants of PGM/PPM files are
    supported. Display of images uses the program "xv" to output
    images to the screen. The filename argument is passed to the
    various methods of this class in the same way as in printf(). */
namespace Raster
{
  // ######################################################################
  // Read/Write in various formats, for byte-based images:

  //! Return true if file exists on disk
  /*! If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  bool fileExists(std::string fname,
                  const RasterFileFormat ftype = RASFMT_AUTO,
                  const bool chkFormat = false);

  //! Get image comments without fully reading the image (PPM only)
  /*! If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  std::string getImageComments(std::string fname,
                               const RasterFileFormat ftype = RASFMT_AUTO);

  //! Get image dimensions without fully reading the image
  /*! If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  Dims getImageDims(std::string fname,
                    const RasterFileFormat ftype = RASFMT_AUTO);

  //! Get image specifications without fully reading the image
  /*! If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  GenericFrameSpec getFrameSpec(std::string fname,
                                const RasterFileFormat ftype = RASFMT_AUTO);

  //! Read a GenericFrame from a raster file.
  /*! This can subsequently be converted to rgb, grayscale,
      floating-point, or video-yuv.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  GenericFrame ReadFrame(std::string fname,
                         RasterFileFormat ftype = RASFMT_AUTO);

  //! Read an RGB byte image from a raster file
  /*! Just calls ReadFrame() internally.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  Image<PixRGB<byte> > ReadRGB(std::string fname,
                               RasterFileFormat ftype = RASFMT_AUTO);

  //! Read a greyscale byte image from a greyscale raster file
  /*! Just calls ReadFrame() internally.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  Image<byte> ReadGray(std::string fname,
                       RasterFileFormat ftype = RASFMT_AUTO);

  //! Read a greyscale byte image, converting to NTSC coordinates if necessary
  /*! Just calls ReadFrame() internally.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  Image<byte> ReadGrayNTSC(std::string fname,
                           const RasterFileFormat ftype = RASFMT_AUTO);

  //! Read a greyscale float image from a greyscale raster file
  /*! Just calls ReadFrame() internally.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  Image<float> ReadFloat(std::string fname,
                         RasterFileFormat ftype = RASFMT_AUTO);

  //! Read a VideoFrame from a raster file
  /*! Just calls ReadFrame() internally.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  VideoFrame ReadVideo(std::string fname,
                       RasterFileFormat ftype = RASFMT_AUTO);

  //! Write a GenericFrame to a raster file
  /*! The GenericFrame could be constructed from rgb, grayscale,
      floating-point, or video-yuv.

      Returns the full name of the image file that was actually
      written; this may differ slightly from fname if a filename
      extension was added to match ftype.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  std::string WriteFrame(const GenericFrame& image,
                         std::string fname,
                         RasterFileFormat ftype = RASFMT_AUTO);

  //! Write an RGB byte image to a raster file
  /*! Just calls WriteFrame() internally.

      Returns the full name of the image file that was actually
      written; this may differ slightly from fname if a filename
      extension was added to match ftype.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  std::string WriteRGB(const Image< PixRGB<byte> >& image,
                       std::string fname,
                       const RasterFileFormat ftype = RASFMT_AUTO);

  //! Write a grayscale byte image to a raster file
  /*! Just calls WriteFrame() internally.

      Returns the full name of the image file that was actually
      written; this may differ slightly from fname if a filename
      extension was added to match ftype.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  std::string WriteGray(const Image<byte>& image,
                        std::string fname,
                        const RasterFileFormat ftype = RASFMT_AUTO);

  //! Write a float image to a raster file
  /*! Just calls WriteFrame() internally.

      Returns the full name of the image file that was actually
      written; this may differ slightly from fname if a filename
      extension was added to match ftype.

      If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  std::string WriteFloat(const Image<float>& image,
                         const int flags, std::string fname,
                         const RasterFileFormat ftype = RASFMT_AUTO);

  //! Turn off the use of xv
  /*! Turn off xv, so that calls to Display() do nothing, and calls to
      Visu() just save the image to disk without displaying it onscreen. */
  void NoXV();

  //! calls xv to display an image that is on disk
  void Display(const char* file);

  //! wait for the [return] key
  void waitForKey();

  // save image and display it:

  //! Save image of type Image< PixRGB<byte> > to disk then display it using xv
  /*! If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  void VisuRGB(const Image< PixRGB<byte> >& image,
               std::string fname,
               const RasterFileFormat ftype = RASFMT_AUTO);

  //! Save image of type Image<byte> to disk then display it using xv
  /*! If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  void VisuGray(const Image<byte>& image,
                std::string fname,
                const RasterFileFormat ftype = RASFMT_AUTO);

  //! Save image of type Image<float> to disk then display it using xv
  /*! If ftype is RASFMT_AUTO, then the raster file type is inferred
      from the filename.
  */
  void VisuFloat(const Image<float>& image,
                 const int flags,
                 std::string fname,
                 const RasterFileFormat ftype = RASFMT_AUTO);
};

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
