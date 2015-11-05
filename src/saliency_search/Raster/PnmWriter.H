/*!@file Raster/PnmWriter.H Write pbm/pgm/ppm image files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PnmWriter.H $
// $Id: PnmWriter.H 15424 2012-11-02 08:02:51Z kai $
//

#ifndef RASTER_PNMWRITER_H_DEFINED
#define RASTER_PNMWRITER_H_DEFINED

#include "Raster/RasterWriter.H"
#include "Util/Types.H"

#include <iosfwd>

template <class T> class Image;
template <class T> class PixRGB;

/// Write pbm/pgm/ppm image files
class PnmWriter : public RasterWriter
{
public:
  PnmWriter(const bool forcepbm = false, const byte pbmthresh = 0);

  virtual ~PnmWriter();

  virtual std::string writeFrame(const GenericFrame& image,
                                 const std::string& fname);

  /// Write the RGB image in PPM (P6) format to the given ostream
  /** There is no checking done on the stream state; so, if you want
      to know whether the write succeded you need to explicitly check
      strm.fail(). */
  static void writeRGB(const Image<PixRGB<byte> >& image,
                       std::ostream& strm);

  /// Write the 8-bit grayscale image in PGM (P5) format to the given ostream
  /** There is no checking done on the stream state; so, if you want
      to know whether the write succeded you need to explicitly check
      strm.fail(). */
  static void writeGray(const Image<byte>& image,
                        std::ostream& strm);
  /// Write the 16-bit grayscale image in PGM (P5) format to the given ostream
  /** There is no checking done on the stream state; so, if you want
      to know whether the write succeded you need to explicitly check
      strm.fail(). */
  static void writeGray(const Image<uint16>& image,
                        std::ostream& strm);

  /// Write the grayscale image in raw PBM (P4) format to the given ostream
  /** Pixels with values greater than or equal to thresh will be
      written as white, and other pixels will be written as black. */
  static void writeRawBW(const Image<byte>& image, const byte thresh,
                         std::ostream& strm);

  /// Write the grayscale image in ascii PBM (P1) format to the given ostream
  /** Pixels with values greater than or equal to thresh will be
      written as white, and other pixels will be written as black. */
  static void writeAsciiBW(const Image<byte>& image, const byte thresh,
                           std::ostream& strm);

  /// Write the RGB image in PPM (P6) format to the named file
  static void writeRGB(const Image<PixRGB<byte> >& image,
                       const std::string& fname);

  /// Write the 8-bit grayscale image in PGM (P5) format to the named file
  static void writeGray(const Image<byte>& image,
                        const std::string& fname);

  /// Write the 16-bit grayscale image in PGM (P2) format to the named file
  static void writeGray(const Image<uint16>& image,
                        const std::string& fname);
  /// Write the grayscale image in raw PBM (P4) format to the named file
  /** Pixels with values greater than or equal to thresh will be
      written as white, and other pixels will be written as black. */
  static void writeRawBW(const Image<byte>& image, const byte thresh,
                         const std::string& fname);

  /// Write the grayscale image in ascii PBM (P1) format to the named file
  /** Pixels with values greater than or equal to thresh will be
      written as white, and other pixels will be written as black. */
  static void writeAsciiBW(const Image<byte>& image, const byte thresh,
                           const std::string& fname);

private:
  const bool itsForcePBM;
  const byte itsPBMThresh;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_PNMWRITER_H_DEFINED
