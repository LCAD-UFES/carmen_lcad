/*!@file Raster/PnmParser.H Parse pbm/pgm/ppm image files. */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PnmParser.H $
// $Id: PnmParser.H 7810 2007-01-26 23:28:37Z rjpeters $
//

#ifndef PNMPARSER_H_DEFINED
#define PNMPARSER_H_DEFINED

#include "Raster/RasterParser.H"
#include "Util/Types.H" // for byte

#include <iosfwd>
#include <string>

class Dims;
template <class T> class Image;
template <class T> class PixRGB;

// ######################################################################
//! This class handles reading/parsing of PBM/PGM/PPM image files.
/*! All six pbm modes are supported:

    -P1: plain/ascii 1-bit black+white
    -P2: plain/ascii grayscale
    -P3: plain/ascii RGB
    -P4: raw/binary packed 1-bit black+white (8 pixels per byte)
    -P5: raw/binary 8-bit grayscale
    -P6: raw/binary 24-bit RGB
*/
class PnmParser : public RasterParser
{
public:
  //! Initialize with a PBM, PGM or PPM file.
  PnmParser(const char* fname);

  //! Initialize with a stream that will contain PBM, PGM or PPM data.
  PnmParser(std::istream& strm);

  //! Destructor.
  virtual ~PnmParser();

  //! Get the specifications of the image without fully reading it
  virtual GenericFrameSpec getFrameSpec() const;

  //! Get the comments from the image without fully reading it
  virtual std::string getComments() const;

  //! Get image tags if the image supports them
  virtual uint getTagCount() const;

  //! Get image tags if the image supports them
  virtual bool getTag(uint tag, std::string &name, std::string &value) const;

  //! Get a GenericFrame representation from the raster file.
  /*! This can subsequently be converted to rgb, grayscale,
      floating-point, or video-yuv. */
  virtual GenericFrame getFrame();

private:
  PnmParser(const PnmParser&);
  PnmParser& operator=(const PnmParser&);

  struct Rep;
  Rep* const rep;
};


/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !PNMPARSER_H_DEFINED
