/*!@file Raster/PfzParser.H Parse pfm image files. */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PfzParser.H $
// $Id: PfzParser.H 6795 2006-06-29 20:45:32Z rjpeters $
//

#ifndef PFZPARSER_H_DEFINED
#define PFZPARSER_H_DEFINED

#define PFZ_CHUNK 262144

#include "Image/Dims.H"
#include "Raster/RasterParser.H"
#include "Util/Types.H"
#include "Util/Promotions.H"
#include <zlib.h>

#include <string>
#include <sstream>

template <class T> class Image;
template <class T> class PixRGB;

// ######################################################################
//! This class handles reading/parsing of PFM image files.
/*! PFM is an unofficial format that looks very much like PGM except
  that raw pixel values are floats. Caution, this is not portable
  across endianness. The PNM magic for this unofficial format is
  PF. */
class PfzParser : public RasterParser
{
public:
  //! Initialize with a PGM or PPM file (either is OK).
  PfzParser(const char* fname);

  //! Destructor.
  virtual ~PfzParser();

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
  //! Read the actual bytes out of the file.
  /*! This works for any supported mode or file type. */
  void readFloats(float* space, int count);

  //! Parse a float grayscale image
  Image<float> parseFloat();

  PfzParser(const PfzParser&);
  PfzParser& operator=(const PfzParser&);

  struct Rep;
  Rep* const rep;
  z_stream zstrm;
  unsigned char buff_uin[PFZ_CHUNK];
  unsigned char buff_uout[PFZ_CHUNK];

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !PFMPARSER_H_DEFINED
