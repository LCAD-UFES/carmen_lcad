/*!@file Raster/JpegParser.H Parser for jpeg image files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/JpegParser.H $
// $Id: JpegParser.H 15288 2012-05-11 22:25:31Z kai $
//

#ifndef RASTER_JPEGPARSER_H_DEFINED
#define RASTER_JPEGPARSER_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/RasterParser.H"
#include "Image/JPEGUtil.H"

//! Parser class for reading jpeg image files by using libjpeg.
class JpegParser : public RasterParser
{
public:
  //! Construct with the name of a jpeg image file.
  JpegParser(const char* fname);

  //! Virtual destructor for proper destruction of derived classes.
  virtual ~JpegParser();

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
  JpegParser(const JpegParser&);
  JpegParser& operator=(const JpegParser&);

  class Impl;
  Impl* rep;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_JPEGPARSER_H_DEFINED
