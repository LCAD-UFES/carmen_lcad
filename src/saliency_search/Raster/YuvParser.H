/*!@file Raster/YuvParser.H Parse raw YUV image files. */

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
// Primary maintainer for this file: Rob Peters <rjpeters@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/YuvParser.H $
// $Id: YuvParser.H 9201 2008-02-01 17:45:38Z rjpeters $
//

#ifndef YUVPARSER_H_DEFINED
#define YUVPARSER_H_DEFINED

#include "Raster/RasterParser.H"

#include <string>

// ######################################################################
//! This class handles reading/parsing of RASFMT_RAW_VIDEO image files.
class YuvParser : public RasterParser
{
public:
  //! Construct with a filename, and use the current default Dims setting.
  /*! The exact Dims and VideoFormat of the resulting frame is
      inferred from the filename extension, which should be of the
      form ".WWWxHHH.PIXFMT", where WWW and HHH are the image
      dimensions and PIXFMT is e.g. "yuv420p" for VIDFMT_YUV420P. For
      backward-compatibility, the image dimensions may be omitted
      (though this is discouraged), in which case the dimensions are
      taken to be the current value of getDefaultDims(); the initial
      default Dims value is 640x480. */
  YuvParser(const char* fname);

  //! Destructor.
  virtual ~YuvParser();

  //! Specify a new default Dims value.
  /*! This will be the Dims that are used if a YuvParser is
      constructed with no Dims specification. */
  static void setDefaultDims(const Dims& d);

  //! Get the current default Dims value.
  /*! The initial default Dims is 640x480. */
  static Dims getDefaultDims();

  //! Whether to pass strictLength=true when calling VideoFrame::fromFile()
  static void setStrictDims(bool v);

  //! Get the current strict dims setting
  static bool getStrictDims();

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
  YuvParser(const YuvParser&);
  YuvParser& operator=(const YuvParser&);

  struct Rep;
  Rep* const rep;
};

#endif // !YUVPARSER_H_DEFINED
