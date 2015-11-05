/*!@file Raster/RasterParser.H Abstract image-file parser class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/RasterParser.H $
// $Id: RasterParser.H 6572 2006-05-05 21:16:38Z mundhenk $
//

#ifndef RASTERPARSER_H_DEFINED
#define RASTERPARSER_H_DEFINED

#include "Util/Types.H"

#include <string>

class Dims;
class GenericFrame;
class GenericFrameSpec;
class VideoFrame;
template <class T> class PixRGB;
template <class T> class Image;

//! This class defines a generic interface for parsing image files.
class RasterParser
{
public:
  //! Virtual destructor for proper destruction of derived classes.
  virtual ~RasterParser();

  //! Get the specifications of the image without fully reading it
  virtual GenericFrameSpec getFrameSpec() const = 0;

  //! Get the comments from the image without fully reading it
  virtual std::string getComments() const = 0;

  //! Get image tags if the image supports them
  virtual uint getTagCount() const = 0;

  //! Get image tags if the image supports them
  virtual bool getTag(uint tag, std::string &name, std::string &value) const = 0;

  //! Get a GenericFrame representation from the raster file.
  /*! Derived classes should implement this so that the native
      representation of the GenericFrame best matches the native
      format of the raster file (e.g., rgb, grayscale, floating-point,
      video-yuv). */
  virtual GenericFrame getFrame() = 0;
};

#endif // !RASTERPARSER_H_DEFINED
