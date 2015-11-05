/*!@file Raster/QuartzQuickTimeParser.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/QuartzQuickTimeParser.H $
// $Id: QuartzQuickTimeParser.H 7960 2007-02-20 23:38:06Z rjpeters $
//

#ifndef RASTER_QUARTZQUICKTIMEPARSER_H_DEFINED
#define RASTER_QUARTZQUICKTIMEPARSER_H_DEFINED

#ifdef HAVE_QUICKTIME_QUICKTIME_H

#include "Image/Dims.H"
#include "Raster/RasterParser.H"
#include "Util/Types.H"

template <class T> class Image;
template <class T> class PixRGB;

// ######################################################################
/// Read image files using Apple's QuickTime library.
/** Useful reference documentation:

    http://developer.apple.com/documentation/QuickTime/Reference/QTRef_GraphicsImportExport/index.html
    http://developer.apple.com/documentation/GraphicsImaging/Reference/CGImage/index.html
    http://developer.apple.com/graphicsimaging/workingwithimageio.html
    http://developer.apple.com/documentation/GraphicsImaging/Reference/CGBitmapContext/index.html
    http://developer.apple.com/samplecode/ImproveYourImage/listing9.html
    http://developer.apple.com/documentation/GraphicsImaging/Reference/CGColorSpace/index.html
    http://developer.apple.com/qa/qa2001/qa1037.html
*/
class QuartzQuickTimeParser : public RasterParser
{
public:
  //! Intialize with the name of a PNG file.
  QuartzQuickTimeParser(const char* filename);

  //! Destructor.
  virtual ~QuartzQuickTimeParser();

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
  QuartzQuickTimeParser(const QuartzQuickTimeParser&);
  QuartzQuickTimeParser& operator=(const QuartzQuickTimeParser&);

  struct Rep;
  Rep* const rep;
};

#endif // HAVE_QUICKTIME_QUICKTIME_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_QUARTZQUICKTIMEPARSER_H_DEFINED
