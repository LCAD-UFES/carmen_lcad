/*!@file Raster/PlaintextWriter.H Plaintext image-file writer class; resulting files usable with matlab's 'load' */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PlaintextWriter.H $
// $Id: PlaintextWriter.H 8312 2007-04-27 22:05:07Z rjpeters $
//

#ifndef RASTER_PLAINTEXTWRITER_H_DEFINED
#define RASTER_PLAINTEXTWRITER_H_DEFINED

#include "Raster/RasterWriter.H"
#include "Util/Types.H"

template <class T> class Image;
template <class T> class PixRGB;

/// Plaintext image-file writer class
/** The resulting files should be usable with matlab's 'load'
    function, for example. */
class PlaintextWriter : public RasterWriter
{
public:
  PlaintextWriter();

  virtual ~PlaintextWriter();

  virtual std::string writeFrame(const GenericFrame& image,
                                 const std::string& fname);

  static void writePlaintextRgbU8(const Image<PixRGB<byte> >& img,
                                  const std::string& fname);

  static void writePlaintextRgbF32(const Image<PixRGB<float> >& img,
                                   const std::string& fname);

  static void writePlaintextGrayU8(const Image<byte>& img,
                                   const std::string& fname);

  static void writePlaintextGrayF32(const Image<float>& img,
                                    const std::string& fname);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_PLAINTEXTWRITER_H_DEFINED
