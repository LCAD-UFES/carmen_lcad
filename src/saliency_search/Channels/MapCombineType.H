/*!@file Channels/MapCombineType.H Different methods of combining channel output maps */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MapCombineType.H $
// $Id: MapCombineType.H 8827 2007-10-12 17:45:29Z rjpeters $
//

#ifndef CHANNELS_MAPCOMBINETYPE_H_DEFINED
#define CHANNELS_MAPCOMBINETYPE_H_DEFINED

#include <string>

template <class T> class Image;

/// Different methods for combining channel output maps
enum MapCombineType
  {
    MAPCOMBINE_SUM = 0,     ///< c = a + b          (pixel-wise sum)
    MAPCOMBINE_MAX = 1      ///< c = takeMax(a, b)  (pixel-wise max)
  };

/// Combine two maps according to the given combination strategy
/** For convenience in writing map-combination loops, one of the
    images may be empty, in which case the result will just be the
    other image. */
Image<float> mapCombine(const MapCombineType typ,
                        const Image<float>& a, const Image<float>& b);

/// MapCombineType -> string
std::string convertToString(const MapCombineType typ);

/// string -> MapCombineType
void convertFromString(const std::string& str, MapCombineType& typ);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_MAPCOMBINETYPE_H_DEFINED
