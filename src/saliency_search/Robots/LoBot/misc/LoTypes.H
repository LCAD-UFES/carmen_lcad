/**
   \file Robots/LoBot/misc/LoTypes.H

   \brief Some frequently used types across different parts of all the
   Lobot/Robolocust related programs.
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoTypes.H $
// $Id: LoTypes.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_COMMON_TYPES_DOT_H
#define LOBOT_COMMON_TYPES_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/triple.hh"

// INVT image support
#include "Image/Image.H"
#include "Image/PixelsTypes.H"
#include "Image/Point2D.H"

// Standard C++ headers
#include <limits>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------------- TYPEDEFS --------------------------------

/// Images captured from the video sources supported by Robolocust will
/// usually have RGB pixels.
///@{
typedef PixRGB<byte>     PixelType ;
typedef Image<PixelType> ImageType ;
///@}

/// Although the "raw" input images are usually RGB and are also
/// visualized in color, most image processing algorithms work with
/// gray-scale images.
typedef Image<float> GrayImage ;

/// A convenient short-cut.
typedef Point2D<int> Point ;

//------------------------------ CLASSES --------------------------------

/// OpenGL colors are represented as a triple of floating point values
/// that lie in the range [0,1].
class GLColor {
   float m_color[3] ;
public:
   GLColor() ;
   GLColor(unsigned char, unsigned char, unsigned char) ;
   GLColor(float, float, float) ;
   GLColor(const GLColor&) ;
   GLColor& operator=(const GLColor&) ;

   //export template<typename T> GLColor(const triple<T, T, T>&) ;
   template<typename T> GLColor(const triple<T, T, T>&) ;

   float red()   const {return m_color[0] ;}
   float green() const {return m_color[1] ;}
   float blue()  const {return m_color[2] ;}
   const float* rgb() const {return m_color ;}
} ;

// Construct from a triple
template<typename T>
GLColor::GLColor(const triple<T, T, T>& t)
{
   if (std::numeric_limits<T>::is_integer)
   {
      m_color[0] = clamp<T>(t.first,  T(0), T(255))/255.0f ;
      m_color[1] = clamp<T>(t.second, T(0), T(255))/255.0f ;
      m_color[2] = clamp<T>(t.third,  T(0), T(255))/255.0f ;
   }
   else
   {
      m_color[0] = static_cast<float>(clamp<T>(t.first,  T(0), T(1))) ;
      m_color[1] = static_cast<float>(clamp<T>(t.second, T(0), T(1))) ;
      m_color[2] = static_cast<float>(clamp<T>(t.third,  T(0), T(1))) ;
   }
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
