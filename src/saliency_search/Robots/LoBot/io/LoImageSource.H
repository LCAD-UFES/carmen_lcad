/**
   \file  Robots/LoBot/io/LoImageSource.H
   \brief Just a source of images.
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoImageSource.H $
// $Id: LoImageSource.H 11256 2009-05-30 01:46:10Z mviswana $
//

#ifndef LOBOT_IMAGE_SOURCE_DOT_H
#define LOBOT_IMAGE_SOURCE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/LoTypes.H"

// INVT image support
#include "Image/Image.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::ImageSource
   \brief Just a source of Images.

   This class serves as an interface for image sources and is meant to be
   used by lobot::ImageDrawable and lobot::LocustModel. The
   lobot::LocustModel class uses this one via the lobot::InputSource
   encapsulation.
*/
template<typename pixel_type>
class ImageSource {
   ImageSource(const ImageSource&) ;
   ImageSource& operator=(const ImageSource&) ;

protected:
   // Each image source will cache the image it retrieves from its
   // "upstream" sources.
   Image<pixel_type> m_image ;
   GrayImage         m_image_gray ;

   ImageSource() ;

public:
   virtual ~ImageSource() ;
   virtual void update() = 0 ;
   Image<pixel_type> getImage() const {return m_image ;}
   GrayImage     getImageGray() const {return m_image_gray ;}
   virtual Dims  getImageSize() const ;
} ;

// Constructor and destructor
template<typename T>
ImageSource<T>::ImageSource(){}

template<typename T>
ImageSource<T>::~ImageSource(){}

// Accessor
template<typename T>
Dims ImageSource<T>::getImageSize() const
{
   return m_image.getDims() ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
