/**
   \file Robots/LoBot/io/LoInputSource.C

   \brief This file defines the non-inline member functions of the
   lobot::InputSource class, which is used to wrap around the classes
   responsible for the different kinds of input lobot can handle.
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
// Primary maintainer for this file: Manu Viswanathan mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoInputSource.C $
// $Id: LoInputSource.C 12853 2010-02-16 16:33:21Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoInputSource.H"
#include "Robots/LoBot/misc/LoExcept.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// Read input from an image source
InputSource::InputSource(ImageSource<PixelType>* is)
   : m_video(is),
     m_laser(0)
{}

// Read input from a laser range finder
InputSource::InputSource(LaserRangeFinder* lrf)
   : m_video(0),
     m_laser(lrf)
{}

//---------------------- VIDEO SOURCE INTERFACE -------------------------

Dims InputSource::get_image_size() const
{
   if (m_video)
      return m_video->getImageSize() ;
   throw vstream_error(NO_VIDEOSTREAM_SOURCE) ;
}

GrayImage InputSource::get_grayscale_image() const
{
   if (m_video)
      return m_video->getImageGray() ;
   throw vstream_error(NO_VIDEOSTREAM_SOURCE) ;
}

//------------------- LASER RANGE FINDER INTERFACE ----------------------

range<int> InputSource::lrf_angular_range() const
{
   if (m_laser)
      return m_laser->get_angular_range() ;
   throw lrf_error(NO_LRF_SOURCE) ;
}

int InputSource::get_distance(int angle) const
{
   if (m_laser)
      return m_laser->get_distance(angle) ;
   throw lrf_error(NO_LRF_SOURCE) ;
}

float InputSource::average_distance(int min, int max) const
{
   if (m_laser)
      return m_laser->average_distance(min, max) ;
   throw lrf_error(NO_LRF_SOURCE) ;
}

//----------------------------- CLEAN-UP --------------------------------

InputSource::~InputSource() {}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
