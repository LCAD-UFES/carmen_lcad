/**
   \file Robots/LoBot/io/LoInputSource.H

   \brief Input source encapsulation for the Lobot/Robolocust project.

   This file defines the lobot::InputSource class, which is used to wrap
   around the classes responsible for the different kinds of input lobot
   can handle. These inputs include video streams, the laser range finder
   and a direct connection to actual locusts.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoInputSource.H $
// $Id: LoInputSource.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_INPUT_SOURCE_DOT_H
#define LOBOT_INPUT_SOURCE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoImageSource.H"
#include "Robots/LoBot/io/LoLaserRangeFinder.H"

#include "Robots/LoBot/config/LoDefaults.H"
#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/util/range.hh"

// INVT image support
#include "Image/Dims.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::InputSource
   \brief Input sources wrapper for Lobot/Robolocust project.

   This class wraps around lobot::ImageSource and lobot::LaserRangeFinder
   so that its clients need only worry about interfacing with this one
   class instead of having to mess around with the APIs of many different
   classes.
*/
class InputSource {
   // Prevent copy and assignment
   InputSource(const InputSource&) ;
   InputSource& operator=(const InputSource&) ;

   /// The input may be coming either from a video stream (i.e., FireWire
   /// camera or MPEG file) or from the laser range finder.
   ImageSource<PixelType>* m_video ;
   LaserRangeFinder*       m_laser ;

public:
   /// This constructor sets up an input source to read from an image
   /// source (either FireWire cameras or MPEG files). Usually, if there
   /// are multiple cameras or MPEGs involved, the image source will be
   /// set up to combine these multiple input images into a single image.
   InputSource(ImageSource<PixelType>*) ;

   /// This constructor sets up the laser range finder as the input
   /// source.
   InputSource(LaserRangeFinder*) ;

   /// Returns true if the input source is configured to use video.
   bool using_video() const {return m_video != 0 ;}

   /// Returns true if the input source is configured to use laser.
   bool using_laser() const {return m_laser != 0 ;}

   /// Return the size of the input image if the input really is a video
   /// source; throw an exception otherwise.
   Dims get_image_size() const ;

   /// Return the grayscale version of the input image. Throws an
   /// exception if the input source is not a video stream.
   GrayImage get_grayscale_image() const ;

   /// Return the angular range of the laser range finder. Throw an
   /// exception if the input source is not the laser range finder.
   range<int> lrf_angular_range() const ;

   /// Return the distance reading corresponding to the supplied angle.
   /// Throw an exception if the input source is not the laser range
   /// finder.
   int get_distance(int angle) const ;

   /// Return the average distance in an angular range. Throw an
   /// exception if the input source is not the laser range finder.
   //@{
   float average_distance(int min, int max) const ;
   float average_distance(const range<int>& R) const {
      return average_distance(R.min(), R.max()) ;
   }
   //@}

   /// Clean-up
   ~InputSource() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
