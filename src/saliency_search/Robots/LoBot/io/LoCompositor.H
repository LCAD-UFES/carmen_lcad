/**
   \file  Robots/LoBot/io/LoCompositor.H
   \brief Quick-and-dirty image stitcher.

   This file defines a class to read images from several sources and then
   stitch them together into a single image.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoCompositor.H $
// $Id: LoCompositor.H 11230 2009-05-27 11:11:23Z mviswana $
//

#ifndef LOBOT_COMPOSITOR_DOT_H
#define LOBOT_COMPOSITOR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoVideoStream.H"
#include "Robots/LoBot/io/LoImageSource.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoTypes.H"

// INVT image support
#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/Point2D.H"
#include "Image/Dims.H"

// Standard C++ headers
#include <list>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Compositor
   \brief Quick-and-dirty "panaromic" image stitcher.

   From Wikipedia:

      "Digital compositing is the process of digitally assembling
      multiple images to make a final image, typically for print, motion
      pictures or screen display. It is the evolution into the digital
      realm of optical film compositing."

   This class does not actually do any image compositing. No fancy alpha
   blending or any other such stuff. But technically, it does assemble
   multiple images into a final image and in that sense can be considered
   a kind of compositor (albeit a laughably crude one).

   Given a number of different video streams, lobot::Compositor reads the
   current frame from each and then simply stitches the images together
   to create a kind of panaromic image. Of course, it doesn't actually
   try to find correspondences between image boundaries and properly
   stitch the images to create the final result. Rather, it simply
   creates a big image and copies each of the source images into this
   final image.

   The motivation for this class is as follows: Robolocust is equipped
   with three FireWire cameras pointed straight ahead and to the left and
   right. We want to take the images from these cameras and create a
   single frame from them. Thus this simple-minded "compositor."
*/
template<typename pixel_type>
class Compositor : public ImageSource<pixel_type> {
   // Handy type to have around in a derived class
   typedef ImageSource<pixel_type> base ;

   // Prevent copy and assignment
   Compositor(const Compositor&) ;
   Compositor& operator=(const Compositor&) ;

   /// The compositor needs a number of video sources from which it can
   /// draw images to stitch into its final product.
   //@{
   typedef std::list<VideoStream*> Streams ;
   Streams m_streams ;
   //@}

   /// We update the dimensions of the output image as new sources are
   /// added so that we don't have to compute these dimensions every time
   /// the compositor's client requests it to output the compositing
   /// result.
   int m_output_width, m_output_height ;

public:
   Compositor() ;
   ~Compositor() ;

   /// This method adds a video stream to the compositor's list of input
   /// sources.
   void push_back(VideoStream*) ;

   /// STL compatibility
   typedef Streams::const_reference const_reference ;

   /// This is the compositor's output routine. When invoked, it reads
   /// frames from all its current sources and stitches them together to
   /// create the final output image.
   void update() ;

   /// This method returns the output image's size
   Dims getImageSize() const {return Dims(m_output_width, m_output_height) ;}
} ;

//-------------------- INITIALIZATION AND CLEAN-UP ----------------------

template<typename pixel_type>
Compositor<pixel_type>::Compositor()
   : m_output_width(0), m_output_height(0)
{}

template<typename pixel_type>
Compositor<pixel_type>::~Compositor()
{}

//----------------------- ADDING IMAGE SOURCES --------------------------

// Every time a source is added, we need to update the dimensions of the
// output image. If we don't do this here, in the output routine, we will
// have to walk through the sources list twice: the first pass to
// determine the output image's size and second to do the actual
// compositing.
template<typename pixel_type>
void Compositor<pixel_type>::push_back(VideoStream* V)
{
   if (! V)
      return ;
   m_streams.push_back(V) ;

   Dims frame_size = V->frameSize() ;
   m_output_width += frame_size.w() ;
   m_output_height = std::max(m_output_height, frame_size.h()) ;
}

//------------------------ IMAGE "COMPOSITING" --------------------------

// The following function object reads the next frame from the specified
// input video source and copies it into the target image. On each
// invocation, it copies the source image into the correct location of
// the target image by maintaining a "cursor" to keep track of the
// insertion point. This cursor only moves along the x-direction, i.e.,
// source images are copied to the target flush up against the top.
namespace {

template<typename T>
class paste_into {
   mutable Image<T>& target ;
   mutable Point2D<int> cursor ;
public:
   paste_into(Image<T>&) ;
   void operator()(const VideoStream* V) const {
      inplacePaste(target, V->readFrame(), cursor) ;
      cursor.i += V->frameSize().w() ;
   }
} ;

// When the above function object is instantiated, we will use the
// supplied image as the target and initialize the cursor to start
// inserting at the top left corner (0,0) of the target image.
template<typename T>
paste_into<T>::paste_into(Image<T>& I)
   : target(I), cursor(0,0)
{}

} // end of local namespace encapsulating above function object

// This is the compositor's output routine. It assembles the output image
// from the frames of each of its input video sources.
template<typename T>
void Compositor<T>::update()
{
   if (m_streams.empty())
      throw vstream_error(NO_COMPOSITOR_SOURCES) ;

   Image<T> I(m_output_width, m_output_height, NO_INIT) ;
   std::for_each(m_streams.begin(), m_streams.end(), paste_into<T>(I)) ;

   base::m_image = I ;
   base::m_image_gray = GrayImage(luminance(I)) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
