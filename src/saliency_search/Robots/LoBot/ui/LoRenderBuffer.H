/**
   \file  Robots/LoBot/ui/LoRenderBuffer.H
   \brief An OpenGL framebuffer object.

   This file defines a class that uses parts of OpenGL's framebuffer
   object API to implement an off-screen rendering buffer for the
   Robolocust UI. All rendering takes place in this off-screen buffer and
   is then transferred to the screen. This architecture allows us to
   easily implement screen captures for later encoding as an MPEG so that
   users can obtain a record of a particular run/experiment of the robot.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoRenderBuffer.H $
// $Id: LoRenderBuffer.H 13838 2010-08-27 20:42:20Z mviswana $
//

#ifndef LOBOT_RENDER_BUFFER_DOT_H
#define LOBOT_RENDER_BUFFER_DOT_H

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::RenderBuffer
   \brief An encapsulation of an OpenGL framebuffer object.

   This class implements an off-screen rendering buffer using OpenGL's
   FBO API. Instead of rendering directly to the OpenGL (on-screen) back
   buffer, the Robolocust main window first renders to this off-screen
   buffer and then copies the contents of the off-screen render buffer to
   the on-screen back buffer.

   If available, this class will use the OpenGL blitting API to copy the
   off-screen buffer to the on-screen buffer so that this operation takes
   place within the GL driver via DMA. However, if blitting is not
   supported, then it will fall back to reading the pixels from the
   off-screen buffer into main memory and then drawing them to the back
   buffer.

   Furthermore, this class takes care of the details of dealing with the
   OpenGL FBO API regardless of whether it is part of the GL core is
   available as an extension.

   This somewhat indirect and convoluted architecture allows us to easily
   implement screen captures.
*/
class RenderBuffer {
   /// When rendering to an off-screen buffer, we need to know the size
   /// of the drawing area (just as we would for an on-screen buffer).
   const int m_width, m_height ;

   /// We cache the contents of the off-screen buffer so as to avoid
   /// copying more than absolutely required.
   //@{
   const int m_size ;
   unsigned char* m_cache ;
   //@}

   /// A flag to indicate whether the internal cache of the off-screen
   /// buffer's contents maintained by this object is in a state that can
   /// be retrieved by clients or not.
   mutable bool m_dirty ;

   /// The off-screen buffer is identified by the following IDs supplied
   /// by OpenGL.
   ///
   /// DEVNOTE: fbo = framebuffer object
   ///          rbo = renderbuffer object
   unsigned int m_fbo, m_rbo ;

public:
   /// Initialization: when clients create an off-screen render buffer,
   /// they should specify its dimensions. Usually, these dimensions
   /// would equal those of the UI window.
   RenderBuffer(int width, int height) ;

   /// Setup off-screen rendering: before performing regular rendering,
   /// clients should call this method so that rendering can be
   /// redirected to the off-screen buffer.
   void setup() ;

   /// Retrieve the off-screen buffer's pixels: clients can use this
   /// method after rendering is done to get the results of the rendering
   /// process that ended up in the off-screen buffer.
   ///
   /// This function returns a pointer to the internal pixel data buffer.
   /// Clients should not store this pointer for later use, delete it, or
   /// write to the buffer it points to. Instead, they should only copy
   /// the contents of the data buffer returned by this function.
   ///
   /// The size of the buffer will be W*H*4, where W and H are the width
   /// and height respectively of the off-screen buffer. These two
   /// parameters would have been specified by the client during
   /// instantiation of the lobot::RenderBuffer object.
   ///
   /// The off-screen buffer's pixels are stored in the GL_BGRA format.
   /// The pixel data type is GL_UNSIGNED_BYTE.
   ///
   /// NOTE: If the OpenGL driver is missing support for framebuffer
   /// objects, the pixel data returned by this function could be
   /// corrupt, i.e., contain random garbage. This would usually occur
   /// when the Robolocust window is wholly or partially obscured by
   /// another window.
   const unsigned char* pixels() const ;

   /// Helper function to return the size (in bytes) of the off-screen
   /// buffer's pixel data array.
   int size() const {return m_size ;}

   /// Helper functions to return the dimensions of the off-screen
   /// buffer.
   ///
   /// NOTE: In general, a client should not really need to use these
   /// functions because they ought to already know this information and
   /// would have passed the width and height to the lobot::RenderBuffer
   /// constructor.
   //@{
   int width()  const {return m_width  ;}
   int height() const {return m_height ;}
   //@}

   /// Copy contents of off-screen buffer to on-screen buffer.
   void to_screen() ;

private:
   /// Helper function to release OpenGL resources.
   void clean_up() ;

public:
   /// Clean-up.
   ~RenderBuffer() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
