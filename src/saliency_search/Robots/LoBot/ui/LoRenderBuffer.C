/**
   \file Robots/LoBot/ui/LoRenderBuffer.C

   \brief This file defines the non-inline member functions of the
   lobot::RenderBuffer class, which is used to implement an off-screen
   rendering buffer via OpenGL's framebuffer object API.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoRenderBuffer.C $
// $Id: LoRenderBuffer.C 13838 2010-08-27 20:42:20Z mviswana $
//

//-------------------------- OPENGL MISSING -----------------------------

#ifndef INVT_HAVE_LIBGL

#include "Robots/LoBot/ui/LoRenderBuffer.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

RenderBuffer::RenderBuffer(int, int)
   : m_width(0), m_height(0),
     m_size(0), m_cache(0), m_dirty(false),
     m_fbo(0), m_rbo(0)
{
   throw missing_libs(MISSING_OPENGL) ;
}

void RenderBuffer::setup(){}
const unsigned char* RenderBuffer::pixels() const {return 0 ;}
void RenderBuffer::to_screen(){}
void RenderBuffer::clean_up(){}
RenderBuffer::~RenderBuffer(){}

} // end of namespace encapsulating above empty definition

#else // OpenGL available

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoRenderBuffer.H"
#include "Robots/LoBot/misc/LoExcept.H"

// INVT headers
#include "Util/log.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLEW
#include <GL/glew.h>
#endif

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

//------------------------------ MACROS ---------------------------------

// Since off-screen rendering support depends heavily on OpenGL versions
// and extension availability, it is useful to be able to see some
// diagnostic messages about how this module goes about implementing its
// off-screen rendering API. This symbol can be used to turn this
// diagnostic tracing on. In normal use, it would be best to keep the
// following line commented.
//#define LOBOT_TRACE_RENDER_BUFFER

// We simply use INVT's LERROR to implement the tracing functionality
// alluded to above. However, to be able to turn tracing on/off without
// having to manually comment each and every LERROR or to wrap each
// LERROR inside of an #ifdef LOBOT_TRACE_RENDER_BUFFER block, we use the
// following macro, viz., LRB_TRACE (LRB = lobot render buffer), and
// define it appropriately depending on whether tracing is on or off.
#ifdef LOBOT_TRACE_RENDER_BUFFER // use INVT LERROR for trace messages
   #define LRB_TRACE LERROR
#else // no tracing ==> LRB_TRACE = nop
   #define LRB_TRACE(...)
#endif

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------ FRAMEBUFFER OBJECTS IN GL CORE ---------------------

// DEVNOTE: THIS CODE IS SUSPECT!
//
// It was never actually built and run on a machine that had the FBO API
// in the GL core.
#ifdef GL_VERSION_4_0
//#warning "framebuffer objects are part of GL core"

// Initialize off-screen rendering buffer of specified size
RenderBuffer::RenderBuffer(int W, int H)
   : m_width(W), m_height(H),
     m_size(W * H * 4), m_cache(new unsigned char[m_size]), m_dirty(true)
{
   LRB_TRACE("initializing off-screen rendering using core GL API...") ;

   glGenFramebuffers(1, &m_fbo) ;
   glBindFramebuffer(GL_FRAMEBUFFER, m_fbo) ;

   glGenRenderbuffers(1, &m_rbo) ;
   glBindRenderbuffer(GL_RENDERBUFFER, m_rbo) ;
   glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, m_width, m_height) ;
   if (glGetError() != GL_NO_ERROR) {
      clean_up() ;
      throw misc_error(OPENGL_FBO_INIT_ERROR) ;
   }

   glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                             GL_RENDERBUFFER, m_rbo) ;

   if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      clean_up() ;
      throw misc_error(OPENGL_FBO_INIT_ERROR) ;
   }

   std::fill_n(m_cache, m_size, 0) ;
}

// Setup off-screen rendering
void RenderBuffer::setup()
{
   glBindFramebuffer(GL_FRAMEBUFFER, m_fbo) ;
   m_dirty = true ;
}

// Retrieve off-screen buffer's pixel data
const unsigned char* RenderBuffer::pixels() const
{
   if (m_dirty)
   {
      LRB_TRACE("retrieving off-screen buffer's pixel data...") ;
      glBindFramebuffer(GL_READ_FRAMEBUFFER, m_fbo) ;
      glReadPixels(0, 0, m_width, m_height,
                   GL_BGRA, GL_UNSIGNED_BYTE, m_cache) ;
      if (glGetError() == GL_NO_ERROR)
         m_dirty = false ;
      else
         std::fill_n(m_cache, m_size, 0) ;
   }
   else
      LRB_TRACE("returning cached off-screen buffer pixel data...") ;
   return m_cache ;
}

// Copy contents of off-screen buffer to the back buffer
void RenderBuffer::to_screen()
{
   LRB_TRACE("blitting off-screen buffer to back buffer...") ;

   glBindFramebuffer(GL_READ_FRAMEBUFFER, m_fbo) ;
   glReadBuffer(GL_COLOR_ATTACHMENT0) ;

   glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0) ;
   glDrawBuffer(GL_BACK) ;

   glBlitFramebuffer(0, 0, m_width, m_height,
                     0, 0, m_width, m_height,
                     GL_COLOR_BUFFER_BIT, GL_NEAREST) ;
}

// Clean-up
void RenderBuffer::clean_up()
{
   LRB_TRACE("cleaning up off-screen buffer...") ;

   glBindFramebuffer (GL_FRAMEBUFFER,  0) ;
   glBindRenderbuffer(GL_RENDERBUFFER, 0) ;

   glDeleteRenderbuffers(1, &m_rbo) ;
   glDeleteFramebuffers (1, &m_fbo) ;

   delete[] m_cache ; m_cache = 0 ;
}

RenderBuffer::~RenderBuffer()
{
   clean_up() ;
}

//---------------- FRAMEBUFFER OBJECTS IN GL EXTENSION ------------------

#elif defined(GL_EXT_framebuffer_object)
//#warning "framebuffer objects are a GL extension"

#ifndef INVT_HAVE_LIBGLEW

RenderBuffer::RenderBuffer(int, int)
   : m_width(0), m_height(0),
     m_size(0), m_cache(0), m_dirty(true),
     m_fbo(0), m_rbo(0)
{
   throw missing_libs(MISSING_GLEW) ;
}

void RenderBuffer::setup(){}
const unsigned char* RenderBuffer::pixels() const {return 0 ;}
void RenderBuffer::to_screen(){}
void RenderBuffer::clean_up(){}
RenderBuffer::~RenderBuffer(){}

#else // OpenGL extension wrangler library (GLEW) is available

// Initialize off-screen rendering buffer of specified size
RenderBuffer::RenderBuffer(int W, int H)
   : m_width(W), m_height(H),
     m_size(W * H * 4), m_cache(new unsigned char[m_size]), m_dirty(true)
{
   if (glewInit() != GLEW_OK) {
      delete[] m_cache ;
      throw misc_error(GLEW_INIT_ERROR) ;
   }

   if (GLEW_EXT_framebuffer_object)
   {
      LRB_TRACE("initializing off-screen rendering using GL extension API...");

      glGenFramebuffersEXT(1, &m_fbo) ;
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo) ;

      glGenRenderbuffersEXT(1, &m_rbo) ;
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, m_rbo) ;
      glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8,
                               m_width, m_height) ;
      if (glGetError() != GL_NO_ERROR) {
         clean_up() ;
         throw misc_error(OPENGL_FBO_INIT_ERROR) ;
      }

      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                   GL_COLOR_ATTACHMENT0_EXT,
                                   GL_RENDERBUFFER_EXT, m_rbo) ;

      GLenum fbo_status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT) ;
      if (fbo_status != GL_FRAMEBUFFER_COMPLETE_EXT) {
         clean_up() ;
         throw misc_error(OPENGL_FBO_INIT_ERROR) ;
      }
   }
   else
      LRB_TRACE("GL driver lacks FBO support ==> no off-screen rendering!") ;
   std::fill_n(m_cache, m_size, 0) ;
}

// Setup off-screen rendering
void RenderBuffer::setup()
{
   if (GLEW_EXT_framebuffer_object)
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo) ;
   m_dirty = true ;
}

// Retrieve off-screen buffer's pixel data
//
// NOTE: If GL doesn't support framebuffer objects, we will simply read
// the back buffer, which can cause bogus screen captures, e.g., window
// obscured by another. But, unfortunately, that's the best we can do
// with GLUT. If we were using GLX instead, we could have tested for the
// pbuffer extension and used that...
const unsigned char* RenderBuffer::pixels() const
{
   if (m_dirty)
   {
      if (GLEW_EXT_framebuffer_object)
      {
         LRB_TRACE("retrieving off-screen buffer's pixel data...") ;
         glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo) ;
      }
      else
      {
         LRB_TRACE("retrieving back buffer's pixel data...") ;
         glReadBuffer(GL_BACK) ;
      }
      glReadPixels(0, 0, m_width, m_height,
                   GL_BGRA, GL_UNSIGNED_BYTE, m_cache) ;
      if (glGetError() == GL_NO_ERROR)
         m_dirty = false ;
      else
         std::fill_n(m_cache, m_size, 0) ;
   }
   else
      LRB_TRACE("returning cached off-screen buffer pixel data...") ;
   return m_cache ;
}

// Copy contents of off-screen buffer to the back buffer
//
// DEVNOTE: The blit code is suspect! It was never run on a host where
// the GL driver supported blitting.
void RenderBuffer::to_screen()
{
#ifdef GL_EXT_framebuffer_blit
//#warning "FBO extension supports blitting"
   if (GLEW_EXT_framebuffer_blit)
   {
      LRB_TRACE("blitting off-screen buffer to back buffer...") ;

      glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, m_fbo) ;
      glReadBuffer(GL_COLOR_ATTACHMENT0_EXT) ;

      glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, 0) ;
      glDrawBuffer(GL_BACK) ;

      glBlitFramebufferEXT(0, 0, m_width, m_height,
                           0, 0, m_width, m_height,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST) ;
   }
   else
#else
//#warning "FBO extension does not support blitting"
#endif
   if (GLEW_EXT_framebuffer_object)
   {
      LRB_TRACE("copying pixels from off-screen buffer to back buffer...") ;

      // Read pixels from off-screen buffer
      pixels() ; // stuffs pixel data into m_cache

      // Copy above pixels to GL back (on-screen) buffer
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0) ;
      glViewport(0, 0, m_width, m_height) ;

      // Since pixel copying will go through the usual graphics pipeline,
      // we should reset the projection and modelview matrices so that
      // the matrices used for rendering to the off-screen buffer don't
      // mess up the scene that finally gets shown on-screen.
      glMatrixMode(GL_PROJECTION) ;
      glPushMatrix() ;
      glLoadIdentity() ;
      glOrtho(0, m_width, 0, m_height, -1, 1) ;

      glMatrixMode(GL_MODELVIEW) ;
      glPushMatrix() ;
      glLoadIdentity() ;
      glRasterPos2i(0, 0) ;

      glDrawPixels(m_width, m_height, GL_BGRA, GL_UNSIGNED_BYTE, m_cache) ;

      // Restore matrices so that clients can remain blissfully unaware
      // of the fact that they are actually rendering to an off-screen
      // buffer...
      glMatrixMode(GL_PROJECTION) ;
      glPopMatrix() ;
      glMatrixMode(GL_MODELVIEW) ;
      glPopMatrix() ;
   }
}

// Clean-up
void RenderBuffer::clean_up()
{
   if (GLEW_EXT_framebuffer_object)
   {
      LRB_TRACE("cleaning up off-screen buffer related GL resources") ;

      glBindFramebufferEXT (GL_FRAMEBUFFER_EXT,  0) ;
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0) ;

      glDeleteRenderbuffersEXT(1, &m_rbo) ;
      glDeleteFramebuffersEXT (1, &m_fbo) ;
   }

   delete[] m_cache ; m_cache = 0 ;
}

RenderBuffer::~RenderBuffer()
{
   clean_up() ;
}

#endif // #ifndef INVT_HAVE_LIBGLEW

//----------------- FRAMEBUFFER OBJECTS NOT AVAILABLE -------------------

#else // GL does not support framebuffer objects
//#warning "GL does not support framebuffer objects"

// Initialization: since we don't have off-screen rendering, all
// "off-screen" rendering is in fact simply redirected to the GL back
// (on-screen) buffer.
RenderBuffer::RenderBuffer(int W, int H)
   : m_width(W), m_height(H),
     m_size(W * H * 4), m_cache(new unsigned char[m_size]), m_dirty(true),
     m_fbo(0), m_rbo(0)
{
   LRB_TRACE("no compile or run-time off-screen rendering support in GL") ;
   std::fill_n(m_cache, m_size, 0) ;
}

// Nothing to setup because all rendering is on-screen
void RenderBuffer::setup(){}

// Retrieve pixel data from back buffer
//
// NOTE: This can be problematic due to pixel ownership tests. For
// example, if the Robolocust window is partially obscured by another,
// the screen capture can be corrupt. But without framebuffer objects, we
// have little recourse short of switching from GLUT to GLX and trying
// our luck with something like the pbuffer extension...
const unsigned char* RenderBuffer::pixels() const
{
   if (m_dirty)
   {
      LRB_TRACE("retrieving back buffer's pixel data...") ;
      glReadBuffer(GL_BACK) ;
      glReadPixels(0, 0, m_width, m_height,
                   GL_BGRA, GL_UNSIGNED_BYTE, m_cache) ;
      if (glGetError() == GL_NO_ERROR)
         m_dirty = false ;
      else
         std::fill_n(m_cache, m_size, 0) ;
   }
   else
      LRB_TRACE("returning cached off-screen buffer pixel data...") ;
   return m_cache ;
}

// Nothing to do for copying off-screen buffer to on-screen buffer
// because all rendering is already taking place in on-screen buffer.
void RenderBuffer::to_screen(){}

// Clean-up
void RenderBuffer::clean_up(){}

RenderBuffer::~RenderBuffer()
{
   delete[] m_cache ;
}

#endif // defined(GL_EXT_framebuffer_object)

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef INVT_HAVE_LIBGL

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
