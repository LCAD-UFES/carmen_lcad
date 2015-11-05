/**
   \file Robots/LoBot/ui/LoGLCanvas.H

   \brief 2D OpenGL canvas.

   This file defines a class that encapsulates window and viewport
   operations for doing 2D graphics in OpenGL. This gives clients a
   convenient, high-level means of doing things such as zoom and pan.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoGLCanvas.H $
// $Id: LoGLCanvas.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_OPENGL_CANVAS_DOT_H
#define LOBOT_OPENGL_CANVAS_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/range.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::GLCanvas
   \brief An encapsulation of window-viewport primitives for 2D graphics
   with OpenGL.

   This class implements some window and viewport operations for doing 2D
   graphics with OpenGL. Supported operations include zoom and pan.
*/
class GLCanvas {
   /// The main purpose of this canvas class is to encapsulate the window
   /// and viewport transformations for 2D graphics. These member
   /// variables take care of holding the current values of these
   /// parameters.
   //@{
public:
   enum {LEFT, RIGHT, BOTTOM, TOP} ;
private:
   float m_window[4] ;
   int   m_viewport[4] ;
   //@}

   /// The canvas supports zoom and pan operations.
   //@{
   float m_zoom_level ;
   //@}

public:
   /// Initialization.
   GLCanvas() ;

   /// This method sets up the world coordinate system.
   //@{
   void set_window(float left, float right, float bottom, float top) ;
   void set_window(const float window[]) {
      set_window(window[LEFT], window[RIGHT], window[BOTTOM], window[TOP]) ;
   }
   //@}

   /// This method sets up the graphics viewport with an aspect ratio
   /// matching that of the world coordinate system.
   void set_viewport(int left, int right, int bottom, int top) ;

   /// Retrieve the current GL viewport in effect.
   void get_viewport(int* x, int* y, int* width, int* height) ;

   /// Convert screen coordinates to world coordinates.
   void screen_to_world(int screen_x, int screen_y,
                        double* world_x, double* world_y) ;

   /// Zoom the scene contained within the canvas to the specified zoom
   /// level.
   void zoom_to(float zoom_level) ;

   /// Zoom the scene by the specified amount.
   void zoom_by(float delta) {zoom_to(m_zoom_level + delta) ;}

   /// Return the current zoom level.
   float zoom_level() const {return m_zoom_level ;}

   /// Pan the scene by the specified amount.
   void pan(float dx, float dy) ;

   /// Reset the zoom and pan parameters.
   void reset_zoom_pan() ;

   /// Clean-up.
   ~GLCanvas() ;

private:
   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of this OpenGL canvas
   class Params : public singleton<Params> {
      /// Private constructor because this is a singleton.
      Params() ;
      friend class singleton<Params> ;

      /// The min and max zoom levels that are allowed.
      range<float> m_zoom_range ;

   public:
      // Accessing the various parameters
      static const range<float>& zoom_range() ;

      // Clean-up
      ~Params() ;
   } ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
