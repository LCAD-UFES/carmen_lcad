/**
   \file  Robots/LoBot/ui/LoMainWindow.H
   \brief The Lobot/Robolocust main window.

   This file defines a class that encapsulates the lobot UI's main
   window. All of the different parts of lobot (such as input image
   streams, locust models, integration algorithms, etc.) are associated
   with corresponding drawables that are responsible for rendering their
   respective "source" objects on the main window.

   Each drawable is assigned an area within the main window and is
   expected to render itself within that area. The geometry
   specifications in the lobot configuration file determine a drawable's
   rendering area.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoMainWindow.H $
// $Id: LoMainWindow.H 13967 2010-09-18 08:00:07Z mviswana $
//

#ifndef LOBOT_MAIN_WINDOW_DOT_H
#define LOBOT_MAIN_WINDOW_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"

#include "Robots/LoBot/thread/LoMutex.H"
#include "Robots/LoBot/thread/LoThread.H"

#include "Robots/LoBot/misc/singleton.hh"

// Standard C++ headers
#include <string>
#include <queue>
#include <list>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class RenderBuffer ;

/**
   \class lobot::MainWindow
   \brief The Lobot/Robolocust main window.

   This class implements the functionality of the Robolocust/lobot UI. It
   pops up a GLUT window on the screen and manages the rendering of all
   the drawables created by the main program and other parts of
   Robolocust. All Robolocust drawables are expected to render themselves
   using OpenGL. Rendering takes place in a separate thread.

   The main window is responsible for managing the visualization of the
   laser range finder, the LGMD spike trains and each of the behaviours
   and DAMN arbiters. Each drawable element is be assigned an area within
   the main window based on the geometry specifications in the individual
   sections of the Robolocust config file.
*/
class MainWindow : public singleton<MainWindow>, private Thread {
   // Boilerplate code to make generic singleton pattern work
   friend class singleton<MainWindow> ;

   /// This class uses GLUT to help visualize the different parts of the
   /// Robolocust program. This member variable holds the ID of the GLUT
   /// window.
   int m_window ;

   /// Instead of rendering directly to the screen, the Robolocust UI
   /// first renders to an off-screen buffer and then copies that buffer
   /// to the screen. This indirect approach to rendering helps with
   /// screen captures.
   RenderBuffer* m_render_buffer ;

   /// The Robolocust UI uses the geometry specifications of the
   /// individual drawables to compute the dimensions of its main window
   /// and always resizes itself to that exact size. These data members
   /// are used to keep track of this "ideal" size.
   int m_width, m_height ;

   /// The main window manages all of the rendering related tasks by
   /// maintaining a list of drawable objects. However, instead of a
   /// plain list, we use a map that associates drawable names to the
   /// drawables. This map is useful in situations where one drawable
   /// needs to render something in another drawable's area.
   //@{
   typedef std::list<Drawable*> Drawables ;
   Drawables m_drawables ;
   //@}

   /// Because the main thread will add drawables to the main window's
   /// drawables list while the main window's thread uses it during
   /// rendering, we need to synchronize accesses to the above data
   /// structure.
   ///
   /// DEVNOTE: This member is declared mutable so that it can be passed
   /// to the pthread mutex functions without requiring a cast when used
   /// from const member functions.
   mutable Mutex m_drawables_mutex ;

   /// We also need a mutex to protect the m_window variable because the
   /// main window is responsible for implementing part of the two-step
   /// initialization sequence for drawables.
   ///
   /// Each drawable may specify OpenGL related initialization in its
   /// gl_init() function. Drawables should not call any OpenGL
   /// functions in their constructors because it is possible that the
   /// GL rendering context may not yet be up when that drawable is
   /// created.
   ///
   /// Since only the main window really knows when GL is ready, it
   /// makes sense for this class to invoke Drawable::gl_init() for each
   /// member of the drawables list. Usually, this would happen right
   /// after the creation of the GL window but before the GL message
   /// loop begins. However, some drawables may be added after the GL
   /// message loop has commenced. We have to trigger gl_init() for
   /// these Johnny-come-latelies just before they get added to the
   /// drawables list by checking if m_window denotes a valid GL window
   /// or not. Hence the need for this mutex.
   Mutex m_window_mutex ;

   /// This inner class encapsulates the pixel data returned by the
   /// off-screen render buffer and is used in conjunction with the
   /// Robolocust UI's screen capture facility.
   class ScreenCapture {
      std::string m_name ; // file name
      const int m_width, m_height ; // dimensions
      std::vector<unsigned char> m_data ; // pixel data

   public:
      /// This constructor is used when we want to save a single frame to
      /// a file of the specified name (screenshots taken by user).
      ScreenCapture(const std::string& file_name, int width, int height,
                    const unsigned char* buf, int bufsiz) ;

      /// This constructor is meant to be used when automatic, continuous
      /// screen captures are on (for later encoding to a movie).
      ScreenCapture(int frame_number, int width, int height,
                    const unsigned char* buf, int bufsiz) ;

      /// This method saves the captured frame to the file whose name was
      /// either supplied directly by the client (first constructor,
      /// single shot mode) or derived from the frame number (second
      /// constructor, continuous capture mode).
      void save() const ;
   } ;

   /// Since writing a screen capture frame to disk can take a while and
   /// hold up the visualization thread's rendering and user interaction
   /// workflow, we refrain from saving these frames right after getting
   /// the pixel data from the off-screen render buffer. Instead, we put
   /// the frame into a queue and then, when the program is idling, take
   /// the next pending frame and write that out.
   std::queue<ScreenCapture*> m_capture_queue ;

   /// Each screen capture frame will be named like so: frame000.png,
   /// frame001.png, frame002.png, and so on. This data member keeps
   /// track of the next index number for the frame names.
   int m_frame_number ;

   /// Private constructor because the main window is a singleton.
   MainWindow() ;

   /// Since the Robolocust UI uses GLUT (which implements its own main
   /// loop), we need to run it in a separate thread from the rest of the
   /// Robolocust system.
   void run() ;

public:
   /// This method adds a drawable to the main window.
   void push_back(Drawable*) ;

   // STL compatibility
   typedef Drawables::const_reference const_reference ;

private:
   /// Since GLUT runs its own main loop, independent of the rest of the
   /// Robolocust system, we cannot update the visualization as part of
   /// the lobot::App object's main loop. Instead, we use a GLUT timer to
   /// trigger updates. This method sets up the update timer.
   void setup_timer() ;

   /// This method is called every time the main window's GLUT update
   /// timer fires. It simply invalidates the GLUT window in order to
   /// trigger a rendering operation.
   void update() ;

   /// This method renders all the drawables currently connected to the
   /// main window.
   void render() ;

   /// This method saves the next pending frame in the screen capture
   /// queue. It is invoked from the idle handler so as to not tie up the
   /// main visualization thread.
   void dump_next_frame() ;

public:
   /// This function can be used to save a screenshot of the Robolocust
   /// UI to the specified file.
   void save_screenshot(const std::string& file_name) const ;

private:
   /// This method responds to UI window resize events.
   void reshape(int width, int height) ;

   /// These methods respond to different key presses. We use a dispatch
   /// table to "route" key presses to the appropriate handler.
   //{@
   void handle_key(unsigned char key) ;

   typedef void (MainWindow::*KeyHandler)() ;
   typedef std::map<unsigned char, KeyHandler> KeyMap ;
   KeyMap m_keymap ; // dispatch table

   void reset_zoom_pan() ;
   void pause() ;
   void quit() ;
   //@}

   /// These functions and variables take care of different mouse events.
   //@{
   int m_drag_button ;
   int m_drag_modifiers ;
   int m_drag_prev[2] ;

   void left_click  (int state, int modifiers, int x, int y) ;
   void middle_click(int state, int modifiers, int x, int y) ;
   void right_click (int state, int modifiers, int x, int y) ;

   void left_drag  (int x, int y) ;
   void middle_drag(int x, int y) ;
   void right_drag (int x, int y) ;
   //@}

   /// GLUT callbacks
   //@{
   static void reshape_callback(int width, int height) ;
   static void render_callback() ;
   static void keyboard_callback(unsigned char key, int mouse_x, int mouse_y) ;
   static void click_callback(int button, int state, int x, int y) ;
   static void drag_callback(int x, int y) ;
   static void timer_callback(int timer_id) ;
   static void idle_callback() ;
   //@}

public:
   /// Destroy the main window.
   ~MainWindow() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
