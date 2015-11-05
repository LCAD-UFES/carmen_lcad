/**
   \file  Robots/LoBot/ui/LoMainWindow.C
   \brief The Lobot/Robolocust main window.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoMainWindow.C $
// $Id: LoMainWindow.C 13967 2010-09-18 08:00:07Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case OpenGL and/or GLUT are missing
//
// NOTE: Don't really need to check INVT_HAVE_LIBGL and INVT_HAVE_LIBGLU
// as well because it ought to be a pretty rare/broken installation that
// has GLUT but not the OpenGL libraries...
#ifndef INVT_HAVE_LIBGLUT

#include "Robots/LoBot/ui/LoMainWindow.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

MainWindow::MainWindow()
{
   throw missing_libs(MISSING_OPENGL) ;
}

// NOTE: Don't need empty definitions of the remaining member functions
// because they don't get used at all if OpenGL and/or GLUT are missing,
// which means that the linker won't complain about missing functions
// (since the compiler doesn't generate code for these functions).

} // end of namespace encapsulating above empty definition

#else // OpenGL and GLUT available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoMainWindow.H"
#include "Robots/LoBot/ui/LoRenderBuffer.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoPause.H"

#include "Robots/LoBot/util/LoFile.H"
#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoTime.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"

// INVT utilities
#include "Util/log.H"

// DevIL headers
#ifdef INVT_HAVE_LIBDEVIL
#include <IL/il.h>
#endif

// OpenGL headers
#include <GL/glut.h>

// Boost headers
#include <boost/bind.hpp>

// Standard C++ headers
#include <iomanip>
#include <sstream>
#include <string>
#include <algorithm>
#include <functional>

// System/Unix headers
#include <sys/stat.h> // for mkdir

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

/// This inner class encapsulates various parameters that can be used to
/// tweak different aspects of the Robolocust UI and the visualizations.
class Params : public singleton<Params> {
   /// Private constructor because this is a singleton.
   Params() ;
   friend class singleton<Params> ;

   /// Users can specify whatever window title they want for the
   /// Robolocust UI.
   std::string m_title ;

   /// The Robolocust UI supports taking screenshots of itself after each
   /// iteration of the render cycle and then saving these screenshots as
   /// JPEG or PNG files. By default, the screen capture functionality is
   /// off. This flag turns it on.
   bool m_screen_capture ;

   /// When screen capturing is turned on, Robolocust will write each
   /// frame to the directory specified by this setting. A time-stamp
   /// corresponding to when the Robolocust application was launched will
   /// be automatically to appended to this setting's value. The format
   /// of the time-stamp is "YYYYmmdd-HHMMSS". Thus, if the value of this
   /// setting is "/tmp/foo-" and the application was launched at
   /// midnight on January 1st, 2000, all frames will be written to
   /// "/tmp/foo-20000101-000000".
   std::string m_sc_dir ;

   /// Each frame saved by the screen capturing process will be named
   /// frame000.png, frame001.png, frame002.png, and so on. This setting
   /// specifies the number of digits in the numeric part of the frame
   /// name. The default is six digits. Thus, by default, frames will be
   /// named frame000000.png, frame000001.png, and so on.
   int m_sc_len ;

   /// Robolocust uses the OpenIL library (aka libdevil) to save frames
   /// to disk. Thus, it can write the individual frames in any of the
   /// file formats supported by OpenIL. This setting specifies the
   /// format to use for the individual screen capture frames. It should
   /// be a 3-letter string such "png", "jpg", "pnm", "tif", etc.
   ///
   /// NOTE: Depending on how OpenIL is compiled and installed, some
   /// image file formats may not be supported. Generally, it is best to
   /// stick to the "png" or "jpg" formats. By default, Robolocust saves
   /// its frames as PNG files.
   std::string m_sc_fmt ;

   // A flag to check if the screen capture file format is PNG or not.
   bool m_sc_png ;

   /// This setting specifies the initial zoom level for drawables that
   /// support zoom/pan operations. Its value should be a floating point
   /// number. Here's how it works: a value of 1 (the default) will
   /// result in things being shown as-is. Fractional values zoom out
   /// the drawables, i.e., make it smaller; for example, a value of 0.5
   /// would show the drawable in half-size. Values greater than unity
   /// zoom into the drawable, e.g., 2 would double the drawable's size.
   float m_initial_zoom ;

   /// We can speed up or slow down the zoom by adjusting this factor.
   /// Higher values will result in amplifying mouse motion so that
   /// even a small movement results in a large zoom in or out; lower
   /// values will damp the mouse motion so that more dragging is
   /// required to achieve the desired zoom level.
   float m_zoom_drag_factor ;

   /// This setting specifies the frequency with which the Robolocust UI
   /// is updated. It is expected to be a time expressed in milliseconds.
   /// Thus, for some value N, the update will be performed once every N
   /// milliseconds.
   int m_update_frequency ;

public:
   // Accessing the various parameters
   //@{
   static const std::string& title()  {return instance().m_title  ;}
   static const std::string& sc_dir() {return instance().m_sc_dir ;}
   static const std::string& sc_fmt() {return instance().m_sc_fmt ;}
   static int   sc_len()              {return instance().m_sc_len ;}
   static bool  sc_png()              {return instance().m_sc_png ;}
   static bool  screen_capture()      {return instance().m_screen_capture   ;}
   static float initial_zoom()        {return instance().m_initial_zoom     ;}
   static float zoom_drag_factor()    {return instance().m_zoom_drag_factor ;}
   static int   update_frequency()    {return instance().m_update_frequency ;}
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_title(ui_conf<std::string>("title", "Robolocust")),
     m_screen_capture(ui_conf("screen_capture", false)),
     m_sc_dir(ui_conf<std::string>("screen_capture_dir", "/tmp/lobot-frames-")
                 + startup_timestamp_str()),
     m_sc_len(clamp(ui_conf("screen_capture_len", 6), 3, 9)),
     m_sc_fmt(downstring(ui_conf<std::string>("screen_capture_fmt", "png"))),
     m_sc_png(m_sc_fmt == "png"),
     m_initial_zoom(clamp(ui_conf("initial_zoom", 1.0f), 0.1f, 5.0f)),
     m_zoom_drag_factor(clamp(ui_conf("zoom_drag_factor", 0.1f), 0.01f, 2.5f)),
     m_update_frequency(clamp(ui_conf("update_frequency", 250), 100, 60000))
{
   if (m_screen_capture && (mkdir(m_sc_dir.c_str(), 0755) != 0))
      LERROR("failed to create directory \"%s\"", m_sc_dir.c_str()) ;
}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

// Since the Robolocust UI relies on GLUT (which implements its own main
// loop), we need to run it in a separate thread from the rest of the
// Robolocust system. Furthermore, since GLUT implements its own main
// loop, this thread will never return. Therefore, we need to specify
// that this thread's run() method won't return when starting up the
// thread by passing false to the second parameter of Thread::start().
MainWindow::MainWindow()
   : m_window(0), m_render_buffer(0), m_width(-1), m_height(-1),
     m_frame_number(0),
     m_drag_button(-1), m_drag_modifiers(-1)
{
   start("lobot_ui", false) ;
}

// Helper function object to compute the total size of the main window
// using the geometry specifications of all its drawables. It works by
// computing the union of all the rectangles described by the geometry
// specs.
namespace {

// DEVNOTE: This function object is meant to be used with the STL
// for_each algorithm. Unlike the usual invocation of for_each, wherein
// we simply discard for_each's return value, when using this function
// object, for_each's return value should be used to retrieve the UI
// dimensions computed and stored by this function object.
class calc_size {
   int left, right, bottom, top ;
public:
   calc_size() ;
   void operator()(Drawable*) ;

   int width()  const {return right - left ;}
   int height() const {return bottom - top ;}
} ;

calc_size::calc_size()
   : left  (std::numeric_limits<int>::max()),
     right (std::numeric_limits<int>::min()),
     bottom(std::numeric_limits<int>::min()),
     top   (std::numeric_limits<int>::max())
{}

void calc_size::operator()(Drawable* d)
{
   if (d->visible()) {
      Drawable::Geometry g = d->geometry() ;
      left   = std::min(left,   g.x) ;
      right  = std::max(right,  g.x + g.width) ;
      bottom = std::max(bottom, g.y + g.height) ;
      top    = std::min(top,    g.y) ;
   }
}

} // end of local namespace encapsulating above helper function object

// The UI thread's run method will initialize GLUT, perform some other
// rendering set up operations and then enter the GLUT main loop.
void MainWindow::run()
{
   try
   {
      App::wait_for_init() ;

      int argc = App::argc() ;
      glutInit(& argc, const_cast<char**>(App::argv())) ;
      glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE) ;

#ifdef INVT_HAVE_LIBDEVIL
      ilInit() ;
#endif

      // Inner block for window creation to ensure that its associated
      // mutex is released as soon as it is no longer required.
      {
         AutoMutex M(m_window_mutex) ;
         m_window = glutCreateWindow(Params::title().c_str()) ;
      }

      typedef MainWindow me ; // typing shortcut
      m_keymap['r'] = & me::reset_zoom_pan ;
      m_keymap['p'] = & me::pause ;
      m_keymap['q'] = & me::quit ;
      m_keymap['Q'] = & me::quit ;
      m_keymap[27]  = & me::quit ; // ESC (assuming ASCII encoding)
      m_keymap[ 3]  = & me::quit ; // Ctrl-C (assuming ASCII encoding)
      m_keymap[ 4]  = & me::quit ; // Ctrl-D (assuming ASCII encoding)
      m_keymap[17]  = & me::quit ; // Ctrl-Q (assuming ASCII encoding)
      m_keymap[24]  = & me::quit ; // Ctrl-X (assuming ASCII encoding)

      m_drag_prev[0] = m_drag_prev[1] = -1 ;

      calc_size s =
         std::for_each(m_drawables.begin(), m_drawables.end(), calc_size()) ;
      m_width  = s.width()  ;
      m_height = s.height() ;

      m_render_buffer = new RenderBuffer(m_width, m_height) ;

      glutReshapeFunc(reshape_callback) ;
      glutDisplayFunc(render_callback) ;
      glutKeyboardFunc(keyboard_callback) ;
      glutMouseFunc(click_callback) ;
      glutMotionFunc(drag_callback) ;
      glutIdleFunc(idle_callback) ;
      setup_timer() ;

      // Now that the GL rendering context is up, we should trigger the GL
      // initializations for all the drawables.
      //
      // NOTE: Inner block to ensure mutex release ASAP.
      {
         AutoMutex M(m_drawables_mutex) ;
         std::for_each(m_drawables.begin(), m_drawables.end(),
                       std::mem_fun(& Drawable::gl_init)) ;
      }

      glutMainLoop() ;
   }
   catch (std::exception& e)
   {
      LERROR("%s", e.what()) ;
      quit() ;
   }
}

// Since the Robolocust UI operates in a separate thread, the main thread
// only needs to setup the main window object by adding drawables to it.
// After that, the UI works asynchronously w.r.t. and independently of
// the main thread.
void MainWindow::push_back(Drawable* d)
{
   if (d)
   {
      // Before adding a new drawable to the drawables list and starting
      // its rendering cycles, we should perform any GL related
      // initialization that drawable might have. But only if the GL
      // rendering context is ready for action; otherwise: segfault.
      //
      // NOTE: Inner block to ensure window mutex is released as soon as
      // it is no longer required.
      {
         AutoMutex W(m_window_mutex) ;
         if (m_window) // GL rendering context is up and running
            d->gl_init() ;
      }

      AutoMutex D(m_drawables_mutex) ;
      m_drawables.push_back(d) ;
   }
}

//----------------------------- RENDERING -------------------------------

// Setup update timer
void MainWindow::setup_timer()
{
   glutTimerFunc(Params::update_frequency(), timer_callback, 0) ;
}

// When update timer fires, invalidate GLUT window in order to trigger
// main window's rendering operation.
void MainWindow::update()
{
   glutPostRedisplay() ;
   if (Pause::is_clear())
      setup_timer() ;
}

// Helper function object for setting up a drawable's drawing area and
// then rendering it.
namespace {

class render_drawable {
   int W, H ;
public:
   render_drawable(int ui_width, int ui_height) ;
   void operator()(Drawable*) const ;
} ;

render_drawable::render_drawable(int w, int h)
   : W(w), H(h)
{}

void render_drawable::operator()(Drawable* d) const
{
   if (d->invisible())
      return ;

   Drawable::Geometry g = d->geometry() ;
   glViewport(g.x, H - (g.y + g.height), g.width, g.height) ;

   d->render() ;

   // Draw a border to demarcate the drawable's drawing area
   if (d->border()) {
      glMatrixMode(GL_PROJECTION) ;
      glPushMatrix() ;
      glLoadIdentity() ;
      gluOrtho2D(0, g.width, 0, g.height) ;

      glMatrixMode(GL_MODELVIEW) ;
      glPushMatrix() ;
      glLoadIdentity() ;

      glPushAttrib(GL_CURRENT_BIT) ;
         glColor3fv(d->border_color().rgb()) ;
         glBegin(GL_LINE_LOOP) ;
            glVertex2i(1, 1) ;
            glVertex2i(g.width - 1, 1) ;
            glVertex2i(g.width - 1, g.height - 1) ;
            glVertex2i(1, g.height - 1) ;
         glEnd() ;
      glPopAttrib() ;

      glMatrixMode(GL_PROJECTION) ;
      glPopMatrix() ;
      glMatrixMode(GL_MODELVIEW) ;
      glPopMatrix() ;
   }
}

} // end of local namespace encapsulating above helper

// Render all the drawables: first to the off-screen buffer and then to
// the on-screen buffer.
void MainWindow::render()
{
   m_render_buffer->setup() ;

   static bool first_time = true ;
   if (first_time)
   {
      reset_zoom_pan() ;
      AutoMutex M(m_drawables_mutex) ;
      std::for_each(m_drawables.begin(), m_drawables.end(),
                    std::bind2nd(std::mem_fun(& Drawable::zoom_by),
                                 Params::initial_zoom() - 1)) ;
      first_time = false ;
   }

   glClear(GL_COLOR_BUFFER_BIT) ;

   // Use block to ensure mutex is released when no longer needed
   {
      AutoMutex M(m_drawables_mutex) ;
      std::for_each(m_drawables.begin(), m_drawables.end(),
                    render_drawable(m_width, m_height)) ;
   }

   // If screen caturing is enabled, retrieve the pixel data from render
   // buffer and queue it for later writing in the GLUT idle handler so
   // that rendering and user interaction continue to take precedence.
   if (Params::screen_capture())
      m_capture_queue.push(new ScreenCapture(m_frame_number++,
                                             m_width, m_height,
                                             m_render_buffer->pixels(),
                                             m_render_buffer->size())) ;

   m_render_buffer->to_screen() ;
   glutSwapBuffers() ;
}

//------------------------- SCREEN CAPTURING ----------------------------

// Save a screenshot to the specified file
void MainWindow::save_screenshot(const std::string& file_name) const
{
   ScreenCapture sc(file_name, m_width, m_height,
                    m_render_buffer->pixels(), m_render_buffer->size()) ;
   sc.save() ;
}

// Helper function to fix the alpha values passed by lobot::RenderBuffer
// so that they're all 255 (fully opaque). Otherwise, the OpenIL/DevIL
// library produces an image with a transparent background when saving as
// PNG.
//
// The function should be passed pointers to the start and end of the
// screen capture's data buffer.
static void fix_alpha_values(unsigned char* start, unsigned char* end)
{
   for (unsigned char* p = start + 3; p < end; p += 4)
      *p = 255 ;
}

// ScreenCapture constructor for saving single frames to client-supplied
// file names.
MainWindow::ScreenCapture::
ScreenCapture(const std::string& file_name,
              int w, int h, const unsigned char* data, int n)
   : m_name(file_name), m_width(w), m_height(h), m_data(data, data + n)
{
   if (downstring(extension(m_name)) == "png")
      fix_alpha_values(&m_data[0], &m_data[0] + n) ;
}

// ScreenCapture constructor for saving multiple frames with the file
// name being derived from the client-supplied frame number.
MainWindow::ScreenCapture::
ScreenCapture(int frame_number, int w, int h, const unsigned char* data, int n)
   : m_width(w), m_height(h), m_data(data, data + n)
{
   using std::setfill ; using std::setw ;
   std::ostringstream file_name ;
   file_name << Params::sc_dir() << "/frame"
             << setfill('0') << setw(Params::sc_len()) << frame_number
             << '.' << Params::sc_fmt() ;
   m_name = file_name.str() ;

   if (Params::sc_png())
      fix_alpha_values(&m_data[0], &m_data[0] + n) ;
}

// This function uses the OpenIL/DevIL library to save a frame to disk.
// If OpenIL/DevIL is not installed, we throw an exception so that the
// first attempt at saving a frame will result in the application
// quitting with a reasonable error message to inform the user of what
// went wrong.
void MainWindow::ScreenCapture::save() const
{
#ifdef INVT_HAVE_LIBDEVIL
   ILuint image ;
   ilGenImages(1, &image) ;
   ilBindImage(image) ;

   unsigned char* data = const_cast<unsigned char*>(&m_data[0]) ;
   ilTexImage(m_width, m_height, 1, 4, IL_BGRA, IL_UNSIGNED_BYTE, data) ;
   ilSaveImage(const_cast<char*>(m_name.c_str())) ;
   if (ilGetError() != IL_NO_ERROR)
      LERROR("error writing \"%s\"", m_name.c_str()) ;

   ilDeleteImages(1, &image) ;
#else
   throw missing_libs(MISSING_LIBDEVIL) ;
#endif
}

// To not tie up the visualization thread too much with screen capture
// saving, we queue captured frames and use GLUT's idling mechanism to
// periodically write these frames to disk. Thus, when the application is
// busy with rendering or user interaction, saving frames to disk will be
// put temporarily on hold.
//
// This function pops the next frame from the screen capture queue and
// saves it to disk.
void MainWindow::dump_next_frame()
{
   ScreenCapture* frame = m_capture_queue.front() ;
   m_capture_queue.pop() ;
   frame->save() ;
   delete frame ;
}

//--------------------------- WINDOW EVENTS -----------------------------

// The Robolocust UI decides its own size based on the geometry specs in
// the config file. We really don't want users messing around with the
// main window by resizing it. Therefore, we respond to such events by
// simply resizing the window back to its old/original size.
void MainWindow::reshape(int W, int H)
{
   if (W == m_width && H == m_height) // window is of "ideal" size
      return ;
   glutReshapeWindow(m_width, m_height) ;
}

//-------------------------- KEYBOARD INPUT -----------------------------

// Use keymap to invoke appropriate handler for key pressed by user
void MainWindow::handle_key(unsigned char key)
{
   // Main window gets dibs on key presses
   KeyMap::iterator handler = m_keymap.find(key) ;
   if (handler != m_keymap.end())
      (this->*(handler->second))() ;

   // It then lets each drawable take a crack at the event
   // NOTE: Use block to ensure mutex is released when no longer required
   {
      AutoMutex M(m_drawables_mutex) ;
      std::for_each(m_drawables.begin(), m_drawables.end(),
                    std::bind2nd(std::mem_fun(&Drawable::keypress), key)) ;
   }

   // Finally, when everyone is done handling the key press, we repaint
   glutPostRedisplay() ;
}

void MainWindow::reset_zoom_pan()
{
   AutoMutex M(m_drawables_mutex) ;
   std::for_each(m_drawables.begin(), m_drawables.end(),
                 std::mem_fun(& Drawable::reset_zoom_pan)) ;
}

void MainWindow::pause()
{
   Pause::toggle() ;
   if (Pause::is_clear())
      setup_timer() ;
}

void MainWindow::quit()
{
   AutoMutex M(m_drawables_mutex) ;
   std::for_each(m_drawables.begin(), m_drawables.end(),
                 std::mem_fun(& Drawable::gl_cleanup)) ;

   glutDestroyWindow(m_window) ;
   Shutdown::signal() ;
   pthread_exit(0) ;
}

//---------------------------- MOUSE INPUT ------------------------------

void MainWindow::left_click(int state, int modifiers, int x, int y)
{
   switch (state)
   {
      case GLUT_DOWN:
         m_drag_button    = GLUT_LEFT_BUTTON ;
         m_drag_modifiers = modifiers ;
         m_drag_prev[0]   = x ;
         m_drag_prev[1]   = y ;
         break ;
      case GLUT_UP:
         m_drag_button    = -1 ;
         m_drag_modifiers = -1 ;
         m_drag_prev[0]   = -1 ;
         m_drag_prev[1]   = -1 ;
         break ;
   }
}

void MainWindow::middle_click(int state, int modifiers, int x, int y)
{
   switch (state)
   {
      case GLUT_DOWN:
         m_drag_button    = GLUT_MIDDLE_BUTTON ;
         m_drag_modifiers = modifiers ;
         m_drag_prev[0]   = x ;
         m_drag_prev[1]   = y ;
         break ;
      case GLUT_UP:
         m_drag_button    = -1 ;
         m_drag_modifiers = -1 ;
         m_drag_prev[0]   = -1 ;
         m_drag_prev[1]   = -1 ;
         break ;
   }
}

void MainWindow::right_click(int state, int modifiers, int x, int y)
{
   switch (state)
   {
      case GLUT_DOWN:
         m_drag_button    = GLUT_RIGHT_BUTTON ;
         m_drag_modifiers = modifiers ;
         m_drag_prev[0]   = x ;
         m_drag_prev[1]   = y ;
         break ;
      case GLUT_UP:
         m_drag_button    = -1 ;
         m_drag_modifiers = -1 ;
         m_drag_prev[0]   = -1 ;
         m_drag_prev[1]   = -1 ;
         break ;
   }
}

void MainWindow::left_drag(int x, int y)
{
   if (m_drag_modifiers & GLUT_ACTIVE_SHIFT) // zoom
   {
      const float dz = (y - m_drag_prev[1]) * Params::zoom_drag_factor() ;
      AutoMutex M(m_drawables_mutex) ;
      std::for_each(m_drawables.begin(), m_drawables.end(),
                    std::bind2nd(std::mem_fun(& Drawable::zoom_by), -dz)) ;
   }
   else // pan
   {
      AutoMutex M(m_drawables_mutex) ;
      std::for_each(m_drawables.begin(), m_drawables.end(),
                    boost::bind(& Drawable::pan, _1,
                                x, y, m_drag_prev[0], m_drag_prev[1])) ;
   }

   m_drag_prev[0] = x ;
   m_drag_prev[1] = y ;

   glutPostRedisplay() ;
}

void MainWindow::middle_drag(int x, int y)
{
   // Block to ensure mutex is released when no longer required
   {
      const float dz = (y - m_drag_prev[1]) * Params::zoom_drag_factor() ;
      AutoMutex M(m_drawables_mutex) ;
      std::for_each(m_drawables.begin(), m_drawables.end(),
                    std::bind2nd(std::mem_fun(& Drawable::zoom_by), -dz)) ;
   }

   m_drag_prev[0] = x ;
   m_drag_prev[1] = y ;

   glutPostRedisplay() ;
}

void MainWindow::right_drag(int, int)
{
}

//-------------------------- GLUT CALLBACKS -----------------------------

void MainWindow::reshape_callback(int width, int height)
{
   instance().reshape(width, height) ;
}

void MainWindow::render_callback()
{
   instance().render() ;
}

void MainWindow::keyboard_callback(unsigned char key, int, int)
{
   instance().handle_key(key) ;
}

void MainWindow::click_callback(int button, int state, int x, int y)
{
   switch (button)
   {
      case GLUT_LEFT_BUTTON:
         instance().left_click(state, glutGetModifiers(), x, y) ;
         break ;
      case GLUT_MIDDLE_BUTTON:
         instance().middle_click(state, glutGetModifiers(), x, y) ;
         break ;
      case GLUT_RIGHT_BUTTON:
         instance().right_click(state, glutGetModifiers(), x, y) ;
         break ;
   }
}

void MainWindow::drag_callback(int x, int y)
{
   switch (instance().m_drag_button)
   {
      case GLUT_LEFT_BUTTON:
         instance().left_drag(x, y) ;
         break ;
      case GLUT_MIDDLE_BUTTON:
         instance().middle_drag(x, y) ;
         break ;
      case GLUT_RIGHT_BUTTON:
         instance().right_drag(x, y) ;
         break ;
   }
}

// We use a timer to continuously update the Robolocust UI because GLUT
// runs its own main loop independent of the rest of the Robolocust
// system (which means that we can't perform regular updates as part of
// lobot::App's main loop).
void MainWindow::timer_callback(int)
{
   instance().update() ;
}

// When there are no messages to respond to, check the application
// shutdown signal. This is useful for a clean termination of the
// Robolocust UI when the user switches input focus from the main window
// back to the terminal that launched the lobot program and terminates it
// using Ctrl-C or something similar.
//
// If the shutdown signal is not active, then take this opportunity to
// write the next screen capture frame to disk. Hopefully, doing this in
// the idle callback prevents the visualization thread from getting too
// bogged down handling screen captures.
void MainWindow::idle_callback()
{
   MainWindow& W = instance() ;
   if (Shutdown::signaled())
   {
      AutoMutex M(W.m_drawables_mutex) ;
      std::for_each(W.m_drawables.begin(), W.m_drawables.end(),
                    std::mem_fun(& Drawable::gl_cleanup)) ;

      glutDestroyWindow(W.m_window) ;
      pthread_exit(0) ;
   }
   if (! W.m_capture_queue.empty())
      W.dump_next_frame() ;
}

//----------------------------- CLEAN-UP --------------------------------

MainWindow::~MainWindow()
{
   delete m_render_buffer ;
   while (! m_capture_queue.empty())
      dump_next_frame() ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef INVT_HAVE_LIBGLUT

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
