/**
   \file  Robots/LoBot/ui/LoDrawable.H
   \brief A base class for rendering things in the Robolocust UI.

   This file defines a base class meant to provide an interface for
   rendering the different things that get shown in the Robolocust main
   window.

   By default, lobot::Drawable's rendering related methods are all empty.
   Every Robolocust object that needs to present some sort of
   visualization must define this file's lobot::Drawable class as a base
   and override the render_me() method (and any other methods that are
   necessary) in order to draw itself.

   NOTE: Robolocust uses OpenGL for visualization. All objects that
   derive from lobot::Drawable and override its render_me() and other
   methods must, therefore, work in terms of the OpenGL API.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/ui/LoDrawable.H $
// $Id: LoDrawable.H 13967 2010-09-18 08:00:07Z mviswana $
//

#ifndef LOBOT_DRAWABLE_DOT_H
#define LOBOT_DRAWABLE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoMutex.H"
#include "Robots/LoBot/misc/LoTypes.H"

// Standard C++ headers
#include <string>
#include <list>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Drawable
   \brief A base class for Robolocust drawable objects.

   This class provides a common interface that allows the Robolocust main
   window to render each of the different Robolocust objects.

   All drawables will be assigned a drawing area within the main window
   and must render themselves inside that area. In some cases, a
   drawable's drawing area is computed by lobot::MainWindow. However, in
   most cases a drawable's drawing area is simply looked up from the
   lobot config file.

   DEVNOTE: By default, this class's methods are all virtual and empty.
   They are not pure virtual to ensure that leaf classes that don't care
   for visualization are not forced to define unnecessary empty methods.
   For example, lobot::SpeedArbiter does not present any visualization;
   however, its base class lobot::Arbiter does use lobot::Drawable as a
   base so as to allow lobot::TurnArbiter to present its visualization.

   Of course, we could simply have leaf classes that need to visualize,
   such as lobot::TurnArbiter, derive directly from lobot::Drawable and
   then declare all of lobot::Drawable's methods pure virtual. However,
   we choose not to do it this way because the main thread's lobot::App
   object would then have to work in terms of the leaf classes/objects.

   For the arbiters, that's not a big deal because the Robolocust system
   only uses two DAMN arbiters and they're both singletons.
   Unfortunately, the situation is not quite as clean when it comes to
   behaviours. If we were to require all the leaf classes to derive from
   lobot::Drawable, then lobot::App would have to downcast to each
   behaviour's instance and add that to lobot::MainWindow's list of
   drawables, which pretty much defeats the whole purpose of having a
   polymorphic factory for creating behaviours.

   Instead, by deriving lobot::Behavior from lobot::Drawable, we ensure
   that all behaviours are drawables and that the main thread need not
   concern itself with the exact type of each object it creates.
   Furthermore, a behaviour or other object that does not care to perform
   any visualization can rely on the empty virtual functions in
   lobot::Drawable instead of being forced to define empty functions.
*/
class Drawable {
   // Prevent copy and assignment
   Drawable(const Drawable&) ;
   Drawable& operator=(const Drawable&) ;

   /// Each drawable has a name that must be set by the derived class.
   std::string m_name ;

   /// Each drawable has a geometry specification read from the lobot
   /// config file. A geometry specification consists of the (x,y)
   /// coordinates of the drawable's top-left corner within the
   /// Robolocust main window and a [width, height] pair in pixels with
   /// width going to the right and height going down (exactly like X
   /// Window geometry specs).
   ///
   /// In some situations, we may want a drawable to be active, i.e.,
   /// added to the main window, so that it can process keyboard input
   /// but not visible, i.e., the drawable doesn't really draw anything
   /// (perhaps it simply renders overlays on other drawables). To make a
   /// drawable invisible, simply set its dimensions in its geometry to
   /// negative quantities.
   //@{
public:
   struct Geometry {
      int x, y ;
      int width, height ;

      /// Default constructor.
      Geometry(int x = 0, int y = 0, int width = -1, int height = -1) ;

      /// A convenience constructor for parsing a geometry spec passed in
      /// as a whitespace-separated string (e.g., a geometry spec read
      /// from the config file).
      Geometry(const std::string& geometry) ;
   } ;
protected:
   Geometry m_geometry ;
   //@}

   /// In addition to rendering itself, each drawable also supports
   /// rendering arbitrary overlays on top of itself. This is useful, for
   /// instance, with the open path behaviour, which likes to paint the
   /// candidate open paths on top of the LRF data visualization.
   ///
   /// Overlays are rendered via callback functions. Client modules
   /// register such a callback along with any client data they require
   /// to be passed back when the callback is triggered. The client data
   /// is always an unsigned long and can be used to pass object
   /// pointers that can then be dereferenced inside the callback to
   /// invoke a member function.
   ///
   /// These types are used to define the rendering callbacks.
   //@{
   typedef void (*RenderCB)(unsigned long client_data) ;
   typedef std::pair<RenderCB, unsigned long> RenderHook ;
   //@}
private:
   /// This data structure is used to hold all of a drawable's overlay
   /// rendering callbacks.
   //@{
   typedef std::list<RenderHook> Hooks ;
   Hooks m_hooks ;
   //@}

   /// Because overlays will often be added by drawbles running in other
   /// threads while the visualization thread reads the list of overlays
   /// during its rendering cycle, we need to synchronize accesses to the
   /// overlays list.
   Mutex m_hooks_mutex ;

   /// Since the rendering functions are invoked by the UI thread and
   /// often need to use state data stored in the objects being rendered,
   /// which usually belong to other threads (such as the arbiters and
   /// behaviours), all drawables will need some mechanism for
   /// coordinating access to the afore-mentioned state data. A mutex is
   /// just the thing we're looking for to solve this problem.
   ///
   /// The lobot::Drawable base class takes care of properly creating and
   /// destroying this mutex. It also provides an API for locking and
   /// unlocking this mutex. However, it cannot enforce the use of the
   /// mutex. Derived classes must do so as required at their discretion.
   Mutex m_viz_mutex ;

protected:
   /// By default, we always draw a border around each drawable to help
   /// clearly demarcate each of them. However, that may not be
   /// appropriate or desirable for some drawables. This flag can be used
   /// by derived classes to turn the border off on a case-by-case basis.
   bool m_border ;

   /// In addition to allowing derived classes to turn the drawable
   /// border on or off, we also allow them specify a suitable border
   /// color. The default border color is yellow.
   GLColor m_border_color ;

   /// A protected constructor because only derived classes can invoke
   /// it. And when they do, they must specify the drawable's name and
   /// its geometry.
   Drawable(const std::string&, const Geometry&) ;

public:
   /// Drawables should refrain from making any OpenGL calls until they
   /// can be absolutely sure that the GL rendering context is open for
   /// business. Unfortunately, a drawable cannot by itself determine
   /// when this happy situation might prevail. Thus, any OpenGL related
   /// initialization that a drawable might want to perform will have to
   /// be deferred to this function, which will be called by the
   /// lobot::MainWindow object when the time is right, viz., after the
   /// GL window is created but before the GL message loop commences
   /// and, for drawables created after the commencement of the GL
   /// message loop, just before they get added to the main window's
   /// drawables list.
   ///
   /// The default implementation of this method does nothing. Derived
   /// classes may override it if they need to.
   virtual void gl_init() ;

   /// Return this drawable's name.
   std::string name() const {return m_name ;}

   /// Return this drawable's geometry.
   Geometry geometry() const {return m_geometry ;}

   /// These functions check the visibility of this drawable. A drawable
   /// that is not visible will not take up any screen space within the
   /// Robolocust UI but can still be configured to actively receive and
   /// respond to keyboard input.
   //@{
   bool invisible() const {return !visible() ;}
   bool visible()   const {
      return m_geometry.width > 0 && m_geometry.height > 0 ;
   }
   //@}

   /// This method returns true if this drawable is configured to have a
   /// border drawn around its drawing area within the Robolocust UI.
   bool border() const {return m_border ;}

   /// This method returns the color in which the drawable's border
   /// should be rendered.
   const GLColor& border_color() const {return m_border_color ;}

   /// This function allows one drawable to add an overlay to another.
   void add_hook(const RenderHook&) ;

   /// This method renders the drawable itself plus any overlays.
   void render() ;

private:
   /// A helper function for triggering the rendering hooks.
   static void trigger_hook(const RenderHook&) ;

   /// This method renders the drawable and must be overridden in derived
   /// classes.
   virtual void render_me() ;

public:
   /// Drawables can support zoom/pan and keypress events.
   //@{
   virtual void zoom_by(float dz) ;
   virtual void pan(int curr_x, int curr_y, int prev_x, int prev_y) ;
   virtual void reset_zoom_pan() ;
   virtual void keypress(unsigned char key) ;
   //@}

protected:
   /// API for locking and unlocking the visualization mutex.
   ///
   /// As mentioned earlier, this class cannot actually enforce proper
   /// use of the visualization mutex. (Actually, it could by
   /// locking/unlocking before/after the action and render methods;
   /// however, that would be too brutal and wasteful as we may only need
   /// the locks in small parts of these methods.) Thus, it is up to each
   /// derived class to call these functions as and when required.
   //@{
   void viz_lock()   {m_viz_mutex.acquire() ;}
   void viz_unlock() {m_viz_mutex.release() ;}
   //@}

   /// Some helper methods.
   //@{
   void unit_view_volume() const ;
   void text_view_volume() const ;
   void setup_view_volume(float L, float R, float B, float T) const ;
   void restore_view_volume() const ;
   //@}

public:
   /// Just as drawables should refrain from performing GL
   /// initialization in their constructors (because the GL rendering
   /// context might not yet be available), they should also not perform
   /// any GL clean-up in their destructors just in case the rendering
   /// context is already gone.
   ///
   /// Instead, subclasses should override this method to perform OpenGL
   /// related clean-up. lobot::MainWindow will take care of triggering
   /// this function for each drawable just before rendering context is
   /// released. The default implementation of this method does nothing.
   virtual void gl_cleanup() ;

   /// Clean-up.
   virtual ~Drawable() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
