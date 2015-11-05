/**
   \file  Robots/LoBot/LoApp.H
   \brief The Robolocust application object.

   This file defines a class that implements the lone application object
   for the Robolocust robot controller. This object is responsible for
   creating all the I/O objects, the different behaviours, etc. and for
   running the main thread.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LoApp.H $
// $Id: LoApp.H 13811 2010-08-21 02:00:08Z mviswana $
//

#ifndef LOBOT_APPLICATION_OBJECT_DOT_H
#define LOBOT_APPLICATION_OBJECT_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoCompositor.H"
#include "Robots/LoBot/thread/LoCondition.H"
#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/misc/singleton.hh"

// INVT model component support
#include "Component/ModelManager.H"
#include "Component/ModelParam.H"

// Standard C++ headers
#include <list>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class Behavior ;
class Map ;
class LocustModel ;

class LocustViz ;
class LaserViz ;
class LaserVizFlat ;

class Robot ;
class LaserRangeFinder ;
class InputSource ;
class VideoRecorder ;
class VideoStream ;

/**
   \class lobot::App
   \brief An application object that ties together various I/O and other
   modules and runs the main thread.

   This class is responsible for instantiating the various objects used
   to interface with the robot's sensors, motors, etc. It also creates
   the different behaviours that make up Robolocust's behaviour-based
   controller and runs the main thread's update loop that takes care of
   continously retrieving the latest sensor measurements, performing the
   necessary low-level state computations, etc.

   The application object is implemented as a singleton. This allows
   other modules/behaviours to easily retrieve references to I/O and
   other objects. Thus, the application object serves as a central
   repository that holds together the various low-level shared objects
   required by the different behaviours.
*/
class App : public singleton<App> {
   // Prevent copy and assignment
   App(const App&) ;
   App& operator=(const App&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<App> ;

public:
   /// One of the main responsibilities of the application object is to
   /// hold a bunch of low-level things together in one place so that
   /// they may be accessed by other modules and threads as required. To
   /// ease the interface between these low-level objects and high-level
   /// behaviours that need them, we use these typedefs.
   //@{
   typedef Compositor<PixelType>       ImageCompositor ;
   typedef std::vector<VideoStream*>   VideoStreams ;
   typedef std::vector<VideoRecorder*> VideoRecorders ;
   typedef std::vector<LocustModel*>   LocustModels ;
   typedef std::vector<Behavior*>      Behaviours ;
   //@}

private:
   /// These are different things the application object is responsible
   /// for creating, updating, or just having around handy so that other
   /// objects in the system can get to them conveniently.
   //@{
   int               m_argc ;
   const char**      m_argv ;
   VideoStreams      m_video_streams ;
   VideoRecorders    m_video_recorders ;
   ImageCompositor*  m_compositor ;
   LaserRangeFinder* m_lrf ;
   InputSource*      m_input_source ;
   LocustModels      m_locusts ;
   Robot*            m_robot ;
   Map*              m_map ;
   Behaviours        m_behaviours ;
   LaserViz*         m_laser_viz ;
   LaserVizFlat*     m_laser_viz_flat ;
   LocustViz*        m_locust_viz ;
   ModelManager      m_model_manager ;
   //@}

public:
   /// Return references/pointers to the things held centrally by the
   /// application object.
   //@{
   static int argc()                     {return instance().m_argc ;}
   static const char** argv()            {return instance().m_argv ;}
   static LaserRangeFinder* lrf()        {return instance().m_lrf  ;}
   static Robot*            robot()      {return instance().m_robot;}
   static Map*              map()        {return instance().m_map  ;}
   static const LocustModels& locusts()  {return instance().m_locusts    ;}
   static const Behaviours& behaviours() {return instance().m_behaviours ;}
   static LaserViz*         laser_viz()  {return instance().m_laser_viz  ;}
   static LaserVizFlat* laser_viz_flat() {return instance().m_laser_viz_flat;}
   //@}

private:
   /// Various command line options specific to the Robolocust program.
   OModelParam<std::string> m_cf_option ; // --config-file

   /// Return the name of the config file specified on the command line.
   std::string config_file() const {return m_cf_option.getVal() ;}

   /// A private constructor because the application object is a
   /// singleton.
   App() ;

public:
   /// A convenient alias for returning the application object's lone
   /// instance.
   static App& create(int argc, const char* argv[]) ;

   /// Parse the command line arguments.
   void parse_command_line() ;

   /// The application object has two main responsibilities. First, it
   /// must instantiate all the different I/O modules, create the
   /// different behaviours, etc. Second, it must run the Robolocust
   /// application's main loop. This method implements the
   /// afore-mentioned loop.
   ///
   /// The main loop runs the update cycle, which takes care of
   /// retrieving the latest measurements from the sensors, computing any
   /// low-level state based on these measurements, etc. The loop exits
   /// when the lobot::Shutdown object is signaled. Before exiting, the
   /// main thread waits for all the other threads to wind-up.
   void run() ;

private:
   /// Some Robolocust threads don't care whether the application object
   /// is fully initialized or not; some do. Those that do can use this
   /// condition variable to coordinate.
   //@{
   Condition m_initialized_cond ;
   bool      m_initialized ;
   //@}

   /// Helper classes for above condition variable.
   //@{
   struct signal_pred {
      bool operator()() ;
   } ;

   struct wait_pred {
      bool operator()() ;
   } ;

   friend struct signal_pred ;
   friend struct wait_pred ;
   //@}

   /// Set the initialized flag to let other threads know that the
   /// application object is fully loaded.
   void signal_init() ;

   /// Let other threads wait for the initialized flag to be signaled.
   void wait_init() ;

public:
   /// External API for other threads to wait until the application
   /// object is fully initialized.
   static void wait_for_init() {instance().wait_init() ;}

   /// Clean-up.
   ~App() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
