/**
   \file Robots/LoBot/LograbMain.C
   \brief Program to test custom multi-grabber for Lobot/Robolocust.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LograbMain.C $
// $Id: LograbMain.C 13905 2010-09-09 21:38:44Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoMainWindow.H"

#include "Robots/LoBot/io/LoCompositor.H"
#include "Robots/LoBot/io/LoVideoRecorder.H"
#include "Robots/LoBot/io/LoVideoStream.H"
#include "Robots/LoBot/io/LoFireWireBus.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/config/LoCommonOpts.H"
#include "Robots/LoBot/config/LoDefaults.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoThread.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/util/LoSTL.H"

// INVT image support
#include "Image/Dims.H"

// INVT model component support
#include "Component/ModelManager.H"
#include "Component/ModelParam.H"

// INVT utilities
#include "Util/log.H"

// Unix headers
#include <unistd.h>

// Standard C++ headers
#include <sstream>
#include <algorithm>
#include <iterator>
#include <list>
#include <vector>

//------------------------ APPLICATION OBJECT ---------------------------

// The following class wraps around the ModelManager and associated
// objects, providing a neatly encapsulated API for the main program.
namespace lobot {

struct LoApp {
   LoApp() ;
   void parse_command_line(int argc, const char* argv[]) ;
   void run() ;
   ~LoApp() ;

   // Some useful types
   typedef Compositor<PixelType>       ImageCompositor ;
   typedef std::vector<VideoStream*>   VideoStreams ;
   typedef std::vector<VideoRecorder*> VideoRecorders ;
   typedef std::list<Drawable*>        Drawables ;

   // Accessing command line arguments
   std::string config_file() const {return m_conf_option.getVal() ;}

private:
   VideoStreams    m_video_streams ;
   VideoRecorders  m_video_recorders ;
   ImageCompositor m_compositor ;
   Drawables       m_drawables ;
   ModelManager    m_model_manager ;

   // Various command line options specific to this program
   OModelParam<std::string> m_conf_option ; // --config-file
} ;

} // end of namespace encapsulating above class definition

//------------------------------- MAIN ----------------------------------

int main(int argc, const char* argv[])
{
   MYLOGVERB = LOG_ERR ; // minimize INVT's incessant chatter
   try
   {
      lobot::LoApp app ;
      app.parse_command_line(argc, argv) ;
      app.run() ;
   }
   catch (lobot::uhoh& e)
   {
      LERROR("%s", e.what()) ;
      return e.code() ;
   }
   catch (std::exception& e)
   {
      LERROR("%s", e.what()) ;
      return 255 ;
   }
   return 0 ;
}

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------- FORWARD DECLARATIONS --------------------------

Dims  grab_size() ;
float grab_rate() ;
bool  recording_enabled() ;
std::string recording_stem() ;
bool show_ui() ;

//----------------------------- APP INIT --------------------------------

// Setup INVT model manager and command line options
LoApp::LoApp()
   : m_model_manager("lograb"),
     m_conf_option(& OPT_ConfigFile, & m_model_manager)
{
   Shutdown::start_listening() ;
}

// Create video streams to grab from each camera connected to FireWire
// bus.
static void create_video_streams(const Dims& resolution, float frame_rate,
                                 LoApp::VideoStreams* V)
{
   const int n = FireWireBus::instance().num_cameras() ;
   V->reserve(n) ;
   for (int i = 0; i < n; ++i)
      V->push_back(new VideoStream(i, resolution, frame_rate)) ;

   // Okay to release the FireWire camera nodes now. We won't be needing
   // them any further in this program.
   FireWireBus::instance().release_camera_nodes() ;
}

// Create video recorders attached to each of the video streams to store
// the input frames to corresponding MPEG movies.
static void create_video_recorders(const LoApp::VideoStreams& V,
                                   const std::string& mpeg_name_root,
                                   LoApp::VideoRecorders* R)
{
   const int n = V.size() ;
   R->reserve(n) ;
   for (int i = 0; i < n; ++i)
      R->push_back(new VideoRecorder(mpeg_name_root + to_string(i), V[i])) ;
}

// DEVNOTE: THIS IS HORRIBLY BROKEN! NEEDS TO BE UPDATED TO USE OpenGL
// FOR RENDERING THE GRABBED FRAMES AS THE OLD DRAWABLE STUFF IS NO
// LONGER AVAILABLE (circa mid-Feb 2010) AS THE MAIN WINDOW WAS CONVERTED
// TO A GLUT WINDOW AFTER THE ROBOLOCUST DEVELOPMENT FOCUS SHIFTED TO
// USING A LASER RANGE FINDER AS THE PRIMARY SENSOR RATHER THAN CAMERAS.
/*
// Create drawables required to show the images being grabbed from the
// cameras on the application's main window.
static void create_drawables(const LoApp::ImageCompositor* C,
                             LoApp::Drawables* D)
{
   typedef ImageDrawable<PixelType> Img ;
   D->push_back(new Img(C, Rectangle(Point(0,0), C->getImageSize()))) ;
}
*/

//----------------------- COMMAND LINE PARSING --------------------------

void LoApp::parse_command_line(int argc, const char* argv[])
{
   std::string default_config_file = getenv("HOME") ;
   default_config_file += "/" ;
   default_config_file += LOBOT_DEFAULT_CONFIG_FILE_NAME ;
   m_model_manager.setOptionValString(& OPT_ConfigFile,
                                      default_config_file.c_str()) ;

   if (! m_model_manager.parseCommandLine(argc, argv, "", 0, 0))
      throw customization_error(BAD_OPTION) ;
}

//----------------------------- MAIN LOOP -------------------------------

// Quick helper class to start and stop model manager (useful when
// exceptions are thrown because destructor automatically stops the model
// manager without requiring an explicit call to the stop method prior to
// throwing the exception).
class ModelManagerStarter {
   ModelManager& mgr ;
public :
   ModelManagerStarter(ModelManager& m) : mgr(m) {mgr.start() ;}
   ~ModelManagerStarter() {mgr.stop() ;}
} ;

// Application object's run method (aka main loop)
void LoApp::run()
{
   ModelManagerStarter M(m_model_manager) ;

   // Load the config file (if any)
   try
   {
      Configuration::load(config_file()) ;
      //Configuration::dump() ;
   }
   catch (customization_error& e) // this is not fatal
   {
      LERROR("%s", e.what()) ; // simply report error and move on
   }

   // Setup input video streams and output recording streams
   create_video_streams(grab_size(), grab_rate(), & m_video_streams) ;
   connect(m_video_streams, m_compositor) ;
   if (recording_enabled())
      create_video_recorders(m_video_streams, recording_stem() + "-",
                             & m_video_recorders) ;

   // DEVNOTE: MAIN WINDOW CODE COMPLETELY BROKEN AND NEEDS TO BE FIXED
   // TO USE OpenGL/GLUT. ROBOLOCUST IS NOW (circa mid-Feb 2010) USING A
   // LASER RANGE FINDER RATHER THAN CAMERAS AS THE PRIMARY SENSING
   // MODALITY.
   /*
   // Create and configure main window
   MainWindow& W = MainWindow::instance() ;
   if (show_ui()) {
      create_drawables(& m_compositor, & m_drawables) ;
      connect(m_drawables, W) ;
      W.resize(m_compositor.getImageSize()) ;
      W.show("Lobot Multi-grab Tester") ;
   }
   */

   // Grab frames and display them
   while (! Shutdown::signaled())
   {
      std::for_each(m_video_streams.begin(), m_video_streams.end(),
                    std::mem_fun(& VideoStream::update)) ;
      std::for_each(m_video_recorders.begin(), m_video_recorders.end(),
                    std::mem_fun(& VideoRecorder::update)) ;
      m_compositor.update() ;

      /*
      W.render() ;
      if (W.dismissed()) {
         Shutdown::signal() ;
         Thread::wait_all() ;
      }
      */
      usleep(15000) ;
   }
}

//--------------------------- APP CLEAN-UP ------------------------------

LoApp::~LoApp()
{
   purge_container(m_drawables) ;
   purge_container(m_video_recorders) ;
   purge_container(m_video_streams) ;
}

//------------------------------ HELPERS --------------------------------

Dims grab_size()
{
   try
   {
      Dims d ;
      convertFromString(video_conf<std::string>("grab_size"), d) ;
      return d ;
   }
   catch (std::exception&) // badly formatted dims string in config file
   {
      return LOBOT_DEFAULT_GRAB_SIZE ; // don't crash!
   }
}

float grab_rate()
{
   return video_conf("grab_rate", LOBOT_DEFAULT_GRAB_RATE) ;
}

bool recording_enabled()
{
   return recording_stem() != "" ;
}

std::string recording_stem()
{
   return video_conf<std::string>("record") ;
}

bool show_ui()
{
   return ui_conf("show_ui", true) ;
}

//-----------------------------------------------------------------------

} // end of namespace lobot encapsulating the LoApp object

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
