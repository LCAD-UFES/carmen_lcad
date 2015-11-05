/**
   \file  Robots/LoBot/LoApp.C
   \brief This file defines the non-inline member functions of the
   lobot::App class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LoApp.C $
// $Id: LoApp.C 13811 2010-08-21 02:00:08Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/LoApp.H"

#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoBehavior.H"

#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/lgmd/LocustModel.H"

#include "Robots/LoBot/ui/LoMainWindow.H"
#include "Robots/LoBot/ui/LoLocustViz.H"
#include "Robots/LoBot/ui/LoLaserVizFlat.H"
#include "Robots/LoBot/ui/LoLaserViz.H"
#include "Robots/LoBot/ui/LoDrawable.H"

#include "Robots/LoBot/io/LoRobot.H"
#include "Robots/LoBot/io/LoLaserRangeFinder.H"
#include "Robots/LoBot/io/LoDangerZone.H"

#include "Robots/LoBot/io/LoInputSource.H"
#include "Robots/LoBot/io/LoVideoStream.H"
#include "Robots/LoBot/io/LoVideoRecorder.H"
#include "Robots/LoBot/io/LoFireWireBus.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/config/LoCommonOpts.H"
#include "Robots/LoBot/config/LoDefaults.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"
#include "Robots/LoBot/thread/LoPause.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/factory.hh"

#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoDebug.H"
#include "Robots/LoBot/util/range.hh"

// INVT image support
#include "Image/Dims.H"

// INVT utilities
#include "Util/log.H"

// Unix headers
#include <glob.h>
#include <unistd.h>

// Standard C++ headers
#include <algorithm>
#include <functional>
#include <iterator>
#include <utility>

#include <cmath>
#include <cstdlib>
#include <ctime>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------- FORWARD DECLARATIONS --------------------------

bool video_enabled() ;

Dims  grab_size() ;
float grab_rate() ;

std::string locust_directions() ;
int locust_fov() ;

bool playback_enabled() ;
bool recording_enabled() ;
std::string playback_stem() ;
std::string recording_stem() ;

bool laser_enabled() ;
std::string laser_device() ;
int laser_baud_rate() ;

bool robot_enabled() ;
std::string robot_platform() ;

bool show_ui() ;
static bool mapping_enabled() ;
static bool viz_on(const Behavior*) ;

//-------------------------- INITIALIZATION -----------------------------

App& App::create(int argc, const char* argv[])
{
   App& app = instance() ;
   app.m_argc = argc ;
   app.m_argv = argv ;
   return app ;
}

// Setup INVT model manager and command line options; start the signal
// handling thread that responds to application shutdown signals.
App::App()
   : m_compositor(0),
     m_lrf(0),
     m_input_source(0),
     m_robot(0),
     m_map(0),
     m_laser_viz(0),
     m_laser_viz_flat(0),
     m_locust_viz(0),
     m_model_manager("lobot"),
     m_cf_option(& OPT_ConfigFile, & m_model_manager),
     m_initialized(false)
{
   Shutdown::start_listening() ;
}

// Create video streams to read images from MPEG files
static void create_mpeg_video_streams(std::string mpeg_file_names_stem,
                                      App::VideoStreams* V)
{
   mpeg_file_names_stem += "*" ;
   glob_t buf ;
   if (glob(mpeg_file_names_stem.c_str(), 0, 0, & buf) != 0)
      throw vstream_error(MPEG_FILES_NOT_FOUND) ;

   const int n = buf.gl_pathc ;
   V->reserve(n) ;
   for (int i = 0; i < n; ++i)
      V->push_back(new VideoStream(buf.gl_pathv[i])) ;

   globfree(& buf) ;
}

// Create video streams to grab from each camera connected to FireWire
// bus.
static void create_camera_video_streams(const Dims& resolution,
                                        float frame_rate,
                                        App::VideoStreams* V)
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
static void create_video_recorders(const App::VideoStreams& V,
                                   const std::string& mpeg_name_root,
                                   App::VideoRecorders* R)
{
   const int n = V.size() ;
   R->reserve(n) ;
   for (int i = 0; i < n; ++i)
      R->push_back(new VideoRecorder(mpeg_name_root + to_string(i), V[i])) ;
}

// Create an instance of the named locust model and set it up to use the
// specified portion of the laser range finder's FOV as its source.
static LocustModel*
create_locust_model(const std::string& name, const LocustModel::InitParams& p)
{
   typedef factory<LocustModel, LocustModel::InitParams> lgmd_factory ;
   try
   {
      return lgmd_factory::create(name, p) ;
   }
   catch (lgmd_factory::unknown_type&)
   {
      throw unknown_model(UNKNOWN_LOCUST_MODEL) ;
   }
}

// Create the desired number of instances of the user-specified locust
// model. These objects will generate the LGMD spike trains from the
// specified input source.
//
// We setup the virtual locusts starting from the left edge of the
// composited input image and move rightwards till we hit the right edge.
// For each virtual locust, we take into account the configured FOV and
// overlap settings.
static void
create_locust_models(const InputSource* S, App::LocustModels* models)
{
   if (S->using_video())
   {
      // FIXME: Since the switch to a laser range finder as the primary
      // sensing modality and changes thereafter to the way locust models
      // are created, rendered, etc., this block of code has become
      // hopelessly incorrect and simply won't work! It needs careful
      // thought and some major surgery...
      /*
      const Dims size = S->get_image_size() ;
      const std::string model_name = locust_model() ;
      const int fov = clamp(locust_fov(), LOBOT_MIN_LOCUST_FOV, size.w()) ;
      const int overlap = clamp(fov_overlap(), 0, fov/2) ;

      const int N = static_cast<int>(
         std::floor(static_cast<double>(size.w())/(fov - overlap))) ;
      models->reserve(N) ;

      const int W = size.w() - 1 ;
      const int B = size.h() - 1 ;
      const int T = 0 ;
      int L = 0 ; // start at left edge of composited input image
      for(;;)
      {
         int R = std::min(L + fov - 1, W) ;
         models->push_back(create_locust_model(model_name, S, L, R, B, T)) ;
         if (R >= W) // hit the right edge of composited input image
            break ;
         L = R + 1 - overlap ;
      }
      */
   }
   else if (S->using_laser())
   {
      std::vector<int> D = string_to_vector<int>(locust_directions());
      const int N = D.size() ;
      if (N <= 0)
         return ;
      std::sort(D.begin(), D.end(), std::greater<int>()) ; // left to right

      const std::string model_name = locust_model() ;
      const range<int> lrf_range   = S->lrf_angular_range() ;
      const int fov = clamp(locust_fov(), lrf_range) ;

      Drawable::Geometry g =
         get_conf<std::string>(model_name, "geometry", "0 75 975 75") ;
      g.width /= N ;

      LocustModel::InitParams p ;
      p.spike_range =
         get_conf<float>(model_name, "spike_range", make_range(0.0f, 300.0f)) ;
      p.source = S ;

      models->reserve(N) ;
      for(int i = 0; i < N; ++i, g.x += g.width)
      {
         const int L = std::min(D[i] + fov/2, lrf_range.max()) ;
         const int R = std::max(D[i] - fov/2, lrf_range.min()) ;

         p.direction = D[i] ;
         p.lrf_range.reset(L, R) ;
         p.name = std::string("lgmd[") + to_string(p.direction) + "]" ;
         p.geometry = g ;

         models->push_back(create_locust_model(model_name, p)) ;
      }
   }
}

// Start the different behaviours as specified in the config file.
static void create_behaviours(App::Behaviours* B)
{
   std::vector<std::string> behaviours = string_to_vector<std::string>(
      global_conf<std::string>("behaviors")) ;
   //dump(behaviours, "create_behaviours", "behaviours") ;

   const int N = behaviours.size() ;
   B->reserve(N) ;
   for (int i = 0; i < N; ++i)
      try
      {
         Behavior* b = factory<Behavior>::create(behaviours[i]) ;
         b->name = behaviours[i] ;
         B->push_back(b) ;
      }
      catch (factory<Behavior>::unknown_type&)
      {
         throw unknown_model(UNKNOWN_BEHAVIOR) ;
      }
}

// Create the motor subsystem's interface object, taking care of
// ModelManager niceties.
static Robot* create_robot(const std::string& robot_platfom, ModelManager& M)
{
   typedef factory<Robot, ModelManager> robot_factory ;
   try
   {
      return robot_factory::create(robot_platfom, M) ;
   }
   catch (robot_factory::unknown_type&)
   {
      throw unknown_model(UNKNOWN_ROBOT_PLATFORM) ;
   }
   return 0 ; // keep compiler happy
}

// Let other threads know that the application object is fully loaded
void App::signal_init()
{
   m_initialized_cond.broadcast(signal_pred()) ;
}

bool App::signal_pred::operator()()
{
   App& app = App::instance() ;
   app.m_initialized = true ;
   return true ;
}

// API to let other threads wait until application object is fully loaded
void App::wait_init()
{
   m_initialized_cond.wait(wait_pred()) ;
}

bool App::wait_pred::operator()()
{
   return App::instance().m_initialized ;
}

//----------------------- COMMAND LINE PARSING --------------------------

void App::parse_command_line()
{
   std::string default_config_file = getenv("HOME") ;
   default_config_file += "/" ;
   default_config_file += LOBOT_DEFAULT_CONFIG_FILE_NAME ;
   m_model_manager.setOptionValString(& OPT_ConfigFile,
                                      default_config_file.c_str()) ;

   if (! m_model_manager.parseCommandLine(m_argc, m_argv, "", 0, 0))
      throw customization_error(BAD_OPTION) ;
}

//----------------------------- MAIN LOOP -------------------------------

// Application object's run method (aka main loop)
void App::run()
{
   m_model_manager.start() ;

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

   // Create the video I/O objects
   if (video_enabled()) {
      if (playback_enabled())
         create_mpeg_video_streams(playback_stem(), & m_video_streams) ;
      else
         create_camera_video_streams(grab_size(), grab_rate(),
                                     & m_video_streams) ;
      m_compositor = new ImageCompositor() ;
      connect(m_video_streams, *m_compositor) ;
      if (recording_enabled())
         create_video_recorders(m_video_streams, recording_stem() + "-",
                                & m_video_recorders) ;
      Configuration::set_internal("frame_rate",
                                  to_string(m_video_streams[0]->frameRate())) ;
   }

   // Create the laser range finder I/O object
   if (laser_enabled()) {
      m_lrf = new LaserRangeFinder(laser_device(), laser_baud_rate()) ;
      DangerZone::use(m_lrf) ;
   }

   // Create the robot interface object
   if (robot_enabled())
      m_robot = create_robot(robot_platform(), m_model_manager) ;

   // Create the map object if mapping is enabled
   if (mapping_enabled())
      m_map = new Map() ;

   // Setup the locust LGMD models
   if (video_enabled() && video_input())
      m_input_source = new InputSource(m_compositor) ;
   else if (laser_enabled() && laser_input())
      m_input_source = new InputSource(m_lrf) ;
   if (m_input_source)
      create_locust_models(m_input_source, & m_locusts) ;

   // Start the different behaviours
   create_behaviours(& m_behaviours) ;

   // Create and configure main window
   if (show_ui()) {
      MainWindow& W = MainWindow::instance() ;
      if (video_enabled() && video_input()) {
      }
      else if (laser_enabled() && laser_input()) {
         if (visualize("laser_viz"))
            W.push_back(m_laser_viz = new LaserViz(m_lrf)) ;
         if (visualize("laser_viz_flat"))
            W.push_back(m_laser_viz_flat = new LaserVizFlat(m_lrf)) ;
      }

      if (mapping_enabled() && visualize("map"))
         W.push_back(m_map) ;

      if (visualize(locust_model()))
         connect(m_locusts, W) ;
      if (visualize("locust_viz"))
         W.push_back(m_locust_viz = new LocustViz(m_locusts)) ;

      if (visualize("turn_arbiter"))
         W.push_back(& TurnArbiter::instance()) ;
      connect_if(m_behaviours, W, viz_on) ;
   }

   // Check whether the application should be started off in a paused
   // state or not.
   bool start_paused = global_conf("start_paused", false) ;
   if (start_paused && show_ui())
      Pause::set() ;
   else
      Pause::clear() ;

   // The application object is, at this point, fully loaded
   srand(time(0)) ;
   signal_init() ;

   // Grab data and do the locust jig
   int update_delay = clamp(global_conf("update_delay", 100), 1, 1000) * 1000;
   while (! Shutdown::signaled())
   {
      if (Pause::is_clear()) {
         UpdateLock::begin_write() ;
            std::for_each(m_video_streams.begin(), m_video_streams.end(),
                          std::mem_fun(& VideoStream::update)) ;
            std::for_each(m_video_recorders.begin(), m_video_recorders.end(),
                          std::mem_fun(& VideoRecorder::update)) ;
            if (m_compositor)
               m_compositor->update() ;
            if (m_lrf) {
               m_lrf->update() ;
               DangerZone::update() ;
            }
            if (m_robot)
               m_robot->update() ;
            std::for_each(m_locusts.begin(), m_locusts.end(),
                          std::mem_fun(& LocustModel::update)) ;
         UpdateLock::end_write() ;
      }
      usleep(update_delay) ;
   }

   // Kill the robot before quitting the application
   if (m_robot)
      m_robot->off() ;
}

//--------------------------- APP CLEAN-UP ------------------------------

App::~App()
{
   LERROR("cleaning up...") ;

   purge_container(m_behaviours) ;

   delete m_locust_viz ;
   delete m_laser_viz_flat ;
   delete m_laser_viz ;
   delete m_map ;
   delete m_robot ;

   purge_container(m_locusts) ;
   delete m_input_source ;
   delete m_lrf ;
   delete m_compositor ;

   purge_container(m_video_recorders) ;
   purge_container(m_video_streams) ;

   if (m_model_manager.started())
      m_model_manager.stop() ;
}

//------------------------------ HELPERS --------------------------------

bool video_enabled()
{
   return video_conf("use_video", true) ;
}

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

bool playback_enabled()
{
   return playback_stem() != "" ;
}

std::string playback_stem()
{
   return video_conf<std::string>("play") ;
}

bool laser_enabled()
{
   return laser_conf("use_laser", true) ;
}

std::string laser_device()
{
   return laser_conf<std::string>("serial_port", "/dev/ttyACM0") ;
}

int laser_baud_rate()
{
   return laser_conf("baud_rate", 115200) ;
}

std::string locust_directions()
{
   return get_conf<std::string>(locust_model(), "locust_directions", "") ;
}

int locust_fov()
{
   return get_conf(locust_model(), "locust_fov", LOBOT_DEFAULT_LOCUST_FOV) ;
}

bool show_ui()
{
   return ui_conf("show_ui", true) ;
}

static bool mapping_enabled()
{
   return get_conf("map", "enable", false) ;
}

static bool viz_on(const Behavior* B)
{
   return visualize(B->name) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
