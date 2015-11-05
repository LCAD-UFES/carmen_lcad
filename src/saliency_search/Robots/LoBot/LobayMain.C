/**
   \file  Robots/LoBot/LobayMain.C
   \brief Robolocust metrics log analyzer for Bayesian TTI predictions.

   This file defines the main function for a multithreaded analysis
   program that loads all the Robolocust metrics logs associated with the
   Bayesian time-to-impact prediction experiments and processes these
   logs to produce the desired results files for each set of related
   experiments.

   Here's the background for this program: the Robolocust project aims to
   use locusts for robot navigation. Specifically, the locust sports a
   visual interneuron known as the Lobula Giant Movement Detector (LGMD)
   that spikes preferentially in response to objects moving toward the
   animal on collisional trajectories. Robolocust's goal is to use an
   array of locusts, each looking in a different direction. As the robot
   moves, we expect to receive greater spiking activity from the locusts
   looking in the direction in which obstacles are approaching (or being
   approached) and use this information to veer the robot away.

   Before we mount actual locusts on a robot, we would like to first
   simulate this LGMD-based navigation. Toward that end, we use a laser
   range finder (LRF) mounted on an iRobot Create driven by a quad-core
   mini-ITX computer. A computational model of the LGMD developed by
   Gabbiani, et al. takes the LRF distance readings and the Create's
   odometry as input and produces artificial LGMD spikes based on the
   time-to-impact of approaching objects. We simulate multiple virtual
   locusts by using different angular portions of the LRF's field of
   view. To simulate reality a little better, we inject Gaussian noise
   into the artificial spikes.

   We have devised three different LGMD-based obstacle avoidance
   algorithms:

      1. EMD: pairs of adjacent LGMD's are fed into Reichardt motion
              detectors to determine the dominant direction of spiking
              activity and steer the robot away from that direction;

      2. VFF: a spike rate threshold is used to "convert" each virtual
              locust's spike into an attractive or repulsive virtual
              force; all the force vectors are combined to produce the
              final steering vector;

      3. TTI: each locust's spike rate is fed into a Bayesian state
              estimator that computes the time-to-impact given a spike
              rate; these TTI estimates are then used to determine
              distances to approaching objects, thereby effecting the
              LGMD array's use as a kind of range sensor; the distances
              are compared against a threshold to produce attractive and
              repulsive forces, with the sum of the force field vectors
              determining the final steering direction.

   As mentioned above, the TTI algorithm uses a Bayesian state estimator
   that predicts the time-to-impact given an LGMD spike rate. The
   Bayesian time-to-impact prediction experiments are designed to
   evaluate this TTI prediction model. Here's how these experiments were
   conducted:

   The robot was driven straight ahead towards a wall starting at a point
   2.5 meters away. A single virtual locust looking straight ahead was
   used to generate LGMD spikes. The robot was configured to stop just
   short of hitting the wall.

   As it moved toward the wall, the robot's controller would record the
   current speed, LGMD spike rate, actual time-to-impact, predicted TTI,
   actual distance to wall, predicted distance and the prediction's
   confidence level to a log file.

   We varied the robot's speed, the amount of noise in the artificially
   generated LGMD spikes and the delta value for the spike generation
   model. The delta value is a parameter that controls when the peak
   spike rate is achieved w.r.t. the point at which a collision takes
   place, i.e., when time-to-impact is zero. To illustrate, let us say we
   use a delta of 1.0 seconds; this means that the spike generation model
   will produce a peak when the time-to-impact is 1.0 seconds, i.e., when
   the approaching wall is 1 second away from the robot. Similarly, when
   delta is 0.5, the LGMD peak will be achieved when the robot is half a
   second away from colliding with the wall; at delta = 2, the peak will
   be at 2 seconds from collision; so on and so forth.

   The TTI prediction experiments were run with the following parameters:

      - noise: 0Hz, 25Hz, 50Hz, 100Hz
      - speed: 0.1m/s, 0.2m/s, 0.3m/s, 0.4m/s
      - delta: 0.25s, 0.50s, 0.75s, 1.00s, 1.25s, 1.50s, 1.75s, 2.00s

   For each noise level, robot speed and delta value, we ran the robot 10
   times. Thus, if we consider each set of 10 such individual runs to be
   one dataset, we have a total of 4 noise levels times 4 speeds times 8
   delta values = 128 datasets.

   If "bay" is the root data directory for the Bayesian TTI experiments,
   then each dataset's log files are stored in a subdirectory of "bay" in
   a hierarchy as depicted below:

                          bay
                           |
                           +-- 000
                           |    |
                           |    +-- 0.1
                           |    |    |
                           |    |    +-- 0.25
                           |    |    +-- 0.50
                           |    |    +-- 0.75
                           |    |    +-- 1.00
                           |    |    +-- 1.25
                           |    |    +-- 1.50
                           |    |    +-- 1.75
                           |    |    +-- 2.00
                           |    |
                           |    +-- 0.2
                           |    |    :
                           |    |    :
                           |    |
                           |    +-- 0.3
                           |    |    :
                           |    |
                           |    +-- 0.4
                           |         :
                           |         :
                           |
                           +-- 025
                           |    :
                           |    :
                           |
                           +-- 050
                           |    :
                           |    :
                           |
                           +-- 100
                                :
                                :

   NOTE: The above directory hierarchy is not fixed by the lobot
   controller. It is simply how we configured the controller to work
   while running these experiments and collecting the data.

   The objective of this program is to load an entire dataset from each
   of the above directories and then write out a results file whose
   format is shown below:

        TTI   LGMD Spike Rate   Predicted TTI   Confidence Level
        ---   ---------------   -------------   ----------------
              mean   stdev      mean   stdev    mean   stdev

   Since each dataset's results can be computed independently of every
   other dataset, we use multiple threads to process several datasets in
   parallel. Given the Bayesian TTI prediction experiments' data root
   directory, this program finds all the subdirectories containing log
   files and then launches as many threads as CPU's available to walk
   through this directory list and perform the necessary log file
   parsing.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LobayMain.C $
// $Id: LobayMain.C 14083 2010-09-30 13:59:37Z mviswana $
//

//--------------------------- LIBRARY CHECKS ----------------------------

#if !defined(INVT_HAVE_BOOST_PROGRAM_OPTIONS)

#include <iostream>

int main()
{
   std::cerr << "Sorry, this program requires the following libraries:\n"
             << "\tlibboost_program_options\n\n" ;
   std::cerr << "Please ensure development packages for above libraries "
             << "are installed\n"
             << "and then rebuild this program to get it to work.\n" ;
   return 255 ;
}

#else // various required libraries available

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/baylog/LoBaylogAnalyzer.H"
#include "Robots/LoBot/baylog/LoDirList.H"

#include "Robots/LoBot/thread/LoThread.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/util/LoFile.H"
#include "Robots/LoBot/util/LoSTL.H"
#include "Robots/LoBot/util/LoSysConf.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"

// Boost headers
#include <boost/program_options.hpp>

// Standard C++ headers
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <stdexcept>

// Standard C headers
#include <stdlib.h>

// Standard Unix headers
#include <unistd.h>

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from global section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return lobot::global_conf<T>(key, default_value) ;
}

/// This inner class encapsulates various parameters that can be used to
/// tweak different aspects of the Bayesian TTI prediction analysis.
class MainParams : public lobot::singleton<MainParams> {
   /// The log files for the Bayesian time-to-impact prediction
   /// experiments are stored under a directory hierarchy that is
   /// arranged to match the different LGMD spike rate noise levels,
   /// robot speeds and delta values used to gather the data.
   ///
   /// This setting specifies the root directory under which the
   /// above-mentioned hierarchy of subdirectories appears.
   std::string m_root ;

   /// The lobay program processes multiple datasets in parallel by first
   /// reading in the entire list of directories containing log files and
   /// then launching threads to process each dataset. This setting
   /// specifies a regular expression that will be used to match dataset
   /// directory names.
   std::string m_dataset ;

   /// Private constructor because this is a singleton.
   MainParams() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class lobot::singleton<MainParams> ;

public:
   /// Accessing the various parameters.
   //@{
   static const std::string& root()    {return instance().m_root    ;}
   static const std::string& dataset() {return instance().m_dataset ;}
   //@}
} ;

// Parameters initialization
MainParams::MainParams()
   : m_root(conf<std::string>("root_dir", "/tmp/lobay")),
     m_dataset(conf<std::string>("dataset_dir_name",
                                 "[01][025][05]/0\\.[1-4]/[0-2]\\.[0257][05]"))
{}

// Shortcut
typedef MainParams Params ;

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- PROGRAM OPTIONS ----------------------------

namespace {

// If the user does not supply the -c option on the command line, we will
// fall back to the default config file name returned by this function.
std::string default_config_file()
{
   return std::string(getenv("HOME")) + "/.lobayrc" ;
}

// Helper function to take care of the annoying details of using
// Boost.program_options to get at the command line arguments.
//
// DEVNOTE: This code is lifted almost verbatim from the
// Boost.program_options tutorial and examples that comes with the Boost
// documentation. There may be better (i.e., neater, more effective,
// clearer, more efficient, whatever) ways to use the library.
std::string parse(int argc, char* argv[])
{
   std::string config_file_name ;

   // Specify the command line options
   namespace po = boost::program_options ;
   po::options_description options("Command line options") ;
   options.add_options()
      // the -c option for specifying the config file
      ("config-file,c",
       po::value<std::string>(&config_file_name)->
          default_value(default_config_file()),
       "specify configuration settings file") ;

   // Setup done: now parse argc and argv...
   po::variables_map varmap ;
   po::store(po::parse_command_line(argc, argv, options), varmap) ;
   po::notify(varmap) ;

   return config_file_name ;
}

// Helper function to read the lobay program's config file. If the
// specified file doesn't exist, the program will rely on default
// settings. If the config file contains problematic constructs, an error
// will be reported but the program will continue on, simply ignoring the
// bad settings.
void load_config_file(const std::string& file_name)
{
   using namespace lobot ;
   try
   {
      Configuration::load(file_name) ;
      //Configuration::dump() ;
   }
   catch (customization_error& e)
   {
      if (e.code() != NO_SUCH_CONFIG_FILE)
         std::cerr << e.what() << '\n' ;
   }
}

} // end of local anonymous namespace encapsulating above helpers

//------------------------- METRICS ANALYSIS ----------------------------

namespace {

// This function reads all the Robolocust metrics logs in the specified
// directory and combines them to produce the desired average trajectory
// and other pertinent info.
void process_datasets(const std::vector<std::string>& dirs)
{
   using namespace lobot ;

   // Analyze datasets in parallel
   DirList dir_list(dirs) ;
   const int T = std::min(dir_list.size(), num_cpu()) ;
   std::vector<BaylogAnalyzer*> analyzer_threads ;
   analyzer_threads.reserve(T) ;
   for (int i = 0; i < T; ++i)
      analyzer_threads.push_back(BaylogAnalyzer::create(dir_list)) ;

   // Now we wait for the analyzer threads to do their thing...
   //
   // DEVNOTE: If we don't pause this main thread briefly before invoking
   // the wait API, we could be in big trouble because the scheduler
   // might decide to go right on executing this thread before it begins
   // any of the analyzer threads, in which case the wait will fail as
   // none of the other threads would have started up just yet, i.e., the
   // thread count would be zero, causing this thread to mistakenly
   // conclude that all the analyzers are done and that it is safe to
   // delete them. When those threads then start executing, they will try
   // to reference their associated analyzer objects, which, because they
   // no longer exist, will end up taking us on a scenic, albeit short
   // and tragic, bus ride to Segfault City.
   //
   // DEVNOTE 2: A better way to do this is to use another condition
   // variable rather than the one used implicitly by Thread::wait_all(),
   // over which we have no control. For example, we could wait on a
   // condition variable that tests a counter going all the way up to T,
   // the number of analyzer threads created. When each analyzer is done,
   // it will increment the counter. Since we would have explicit control
   // over this variable over here, we can be assured that this thread
   // won't mistakenly assume all the analyzers are done.
   //
   // Of course, the Right Thing is good and all. But why bother? A short
   // sleeps works just as well for most practical purposes...
   sleep(1) ; // HACK! to prevent segfault; see comment above
   Thread::wait_all() ;
   purge_container(analyzer_threads) ;
}

} // end of local anonymous namespace encapsulating above helpers

//------------------------------- MAIN ----------------------------------

int main(int argc, char* argv[])
{
   int ret = 0 ;
   try
   {
      load_config_file(parse(argc, argv)) ;
      process_datasets(lobot::find_dir(Params::root(), Params::dataset())) ;
   }
   catch (lobot::uhoh& e)
   {
      std::cerr << e.what() << '\n' ;
      ret = e.code() ;
   }
   catch (std::exception& e)
   {
      std::cerr << e.what() << '\n' ;
      ret = 127 ;
   }
   catch(...)
   {
      std::cerr << "unknown exception\n" ;
      ret = 255 ;
   }
   return ret ;
}

//-----------------------------------------------------------------------

#endif // library checks

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
