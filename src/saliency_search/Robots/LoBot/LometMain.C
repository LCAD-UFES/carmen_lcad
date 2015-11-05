/**
   \file  Robots/LoBot/LometMain.C
   \brief Robolocust metrics log analyzer.

   This file defines the main function for a multithreaded analysis
   program that loads all the Robolocust metrics logs associated with an
   experiment and then combines the information contained in these logs
   to produce average trajectory information, exploiting parallelism
   wherever possible.

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

   We also implemented a very simple algorithm that just steers the robot
   towards the direction of least spiking activity. However, although it
   functioned reasonably well as an obstacle avoidance technique, it was
   found to be quite unsuitable for navigation tasks. Therefore, we did
   not pursue formal tests for this algorithm, focusing instead on the
   three algorithms mentioned above.

   To evaluate the relative merits of the above algorithms, we designed a
   slalom course in an approximately 12'x6' enclosure. One end of this
   obstacle course was designated the start and the other end the goal.
   The robot's task was to drive autonomously from start to goal,
   keeping track of itself using Monte Carlo Localization. As it drove,
   it would collect trajectory and other pertinent information in a
   metrics log.

   For each algorithm, we used four noise profiles: no noise, 25Hz
   Gaussian noise in the LGMD spikes, 50Hz, and 100Hz. For each noise
   profile, we conducted 25 individual runs. We refer to an individual
   run from start to goal as an "experiment" and a set of 25 experiments
   as a "dataset."

   The objective of this program is to load an entire dataset and then
   perform the necessary computations to produce an "average" trajectory
   from start to goal. Other useful metrics are also computed such as
   average forward driving speed, total number of collisions across all
   experiments, etc.

   Since the above computations can take a while, this analysis program
   uses multiple threads to speed up various subtasks. Therefore, it
   would be best to run it on a multiprocessor machine (as there are 25
   experiments per dataset, something with 24 to 32 CPU's would provide
   maximal benefit).
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LometMain.C $
// $Id: LometMain.C 14294 2010-12-02 06:28:57Z mviswana $
//

//--------------------------- LIBRARY CHECKS ----------------------------

#if !defined(INVT_HAVE_BOOST_PROGRAM_OPTIONS) || \
    !defined(INVT_HAVE_BOOST_FILESYSTEM)

#include <iostream>

int main()
{
   std::cerr << "Sorry, this program requires the following Boost libraries:\n"
             << "\tprogram_options filesystem\n\n" ;
   std::cerr << "Please ensure development packages for above libraries "
             << "are installed\n"
             << "and then rebuild this program to get it to work.\n" ;
   return 255 ;
}

#else // various required libraries available

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoCorrFinder.H"
#include "Robots/LoBot/metlog/LoPointMatrix.H"
#include "Robots/LoBot/metlog/LoMetlogLoader.H"
#include "Robots/LoBot/metlog/LoDataset.H"
#include "Robots/LoBot/metlog/LoExperiment.H"
#include "Robots/LoBot/metlog/LoMetlogList.H"
#include "Robots/LoBot/metlog/LoPointTypes.H"

#include "Robots/LoBot/thread/LoThread.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/util/LoFile.H"
#include "Robots/LoBot/util/LoStats.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoSTL.H"
#include "Robots/LoBot/util/LoSysConf.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"

// Boost headers
#include <boost/program_options.hpp>
#include <boost/lambda/lambda.hpp>

// Standard C++ headers
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <utility>

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

/// This inner class encapsulates various parameters that can be used
/// to tweak different aspects of the trajectory metrics analysis.
class LometParams : public lobot::singleton<LometParams> {
   /// The lomet program expects to be passed a list of directories on
   /// the command line. Each directory is assumed to contain a dataset
   /// consisting of 25 (or more) metrics log files collected from
   /// experiments conducted to gauge the performance of an LGMD-based
   /// obstacle avoidance algorithm in a local navigation task.
   ///
   /// The program reads the metlogs in these directories, parses them to
   /// extract the relevant info and performs the necessary analysis that
   /// highlights the algorithm's average-case behaviour. Since the
   /// directories may well contain files other than the metlogs, the
   /// analysis program needs some way to figure out which ones to load.
   ///
   /// This setting specifies a regular expression that matches the names
   /// of all the metrics logs.
   std::string m_log_name ;

   /// Once we have analyzed the log files stored in a directory, the
   /// results will be written to a file in that directory. This setting
   /// specifies the name of the results file.
   ///
   /// By default, the result file is named "result". Thus, if lomet is
   /// invoked with the command line argument "foo", the analysis will be
   /// written to the file "foo/result".
   std::string m_result ;

   /// To help with debugging, lomet can be configured to dump the
   /// datasets it loads once it's done parsing the metlog files making
   /// up the dataset. The dump will be written to the same directory
   /// as the one from which the logs were loaded and will be named
   /// "foo.dump", where "foo" is the original name of the metlog file.
   ///
   /// This setting turns dataset dumping on. By default, it is off.
   bool m_dump_dataset ;

   /// Private constructor because this is a singleton.
   LometParams() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class lobot::singleton<LometParams> ;

public:
   /// Accessing the various parameters.
   //@{
   static const std::string& log_name() {return instance().m_log_name     ;}
   static const std::string& result()   {return instance().m_result       ;}
   static bool  dump_dataset()          {return instance().m_dump_dataset ;}
   //@}
} ;

// Parameters initialization
LometParams::LometParams()
   : m_log_name(conf<std::string>("log_file_name",
                                  "/(metlog-[[:digit:]]{8}-[[:digit:]]{6})$")),
     m_result(conf<std::string>("result_file", "result")),
     m_dump_dataset(conf("dump_dataset", false))
{}

// Shortcut
typedef LometParams Params ;

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- PROGRAM OPTIONS ----------------------------

namespace {

// This program recognizes just one option, viz., -c or --config-file.
// All other command line arguments are interpreted as names of
// directories, each of which is assumed to contain the Robolocust
// metrics log files for one experiment, i.e., each directory specified
// on the command line should hold one dataset.
//
// This data type is used to store the list of directories specified on
// the command line.
typedef std::vector<std::string> DirList ;

// The following type stores the metlog list as well as the name of the
// config file, i.e., it encapsulates all of the interesting stuff from
// the command line in one neat little packet.
typedef std::pair<std::string, DirList> CmdLine ;

// If the user does not supply the -c option on the command line, we will
// fall back to the default config file name returned by this function.
std::string default_config_file()
{
   return std::string(getenv("HOME")) + "/.lometrc" ;
}

// Helper function to take care of the annoying details of using
// Boost.program_options to get at the command line arguments.
//
// DEVNOTE: This code is lifted almost verbatim from the
// Boost.program_options tutorial and examples that comes with the Boost
// documentation. There may be better (i.e., neater, more effective,
// clearer, more efficient, whatever) ways to use the library.
CmdLine parse(int argc, char* argv[])
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
       "specify configuration settings file")

      // the -d option for specifying metlog directories; this option
      // need not actually be supplied on the command line as all
      // non-option arguments will be "converted" to multiple -d options
      ("metlog-dir,d",
       po::value<DirList>(),
       "directory containing Robolocust metrics log files for an experiment") ;

   // Convert all non-option arguments to a list of -d specs
   po::positional_options_description p ;
   p.add("metlog-dir", -1) ;

   // Setup done: now parse argc and argv...
   po::variables_map varmap ;
   po::store(po::command_line_parser(argc, argv).
             options(options).positional(p).run(), varmap) ;
   po::notify(varmap) ;

   // Stuff the metlog directories specified on command line into a
   // vector of strings and return that along with the config file name
   // to the caller...
   if (varmap.count("metlog-dir"))
      return CmdLine(config_file_name, varmap["metlog-dir"].as<DirList>()) ;

   // Great, user did not supply any directory names to work on; s/he's
   // gonna get a smackin'...
   return CmdLine(config_file_name, DirList()) ;
}

// Helper function to read the lomet program's config file. If the
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

// Shortcuts
using lobot::Dataset ;
using lobot::PointList ;
using lobot::PointListName ;

// Forward declarations
PointList analyze(Dataset&, PointListName) ;
PointList analyze_bumps(Dataset&) ;
std::vector<float> analyze_speeds(Dataset&) ;
void analyze_events(Dataset&, lobot::Experiment* result) ;

// This function reads all the Robolocust metrics logs in the specified
// directory and combines them to produce the desired average trajectory
// and other pertinent info.
void process_experiment(const std::string& dir)
{
   using namespace lobot ;

   // Make sure we're dealing with a valid directory...
   if (! is_dir(dir)) {
      std::cerr << dir << ": no such directory\n" ;
      return ;
   }

   // Read names of all metrics logs available under directory of interest
   std::vector<std::string> log_list = find_file(dir, Params::log_name()) ;
   if (log_list.empty()) {
      std::cerr << dir << ": no Robolocust metrics logs found\n" ;
      return ;
   }
   MetlogList metlog_list(log_list) ;

   // Create the dataset object that will collect the individual parsed
   // metlogs and help with their analysis...
   Dataset dataset ;

   // Load and parse all the logs in parallel
   const int T = std::min(static_cast<int>(log_list.size()), num_cpu()) ;
   std::vector<MetlogLoader*> loader_threads ;
   loader_threads.reserve(T) ;
   for (int i = 0; i < T; ++i)
      loader_threads.push_back(MetlogLoader::create(metlog_list, &dataset)) ;

   // Now we wait for the loader threads to do their thing...
   //
   // DEVNOTE: If we don't pause this main thread briefly before invoking
   // the wait API, we could be in big trouble because the scheduler
   // might decide to go right on executing this thread before it begins
   // any of the loader threads, in which case the wait will fail as none
   // of the other threads would have started up just yet, i.e., the
   // thread count would be zero, causing this thread to mistakenly
   // conclude that all the loaders are done and that it is safe to
   // delete them. When those threads then start executing, they will try
   // to reference their associated loader objects, which, because they
   // no longer exist, will end up taking us on a scenic, albeit short
   // and tragic, bus ride to Segfault City.
   //
   // DEVNOTE 2: A better way to do this is to use another condition
   // variable rather than the one used implicitly by Thread::wait_all(),
   // over which we have no control. For example, we could wait on a
   // condition variable that tests a counter going all the way up to T,
   // the number of loader threads created. When each loader is done, it
   // will increment the counter. Since we would have explicit control
   // over this variable over here, we can be assured that this thread
   // won't mistakenly assume all the loaders are done.
   //
   // Of course, the Right Thing is good and all. But why bother? A short
   // sleeps works just as well for most practical purposes...
   sleep(1) ; // HACK! to prevent segfault; see comment above
   Thread::wait_all() ;
   purge_container(loader_threads) ;

   // Make sure the dataset actually has something in it that can be
   // analyzed...
   if (dataset.empty()) {
      std::cerr << dir << ": unable to load any metrics logs\n" ;
      return ;
   }
   if (Params::dump_dataset())
      dataset.dump() ;

   // Now that all the metlogs have been loaded into Experiment objects,
   // analyze the data to help gauge the robot's average case behaviour.
   std::string result_file = dir + "/" + Params::result() ;
   Experiment* result = Experiment::create(result_file) ;

   result->point_list(TRAJECTORY,     analyze(dataset, TRAJECTORY)) ;
   result->point_list(EMERGENCY_STOP, analyze(dataset, EMERGENCY_STOP)) ;
   result->point_list(EXTRICATE,      analyze(dataset, EXTRICATE)) ;
   result->point_list(LGMD_EXTRICATE, analyze(dataset, LGMD_EXTRICATE)) ;
   result->point_list(BUMP, analyze_bumps(dataset)) ;
   result->speed_list(analyze_speeds(dataset)) ;
   analyze_events(dataset, result) ;

   if (! result->save())
      std::cerr << result_file << ": will not overwrite\n" ;
   delete result ;
}

// This function finds the "reference experiment" for the specified point
// list and then launches multiple threads to "normalize" the remaining
// experiments so that they all have the same number of points in the
// point list of interest as the reference experiment. The average of all
// these transformed point lists is then used as the final result for
// that category.
//
// To make the above discussion a little more concrete and a little more
// clear, let us say we have 25 experiments in a dataset and want to find
// the robot's average trajectory from start to finish. Now, each of the
// 25 experiments would have recorded some points for the robot's
// trajectory. Unfortunately, we cannot simply take the centroids of the
// "corresponding" points across all experiments because each
// experiment's list of trajectory points will have a different
// cardinality.
//
// For example, the first experiment might record 100 points, the second
// one might record 120 points, the third one 92 points; so on and so
// forth. Therefore, we first need to decide which experiment to use as a
// reference. Once we have the reference experiment, we find point
// correspondences between all the other experiments and it using a
// simple Euclidean distance check.
//
// The above procedure will end up discarding points in experiments that
// have more trajectory points than the reference experiment and
// duplicating points in those experiments that have fewer trajectory
// points than the reference experiment. At the end of this
// transformation, we would have 25 trajectory point lists all with the
// same cardinality. Now we can go ahead and take the centroids of all
// the corresponding points to obtain the final average trajectory.
PointList analyze(Dataset& dataset, PointListName point_list)
{
   using namespace lobot ;

   dataset.rewind() ;
   const Experiment* refexp = dataset.find_refexp(point_list) ;

   PointMatrix point_matrix(refexp->size(point_list), dataset.size()) ;

   const int T = std::min(dataset.size() - 1, num_cpu()) ;
   std::vector<CorrFinder*> corr_threads ;
   corr_threads.reserve(T) ;
   for (int i = 0; i < T; ++i)
      corr_threads.push_back(CorrFinder::create(refexp, dataset,
                                                point_list, &point_matrix)) ;

   sleep(1) ; // prevent segfault; see comment in process_experiment()
   Thread::wait_all() ;
   purge_container(corr_threads) ;

   const PointList& refpl = refexp->point_list(point_list) ;
   if (! refpl.empty())
      point_matrix.add(refpl) ;
   return point_matrix.average() ;
}

// This function "analyzes" the dataset's bump events, which simply
// involves appending each experiment's bump list and returning the
// resulting list as the final result to be saved. We don't bother with
// finding a reference experiment and computing an average w.r.t. that
// reference because the total number of bumps across all experiments
// making up a dataset ought to be fairly low (on the order of 2-5).
// Therefore, in the pretty pictures that all of this data processing
// will eventually lead up to, we simply show all the bump points,
// stating that they are the total across all experiments and not
// averaged like the other point lists.
PointList analyze_bumps(Dataset& dataset)
{
   PointList bumps(10) ;
   try
   {
      dataset.rewind() ;
      for(;;)
      {
         const lobot::Experiment* E = dataset.next() ;
         bumps += E->point_list(lobot::BUMP) ;
      }
   }
   catch (Dataset::eol&){}
   return bumps ;
}

// This function "analyzes" the speed readings recorded by each
// experiment, which involves simply appending all of the readings into
// one giant list. This list is then stored in the result object, which
// takes care of computing mean and standard deviation prior to saving
// the result file.
std::vector<float> analyze_speeds(Dataset& dataset)
{
   std::vector<float> speeds ;
   speeds.reserve(dataset.size() * 30) ;
   try
   {
      dataset.rewind() ;
      for(;;)
      {
         const lobot::Experiment*  E = dataset.next() ;
         const std::vector<float>& S = E->speed_list() ;
         std::copy(S.begin(), S.end(), std::back_inserter(speeds)) ;
      }
   }
   catch (Dataset::eol&){}

   // Convert all speeds from m/s to mm/s to get non-fractional speed
   // values. Fractional floating point numbers result in extreme
   // roundoff errors when we perform the two-way ANOVA computations for
   // the speed_stats.
   using namespace boost::lambda ;
   std::transform(speeds.begin(), speeds.end(), speeds.begin(), _1 * 1000.0f) ;

   return speeds ;
}

// Quick helper to return the mean and standard deviation of a vector of
// integers.
lobot::generic_stats<int> stats(const std::vector<int>& v)
{
   using namespace lobot ;
   generic_stats<float> s = compute_stats<float>(v.begin(), v.end()) ;
   return generic_stats<int>(s.n,
                             round(s.sum),  round(s.ssq),
                             round(s.mean), round(s.stdev)) ;
}

// Quick helper to return the mean and standard deviation of a vector of
// floats.
lobot::generic_stats<float> stats(const std::vector<float>& v)
{
   return lobot::compute_stats<float>(v.begin(), v.end()) ;
}

// This function analyzes the occurences of emergency stop events and LRF
// and LGMD extrication events. It computes the means and standard
// deviations for these event types as well as the means and standard
// deviations for the total number of extrications and the LGMD success
// rate. Finally, it also computes the mean time-to-goal and its standard
// deviation (i.e., the "reached goal" event).
void analyze_events(Dataset& dataset, lobot::Experiment* result)
{
   std::vector<int>   em_stop, lrf_extr, lgmd_extr, total_extr ;
   std::vector<float> lgmd_success, extr_success, durations ;

   em_stop.reserve(dataset.size()) ;
   lrf_extr.reserve(dataset.size()) ;
   lgmd_extr.reserve(dataset.size()) ;
   total_extr.reserve(dataset.size()) ;
   lgmd_success.reserve(dataset.size()) ;
   extr_success.reserve(dataset.size()) ;
   durations.reserve(dataset.size()) ;

   try
   {
      dataset.rewind() ;
      for(;;)
      {
         const lobot::Experiment* E = dataset.next() ;

         int stops = E->emergency_stop_size() ;
         em_stop.push_back(stops) ;

         int lrf = E->extricate_size() ;
         lrf_extr.push_back(lrf) ;

         int lgmd = E->lgmd_extricate_size() ;
         lgmd_extr.push_back(lgmd) ;

         int total = lrf + lgmd ;
         total_extr.push_back(total) ;

         lgmd_success.push_back(lgmd  * 100.0f/total) ;
         extr_success.push_back(total * 100.0f/stops) ;

         durations.push_back(E->duration()/1000.0f) ;
      }
   }
   catch (Dataset::eol&){}

   result->emergency_stop_stats(stats(em_stop)) ;
   result->lrf_extricate_stats(stats(lrf_extr)) ;
   result->lgmd_extricate_stats(stats(lgmd_extr)) ;
   result->total_extricate_stats(stats(total_extr)) ;
   result->lgmd_success_stats(stats(lgmd_success)) ;
   result->extricate_success_stats(stats(extr_success)) ;
   result->duration_stats(stats(durations)) ;
}

// This function walks through the list of directories specified on the
// command line and processes the datasets in each of them one-by-one.
void process_metlogs(const DirList& dirs)
{
   std::for_each(dirs.begin(), dirs.end(), process_experiment) ;
}

} // end of local anonymous namespace encapsulating above helpers

//------------------------------- MAIN ----------------------------------

int main(int argc, char* argv[])
{
   int ret = 0 ;
   try
   {
      CmdLine args = parse(argc, argv) ;
      if (args.second.empty())
         throw lobot::misc_error(lobot::MISSING_CMDLINE_ARGS) ;

      load_config_file(args.first) ;
      process_metlogs(args.second) ;
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
