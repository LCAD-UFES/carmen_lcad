/**
   \file  Robots/LoBot/control/LoRenderResults.H
   \brief An offline behaviour for rendering the results of the
   trajectory experiments.

   The Robolocust project aims to use locusts for robot navigation.
   Specifically, the locust sports a visual interneuron known as the
   Lobula Giant Movement Detector (LGMD) that spikes preferentially in
   response to objects moving toward the animal on collisional
   trajectories. Robolocust's goal is to use an array of locusts, each
   looking in a different direction. As the robot moves, we expect to
   receive greater spiking activity from the locusts looking in the
   direction in which obstacles are approaching (or being approached) and
   use this information to veer the robot away.

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

   The lobot program, properly configured, was used to produce metrics
   log files for the individual experiments. The lomet program was used
   to parse these metlogs and produce information regarding the robot's
   average-case behaviour associated with a dataset. These files are
   referred to as results files.

   The lobot::RenderResults class defined here implements an offline
   behaviour to read metlog and results files and visualize them on the
   map of the slalom enclosure. This behaviour is not meant to control
   the robot; rather, it is supposed to visualize the robot's trajectory
   from start to finish and plot the points where its emergency stop and
   extrication behaviours were active and those points where the robot
   bumped into things. As mentioned above, the point lists for the
   trajectory, emergency stop and extrication behaviours and bump
   locations comes from either metlog files produced by lobot or results
   files output by lomet.

   The ultimate objective behind this visualization is to be able to
   collect screenshots that can then be presented as data in papers,
   theses, etc. We could have written Perl/Python scripts to process the
   metlog and results files and generate pstricks code for inclusion in
   LaTeX documents. However, since the lobot program already implements
   map visualization and screen capturing, it is easier to bootstrap off
   off that functionality to produce JPG/PNG images that can then be
   included by LaTeX.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoRenderResults.H $
// $Id: LoRenderResults.H 13982 2010-09-19 09:45:34Z mviswana $
//

#ifndef LOBOT_RENDER_RESULTS_BEHAVIOUR_DOT_H
#define LOBOT_RENDER_RESULTS_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/metlog/LoPointList.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <istream>
#include <vector>
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declaration

/**
   \class lobot::RenderResults
   \brief An offline behaviour for rendering the robot's trajectory from
   start to finish and the locations where its emergency stop and
   extrication behaviours were active.

   This class implements an offline behaviour that reads the obstacle map
   and goal location used to test different LGMD-based avoidance
   algorithms in a local navigation task; it also reads, from either a
   metlog or results file, the list of points comprising the robot's
   trajectory, the locations where its emergency stop and extrication
   behaviours were activated, and the locations where it bumped into
   things. It then renders all of these things in the Robolocust UI.

   This behaviour is meant to be used to collect screen shots for
   inclusion in papers, etc. illustrating the performance of the
   LGMD-based obstacle avoidance algorithms implemented by Robolocust. It
   does not and is not meant to control the robot; rather, it implements
   an offline, rendering task to be used in conjunction with certain data
   processing and analysis efforts.
*/
class RenderResults : public Behavior {
   // Prevent copy and assignment
   RenderResults(const RenderResults&) ;
   RenderResults& operator=(const RenderResults&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<RenderResults, base> ;
   typedef register_factory<RenderResults, base> my_factory ;
   static  my_factory register_me ;

   /// We assume that the lomet program was used to store results for
   /// different datasets in subdirectories of a "root" data directory.
   /// When this behaviour starts up, it will create a list of all these
   /// results files and allow the user to walk through the list with the
   /// '+' or '-' keys, starting, of course, at the first name in the
   /// list. As each name in the list is visited, the behaviour will load
   /// and visualize the corresponding results file.
   ///
   /// This data member holds the list of result file names as described
   /// above.
   //@{
   typedef std::vector<std::string> ResultsList ;
   ResultsList m_results_files ;
   //@}

   /// This iterator points to the current entry in the results files
   /// list, i.e., the file currently being visualized.
   ResultsList::const_iterator m_results ;

   /// A helper struct for holding a goal's bounds (in map coordinates).
   struct Goal {
      float left, right, bottom, top ;

      /// Helper to read a goal from an input stream.
      friend std::istream& operator>>(std::istream& is, Goal& g) {
         return is >> g.left >> g.right >> g.bottom >> g.top ;
      }
   } ;

   /// This data member holds the goal specified for the slalom obstacle
   /// course. The render_results behaviour reads this information from
   /// the goal behaviour's section of the config file.
   ///
   /// NOTE: Although the goal behaviour supports a list of goals, only
   /// one was actually used in the trajectory recording experiments.
   /// Accordingly, this behaviour only bothers with the first goal from
   /// the entire list (which should, anyway, contain only one entry).
   Goal m_goal ;

   /// Every time the render_results behaviour loads a results file, it
   /// will store the different point lists of interest in these data
   /// members.
   PointList m_traj, m_stop, m_extr, m_lgmd, m_bump ;

   /// This behaviour supports a slideshow mode, wherein, periodically,
   /// it switches to the next file in its list of metlogs and/or results
   /// files to be visualized. In this mode, the user may want to
   /// temporarily pause the slideshow when a results file is being
   /// visualized after a long string of metlogs in order to be able to
   /// study the average-case behaviour in greater length.
   ///
   /// This flag keeps track of the type of file currently being
   /// visualized. It is true when the file is a dataset results file and
   /// false when it is a metlog.
   bool m_results_file_being_visualized ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   RenderResults() ;

   /// When the behaviour starts up, we will read the map file, the start
   /// and end points and the various point lists from the results file.
   void pre_run() ;

   /// These functions load the specified file that is to be visualized.
   //@{
   void load(const std::string& file_name) ;
   void load_metlog (const std::string& file_name) ;
   void load_results(const std::string& file_name) ;
   //@}

   /// This behaviour's action isn't much: it simply implements a
   /// slideshow feature that allows users to sit back and let the
   /// Robolocust application automatically display each metlog or
   /// results file one-by-one.
   void action() ;

   /// This function responds to the '+', '-' and 's' keys to load the
   /// next or previous results file and to take a screenshot of the
   /// current one.
   void keypress(unsigned char key) ;

   /// These functions walk through the results files list.
   //@{
   void prev() ;
   void next() ;
   //@}

   /// This function returns the name of the current results file being
   /// visualized.
   std::string curr() ;

   /// These methods render the start point, the goal, the robot's
   /// trajectory and locations where the emergency stop and extrication
   /// behaviours were active.
   //@{
   static void render_results(unsigned long client_data) ;
   void render_goal() const ;
   void render_traj() const ;
   //@}

   /// Clear all the point lists.
   void clear_point_lists() ;

   /// Clean-up.
   ~RenderResults() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
