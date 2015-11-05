/**
   \file  Robots/LoBot/misc/LoExperiment.H
   \brief An object to store a parsed metrics log.

   This file defines a class that implements a simple object for storing
   a parsed metrics log, i.e., this class holds all the relevant data for
   an experiment conducted to record the robot's trajectory from start
   position to goal using an LGMD-based obstacle avoidance algorithm.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoExperiment.H $
// $Id: LoExperiment.H 14285 2010-12-01 17:33:15Z mviswana $
//

#ifndef LOBOT_EXPERIMENT_DOT_H
#define LOBOT_EXPERIMENT_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoPointList.H"
#include "Robots/LoBot/metlog/LoPointTypes.H"
#include "Robots/LoBot/util/LoStats.H"

// Standard C++ headers
#include <string>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Experiment
   \brief A class for storing all the relevant info from
   trajectory-related metrics logs.

   This class is used to hold all of the relevant data from a metlog and
   provide a convenient API for assisting with various number crunching
   tasks for the data analysis.
*/
class Experiment {
   // Prevent copy and assignment
   Experiment(const Experiment&) ;
   Experiment& operator=(const Experiment&) ;

   /// Each experiment is loaded from a metrics log. This data member is
   /// used to store the name of that log file.
   std::string m_name ;

   /// These two data members hold the experiment's start and end times
   /// expressed as the number of milliseconds elapsed since the Unix
   /// epoch.
   ///
   /// DEVNOTE: We use "finis" rather than "finish" because "finis" is
   /// the same length string as "start" and this makes code line up
   /// nicely.
   long long int m_start_time, m_finis_time ;

   /// As the robot moves from start to finish, it periodically records
   /// its position. Some of these position checks correspond to specific
   /// events such as an emergency stop, an extrication or a bump. Most
   /// of them are simply trajectory information recorded for analysis.
   /// These lists are used to store the positions in their proper
   /// "slots."
   //@{
   PointList m_trajectory ;
   PointList m_emergency_stop ;
   PointList m_extricate ;
   PointList m_lgmd_extricate ;
   PointList m_bump ;
   //@}

   /// In addition to recording its position, the robot also periodically
   /// records its speed. This list holds all the speed readings.
   std::vector<float> m_speed ;

   /// These data members record the means and standard deviations for
   /// the different kinds of events we are interested in analyzing from
   /// all the experiments in a dataset. In addition to the means and
   /// standard deviations (which are used to produce pretty plots), we
   /// also record some other values such as the number of data points,
   /// the sum of all the data and the sum of the squares. These values
   /// are used for two-way ANOVA.
   ///
   /// DEVNOTE: This class does double duty: it stores the information
   /// extracted by the metlog parsing process and is also used to store
   /// the final data analysis results.
   ///
   /// Yes, this is lousy design.
   ///
   /// However, since lomet was originally conceived as a
   /// quick-and-dirty, one-time use, throwaway kind of program,
   /// expediency was put above design correctness, extensibility,
   /// robustness and all that jazz.
   //@{
   typedef generic_stats<int>   istats ; // shortcut to make code line up
   typedef generic_stats<float> fstats ; // shortcut to make code line up
   istats m_emstop_stats,
          m_lrf_extr_stats, m_lgmd_extr_stats, m_total_extr_stats ;
   fstats m_lgmd_success_stats, m_extr_success_stats, m_duration_stats ;
   //@}

   /// A private constructor because Experiment objects are created with
   /// a "named constructor" to ensure that clients don't create them on
   /// the stack.
   Experiment(const std::string& name) ;

public:
   /// This method can be used to create Experiment objects. This "named
   /// constructor" ensures that all instances of this class are created
   /// with the new operator and not as local objects on the stack.
   static Experiment* create(const std::string& name) ;

   /// Return this experiment's name.
   std::string name() const {return m_name ;}

   /// These methods record the times at which the experiment began and
   /// ended. These time stamps are expressed as the number of
   /// milliseconds elapsed since the Unix epoch.
   ///
   /// These methods are meant to be used by the metlog parsing module,
   /// i.e., within the lex-generated rules in LoMetlogParser.l.
   ///
   /// DEVNOTE: We use "finis" rather than "finish" because "finis" is
   /// the same length string as "start" and this makes code line up
   /// nicely (as can be seen with the definitions of these functions).
   //@{
   void start_time(long long int t) {m_start_time = t ;}
   void finis_time(long long int t) {m_finis_time = t ;}
   //@}

   /// This function returns the duration of the experiment in
   /// milliseconds.
   int duration() const {return m_finis_time - m_start_time ;}

   /// These methods add trajectory info to their corresponding lists.
   /// They are meant to be used by the metlog parsing module, i.e.,
   /// within the lex-generated rules in LoMetlogParser.l.
   //@{
   void add_trajectory(int x, int y)     {m_trajectory.add(x, y)     ;}
   void add_emergency_stop(int x, int y) {m_emergency_stop.add(x, y) ;}
   void add_extricate(int x, int y)      {m_extricate.add(x, y)      ;}
   void add_lgmd_extricate(int x, int y) {m_lgmd_extricate.add(x, y) ;}
   void add_bump(int x, int y)           {m_bump.add(x, y)           ;}
   //@}

   /// This method adds a point to the named list.
   void add_point(PointListName, int x, int y) ;

   /// This method adds an entire point list to the one named by its
   /// parameter. It is meant to be used by the lomet program's main
   /// thread for building the result "experiment" from the analysis
   /// performed on the entire dataset.
   void point_list(PointListName, const PointList&) ;

   /// These functions return the sizes of the different point lists
   /// maintained by an experiment.
   //@{
   int trajectory_size()     const {return m_trajectory.size()     ;}
   int emergency_stop_size() const {return m_emergency_stop.size() ;}
   int extricate_size()      const {return m_extricate.size()      ;}
   int lgmd_extricate_size() const {return m_lgmd_extricate.size() ;}
   int bump_size()           const {return m_bump.size()           ;}
   //@}

   /// This function returns the size of the named point list.
   int size(PointListName) const ;

   /// This method returns the named point list.
   ///
   /// NOTE: This method returns a reference to an internal data
   /// structure. Clients should consider the returned data structure
   /// read-only, i.e., they should refrain from casting away the
   /// constness of the returned object and doing nasty things to it.
   const PointList& point_list(PointListName) const ;

   /// This method adds a speed reading to the list of speed readings
   /// subject to the condition that the speed actually represents a
   /// forward driving speed. This information is used to compute the
   /// average forward driving speed over the entire run.
   ///
   /// This method is meant to be used by the metlog parsing module,
   /// i.e., with the lex-generated rules in LoMetlogParser.l.
   void add_speed(float) ;

   /// This method adds all the speed readings in the given std::vector
   /// to this object's list of speed readings.
   ///
   /// NOTE: This method is meant to be used by the lomet main thread for
   /// computing the final result, which is represented using an instance
   /// of this class. Yes, dreadful design. Ideally, we should have split
   /// the result related functionality into a separate class, viz.,
   /// Result, which could have used private derivation from Experiment
   /// to take advantage of the functionality encapsulated here. Anyway,
   /// since the lomet program was originally conceived as a one-time,
   /// throwaway sort of thing, these shortcuts were deemed acceptable.
   void speed_list(const std::vector<float>&) ;

   /// This function returns an std::vector of floats containing the
   /// speed readings recorded by this experiment.
   ///
   /// WARNING: This function actually returns a const reference to an
   /// internal data structure. Yes, yes, bad design. Anyhoo, clients are
   /// advised/requested to treat the returned object as read-only.
   const std::vector<float>& speed_list() const {return m_speed ;}

   /// These functions record the means and standard deviations for
   /// various things such as the time-to-goal, number of extrication
   /// events, LGMD-based obstacle avoidance algorithm success rate, etc.
   /// These functions are meant to be used by the lomet main thread as
   /// part of the results computation (double duty, bad design, etc.;
   /// see comments appearing earlier).
   ///
   /// NOTE: In addition to the means and standard deviations, we also
   /// record some other values such as the n and sum and sum of squares.
   /// These are useful for two-way ANOVA.
   //@{
   void emergency_stop_stats   (const istats& s) {m_emstop_stats       = s ;}
   void lrf_extricate_stats    (const istats& s) {m_lrf_extr_stats     = s ;}
   void lgmd_extricate_stats   (const istats& s) {m_lgmd_extr_stats    = s ;}
   void total_extricate_stats  (const istats& s) {m_total_extr_stats   = s ;}
   void lgmd_success_stats     (const fstats& s) {m_lgmd_success_stats = s ;}
   void extricate_success_stats(const fstats& s) {m_extr_success_stats = s ;}
   void duration_stats         (const fstats& s) {m_duration_stats     = s ;}
   //@}

   /// This method saves the experiment to a file whose name is the same
   /// as the experiment's name. It is meant to be called by the lomet
   /// main thread for saving the data analysis results.
   ///
   /// It returns false to indicate that saving failed; this would happen
   /// if there is already an extant file that has the same name as this
   /// experiment. This feature ensures that metlogs loaded and parsed
   /// into an Experiment won't be overwritten if a client inadvertently
   /// calls this function. It also ensures that a previous result file
   /// won't be overwritten unless the user explicitly takes some action
   /// to allow the save to proceed, e.g., delete the old result file or
   /// reconfigure the program to save results to a differently named
   /// file.
   bool save() const ;

   /// Debugging support: dumps all the info to a file named dump-foo,
   /// where "foo" is actually the experiment's name, i.e., the same as
   /// the name of the log file from which this data object was created.
   void dump() const ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
