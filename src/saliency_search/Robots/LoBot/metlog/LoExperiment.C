/**
   \file  Robots/LoBot/misc/LoExperiment.C
   \brief This file defines the non-inline member functions of the
   lobot::Experiment class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoExperiment.C $
// $Id: LoExperiment.C 14285 2010-12-01 17:33:15Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoExperiment.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/util/LoFile.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"

// Standard C++ headers
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

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
class ExperimentParams : public singleton<ExperimentParams> {
   /// The metlog files collected as part of the trajectory experiments
   /// will contain several "tracking speed" entries. Some of these speed
   /// entries will have negative or small values, indicating brief
   /// periods wherein the robot was backing up from an obstacle or
   /// speeding up to its normal forward driving speed.
   ///
   /// These negative and small speeds will incorrectly skew the average
   /// forward driving speed computation. To work around the problem, we
   /// ignore speed entries below a certain threshold.
   ///
   /// This setting specifies the value of the above-mentioned threshold.
   /// It should be a a floating point number expressed in meters per
   /// second.
   ///
   /// NOTE: A good way to pick this threshold is to look at the forward
   /// behaviour's section in the lobot config file used in conjunction
   /// with the experiments that yielded the dataset being used as the
   /// input for the lomet program and set this value to something
   /// reasonably close to but slightly lower than the cruising speed
   /// configured there.
   ///
   /// Alternatively, we can also peek at the metlogs themselves and
   /// decide on a suitable figure.
   float m_forward_speed_threshold ;

   /// Private constructor because this is a singleton.
   ExperimentParams() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<ExperimentParams> ;

public:
   /// Accessing the various parameters.
   //@{
   static float forward_speed_threshold() {
      return instance().m_forward_speed_threshold ;
   }
   //@}
} ;

// Parameters initialization
ExperimentParams::ExperimentParams()
   : m_forward_speed_threshold(conf("forward_speed_threshold", 0.099f))
{}

// Shortcut
typedef ExperimentParams Params ;

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

Experiment* Experiment::create(const std::string& name)
{
   return new Experiment(name) ;
}

Experiment::Experiment(const std::string& name)
   : m_name(name),
     m_start_time(0), m_finis_time(0),
     m_trajectory(300),
     m_emergency_stop(100),
     m_extricate(100),
     m_lgmd_extricate(100),
     m_bump(5),
     m_emstop_stats(0, 0),
     m_lrf_extr_stats(0, 0),
     m_lgmd_extr_stats(0, 0),
     m_total_extr_stats(0, 0),
     m_lgmd_success_stats(0, 0),
     m_duration_stats(0, 0)
{
   m_speed.reserve(40) ;
}

//-------------------------- RECORDING DATA -----------------------------

// Add a point to the named list
void Experiment::add_point(PointListName n, int x, int y)
{
   switch (n)
   {
      case TRAJECTORY:
         add_trajectory(x, y) ;
         break ;
      case EMERGENCY_STOP:
         add_emergency_stop(x, y) ;
         break ;
      case EXTRICATE:
         add_extricate(x, y) ;
         break ;
      case LGMD_EXTRICATE:
         add_lgmd_extricate(x, y) ;
         break ;
      case BUMP:
         add_bump(x, y) ;
         break ;
      default:
         throw misc_error(LOGIC_ERROR) ;
   }
}

// Add an entire point list instead of one piddling point at a time...
void Experiment::point_list(PointListName n, const PointList& L)
{
   switch (n)
   {
      case TRAJECTORY:
         m_trajectory = L ;
         break ;
      case EMERGENCY_STOP:
         m_emergency_stop = L ;
         break ;
      case EXTRICATE:
         m_extricate = L ;
         break ;
      case LGMD_EXTRICATE:
         m_lgmd_extricate = L ;
         break ;
      case BUMP:
         m_bump = L ;
         break ;
      default:
         throw misc_error(LOGIC_ERROR) ;
   }
}

// We are only interested in recording those speeds that correspond to
// normal forward driving. To determine whether a speed qualifies for
// this exalted status, we use a threshold specified in the config file.
// Values below this threshold will be assumed to indicate the robot
// backing up or accelerating to a "normal" forward driving speed.
void Experiment::add_speed(float speed)
{
   if (speed >= Params::forward_speed_threshold())
      m_speed.push_back(speed) ;
}

// Append an entire vector of speed readings to this object's list of
// speed readings.
void Experiment::speed_list(const std::vector<float>& speeds)
{
   std::copy(speeds.begin(), speeds.end(), std::back_inserter(m_speed)) ;
}

//-------------------------- ACCESSING DATA -----------------------------

int Experiment::size(PointListName n) const
{
   switch (n)
   {
      case TRAJECTORY:
         return trajectory_size() ;
      case EMERGENCY_STOP:
         return emergency_stop_size() ;
      case EXTRICATE:
         return extricate_size() ;
      case LGMD_EXTRICATE:
         return lgmd_extricate_size() ;
      case BUMP:
         return bump_size() ;
   }
   throw misc_error(LOGIC_ERROR) ;
}

const PointList& Experiment::point_list(PointListName n) const
{
   switch (n)
   {
      case TRAJECTORY:
         return m_trajectory ;
      case EMERGENCY_STOP:
         return m_emergency_stop ;
      case EXTRICATE:
         return m_extricate ;
      case LGMD_EXTRICATE:
         return m_lgmd_extricate ;
      case BUMP:
         return m_bump ;
   }
   throw misc_error(LOGIC_ERROR) ;
}

//---------------------------- SAVING DATA ------------------------------

static void
save_point_list(std::ostream& os, const PointList& L, const char* label)
{
   for (PointList::iterator i = L.begin(); i != L.end(); ++i)
      os << label << ' ' << i->first << ' ' << i->second << '\n' ;
}

static std::ostream&
operator<<(std::ostream& os, const generic_stats<int>& s)
{
   os << s.n    << ' ' << s.sum << ' ' << s.ssq << ' '
      << s.mean << ' ' << s.stdev ;
   return os ;
}

static std::ostream&
operator<<(std::ostream& os, const generic_stats<float>& s)
{
   using std::setprecision ; using std::fixed ;
   os << s.n << ' '
      << setprecision(3) << fixed << s.sum  << ' '
      << setprecision(3) << fixed << s.ssq  << ' '
      << setprecision(3) << fixed << s.mean << ' '
      << setprecision(3) << fixed << s.stdev ;
   return os ;
}

bool Experiment::save() const
{
   if (exists(m_name.c_str()))
      return false ;

   std::ofstream ofs(m_name.c_str()) ;
   save_point_list(ofs, m_trajectory,     "trajectory") ;
   save_point_list(ofs, m_emergency_stop, "emergency_stop") ;
   save_point_list(ofs, m_extricate,      "extricate") ;
   save_point_list(ofs, m_lgmd_extricate, "lgmd_extricate") ;
   save_point_list(ofs, m_bump,           "bump") ;

   ofs << "emstop_stats " << m_emstop_stats << '\n' ;
   ofs << "lrf_extrication_stats "   << m_lrf_extr_stats     << '\n' ;
   ofs << "lgmd_extrication_stats "  << m_lgmd_extr_stats    << '\n' ;
   ofs << "total_extrication_stats " << m_total_extr_stats   << '\n' ;
   ofs << "lgmd_success_rate_stats " << m_lgmd_success_stats << '\n' ;
   ofs << "extr_success_rate_stats " << m_extr_success_stats << '\n' ;
   ofs << "speed_stats "
       << compute_stats<float>(m_speed.begin(),m_speed.end())<< '\n' ;
   ofs << "duration_stats " << m_duration_stats << '\n' ;

   return true ;
}

//--------------------------- DEBUG SUPPORT -----------------------------

static void
dump_trajectory(std::ostream& os, const PointList& L, const char* label)
{
   os << L.size() << ' ' << label << " points:\n\t" ;

   using namespace std ;
   for (PointList::iterator i = L.begin(); i != L.end(); ++i)
      os << '(' << left << setw(6) << i->first << i->second << ")\n\t" ;
   os << "\n\n" ;
}

void Experiment::dump() const
{
   using namespace std ;

   ofstream ofs((m_name + ".dump").c_str()) ;
   ofs << "name: " << m_name << "\n\n" ;

   ofs << "start time: " << m_start_time    << '\n' ;
   ofs << "finis time: " << m_finis_time    << '\n' ;
   ofs << "  duration: " << setprecision(3) << fixed
       << (duration()/1000.0f) << " seconds\n\n" ;

   dump_trajectory(ofs, m_trajectory, "normal trajectory") ;
   dump_trajectory(ofs, m_emergency_stop, "emergency stop") ;
   dump_trajectory(ofs, m_extricate, "extricate") ;
   dump_trajectory(ofs, m_lgmd_extricate, "LGMD extricate") ;
   dump_trajectory(ofs, m_bump, "bump") ;

   std::pair<float, float> s =
      mean_stdev<float>(m_speed.begin(), m_speed.end()) ;
   ofs << "forward driving speed = "
       << setprecision(3) << fixed << s.first  << " m/s +/- "
       << setprecision(3) << fixed << s.second << " m/s from "
       << m_speed.size()  << " readings:\n\t" ;
   copy(m_speed.begin(), m_speed.end(),
        ostream_iterator<float>(ofs, " m/s\n\t")) ;
   ofs << '\n' ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
