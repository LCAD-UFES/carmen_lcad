/**
   \file  Robots/LoBot/control/LoTestScanMatching.C
   \brief This file defines the non-inline member functions of the
   lobot::TestScanMatching class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoTestScanMatching.C $
// $Id: LoTestScanMatching.C 13570 2010-06-16 15:56:00Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoTestScanMatching.H"
#include "Robots/LoBot/slam/LoScanMatch.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/misc/LoRegistry.H"

// Standard C++ headers
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from test_idc section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_TEST_SCAN_MATCHING, key, default_value) ;
}

// Retrieve a line from the given input stream
static std::string getline(std::istream& is)
{
   std::string line ;
   std::getline(is, line) ;
   return line ;
}

// Convert a line read from the scan data file to a Scan object
static Scan to_scan(const std::string& line)
{
   std::istringstream str(line) ;

   float x, y, theta ;
   str >> x >> y >> theta ;

   Pose P(x, y, theta) ;
   std::vector<float> R =
      std::vector<float>(std::istream_iterator<float>(str),
                         std::istream_iterator<float>()) ;

   return Scan(P, R) ;
}

//-------------------------- INITIALIZATION -----------------------------

TestScanMatching::TestScanMatching()
   : base(clamp(conf("update_delay", 500), 1, 1500))
{
   start(LOBE_TEST_SCAN_MATCHING) ;
}

void TestScanMatching::pre_run()
{
   std::string file_name =
      conf<std::string>("test_data", "/tmp/lobot-scan-matching.dat") ;
   std::ifstream scan_file(file_name.c_str()) ;

   Scan prev(to_scan(getline(scan_file))) ;
   Scan curr(to_scan(getline(scan_file))) ;

   match_scans(curr, prev) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// All the processing takes place in pre_run()...
void TestScanMatching::action()
{
   Shutdown::signal() ;
}

//----------------------------- CLEAN-UP --------------------------------

TestScanMatching::~TestScanMatching(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
