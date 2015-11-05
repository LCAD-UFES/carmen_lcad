/**
   \file  Robots/LoBot/control/LoTestScanMatching.H
   \brief A behaviour for testing the scan matching module using some
   test data to see how well the algorithm works.

   This file defines a class that uses some test data to see whether or
   not the Robolocust implementation of the laser range finder scan
   matching algorithm works or not. This behaviour does not affect the
   robot in any way; it is, in fact, simply an offline test used to check
   the scan matching algorithm.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoTestScanMatching.H $
// $Id: LoTestScanMatching.H 13570 2010-06-16 15:56:00Z mviswana $
//

#ifndef LOBOT_TEST_SCAN_MATCHING_BEHAVIOUR_DOT_H
#define LOBOT_TEST_SCAN_MATCHING_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/factory.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::TestScanMatching
   \brief A behaviour for testing the laser range finder scan matching
   algorithm.

   This class implements a behaviour that uses some test data to check
   whether the Robolocust's scan matching algorithm works or not. It
   reads the test data from a file and feeds that to the scan matching
   module. This behaviour does not affect the robot in any way; rather it
   is meant to be run standalone and not even on the robot.
*/
class TestScanMatching : public Behavior {
   // Prevent copy and assignment
   TestScanMatching(const TestScanMatching&) ;
   TestScanMatching& operator=(const TestScanMatching&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<TestScanMatching, base> ;
   typedef register_factory<TestScanMatching, base> my_factory ;
   static  my_factory register_me ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   TestScanMatching() ;

   /// Some things to do before commencing regular action processing.
   void pre_run() ;

   /// This method provides the body of the behaviour's main loop.
   void action() ;

   /// Clean-up.
   ~TestScanMatching() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
