/**
   \file  Robots/LoBot/util/LoTime.C
   \brief Time related functions.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoTime.C $
// $Id: LoTime.C 13838 2010-08-27 20:42:20Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/LoTime.H"

// Standard C headers
#include <time.h>

// Standard Unix headers
#include <sys/time.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//---------------------- TIME RELATED FUNCTIONS -------------------------

// Current time as number of milliseconds since Unix epoch
long long current_time()
{
   struct timeval T ;
   if (gettimeofday(&T, 0) == -1) // error getting current time
      return 0 ;
   return (static_cast<long long>(T.tv_sec) * 1000000L
         + static_cast<long long>(T.tv_usec))/1000L ;
}

// Uniform start-up time-stamp string for log files, directory names, etc.
//
// WARNING: NOT THREAD SAFE!
const char* startup_timestamp_str()
{
   static char app_start_time[20] ;
   if (app_start_time[0] == '\0')
   {
      time_t t = time(0) ;
      struct tm now ;
      strftime(app_start_time, sizeof(app_start_time),
               "%Y%m%d-%H%M%S", localtime_r(&t, &now)) ;
   }
   return app_start_time ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
