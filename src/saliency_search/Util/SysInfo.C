/*!@file Util/SysInfo.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/SysInfo.C $
// $Id: SysInfo.C 8433 2007-05-25 18:44:47Z rjpeters $
//

#ifndef UTIL_SYSINFO_C_DEFINED
#define UTIL_SYSINFO_C_DEFINED

#include "Util/SysInfo.H"

#include "rutz/pipe.h"

#include <fstream>
#include <sstream>

int numCpus()
{
  std::ifstream ifs("/proc/cpuinfo");
  if (ifs.is_open())
    {
      int n = 0;
      std::string line;
      while (std::getline(ifs, line))
        {
          std::istringstream iss(line);
          std::string word;
          iss >> word;
          if (word.compare("processor") == 0)
            ++n;
        }
      return n;
    }
  else
    {
      rutz::exec_pipe p("r", "/usr/sbin/sysctl", "-a", "hw", NULL);
      std::string line;
      while (std::getline(p.stream(), line))
        {
          std::istringstream iss(line);
          std::string key;
          iss >> key;
          if (key.compare("hw.availcpu") == 0)
            {
              std::string equal;
              iss >> equal;
              if (equal.compare("=") == 0)
                {
                  int n = -1;
                  iss >> n;
                  return n;
                }
            }
        }
    }

  return -1;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_SYSINFO_C_DEFINED
