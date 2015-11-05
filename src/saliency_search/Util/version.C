/*!@file Util/version.C strings giving version info about the package */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/version.C $
// $Id: version.C 6357 2006-03-10 00:07:35Z rjpeters $
//

#ifndef UTIL_VERSION_C_DEFINED
#define UTIL_VERSION_C_DEFINED

#include "Util/version.H"

#include "Util/log.H"
#include "Util/sformat.H"

#include <cctype>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef PACKAGE_NAME
#define PACKAGE_NAME "packagename-unknown"
#endif

#ifndef PACKAGE_VERSION
#define PACKAGE_VERSION "packageversion-unknown"
#endif

#ifndef PACKAGE_TARNAME
#define PACKAGE_TARNAME "packagetarname-unknown"
#endif

namespace
{
  // don't access these variables directly; instead always go through
  // svnversion() since that will ensure that it is properly
  // initialized via a pthread_once() call
  std::string g_fullversion;
  std::string g_svnversion;
  std::string g_date;

  pthread_once_t version_init_once = PTHREAD_ONCE_INIT;

  void version_init()
  {
    g_svnversion = "svnversion-unknown";
    g_date = __DATE__;

#ifdef INVT_LIB_DIR
    LDEBUG("looking for svnversion in %s", INVT_LIB_DIR "/buildversion");

    // we flush stderr so that the previous LDEBUG() message is sure
    // to reach the terminal before we try to open the file in the
    // next line; that way, in case the filesystem hangs for some
    // reason, the user will have an idea where to start looking for
    // the problem
    fflush(stderr);

    std::ifstream f(INVT_LIB_DIR "/buildversion");

    if (f.is_open())
      {
        std::string s;

        f >> s;

        if (s.length() != 0)
          g_svnversion = s;

        LDEBUG("got svnversion=%s", g_svnversion.c_str());

        struct stat statbuf;
        const int code = stat(INVT_LIB_DIR "/buildversion", &statbuf);

        if (code == 0)
          {
            // use the last-modification time of the buildversion time
            // to determine the date in the fullversion string; this
            // is more accurate than the value of __DATE__ in this
            // source file (version.C), since __DATE__ changes only
            // when version.C is recompiled, while the timestamp on
            // buildversion changes each time 'make' is called
            const time_t t = statbuf.st_mtime;
            struct tm tt;

            if (localtime_r(&t, &tt) != 0)
              {
                const char* format = "%a %b %d %H:%M:%S %Z %Y";
                char buf[512];
                const size_t count =
                  strftime(buf, sizeof(buf), format, &tt);

                buf[sizeof(buf)-1] = '\0';

                if (count > 0)
                  g_date = &buf[0];
              }
          }
      }
    else
      {
        LDEBUG("couldn't open %s for reading", INVT_LIB_DIR "/buildversion");
      }
#endif

    g_fullversion = sformat("%s (%s) %s [%s] (%s)",
                            PACKAGE_NAME,
                            PACKAGE_TARNAME,
                            PACKAGE_VERSION,
                            g_date.c_str(),
                            g_svnversion.c_str());
  }
}

const char* fullversion()
{
  pthread_once(&version_init_once, &version_init);
  return g_fullversion.c_str();
}

const char* svnversion()
{
  pthread_once(&version_init_once, &version_init);
  return g_svnversion.c_str();
}

bool isSourcePristine()
{
  bool gotdigits = false;
  bool gotnondigits = false;

  for (const char* p = svnversion(); *p != '\0'; ++p)
    {
      if (isdigit(*p)) gotdigits = true;
      else             gotnondigits = true;
    }

  // to be pristine we need to have a non-empty string with digits and
  // only digits (i.e. 5864 is OK, but 5864M, 5864:5865, and
  // 5864:5865M are not OK):
  return gotdigits && !gotnondigits;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_VERSION_C_DEFINED
