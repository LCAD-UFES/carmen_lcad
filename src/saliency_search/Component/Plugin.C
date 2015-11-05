/*!@file Component/Plugin.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/Plugin.C $
// $Id: Plugin.C 7425 2006-11-09 23:46:58Z rjpeters $
//

#ifndef COMPONENT_PLUGIN_C_DEFINED
#define COMPONENT_PLUGIN_C_DEFINED

#include "Component/Plugin.H"

#include "Util/StringUtil.H" // for toLowerCase()
#include "Util/log.H"
#include "Util/sformat.H"

#include <dlfcn.h>

bool loadPlugin(OptionManager* mgr,
                const char* realm, const char* name)
{
  const std::string soname = sformat("%s/%s/%s.so",
                                     INVT_PLUGIN_DIR, realm, name);

  const std::string initname =
    toLowerCase(sformat("%s_%s_init", realm, name));

  LDEBUG("opening %s", soname.c_str());

  // open the shared object with RTLD_LAZY -- this means don't resolve
  // all symbols now, which makes the dlopen() faster, but also makes
  // errors asynchronous

  void* const handle = dlopen(soname.c_str(), RTLD_LAZY);
  const char* err = dlerror();
  if (err != 0)
    {
      LDEBUG("dlopen() failed: %s", err);
      return false;
    }

  LDEBUG("looking for %s() in %s", initname.c_str(), soname.c_str());

  void* const sym = dlsym(handle, initname.c_str());
  err = dlerror();
  if (err != 0)
    {
      LDEBUG("dlsym() failed: %s", err);
      dlclose(handle);
      return false;
    }

  PluginInitFunc* f = (PluginInitFunc*)(sym);

  const int status = (*f)(mgr);

  if (status == 0)
    LINFO("loaded %s/%s plugin (from %s)",
          realm, name, soname.c_str());
  else
    LERROR("%s/%s plugin initialization failed (from %s)",
           realm, name, soname.c_str());

  return (status == 0);
}

PluginFallback::PluginFallback(OptionManager* mgr, const char* realm)
  :
  itsManager(mgr),
  itsRealm(realm)
{}

PluginFallback::~PluginFallback() throw() {}

void PluginFallback::try_fallback(const rutz::fstring& key) const
{
  loadPlugin(itsManager, itsRealm, key.c_str());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_PLUGIN_C_DEFINED
