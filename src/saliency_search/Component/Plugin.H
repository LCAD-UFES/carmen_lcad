/*!@file Component/Plugin.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/Plugin.H $
// $Id: Plugin.H 5966 2005-11-18 20:55:46Z rjpeters $
//

#ifndef COMPONENT_PLUGIN_H_DEFINED
#define COMPONENT_PLUGIN_H_DEFINED

#include "rutz/factory.h"

class OptionManager;

/// Plugin initialization function type.
/** See ComponentFactory for an overview of the factory+plugin system.

    See frameostream_debug_init() for an example plugin init function.
*/
extern "C" typedef int (PluginInitFunc)(OptionManager*);

/// Load a plugin for a given realm+name.
/** Returns true on success, false on failure.

    See ComponentFactory for an overview of the factory+plugin system.

    See frameostream_debug_init() for an example plugin init function.
*/
bool loadPlugin(OptionManager* mgr,
                const char* realm, const char* name);

/// Class that can be used as a fallback for rutz::factory::set_fallback()
/** See ComponentFactory for an overview of the factory+plugin system.

    See frameostream_debug_init() for an example plugin init function.
*/
class PluginFallback : public rutz::factory_fallback
{
public:
  /// Construct with a given realm.
  PluginFallback(OptionManager* mgr, const char* realm);

  /// Virtual destructor.
  virtual ~PluginFallback() throw();

  /// Fallback function -- try to load a plugin from <plugindir>/realm/key.so
  virtual void try_fallback(const rutz::fstring& key) const;

private:
  OptionManager* itsManager;
  const char* itsRealm;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_PLUGIN_H_DEFINED
