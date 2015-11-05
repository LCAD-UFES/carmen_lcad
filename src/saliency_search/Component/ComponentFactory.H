/*!@file Component/ComponentFactory.H Factory to create ModelComponent objects from a key string */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ComponentFactory.H $
// $Id: ComponentFactory.H 8274 2007-04-19 17:44:48Z rjpeters $
//

#ifndef COMPONENT_COMPONENTFACTORY_H_DEFINED
#define COMPONENT_COMPONENTFACTORY_H_DEFINED

#include "nub/ref.h"
#include "rutz/factory.h"

class OptionManager;

/// Factory to create ModelComponent-derivative objects from a key string.
/** The template class P should be some class that is derived from
    ModelComponent, and that is also the root of some sub-tree of the
    full inheritance graph.

    Here's a quick overview of how ComponentFactory can be used with
    plugins and dynamic loading. You would use this system when you
    have multiple concrete classes derived from a base class, and you
    want to be able to select a particular concrete type to use at run
    time, based on some user input (e.g. a string passed to a
    command-line option). First, you create a ComponentFactory that is
    going to build products derived from a different type. Then, you
    install a PluginFallback for that factory that tells it "if you
    don't already know about the type, then first try to load an
    appropriately-named plugin module, and then try again to see if
    that plugin has now informed you about the type".

    For an example of all this, see Media/FrameSeries.C, where we use
    the factories defined in Transport/FrameIstreamFactory.H. We
    populate the factories with some statically-known types, and then
    we install a PluginFallback. The PluginFallback essentially says,
    if we get --in=foobar on the command line, and our factory doesn't
    know anything about 'foobar' yet, then it should look for a plugin
    library named <plugins>/FrameIstream/foobar.so, and it should try
    to find an init function named frameistream_foobar_init() in that
    library. Presumably, that function frameistream_foobar_init() will
    then tell the factory how to make a 'foobar', and so when the
    PluginFallback returns, the factory can try again to make a
    'foobar'.

    To actually try out a plugin, try the following:
    \verbatim
    make bin/ezvision
    make plugins
    ./bin/ezvision --in=tests/inputs/mpegclip1.mpg -T --out=display --out=debug
    \endverbatim

    and within the log output you will see something like this:
    \verbatim
    Plugin::loadPlugin: opening /lab/rjpeters/projects/saliency/lib/invt/plugins/FrameOstream/debug.so
    Plugin::loadPlugin: running frameostream_debug_init() from /lab/rjpeters/projects/saliency/lib/invt/plugins/FrameOstream/debug.so
    debug::frameostream_debug_init: Hello from frameostream_debug_init()!
    \endverbatim

    and then you will start seeing output from the DebugOstream that
    is built by --out=debug.
*/
template <class P>
class ComponentFactory : public rutz::factory<nub::ref<P> >
{
public:
  typedef nub::ref<P> ProductType;

  ComponentFactory(const char* keydescr, bool nocase = false)
    :
    rutz::factory<nub::ref<P> >(keydescr, nocase)
  {}

  template <class T>
  void registerType(const char* name, OptionManager& mgr)
  {
    this->register_creator(new ComponentCreator<T>(mgr), name);
  }

  nub::ref<P> createObj(const char* type) const
  {
    return this->new_checked_object(rutz::fstring(type));
  }

private:
  /// Creator class that creates ModelComponent derivatives on demand.
  /** The only trick here is that we have to store a reference to an
      OptionManager, since (by convention) all ModelComponent classes
      take an OptionManager as the first arg to their constructor. */
  template <class T>
  class ComponentCreator : public rutz::creator_base<nub::ref<P> >
  {
  public:
    ComponentCreator(OptionManager& mgr)
      : itsManager(mgr) {}

    ComponentCreator(const ComponentCreator& that)
      : rutz::creator_base<nub::ref<P> >(that),
        itsManager(that.itsManager) {}

    virtual ~ComponentCreator() {}

    virtual rutz::creator_base<nub::ref<P> >* clone() const
    { return new ComponentCreator(*this); }

    virtual nub::ref<P> create()
    { return nub::ref<P>(new T(itsManager)); }

  private:
    OptionManager& itsManager;
  };
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_COMPONENTFACTORY_H_DEFINED
