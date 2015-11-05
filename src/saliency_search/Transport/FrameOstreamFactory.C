/*!@file Transport/FrameOstreamFactory.C Factories for building FrameOstream objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameOstreamFactory.C $
// $Id: FrameOstreamFactory.C 8274 2007-04-19 17:44:48Z rjpeters $
//

#ifndef TRANSPORT_FRAMEOSTREAMFACTORY_C_DEFINED
#define TRANSPORT_FRAMEOSTREAMFACTORY_C_DEFINED

#include "Transport/FrameOstreamFactory.H"

#include "Transport/ParseArg.H"
#include "Util/Assert.H"
#include "rutz/error_context.h"
#include "rutz/sfmt.h"

namespace
{
  ComponentFactory<FrameOstream>* theTypeFactory = 0;
  ComponentFactory<FrameOstream>* theExtFactory = 0;

  pthread_once_t type_factory_once = PTHREAD_ONCE_INIT;
  void init_type_factory()
  {
    ASSERT(theTypeFactory == 0);
    theTypeFactory =
      new ComponentFactory<FrameOstream>("FrameOstream type",
                                         /* case-insensitive = */ true);
  }

  pthread_once_t ext_factory_once = PTHREAD_ONCE_INIT;
  void init_ext_factory()
  {
    ASSERT(theExtFactory == 0);
    theExtFactory =
      new ComponentFactory<FrameOstream>("FrameOstream file extension",
                                         /* case-insensitive = */ true);
  }
}

// ######################################################################
ComponentFactory<FrameOstream>& getFrameOstreamTypeFactory()
{
  pthread_once(&type_factory_once, &init_type_factory);
  ASSERT(theTypeFactory != 0);
  return *theTypeFactory;
}

// ######################################################################
ComponentFactory<FrameOstream>& getFrameOstreamExtFactory()
{
  pthread_once(&ext_factory_once, &init_ext_factory);
  ASSERT(theExtFactory != 0);
  return *theExtFactory;
}

// ######################################################################
nub::ref<FrameOstream> makeFrameOstream(const std::string& arg,
                                        OptionManager& mgr)
{
  GVX_ERR_CONTEXT(rutz::sfmt("building a FrameOstream from '%s'",
                             arg.c_str()));

  ComponentFactory<FrameOstream>& outTypeFactory =
    getFrameOstreamTypeFactory();

  ComponentFactory<FrameOstream>& outExtFactory =
    getFrameOstreamExtFactory();

  std::string extrainfo;
  nub::ref<FrameOstream> s =
    parseStreamArg(arg, extrainfo, &outTypeFactory, &outExtFactory);

  s->setConfigInfo(extrainfo);
  return s;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_FRAMEOSTREAMFACTORY_C_DEFINED
