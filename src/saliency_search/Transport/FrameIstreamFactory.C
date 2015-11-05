/*!@file Transport/FrameIstreamFactory.C Factories for building FrameIstream objects  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameIstreamFactory.C $
// $Id: FrameIstreamFactory.C 14128 2010-10-12 23:39:22Z rand $
//

#ifndef TRANSPORT_FRAMEISTREAMFACTORY_C_DEFINED
#define TRANSPORT_FRAMEISTREAMFACTORY_C_DEFINED

#include "Transport/FrameIstreamFactory.H"

#include "Transport/ParseArg.H"
#include "Util/Assert.H"
#include "rutz/error_context.h"
#include "rutz/sfmt.h"

namespace
{
  ComponentFactory<FrameIstream>* theTypeFactory = 0;
  ComponentFactory<FrameIstream>* theExtFactory = 0;

  pthread_once_t type_factory_once = PTHREAD_ONCE_INIT;
  void init_type_factory()
  {
    ASSERT(theTypeFactory == 0);
    theTypeFactory =
      new ComponentFactory<FrameIstream>("FrameIstream type",
                                         /* case-insensitive = */ true);
  }

  pthread_once_t ext_factory_once = PTHREAD_ONCE_INIT;
  void init_ext_factory()
  {
    ASSERT(theExtFactory == 0);
    theExtFactory =
      new ComponentFactory<FrameIstream>("FrameIstream file extension",
                                         /* case-insensitive = */ true);
  }
}

// ######################################################################
ComponentFactory<FrameIstream>& getFrameIstreamTypeFactory()
{
  pthread_once(&type_factory_once, &init_type_factory);
  ASSERT(theTypeFactory != 0);
  return *theTypeFactory;
}

// ######################################################################
ComponentFactory<FrameIstream>& getFrameIstreamExtFactory()
{
  pthread_once(&ext_factory_once, &init_ext_factory);
  ASSERT(theExtFactory != 0);
  return *theExtFactory;
}

// ######################################################################
nub::ref<FrameIstream> makeFrameIstream(const std::string& arg)
{
  GVX_ERR_CONTEXT(rutz::sfmt("building a FrameIstream from '%s'",
                             arg.c_str()));

  ComponentFactory<FrameIstream>& inTypeFactory =
    getFrameIstreamTypeFactory();

  ComponentFactory<FrameIstream>& inExtFactory =
    getFrameIstreamExtFactory();

  std::string extrainfo;
  nub::ref<FrameIstream> s =
    parseStreamArg(arg, extrainfo, &inTypeFactory, &inExtFactory);

  s->setConfigInfo(extrainfo);
  return s;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_FRAMEISTREAMFACTORY_C_DEFINED
