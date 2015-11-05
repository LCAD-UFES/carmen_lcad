/*!@file Channels/SubmapAlgorithm.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SubmapAlgorithm.C $
// $Id: SubmapAlgorithm.C 6636 2006-05-23 18:44:38Z rjpeters $
//

#ifndef CHANNELS_SUBMAPALGORITHM_C_DEFINED
#define CHANNELS_SUBMAPALGORITHM_C_DEFINED

#include "Channels/SubmapAlgorithm.H"

#include "Util/Assert.H"
#include "rutz/error_context.h"
#include "rutz/sfmt.h"

namespace
{
  ComponentFactory<SubmapAlgorithm>* theFactory = 0;

  pthread_once_t factory_once = PTHREAD_ONCE_INIT;
  void init_factory()
  {
    ASSERT(theFactory == 0);
    theFactory =
      new ComponentFactory<SubmapAlgorithm>("SingleChannel SubmapAlgorithm type");
  }
}

// ######################################################################
SubmapAlgorithm::SubmapAlgorithm(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName)
{}

// ######################################################################
SubmapAlgorithm::~SubmapAlgorithm()
{}

// ######################################################################
ComponentFactory<SubmapAlgorithm>& SubmapAlgorithm::getFactory()
{
  pthread_once(&factory_once, &init_factory);
  ASSERT(theFactory != 0);
  return *theFactory;
}

// ######################################################################
nub::ref<SubmapAlgorithm> SubmapAlgorithm::make(const std::string& arg)
{
  GVX_ERR_CONTEXT(rutz::sfmt("building a SubmapAlgorithm from '%s'",
                             arg.c_str()));

  return SubmapAlgorithm::getFactory().createObj(arg.c_str());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SUBMAPALGORITHM_C_DEFINED
