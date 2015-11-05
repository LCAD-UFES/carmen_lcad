/*!@file Component/JobServerConfigurator.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/JobServerConfigurator.C $
// $Id: JobServerConfigurator.C 7470 2006-11-22 01:08:19Z rjpeters $
//

#ifndef COMPONENT_JOBSERVERCONFIGURATOR_C_DEFINED
#define COMPONENT_JOBSERVERCONFIGURATOR_C_DEFINED

#include "Component/JobServerConfigurator.H"

#include "Component/ModelOptionDef.H"
#include "Util/MainJobServer.H"
#include "Util/SyncJobServer.H"
#include "Util/WorkThreadServer.H"

// Used by: JobServerConfigurator
static const ModelOptionDef OPT_JobServerNumThreads =
  { MODOPT_ARG(uint), "JobServerNumThreads", &MOC_GENERAL, OPTEXP_CORE,
    "How many worker threads to use for parallel computations; "
    "specify 0 to indicate that all computations should happen "
    "synchronously in a single main thread",
    "num-threads", 'j', "<uint>", "0" };

// ######################################################################
JobServerConfigurator::JobServerConfigurator(OptionManager& mgr,
                                             const std::string& descrName,
                                             const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName),
  itsNumThreads(&OPT_JobServerNumThreads, this)
{}

// ######################################################################
JobServerConfigurator::~JobServerConfigurator()
{}

// ######################################################################
void JobServerConfigurator::start1()
{
  if (itsNumThreads.getVal() == 0)
    setMainJobServer(rutz::make_shared<JobServer>(new SyncJobServer));
  else
    setMainJobServer(rutz::make_shared<JobServer>
                     (new WorkThreadServer("MainJobServer",
                                           itsNumThreads.getVal())));
}

// ######################################################################
void JobServerConfigurator::stop2()
{
  // destroy the existing JobServer so it can print final stats, etc.
  setMainJobServer(rutz::make_shared<JobServer>(new SyncJobServer));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_JOBSERVERCONFIGURATOR_C_DEFINED
