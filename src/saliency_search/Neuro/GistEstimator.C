/*!@file Neuro/GistEstimator.C */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimator.C $
// $Id: GistEstimator.C 13065 2010-03-28 00:01:00Z itti $
//

#ifndef NEURO_GISTESTIMATOR_C_DEFINED
#define NEURO_GISTESTIMATOR_C_DEFINED

#include "Neuro/GistEstimator.H"
#include "Neuro/GistEstimatorConfigurator.H"
#include "Transport/FrameInfo.H"
#include "rutz/trace.h"

// ######################################################################
// ######################################################################
// ########## GistEstimator implementation
// ######################################################################
// ######################################################################
GistEstimator::GistEstimator(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName)
  :
  SimModule(mgr, descrName, tagName)
{}

// ######################################################################
GistEstimator::~GistEstimator()
{}


// ######################################################################
// ######################################################################
// ########## GistEstimatorAdapter implementation
// ######################################################################
// ######################################################################
GistEstimatorAdapter::GistEstimatorAdapter(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName)
  : GistEstimator(mgr, descrName, tagName),
    SIMCALLBACK_INIT(SimEventSaveOutput),
    itsSaveResults(&OPT_SaveGistFlag, this)
{
  GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
GistEstimatorAdapter::~GistEstimatorAdapter()
{
  GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void GistEstimatorAdapter::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

  const Image<float> gistVec = getGist() ;

  if(itsSaveResults.getVal())
    {
      ofs->writeFloat(gistVec, FLOAT_NORM_PRESERVE, "Gist", FrameInfo("gist vector", SRC_POS));
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_GISTESTIMATOR_C_DEFINED
