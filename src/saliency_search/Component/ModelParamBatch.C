/*!@file Component/ModelParamBatch.C Batch setting and restoring of model parameter values */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelParamBatch.C $
// $Id: ModelParamBatch.C 8762 2007-09-06 22:58:08Z rjpeters $
//

#ifndef COMPONENT_MODELPARAMBATCH_C_DEFINED
#define COMPONENT_MODELPARAMBATCH_C_DEFINED

#include "Component/ModelParamBatch.H"

#include "Component/ModelComponent.H"

using rutz::shared_ptr;
using std::pair;
using std::string;
using std::vector;

// ######################################################################
ModelParamBatch::ModelParamBatch()
{}

// ######################################################################
ModelParamBatch::~ModelParamBatch()
{}

// ######################################################################
void ModelParamBatch::installValues(ModelComponent* comp)
{
  itsRestoreStack.push_back(vector<pair<string, string> >());
  for (size_t i = 0; i < itsParamValues.size(); ++i)
    {
      itsRestoreStack.back().push_back
        (std::make_pair(itsParamValues[i].first,
                        comp->getModelParamString(itsParamValues[i].first,
                                                  MC_RECURSE)));
      comp->setModelParamValAux(itsParamValues[i].first,
                                *itsParamValues[i].second,
                                MC_RECURSE);
    }
}

// ######################################################################
void ModelParamBatch::restoreValues(ModelComponent* comp)
{
  if (itsRestoreStack.size() == 0)
    LFATAL("no saved values to restore from");

  for (size_t i = 0; i < itsRestoreStack.back().size(); ++i)
    {
      comp->setModelParamString(itsRestoreStack.back()[i].first,
                                itsRestoreStack.back()[i].second,
                                MC_RECURSE);
    }

  itsRestoreStack.pop_back();
}

// ######################################################################
void ModelParamBatch::addParamValueAux(const string& paramname,
                                       const shared_ptr<RefHolder>& valref)
{
  itsParamValues.push_back(std::make_pair(paramname, valref));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_MODELPARAMBATCH_C_DEFINED
