/*!@file Neuro/EnvSegmenterConfigurator.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvSegmenterConfigurator.C $
// $Id: EnvSegmenterConfigurator.C 12782 2010-02-05 22:14:30Z irock $
//

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include "Neuro/EnvSegmenterConfigurator.H"

#include "Component/ModelOptionDef.H"
#include "Neuro/EnvSegmenterCannyContour.H"
#include "Neuro/EnvSegmenterColorRegion.H"
#include "Neuro/EnvObjDetection.H"
#include "Neuro/NeuroOpts.H"

static const ModelOptionDef OPT_EnvSegmenterType =
  { MODOPT_ARG_STRING, "EnvSegmenterType", &MOC_ITC, OPTEXP_CORE,
    "Segmentation algorithm type",
    "ese-type", '\0', "<CannyContour|ColorRegion|ObjDetection>", "ColorRegion" };

// ######################################################################
EnvSegmenterConfigurator::EnvSegmenterConfigurator(OptionManager& mgr,
                                                   const std::string& descrName,
                                                   const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName),
  itsSegType(&OPT_EnvSegmenterType, this),
  itsSeg(new EnvSegmenterColorRegion(mgr))
{
  this->addSubComponent(itsSeg);
}

// ######################################################################
EnvSegmenterConfigurator::~EnvSegmenterConfigurator()
{}

// ######################################################################
nub::ref<EnvSegmenter> EnvSegmenterConfigurator::getSeg() const
{
  return itsSeg;
}

// ######################################################################
void EnvSegmenterConfigurator::paramChanged(ModelParamBase* const param,
                                            const bool valueChanged,
                                            ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsSegType)
    {
      // if we had one, let's unregister it (when we later reset() the
      // nub::ref, the current SaccadeController will unexport its
      // command-line options):
      removeSubComponent(*itsSeg);

      // instantiate a segmenter of the appropriate type:
      if (itsSegType.getVal().compare("ColorRegion") == 0)
        itsSeg.reset(new EnvSegmenterColorRegion(getManager()));
      else if (itsSegType.getVal().compare("CannyContour") == 0)
        itsSeg.reset(new EnvSegmenterCannyContour(getManager()));
      else if (itsSegType.getVal().compare("ObjDetection") == 0)
        itsSeg.reset(new EnvObjDetection(getManager()));
      else
        LFATAL("Unknown EnvSegmenter type %s",
               itsSegType.getVal().c_str());

      addSubComponent(itsSeg);

      itsSeg->exportOptions(MC_RECURSE);

      LINFO("Selected EnvSegmenter of type %s", itsSegType.getVal().c_str());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
