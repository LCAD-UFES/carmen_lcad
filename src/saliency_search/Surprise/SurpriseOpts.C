/*!@file Surprise/SurpriseOpts.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseOpts.C $
// $Id: SurpriseOpts.C 11562 2009-08-08 00:35:40Z dberg $
//

#ifndef SURPRISE_SURPRISEOPTS_C_DEFINED
#define SURPRISE_SURPRISEOPTS_C_DEFINED

#include "Surprise/SurpriseOpts.H"

#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"
#include "Image/Point2D.H"

const ModelOptionCateg MOC_SURPRISE = {
  MOC_SORTPRI_3, "Surprise-Related Options" };

// Format here is:
//
// { MODOPT_TYPE, "name", &MOC_CATEG, OPTEXP_CORE,
//   "description of what option does",
//   "long option name", 'short option name', "valid values", "default value" }
//

// alternatively, for MODOPT_ALIAS option types, format is:
//
// { MODOPT_ALIAS, "", &MOC_ALIAS, OPTEXP_CORE,
//   "description of what alias does",
//   "long option name", 'short option name', "", "list of options" }
//

// NOTE: do not change the default value of any existing option unless
// you really know what you are doing!  Many components will determine
// their default behavior from that default value, so you may break
// lots of executables if you change it.

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseSQlen =
  { MODOPT_ARG(unsigned int), "SingleChannelSurpriseSQlen", &MOC_SURPRISE, OPTEXP_CORE,
    "Number of feed-through surprise models",
    "surprise-qlen", '\0', "<uint>", "5" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseUpdFac =
  { MODOPT_ARG(double), "SingleChannelSurpriseUpdFac", &MOC_SURPRISE, OPTEXP_CORE,
    "Local surprise update factor",
    "surprise-updfac", '\0', "<double>", "0.7" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseNeighUpdFac =
  { MODOPT_ARG(double), "SingleChannelSurpriseNeighUpdFac", &MOC_SURPRISE, OPTEXP_CORE,
    "Neighborhood surprise update factor, or 0.0 to use the same value as "
    "the local surprise update factor",
    "surprise-neighupdfac", '\0', "<double>", "0.7" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseProbe =
  { MODOPT_ARG(Point2D<int>), "SingleChannelSurpriseProbe", &MOC_SURPRISE, OPTEXP_CORE,
    "Location of a virtual electrode probe; various model and surprise "
    "readings from this location will be printed out if not (-1, -1)",
    "surprise-probe", '\0', "<x>,<y>", "-1,-1" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseSLfac =
  { MODOPT_ARG(double), "SingleChannelSurpriseSLfac", &MOC_SURPRISE, OPTEXP_CORE,
    "Factor for contribution of local temporal surprise to total surprise",
    "surprise-slfac", '\0', "<double>", "1.0" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseSSfac =
  { MODOPT_ARG(double), "SingleChannelSurpriseSSfac", &MOC_SURPRISE, OPTEXP_CORE,
    "Factor for contribution of spatial surprise to total surprise",
    "surprise-ssfac", '\0', "<double>", "0.1" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseNeighSigma =
  { MODOPT_ARG(float), "SingleChannelSurpriseNeighSigma", &MOC_SURPRISE, OPTEXP_CORE,
    "Factor for neighborhoods surprise size",
    "surprise-neighsig", '\0', "<float>", "0.5" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseLocSigma =
  { MODOPT_ARG(float), "SingleChannelSurpriseLocSigma", &MOC_SURPRISE, OPTEXP_CORE,
    "Factor for the local surprise size",
    "surprise-locsig", '\0', "<float>", "3.0" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseLogged =
  { MODOPT_FLAG, "SingleChannelSurpriseLogged", &MOC_SURPRISE, OPTEXP_CORE,
    "Save each surprise map and log the parameters used in its creation",
    "surprise-logged", '\0', "", "false"};

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseKLBias =
  { MODOPT_ARG(std::string), "VisualCortexSurpriseKLBias", &MOC_SURPRISE, OPTEXP_CORE,
    "Bias to apply to KL is joint surprise model",
    "surprise-kl-bias", '\0', "<None|Static>", "None" };

// Used by: SingleChannelSurprise
const ModelOptionDef OPT_SingleChannelSurpriseTakeSTMax =
  { MODOPT_FLAG, "SingleChannelSurpriseTakeSTMax", &MOC_SURPRISE, OPTEXP_CORE,
    "Use a max operator to combine space-time rather than using the product",
    "surprise-take-st-max", '\0', "", "false"};

// Used by: VisualCortexSurprise
const ModelOptionDef OPT_VisualCortexSurpriseType =
  { MODOPT_ARG(std::string), "VisualCortexSurpriseType", &MOC_SURPRISE, OPTEXP_CORE,
    "Type of surprise models to use for the VisualCortex and Channels",
    "surprise-type", '\0', "<Gaussian|Poisson|Poisson1|PoissonConst|PoissonFloat|ChiSqaure|JointGG|Nathan|Outlier>", "Poisson" };



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SURPRISE_SURPRISEOPTS_C_DEFINED
