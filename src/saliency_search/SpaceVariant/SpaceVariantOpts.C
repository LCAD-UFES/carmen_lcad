/*!@file SpaceVariant/SpaceVariantOpts.C */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SpaceVariantOpts.C $
// $Id: SpaceVariantOpts.C 14513 2011-02-16 05:02:38Z dberg $
//

#ifndef NEURO_NEUROOPTS_C_DEFINED
#define NEURO_NEUROOPTS_C_DEFINED

#include "SpaceVariant/SpaceVariantOpts.H"
#include "SpaceVariant/SVChanLevels.H"
#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"

// #################### SpaceVariant options:
const ModelOptionCateg MOC_SPACEVARIANT = {
  MOC_SORTPRI_3,   "Space Variant Processing - Related Options" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantScale =
  { MODOPT_ARG_STRING, "SpaceVariantScale", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "Decide the scale of the transform. 'FULL' to scale along the longest "
    "input dimension (will leave some of the transform unfilled. 'CROP' to "
    "scale along the shortest input dimension (will not transform all of "
    "the input image). 'OPTIM' to scale each orientation seperately (full "
    "coverage, but not isotropic anymore). Or 'NONE' to supply a user "
    "defined scaling factor", "retinasv-scale", '\0', "", "FULL" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantFoveaSize =
  { MODOPT_ARG(float), "SpaceVariantFoveaSize", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "factor to use in scaling fovea", "retinasv-fovea-size",
    '\0', "<float>", "0.01" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantSfac =
  { MODOPT_ARG(float), "SpaceVariantSfac", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "If the retina's scale type is NONE we will use this scaling factor",
    "retinasv-sfac", '\0', "<float>", "24.75"};

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantBeta =
  { MODOPT_ARG(float), "SpaceVariantBeta", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "Choose the beta parameter of the transform equation "
    "'u=s*ln(r/alpha) + beta' where u is the space variant point and r "
    "is the radius. s is the scaling factor and alpha is the fovea size. "
    "This parameter shifts the fovea left (<1) or right (>1) of the "
    "midline.", "retinasv-beta",'\0', "<float>", "1.0" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantGain =
  { MODOPT_ARG(float), "SpaceVariantGain", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "The receptive field of each pixel in the space variant image is modeled "
    "as a guassian parameterized by the standard deviation in pixels. For a given eccentricty "
    "r in degrees of visual space, the standard deviation 's', can be modeled as "
    "'s=g*r^e+b' where 'g','e', 'b' are the gain, exponent and offset which describe the shape of "
    "of the ralationship. The default parameters are estimated from the data of "
    "Croner & Kaplan (1995) for parvocellular ganglion cells.", "retinasv-gain", '\0', "<float>", "0.0002" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantExponent =
  { MODOPT_ARG(float), "SpaceVariantExponent", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "The receptive field of each pixel in the space variant image is modeled "
    "as a guassian parameterized by the standard deviation in pixels. For a given eccentricty "
    "r in degrees of visual space, the standard deviation 's', can be modeled as "
    "'s=g*r^e+b' where 'g','e', 'b' are the gain, exponent and offset which describe the shape of "
    "of the ralationship. The default parameters are estimated from the data of "
    "Croner & Kaplan (1995) for parvocellular ganglion cells.", "retinasv-exponent", '\0', "<float>", "1.7689" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantOffset =
  { MODOPT_ARG(float), "SpaceVariantOffset", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "The receptive field of each pixel in the space variant image is modeled "
    "as a guassian parameterized by the standard deviation in pixels. For a given eccentricty "
    "r in degrees of visual space, the standard deviation 's', can be modeled as "
    "'s=g*r^e+b' where 'g','e', 'b' are the gain, exponent and offset which describe the shape of "
    "of the ralationship. The default parameters are estimated from the data of "
    "Croner & Kaplan (1995) for parvocellular ganglion cells.", "retinasv-offset", '\0', "<float>", "0.0252" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantFovCut =
  { MODOPT_ARG(float), "SpaceVariantFovCut", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "Normally, the fovea is considered the area of expansion, where the "
    "first derivitive of the transform function is > 1. However, if "
    "the paramters are adjusted so no oversampling occurs then this "
    "will never be the case. Use this parameter to adjust the value "
    "(between 0 and 1) of the first derivitive that is considered the "
    "fovea. ", "retinasv-fovea-cutoff", '\0', "<float>", "2.0" };

// Used by: SpaceVariantModule
const ModelOptionDef OPT_SpaceVariantDims =
  { MODOPT_ARG(Dims), "SpaceVariantDims", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "The dimensions of the space variant transform in rings x wedges. "
    "The Number of wedges (rays) in the log-polar map will have "
    "horizontal dimensions 1/2 this value as the wedges are split between "
    "hemifields. The number of rings in the log-polar map will have "
    "horizontal dimensions twice this value (for each hemifield).",
    "retinasv-dims", '\0', "<Dims>", "160x480" };

extern const ModelOptionDef OPT_SpaceVariantDogCenter = 
  { MODOPT_FLAG, "SpaceVariantDogCenter", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "Sets center-surround, or surround-center when performing a difference "
    "of gaussians space variant transform.", 
    "use-channelasv-oncenter", '\0', "", "true" };

extern const ModelOptionDef OPT_SpaceVariantDogSize = 
  { MODOPT_ARG(float), "SpaceVariantDogSize", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "Sets the size of the surround when performing a difference "
    "of gaussians space variant transform. The surround will be the "
    "center standard deviation multiplied by this factor. A factor of 6 approximates "
    "retinal ganglion cell responses estimated from Croner & Kaplan (1993), and "
    "a factor of 1.6 the approximates laplacian transform.", 
    "channelasv-surround-factor", '\0', "<float>", "6.9348" };

extern const ModelOptionDef OPT_SpaceVariantEdgeOrient = 
  { MODOPT_ARG(float), "SpaceVariantEdgeOrient", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "The edge orientation for the space variant retina", 
    "channelsv-edge-orientation", '\0', "<uint>", "0.0" };

extern const ModelOptionDef OPT_SpaceVariantEdgeLength = 
  { MODOPT_ARG(uint), "SpaceVariantEdgeLenth", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "The edge length in pixels for the space variant retina", 
    "channelsv-edge-length", '\0', "<uint>", "3" };

extern const ModelOptionDef OPT_SpaceVariantEdgeDensity = 
  { MODOPT_ARG(uint), "SpaceVariantEdgeDensity", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "the number of pixels used to create the edge", 
    "channelsv-edge-density", '\0', "<float>", "3" };

extern const ModelOptionDef OPT_SpaceVariantChanScales = 
  { MODOPT_ARG(SVChanLevels), "SpaceVariantChanScales", &MOC_SPACEVARIANT, OPTEXP_CORE,
    "the scales to use when creating a space variant pyramid", 
    "channelsv-scales", '\0', "<float>,<float>,...", "0.5,1.0,2.0,4.0,8.0" };

extern const ModelOptionDef OPT_SpaceVariantMaxVisualAngle = 
{ MODOPT_ARG(float), "SpaceVariantMaxVisualAngle", &MOC_SPACEVARIANT, OPTEXP_CORE,
  "The maximum visual angle to simulate. ", 
  "max-visual-angle", '\0', "<uint>", "180" };

extern const ModelOptionDef OPT_SpaceVariantCellType = 
{ MODOPT_ARG(std::string), "SpaceVariantCellType", &MOC_SPACEVARIANT, OPTEXP_CORE,
  "The type of ganglion cell to use to determine receptive field size as a function of eccentricity. ", 
  "ganglioncell-type", '\0', "<MAGNO, PARVO>", "MAGNO" };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_NEUROOPTS_C_DEFINED
