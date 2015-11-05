/*!@file ModelNeuron/SCFitOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SCFitOpts.C $

#include "ModelNeuron/SCFitOpts.H"
#include "ModelNeuron/FreeViewingData.H"
#include "Component/ModelOptionDef.H"
#include "Util/SimTime.H"
#include "Image/Point2D.H"
#include "Image/Dims.H"

// #################### SC Fitting options:
const ModelOptionCateg MOC_FitSC = { MOC_SORTPRI_3, "SC fitting options" };

const ModelOptionDef OPT_StimLocation = { MODOPT_ARG(Point2D<float>), "StimLocation", &MOC_FitSC, OPTEXP_CORE,
                                          "probe location", "probe-location", '\0', "<x,y>", "-1,-1"};

const ModelOptionDef OPT_RFSize = { MODOPT_ARG(float), "RFSize", &MOC_FitSC, OPTEXP_CORE,
                                    "probe size", "probe-size", '\0', "<x,y>", "0"};

const ModelOptionDef OPT_SimDim = { MODOPT_ARG(Dims), "SimDims", &MOC_FitSC, OPTEXP_CORE,
                                    "dimensions of network", "model-dims", '\0', "<wxh>", "60x33" };

const ModelOptionDef OPT_ModelErrorFunc = { MODOPT_ARG(std::string), "ModelErrorFunc", &MOC_FitSC, OPTEXP_CORE,
                                            "type of model error function", "model-error-type", '\0', "<Slice, Monkey>", "Slice"};

const ModelOptionDef OPT_EyeFileNames = { MODOPT_ARG(std::string), "EyeFileNames", &MOC_FitSC, OPTEXP_CORE,
                                          "names of all the eye position files", "eye-fnames", '\0', "<file1, file2,...FileN>", ""};

const ModelOptionDef OPT_SCModelType = {MODOPT_ARG(std::string), "SCModelType", &MOC_FitSC, OPTEXP_CORE, "SC model type", "SC-model-type",
                                          '\0', "<SCsSigmoid, SCsRectify, LowPassSCcSigmoid, LowPassSCsRectify, None>", "None"};

const ModelOptionDef OPT_FeatureMapDims = { MODOPT_ARG(Dims), "MapDims", &MOC_FitSC, OPTEXP_CORE, "Dimension to which feature maps have been rescaled", "feature-map-dims", '\0', "<wxh>", "120x67" };

const ModelOptionDef OPT_RetinalDims = { MODOPT_ARG(Dims), "RetinalDims", &MOC_FitSC, OPTEXP_CORE,
                                        "dimensions of retinal image", "retinal-dims", '\0', "<wxh>", "2400x2100" };

const ModelOptionDef OPT_ModelTime = { MODOPT_ARG(SimTime), "ModelTime", & MOC_FitSC, OPTEXP_CORE, "time step of any models", "model-timestep", '\0', "<SimTime>", "1ms" };

const ModelOptionDef OPT_DemoPlotDims = { MODOPT_ARG(Dims), "DemoPlotDims", &MOC_FitSC, OPTEXP_CORE,
                                          "size of demo plot", "demo-dims", '\0', "<wxh>", "1024x768" };

const ModelOptionDef OPT_NormFileName = { MODOPT_ARG(std::string), "NormFileName", &MOC_FitSC, OPTEXP_CORE,
                                          "name of a file which contains information about how to normalize each channel", "channel-norm-file", '\0', "<file>", ""};

const ModelOptionDef OPT_FreeDataTime = { MODOPT_ARG(SimTime), "FreeDataTime", & MOC_FitSC, OPTEXP_CORE, "the sampling rate of the freeviewing data", "freeviewing-timestep", '\0', "<SimTime>", "1000Hz" };

const ModelOptionDef OPT_ProbeType = { MODOPT_ARG(ProbeType), "ProbeType", & MOC_FitSC, OPTEXP_CORE, "Type of data to be collected at probe location", "probe-type", '\0', "<Probe, Avg Max>", "Probe" };

const ModelOptionDef OPT_UseChannelMaxNorm = { MODOPT_FLAG, "UseChannelMaxNorm", &MOC_FitSC, OPTEXP_CORE, "should we use channel max norm?", "use-channel-maxnorm", '\0', "<>", "true"};

const ModelOptionDef OPT_UseScMaxNorm = { MODOPT_FLAG, "UseScMaxNorm", &MOC_FitSC, OPTEXP_CORE, "should we use sc max norm?", "use-sc-maxnorm", '\0', "<>", "true"};

const ModelOptionDef OPT_UseSurpChanKL = { MODOPT_FLAG, "UseSurpChanKL", &MOC_FitSC, OPTEXP_CORE, "If set to true then channel level surprise will output KL, otherwise just the mean.", "use-channel-surprise-kl", '\0', "<>", "true"};

const ModelOptionDef OPT_UseChannelSurprise = { MODOPT_FLAG, "UseChannelSurprise", &MOC_FitSC, OPTEXP_CORE, "should we use channel surprise?", "use-channel-surprise", '\0', "<>", "true"};

const ModelOptionDef OPT_UseScSurprise = { MODOPT_FLAG, "UseScSurprise", &MOC_FitSC, OPTEXP_CORE, "should we use sc surprise?", "use-sc-surprise", '\0', "<>", "true"};

const ModelOptionDef OPT_ChannelGain = { MODOPT_ARG(float), "ChannelGain", &MOC_FitSC, OPTEXP_CORE, "apply a gain to the channels", "channel-gain", '\0', "<float>", "1.0"};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
