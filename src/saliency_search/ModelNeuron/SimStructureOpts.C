/*!@file ModelNeuron/SimStructureOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SimStructureOpts.C $
// $Id: SimStructureOpts.C 15310 2012-06-01 02:29:24Z itti $
//

#include "ModelNeuron/SimStructureOpts.H"
#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"
#include "Image/Point2D.H"
#include "Image/Range.H"
#include "Util/SimTime.H"
#include "ModelNeuron/Location.H"

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

// #################### Super Colliculus Map options:
const ModelOptionCateg MOC_SC = {
  MOC_SORTPRI_2, "Superior Colliculus related options" };

const ModelOptionDef OPT_SCSimTimeStep = 
{ MODOPT_ARG(SimTime), "SCSimTimeStep", &MOC_SC, OPTEXP_CORE,
  "The time step used for SC simulations", 
  "asc-timestep",'\0',"<SimTime>", ".001s"};

const ModelOptionDef OPT_SCDims =   
{ MODOPT_ARG(Dims), "SCDims", &MOC_SC, OPTEXP_CORE,
  "Dimensions of the SC model, 0x0 to set to the "
  "dimensions of the input.", "asc-dims", '\0', "<w>x<h>", "0x0"};

const ModelOptionDef OPT_SCBUInpGain = 
{ MODOPT_ARG(float), "SCBUInpGain", &MOC_SC, OPTEXP_CORE,
  "Bu input gain", "asc-bugain", '\0', "", "1.0" };

const ModelOptionDef OPT_SCTDInpGain = 
{ MODOPT_ARG(float), "SCTDInpGain", &MOC_SC, OPTEXP_CORE,
  "Td input gain", "asc-tdgain", '\0', "", "1.0" };

const ModelOptionDef OPT_SCInpGain = 
{ MODOPT_ARG(float), "SCInpGain", &MOC_SC, OPTEXP_CORE,
  "input gain", "asc-gain", '\0', "", "1.0" };

const ModelOptionDef OPT_SCSaveResults = 
{ MODOPT_FLAG, "SCSaveResults", &MOC_SC, OPTEXP_CORE, 
  "save our module results","asc-save-results",'\0', "<bool>", "false" };

const ModelOptionCateg MOC_SCDisplay = { MOC_SORTPRI_2, "Superior Colliculus PLotting Options" };

// Used by SupColliculus
const ModelOptionDef OPT_SCSCType =
{ MODOPT_ARG_STRING, "SCSCType", &MOC_SC, OPTEXP_CORE,
  "Type of superior colliculus model to use.",
  "asc-sc-type", '\0', "<NFGauss, NFNbumpCS, NF1bumpCS, NFNbumpDoG, NF0bumpDoG, LowpassNeuronFeedbackLayer, LowpassSCsSigmoid, LowpassSCsRectify>", "NFGauss"};

// Used by NeualField
const ModelOptionDef OPT_SCNFType =
{ MODOPT_ARG_STRING, "SCNFType", &MOC_SC, OPTEXP_CORE,
  "Type of neural field to use.",
  "asc-nf-type", '\0', "<>", "" };

const ModelOptionDef OPT_SCProbe = 
{ MODOPT_ARG(nsu::Location), "SCProbe", &MOC_SCDisplay, OPTEXP_CORE,
  "N-D location of probe. -1 can be used along one dimension to display "
  "multiple modules",
  "asc-probe", '\0', "<d1>,<d2>,<d3>,...", "-1,-1,-1" };

const ModelOptionDef OPT_SCPlotLength = 
{ MODOPT_ARG(uint), "SCPlotLength", &MOC_SCDisplay, OPTEXP_CORE,
  "length in ms to display plot. Default, 0, grows foverever. ",
  "asc-plot-length", '\0', "<uint>", "0" };

const ModelOptionDef OPT_SC2DPlotDepth = 
{ MODOPT_ARG(uint), "SC2DPlotDepth", &MOC_SCDisplay, OPTEXP_CORE,
  "depth to recurse into strctures",
  "asc-2dplot-depth", '\0', "<uint>", "0" };

const ModelOptionDef OPT_SCProbeDepth = 
{ MODOPT_ARG(uint), "SCProbeDepth", &MOC_SCDisplay, OPTEXP_CORE,
  "depth to recurse into modules at probe location",
  "asc-probe-depth", '\0', "<uint>", "1" };

const ModelOptionDef OPT_SCUseDisplayOutput = 
{ MODOPT_FLAG, "SCUseDisplayOutput", &MOC_SCDisplay, OPTEXP_CORE, 
  "Should we use the firing rate output (false), or the "
  "display output (true) which is usually one of the underlying "
  "variables in the difference equations. ", 
  "use-asc-displayoutput", '\0', "<bool>", "false" };

const ModelOptionDef OPT_SCDisplayRange =
{ MODOPT_ARG(Range<double>), "SCDisplayRange", &MOC_SCDisplay, OPTEXP_CORE, 
  "sets the display range. Use both less than 0 to normalize at every time step, both"
  "zero to set to min/max over time, and any other values to fix the range", "asc-displayrange",
  '\0', "<min>-<max>", "0.0-0.0" };

const ModelOptionDef OPT_SCDisplayDecoder = 
{ MODOPT_ARG_STRING, "SCDisplayDecoder", &MOC_SCDisplay, OPTEXP_CORE,
  "The display decoder to use in plotting if desired, None otherwise",
  "asc-decoder-type", '\0', "<None, AlphaDecoder, RectDecoder, ExpDecoder, HistDecoder>", "None" };

const ModelOptionDef OPT_SC2DPlotSize = 
{ MODOPT_ARG(Dims), "SC2DPlotSize", &MOC_SCDisplay, OPTEXP_CORE,
  "Dimensions of the SC model save/plot figures",
  "asc-2dplot-dims", '\0', "<w>x<h>", "320x240"};

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
