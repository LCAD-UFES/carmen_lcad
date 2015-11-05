/*!@file Neuro/NeuroOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/NeuroOpts.C $
// $Id: NeuroOpts.C 14984 2011-10-14 00:17:14Z dberg $
//

#ifndef NEURO_NEUROOPTS_C_DEFINED
#define NEURO_NEUROOPTS_C_DEFINED

#include "Neuro/NeuroOpts.H"

#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H" // for REQUEST_OPTIONALIAS_NEURO()
#include "Image/Dims.H"
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Psycho/PixPerDeg.H"
#include "Image/fancynorm.H"
#include "ModelNeuron/Location.H"
#include "Media/MediaOpts.H" // for MOC_SAC
#include "Neuro/IORtypes.H"
#include "Neuro/ShapeEstimatorModes.H"
#include "SIFT/VisualObjectMatchAlgo.H"
#include "Util/SimTime.H"

const ModelOptionCateg MOC_DISPLAY = {
  MOC_SORTPRI_2, "What to Display/Save Options" };

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

// #################### PrefrontalCortex options:
const ModelOptionCateg MOC_PFC = {
  MOC_SORTPRI_3,   "PrefrontalCortex-Related Options" };

// Used by: RetinaConfigurator
const ModelOptionDef OPT_PrefrontalCortexType =
  { MODOPT_ARG_STRING, "PFCType", &MOC_PFC, OPTEXP_CORE,
    "Type of PrefrontalCortex to use. 'Stub' for a simple pass-through "
    "of input (no biasing), 'OG' for the Optimal Gains pfc, 'SB' "
    "for SalBayes Bayesian tuning of receptors, 'GS' for Guided Search. ",
    "pfc-type", '\0', "<Stub|OG|SB|GS>", "Stub" };


// #################### Retina options:
const ModelOptionCateg MOC_RETINA = {
  MOC_SORTPRI_3,   "Retina-Related Options" };

// Used by: RetinaConfigurator
const ModelOptionDef OPT_RetinaType =
  { MODOPT_ARG_STRING, "RetinaType", &MOC_RETINA, OPTEXP_CORE,
    "Type of Retina to use. 'Stub' for a simple pass-through of input "
    "frames, 'Std' for the standard retina that can foveate, shift inputs "
    "to eye position, embed inputs within a larger framing image, foveate "
    "inputs, etc. Use CT for a cortical or collicular transfrom (log-polar) of "
    "the input image", "retina-type", '\0', "<Stub|Std|CT>", "Std" };

// Used by: RetinaStd
const ModelOptionDef OPT_InputFramingImageName =
  { MODOPT_ARG_STRING, "InputFramingImageName", &MOC_RETINA, OPTEXP_CORE,
    "Filename of an image to used as a background into which input "
    "frames will be embedded before processing. This image must be larger "
    "than the input frames to be processed.",
    "input-framing", '\0', "<imagefile>", "" };

// Used by: RetinaStd
const ModelOptionDef OPT_InputFramingImagePos =
  { MODOPT_ARG(Point2D<int>), "InputFramingImagePos", &MOC_RETINA, OPTEXP_CORE,
    "Position of the top-left corner of the input frames once embedded "
    "into the larger background image specified by --input-framing, if any.",
    "input-framing-pos", '\0', "<i,j>", "0,0" };

// Used by: Retina
const ModelOptionDef OPT_FoveateInputDepth =
  { MODOPT_ARG(uint), "FoveateInputDepth", &MOC_RETINA, OPTEXP_CORE,
    "Depth of pyramid to use to foveate input frames",
    "foveate-input-depth", '\0', "<uint>", "0" };

// Used by: Retina
const ModelOptionDef OPT_ShiftInputToEye =
  { MODOPT_FLAG, "ShiftInputToEye", &MOC_RETINA, OPTEXP_CORE,
    "Shift input frames so that they are centered at current eye position",
    "shift-input", '\0', "", "false" };

// Used by: Retina
const ModelOptionDef OPT_ShiftInputToEyeBGcol =
  { MODOPT_ARG(PixRGB<byte>), "ShiftInputToEyeBGcol", &MOC_RETINA, OPTEXP_CORE,
    "Background color to use when shifting inputs using --shift-input",
    "shift-input-bgcol", '\0', "<r,g,b>", "64,64,64" };

// Used by: Retina
const ModelOptionDef OPT_InputFOV =
  { MODOPT_ARG(Dims), "InputFOV", &MOC_RETINA, OPTEXP_CORE,
    "If non-empty, centrally crop the input images to the given "
    "field-of-view dimensions",
    "input-fov", '\0', "<w>x<h>", "0x0" };

// Used by: Retina
const ModelOptionDef OPT_RetinaSaveInput =
  { MODOPT_FLAG, "RetinaSaveInput", &MOC_RETINA, OPTEXP_CORE,
    "Save retinal input images, with prefix RETIN-",
    "save-retina-input", '\0', "", "false" };

// Used by: Retina
const ModelOptionDef OPT_RetinaSaveOutput =
  { MODOPT_FLAG, "RetinaSaveOutput", &MOC_RETINA, OPTEXP_CORE,
    "Save retina output (possibly including foveation, shifting, "
    "embedding, etc), with prefix RETOUT-",
    "save-retina-output", '\0', "", "false" };

// Used by: RetinaStd
const ModelOptionDef OPT_RetinaStdSavePyramid =
  { MODOPT_FLAG, "RetinaStdSavePyramid", &MOC_RETINA, OPTEXP_CORE,
    "Save pyramid used for retinal foveation, with prefix RET<level>-",
    "save-retina-pyr", '\0', "", "false" };

// Used by: RetinaStd
const ModelOptionDef OPT_RetinaMaskFname =
  { MODOPT_ARG_STRING, "RetinaMaskFname", &MOC_RETINA, OPTEXP_CORE,
    "Mask retinal input images by a greyscale byte image of same dims, "
    "where 0 will transform a retinal pixel to black, 255 will not affect "
    "it, and intermediate values will fade it to black.",
    "retina-mask", '\0', "<filename>", "" };

// Used by: Retina
const ModelOptionDef OPT_RetinaFlipHoriz =
  { MODOPT_FLAG, "RetinaFlipHoriz", &MOC_RETINA, OPTEXP_CORE,
    "Flip raw input images (before any embedding) horizontally.",
    "flip-input-horiz", '\0', "", "false" };

// Used by: Retina
const ModelOptionDef OPT_RetinaFlipVertic =
  { MODOPT_FLAG, "RetinaFlipVertic", &MOC_RETINA, OPTEXP_CORE,
    "Flip raw input images (before any embedding) vertically.",
    "flip-input-vertic", '\0', "", "false" };

// #################### VisualCortex options:

// Used by: VisualCortex, Brain, SingleChannel (and derivatives)
const ModelOptionDef OPT_VisualCortexType =
  { MODOPT_ARG_STRING, "VisualCortexType", &MOC_VCX, OPTEXP_CORE,
    "Type of VisualCortex to use:\n"
    "  None: no VisualCortex at all\n"
    "  Std: use standard (floating-point) channels, most flexible\n"
    "  Beo: use Beowulf channels (requires Beowulf cluster)\n"
    "  Surp: use Surprise channels, the fanciest to date\n"
    "  Int: use integer-math channels, fast yet somewhat flexible\n"
    "  Env: use super-fast integer-math channels, the fastest to date\n"
    "  Entrop: entropy model, computing pixel entropy in 16x16 image patches\n"
    "  EyeMvt: fake visual cortex built from human eye movement traces\n"
    "  PN03contrast: Parkhurst & Niebur'03 contrast model\n"
    "  Variance: local variance in 16x16 image patches\n"
    "  Michelson: Michelson contrast as in Mannan, Ruddock & Wooding '96\n"
    "  Tcorr: temporal correlation in image patches across frames\n"
    "  Scorr: spatial correlation between image patches in a frame\n"
    "  Info: DCT-based local information measure as in Itti et al, PAMI 1998\n"
    "  SpectralResidual: Spectral residual cortex\n"
    "  MultiSpectralResidual: Multi spectral residual cortex\n"
    "You may also configure which channels to use in your VisualCortex by "
    "specifying a series of letters through the option --vc-chans. Finally, "
    "you can apply modifiers by prefixing them to your vc type:\n"
    "  Thread: is a modifier that works with Std, Surp, etc and\n"
    "      is used to dispatch computations to worker threads; (note\n"
    "      that to actually create the worker threads you must also give\n"
    "      a '-j N' option to create N threads); it is OK to have fewer\n"
    "      threads than channels, in which case each thread would simply\n"
    "      perform more than one channel computation per input cycle).\n"
    "      EXAMPLE: --vc-type=Thread:Surp",
    "vc-type", '\0',
    "<None|Std|Beo|Surp|Int|Env|...>",
    "Std" };

// Used by: VisualCortexEyeMvt
const ModelOptionDef OPT_VCEMeyeFnames =
  { MODOPT_ARG_STRING, "VCEMeyeFnames", &MOC_VCX, OPTEXP_CORE,
    "Comma-separated list of human eye movement file names",
    "vcem-eyefnames", '\0', "<name1,name2,...>", "" };

// Used by: VisualCortexEyeMvt
const ModelOptionDef OPT_VCEMsigma =
  { MODOPT_ARG(float), "VCEMsigma", &MOC_VCX, OPTEXP_CORE,
    "Sigma of gaussian blob plotted at each human eye position, in pixels "
    "at the scale of the saliency map, or 0.0 to plot a single pixel",
    "vcem-sigma", '\0', "<float>", "3.0" };

// Used by: VisualCortexEyeMvt
const ModelOptionDef OPT_VCEMforgetFac =
  { MODOPT_ARG(float), "VCEMforgetFac", &MOC_VCX, OPTEXP_CORE,
    "Forgetting factor to be applied at every evolve of VisualCortexEyeMvt",
    "vcem-forgetfac", '\0', "<float>", "0.9995" };

// Used by: VisualCortexEyeMvt
const ModelOptionDef OPT_VCEMdelay =
  { MODOPT_ARG(uint), "VCEMdelay", &MOC_VCX, OPTEXP_CORE,
    "Human-to-human delay, in eye tracker samples",
    "vcem-delay", '\0', "<uint>", "24" };

// Used by: VisualCortexEyeMvt
const ModelOptionDef OPT_VCEMuseMax =
  { MODOPT_FLAG, "VCEMuseMax", &MOC_VCX, OPTEXP_CORE,
    "Use max to comabine across humans, otherwise sum",
    "vcem-usemax", '\0', "", "false" };

// Used by: VisualCortexEyeMvt
const ModelOptionDef OPT_VCEMsaccadeOnly =
  { MODOPT_FLAG, "VCEMsaccadeOnly", &MOC_VCX, OPTEXP_CORE,
    "Only plot endpoint locations of saccades instead of every sample",
    "vcem-saconly", '\0', "", "false" };

// #################### SimulationViewer options:
// Used by: SimulationViewerConfigurator, StdBrain, etc
const ModelOptionDef OPT_SimulationViewerType =
  { MODOPT_ARG_STRING, "SimulationViewerType", &MOC_DISPLAY, OPTEXP_CORE,
    "Type of SimulationViewer to use. \n"
    "\t'Std' is the standard 2D viewer. \n"
    "\t'Compress' is a simple multi-foveated blurring viewer, in "
    "which a selection of most salient locations will be represented "
    "crisply, while the rest of the image will be increasingly blurred as "
    "we move away from those hot spots. \n"
    "\t'EyeMvt' is a viewer for comparison between saliency maps and "
    "human eye movements.\n"
    "\t'EyeMvt2' is 'EyeMvt' plus it adds a concept of visual memory buffer.\n"
    "\t'EyeMvtNeuro' is a viewer for comparing saliency maps and neural "
    "responses.\n"
    "\t'EyeRegion' is 'EyeMvt' plus it adds support for object definitions/"
    "visualizations.\n"
    "\t'EyeSim' simulates an eye-tracker recording from the model. "
    "\tWe run surprise control (ASAC) also from this command option. "
    "To use surprise control use 'ASAC'.\n "
    "\t'Stats' saves some simple statistics like mean and variance "
    "saliency, location of peak saliency, etc.\n"
    "\t'Hand' is a viewer for showing the combination of video and"
    "human hand movement(joystick, etc).\n"
    "\t'EyeHand' is a combination of EyeMvt and Hand.",
    "sv-type", '\0',
    "<None|Std|Compress|EyeMvt|EyeMvt2|EyeRegion|EyeSim"
    "ASAC|NerdCam|Stats|RecStats|Hand|EyeHand>",
    "Std" };

// Used by: SimulationViewer and derivatives
const ModelOptionDef OPT_SVfontSize =
  { MODOPT_ARG(uint), "SVfontSize", &MOC_DISPLAY, OPTEXP_CORE,
    "Use the largest font available with width <= the value given here, or "
    "if there is no such font, then use the smallest available font. Available "
    "fonts range in width from 6 to 20.",
    "font-size", '\0', "<uint>", "10" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVsaveTraj =
  { MODOPT_FLAG, "SVsaveTraj", &MOC_DISPLAY, OPTEXP_CORE,
    "Save attention/eye/head trajectories",
    "save-trajectory", 'T', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVsaveXcombo =
  { MODOPT_FLAG, "SVsaveXcombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Show combination trajec (left) + salmap (right)",
    "save-x-combo", 'X', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVsaveYcombo =
  { MODOPT_FLAG, "SVsaveYcombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Show combination trajec (top) + salmap (bottom)",
    "save-y-combo", 'Y', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVsaveTRMXcombo =
  { MODOPT_FLAG, "SVsaveTRMXcombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Show combination trajec (left) + salmap (middle)+ "
    " task-relevance-map (right)",
    "save-trm-x-combo", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVsaveTRMYcombo =
  { MODOPT_FLAG, "SVsaveTRMYcombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Show combination trajec (top) + salmap (middle) + "
    " task-relevance-map (bottom) ",
    "save-trm-y-combo", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVsaveTRMmegaCombo =
  { MODOPT_FLAG, "SVsaveTRMmegaCombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Show combination trajec (top left) + salmap (top right) + "
    " task-relevance-map (bottom left) + AGM (bottom right) ",
    "save-trm-mega-combo", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVwarp3D =
  { MODOPT_FLAG, "SVwarp3D", &MOC_DISPLAY, OPTEXP_CORE,
    "Show color image warped onto 3D salmap",
    "warp-salmap-3d", '3', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVmegaCombo =
  { MODOPT_FLAG, "SVmegaCombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Show trajectory, saliency and channels as a combo",
    "mega-combo", 'K', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVmegaComboZoom =
  { MODOPT_ARG(uint), "SVmegaComboZoom", &MOC_DISPLAY, OPTEXP_CORE,
    "Zoom factor to use to display the --mega-combo conspicuity and feature maps.",
    "mega-combo-zoom", '\0', "<uint>", "8" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVmegaComboTopCMapsOnly =
  { MODOPT_FLAG, "SVmegaComboTopCMapsOnly", &MOC_DISPLAY, OPTEXP_CORE,
    "In --mega-combo displays, show only the top-level conspicuity maps, as "
    "opposed to recursing through the channel hierarchy and showing all the "
    "conspicuity maps, i.e., all output maps from all channels and "
    "subchannels.",
    "mega-combo-topcm", '\0', "", "true" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVcropFOA =
  { MODOPT_ARG(Dims), "SVcropFOA", &MOC_DISPLAY, OPTEXP_CORE,
    "Crop image to <w>x<h> around FOA location",
    "crop-foa", 'v', "<w>x<h>", "0x0" };

// Used by: SimulationViewer, SaliencyMap, etc
const ModelOptionDef OPT_SVdisplayMapFactor =
  { MODOPT_ARG(float), "SVdisplayMapFactor", &MOC_DISPLAY, OPTEXP_CORE,
    "When displaying/saving a saliency map, visual cortex output, task "
    "relevance map, attention guidance map, etc, multiply the map by <factor> "
    "to convert its raw values to values which can be displayed (typically, "
    "in the [0..255] range). If the given <factor> is 0.0, then the map will "
    "be normalized to [0..255] on a frame-by-frame basis (which may "
    "sometimes be misleading as it may visually overemphasize maps which "
    "only contain very low activation).",
    "display-map-factor", '\0', "<factor>", "0.0" };

// Used by: SaliencyMap and derivatives
const ModelOptionDef OPT_SVdisplayMapType =
  { MODOPT_ARG_STRING, "SVdisplayMapType", &MOC_DISPLAY, OPTEXP_CORE,
    "Select which map to display in all displays which show a map alongside "
    "the attention trajectory. The map values will be normalized according "
    "to the value of --display-map-factor.",
    "display-map", '\0', "<SM|AGM|TRM|VCO>", "AGM" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVxwindow =
  { MODOPT_OBSOLETE, "SVxwindow", &MOC_DISPLAY, OPTEXP_CORE,
    "This option is obsolete; if you want to see results in an "
    "onscreen window you can try using --out=display or --out=qt "
    "instead",
    "show-xwindow", 'x', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVfoveateTraj =
  { MODOPT_FLAG, "SVfoveateTraj", &MOC_DISPLAY, OPTEXP_CORE,
    "foveate trajectory (cumulative if --display-additive)",
    "foveate-traj", '\0', "", "false" };

// Used by: SimulationViewerStd
const ModelOptionDef OPT_SVstatsFname =
  { MODOPT_ARG_STRING, "SVstatsFname", &MOC_DISPLAY, OPTEXP_CORE,
    "Save various statistics like min/max/avg salience, feature map "
    "values at the location of max salience, and feature map values "
    "at random locations, into a text file. The text file will also "
    "contain a dump of the model's architecture so that the numbers "
    "can be interpreted unambiguously.",
    "sv-stats-fname", '\0', "<filename>", "" };


// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayFOA =
  { MODOPT_FLAG, "SVdisplayFOA", &MOC_DISPLAY, OPTEXP_CORE,
    "Display focus-of-attention as a circle",
    "display-foa", '\0', "", "true" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayPatch =
  { MODOPT_FLAG, "SVdisplayPatch", &MOC_DISPLAY, OPTEXP_CORE,
    "Display small filled square at attended location",
    "display-patch", '\0', "", "true" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVpatchSize =
  { MODOPT_ARG(int), "SVpatchSize", &MOC_DISPLAY, OPTEXP_CORE,
    "Size of square to display at observers eye position",
    "patch-size", '\0', "<int>", "4" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVeraseMarker =
  { MODOPT_FLAG, "SVeraseMarker", &MOC_DISPLAY, OPTEXP_CORE,
    "Erase the eye position marker after each frame. ",
    "erase-marker", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayFOAnum =
  { MODOPT_FLAG, "SVdisplayFOAnum", &MOC_DISPLAY, OPTEXP_CORE,
    "Display attention shift number (0-based)",
    "display-foanum", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayFOALinks =
  { MODOPT_FLAG, "SVdisplayFOALinks", &MOC_DISPLAY, OPTEXP_CORE,
    "Display attentional trajectory using red arrows",
    "display-traj", '\0', "", "true" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayEye =
  { MODOPT_FLAG, "SVdisplayEye", &MOC_DISPLAY, OPTEXP_CORE,
    "Display small hollow square at eye position",
    "display-eye", '\0', "", "true" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayEyeLinks =
  { MODOPT_FLAG, "SVdisplayEyeLinks", &MOC_DISPLAY, OPTEXP_CORE,
    "Display eye trajectory using red lines",
    "display-eye-traj", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayHead =
  { MODOPT_FLAG, "SVdisplayHead", &MOC_DISPLAY, OPTEXP_CORE,
    "Display larger hollow square at head position",
    "display-head", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayHeadLinks =
  { MODOPT_FLAG, "SVdisplayHeadLinks", &MOC_DISPLAY, OPTEXP_CORE,
    "Display head trajectory using red lines",
    "display-head-traj", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayAdditive =
  { MODOPT_FLAG, "SVdisplayAdditive", &MOC_DISPLAY, OPTEXP_CORE,
    "Display things additively",
    "display-additive", '\0', "", "true" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayTime =
  { MODOPT_FLAG, "SVdisplayTime", &MOC_DISPLAY, OPTEXP_CORE,
    "Display internal simulation time",
    "display-time", '\0', "", "true" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayHighlights =
  { MODOPT_FLAG, "SVdisplayHighlights", &MOC_DISPLAY, OPTEXP_CORE,
    "Display highlight at focus-of-attention",
    "display-highlight", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplaySMmodulate =
  { MODOPT_FLAG, "SVdisplaySMmodulate", &MOC_DISPLAY, OPTEXP_CORE,
    "Display surprise-modulated image, using the saliency map (possibly "
    "normalized using --display-map-factor) as contrast modulator",
    "display-smmod", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayBoring =
  { MODOPT_FLAG, "SVdisplayBoring", &MOC_DISPLAY, OPTEXP_CORE,
    "Display attention shifts to boring targets in green",
    "display-boring", '\0', "", "true" };

// Used by: SimulationViewerStd
const ModelOptionDef OPT_SVuseLargerDrawings =
  { MODOPT_FLAG, "SVuseLargerDrawings", &MOC_DISPLAY, OPTEXP_CORE,
    "Use larger drawings for FOA, FOV, and head markers than default",
    "display-larger-markers", '\0', "", "false" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVdisplayInterp =
  { MODOPT_FLAG, "SVdisplayInterp", &MOC_DISPLAY, OPTEXP_CORE,
    "Use bilinear interpolation when rescaling saliency or other maps to "
    "larger image sizes for display (this option does not affect the maps "
    "themselves, just the way in which they are displayed)",
    "display-interp-maps", '\0', "", "true" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPsaveMask =
  { MODOPT_FLAG, "SVCOMPsaveMask", &MOC_DISPLAY, OPTEXP_CORE,
    "Show the bluring mask only",
    "save-compress-mask", '\0', "", "false" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPsaveFoveatedImage =
  { MODOPT_FLAG, "SVCOMPsaveFoveatedImage", &MOC_DISPLAY, OPTEXP_CORE,
    "Show the foveated image only",
    "save-compress-foveatedimage", '\0', "", "false" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPDistanceFactor =
  { MODOPT_ARG(float), "SVCOMPDistanceFactor", &MOC_DISPLAY, OPTEXP_CORE,
    "Set the distance factor",
    "distance-factor", '\0', "<float>", "1.0" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPsaveEyeCombo =
  { MODOPT_FLAG, "SVCOMPsaveEyeCombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Show combination of original (left), foveated image + human eye "
    "(middle) and raw saliency map (right)",
    "save-compress-eyecombo", '\0', "", "false" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPnumFoveas =
  { MODOPT_ARG(int), "SVCOMPnumFoveas", &MOC_DISPLAY, OPTEXP_CORE,
    "Number of foveas to use for multi-foveated image blurring. A value of "
    "zero is valid and means that the blur will be continuously computed "
    "from the saliency map, rather than from a discrete number of foveas",
    "num-foveas", '\0', "<int>", "0" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPeyeCompare =
  { MODOPT_FLAG, "SVCOMPeyeCompare", &MOC_DISPLAY, OPTEXP_CORE,
    "use the eyetracking data to compared with teh blur mask or not",
    "eyecompare-compress", '\0', "<true|false>", "false"};

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPcacheSize =
  { MODOPT_ARG(int), "SVCOMPcacheSize", &MOC_DISPLAY, OPTEXP_CORE,
    "Number of frames over which the object masks are averaged to "
    "determine current foveation mask",
    "maskcache-size", '\0', "<int>", "5" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPiframePeriod =
  { MODOPT_ARG(int), "SVCOMPiframePeriod", &MOC_DISPLAY, OPTEXP_CORE,
    "Period of occurence of I-Frames when compressing with MPEG-1. If this "
    "is not 1, the foveation will be allowed to change only on every "
    "I-Frame, but will remain stable on P-frames and B-frames",
    "iframe-period", '\0', "<int>", "1" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPdisplayHumanEye =
  { MODOPT_FLAG, "SVCOMPdisplayHumanEye", &MOC_DISPLAY, OPTEXP_CORE,
    "Display human eye positions",
    "display-human-eye", '\0', "<true|false>", "true" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPuseTRMmax =
  { MODOPT_FLAG, "SVCOMPuseTRMmax", &MOC_DISPLAY, OPTEXP_CORE,
    "Use TRM-max trick",
    "use-trmmax", '\0', "", "false" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPfoveaSCtype =
  { MODOPT_ARG_STRING, "SVCOMPfoveaSCtype", &MOC_DISPLAY, OPTEXP_CORE,
    "Type of SC to use for the foveas of the compression viewer",
    "compress-sc-type", '\0', "<type>", "Friction" };

// Used by: SimulationViewerCompress
const ModelOptionDef OPT_SVCOMPMultiRetinaDepth =
  { MODOPT_ARG(int), "SVCOMPMultiRetinaDepth", &MOC_DISPLAY, OPTEXP_CORE,
    "Depth of pyramid used for multi-foveation",
    "compress-multiretina-depth", '\0', "<int>", "5" };


// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMsaveMegaCombo =
  { MODOPT_FLAG, "SVEMsaveMegaCombo", &MOC_DISPLAY, OPTEXP_CORE,
    "If --maxcache-size is non-zero, then generate a 4-image combo: "
    "original (top-left), max-saliency-weighted image "
    "(top-right), raw saliency map (bottom-left) and max-saliency mask "
    "(bottom-right); otherwise of --maxcache-size is zero, then "
    "generate a 2-image combo: original plus eye position (left) and "
    "saliency map (right)",
    "save-eyemvt-megacombo", '\0', "", "false" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMdelayCacheSize =
  { MODOPT_ARG(int), "SVEMdelayCacheSize", &MOC_DISPLAY, OPTEXP_CORE,
    "Number of eye movement samples to skip between current time and "
    "time at which we start computing the max saliency over a period of "
    "time, or 0 for no delay",
    "delaycache-size", '\0', "<int>", "0" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMmaxCacheSize =
  { MODOPT_ARG(int), "SVEMmaxCacheSize", &MOC_DISPLAY, OPTEXP_CORE,
    "Number of eye movement samples over which to compute the max saliency, "
    "or 0 for no cache",
    "maxcache-size", '\0', "<int>", "0" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMoutFname =
  { MODOPT_ARG_STRING, "SVEMoutFname", &MOC_DISPLAY, OPTEXP_CORE,
    "File name for output data (or empty to not save output data).",
    "svem-out-fname", '\0', "<file>", "" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMsampleAtStart =
  { MODOPT_ARG(bool), "SVEMsampleAtStart", &MOC_DISPLAY, OPTEXP_CORE,
    "Sample at start of saccade (otherwise at end of saccade)",
    "svem-sampleatstart", '\0', "<true|false>", "true" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMdisplaySacNum =
  { MODOPT_FLAG, "SVEMdisplaySacNum", &MOC_DISPLAY, OPTEXP_CORE,
    "Display saccade number",
    "svem-display-sacnum", '\0', "", "false" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMnumRandomSamples =
  { MODOPT_ARG(int), "SVEMnumRandomSamples", &MOC_DISPLAY, OPTEXP_CORE,
    "Number of random samples to include in the file given by "
    "--svem-out-fname. NOTE: if the number given here is greater "
    "than or equal to the number of pixels in the saliency map, then "
    "instead of choosing random values, we simply loop over the "
    "entire saliency map, listing each value just once; so, note that "
    "regardless of the value given here, the number of 'random' values "
    "written will never be more than the number of pixels in the "
    "saliency map.",
    "svem-num-random", '\0', "<integer>",
    // the default value needs to stay at 100 in order to maintain
    // backward compatibility with previous versions where
    // SimulationViewerEyeMvt had a hardcoded "#define NBRND 100"
    "100" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMhistoryNumRandomSamples =
  { MODOPT_ARG(uint), "SVEMhistoryNumRandomSamples", &MOC_DISPLAY, OPTEXP_CORE,
		"Number of random samples in the history (sm-history-qlen) "
		"to include in the file fiven by --svem-out-fname. ",
    "svem-history-num-random", '\0', "<uint>",
    "1" };

//score a saccade that is in a blink?
const ModelOptionDef OPT_SVEMuseSaccadeInBlink =
{ MODOPT_FLAG, "SVEMuseSaccadeInBlink", &MOC_DISPLAY, OPTEXP_CORE,
  "When sampling a saliency map from human fixations choose whether "
  "to use saccades during blinks.","use-blink", '\0', "",
  "true"
};


//use our diagnostic color scheme when making visualizations
const ModelOptionDef OPT_SVEMuseDiagColors =
{ MODOPT_FLAG, "SVEMuseDiagColors", &MOC_DISPLAY, OPTEXP_CORE,
    "Use the same color scheme as MarkEye so to easily check "
    "the accuracy of parsing",
    "use-diagnostic-color", '\0', "",
    "false"
};

//use our diagnostic color scheme when making visualizations
const ModelOptionDef OPT_SVEMlabelEyePatch =
  { MODOPT_FLAG, "SVEMlabelEyePatch", &MOC_DISPLAY, OPTEXP_CORE,
    "Label each patch with an identifying label",
    "svem-label-patch", '\0', "", "false"};

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMpriorRandomDistro =
  { MODOPT_ARG_STRING, "SVEMpriorRandomDistro", &MOC_DISPLAY, OPTEXP_CORE,
    "When sampling the saliency map use the supplied text file of saccadic "
    "endpoints in screen coordinates instead of a uniform sampling. "
    "File format is 1 sample per line.  Each line should be formatted: Xcoord"
    " Ycoord","svem-prior-distro", '\0', "<file>", "" };

/// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMviewingStrategy=
  { MODOPT_ARG(bool), "SVEMviewingStrategy", &MOC_DISPLAY, OPTEXP_CORE,
		"Compensate viewing strategy with random sampling baseline "
		"(see Viewing Strategy in Tseng et al. (2009)).  Please specify "
		"svem-vs-timing and svem-vs-distro as well.",
    "svem-vs-baseline", '\0', "<true|false>", "false" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMviewingStrategyTiming=
  { MODOPT_ARG_STRING, "SVEMviewingStrategyTiming", &MOC_DISPLAY, OPTEXP_CORE,
		"When sampling the saliency map with a gaussian described in svem-vs-distro, "
		"the file lists frame numbers (start from 0, line by line) that will reset "
		"the viewing time back to 0, which is useful for videos of scene changes. "
		"Saccade onset time from scene change (time_scene_change) will be added into "
		"ezvision output.  If svem-vs-baseline is false but this option is provided with "
		"a file, then the information about saccade onset time from scene change will be "
		"added but the viewing strategy is not compensated in the random sampling baseline.",
    "svem-vs-timing", '\0', "<file>", "" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMviewingStrategyRandomDistro =
  { MODOPT_ARG_STRING, "SVEMviewingStrategyRandomDistro", &MOC_DISPLAY, OPTEXP_CORE,
		"Sampling the saliency map by a gaussian distribution whose standard deviation (SD)"
		"increases with viewing time.  "
		"The evolution of SD is described by a sigmoid function: y = a + b/(1+exp(-(t+c))/d), "
		"where t is time and a, b, c, and d are parameters (parameter a and b are in the "
		"unit of pixel (default values were obtained with 19 pixels per degree (ppd). You can "
		"scale the number if you have different ppd), and c and d are in the unit of mini-second). "
		"This option takes 8 parameters, where the first 4 describes the SD of the horizontal axis (x), "
		"and the later 4 describes the SD of the vertical axis (y).  This option won't take effect "
		"unless svem-vs-baseline is true. ",
    "svem-vs-distro", '\0', "<xa,xb,xc,xd,ya,yb,yc,yd>", 
		"39.71,479.56,-553.62,211.53,9.88,3828.5,-2366.70,552.48" };

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMwriteFrameNumber =
  { MODOPT_FLAG, "SVEMwriteFrameNumber",&MOC_DISPLAY, OPTEXP_CORE,
    "Report the frame number of the saccade in the exported eyesal file.",
    "svem-write-framenum",'\0',"","false"};

// Used by: SimulationViewerEyeMvt
const ModelOptionDef OPT_SVEMmaxComboWidth =
  { MODOPT_ARG(int), "SVEMmaxComboWidth", &MOC_DISPLAY, OPTEXP_CORE,
    "Maximum width for the result of --save-eyemvt-megacombo; if the "
    "raw width of the combo image is greater than this value, then "
    "smooth+reduce the image by factors of 2 until the width is less "
    "than the desired size",
    "svem-max-combo-width", '\0', "<integer>",
    // the default value needs to stay at 1024 in order to maintain
    // backward compatibility with previous versions where
    // SimulationViewerEyeMvt had a hardcoded 1024 in
    // SimulationViewerEyeMvt::getTraj()
    "1024" };

// Used by: SimulationViewerEyeMvt2
const ModelOptionDef OPT_SVEMbufDims =
  { MODOPT_ARG(Dims), "SVEMbufDims", &MOC_DISPLAY, OPTEXP_CORE,
    "Dimensions of internal visual buffer",
    "svem-bufdims", '\0', "<w>x<h>", "" };

// Used by: SimulationViewerEyeMvt2
const ModelOptionDef OPT_SVEMuseIOR =
  { MODOPT_FLAG, "SVEMuseIOR", &MOC_DISPLAY, OPTEXP_CORE,
    "Attempt to guess IOR from human eye movements",
    "svem-use-ior", '\0', "", "true" };


// Used by: SimulationViewerHand
const ModelOptionDef OPT_SVHandSaveCombo =
  { MODOPT_FLAG, "SVHandSaveCombo", &MOC_DISPLAY, OPTEXP_CORE,
    "Generate a 3-image combo: original plus eye position (left),"
    "and hand-movement (right)",
    // Should i change this? :
    //"If --maxcache-size is non-zero, then generate a 4-image combo: "
    //"original (top-left), max-saliency-weighted image "
    //"(top-right), raw saliency map (bottom-left) and max-saliency mask "
    //"(bottom-right); otherwise of --maxcache-size is zero, then "
    //"generate a 2-image combo: original plus eye position (left) and "
    //"hand movement (right)",
    "svhand-savecombo", '\0', "", "false" };

// Used by: SimulationViewerHand
const ModelOptionDef OPT_SVHandMaxComboWidth =
  { MODOPT_ARG(int), "SVHandMaxComboWidth", &MOC_DISPLAY, OPTEXP_CORE,
    "Maximum width for the result of --svhand-savecombo; if the "
    "raw width of the combo image is greater than this value, then "
    "smooth+reduce the image by factors of 2 until the width is less "
    "than the desired size",
    "svhand-maxcombowidth", '\0', "<integer>",
    // the default value needs to stay at 1024 in order to maintain
    // backward compatibility with previous versions where
    // SimulationViewerHand had a hardcoded 1024 in
    // SimulationViewerHand::getTraj()
    "1024" };

// Used by: SimulationViewerHand
const ModelOptionDef OPT_SVHandDisplay =
  { MODOPT_FLAG, "SVHandDisplay", &MOC_DISPLAY, OPTEXP_CORE,
    "Display the hand movement (joystick) on screen",
    "svhand-display", '\0', "", "true" };


// Used by: SimulationViewerEyeSim
const ModelOptionDef OPT_SVeyeSimFname =
  { MODOPT_ARG_STRING, "SVeyeSimFname", &MOC_DISPLAY, OPTEXP_CORE,
    "File name for output eye movement data (required)",
    "eyesim-fname", '\0', "<file>", "" };

// Used by: SimulationViewerEyeSim
const ModelOptionDef OPT_SVeyeSimPeriod =
  { MODOPT_ARG(SimTime), "SVeyeSimPeriod", &MOC_DISPLAY, OPTEXP_CORE,
    "Eye movements sampling period",
    "eyesim-period", '\0', "<float>", "0.0041666666" }; // 240Hz

// Used by: SimulationViewerEyeSim
const ModelOptionDef OPT_SVeyeSimTrash =
  { MODOPT_ARG(int), "SVeyeSimTrash", &MOC_DISPLAY, OPTEXP_CORE,
    "Number of initial eye movement samples to trash (e.g., "
    "corresponding to initial fixation before actual stimulus)",
    "eyesim-trash", '\0', "<int>", "0" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNprobeLocation =
  { MODOPT_ARG(Point2D<float>),"SVEMNprobeLocation", &MOC_DISPLAY, OPTEXP_CORE,
    "The location of a virtual probe which saves measurements of values in "
    "model maps. The probe location is relative to the center of gaze "
    "in degrees of visual angle. Negative values are left and down of center. "
    "Use -1,-1 for no probe.",
    "svemn-probe", '\0', "<x>,<y>", "-1,-1" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNrfSize =
  { MODOPT_ARG(float), "SVEMNrfSize", &MOC_DISPLAY, OPTEXP_CORE,
    "The size of the neurons receptive field in degrees of visual angle. "
    "Saliency values will be averaged in this window. If set to zero, "
    "the pixel at the probe location will be used.",
    "svemn-rf-size", '\0', "<uint>", "0" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNrfMaskName =
  { MODOPT_ARG_STRING, "SVEMNrfMaskName", &MOC_DISPLAY, OPTEXP_CORE,
    "Use a weighting mask instead of taking the average saliency in the "
    "neurons receptive field centered on the probe location. This image "
    "should be the same size as the input.", "svemn-rf-maskname", '\0', 
    "<file>", "" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNoutFName =
  { MODOPT_ARG_STRING, "SVEMNoutFName", &MOC_DISPLAY, OPTEXP_CORE,
    "If set, output saliency values at the probe location set by "
    "--svemn-probe to this file.", "svemn-out-fname", '\0', "<file>", "" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNneuroFileName =
  { MODOPT_ARG_STRING, "SVEMNneuroFileName", &MOC_DISPLAY, OPTEXP_CORE,
    "If set, read spike or voltage data from a text file and display it "
    "along side the saliency values. The text file should be 1 time step "
    "per line, and the sampling rate is assumed to be the same as the eye "
    "movement file set with --eyetrack-data",
    "svemn-neuron-file", '\0', "<file>", "" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNdisplayTime = 
  { MODOPT_FLAG, "SVEMNdisplayTime", &MOC_DISPLAY, OPTEXP_CORE,
    "Display the simulation time on the output ",
    "svemn-displaytime", '\0', "<bool>", "true" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNdelayNeuron = 
  { MODOPT_ARG(int), "SVEMNdelayNeuron", &MOC_DISPLAY, OPTEXP_CORE,
    "delay the neural data by this number of samples ",
    "svemn-delay-neuron", '\0', "<int>", "0" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNplotBufLength = 
  { MODOPT_ARG(SimTime), "SVEMNplotBufLength", &MOC_DISPLAY, OPTEXP_CORE,
    "Set how long of a history should we output in the plotting window",
    "svemn-plot-length", '\0', "<SimTime>", "1s" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNSalScale = 
  { MODOPT_ARG(float), "SVEMNSalScale", &MOC_DISPLAY, OPTEXP_CORE,
    "y-axis scale for history plot of saliency values. Use 0 for auto "
    "scale to max.",
    "svemn-sal-scale", '\0', "<float>", "0.0" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNNeuroScale = 
  { MODOPT_ARG(float), "SVEMNNeuroScale", &MOC_DISPLAY, OPTEXP_CORE,
    "y-axis scale for history plot of saliency values. Use 0 for auto "
    "scale to max.",
    "svemn-neuro-scale", '\0', "<float>", "0.0" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNNeuroRFVisFile = 
  { MODOPT_ARG_STRING, "SVEMNNeuroRFVisFile", &MOC_DISPLAY, OPTEXP_CORE,
    "If set, attempt to construct a map of the visual receptive field. "
    "The file will contain, for each pixel, the sum of snapshots from the"
    "display map everytime a spike occured. If the file exists and contains"
    "values the sums will continue. The first pixel in the image will be"
    "sacrificed to store the spike count.",
    "svemn-visualrf-file", '\0', "<file>", "" };


// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNNeuroRFMotFile = 
  { MODOPT_ARG_STRING, "SVEMNNeuroRFMotFile", &MOC_DISPLAY, OPTEXP_CORE,
    "If set, attempt to construct a map of the motor response field. "
    "The file will contain, for each pixel, the sum of spikes that occured"
    "to that pixels location in space. If the file exists and contains"
    "values the sums will continue. The first pixel in the image will be"
    "sacrificed to store the spike count.",
    "svemn-motorrf-file", '\0', "<file>", "" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNMotWindow =
  { MODOPT_ARG(SimTime), "SVEMNMotWindow", &MOC_DISPLAY, OPTEXP_CORE,
    "Small window around +- saccade onset to count spikes when creating"
    "response field.",
    "svemn-motrf-window", '\0', "<float>", "25ms" }; 

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNVisWindow = 
  { MODOPT_ARG(SimTime), "SVEMNVisWindow", &MOC_DISPLAY, OPTEXP_CORE,
    "Small window to store an ensemble of spike triggered stimuli.",
    "svemn-visrf-window", '\0', "<float>", "100ms" }; 

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNVisOffset =   
  { MODOPT_ARG(SimTime), "SVEMNVisOffset", &MOC_DISPLAY, OPTEXP_CORE,
    "Offset to store spike triggreed stimuli, to account for stimulus to "
    "spike delay",
    "svemn-visrf-offset", '\0', "<float>", "25ms" }; 

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNShowSCMap =
{ MODOPT_FLAG, "SVEMNShowSCMap", &MOC_DISPLAY, OPTEXP_CORE,
  "should we show the anatomical SC map in our display?",
  "show-scmap", '\0', "", "false" };

// Used by: SimulationViewerEyeMvtNeuro
const ModelOptionDef OPT_SVEMNSCMapDims =
{ MODOPT_ARG(Dims), "SVEMNSCMapDims", &MOC_DISPLAY, OPTEXP_CORE,
  "output dims of anatomical sc map",
  "scmap-dims", '\0', "<wxh>", "480x270" };

// Used by: Surprise Control ASAC
const ModelOptionDef OPT_ASACdrawDiffParts =
  { MODOPT_FLAG, "ASACdrawDiffParts", &MOC_DISPLAY, OPTEXP_CORE,
    "Cause Surprise Control to draw difference images",
    "ASAC-draw-diff-parts", '\0', "", "false" };

// Used by: Surprise Control ASAC
const ModelOptionDef OPT_ASACdrawBetaParts =
  { MODOPT_FLAG, "ASACdrawBetaParts", &MOC_DISPLAY, OPTEXP_CORE,
    "Cause Surprise Control to draw beta images",
    "ASAC-draw-beta-parts", '\0', "", "false" };

// Used by: Surprise Control ASAC
const ModelOptionDef OPT_ASACdrawBiasParts =
  { MODOPT_FLAG, "ASACdrawBiasParts", &MOC_DISPLAY, OPTEXP_CORE,
    "Cause Surprise Control to draw bias images",
    "ASAC-draw-bias-parts", '\0', "", "false" };

// Used by: Surprise Control ASAC
const ModelOptionDef OPT_ASACdrawSeperableParts =
  { MODOPT_FLAG, "ASACdrawSeperableParts", &MOC_DISPLAY, OPTEXP_CORE,
    "Cause Surprise Control to draw layers of seperable filter images",
    "ASAC-draw-seperable-parts", '\0', "", "false" };

// Used by: Surprise Control ASAC
const ModelOptionDef OPT_ASACconfigFile =
{ MODOPT_ARG_STRING, "ASACconfigFile", &MOC_DISPLAY, OPTEXP_CORE,
    "What config file to load with Surprise Control",
    "ASAC-config-file", '\0', "<file>",
    "/lab/mundhenk/saliency/etc/surpriseControl.conf"};

// Used by: Nerd-Cam
const ModelOptionDef OPT_NerdCamConfigFile =
{ MODOPT_ARG_STRING, "NerdCamConfigFile", &MOC_DISPLAY, OPTEXP_CORE,
    "What config file to load with Nerd Cam",
    "nerdcam-config-file", '\0', "<file>",
    "/etc/nerdcam.conf"};


// Used by: SimulationViewer
const ModelOptionDef OPT_FOAradius =
  { MODOPT_ARG(int), "FOAradius", &MOC_DISPLAY, OPTEXP_CORE,
    "Radius of the Focus Of Attention, or 0 to set it from input image dims",
    "foa-radius", '\0', "<pixels>", "0" };

// Used by: SimulationViewer
const ModelOptionDef OPT_SVInverseTransform =
  { MODOPT_FLAG, "InvertTransform", &MOC_DISPLAY, OPTEXP_CORE,
    "If we are using a retinal type that performs a coordinate transfrom on the input, "
    "should we undo that transform for display?", "inverse-retinal", '\0', "", "true" };

// Used by: SimulationViewer, SaccadeControllers
const ModelOptionDef OPT_FoveaRadius =
  { MODOPT_ARG(int), "FoveaRadius", &MOC_DISPLAY, OPTEXP_CORE,
    "Radius of the fovea, or 0 to set it from input image dims",
    "fovea-radius", '\0', "<pixels>", "0" };

// Used by: SimulationViewerStats
const ModelOptionDef OPT_AGStatsSaveFile =
{ MODOPT_ARG_STRING, "AGStatsSaveFile", &MOC_DISPLAY, OPTEXP_CORE,
    "Prefix for saving attention gate stats",
    "ag-statsfile", '\0', "<file>", "ag-stats"};

// Used by: SimulationViewerStats
const ModelOptionDef OPT_ComputeAGStats =
  { MODOPT_FLAG, "ComputeAGStats", &MOC_DISPLAY, OPTEXP_CORE,
    "Should we compute the AG stats with ground truth?",
    "savestats-ag", '\0', "", "false" };

// #################### InferoTemporal options:

const ModelOptionCateg MOC_ITC = {
  MOC_SORTPRI_3, "Inferior Temporal Cortex Options" };

// Used by: Brain
const ModelOptionDef OPT_InferoTemporalType =
  { MODOPT_ARG_STRING, "ITCInferoTemporalType", &MOC_ITC, OPTEXP_CORE,
    "Type of InferoTemporal to use. 'None','Std','SalBayes', 'HMAX' or 'CUDAHMAX'",
    "it-type", '\0', "<None|Std|SalBayes|HMAX|CUDAHMAX>", "None" };


// Used by: InferoTemporal
const ModelOptionDef OPT_AttentionObjRecog =
  { MODOPT_FLAG, "ITCAttentionObjRecog", &MOC_ITC, OPTEXP_CORE,
    "Use ShapeEstimator to create a mask around the attended object prior "
    "to recognition.",
    "it-use-se", '\0', "", "no" };


// Used by: InferoTemporal
const ModelOptionDef OPT_ObjectDatabaseFileName =
  { MODOPT_ARG_STRING, "ITCObjectDatabaseFileName", &MOC_ITC, OPTEXP_CORE,
    "Filename for the object database",
    "it-object-db", '\0', "<filename>", "" };


// Used by: InferoTemporal
const ModelOptionDef OPT_TrainObjectDB =
  { MODOPT_FLAG, "ITCTrainObjectDB", &MOC_ITC, OPTEXP_CORE,
    "If set to yes, train the database by adding to it new objects which "
    "could not be recognized.",
    "it-train-db", '\0', "", "no" };


// Used by: InferoTemporal
const ModelOptionDef OPT_PromptUserTrainDB =
  { MODOPT_FLAG, "ITCPromptUserTrainDB", &MOC_ITC, OPTEXP_CORE,
    "Whether or not to prompt the user before training the database on a "
    "new object",
    "it-prompt-user-train-db", '\0', "", "no" };


// Used by: InferoTemporal
const ModelOptionDef OPT_MatchObjects =
  { MODOPT_FLAG, "ITCMatchObjects", &MOC_ITC, OPTEXP_CORE,
    "If true, attempt to recognize objects at every new attended location. "
    "When this is false, having an InferoTemporal may still make sense as "
    "it will be in pure training mode, just learning the representation "
    "of every object attended to.",
    "it-match-objects", '\0', "", "yes"};


// Used by: InferoTemporal
const ModelOptionDef OPT_MatchingAlgorithm =
  { MODOPT_ARG(VisualObjectMatchAlgo), "ITCMatchingAlgorithm", &MOC_ITC, OPTEXP_CORE,
    "The algorithm to use for object matching",
    "matching-alg", '\0', "<Simple|KDtree|KDtreeBBF>", "KDtreeBBF"};


// Used by: InferoTemporal
const ModelOptionDef OPT_RecognitionMinMatch =
  { MODOPT_ARG(int), "ITCRecognitionMinMatch", &MOC_ITC, OPTEXP_CORE,
    "The minimum number of keypoint matches for recognizing an object. If "
    "greater than 0, search will be terminated once an object with the "
    "minimum number of keypoint matches is found. If equal to 0, the "
    "percentage of keypoints specified by recog-minmatch-percent must be "
    "matched to declare an object match",
    "recog-minmatch", '\0', "<int>", "3" };


// Used by: InferoTemporal
const ModelOptionDef OPT_RecognitionMinMatchPercent =
  { MODOPT_ARG(float), "ITCRecognitionMinMatchPercent", &MOC_ITC, OPTEXP_CORE,
    "The minimum percentage of keypoints in the target object that must be "
    "successfully matched in order to declare an object match.",
    "recog-minmatch-percent", '\0', "<float>", "0.75" };


// Used by: InferoTemporal
const ModelOptionDef OPT_SortObjects =
  { MODOPT_FLAG, "ITCSortObjects", &MOC_ITC, OPTEXP_CORE,
    "Whether or not to sort the objects in the database before matching. "
    "similarity before matching.",
    "sort-objects", '\0', "", "no"};


// Used by: InferoTemporal
const ModelOptionDef OPT_SortObjectsBy =
  { MODOPT_ARG_STRING, "ITCSortObjectsBy", &MOC_ITC, OPTEXP_CORE,
    "The metric to use to sort the objects in the database - 'features' or "
    "'location'. Only has effect when --sort-objects=yes. ",
    "sort-objects-by", '\0', "<features|location>", "features"};


// Used by: InferoTemporal
const ModelOptionDef OPT_SortKeys =
  { MODOPT_FLAG, "ITCSortKeys", &MOC_ITC, OPTEXP_CORE,
    "Whether or not to sort the keys by scale space extrema magnitude "
    "before matching.",
    "sort-keys", '\0', "", "no"};


// Used by: InferoTemporal
const ModelOptionDef OPT_OutputFileName =
  { MODOPT_ARG_STRING, "ITCOutputFileName", &MOC_ITC, OPTEXP_CORE,
    "The name of the object recognition output file",
    "objrecog-output", '\0', "<filename>", ""};




// #################### ShapeEstimator options:
// Used by: ShapeEstimator
const ModelOptionDef OPT_ShapeEstimatorMode =
  { MODOPT_ARG(ShapeEstimatorMode), "ShapeEstimatorMode",
    &MOC_BRAIN, OPTEXP_CORE,
    "Shape estimator mode",
    "shape-estim-mode", '\0', 
    "<None|FeatureMap|ConspicuityMap|SaliencyMap|"
    "MTfeatureMap|ContourBoundaryMap|CSHistogram>",
    "None" };

const ModelOptionDef OPT_ShapeEstimatorSmoothMethod =
  { MODOPT_ARG(ShapeEstimatorSmoothMethod), "ShapeEstimatorSmoothMethod",
    &MOC_BRAIN, OPTEXP_CORE,
    "Shape estimator smooth method",
    "shape-estim-smoothmethod", '\0', "<None|Gaussian|Chamfer>",
    "Gaussian" };




// #################### Brain options:

const ModelOptionCateg MOC_BRAIN = {
  MOC_SORTPRI_3, "General Brain/Attention-Related Options" };

// Used by: Brain
const ModelOptionDef OPT_BrainBoringDelay =
  { MODOPT_ARG(SimTime), "BrainBoringDelay", &MOC_BRAIN, OPTEXP_CORE,
    "Attention shifts that come after an interval longer than this "
    "will be considered 'boring' (for example, they may be represented "
    "differently in output displays)",
    "boring-delay", '\0', "<time>", "200.0ms" };

// Used by: Brain
const ModelOptionDef OPT_BrainBoringSMmv =
  { MODOPT_ARG(float), "BrainBoringSMmv", &MOC_SM, OPTEXP_CORE,
    "Saliency map level (in mV) below which attention shifts will be "
    "considered 'boring' (for example, they may be represented "
    "differently in output displays)",
    "boring-sm-mv", '\0', "<float>", "3.0" };

// Used by: Brain
const ModelOptionDef OPT_IORtype =
  { MODOPT_ARG(IORtype), "IORtype", &MOC_BRAIN, OPTEXP_CORE,
    "Type of IOR to use; default is ShapeEstimator-based, but if "
    "no ShapeEstimator information is available, then we fall back to "
    "Disc IOR",
    "ior-type", '\0', "<None|Disc|ShapeEst>", "ShapeEst" };

// Used by: Brain
const ModelOptionDef OPT_BrainSaveObjMask =
  { MODOPT_FLAG, "BrainSaveObjMask", &MOC_DISPLAY, OPTEXP_SAVE,
    "Save object mask",
    "save-objmask", '\0', "", "false" };

// Used by: Brain
const ModelOptionDef OPT_BlankBlink =
  { MODOPT_FLAG, "BlankBlink", &MOC_BRAIN, OPTEXP_CORE,
    "Blank-out visual input during eye blinks",
    "blank-blink", '\0', "", "false" };

// Used by: Brain
const ModelOptionDef OPT_BrainTooManyShifts =
  { MODOPT_ARG(int), "BrainTooManyShifts", &MOC_BRAIN, OPTEXP_CORE,
    "Exit after this number of covert attention shifts",
    "too-many-shifts", '\0', "<int>", "0" };

const ModelOptionDef OPT_BrainTooManyShiftsPerFrame =
  { MODOPT_ARG(int), "BrainTooManyShiftsPerFrame", &MOC_BRAIN, OPTEXP_CORE,
    "Wait untill the next frame after this number of covert attention shifts",
    "too-many-shifts-per-frame", '\0', "<int>", "0" };

// Used by: Brain
const ModelOptionDef OPT_TargetMaskFname =
  { MODOPT_ARG_STRING, "TargetMaskFname", &MOC_BRAIN, OPTEXP_CORE,
    "Name of a grayscale image file to be loaded and used as a "
    "targetmask for Brain",
    "target-mask", '\0', "<filename>", "" };





// #################### Saliency Map options:

const ModelOptionCateg MOC_SM = {
  MOC_SORTPRI_3, "Saliency Map Related Options" };

// Used by: SaliencyMapConfigurator
const ModelOptionDef OPT_SaliencyMapType =
  { MODOPT_ARG_STRING, "SaliencyMapType", &MOC_SM, OPTEXP_CORE,
    "Type of Saliency Map to use. 'None' for no saliency map, 'Std' for the "
    "standard saliency map using LeakyIntegrator neurons (takes a lot "
    "of CPU cycles to evolve), 'Trivial' for just a non-evolving copy of the "
    "inputs, 'Fast' for a faster implementation that"
    "just takes a weighted average between current and new saliency "
    "map at each time step.",
    "sm-type", '\0', "<None|Std|StdOptim|Trivial|Fast>", "Std" };

// Used by: SaliencyMap and derivatives
const ModelOptionDef OPT_SMsaveResults =
  { MODOPT_FLAG, "SMsaveResults", &MOC_SM, OPTEXP_SAVE,
    "Save saliency map as an un-normalized float image, which is useful "
    "to compare the absolute saliency values across several images or "
    "several frames of a movie. You may use saliency/matlab/pfmread.m "
    "to read these images into matlab, or saliency/bin/pfmtopgm to convert "
    "them to PGM format.",
    "save-salmap", '\0', "", "false" };

// Used by: SaliencyMap and derivatives
const ModelOptionDef OPT_SMsaveCumResults =
  { MODOPT_FLAG, "SMsaveCumResults", &MOC_SM, OPTEXP_SAVE,
    "Save cumulative saliency map so far as an unnormalized float image.",
    "save-cumsalmap", '\0', "", "false" };

// Used by: SaliencyMap and derivatives
const ModelOptionDef OPT_SMuseSacSuppress =
  { MODOPT_FLAG, "SMuseSacSuppress", &MOC_SM, OPTEXP_CORE,
    "Use saccadic suppression in the saliency map",
    "salmap-sacsupp", '\0', "", "true" };

// Used by: SaliencyMap and derivatives
const ModelOptionDef OPT_SMuseBlinkSuppress =
  { MODOPT_FLAG, "SMuseBlinkSuppress", &MOC_SM, OPTEXP_CORE,
    "Use blink suppression in the saliency map",
    "salmap-blinksupp", '\0', "",
    // NOTE: the default value here is 'false' because historically in
    // Brain we had commented out the line that triggered an eyeBlink
    // in the SaliencyMap; now, that line is re-instated, but we
    // retain the default=off behavior by setting the default value to
    // false here
    "false" };

// Used by: SaliencyMapFast
const ModelOptionDef OPT_SMfastInputCoeff =
  { MODOPT_ARG(float), "SMfastInputCoeff", &MOC_SM, OPTEXP_CORE,
    "Coefficient by which new inputs merge with old ones at each time step",
    "salmap-inputcoeff", '\0', "<0..1>", "0.1" };

// Used by: SaliencyMapFast
const ModelOptionDef OPT_SMItoVcoeff =
  { MODOPT_ARG(float), "SMItoVcoeff", &MOC_SM, OPTEXP_CORE,
    "Coefficient used to convert from synaptic current inputs (typically "
    "in the nA range) to voltage outputs (typically in the mV range)",
    "salmap-itovcoeff", '\0', "<float>", "5000000.0" };

// Used by: SaliencyMapStd
const ModelOptionDef OPT_SMginhDecay =
  { MODOPT_ARG(float), "SMginhDecay", &MOC_SM, OPTEXP_CORE,
    "Coefficient by which inhibition-of-return dies off at each time step",
    "salmap-iordecay", '\0', "<0..1>", "0.9999" };

// Used by: SaliencyMapStd
const ModelOptionDef OPT_SMmaxWinV =
  { MODOPT_ARG(float), "SMmaxWinV", &MOC_SM, OPTEXP_CORE,
    "Winner voltage (in millivolts) which, if exceeded, will trigger global "
    "inhibition over the saliency map. This helps avoiding that the "
    "saliency map explodes if it receives constantly strong inputs.",
    "salmap-maxwinv", '\0', "<mV>", "5.0" };



// #################### Task-Relevance Map options:

const ModelOptionCateg MOC_TRM = {
  MOC_SORTPRI_3, "Task-Relevance Map Related Options" };

// Used by: TaskRelevanceMapConfigurator
const ModelOptionDef OPT_TaskRelevanceMapType =
  { MODOPT_ARG_STRING, "TaskRelevanceMapType", &MOC_TRM, OPTEXP_CORE,
    "Type of Task-Relevance Map to use. 'None' for no map, 'Std' for the "
    "standard TRM that needs an agent to compute the relevance of "
    "objects, 'KillStatic' for a TRM that progressively decreases "
    "the relevance of static objects, and 'KillN' for a TRM that kills "
    "the last N objects that have been passed to it, 'GistClassify' for classify"
    "the current frame into different categories and use the corresponding"
    "pre-defined TD map, 'Tigs' for use the gist vector to get a top down map"
    "Tigs2 use the gist vector combined with the PCA image vector to get a"
    "top down map", "trm-type", '\0',
    "<None|Std|KillStatic|KillN|GistClassify|Tigs|Tigs2>", "None" };

// Used by: TaskRelevanceMap and derivatives
const ModelOptionDef OPT_TRMsaveResults =
  { MODOPT_FLAG, "TRMsaveResults", &MOC_TRM, OPTEXP_SAVE,
    "Save task-relevance map",
    "save-trm", '\0', "", "false" };

// Used by: Scene understanding model
const ModelOptionDef OPT_LearnTRM =
  { MODOPT_FLAG, "LearnTRM", &MOC_TRM, OPTEXP_CORE,
    "Learn TRM over a number of scenes", "learn-trm", '\0', "", "false" };

// Used by: Scene understanding model
const ModelOptionDef OPT_LearnUpdateTRM =
  { MODOPT_FLAG, "LearnUpdateTRM", &MOC_TRM, OPTEXP_CORE,
    "Learn and update TRM over a number of scenes",
    "learn-update-trm", '\0', "", "false" };

// Used by: Brain for the Scene understanding model
const ModelOptionDef OPT_BiasTRM =
  { MODOPT_ARG_STRING, "BiasTRM", &MOC_TRM, OPTEXP_CORE,
    "bias the TRM with the given image", "bias-trm", '\0', "<file>", "" };

// Used by: TaskRelevanceMapKillStatic
const ModelOptionDef OPT_TRMKillStaticThresh =
  { MODOPT_ARG(float), "TRMKillStaticThresh", &MOC_TRM, OPTEXP_CORE,
    "Minimum difference between current luminance and previously cumulated "
    "luminance for a point to be considered non-static",
    "kill-static-thresh", '\0', "<float>", "20.0" };

// Used by: TaskRelevanceMapKillStatic
const ModelOptionDef OPT_TRMKillStaticCoeff =
  { MODOPT_ARG(float), "TRMKillStaticCoeff", &MOC_TRM, OPTEXP_CORE,
    "Killing achieved by a purely static location. "
    "A value of 1.0 here means that all salience of a static location will "
    "be killed; a value of 0 means no killing",
    "kill-static-coeff", '\0', "<0.0 .. 1.0>", "0.95" };

// Used by: TaskRelevanceMapKillN
const ModelOptionDef OPT_TRMkillN =
  { MODOPT_ARG(int), "TRMkillN", &MOC_TRM, OPTEXP_CORE,
    "Number of objects to kill.",
    "killn", '\0', "<int>", "3" };

// Used by: TaskRelevanceMapGistClassify
const  ModelOptionDef OPT_TRMClusterCenterFile =
  { MODOPT_ARG_STRING, "TRMClusterCenterFile",  &MOC_TRM, OPTEXP_CORE,
    "the centroids of different gist clusters are saved in a file, which "
    "first number means the category number, the second number is the gist vector "
    "dims, then the following data are the gist vectors for each of the cluster",
    "cluster-center-file", '\0',"<file>",
    "/lab/tmpib/u/gist-trm/training/cluster_center/cluster_center.dat"
  };

// Used by: TaskRelevanceMapGistClassify
const  ModelOptionDef OPT_TRMTemplateDir =
  { MODOPT_ARG_STRING, "TRMTemplateDir",  &MOC_TRM, OPTEXP_CORE,
    "the template files under the template directiry are adopted "
    "according to the classify result each template file is a "
    "pre-defined TD map, and we can rescale it to the current frame size",
    "Template-dir", '\0',"<dir>",
    "/lab/tmpib/u/gist-trm/training/template/"
  };

// Used by: TaskRelevanceMapGistClassify
const ModelOptionDef OPT_TRMGistPCADims =
  {
    MODOPT_ARG(int),"GistPCADims", &MOC_TRM, OPTEXP_CORE,
    "use PCA Matrix to reduce the gist vector's dim"
    "dims=5 keep 95% variance, dims=4 keep 91.3%, "
    "dims=3 keep 76.8%, dims=2 keep 60%",
    "gist-PCADims", '\0',"<int>", "5"
  };

// Used by: TaskRelevanceMapGistClassify
const ModelOptionDef OPT_TRMGistPCAMatrix =
  {
    MODOPT_ARG_STRING,"GistPCAMatrix", &MOC_TRM, OPTEXP_CORE,
    "use PCA Matrix to reduce the gist vector's dim"
    "specify the PCA matrix file directory",
    "gist-PCAmatrix", '\0',"<dir>",
    "/lab/tmpib/u/gist-trm/training/PCA/PCAMatrix.dat"
  };

// Used by: TaskRelevanceMapGistClassify
const ModelOptionDef OPT_TRMCacheSize =
  {
    MODOPT_ARG(int),"TRMCacheSize", &MOC_TRM,  OPTEXP_CORE,
    "the cache size of the TRM",
    "trm-cacheSize", '\0', "<int>", "5"
  };

// Used by: TaskRelevanceMapGistClassify
const ModelOptionDef OPT_TRMUpdatePeriod =
  {
    MODOPT_ARG(int),"TRMUpdatePeriod", &MOC_TRM,  OPTEXP_CORE,
    "the period update the TRM",
    "trm-updatePeriod", '\0', "<int>",
    "1"
  };

// Used by: TaskRelevanceMapTigs
const ModelOptionDef OPT_TRMTigsMatrix =
  {
    MODOPT_ARG_STRING,"TigsMatrix", &MOC_TRM,  OPTEXP_CORE,
    "the trained Tigs Matrix",
    "tigs-matrix", '\0', "<dir>",
    "/lab/tmpib/u/gist-trm/training/extend/Matlab/TigsMatrix.dat"
  };

// Used by: TaskRelevanceMapTigs2
const ModelOptionDef OPT_TRMImgPCADims =
  {
    MODOPT_ARG(int),"ImagePCADims", &MOC_TRM, OPTEXP_CORE,
    "use PCA Matrix to reduce the image vector's dim",
    "img-PCADims", '\0',"<int>", "20"
  };

// Used by: TaskRelevanceMapTigs2
const ModelOptionDef OPT_TRMImgPCAMatrix =
  {
    MODOPT_ARG_STRING,"ImagePCAMatrix", &MOC_TRM, OPTEXP_CORE,
    "use PCA Matrix to reduce the image vector's dim"
    "specify the PCA matrix file directory",
    "img-PCAmatrix", '\0',"<dir>",
    "/lab/tmpib/u/gist-trm/training/extend/imgPCA/imgPCAMatrix.dat"
  };

// Used by: TaskRelevanceMapTigs2
const ModelOptionDef OPT_TRMTigs2Matrix =
  {
    MODOPT_ARG_STRING,"Tigs2Matrix", &MOC_TRM,  OPTEXP_CORE,
    "the trained Tigs2 Matrix",
    "tigs2-matrix", '\0', "<dir>",
    "/lab/tmpib/u/gist-trm/training/Tigs2/code/Tigs2Matrix.dat"
  };

const ModelOptionDef OPT_TRMSocialRegionFName = 
  { MODOPT_ARG_STRING, "TRMRegionFile", &MOC_TRM, OPTEXP_CORE,
    "XML input file with labeled/defined regions for all frames.",
    "trmsocial-input-file", '\0', "<file>", "" 
  };

// #################### Attention Guidance Map options:

const ModelOptionCateg MOC_AGM = {
  MOC_SORTPRI_3, "Attention Guidance Map Related Options" };

// Used by: AttentionGuidanceMapConfigurator
const ModelOptionDef OPT_AttentionGuidanceMapType =
  { MODOPT_ARG_STRING, "AttentionGuidanceMapType", &MOC_AGM, OPTEXP_CORE,
    "Type of Attention Guidance Map to use. 'None' for no map (in which "
    "case Brain will use the bottom-up saliency map for attention "
    "guidance), 'Std' for the standard AGM that just computes the pointwise "
    "product between bottom-up saliency map and top-down task-relevance "
    "map.  'NF' for maps based on simple neural fields. 'SC' for an AGM "
    "based on the neural architecture and "
    "functionality of the superior colliculus.",
    "agm-type", '\0', "<None|Std|Opt|NF|SC>", "Std" };

// Used by: AttentionGuidanceMap and derivatives
const ModelOptionDef OPT_AGMsaveResults =
  { MODOPT_FLAG, "AGMsaveResults", &MOC_AGM, OPTEXP_SAVE,
    "Save attention guidance map",
    "save-agm", '\0', "", "false" };


// Used by: AttentionGuidanceMapSC and AttentionGuidanceMapNF
const ModelOptionDef OPT_AGMoutputRate =
  { MODOPT_ARG(SimTime), "AGMoutputRate", &MOC_AGM, OPTEXP_SAVE,
    "the rate at which we post our output maps",
    "agm-output-rate", '\0', "<SimTime>", "10ms" };

// #################### Attention Gate options:

const ModelOptionCateg MOC_AG = {
  MOC_SORTPRI_3, "Attention Gate Related Options" };

// Used by: AttentionGateConfigurator
const ModelOptionDef OPT_AttentionGateType =
  { MODOPT_ARG_STRING, "AttentionGateType", &MOC_AG, OPTEXP_CORE,
    "Type of Attention Gate to use. "
    "Std or None",
    "ag-type", '\0', "<Std|None>", "None" };

// Used by: AttentionGate and derivatives
const ModelOptionDef OPT_AttentionGateStageOneType =
  { MODOPT_ARG_STRING, "AttentionGateStageOneType", &MOC_AG, OPTEXP_CORE,
    "Type of stage one Attention Gate to use. "
    "Simple or Complex",
    "ag-so-type", '\0', "<Simple|Complex>", "Simple" };

// Used by: AttentionGate and derivatives
const ModelOptionDef OPT_AttentionGateStageTwoType =
  { MODOPT_ARG_STRING, "AttentionGateStageTwoType", &MOC_AG, OPTEXP_CORE,
    "Type of stage two Attention Gate to use. "
    "Std or None",
    "ag-st-type", '\0', "<Std|None>", "None" };

// Used by: AttentionGate and derivatives
const ModelOptionDef OPT_AttentionGateStageTwoEpochs =
  { MODOPT_ARG(int), "AttentionGateStageTwoEpochs", &MOC_AG, OPTEXP_CORE,
    "How many epochs to look forward and backwards. "
    "An integer such as 5.",
    "ag-st-epochs", '\0', "<int>", "5" };

// Used by: AttentionGate and derivatives
const ModelOptionDef OPT_AGsaveResults =
  { MODOPT_FLAG, "AGsaveResults", &MOC_AG, OPTEXP_SAVE,
    "Save attention gate maps",
    "save-ag", '\0', "", "false" };

// Used by: AttentionGate and derivatives
const ModelOptionDef OPT_AGTargetFrame =
  { MODOPT_ARG(int), "AGTargetFrame", &MOC_AG, OPTEXP_CORE,
    "Which frame contains the target if needed "
    "An integer such as 11.",
    "ag-tframe", '\0', "<int>", "0" };

// Used by: AttentionGate and derivatives
const ModelOptionDef OPT_AGMaskFile =
  { MODOPT_ARG_STRING, "AGMaskFile", &MOC_AG, OPTEXP_CORE,
    "Name of the ground truth mask file to use if any. "
    "Should be a file name string",
    "ag-maskfile", '\0', "<string>", "mask.png" };

// #################### Winner-Take-All options:

const ModelOptionCateg MOC_WTA = {
  MOC_SORTPRI_3, "Winner-Take-All Related Options" };

// Used by: WinnerTakeAllConfigurator
const ModelOptionDef OPT_WinnerTakeAllType =
  { MODOPT_ARG_STRING, "WinnerTakeAllType", &MOC_WTA, OPTEXP_CORE,
    "Type of Winner-Take-All to use. 'None' for no winner-take-all, 'Std' "
    "for the standard winner-take-all using LeakyIntFire neurons "
    "(takes a lot of CPU cycles to evolve), 'Fast' for a faster "
    "implementation that just computes the location of the max at "
    "every time step, 'Greedy' is one that returns, out of a number of "
    "possible targets above a threshold, the one closest to current eye "
    "position. 'Notice' uses an adaptive leaky integrate and fire neuron "
    "that trys to notice things across frames",
    "wta-type", '\0', "<None|Std|StdOptim|Fast|Greedy|Notice>", "Std" };

// Used by: WinnerTakeAll and derivatives
const ModelOptionDef OPT_WTAsaveResults =
  { MODOPT_FLAG, "WTAsaveResults", &MOC_WTA, OPTEXP_SAVE,
    "Save winner-take-all membrane potential values",
    "save-wta", '\0', "", "false" };

// Used by: WinnerTakeAll and derivatives
const ModelOptionDef OPT_WTAuseSacSuppress =
  { MODOPT_FLAG, "WTAuseSacSuppress", &MOC_WTA, OPTEXP_CORE,
    "Use saccadic suppression in the winner-take-all",
    "wta-sacsupp", '\0', "", "true" };

// Used by: WinnerTakeAll and derivatives
const ModelOptionDef OPT_WTAuseBlinkSuppress =
  { MODOPT_FLAG, "WTAuseBlinkSuppress", &MOC_WTA, OPTEXP_CORE,
    "Use blink suppression in the winner-take-all",
    "wta-blinksupp", '\0', "", "true" };

// Used by: WinnerTakeAllGreedy
const ModelOptionDef OPT_WinnerTakeAllGreedyThreshFac =
  { MODOPT_ARG(float), "WinnerTakeAllGreedyThreshFac", &MOC_WTA, OPTEXP_CORE,
    "Threshold, as a factor of the max, over which possible local "
    "maxima will be considered",
    "wta-greedyfac", '\0', "<0.0 .. 1.0>", "0.5" };

// #################### SaccadeController options:

const ModelOptionCateg MOC_SAC = {
  MOC_SORTPRI_3, "Eye/Head Saccade Controller Options" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeInitialPosition =
  { MODOPT_ARG(Point2D<int>), "SCeyeInitialPosition", &MOC_SAC, OPTEXP_CORE,
    "Initial eye position, or (-1,-1) to start as undefined until the first "
    "shift of attention, or (-2,-2) to start at the center of the image",
    "initial-eyepos", '\0', "<x,y>", "-1,-1" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadInitialPosition =
  { MODOPT_ARG(Point2D<int>), "SCheadInitialPosition", &MOC_SAC, OPTEXP_CORE,
    "Initial head position, or (-1,-1) to start as undefined until the first "
    "shift of attention, or (-2,-2) to start at the center of the image",
    "initial-headpos", '\0', "<x,y>", "-1,-1" };

// Used by: TrivialSaccadeController
const ModelOptionDef OPT_SCeyeMinSacLen =
  { MODOPT_ARG(float), "SCeyeMinSacLen", &MOC_SAC, OPTEXP_CORE,
    "Minimum eye movement distance for a saccade (in pixels)",
    "eye-minsac", '\0', "<float>", "10.0" };

// Used by: TrivialSaccadeController
const ModelOptionDef OPT_SCheadMinSacLen =
  { MODOPT_ARG(float), "SCheadMinSacLen", &MOC_SAC, OPTEXP_CORE,
    "Minimum head movement distance for a saccade (in pixels)",
    "head-minsac", '\0', "<float>", "10.0" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeSpringK =
  { MODOPT_ARG(double), "SCeyeSpringK", &MOC_SAC, OPTEXP_CORE,
    "Eye spring stiffness to use in mass/spring simulations",
    "eye-spring-k", '\0', "<float>", "50000.0" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeFrictionMu =
  { MODOPT_ARG(double), "SCeyeFrictionMu", &MOC_SAC, OPTEXP_CORE,
    "Eye friction coefficient to use in mass/spring simulations",
    "eye-friction-mu", '\0', "<float>", "500.0" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeThreshMinOvert =
  { MODOPT_ARG(float), "SCeyeThreshMinOvert", &MOC_SAC, OPTEXP_CORE,
    "Minimum overt distance for a saccade (as factor of fovea radius)",
    "eye-minovert-fac", '\0', "<float>", "2.0" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeThreshMaxCovert =
  { MODOPT_ARG(float), "SCeyeThreshMaxCovert", &MOC_SAC, OPTEXP_CORE,
    "Maximum covert distance for a saccade (as factor of fovea radius)",
    "eye-maxcovert-fac", '\0', "<float>", "1.5" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeThreshMinNum =
  { MODOPT_ARG(int), "SCeyeThreshMinNum", &MOC_SAC, OPTEXP_CORE,
    "Minimum required number of covert shifts within max covert distance",
    "eye-mincovertnum", '\0', "<int>", "4" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeThreshSalWeigh =
  { MODOPT_FLAG, "SCeyeThreshSalWeigh", &MOC_SAC, OPTEXP_CORE,
    "Weight covert positions by saliency when computing average distance",
    "eye-saliencyweight", '\0', "<true|false>", "true" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeMaxIdleSecs =
  { MODOPT_ARG(SimTime), "SCeyeMaxIdleSecs", &MOC_SAC, OPTEXP_CORE,
    "Maximum idle time (in s) before eyes return to initial eye position",
    "eye-maxidle-time", '\0', "<float>", "0.75" };


// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadSpringK =
  { MODOPT_ARG(double), "SCheadSpringK", &MOC_SAC, OPTEXP_CORE,
    "Head spring stiffness to use in mass/spring simulations",
    "head-spring-k", '\0', "<float>", "10000.0" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadFrictionMu =
  { MODOPT_ARG(double), "SCheadFrictionMu", &MOC_SAC, OPTEXP_CORE,
    "Head friction coefficient to use in mass/spring simulations",
    "head-friction-mu", '\0', "<float>", "5000.0" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadThreshMinOvert =
  { MODOPT_ARG(float), "SCheadThreshMinOvert", &MOC_SAC, OPTEXP_CORE,
    "Minimum overt distance for a head saccade (as factor of fovea radius)",
    "head-minovert-fac", '\0', "<float>", "2.0" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadThreshMaxCovert =
  { MODOPT_ARG(float), "SCheadThreshMaxCovert", &MOC_SAC, OPTEXP_CORE,
    "Maximum covert distance for a head saccade (as factor of fovea radius)",
    "head-maxcovert-fac", '\0', "<float>", "1.5" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadThreshMinNum =
  { MODOPT_ARG(int), "SCheadThreshMinNum", &MOC_SAC, OPTEXP_CORE,
    "Minimum required number of covert shifts within max covert distance",
    "head-mincovertnum", '\0', "<int>", "4" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadThreshSalWeigh =
  { MODOPT_FLAG, "SCheadThreshSalWeigh", &MOC_SAC, OPTEXP_CORE,
    "Weight covert positions by saliency when computing average distance",
    "head-saliencyweight", '\0', "<true|false>", "true" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCheadMaxIdleSecs =
  { MODOPT_ARG(SimTime), "SCheadMaxIdleSecs", &MOC_SAC, OPTEXP_CORE,
    "Maximum idle time (in s) before head return to initial eye position",
    "head-maxidle-time", '\0', "<float>", "0.75" };


// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeBlinkWaitTime =
  { MODOPT_ARG(SimTime), "SCeyeBlinkWaitTime", &MOC_SAC, OPTEXP_CORE,
    "Typical wait time between eye blinks (in s)",
    "blink-wait", '\0', "<float>", "1.8" };

// Used by: SaccadeControllers
const ModelOptionDef OPT_SCeyeBlinkDuration =
  { MODOPT_ARG(SimTime), "SCeyeBlinkDuration", &MOC_SAC, OPTEXP_CORE,
    "Eye blink duration (in s)",
    "blink-duration", '\0', "<float>", "0.190" };

// Used by: EyeTrackerEyeHeadController
const ModelOptionDef OPT_EHCeyeTrackConfig =
  { MODOPT_ARG_STRING, "EHCeyeTrackConfig", &MOC_SAC, OPTEXP_CORE,
    "Config string to determine which eye-tracking files to use for "
    "comparison with model predictions. This should be a "
    "comma-separated list of eyetrace filenames.",
    "eyetrack-data", '\0',
    "<filename,filename,...>", "" };

// Used by: EyeHeadControllerConfigurator
const ModelOptionDef OPT_EyeHeadControllerType =
  { MODOPT_ARG_STRING, "EyeHeadControllerType", &MOC_SAC, OPTEXP_CORE,
    "Eye/Head Controller name (pick a name and then use --help to see "
    "options specific to the chosen controller)",
    "ehc-type", '\0',
    "<None|Simple|EyeTrack|Monkey>", "None" };

// Used by: SaccadeControllerConfigurator
const ModelOptionDef OPT_SaccadeControllerEyeType =
  { MODOPT_ARG_STRING, "SaccadeControllerEyeType", &MOC_SAC, OPTEXP_CORE,
    "Eye Saccade Controller name (pick a name and then use --help to see "
    "options specific to the chosen controller)",
    "esc-type", '\0',
    "<None|Trivial|Fixed|Friction|Threshold|Threshfric>", "None" };

// Used by: SaccadeControllerConfigurator
const ModelOptionDef OPT_SaccadeControllerHeadType =
  { MODOPT_ARG_STRING, "SaccadeControllerHeadType", &MOC_SAC, OPTEXP_CORE,
    "Head Saccade Controller name (pick a name and then use --help to see "
    "options specific to the chosen controller)",
    "hsc-type", '\0',
    "<None|Trivial|Fixed|Friction|Threshold|Threshfric>", "None" };

// #################### HandController options:

const ModelOptionCateg MOC_HAND = {
  MOC_SORTPRI_3, "Hand Controller Options" };

// Used by: HandControllerConfigurator
const ModelOptionDef OPT_HandControllerType =
  { MODOPT_ARG_STRING, "HandControllerType", &MOC_HAND, OPTEXP_CORE,
    "Hand Controller name (pick a name and then use --help to see "
    "options specific to the chosen controller)",
    "hand-type", '\0',
    "<None|HandTrack>", "None" };

// Used by: HandController
const ModelOptionDef OPT_HandConfig =
  { MODOPT_ARG_STRING, "HandConfig", &MOC_HAND, OPTEXP_CORE,
    "Config string to determine which hand-tracking files to use for "
    "comparison with model predictions. This should be a "
    "comma-separated list of handtrace filenames.",
    "handtrack-data", '\0',
    "<filename,filename,...>", "" };



// #################### GistEstimator options:

const ModelOptionCateg MOC_GE = {
  MOC_SORTPRI_3, "Gist Estimator Options" };

// Used by: SaccadeControllerConfigurator
const ModelOptionDef OPT_GistEstimatorType =
  { MODOPT_ARG_STRING, "GistEstimatorType", &MOC_GE, OPTEXP_CORE,
    "Type of gist estimator to use",
    "ge-type", '\0',
    "<None|Std|FFT|Texton|CB|BBoF|SurfPMK|Gen>", "None" };

// Used by: GistEstimatorGen
const ModelOptionDef OPT_GistCenterSurroundFlag =
  { MODOPT_ARG(int), "GistEstimatorCenterSurroundFlag", &MOC_GE, OPTEXP_CORE,
    "use center surround to compute the gist or not"
    "0: no center-surround, 1: only use center-surround"
    "2: use both center-surround and non center-surround",
    "gist-cs", '\0',"<0|1|2>", "1" };

// Used by: GistSaveAdapter
const ModelOptionDef OPT_SaveGistFlag =
  { MODOPT_FLAG, "SaveGistFlag", &MOC_GE, OPTEXP_CORE,
    "save the gist to disk or not",
    "save-gist", '\0',"", "false" };



// #################### VisualBuffer options:

const ModelOptionCateg MOC_VB = {
  MOC_SORTPRI_3, "Visual Buffer Related Options" };

// Used by: VisualBufferConfigurator
const ModelOptionDef OPT_VisualBufferType =
  { MODOPT_ARG_STRING, "VisualBufferType", &MOC_VB, OPTEXP_CORE,
    "Type of Visual Buffer top use. 'Stub' does nothing, and 'Std' is a "
    "world-centered cumulative buffer that can be updated either "
    "continually or at every attention shift and can make prediuctions "
    "about next saccade target in world coordinates.",
    "vb-type", '\0', "<Stub|Std>", "Stub" };

// Used by: VisualBuffer
const ModelOptionDef OPT_VBignoreBoring =
  { MODOPT_FLAG, "VBignoreBoring", &MOC_VB, OPTEXP_CORE,
    "Ignore boring shifts of attention if true",
    "vb-ignore-boring", '\0', "", "true" };

// Used by: VisualBuffer
const ModelOptionDef OPT_VBobjectBased =
  { MODOPT_FLAG, "VBobjectBased", &MOC_VB, OPTEXP_CORE,
    "Use object-based saliency imprinting if true",
    "vb-object-based", '\0', "", "true" };

// Used by: VisualBuffer
const ModelOptionDef OPT_VBdims =
  { MODOPT_ARG(Dims), "VBdims", &MOC_VB, OPTEXP_CORE,
    "Use given dims for visual buffer (at saliency map scale), or use same "
    "dims as the saliency map if set to (0, 0)",
    "vb-dims", '\0', "<w>x<h>", "0x0" };

// Used by: VisualBuffer
const ModelOptionDef OPT_VBtimePeriod =
  { MODOPT_ARG(SimTime), "VBtimePeriod", &MOC_VB, OPTEXP_CORE,
    "Apply internal competitive interactions within the buffer at "
    "intervals given by this parameter (in s).",
    "vb-period", '\0', "<double>", "0.05" };

// Used by: VisualBuffer
const ModelOptionDef OPT_VBdecayFactor =
  { MODOPT_ARG(float), "VBdecayFactor", &MOC_VB, OPTEXP_CORE,
    "Multiply buffer by <factor> at each time period, if not 1.0",
    "vb-decay", '\0', "<factor>", "1.0" };

// Used by: VisualBuffer
const ModelOptionDef OPT_VBmaxNormType =
  { MODOPT_ARG(MaxNormType), "VBmaxNormType", &MOC_VB, OPTEXP_CORE,
    "Type of MaxNormalization to use",
    "vb-maxnorm-type", '\0', "<see --maxnorm-type>", "Fancy" };

// #################### Other:
// Used by: SpatialMetrics, SaccadeControllers
const ModelOptionDef OPT_PixelsPerDegree =
  { MODOPT_ARG(PixPerDeg), "PixelsPerDegree", &MOC_BRAIN, OPTEXP_CORE,
    "Pixels per degree of visual angle in the horizontal direction and vertical direction"
    "if only one value is desired set the second to 0.0 (default).",
    "pixperdeg", '\0', "<float,float>", "5.333333333,0.0" };

// Used by: SimulationViewer
const ModelOptionDef OPT_HeadMarkerRadius =
  { MODOPT_ARG(int), "HeadMarkerRadius", &MOC_BRAIN, OPTEXP_CORE,
    "Radius of the head position marker",
    "head-radius", '\0', "<pixels>", "50" };

// Used by: SimulationViewer
const ModelOptionDef OPT_MultiRetinaDepth =
  { MODOPT_ARG(int), "MultiRetinaDepth", &MOC_BRAIN, OPTEXP_CORE,
    "Depth of pyramid used for foveation",
    "multiretina-depth", '\0', "<int>", "5" };

// #################### Option Aliases and Shortcuts
// These can be requested using ModelManager::requestOptionAlias().
// See in ModelOptionAliases.H some macros that will request groups
// of aliases for various models (e.g., all aliases applicable to
// attention models, etc)

const ModelOptionDef OPT_ALIASinitialVCO =
  { MODOPT_ALIAS, "ALIASinitialVCO", &MOC_ALIAS, OPTEXP_CORE,
    "Just save the initial saliency map (i.e., output from the VisualCortex) "
    "before any shift of attention, and exit. The floating-point map will "
    "be saved in our proprietary PFM image format and with prefix 'VCO', "
    "which is harder to visualize but good for comparisons across images "
    "as it contains absolute saliency values. See saliency/matlab/"
    "pfmreadmatlab.m for a simple function to read a PFM image into Matlab. "
    "Note that the values in the map are typically very small, as they "
    "are in Amperes of synaptic current flowing into the integrate-and-fire "
    "neurons of the dynamic saliency map. Typical values are in the nA "
    "range (1.0e-9 Amps). You can also use saliency/bin/pfmtopgm to convert "
    "the map to a PGM image.",
    "just-initial-saliency-map", '\0', "",
    "--nouse-fpe --nouse-random --save-vcx-output --out=raster: "
    "--out=pfm: --too-much-time=0.1ms --output-frames=0-0@0.1ms " };


const ModelOptionDef OPT_ALIAStop5 =
  { MODOPT_ALIAS, "ALIAStop5", &MOC_ALIAS, OPTEXP_CORE,
    "Compute the saliency map and save the first five attended locations. "
    "This will save five images, T000000.pnm to T000004.pnm which show the "
    "trajectory taken by the first five shifts of the focus of attention, "
    "as well as a text file 'top5.txt' with the coordinates of the attended "
    "locations as well as other information.",
    "top5", '\0', "",
    "--nouse-fpe --nouse-random -T --textlog=top5.txt --too-many-shifts=5 "
    "--output-frames=0-4@EVENT --out=raster: -+" };


const ModelOptionDef OPT_ALIASmovie =
  { MODOPT_ALIAS, "ALIASmovie", &MOC_ALIAS, OPTEXP_CORE,
    "Process in movie (multiframe) mode",
    "movie", '\0', "",
    "--nodisplay-traj --nodisplay-additive --nouse-fpe "
    "--display-map-factor=50000 --display-map=AGM "
    "--nouse-random --nodisplay-foa --display-patch" };


const ModelOptionDef OPT_ALIASmovieFast =
  { MODOPT_ALIAS, "ALIASmovieFast", &MOC_ALIAS, OPTEXP_CORE,
    "Process in movie (multiframe) mode, making some approximations in the "
    "computations to make processing fast albeit slightly different from the "
    "gold standard. You still need to provide:\n"
    "   --in=<movie>    input movie\n"
    "   --out=display   or any other output\n"
    "   -j <n>          for the number of parallel threads (default 4)\n"
    "   -T              or similar option (like -X, -Y, -K) for what to display.",
    "moviefast", '\0', "",
    "--movie --display-foa --sm-type=Fast --wta-type=Fast --nodisplay-interp-maps "
    "--ior-type=None --vc-type=Thread:Std --vc-chans=CIOFM -j 4 --timestep=30ms --maxnorm-type=FancyWeak "
    "--input-frames=0-MAX@30ms --output-frames=0-MAX@30ms --logverb=Error" };


const ModelOptionDef OPT_ALIASmoviefov =
  { MODOPT_ALIAS, "ALIASmoviefov", &MOC_ALIAS, OPTEXP_CORE,
    "Process in foveated movie (multiframe) mode with a ThreshFric "
    "SaccadeController",
    "moviefov", '\0', "",
    "--movie --foveate-input-depth=6 --ehc-type=Simple "
    "--esc-type=Threshfric --ior-type=None --trm-type=KillStatic" };


const ModelOptionDef OPT_ALIASmovieeyehead =
  { MODOPT_ALIAS, "ALIASmovieeyehead", &MOC_ALIAS, OPTEXP_CORE,
    "Process in foveated movie (multiframe) mode with a Monkey2 "
    "Eye/Head SaccadeController and displays of head position",
    "movieeyehead", '\0', "",
    "--moviefov --sc-type=Monkey2 --display-head" };


const ModelOptionDef OPT_ALIASmovieanim =
  { MODOPT_ALIAS, "ALIASmovieanim", &MOC_ALIAS, OPTEXP_CORE,
    "Process in foveated movie (multiframe) mode with a human-derived "
    "ModelW Eye/Head SaccadeController, a WinnerTakeAllGreedy, no IOR, "
    "and displays of head position",
    "movieanim", '\0', "",
    "--movie --foveate-input-depth=4 --sc-type=ModelW "
    "--ior-type=None --display-head --shape-estim-mode=None "
    "--wta-type=Greedy --initial-eyepos=-2,-2 --initial-headpos=-2,-2" };


const ModelOptionDef OPT_ALIASsurprise =
  { MODOPT_ALIAS, "ALIASsurprise", &MOC_ALIAS, OPTEXP_CORE,
    "Use standard surprise mode",
    "surprise", '\0', "",
    "--vc-type=Surp --vc-chans=CFIOM --maxnorm-type=Surprise "
    "--gabor-intens=20.0 --direction-sqrt " // 0..255 range every channel
    "--display-map=VCO --display-map-factor=1e11 " // use raw VCX outputs
    "--vcx-outfac=5.0e-9" };

//! Current optimal settings for RSVP sequences
/*! See matlab/linear-classifier/linear_model.m
    Error = 0.2179999999999999993338661852249060757458209991455078125000000000
    X = [ 0.8773092344262305442015303924563340842723846435546875000000000000 ...
          0.3127783654225734788489887705509318038821220397949218750000000000 ...
          1.5660499968021082128899479357642121613025665283203125000000000000 ];
*/
const ModelOptionDef OPT_ALIASsurpriseRSVP =
  { MODOPT_ALIAS, "ALIASsurpriseRSVP", &MOC_ALIAS, OPTEXP_CORE,
    "Use RSVP optimized surprise mode",
    "surpriseRSVP", '\0', "",
    "--vc-type=Surp --vc-chans=I:0.312778365O:1.566049997H:0.877309234 --maxnorm-type=Surprise "
    "--gabor-intens=20.0 --direction-sqrt " // 0..255 range every channel
    "--display-map=VCO --display-map-factor=1e11 " // use raw VCX outputs
    "--vcx-outfac=5.0e-9"
    "--ior-type=None --nouse-random --display-foa"
    "--sm-type=Trivial --wta-type=None"
    "--salmap-factor=1e12 --agm-factor=1e12" };

const ModelOptionDef OPT_ALIASeyeCompare =
  { MODOPT_ALIAS, "ALIASeyeCompare", &MOC_ALIAS, OPTEXP_CORE,
    "Use standard mode for comparison between the model's output and "
    "a human or monkey (or other) eye movement trace. Do not forget to set "
    "at least the following depending on your specific experiment and data:\n"
    "  --foa-radius=<radius>        depending on size of display;\n"
    "  --eyetrack-data=<file, ...>  your .eyeS eye movement record file(s);\n"
    "  --svem-out-fname=<file>      where the results will be saved;\n"
    "  --svem-prior-distro=<file>   for non-uniform random sampling. \n"
    "And very carefully check for other settings as well",
    "eyecompare", '\0', "",
    "--sv-type=EyeMvt --save-eyemvt-megacombo "
    "--wta-type=Std --ior-type=None --shape-estim-mode=None "
    "--maxcache-size=0 --nowta-sacsupp --nowta-blinksupp "
    "--nosalmap-sacsupp --nosalmap-blinksupp" };


const ModelOptionDef OPT_ALIASeyeDisplay =
  { MODOPT_ALIAS, "ALIASeyeDisplay", &MOC_ALIAS, OPTEXP_CORE,
    "Display one or more eye-tracking traces on top of a movie. This should "
    "be used with the following options:\n"
    "  --foa-radius=<radius>        depending on size of display;\n"
    "  --eyetrack-data=<file, ...>  your .eyeS eye movement record file(s);\n"
    "And very carefully check for other settings as well",
    "eyedisplay", '\0', "",
    "--sv-type=EyeMvt -T --vc-type=None --sm-type=None --trm-type=None "
    "--wta-type=None --shape-estim-mode=None --fovea-radius=32 "
    "--maxcache-size=0 --delaycache-size=0" };


const ModelOptionDef OPT_ALIASeyeMap =
  { MODOPT_ALIAS, "ALIASeyeMap", &MOC_ALIAS, OPTEXP_CORE,
    "Build a cumulative eye movement density map from one or more eye "
    "movement traces. This is useful if you have one or more observers that "
    "saw a same stimulus and you want to build a spatial probability density "
    "function from the eye traces. The resulting eye map can be compared, "
    "for example, with saliency maps or can be sampled in specific target "
    "regions. This should be used used with the following options:\n"

    "  --in=<image>                  input image, for dims only;\n"
    "  --vcem-eyefnames=<file, ...>  your .eyeS eye movement record file(s);\n"
    "  --output-frames=0-0@XXms      set XX to the time when to save;\n"
    "And very carefully check for other settings as well",
    "eyemap", '\0', "",
    "--nouse-fpe --nouse-random --save-vcx-output --out=raster: -+ "
    "--vc-type=EyeMvt --sm-type=None --trm-type=None --wta-type=None "
    "--shape-estim-mode=None --sv-type=None --vcem-forgetfac=1.0 "
    "--vcem-delay=0 --vcem-sigma=0 --vcem-saconly=true --vcem-usemax=true "
    "--output-format=PFM" };

const ModelOptionDef OPT_ALIASeyeHandDisplay =
  { MODOPT_ALIAS, "ALIASeyeHandDisplay", &MOC_ALIAS, OPTEXP_CORE,
    "Display one or more eye-tracking and hand movement traces on top of "
    "a movie. This should be used with the following options:\n"
    "  --foa-radius=<radius>        depending on size of display;\n"
    "  --eyetrack-data=<file, ...>  your .eyeS eye movement record file(s);\n"
    "  --handtrack-data=<file, ...> your .hanD hand movement record file(s);\n"
    "And very carefully check for other settings as well",
    "eyehand-display", '\0', "",
    "--sv-type=EyeHand -T --vc-type=None --sm-type=None --trm-type=None "
    "--wta-type=None --shape-estim-mode=None --fovea-radius=32 "
    "--maxcache-size=0 --delaycache-size=0" };


const ModelOptionDef OPT_ALIASbestMultiSpectralResidual =
  { MODOPT_ALIAS, "ALIASbestMultiSpectralResidual", &MOC_CHANNEL, OPTEXP_CORE,
    "Use a tuned multi-spectral residual model",
    "best-multi-srs", '\0', "",
    "--vc-type=MultiSpectralResidual --maxnorm-type=Ignore "
    "--noenable-pyramid-caches --multi-srs-sizes=/16,/8,/4,/2 "
    "--srs-output-blur-factor=0.055 --srs-spectral-blur=1 "
    "--srs-attenuation-width=0.0625 --map-combine-type=Max "
    "--srs-lowpass-filter-width=2 " };

const ModelOptionDef OPT_ALIASoptiGainTrain =
  { MODOPT_ALIAS, "ALIASoptiGainTrain", &MOC_ALIAS, OPTEXP_CORE,
    "Extract salience information to train the optimal gain biasing model of "
    "Navalpakkam and Itti (IEEE-CVPR 2006; Neuron 2007). This will "
    "essentially select multi-band features and setup the training process. "
    "You will then also need to provide:\n"
    "  --in=xmlfile:file.xml   input image and region of interest metadata;\n"
    "  --stsd-filename=file.pmap  to save saliency of target/distractors;\n"
    "The resulting sT/sD pmap files can be combined together using "
    "app-combineOptimalGains to yield a gains file that can be used with "
    "the --guided-search option",
    "train-optigains", '\0', "",
    "--vc-type=Std --vc-chans=GNO --nouse-older-version --out=display --pfc-type=OG "
    "--num-colorband=6 --num-satband=6 --num-intensityband=6 --num-orient=8"
  };

const ModelOptionDef OPT_ALIASoptiGainBias =
  { MODOPT_ALIAS, "ALIASoptiGainBias", &MOC_ALIAS, OPTEXP_CORE,
    "Use a set of gains to bias salience computation in a manner similar "
    "to Jeremy Wolfe's Guided Search (Wolfe, 1994). The gains are read "
    "from a ParamMap file which can be obtained, for example, by using "
    "--train-optigains. You will need to set the following:\n"
    "  --in=image.png              input image to process;\n"
    "  --gains-filename=file.pmap  file to load the gains from",
    "guided-search", '\0', "",
    "--vc-type=Std --vc-chans=GNO --nouse-older-version -K --out=display --pfc-type=GS "
    "--num-colorband=6 --num-satband=6 --num-intensityband=6 --num-orient=8"
  };

const ModelOptionDef OPT_ALIASkinectDemo =
  { MODOPT_ALIAS, "ALIASkinectDemo", &MOC_ALIAS, OPTEXP_CORE,
    "Demonstrate the Kinect sensor by computing saliency over a combination of RGB and Depth images.",
    "kinect-demo", '\0', "",
    "-K --in=kinect --out=display --maxnorm-type=Maxnorm --retina-type=Stub --nodisplay-interp-maps "
    "--ior-type=None --vc-type=Thread:Std -j 6 --vc-chans=YCIOFM"
  };

void REQUEST_OPTIONALIAS_NEURO(OptionManager& m)
{
  REQUEST_OPTIONALIAS_CHANNEL(m);

  m.requestOptionAlias(&OPT_ALIAStop5);
  m.requestOptionAlias(&OPT_ALIASinitialVCO);
  m.requestOptionAlias(&OPT_ALIASmovie);
  m.requestOptionAlias(&OPT_ALIASmovieFast);
  m.requestOptionAlias(&OPT_ALIASmoviefov);
  m.requestOptionAlias(&OPT_ALIASmovieeyehead);
  m.requestOptionAlias(&OPT_ALIASmovieanim);
  m.requestOptionAlias(&OPT_ALIASsurprise);
  m.requestOptionAlias(&OPT_ALIASsurpriseRSVP);
  m.requestOptionAlias(&OPT_ALIASeyeCompare);
  m.requestOptionAlias(&OPT_ALIASeyeDisplay);
  m.requestOptionAlias(&OPT_ALIASeyeMap);
  m.requestOptionAlias(&OPT_ALIASeyeHandDisplay);
  m.requestOptionAlias(&OPT_ALIASbestMultiSpectralResidual);
  m.requestOptionAlias(&OPT_ALIASoptiGainTrain);
  m.requestOptionAlias(&OPT_ALIASoptiGainBias);
  m.requestOptionAlias(&OPT_ALIASkinectDemo);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_NEUROOPTS_C_DEFINED
