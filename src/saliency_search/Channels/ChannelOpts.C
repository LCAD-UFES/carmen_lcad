/*!@file Channels/ChannelOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelOpts.C $
// $Id: ChannelOpts.C 15098 2011-11-28 23:48:42Z dberg $
//

#ifndef CHANNELS_CHANNELOPTS_C_DEFINED
#define CHANNELS_CHANNELOPTS_C_DEFINED

#include "Channels/ChannelOpts.H"

#include "Channels/ColorComputeTypes.H"
#include "Channels/IntegerDecodeType.H"
#include "Channels/MapCombineType.H"
#include "Channels/OrientComputeTypes.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/LevelSpec.H"
#include "Image/fancynorm.H"
#include "Image/Dims.H"

const ModelOptionCateg MOC_CHANNEL = {
  MOC_SORTPRI_3, "Channel-Related Options" };

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

// #################### Channel options:
// Used by: VisualCortex, ComplexChannel, SingleChannel (and derivatives)
const ModelOptionDef OPT_UseOlderVersion =
  { MODOPT_FLAG, "UseOlderVersion", &MOC_CHANNEL, OPTEXP_CORE,
    "Use the older version where we normalize responses within all feature "
    "types",
    "use-older-version", '\0', "", "true" };

// Used by: ChannelBase (and derivatives)
const ModelOptionDef OPT_ChannelOutputRangeMax =
  { MODOPT_ARG(float), "ChannelOutputRangeMax", &MOC_CHANNEL, OPTEXP_CORE,
    "Max of the channel's output range",
    "chanout-max", '\0', "<float>", "10.0" };

// Used by: ChannelBase (and derivatives)
const ModelOptionDef OPT_ChannelOutputRangeMin =
  { MODOPT_ARG(float), "ChannelOutputRangeMin", &MOC_CHANNEL, OPTEXP_CORE,
    "Min of the channel's output range",
    "chanout-min", '\0', "<float>", "0.0" };

// Used by: DirectFeedChannel
const ModelOptionDef OPT_MapLevel =
  { MODOPT_ARG(unsigned int), "MapLevel", &MOC_CHANNEL, OPTEXP_CORE,
    "The level of the saliency map in Channels that don't need LevelSpec",
    "maplevel", '\0', "<uint>", "4" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_LevelSpec =
  { MODOPT_ARG(LevelSpec), "LevelSpec", &MOC_CHANNEL, OPTEXP_CORE,
    "LevelSpec to use in channels. This controls the range of "
    "spatial scales used in constructing center-surround maps, and "
    "also controls the scale of the channel output maps. cmin and "
    "cmax are the lowest (largest) and highest (largest) pyramid "
    "levels to be used for the center scale in center-surround "
    "operations. A level of 0 is the bottom pyramid level at the "
    "original dimensions of the input image; each level above 0 "
    "is reduced by a factor of 2 in the x and y dimensions, so e.g. "
    "level 4 is 16-fold reduced in x and y. delmin and delmax "
    "represent the range of differences at which the surround level "
    "is offset from the center level. maplev is the scale at which "
    "channel output should be generated. For example, the default "
    "setting of 2,4,3,4,4 will use center scales 2-4 with deltas "
    "of 3 and 4, for six center/surround pairs of 2/5, 2/6, 3/6, 3/7, "
    "4/7 and 4/8, and the channel output will be at scale 4.",
    "levelspec", '\0', "<cmin,cmax,delmin,delmax,maplev>", "2,4,3,4,4" };

// Used by: SingleChannel, ComplexChannel (and derivatives), VisualCortex
const ModelOptionDef OPT_MaxNormType =
  { MODOPT_ARG(MaxNormType), "MaxNormType", &MOC_CHANNEL, OPTEXP_CORE,
    "Type of MaxNormalization to use",
    "maxnorm-type", '\0', "<None|Maxnorm|Fancy|FancyFast|FancyOne|"
    "FancyLandmark|Landmark|FancyWeak|Ignore|Surprise>", "Fancy" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_SingleChannelQueueLen =
  { MODOPT_ARG(int), "SingleChannelQueueLen", &MOC_CHANNEL, OPTEXP_CORE,
    "Queue length for channels",
    "qlen", '\0', "<int>", "1" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_SingleChannelTimeDecay =
  { MODOPT_ARG(double), "SingleChannelTimeDecay", &MOC_CHANNEL, OPTEXP_CORE,
    "Time decay for channel queues",
    "qtime-decay", '\0', "<float>", "20.0" };

// Used by: OrientationChannel
const ModelOptionDef OPT_NumOrientations =
  { MODOPT_ARG(unsigned int), "NumOrientations", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of oriented channels",
    "num-orient", '\0', "<int>", "4" };

// Used by: OrientationChannel
const ModelOptionDef OPT_OriInteraction =
  { MODOPT_ARG_STRING, "OriInteraction", &MOC_CHANNEL, OPTEXP_CORE,
    "This describes the way in which the different orientations "
    "within the OrientationChannel interact with each other. "
    "'None' is for no interaction (default); for 'SubtractMean', "
    "the mean of all orientation pyramids is subtracted from each "
    "orientation pyramid. You can determine your own interaction by "
    "specifying a vector with interaction coefficients: 'c1,...,cn'"
    " where n is the number of orientations. c1 is the coeffiecient "
    "for an orientation itself, c2 for the one with the next higher "
    "angle and so on. For instance, for n=4, 'None' is the same as "
    "'1,0,0,0', and 'SubtractMean' is the same as "
    "'0.75,-0.25,-0.25,-0.25'",
    "ori-interaction", '\0', "<None|SubtractMean|c1,...,cn>",
    "None" };

// Used by: GaborChannel
const ModelOptionDef OPT_OrientComputeType =
  { MODOPT_ARG(OrientComputeType), "OrientComputeType", &MOC_CHANNEL, OPTEXP_CORE,
    "Type of computation used to compute orientations",
    "oricomp-type", '\0', "<Steerable|Gabor|GaborEnergyNorm>",
    "Steerable" };

// Used by: GaborChannel
const ModelOptionDef OPT_GaborChannelIntensity =
  { MODOPT_ARG(double), "GaborChannelIntensity", &MOC_CHANNEL, OPTEXP_CORE,
    "Intensity coefficient for Gabor channel",
    "gabor-intens", '\0', "<double>", "10.0" };

// Used by: GaborChannel
const ModelOptionDef OPT_UseTrigTab =
  { MODOPT_FLAG, "UseTrigTab", &MOC_CHANNEL, OPTEXP_CORE,
    "Whether to accelerate trig operations by using table lookup, at the "
    "expense of some loss of numerical precision",
    "use-trig-tab", '\0', "", "false" };

// Used by: DirectionChannel
const ModelOptionDef OPT_DirectionChannelTakeSqrt =
  { MODOPT_FLAG, "DirectionChannelTakeSqrt", &MOC_CHANNEL, OPTEXP_CORE,
    "Take square root of our Reichardt output if true",
    "direction-sqrt", '\0', "", "false" };

// Used by: DirectionChannel
const ModelOptionDef OPT_DirectionChannelLowThresh =
  { MODOPT_ARG(float), "DirectionChannelLowThresh", &MOC_CHANNEL, OPTEXP_CORE,
    "Low threshold to allpy to eliminate small motion responses",
    "direction-lowthresh", '\0', "<float>", "3.0" };

// Used by: MultiColorBandChannel
const ModelOptionDef OPT_NumColorBands =
  { MODOPT_ARG(unsigned int), "NumColorBands", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of hue-band channels",
    "num-colorband", '\0', "<int>", "3" };

// Used by: MultiColorBandChannel
const ModelOptionDef OPT_NumSatBands =
  { MODOPT_ARG(unsigned int), "NumSatBands", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of saturation-band channels",
    "num-satband", '\0', "<int>", "3" };

// Used by: MultiColorBandChannel
const ModelOptionDef OPT_HueBandWidth =
  { MODOPT_ARG(float), "HueBandWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "Width of each hue band",
    "sigma-hueband", '\0', "<float>", "30" };

// Used by: MultiColorBandChannel
const ModelOptionDef OPT_SatBandWidth =
  { MODOPT_ARG(float), "SatBandWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "Width of each saturation band",
    "sigma-satband", '\0', "<float>", "0.3" };

// Used by: IntensityBandChannel
const ModelOptionDef OPT_NumIntensityBands =
  { MODOPT_ARG(unsigned int), "NumIntensityBands", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of intensity-band channels",
    "num-intensityband", '\0', "<int>", "3" };

// Used by:IntensityBandChannel
const ModelOptionDef OPT_IntensityBandWidth =
  { MODOPT_ARG(float), "IntensityBandWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "Width of each intensity band",
    "sigma-intband", '\0', "<float>", "22" };

// Used by: MotionChannel
const ModelOptionDef OPT_NumDirections =
  { MODOPT_ARG(unsigned int), "NumDirections", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of direction-selective motion channels",
    "num-directions", '\0', "<int>", "4" };

// Used by: MotionSpatioTemporalChannel
const ModelOptionDef OPT_NumSpatioTemporalDirections =
  { MODOPT_ARG(unsigned int), "NumSpatioTemporalDirections", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of directions for spatiotemporal motion channels",
    "num-spatiotemporaldirections", '\0', "<int>", "8" };

// Used by: MotionSpatioTemporalChannel
const ModelOptionDef OPT_NumSpatioTemporalSpeeds =
  { MODOPT_ARG(unsigned int), "NumSpatioTemporalSpeeds", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of optimal speeds for spatiotemporal motion channels",
    "num-spatiotemporalspeeds", '\0', "<int>", "3" };

// Used by: MotionOpticalFlowChannel
const ModelOptionDef OPT_NumOpticalFlowDirections =
  { MODOPT_ARG(unsigned int), "NumOpticalFlowDirections", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of directions for optical flow motion channels",
    "num-opticalflowdirections", '\0', "<int>", "8" };

// Used by: FOEChannel
const ModelOptionDef OPT_NumDirectionsFOE =
  { MODOPT_ARG(unsigned int), "NumDirections", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of direction-selective foe channels",
    "num-foedirections", '\0', "<int>", "4" };


// Used by: FOEMstChannel
const ModelOptionDef OPT_FoeMSTChannelDelta =
  { MODOPT_ARG(uint), "FoeMSTChannelDelta", &MOC_CHANNEL, OPTEXP_CORE,
    "Distance from current pixel at which the presence or absence of "
    "features should be checked for, when computing junction filter "
    "outputs in foeMstChannel (--vc-chans=B).",
    "foeMst-delta", '\0', "<uint>", "6" };


// Used by: StereoChannel
const ModelOptionDef OPT_NumTheta =
  { MODOPT_ARG(unsigned int), "NumTheta", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of oriented disparity channels",
    "num-theta", '\0', "<int>", "1" };

// Used by: LJunctionChannel
const ModelOptionDef OPT_LJunctionChannelDelta =
  { MODOPT_ARG(uint), "LJunctionChannelDelta", &MOC_CHANNEL, OPTEXP_CORE,
    "Distance from current pixel at which the presence or absence of "
    "features should be checked for, when computing junction filter "
    "outputs in LJunctionChannel (--vc-chans=L).",
    "l-junction-delta", '\0', "<uint>", "6" };

// Used by: TJunctionChannel
const ModelOptionDef OPT_TJunctionChannelDelta =
  { MODOPT_ARG(uint), "TJunctionChannelDelta", &MOC_CHANNEL, OPTEXP_CORE,
    "Distance from current pixel at which the presence or absence of "
    "features should be checked for, when computing junction filter "
    "outputs in TJunctionChannel (--vc-chans=T).",
    "t-junction-delta", '\0', "<uint>", "6" };

// Used by: XJunctionChannel
const ModelOptionDef OPT_XJunctionChannelDelta =
  { MODOPT_ARG(uint), "XJunctionChannelDelta", &MOC_CHANNEL, OPTEXP_CORE,
    "Distance from current pixel at which the presence or absence of "
    "features should be checked for, when computing junction filter "
    "outputs in XJunctionChannel (--vc-chans=X).",
    "x-junction-delta", '\0', "<uint>", "6" };

// Used by: EndPointChannel (and derivatives)
const ModelOptionDef OPT_EndPointChannelDelta =
  { MODOPT_ARG(uint), "EndPointChannelDelta", &MOC_CHANNEL, OPTEXP_CORE,
    "Distance from current pixel at which the presence or absence of "
    "features should be checked for, when computing junction filter "
    "outputs in EndPointChannel (--vc-chans=E).",
    "end-point-delta", '\0', "<uint>", "6" };

// Used by: LJunctionChannel
const ModelOptionDef OPT_NumLOrients =
  { MODOPT_ARG(unsigned int), "NumLOrients", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of oriented l-junction channels",
    "num-l-orients", '\0', "<int>", "4" };

// Used by: TJunctionChannel
const ModelOptionDef OPT_NumTOrients =
  { MODOPT_ARG(unsigned int), "NumTOrients", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of oriented t-junction channels",
    "num-t-orients", '\0', "<int>", "4" };

// Used by: XJunctionChannel
const ModelOptionDef OPT_NumXOrients =
  { MODOPT_ARG(unsigned int), "NumXOrients", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of oriented x-junction channels",
    "num-x-orients", '\0', "<int>", "2" };

// Used by: EndPointChannel
const ModelOptionDef OPT_NumEOrients =
  { MODOPT_ARG(unsigned int), "NumEOrients", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of oriented end-point channels",
    "num-e-orients", '\0', "<int>", "4" };

// Used by: LJunctionChannel
const ModelOptionDef OPT_LFullImplementation =
  { MODOPT_FLAG, "LFullImplementation", &MOC_CHANNEL, OPTEXP_CORE,
    "Use full implementation of the l-junction channel",
    "use-full-l", '\0', "", "false" };

// Used by: TJunctionChannel
const ModelOptionDef OPT_TFullImplementation =
  { MODOPT_FLAG, "TFullImplementation", &MOC_CHANNEL, OPTEXP_CORE,
    "Use full implementation of the t-junction channel",
    "use-full-t", '\0', "", "false" };

// Used by: XJunctionChannel
const ModelOptionDef OPT_XFullImplementation =
  { MODOPT_FLAG, "XFullImplementation", &MOC_CHANNEL, OPTEXP_CORE,
    "Use full implementation of the x-junction channel",
    "use-full-x", '\0', "", "false" };

// Used by: EndPointChannel
const ModelOptionDef OPT_EFullImplementation =
  { MODOPT_FLAG, "EFullImplementation", &MOC_CHANNEL, OPTEXP_CORE,
    "Use full implementation of the end-point channel",
    "use-full-e", '\0', "", "false" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_SingleChannelUseSplitCS =
  { MODOPT_FLAG, "SingleChannelUseSplitCS", &MOC_CHANNEL, OPTEXP_CORE,
    "Use split positive/negative center-surround computations",
    "use-split-cs", '\0', "", "false" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_SingleChannelSaveRawMaps =
  { MODOPT_FLAG, "SingleChannelSaveRawMaps", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save raw input maps (pyramid levels) from all single channels (\"SR\")",
    "save-raw-maps", '\0', "", "false" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_SingleChannelComputeFullPyramidForGist =
  { MODOPT_FLAG, "SingleChannelComputeFullPyramidForGist", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save raw input maps (pyramid levels) from all single channels for gist"
    "computation  (\"SR\")",
    "save-raw-maps-gist", '\0', "", "false" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_SingleChannelSaveFeatureMaps =
  { MODOPT_FLAG, "SingleChannelSaveFeatureMaps", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save center-surround feature maps from all single channels (\"SF\")",
    "save-feature-maps", '\0', "", "false" };

// Used by: SingleChannel (and derivatives)
const ModelOptionDef OPT_SingleChannelSaveOutputMap =
  { MODOPT_FLAG, "SingleChannelSaveOutputMap", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save combined center-surround output maps from all single channels (\"SO\")",
    "save-featurecomb-maps", '\0', "", "false" };

// Used by: ComplexChannel (and derivatives)
const ModelOptionDef OPT_ComplexChannelSaveOutputMap =
  { MODOPT_FLAG, "ComplexChannelSaveOutputMap", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save conspicuity maps from all complex channels (\"CO\")",
    "save-conspic-maps", '\0', "", "false" };

// Used by: ComplexChannel
const ModelOptionDef OPT_ComplexChannelMapCombineType =
  { MODOPT_ARG(MapCombineType), "ComplexChannelMapCombineType", &MOC_CHANNEL, OPTEXP_CORE,
    "Strategy used by ComplexChannel for combining output maps from "
    "subchannels. Default strategy is summation; alternatives include "
    "pixel-wise max.",
    "map-combine-type", '\0', "<Sum|Max>", "Sum" };

// Used by: ColorChannel
const ModelOptionDef OPT_ColorComputeType =
  { MODOPT_ARG(ColorComputeType), "ColorComputeType", &MOC_CHANNEL, OPTEXP_CORE,
    "Type of computation used to compute RG and BY color opponencies",
    "color-comp-type", '\0', "<Standard|Simple|StandardFull>",
    "Standard" };

// Used by: DummyChannel
const ModelOptionDef OPT_DummyChannelFactor =
  { MODOPT_ARG(float), "DummyChannelFactor", &MOC_CHANNEL, OPTEXP_CORE,
    "Factor to multiply the input by, or 0.0 for nothing",
    "dummychannel-factor", '\0', "<float>", "0.0" };


// Used by: TcorrChannel
const ModelOptionDef OPT_TcorrChannelFrameLag =
  { MODOPT_ARG(int), "TcorrChannelFrameLag", &MOC_CHANNEL, OPTEXP_CORE,
    "Frame lag with which to compute the temporal correlations",
    "tcorrchannel-framelag", '\0', "<int>", "1" };


// Used by: ScorrChannel
const ModelOptionDef OPT_ScorrChannelRadius =
  { MODOPT_ARG(int), "ScorrChannelRadius", &MOC_CHANNEL, OPTEXP_CORE,
    "Radius (in saliency map pixels) to compute the spatial correlations",
    "scorrchannel-radius", '\0', "<int>", "1" };

// Used by: SingleChannel
const ModelOptionDef OPT_SubmapAlgoType =
  { MODOPT_ARG_STRING, "SubmapAlgoType", &MOC_CHANNEL, OPTEXP_CORE,
    "Name of the SubmapAlgorithm type to use in SingleChannel",
    "submap-algo", '\0', "<Std|List>", "Std" };

// Used by: SingleChannel
const ModelOptionDef OPT_GetSingleChannelStats =
  { MODOPT_FLAG, "GetSingleChannelStats", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save (append) several types of statistics  for each single channel to sc_stats.txt",
    "save-channel-stats", '\0', "", "false" };

// Used by: SingleChannel
const ModelOptionDef OPT_SaveStatsPerChannel =
  { MODOPT_FLAG, "SaveStatsPerChannel", &MOC_CHANNEL, OPTEXP_SAVE,
    "When saving channel stats, should we put the data for each channel into its own file?",
    "save-stats-per-channel", '\0', "", "false" };

// Used by: SingleChannel
const ModelOptionDef OPT_SaveStatsPerChannelFreq =
  { MODOPT_FLAG, "SaveStatsPerChannelFreq", &MOC_CHANNEL, OPTEXP_SAVE,
    "When saving channel stats, should we also save frequency data?",
    "save-stats-per-channel-freq", '\0', "", "false" };

// Used by: SingleChannel
const ModelOptionDef OPT_GetSingleChannelStatsFile =
  { MODOPT_ARG(std::string), "GetSingleChannelStatsFile", &MOC_CHANNEL,
    OPTEXP_SAVE,
    "File name to use for single channel stats file",
    "save-channel-stats-name", '\0', "<string>", "sc_stats.txt" };

// Used by: SingleChannel
const ModelOptionDef OPT_GetSingleChannelStatsTag =
  { MODOPT_ARG(std::string), "GetSingleChannelStatsTag", &MOC_CHANNEL,
    OPTEXP_SAVE,
    "Tag name to use for single channel stats file",
    "save-channel-stats-tag", '\0', "<string>", "NULL" };

// Used by: H2SVChannel
const ModelOptionDef OPT_UseH2SV1 =
  { MODOPT_FLAG, "UseH2SV1", &MOC_CHANNEL, OPTEXP_CORE,
    "Use H2SV1 color type in H2SV rather than H2SV2",
    "use-h2sv1", '\0', "", "false" };

// Used by: CompositeColorChannel
const ModelOptionDef OPT_CompColorDoubleOppWeight =
  { MODOPT_ARG(double), "CompColorDoubleOppWeight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to double-opponent channels in CompositeColorChannel",
    "compcolor-double-weight", '\0', "<double>", "1.0" };

// Used by: CompositeColorChannel
const ModelOptionDef OPT_CompColorSingleOppWeight =
  { MODOPT_ARG(double), "CompColorSingleOppWeight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to single-opponent channels in CompositeColorChannel",
    "compcolor-single-weight", '\0', "<double>", "1.0" };

const ModelOptionCateg MOC_INTCHANNEL = {
  MOC_SORTPRI_3, "Channel-Related Integer Math Options" };

// Used by: IntegerSimpleChannel, IntegerComplexChannel
const ModelOptionDef OPT_IntChannelScaleBits =
  { MODOPT_ARG(uint), "IntChannelScaleBits", &MOC_INTCHANNEL, OPTEXP_CORE,
    "Number of working bits to use in integer channels",
    "int-chan-scale-bits", '\0', "<uint>", "30" };

// Used by: IntegerSimpleChannel, IntegerComplexChannel
const ModelOptionDef OPT_IntChannelOutputRangeMax =
  { MODOPT_ARG(int), "IntChannelOutputRangeMax", &MOC_INTCHANNEL, OPTEXP_CORE,
    "Max of the integer channel's output range",
    "int-chanout-max", '\0', "<int>", "32768" };

// Used by: IntegerSimpleChannel, IntegerComplexChannel
const ModelOptionDef OPT_IntChannelOutputRangeMin =
  { MODOPT_ARG(int), "IntChannelOutputRangeMin", &MOC_INTCHANNEL, OPTEXP_CORE,
    "Min of the integer channel's output range",
    "int-chanout-min", '\0', "<int>", "0" };

// Used by: IntegerMathEngine
const ModelOptionDef OPT_IntMathLowPass5 =
  { MODOPT_ARG(std::string), "IntMathLowPass5", &MOC_INTCHANNEL, OPTEXP_CORE,
    "Which integer lowpass5 implementation to use",
    "int-math-lowpass5", '\0', "<lp5std|lp5optim>", "lp5optim" };

// Used by: IntegerMathEngine
const ModelOptionDef OPT_IntMathLowPass9 =
  { MODOPT_ARG(std::string), "IntMathLowPass9", &MOC_INTCHANNEL, OPTEXP_CORE,
    "Which integer lowpass9 implementation to use",
    "int-math-lowpass9", '\0', "<lp9std|lp9optim>", "lp9optim" };

const ModelOptionDef OPT_IntInputDecode =
  { MODOPT_ARG(IntegerDecodeType), "IntDecodeType",
    &MOC_INTCHANNEL, OPTEXP_CORE,
    "Which type of integer decoding to apply to input frames to "
    "extract the luminance and chrominance components. The 'rgb' "
    "decoding strategy is used by default, in which the input frame "
    "is converted to 8-bit RGB, and from there the luminance and "
    "chrominance are computed separately. The 'video' decoding "
    "strategy may be more efficient if the input frames are natively "
    "in some YUV format, since the luminance and chrominance are "
    "computed in a single pass over the YUV data, with the Y component "
    "used for the luminance and Y-normalized U/Y and V/Y components "
    "used for chrominance; note that 'video' decoding strategy will "
    "give somewhat different results from the 'rgb' decoding.",
    "int-input-decode", '\0', "<rgb|video>", "rgb" };

const ModelOptionDef OPT_ALIASsaveChannelOutputs =
  { MODOPT_ALIAS, "ALIASsaveChannelOutputs", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save all channel outputs",
    "save-channel-outputs", '\0', "",
    "--save-featurecomb-maps --save-conspic-maps" };

const ModelOptionDef OPT_ALIASsaveChannelInternals =
  { MODOPT_ALIAS, "ALIASsaveChannelInternals", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save all available channel internal maps",
    "save-channel-internals", '\0', "",
    "--save-raw-maps --save-feature-maps" };

// ######################################################################
const ModelOptionCateg MOC_VCX = {
  MOC_SORTPRI_3, "VisualCortex-Related Options" };

// Used by: VisualCortex (and derivatives)
const ModelOptionDef OPT_RawVisualCortexOutputFactor =
  { MODOPT_ARG(float), "RawVisualCortexOutputFactor", &MOC_VCX, OPTEXP_CORE,
    "Factor applied to outputs of VisualCortex to scale them to Amps of "
    "synaptic input currents to saliency map",
    "vcx-outfac", '\0', "<float>", "1.0e-9" };

// Used by: VisualCortex (and derivatives)
const ModelOptionDef OPT_RawVisualCortexNoise =
  { MODOPT_ARG(float), "RawVisualCortexNoise", &MOC_VCX, OPTEXP_CORE,
    "Noise applied to outputs of VisualCortex after --vcx-outfac has been applied",
    "vcx-noise", '\0', "<float>", "1.0e-12" };

// Used by: VisualCortex and derivatives
const ModelOptionDef OPT_VCXsaveOutTo =
  { MODOPT_ARG_STRING, "VCXsaveOutTo", &MOC_VCX, OPTEXP_CORE,
    "Save the raw VisualCortex output map to the designated MGZ file if "
    "a filename is specified. The saved outputs can later be re-read using "
    "the --vcx-load-out-from option.",
    "vcx-save-out-to", '\0', "<filename.mgz>", "" };

// Used by: VisualCortex and derivatives
const ModelOptionDef OPT_VCXloadOutFrom =
  { MODOPT_ARG_STRING, "VCXloadOutFrom", &MOC_VCX, OPTEXP_CORE,
    "Load the raw VisualCortex output map from the designated MGZ file if "
    "a filename is specified. Typically, the MGZ file should contain maps "
    "which have been previously saved using the --vcx-save-out-to option. "
    "CAUTION: When this option is in use, we will not compute much in "
    "the VisualCortex, instead reading the precomputed results off the disk. "
    "This means in particular that whichever channels you may have in your "
    "VisualCortex will not affect its output.",
    "vcx-load-out-from", '\0', "<filename.mgz>", "" };

// Used by: VisualCortex and derivatives
const ModelOptionDef OPT_VCXsaveRawCSOutTo =
  { MODOPT_ARG_STRING, "VCXsaveRawCSOutTo", &MOC_VCX, OPTEXP_CORE,
    "Save the raw center-surround maps for each channel to a single "
    "designated MGZ file if a filename is specified. The maps will be "
    "concatinated vertically. ",
    "vcx-save-rawcs-out-to", '\0', "<filename.mgz>", "" };

// Used by: VisualCortex and derivatives
const ModelOptionDef OPT_VCXsaveRawCSOutDims =
{ MODOPT_ARG(Dims), "VCXsaveRawCSOutToDims", &MOC_VCX, OPTEXP_CORE,
  "The dimensions at which to save the raw center-surround maps. "
  "Rescaling without interpolation will be performed if necessary. ",
    "vcx-rawcs-dims", '\0', "<wxh>", "40x30" };

// Used by: RawVisualCortex
const ModelOptionDef OPT_RawVisualCortexChans =
  { MODOPT_ARG_STRING, "RawVisualCortexChans", &MOC_VCX, OPTEXP_CORE,
    "Configure which channels to use in your VisualCortex by "
    "specifying a series of letters from:\n"
    "  C: double-opponent color center-surround\n"
    "  D: dummy channel\n"
    "  E: end-stop detector\n"
    "  F: flicker center-surround\n"
    "  G: multi-color band channel\n"
    "  H: H2SV channel\n"
    "  Y: Use a Depth Channel with spatial depth information\n"
    "  I: intensity center-surround\n"
    "  K: skin hue detector\n"
    "  L: L-junction detector\n"
    "  M: motion energy center-surround\n"
    "  U: foe\n"
    "  B: FoeMST\n"
    "  N: intensity-band channel\n"
    "  O: orientation contrast\n"
    "  P: SIFT channel\n"
    "  Q: CIELab Color channel\n"
    "  R: Pedestrian channel\n"
    "  S: composite single-opponent color center-surround\n"
    "  T: T-junction detector\n"
    "  V: short-range orientation interactions (\"sox\") channel\n"
    "  W: contour channel\n"
    "  X: X-junction detector\n"
    "  Z: dummy zero channel\n"
    "  A: Object Detection channel\n"
    "  J: DKL Color channel\n"
    "  U: Foreground Detection Channel\n"
    "  i: Imagize 3-chanel Silicon Retina Channel\n"
    "  s: Motion SpatioTemporal Energy Channel\n"
    "  o: Motion Optical Flow Channel\n"
    "with possible optional weights (given just after the channel letter, "
    "with a ':' in between). EXAMPLE: 'IO:5.0M' will use intensity (weight 1.0), orientation "
    "(weight 5.0) and motion (weight 1.0) channels",
    "vc-chans", '\0',
    "<CIOFM...>",
    "CFIOM" };

// Used by: IntegerRawVisualCortex
const ModelOptionDef OPT_IntegerRawVisualCortexChans =
  { MODOPT_ARG_STRING, "IntegerRawVisualCortexChans", &MOC_VCX, OPTEXP_CORE,
    "The string specifies a list of channels to use in your IntegerVisualCortex as a series of "
    "letters from:\n"
    "  C: double-opponent color center-surround\n"
    "  I: intensity center-surround\n"
    "  O: orientation contrast\n"
    "  F: flicker center-surround\n"
    "  M: motion energy center-surround\n"
    "  U: foe\n"
    "  B: FoeMST\n"
    "with possible optional weights (given just after the channel letter, "
    "with a ':' in between). Note that this is a subset of what is "
    "supported by --vc-chans for the standard (float) VisualCortex.\n"
    "EXAMPLE: 'IO:5.0M' will use intensity (weight 1.0), orientation "
    "(weight 5.0) and motion (weight 1.0) channels",
    "ivc-chans", '\0', "<CIOFM>", "CFIOM" };

// Used by: VisualCortex (and derivatives)
const ModelOptionDef OPT_RawVisualCortexSaveOutput =
  { MODOPT_FLAG, "RawVisualCortexSaveOutput", &MOC_VCX, OPTEXP_SAVE,
    "Save output of visual cortex (=input to the saliency map) as a float "
    "image in PFM format with absolute saliency values. This is good "
    "for comparing several saliency maps obtained for different images. "
    "See saliency/matlab/pfmreadmatlab.m for a program that reads PFM "
    "images into Matlab.",
    "save-vcx-output", '\0', "", "false" };

// Used by: RawVisualCortex (and derivatives)
const ModelOptionDef OPT_VCXuseMax =
  { MODOPT_FLAG, "VCXuseMax", &MOC_VCX, OPTEXP_SAVE,
    "Use max across features instead of sum to yield the combined saliency map.",
    "vcx-usemax", '\0', "", "false" };

// Used by: RawVisualCortex (and derivatives)
const ModelOptionDef OPT_VCXweightThresh =
  { MODOPT_ARG(float), "VCXweightThresh", &MOC_VCX, OPTEXP_SAVE,
    "Lower threshold on total channel weight when counting the number of non-zero "
    "weight channels to decide whether or not to apply one last round of maxnorm "
    "to the VisualCortex output, if the number of non-zero-weight channels is 2 or more.",
    "vcx-weight-thresh", '\0', "<float>", "0.0" };


//! request all of the above-named OPT_ALIAS options
void REQUEST_OPTIONALIAS_CHANNEL(OptionManager& m)
{
  m.requestOptionAlias(&OPT_ALIASsaveChannelOutputs);
  m.requestOptionAlias(&OPT_ALIASsaveChannelInternals);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_CHANNELOPTS_C_DEFINED
