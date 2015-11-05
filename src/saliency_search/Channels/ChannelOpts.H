/*!@file Channels/ChannelOpts.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelOpts.H $
// $Id: ChannelOpts.H 15097 2011-11-28 18:35:15Z dberg $
//

#ifndef CHANNELS_CHANNELOPTS_H_DEFINED
#define CHANNELS_CHANNELOPTS_H_DEFINED

class OptionManager;
struct ModelOptionCateg;
struct ModelOptionDef;

//! Command-line options for ChannelBase and derivatives
//@{

//! Channel-related options
extern const ModelOptionCateg MOC_CHANNEL;

extern const ModelOptionDef OPT_UseOlderVersion;
extern const ModelOptionDef OPT_ChannelOutputRangeMax;
extern const ModelOptionDef OPT_ChannelOutputRangeMin;
extern const ModelOptionDef OPT_MapLevel;
extern const ModelOptionDef OPT_LevelSpec;
extern const ModelOptionDef OPT_MaxNormType;
extern const ModelOptionDef OPT_SingleChannelQueueLen;
extern const ModelOptionDef OPT_SingleChannelTimeDecay;
extern const ModelOptionDef OPT_NumOrientations;
extern const ModelOptionDef OPT_OriInteraction;
extern const ModelOptionDef OPT_OrientComputeType;
extern const ModelOptionDef OPT_GaborChannelIntensity;
extern const ModelOptionDef OPT_UseTrigTab;
extern const ModelOptionDef OPT_DirectionChannelTakeSqrt;
extern const ModelOptionDef OPT_DirectionChannelLowThresh;
extern const ModelOptionDef OPT_NumColorBands;
extern const ModelOptionDef OPT_NumSatBands;
extern const ModelOptionDef OPT_HueBandWidth;
extern const ModelOptionDef OPT_SatBandWidth;
extern const ModelOptionDef OPT_NumDirections;
extern const ModelOptionDef OPT_NumSpatioTemporalDirections;
extern const ModelOptionDef OPT_NumSpatioTemporalSpeeds;
extern const ModelOptionDef OPT_NumOpticalFlowDirections;
extern const ModelOptionDef OPT_NumTheta;
extern const ModelOptionDef OPT_LJunctionChannelDelta;
extern const ModelOptionDef OPT_TJunctionChannelDelta;
extern const ModelOptionDef OPT_XJunctionChannelDelta;
extern const ModelOptionDef OPT_EndPointChannelDelta;
extern const ModelOptionDef OPT_FoeMSTChannelDelta;
extern const ModelOptionDef OPT_NumLOrients;
extern const ModelOptionDef OPT_NumTOrients;
extern const ModelOptionDef OPT_NumXOrients;
extern const ModelOptionDef OPT_NumEOrients;
extern const ModelOptionDef OPT_LFullImplementation;
extern const ModelOptionDef OPT_TFullImplementation;
extern const ModelOptionDef OPT_XFullImplementation;
extern const ModelOptionDef OPT_EFullImplementation;
extern const ModelOptionDef OPT_SingleChannelUseSplitCS;
extern const ModelOptionDef OPT_SingleChannelSaveRawMaps;
extern const ModelOptionDef OPT_SingleChannelComputeFullPyramidForGist;
extern const ModelOptionDef OPT_SingleChannelSaveFeatureMaps;
extern const ModelOptionDef OPT_SingleChannelSaveOutputMap;
extern const ModelOptionDef OPT_ComplexChannelSaveOutputMap;
extern const ModelOptionDef OPT_ComplexChannelMapCombineType;
extern const ModelOptionDef OPT_ColorComputeType;
extern const ModelOptionDef OPT_DummyChannelFactor;
extern const ModelOptionDef OPT_TcorrChannelFrameLag;
extern const ModelOptionDef OPT_ScorrChannelRadius;
extern const ModelOptionDef OPT_NumIntensityBands;
extern const ModelOptionDef OPT_IntensityBandWidth;
extern const ModelOptionDef OPT_SubmapAlgoType;
extern const ModelOptionDef OPT_GetSingleChannelStats;
extern const ModelOptionDef OPT_SaveStatsPerChannel;
extern const ModelOptionDef OPT_SaveStatsPerChannelFreq;
extern const ModelOptionDef OPT_GetSingleChannelStatsFile;
extern const ModelOptionDef OPT_GetSingleChannelStatsTag;
extern const ModelOptionDef OPT_UseH2SV1;
extern const ModelOptionDef OPT_CompColorDoubleOppWeight;
extern const ModelOptionDef OPT_CompColorSingleOppWeight;
//@}

//! Command-line options for raw VisualCortex
//@{

//! VisualCortex-related options
extern const ModelOptionCateg MOC_VCX;

extern const ModelOptionDef OPT_RawVisualCortexOutputFactor;
extern const ModelOptionDef OPT_RawVisualCortexNoise;
extern const ModelOptionDef OPT_VCXsaveOutTo;
extern const ModelOptionDef OPT_VCXloadOutFrom;
extern const ModelOptionDef OPT_VCXsaveRawCSOutTo;
extern const ModelOptionDef OPT_VCXsaveRawCSOutDims;
extern const ModelOptionDef OPT_RawVisualCortexChans;
extern const ModelOptionDef OPT_IntegerRawVisualCortexChans;
extern const ModelOptionDef OPT_RawVisualCortexSaveOutput;
extern const ModelOptionDef OPT_VCXuseMax;
extern const ModelOptionDef OPT_VCXweightThresh;

//@}

//! Command-line options for integer math in the channel classes
//@{

//! Channel-related integer-math options
extern const ModelOptionCateg MOC_INTCHANNEL;

extern const ModelOptionDef OPT_IntChannelScaleBits;
extern const ModelOptionDef OPT_IntChannelOutputRangeMax;
extern const ModelOptionDef OPT_IntChannelOutputRangeMin;
extern const ModelOptionDef OPT_IntMathLowPass5;
extern const ModelOptionDef OPT_IntMathLowPass9;
extern const ModelOptionDef OPT_IntInputDecode;
//@}

//! Channel-related command-line option aliases
//@{
extern const ModelOptionDef OPT_ALIASsaveChannelOutputs;
extern const ModelOptionDef OPT_ALIASsaveChannelInternals;

//! request all of the above-named OPT_ALIAS options
void REQUEST_OPTIONALIAS_CHANNEL(OptionManager& m);

//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_CHANNELOPTS_H_DEFINED
