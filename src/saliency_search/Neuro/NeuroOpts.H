/*!@file Neuro/NeuroOpts.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/NeuroOpts.H $
// $Id: NeuroOpts.H 14984 2011-10-14 00:17:14Z dberg $
//

#ifndef NEURO_NEUROOPTS_H_DEFINED
#define NEURO_NEUROOPTS_H_DEFINED

class OptionManager;
struct ModelOptionCateg;
struct ModelOptionDef;

//! What to display/save options
extern const ModelOptionCateg MOC_DISPLAY;

//! Command-line options for Retina
//@{

//! PFC-related options
extern const ModelOptionCateg MOC_PFC;

extern const ModelOptionDef OPT_PrefrontalCortexType;

//! Retina-related options
extern const ModelOptionCateg MOC_RETINA;

extern const ModelOptionDef OPT_RetinaType;
extern const ModelOptionDef OPT_InputFramingImageName;
extern const ModelOptionDef OPT_InputFramingImagePos;
extern const ModelOptionDef OPT_FoveateInputDepth;
extern const ModelOptionDef OPT_ShiftInputToEye;
extern const ModelOptionDef OPT_ShiftInputToEyeBGcol;
extern const ModelOptionDef OPT_InputFOV;
extern const ModelOptionDef OPT_RetinaSaveInput;
extern const ModelOptionDef OPT_RetinaSaveOutput;
extern const ModelOptionDef OPT_RetinaStdSavePyramid;
extern const ModelOptionDef OPT_RetinaMaskFname;
extern const ModelOptionDef OPT_RetinaFlipHoriz;
extern const ModelOptionDef OPT_RetinaFlipVertic;
//@}

//! Command-line options for VisualCortex and derivatives
//@{

extern const ModelOptionDef OPT_VisualCortexType;
extern const ModelOptionDef OPT_VCEMeyeFnames;
extern const ModelOptionDef OPT_VCEMsigma;
extern const ModelOptionDef OPT_VCEMforgetFac;
extern const ModelOptionDef OPT_VCEMdelay;
extern const ModelOptionDef OPT_VCEMuseMax;
extern const ModelOptionDef OPT_VCEMsaccadeOnly;
//@}

//! Command-line options for SimulationViewer and derivatvies
extern const ModelOptionDef OPT_SimulationViewerType;
extern const ModelOptionDef OPT_SVfontSize;
extern const ModelOptionDef OPT_SVsaveTraj;
extern const ModelOptionDef OPT_SVsaveXcombo;
extern const ModelOptionDef OPT_SVsaveYcombo;
extern const ModelOptionDef OPT_SVsaveTRMXcombo;
extern const ModelOptionDef OPT_SVsaveTRMYcombo;
extern const ModelOptionDef OPT_SVsaveTRMmegaCombo;
extern const ModelOptionDef OPT_SVwarp3D;
extern const ModelOptionDef OPT_SVmegaCombo;
extern const ModelOptionDef OPT_SVmegaComboZoom;
extern const ModelOptionDef OPT_SVmegaComboTopCMapsOnly;
extern const ModelOptionDef OPT_SVcropFOA;
extern const ModelOptionDef OPT_SVxwindow;
extern const ModelOptionDef OPT_SVfoveateTraj;
extern const ModelOptionDef OPT_SVstatsFname;
extern const ModelOptionDef OPT_SVdisplayFOA;
extern const ModelOptionDef OPT_SVdisplayPatch;
extern const ModelOptionDef OPT_SVpatchSize;
extern const ModelOptionDef OPT_SVeraseMarker;
extern const ModelOptionDef OPT_SVdisplayFOAnum;
extern const ModelOptionDef OPT_SVdisplayFOALinks;
extern const ModelOptionDef OPT_SVdisplayEye;
extern const ModelOptionDef OPT_SVdisplayEyeLinks;
extern const ModelOptionDef OPT_SVdisplayHead;
extern const ModelOptionDef OPT_SVdisplayHeadLinks;
extern const ModelOptionDef OPT_SVdisplayAdditive;
extern const ModelOptionDef OPT_SVdisplayTime;
extern const ModelOptionDef OPT_SVdisplayHighlights;
extern const ModelOptionDef OPT_SVdisplaySMmodulate;
extern const ModelOptionDef OPT_SVdisplayBoring;
extern const ModelOptionDef OPT_SVuseLargerDrawings;
extern const ModelOptionDef OPT_SVdisplayInterp;
extern const ModelOptionDef OPT_SVdisplayMapFactor;
extern const ModelOptionDef OPT_SVdisplayMapType;
extern const ModelOptionDef OPT_SVCOMPsaveMask;
extern const ModelOptionDef OPT_SVCOMPsaveFoveatedImage;
extern const ModelOptionDef OPT_SVCOMPDistanceFactor;
extern const ModelOptionDef OPT_SVCOMPsaveEyeCombo;
extern const ModelOptionDef OPT_SVCOMPnumFoveas;
extern const ModelOptionDef OPT_SVCOMPeyeCompare;
extern const ModelOptionDef OPT_SVCOMPcacheSize;
extern const ModelOptionDef OPT_SVCOMPiframePeriod;
extern const ModelOptionDef OPT_SVCOMPdisplayHumanEye;
extern const ModelOptionDef OPT_SVCOMPuseTRMmax;
extern const ModelOptionDef OPT_SVCOMPfoveaSCtype;
extern const ModelOptionDef OPT_SVCOMPMultiRetinaDepth;
extern const ModelOptionDef OPT_SVEMsaveMegaCombo;
extern const ModelOptionDef OPT_SVEMdelayCacheSize;
extern const ModelOptionDef OPT_SVEMmaxCacheSize;
extern const ModelOptionDef OPT_SVEMoutFname;
extern const ModelOptionDef OPT_SVEMsampleAtStart;
extern const ModelOptionDef OPT_SVEMdisplaySacNum;
extern const ModelOptionDef OPT_SVEMnumRandomSamples;
extern const ModelOptionDef OPT_SVEMhistoryNumRandomSamples;
extern const ModelOptionDef OPT_SVEMuseSaccadeInBlink;
extern const ModelOptionDef OPT_SVEMuseDiagColors;
extern const ModelOptionDef OPT_SVEMlabelEyePatch;
extern const ModelOptionDef OPT_SVEMwriteFrameNumber;
extern const ModelOptionDef OPT_SVEMpriorRandomDistro;
extern const ModelOptionDef OPT_SVEMviewingStrategy;
extern const ModelOptionDef OPT_SVEMviewingStrategyRandomDistro;
extern const ModelOptionDef OPT_SVEMviewingStrategyTiming;
extern const ModelOptionDef OPT_SVEMmaxComboWidth;
extern const ModelOptionDef OPT_SVEMbufDims;
extern const ModelOptionDef OPT_SVEMuseIOR;
extern const ModelOptionDef OPT_SVHandSaveCombo;
extern const ModelOptionDef OPT_SVHandMaxComboWidth;
extern const ModelOptionDef OPT_SVHandDisplay;
extern const ModelOptionDef OPT_SVeyeSimFname;
extern const ModelOptionDef OPT_SVeyeSimPeriod;
extern const ModelOptionDef OPT_SVeyeSimTrash;
extern const ModelOptionDef OPT_ASACdrawDiffParts;
extern const ModelOptionDef OPT_ASACdrawBetaParts;
extern const ModelOptionDef OPT_ASACdrawBiasParts;
extern const ModelOptionDef OPT_ASACdrawSeperableParts;
extern const ModelOptionDef OPT_ASACconfigFile;
extern const ModelOptionDef OPT_NerdCamConfigFile;

extern const ModelOptionDef OPT_FOAradius;
extern const ModelOptionDef OPT_FoveaRadius;

extern const ModelOptionDef OPT_AGStatsSaveFile;
extern const ModelOptionDef OPT_ComputeAGStats;

extern const ModelOptionDef OPT_SVEMNprobeLocation;
extern const ModelOptionDef OPT_SVEMNrfSize;
extern const ModelOptionDef OPT_SVEMNrfMaskName;
extern const ModelOptionDef OPT_SVEMNoutFName;
extern const ModelOptionDef OPT_SVEMNneuroFileName;
extern const ModelOptionDef OPT_SVEMNdisplayTime;
extern const ModelOptionDef OPT_SVEMNdelayNeuron;
extern const ModelOptionDef OPT_SVEMNplotBufLength;
extern const ModelOptionDef OPT_SVEMNSalScale;
extern const ModelOptionDef OPT_SVEMNNeuroScale;

extern const ModelOptionDef OPT_SVEMNNeuroRFVisFile;
extern const ModelOptionDef OPT_SVEMNNeuroRFMotFile;
extern const ModelOptionDef OPT_SVEMNMotWindow;
extern const ModelOptionDef OPT_SVEMNVisWindow;
extern const ModelOptionDef OPT_SVEMNVisOffset;
extern const ModelOptionDef OPT_SVEMNShowSCMap;
extern const ModelOptionDef OPT_SVEMNSCMapDims;

extern const ModelOptionDef OPT_SVInverseTransform;

//@}

//! Command-line options for InferoTemporal
//@{

//! Inferior Temporal Cortex Options
extern const ModelOptionCateg MOC_ITC;

extern const ModelOptionDef OPT_InferoTemporalType;
extern const ModelOptionDef OPT_AttentionObjRecog;
extern const ModelOptionDef OPT_ObjectDatabaseFileName;
extern const ModelOptionDef OPT_TrainObjectDB;
extern const ModelOptionDef OPT_PromptUserTrainDB;
extern const ModelOptionDef OPT_MatchObjects;
extern const ModelOptionDef OPT_MatchingAlgorithm;
extern const ModelOptionDef OPT_RecognitionMinMatch;
extern const ModelOptionDef OPT_OutputFileName;
//@}

//! Command-line options for ShapeEstimator
//@{
extern const ModelOptionDef OPT_ShapeEstimatorMode;
extern const ModelOptionDef OPT_ShapeEstimatorSmoothMethod;
//@}

//! Command-line options for Brain
//@{

//! Brain-related options
extern const ModelOptionCateg MOC_BRAIN;

extern const ModelOptionDef OPT_IORtype;
extern const ModelOptionDef OPT_BrainBoringDelay;
extern const ModelOptionDef OPT_BrainBoringSMmv;
extern const ModelOptionDef OPT_BrainSaveObjMask;
extern const ModelOptionDef OPT_BlankBlink;
extern const ModelOptionDef OPT_BrainTooManyShifts;
extern const ModelOptionDef OPT_BrainTooManyShiftsPerFrame;
extern const ModelOptionDef OPT_TargetMaskFname;

//@}

//! Command-line options for SaliencyMap
//@{

//! SaliencyMap options
extern const ModelOptionCateg MOC_SM;

extern const ModelOptionDef OPT_SaliencyMapType;
extern const ModelOptionDef OPT_SMsaveResults;
extern const ModelOptionDef OPT_SMsaveCumResults;
extern const ModelOptionDef OPT_SMuseSacSuppress;
extern const ModelOptionDef OPT_SMuseBlinkSuppress;
extern const ModelOptionDef OPT_SMfastInputCoeff;
extern const ModelOptionDef OPT_SMItoVcoeff;
extern const ModelOptionDef OPT_SMginhDecay;
extern const ModelOptionDef OPT_SMmaxWinV;
//@}

//! Command-line options for TaskRelevanceMap
//@{

//! TaskRelevanceMap options
extern const ModelOptionCateg MOC_TRM;

extern const ModelOptionDef OPT_TaskRelevanceMapType;
extern const ModelOptionDef OPT_TRMsaveResults;
extern const ModelOptionDef OPT_LearnTRM;
extern const ModelOptionDef OPT_LearnUpdateTRM;
extern const ModelOptionDef OPT_BiasTRM;
extern const ModelOptionDef OPT_TRMKillStaticThresh;
extern const ModelOptionDef OPT_TRMKillStaticCoeff;
extern const ModelOptionDef OPT_TRMkillN;
extern const ModelOptionDef OPT_TRMClusterCenterFile;
extern const ModelOptionDef OPT_TRMTemplateDir;
extern const ModelOptionDef OPT_TRMGistPCADims;
extern const ModelOptionDef OPT_TRMGistPCAMatrix;
extern const ModelOptionDef OPT_TRMCacheSize;
extern const ModelOptionDef OPT_TRMUpdatePeriod;
extern const ModelOptionDef OPT_TRMTigsMatrix;
extern const ModelOptionDef OPT_TRMTigs2Matrix;
extern const ModelOptionDef OPT_TRMImgPCADims;
extern const ModelOptionDef OPT_TRMImgPCAMatrix;
extern const ModelOptionDef OPT_TRMSocialRegionFName;
//@}

//! Command-line options for AttentionGuidanceMap
//@{

//! AttentionGuidanceMap options
extern const ModelOptionCateg MOC_AGM;

extern const ModelOptionDef OPT_AttentionGuidanceMapType;
extern const ModelOptionDef OPT_AGMsaveResults;

//! used by AttentionGuidanceMapSC and AttentionGuidanceMapNF
extern const ModelOptionDef OPT_AGMoutputRate;

//@}
//! AttentionGate options
extern const ModelOptionCateg MOC_AG;

extern const ModelOptionDef OPT_AttentionGateType;
extern const ModelOptionDef OPT_AttentionGateStageOneType;
extern const ModelOptionDef OPT_AttentionGateStageTwoType;
extern const ModelOptionDef OPT_AttentionGateStageTwoEpochs;
extern const ModelOptionDef OPT_AGsaveResults;
extern const ModelOptionDef OPT_AGTargetFrame;
extern const ModelOptionDef OPT_AGMaskFile;
//@}

//! Command-line options for WinnerTakeAll
//@{

//! WinnerTakeAll options
extern const ModelOptionCateg MOC_WTA;

extern const ModelOptionDef OPT_WinnerTakeAllType;
extern const ModelOptionDef OPT_WTAsaveResults;
extern const ModelOptionDef OPT_WTAuseSacSuppress;
extern const ModelOptionDef OPT_WTAuseBlinkSuppress;
extern const ModelOptionDef OPT_WinnerTakeAllGreedyThreshFac;
//@}

//! Command-line options for SaccadeController and derivatives
//@{

//! SaccadeController-related options
extern const ModelOptionCateg MOC_SAC;

extern const ModelOptionDef OPT_SCeyeInitialPosition;
extern const ModelOptionDef OPT_SCheadInitialPosition;
extern const ModelOptionDef OPT_SCeyeMinSacLen;
extern const ModelOptionDef OPT_SCheadMinSacLen;
extern const ModelOptionDef OPT_SCeyeSpringK;
extern const ModelOptionDef OPT_SCeyeFrictionMu;
extern const ModelOptionDef OPT_SCeyeThreshMinOvert;
extern const ModelOptionDef OPT_SCeyeThreshMaxCovert;
extern const ModelOptionDef OPT_SCeyeThreshMinNum;
extern const ModelOptionDef OPT_SCeyeThreshSalWeigh;
extern const ModelOptionDef OPT_SCeyeMaxIdleSecs;
extern const ModelOptionDef OPT_SCheadSpringK;
extern const ModelOptionDef OPT_SCheadFrictionMu;
extern const ModelOptionDef OPT_SCheadThreshMinOvert;
extern const ModelOptionDef OPT_SCheadThreshMaxCovert;
extern const ModelOptionDef OPT_SCheadThreshMinNum;
extern const ModelOptionDef OPT_SCheadThreshSalWeigh;
extern const ModelOptionDef OPT_SCheadMaxIdleSecs;
extern const ModelOptionDef OPT_SCeyeBlinkWaitTime;
extern const ModelOptionDef OPT_SCeyeBlinkDuration;

extern const ModelOptionDef OPT_EyeHeadControllerType;
extern const ModelOptionDef OPT_SaccadeControllerEyeType;
extern const ModelOptionDef OPT_SaccadeControllerHeadType;

extern const ModelOptionDef OPT_EHCeyeTrackConfig;

//! HandController-related options
extern const ModelOptionCateg MOC_HAND;

extern const ModelOptionDef OPT_HandControllerType;
extern const ModelOptionDef OPT_HandConfig;

//@}

//! Command-line options for GistEstimator
//@{

//! GistEstimator Options
extern const ModelOptionCateg MOC_GE;

extern const ModelOptionDef OPT_GistEstimatorType;
extern const ModelOptionDef OPT_GistCenterSurroundFlag;
extern const ModelOptionDef OPT_SaveGistFlag;
//@}

//! Command-line options for VisualBuffer
//@{

//! VisualBuffer options
extern const ModelOptionCateg MOC_VB;

extern const ModelOptionDef OPT_VisualBufferType;
extern const ModelOptionDef OPT_VBignoreBoring;
extern const ModelOptionDef OPT_VBobjectBased;
extern const ModelOptionDef OPT_VBdims;
extern const ModelOptionDef OPT_VBtimePeriod;
extern const ModelOptionDef OPT_VBdecayFactor;
extern const ModelOptionDef OPT_VBmaxNormType;
//@}

//! Miscellaneous shared command-line options for neuro classes
//@{
extern const ModelOptionDef OPT_HeadMarkerRadius;
extern const ModelOptionDef OPT_MultiRetinaDepth;
extern const ModelOptionDef OPT_PixelsPerDegree;
//@}

//! Neuro-related command-line option aliases
//@{
extern const ModelOptionDef OPT_ALIAStop5;
extern const ModelOptionDef OPT_ALIASinitialVCO;
extern const ModelOptionDef OPT_ALIASmovie;
extern const ModelOptionDef OPT_ALIASmovieFast;
extern const ModelOptionDef OPT_ALIASmoviefov;
extern const ModelOptionDef OPT_ALIASmovieeyehead;
extern const ModelOptionDef OPT_ALIASmovieanim;
extern const ModelOptionDef OPT_ALIASsurprise;
extern const ModelOptionDef OPT_ALIASsurpriseRSVP;
extern const ModelOptionDef OPT_ALIASeyeCompare;
extern const ModelOptionDef OPT_ALIASeyeDisplay;
extern const ModelOptionDef OPT_ALIASeyeMap;
extern const ModelOptionDef OPT_ALIASeyeHandDisplay;
extern const ModelOptionDef OPT_ALIASbestMultiSpectralResidual;
extern const ModelOptionDef OPT_ALIASoptiGainTrain;
extern const ModelOptionDef OPT_ALIASoptiGainBias;
extern const ModelOptionDef OPT_ALIASkinectDemo;

//! request all of the above-named OPT_ALIAS options
void REQUEST_OPTIONALIAS_NEURO(OptionManager& m);
//@}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_NEUROOPTS_H_DEFINED
