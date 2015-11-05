/*!@file Channels/VisualFeatures.H definitions of known visual features */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2002 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/VisualFeatures.H $
// $Id: VisualFeatures.H 14655 2011-03-30 23:42:00Z pez $
//

#ifndef VISUALFEATURES_H_DEFINED
#define VISUALFEATURES_H_DEFINED

#include "Util/Assert.H"

#include <string>

//! Various features that can be represented
/*! We assign a specific number to each definition to speedup name printing */
enum VisualFeature
{
  UNKNOWN          = 0,            //!< Unknown feature type
  COMPOSITECOLOR   = 1,            //!< composite Color channel
  COLOR            = 2,            //!< double opponent Color complex feature
  SOCOLOR          = 3,            //!< single opponent Color complex feature
  RG               = 4,            //!< Red/green double opponency
  BY               = 5,            //!< Blue/yellow double-opponency
  SORG             = 6,            //!< Red/green single opponency
  SOGR             = 7,            //!< Green/Red single opponency
  SOBY             = 8,            //!< Blue/Yellow single opponency
  SOYB             = 9,            //!< Yellow/Blue single opponency
  R                = 10,           //!< Red channel
  G                = 11,           //!< green channel
  B                = 12,           //!< Blue channel
  Y                = 13,           //!< yellow channel
  INTENS           = 14,           //!< Intensity center-surround
  FLICKER          = 15,           //!< Flicker center-surround
  ORI              = 16,           //!< Orientation contrast
  TEMPLATE         = 17,           //!< Template matching
  COLBAND          = 18,           //!< Color band
  MOTION           = 19,           //!< Motion channel
  CONVOLVE         = 20,           //!< Convolution channel
  LJUNCTION        = 21,           //!< L junction channel
  TJUNCTION        = 22,           //!< T junction channel
  XJUNCTION        = 23,           //!< X junction channel
  ENDPOINT         = 24,           //!< End Point channel
  RGBCONVOLVE      = 25,           //!< Color convolution channel
  ENTROPY          = 26,           //!< Local entropy channel
  DUMMY            = 27,           //!< Dummy channel
  PN03CONTRAST     = 28,           //!< Local contrast (Parkhurst&Niebur 2003)
  DISPARITY        = 29,           //!< Disparity Channel
  STEREO           = 30,           //!< Stereo channel
  INFORMATION      = 31,           //!< Information channel
  RECOGNITION      = 32,           //!< Recognition channel
  HUE              = 33,           //!< Hue channel
  DIRECTFEED       = 34,           //!< DirectFeed channel
  MULTIDIRECTFEED  = 35,           //!< MultiDirectFeed channel
  VARIANCE         = 36,           //!< Variance channel
  TCORR            = 37,           //!< Tcorr (temporal correlation) channel
  SCORR            = 38,           //!< Scorr (spatial correlation) channel
  INTBAND          = 39,           //!< Intensity bands
  SIFT             = 40,           //!< SIFT keypoint channel
  H2SV             = 41,           //!< H2SV
  H1               = 42,           //!< H2SV Hue1 or CIE Lab A Red/Green
  H2               = 43,           //!< H2SV Hue2 or CIE Lab B Yellow/Blue
  SAT              = 44,           //!< H2SV Saturation
  VAL              = 45,           //!< H2SV Value Intensity or CIE Lab L
  CIELAB           = 46,           //!< CIE Lab
  GR               = 47,           //!< green/Red double opponency
  YB               = 48,           //!< yellow/Blue double-opponency
  FACE             = 49,           //!< face detection
  DEPTH            = 50,           //!< Depth
  DCOLOR           = 51,           //!< D component of DKL color
  KCOLOR           = 52,           //!< K component of DKL color
  LCOLOR           = 53,           //!< L component of DKL color
  DKLCOLOR         = 54,           //!< DKL color complex channel
  FOE              = 55,
  MST              = 56,
  FOEMST           = 57,
  FOREGROUND       = 58,           //!< Foreground probability channel
  MICHELSON        = 59,           //!< Michelson contrast
  IMGZCOLOR        = 60,           //!< Imagize 3-channel silicon retina
  IMGZALPHA        = 61,           //!< Imagize alpha channel
  IMGZBETA         = 62,           //!< Imagize beta channel
  IMGZLED          = 63,           //!< Imagize LED channel
  FLKNBACK         = 64,           //!< Flicker N-Back channel
  MOTIONSPATIOTEMPORAL = 65,       //!< Motion SpatioTemporal channel
  MOTIONOPTICALFLOW    = 66,       //!< Motion Optical Flow   channel
  DEPTHMOTION          = 67,       //!< Depth Motion channel
  MOVINGOBJECTDETECTION = 68       //!< Moving Object Detection channel
  // if you add any to this list, be sure to update the following below:
  // NBVISUALFEATURES [add 1]
  // featureName() [add a new name for your feature at end of list]
  // featureHierarchyLevel [add a new entry for your feature]
};

//! Number of visual features
#define NBVISUALFEATURES 69

//! Various types of feature representations
/*! We assign a specific number to each definition to speedup name printing */
enum VisualFeatureType
{
  RAW      = 0, //!< Raw pixel data
  RAW_CS   = 1, //!< Center-surround data before maxnormalization
  NORM_CS  = 2  //!< Center-surround data after maxnormalization
};
//!< Number of visual feature types:
#define NBVISUALFEATURETYPES 3

enum VisualFeatureHierarchy
{
  V1 = 0, //!< V1 channel in visual hierarchy
  V2 = 1, //!< V2 channel in visual hierarchy
  V4 = 2  //!< V4 channel in visual hierarchy
};
//!< Number of levels in the visual hierarchy
#define NBVISUALFEATUREHIERARCHY 3

//! Access feature name in clear
inline const char *featureName(const VisualFeature f);

//! Access feature type name in clear
inline const char *featureTypeName(const VisualFeatureType t);

//! Access feature level in the visual hierarchy
inline VisualFeatureHierarchy featureHierarchyLevel(const VisualFeature f);

//! Access feature visual hierarchy name in clear
inline const char *featureHierarchyName(const VisualFeatureHierarchy h);

// ######################################################################
// ######################################################################
// ######################################################################
inline const char *featureName(const VisualFeature f)
{
  static const char* n[NBVISUALFEATURES] = {
    "<Unknown>",
    "CompositeColor",
    "Color",
    "SOColor",
    "Red-Green",
    "Blue-Yellow",
    "SORed-Green",
    "SOGreen-Red",
    "SOBlue-Yellow",
    "SOYellow-Blue",
    "Red",
    "Green",
    "Blue",
    "Yellow",
    "Intensity",
    "Flicker",
    "Orientation",
    "Template",
    "ColorBand",
    "Motion",
    "Convolve",
    "L-Junction",
    "T-Junction",
    "X-Junction",
    "EndPoint",
    "RGBConvolve",
    "Entropy",
    "Dummy",
    "PN03contrast",
    "Disparity",
    "Stereo",
    "Information",
    "Recognition",
    "Hue",
    "DirectFeed",
    "MultiDirectFeed",
    "Variance",
    "Tcorr",
    "Scorr",
    "IntensityBand",
    "SIFT",
    "H2SV",
    "Hue1",
    "Hue2",
    "Saturation",
    "ValueIntensity",
    "CIELab",
    "GreenRed",
    "YellowBlue",
    "Face",
    "Depth",
    "Dcolor",
    "Kcolor",
    "Lcolor",
    "DKLcolor",
    "Foe",
    "MSsT",
    "FOEMSTtt",
    "Foreground",
    "Michelson",
    "ImagizeColor",
    "ImagizeAlpha",
    "ImagizeBeta",
    "ImagizeLED",
    "FlickerNBack",
    "MotionSpatioTemporal",
    "MotionOpticalFlow",
    "DepthMotion",
    "MovingObjectDetection"
  };
  ASSERT(int(f) < NBVISUALFEATURES);
  return n[int(f)];
}

// ######################################################################
inline const char *featureTypeName(const VisualFeatureType t)
{
  static const char n[NBVISUALFEATURETYPES][27] = {
    "Raw", "Raw Center-Surround", "Normalized Center-Surround" };
  return n[int(t)];
}

// ######################################################################
inline VisualFeatureHierarchy featureHierarchyLevel(const VisualFeature f)
{
  static const VisualFeatureHierarchy n[NBVISUALFEATURES] = {
    V1,    // UNKNOWN
    V1,    // COMPOSITECOLOR
    V1,    // COLOR
    V1,    // SOCOLOR
    V1,    // RG
    V1,    // BY
    V1,    // SORG
    V1,    // SOGR
    V1,    // SOBY
    V1,    // SOYB
    V1,    // R
    V1,    // G
    V1,    // B
    V1,    // Y
    V1,    // INTENS
    V1,    // FLICKER
    V1,    // ORI
    V1,    // TEMPLATE
    V1,    // COLBAND
    V1,    // MOTION
    V1,    // CONVOLVE
    V2,    // LJUNCTION
    V2,    // TJUNCTION
    V2,    // XJUNCTION
    V2,    // ENDPOINT
    V1,    // RGBCONVOLVE
    V1,    // ENTROPY
    V1,    // DUMMY
    V1,    // PN03CONTRAST
    V1,    // DISPARITY
    V1,    // STEREO
    V1,    // INFORMATION
    V2,    // RECOGNITION
    V1,    // Hue
    V1,    // DirectFeed
    V1,    // MultiDirectFeed
    V1,    // Variance
    V1,    // Tcorr
    V1,    // Scorr
    V1,    // Intensity bands
    V1,    // SIFT
    V1,    // H2SV
    V1,    // H2SV Hue1
    V1,    // H2SV Hue2
    V1,    // H2SV Saturation
    V1,    // H2SV Value Intensity
    V1,    // CIE Lab
    V1,    // GR
    V1,    // YB
    V1,    // Face Detector
    V1,    // Depth
    V1,    // Dcolor
    V1,    // Kcolor
    V1,    // Lcolor
    V1,    // DKLcolor
    V2,    // Foe
    V2,    // MSsT
    V2,    // FOEMSTtt
    V1,    // Foreground
    V1,    // Michelson
    V1,    // Imagize color
    V1,    // Imagize alpha
    V1,    // Imagize beta
    V1,    // Imagize LED
    V1,    // FlickerNBack
    V1,    // MotionSpatioTemporal    
    V1,    // MotionOpticalFlow
    V1,    // Depth Motion
    V1,    // Moving Object Detection
  };
  return n[int(f)];
}

// ######################################################################
inline const char *featureHierarchyName(const VisualFeatureHierarchy h)
{
  static const char n[NBVISUALFEATUREHIERARCHY][3] = { "V1", "V2", "V4" };
  return n[int(h)];
}


//! VisualFeature overload
/*! Format is "name" as defined by featureName() in VisualFeatures.H */
std::string convertToString(const VisualFeature val);
//! VisualFeature overload
/*! Format is "name" as defined by featureName() in VisualFeatures.H */
void convertFromString(const std::string& str, VisualFeature& val);


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
