/*!@file SpaceVariant/SpaceVariantOpts.H */

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
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SpaceVariantOpts.H $
// $Id: SpaceVariantOpts.H 14513 2011-02-16 05:02:38Z dberg $
//

#ifndef SPACEVARIANT_SPACEVARIANTOPTS_H_DEFINED
#define SPACEVARIANT_SPACEVARIANTOPTS_H_DEFINED

class OptionManager;
struct ModelOptionCateg;
struct ModelOptionDef;

//! SpaceVariant module related options
extern const ModelOptionCateg MOC_SPACEVARIANT;

extern const ModelOptionDef OPT_SpaceVariantScale;
extern const ModelOptionDef OPT_SpaceVariantFoveaSize;
extern const ModelOptionDef OPT_SpaceVariantSfac;
extern const ModelOptionDef OPT_SpaceVariantBeta;
extern const ModelOptionDef OPT_SpaceVariantGain;
extern const ModelOptionDef OPT_SpaceVariantExponent;
extern const ModelOptionDef OPT_SpaceVariantOffset;
extern const ModelOptionDef OPT_SpaceVariantFovCut;
extern const ModelOptionDef OPT_SpaceVariantDims;
extern const ModelOptionDef OPT_SpaceVariantDogCenter;
extern const ModelOptionDef OPT_SpaceVariantDogSize;

extern const ModelOptionDef OPT_SpaceVariantEdgeOrient;
extern const ModelOptionDef OPT_SpaceVariantEdgeLength;
extern const ModelOptionDef OPT_SpaceVariantEdgeDensity;

extern const ModelOptionDef OPT_SpaceVariantChanScales;

extern const ModelOptionDef OPT_SpaceVariantMaxVisualAngle;
extern const ModelOptionDef OPT_SpaceVariantCellType;
//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SPACEVARIANT_SPACEVARIANTOPTS_H_DEFINED
