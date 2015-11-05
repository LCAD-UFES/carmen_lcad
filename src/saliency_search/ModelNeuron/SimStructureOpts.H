/*!@file ModelNeuron/SimStructureOpts.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SimStructureOpts.H $
// $Id: SimStructureOpts.H 15310 2012-06-01 02:29:24Z itti $
//


#ifndef MODELNEURON_SIMSTRUCTUREOPTS_H_DEFINED
#define MODELNEURON_SIMSTRUCTUREOPTS_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

class OptionManager;
struct ModelOptionCateg;
struct ModelOptionDef;

//! Command-line options for SC and SCtypes
//@{
extern const ModelOptionCateg MOC_SC;

//for SC simulations options 
extern const ModelOptionDef OPT_SCSimTimeStep;
extern const ModelOptionDef OPT_SCDims;
extern const ModelOptionDef OPT_SCBUInpGain;
extern const ModelOptionDef OPT_SCTDInpGain;
extern const ModelOptionDef OPT_SCInpGain;

extern const ModelOptionCateg MOC_SCDisplay;

//probe and display options
extern const ModelOptionDef OPT_SCSaveResults;
extern const ModelOptionDef OPT_SCProbe;
extern const ModelOptionDef OPT_SCPlotLength;
extern const ModelOptionDef OPT_SC2DPlotDepth;
extern const ModelOptionDef OPT_SCProbeDepth;
extern const ModelOptionDef OPT_SCUseDisplayOutput;
extern const ModelOptionDef OPT_SCDisplayRange;
extern const ModelOptionDef OPT_SCDisplayDecoder;
extern const ModelOptionDef OPT_SC2DPlotSize;

//!SC and NF type options
extern const ModelOptionDef OPT_SCSCType;
extern const ModelOptionDef OPT_SCNFType;
//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
#endif // NEURO_NEUROOPTS_H_DEFINED
