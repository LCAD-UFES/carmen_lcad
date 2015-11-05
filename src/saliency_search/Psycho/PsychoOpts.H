/*!@file Psycho/PsychoOpts.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoOpts.H $
// $Id: PsychoOpts.H 13621 2010-06-25 07:55:59Z dberg $
//

#ifndef PSYCHO_PSYCHOOPTS_H_DEFINED
#define PSYCHO_PSYCHOOPTS_H_DEFINED

struct ModelOptionCateg;
struct ModelOptionDef;

//! Command-line options for PsychoDisplay et al
//@{

//! PsychoDisplay options
extern const ModelOptionCateg MOC_PSYCHODISP;

extern const ModelOptionDef OPT_IscanSerialPortDevice;
//@}

//! Command-line options for some psycho programs
//@{

//! Psycho program options
extern const ModelOptionCateg MOC_PSYCHOPROG;

extern const ModelOptionDef OPT_Hflip;
extern const ModelOptionDef OPT_FixSize;
extern const ModelOptionDef OPT_Ppd;
extern const ModelOptionDef OPT_KeepFix;
extern const ModelOptionDef OPT_DisplayTime;
extern const ModelOptionDef OPT_TrialWait;
extern const ModelOptionDef OPT_Testrun;
extern const ModelOptionDef OPT_GrayFramePrcnt;
extern const ModelOptionDef OPT_GrayFrameRange;
//@}

//! Command-line options for EyeTracker et al
//@{

//! EyeTracker options
extern const ModelOptionCateg MOC_EYETRACK;

extern const ModelOptionDef OPT_EyeTrackerType;
extern const ModelOptionDef OPT_EyeTrackerParTrig;
extern const ModelOptionDef OPT_EyeTrackerSerDev;
extern const ModelOptionDef OPT_EyeTrackerParDev;
extern const ModelOptionDef OPT_EyeTrackerEDFfname;
//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_PSYCHOOPTS_H_DEFINED
