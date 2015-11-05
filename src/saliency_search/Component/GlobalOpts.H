/*!@file Component/GlobalOpts.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/GlobalOpts.H $
// $Id: GlobalOpts.H 9602 2008-04-10 00:16:46Z rjpeters $
//

#ifndef COMPONENT_GLOBALOPTS_H_DEFINED
#define COMPONENT_GLOBALOPTS_H_DEFINED

struct ModelOptionDef;

//! Global command-line options for ModelManager et al
//@{
extern const ModelOptionDef OPT_ShowHelpMessage;
extern const ModelOptionDef OPT_ShowVersion;
extern const ModelOptionDef OPT_ShowSvnVersion;
extern const ModelOptionDef OPT_CheckPristine;
extern const ModelOptionDef OPT_DebugMode;
extern const ModelOptionDef OPT_UsingFPE;
extern const ModelOptionDef OPT_FpuPrecision;
extern const ModelOptionDef OPT_FpuRoundingMode;
extern const ModelOptionDef OPT_TestMode;
extern const ModelOptionDef OPT_LoadConfigFile;
extern const ModelOptionDef OPT_SaveConfigFile;
extern const ModelOptionDef OPT_TextLogFile;
extern const ModelOptionDef OPT_UseRandom;
extern const ModelOptionDef OPT_ProfileOutFile;
extern const ModelOptionDef OPT_LogVerb;
extern const ModelOptionDef OPT_EchoArgs;
extern const ModelOptionDef OPT_MemCaching;
//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_GLOBALOPTS_H_DEFINED
