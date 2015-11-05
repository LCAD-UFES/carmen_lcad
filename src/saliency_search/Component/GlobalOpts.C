/*!@file Component/GlobalOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/GlobalOpts.C $
// $Id: GlobalOpts.C 9602 2008-04-10 00:16:46Z rjpeters $
//

#ifndef COMPONENT_GLOBALOPTS_C_DEFINED
#define COMPONENT_GLOBALOPTS_C_DEFINED

#include "Component/GlobalOpts.H"

#include "Component/ModelOptionDef.H"
#include "Util/fpu.H"

// #################### General model options:
// Used by: ModelManager
const ModelOptionDef OPT_ShowHelpMessage =
  { MODOPT_FLAG, "ShowHelpMessage", &MOC_GENERAL, OPTEXP_CORE,
    "Show help message and option syntax",
    "help", 'h', "", "true" };

// Used by: ModelManager
const ModelOptionDef OPT_ShowVersion =
  { MODOPT_FLAG, "ShowVersion", &MOC_GENERAL, OPTEXP_CORE,
    "Show the package name, version, and the svn repository "
    "version of the current build",
    "version", '\0', "", "false" };

// Used by: ModelManager
const ModelOptionDef OPT_ShowSvnVersion =
  { MODOPT_FLAG, "ShowSvnVersion", &MOC_GENERAL, OPTEXP_CORE,
    "Show the svn repository version of the current build",
    "svnversion", '\0', "", "false" };

// Used by: ModelManager
const ModelOptionDef OPT_CheckPristine =
  { MODOPT_FLAG, "CheckPristine", &MOC_GENERAL, OPTEXP_CORE,
    "Check if the executable was built from pristine source (i.e., "
    "all files up-to-date and not locally modified), and return an "
    "exit code of zero if true, and non-zero otherwise",
    "check-pristine", '\0', "", "false" };

// Used by: ModelManager
const ModelOptionDef OPT_DebugMode =
  { MODOPT_FLAG, "DebugMode", &MOC_GENERAL, OPTEXP_CORE,
    "Use debug mode, which, in particular, will increase the verbosity of "
    "the log messages and printout all model parameter values just before "
    "the model starts",
    "debug", 'd', "", "false" };

// Used by: ModelManager
const ModelOptionDef OPT_UsingFPE =
  { MODOPT_FLAG, "UsingFPE", &MOC_GENERAL, OPTEXP_CORE,
      "Use floating-point exceptions, which will abort execution if overflow, "
      "underflow, invalid or division by zero are encountered",
      "use-fpe", '\0', "", "true" };

// Used by: ModelManager
const ModelOptionDef OPT_FpuPrecision =
  { MODOPT_ARG(FpuPrecision), "FpuPrecision", &MOC_GENERAL, OPTEXP_CORE,
    "Specifies the precision in which the floating-point unit (FPU) "
    "should operate; default is 'extended'",
    "fpu-precision", '\0', "<single|double|extended>", "extended" };

// Used by: ModelManager
const ModelOptionDef OPT_FpuRoundingMode =
  { MODOPT_ARG(FpuRoundingMode), "FpuRoundingMode", &MOC_GENERAL, OPTEXP_CORE,
    "Specifies the rounding-mode in which the floating-point unit (FPU) "
    "should operate; options are nearest (toward nearest), down "
    "(toward -inf), up (toward +inf), zero (toward zero)",
    "fpu-rounding", '\0', "<nearest|down|up|zero>", "nearest" };

// Used by: ModelManager
const ModelOptionDef OPT_TestMode =
  { MODOPT_FLAG, "TestMode", &MOC_GENERAL, OPTEXP_CORE,
    "Use test mode, which, in particular, will turn off randomness (like "
    "--nouse-random), reset random numbers to a reproducible pseudo "
    "sequence, and turn off most displays. This is mostly useful for "
    "execution of the test suite, or any situation where you need "
    "deterministic, reproducible behavior",
    "test-mode", 'Z', "", "false" };

// Used by: ModelManager
const ModelOptionDef OPT_LoadConfigFile =
  { MODOPT_ARG_STRING, "LoadConfigFile", &MOC_GENERAL, OPTEXP_CORE,
    "Load configuration from file",
    "load-config-from", '\0', "<file.pmap>", "" };

// Used by: ModelManager
const ModelOptionDef OPT_SaveConfigFile =
  { MODOPT_ARG_STRING, "SaveConfigFile", &MOC_GENERAL, OPTEXP_CORE,
    "Save configuration to file",
    "save-config-to", '\0', "<file.pmap>", "" };

// Used by: ModelManager
const ModelOptionDef OPT_TextLogFile =
  { MODOPT_ARG_STRING, "TextLogFile", &MOC_GENERAL, OPTEXP_CORE,
    "Save text log messages to file",
    "textlog", '\0', "<file.log>", "" };

// Used by: many, including VisualCortex, Brain, SingleChannel and derivatives
const ModelOptionDef OPT_UseRandom =
  { MODOPT_FLAG, "UseRandom", &MOC_GENERAL, OPTEXP_CORE,
    "Add small amounts of random noise to various stages of model",
    "use-random", '\0', "", "true" };

// Used by: ModelManager
const ModelOptionDef OPT_ProfileOutFile =
  { MODOPT_ARG_STRING, "ProfileOutFile", &MOC_GENERAL, OPTEXP_CORE,
    "Where to save profiling information upon program exit (if 'none' "
    "is passed for the filename, then profiling information will not "
    "be saved at all)",
    "profile-out", '\0', "<filename>", "" };

// Used by: ModelManager
const ModelOptionDef OPT_LogVerb =
  { MODOPT_ARG_STRING, "LogVerb", &MOC_GENERAL, OPTEXP_CORE,
    "Verbosity of log messages displayed during execution. If 'Default' is "
    "selected, will use whatever value may have been set as default "
    "in the program.",
    "logverb", '\0', "<Debug|Info|Error|Fatal|Default>", "Default" };

// Used by: ModelManager
const ModelOptionDef OPT_EchoArgs =
  { MODOPT_FLAG, "EchoArgs", &MOC_GENERAL, OPTEXP_CORE,
    "Echo all the command-line arguments to stdout after "
    "command-line parsing is done.",
    "echo-args", '\0', "", "false" };

// Used by: ModelManager
const ModelOptionDef OPT_MemCaching =
  { MODOPT_FLAG, "MemCaching", &MOC_GENERAL, OPTEXP_CORE,
    "Whether to cache memory allocations of commonly-used sizes.",
    "mem-caching", '\0', "", "true" };


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_GLOBALOPTS_C_DEFINED
