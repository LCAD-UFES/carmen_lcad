/*!@file Psycho/PsychoOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoOpts.C $
// $Id: PsychoOpts.C 13632 2010-06-29 06:15:50Z dberg $
//

#ifndef PSYCHO_PSYCHOOPTS_C_DEFINED
#define PSYCHO_PSYCHOOPTS_C_DEFINED

#include "Psycho/PsychoOpts.H"

#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"
#include "Image/Range.H"
#include "Util/SimTime.H"

const ModelOptionCateg MOC_PSYCHODISP = {
  MOC_SORTPRI_2, "PsychoDisplay-Related Options" };

const ModelOptionCateg MOC_EYETRACK = {
  MOC_SORTPRI_2, "EyeTracker-Related Options" };

const ModelOptionCateg MOC_PSYCHOPROG = {
  MOC_SORTPRI_2, "psycho physics programs related Options" };

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

// #################### PsychoDisplay options:

// Used by: PsychoDisplay
const ModelOptionDef OPT_IscanSerialPortDevice =
  { MODOPT_ARG_STRING, "IscanSerialPortDevice", &MOC_PSYCHODISP, OPTEXP_CORE,
    "Device file for serial-port connection to the ISCAN eyetracker",
    "iscan-serial-port", '\0', "<devicefile>", "/dev/ttyS0" };

// Used by: EyeTrackerConfigurator
const ModelOptionDef OPT_EyeTrackerType =
  { MODOPT_ARG_STRING, "EyeTrackerType", &MOC_EYETRACK, OPTEXP_CORE,
    "Type of EyeTracker to use. ISCAN has been tested with an ISCAN, "
    "Inc. model RK-464, 240Hz video-based eye-tracker (trigger can be "
    "over serial or parallel ports, see available options after you "
    "have selected this tracker type). DML is the monkey eye-tracker "
    "model used in Doug Munoz' Lab at Queen's University, Kingston, ON, "
    "Canada, based on a scleral search coil (Trigger is over the parallel "
    "port, no data streaming). TIL is the monkey eye-tracker model used in "
    "Tadashi Isa's Lab at the National Institute for Physiological Science, "
    "Okazaki, Japan, also based on a scleral search coil. EL is for "
    "EyeLink-II.",
    "et-type", '\0', "<None|ISCAN|DML|UDP|TIL|EL>", "None" };

// Used by: EyeTrackerISCAN
const ModelOptionDef OPT_EyeTrackerParTrig =
  { MODOPT_FLAG, "EyeTrackerParTrig", &MOC_EYETRACK, OPTEXP_CORE,
    "Use parallel trigger mode. If this option is selected, the ISCAN "
    "tracker will be started/stopped by sending it signals over the "
    "parallel port, while the serial port will be reserved for "
    "streaming eye position data back to us. Otherwise, the tracker "
    "will be started/stopped over the serial line, and no data "
    "streaming will be possible.",
    "et-partrig", '\0', "<bool>", "true" };

// Used by: EyeTrackerISCAN
const ModelOptionDef OPT_EyeTrackerSerDev =
  { MODOPT_ARG_STRING, "EyeTrackerSerDev", &MOC_EYETRACK, OPTEXP_CORE,
    "Device file for serial-port connection to the eye tracker.",
    "et-serdev", '\0', "<devicefile>", "/dev/ttyS0" };

// Used by: EyeTrackerISCAN, EyeTrackerDML, EyeTrackerTIL
const ModelOptionDef OPT_EyeTrackerParDev =
  { MODOPT_ARG_STRING, "EyeTrackerParDev", &MOC_EYETRACK, OPTEXP_CORE,
    "Device file for parallel-port connection to the eye tracker.",
    "et-pardev", '\0', "<devicefile>", "/dev/parport0" };

// Used by: EyeTrackerEyeLink
const ModelOptionDef OPT_EyeTrackerEDFfname =
  { MODOPT_ARG_STRING, "EyeTrackerEDFfname", &MOC_EYETRACK, OPTEXP_CORE,
    "Name of EDF file to use for EyeLink data collection.",
    "et-edf-fname", '\0', "<filename>", "" };

// Used by: various psychophysics programs
const ModelOptionDef OPT_Hflip =
  { MODOPT_FLAG, "hflip", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "Flip input images horizontally.",
    "hflip", '\0', "<bool>", "false" };

// Used by: various psychophysics programs
const ModelOptionDef OPT_FixSize =
  { MODOPT_ARG(float), "fixsize", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "fixation point size in degrees of visual angle",
    "fixsize", '\0', "<float>", "1" };

// Used by: various psychophysics programs
const ModelOptionDef OPT_Ppd =
  { MODOPT_ARG(float), "ppd", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "pixels per degree of visual angle",
    "ppd", '\0', "<float>", "11" };

// Used by: various psychophysics programs
const ModelOptionDef OPT_KeepFix =
  { MODOPT_FLAG, "keepfix", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "keep the fixation point on during the movie",
    "keepfix", '\0', "<bool>", "false" };

//Used by: various psychophysics programs
const ModelOptionDef OPT_DisplayTime =
  { MODOPT_ARG(SimTime), "DisplayTime", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "The amount of time to display the stimulus. Append"
    "a time <s, ms> to indicate the units", "displaytime", '\0',
    "<SimTime>", "1s" };

//Used by: various psychophysics programs
const ModelOptionDef OPT_TrialWait =
  { MODOPT_ARG(Range<int>), "TrialWait", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "Wait a random amount of time (in ms) before the start of the next"
    "trial", "waittime", '\0',
    "<min>-<max>", "0-0" };

//Used by: various psychophysics programs
const ModelOptionDef OPT_Testrun =
  { MODOPT_FLAG, "Testrun", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "This option will disable user input while running psychophysics "
    "programs so that we can test our experiments without waiting for "
    "trial triggers . ", "testrun", '\0', "<bool>", "false" };

const ModelOptionDef OPT_GrayFramePrcnt = 
  { MODOPT_ARG(int), "GrayFramePrcnt", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "If this value is set to greater than 0, that percentage of the total "
    "clip count will be added back into the movie queue as gray "
    "movies. Also see --gray-frame-range", "gray-frame-percent", '\0', "<int>", 
    "0" };

const ModelOptionDef OPT_GrayFrameRange = 
  { MODOPT_ARG(Range<int>), "GrayFrameRange", &MOC_PSYCHOPROG, OPTEXP_CORE,
    "Gray frame durations will be uniform randomly distributed between "
    "these values in milliseconds. Also see --gray-frame-percent", 
    "gray-frame-range", '\0', "<int>-<int>", "3000-5000" };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_PSYCHOOPTS_C_DEFINED
