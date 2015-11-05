/**
   \file  Robots/LoBot/config/LoCommonOpts.C
   \brief Common options for all Lobot/Robolocust related programs.
*/

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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/config/LoCommonOpts.C $
// $Id: LoCommonOpts.C 11256 2009-05-30 01:46:10Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/config/LoCommonOpts.H"
#include "Robots/LoBot/config/LoDefaults.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------ CONFIG FILE OPTIONS --------------------------

const ModelOptionCateg MOC_CONFIG = {
  MOC_SORTPRI_2, "Controlling Lobot config file parameters"
} ;

const ModelOptionDef OPT_ConfigFile = {
   MODOPT_ARG_STRING, "ConfigFile", & MOC_CONFIG, OPTEXP_CORE,
   "This option specifies the configuration file that contains various\n"
   "tweaks and settings that different parts of the Robolocust programs\n"
   "use.\n",
   "config-file", '\0', "config-file-name",
   LOBOT_DEFAULT_CONFIG_FILE_NAME,
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
