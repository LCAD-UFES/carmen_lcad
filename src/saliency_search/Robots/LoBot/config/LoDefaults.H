/**
   \file Robots/LoBot/config/LoDefaults.H

   \brief Various fallback values for different parts of the
   lobot/Robolocust related programs.
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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/config/LoDefaults.H $
// $Id: LoDefaults.H 13811 2010-08-21 02:00:08Z mviswana $
//

#ifndef LOBOT_DEFAULT_FALLBACKS_DOT_H
#define LOBOT_DEFAULT_FALLBACKS_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/LoRegistry.H"

// INVT headers
#include "Image/Dims.H"

//----------------------- CONFIG FILE DEFAULTS --------------------------

// lobot supports a single command-line option, viz., --config-file. All
// other settings must be specified in this file.
//
// If the user does not supply the --config-file option explicitly on the
// command line, lobot will fall back to looking for this all-important
// file in the user's home directory under the following name.
#ifndef LOBOT_DEFAULT_CONFIG_FILE_NAME
   #define LOBOT_DEFAULT_CONFIG_FILE_NAME ".lobotrc"
#endif

//---------------------- FRAME GRABBING DEFAULTS ------------------------

// lobot can grab frames from FireWire cameras at one of two resolutions,
// viz., 160x120 and 320x240 (because lobot was originally written to
// work with Unibrain Fire-i cameras, which grab at 30 fps at these two
// resolutions).
//
// If the user does not explicitly set the video frame grab size in the
// lobot config file, we will fall back to using the following defaults.
#ifndef LOBOT_DEFAULT_GRAB_WIDTH
   #define LOBOT_DEFAULT_GRAB_WIDTH 320
#endif
#ifndef LOBOT_DEFAULT_GRAB_HEIGHT
   #define LOBOT_DEFAULT_GRAB_HEIGHT 240
#endif

// This is a convenience macro that puts the above two dimensions
// together in a Dims object.
#define LOBOT_DEFAULT_GRAB_SIZE Dims(LOBOT_DEFAULT_GRAB_WIDTH, \
                                     LOBOT_DEFAULT_GRAB_HEIGHT)

// lobot can grab frames at several different rates (as supported by
// libdc1394). This one is the default frame rate in case it is not
// explicitly specified in the config file.
//
// NOTE: Though several frame rates are supported, not all combinations
// of frame rate and size may work together.
#ifndef LOBOT_DEFAULT_GRAB_RATE
   #define LOBOT_DEFAULT_GRAB_RATE 30.0f
#endif

//--------------------------- LOCUST MODELS -----------------------------

// lobot supports several different LGMD models. The following symbol
// defines the default one to use in case the user doesn't specify this
// in the config file.
#define LOBOT_DEFAULT_LOCUST_MODEL LOLM_GABBIANI

// Each virtual locust monitors some portion of the entire input image
// combined from all the video sources. How much of the image a locust
// should monitor is specified via its FOV setting. This is the width (in
// pixels) of the portion of the input image each locust should monitor.
// The vertical extents of each locust's portion always spans the entire
// height of the input image.
//
// The following symbol specifies the default width of a locust's FOV in
// case users don't set it in the config file. Each LGMD model can use a
// different value for the locust FOV. Users may achieve this by
// specifying the appropriate setting in each LGMD model's section of the
// config file.
#ifndef LOBOT_DEFAULT_LOCUST_FOV
   #define LOBOT_DEFAULT_LOCUST_FOV 30
#endif

//-----------------------------------------------------------------------

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
