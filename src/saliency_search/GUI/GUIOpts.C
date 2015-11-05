/*!@file GUI/GUIOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/GUIOpts.C $
// $Id: GUIOpts.C 13123 2010-04-01 22:25:09Z dberg $
//

#ifndef GUI_GUIOPTS_C_DEFINED
#define GUI_GUIOPTS_C_DEFINED

#include "GUI/GUIOpts.H"

#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"
#include "Image/Rectangle.H"

const ModelOptionCateg MOC_SDLDISP = {
  MOC_SORTPRI_2, "SDLdisplay-Related Options" };

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


// #################### SDLdisplay options:
// Used by: SDLdisplay
const ModelOptionDef OPT_SDLdisplayDims =
  { MODOPT_ARG(Dims), "SDLdisplayDims", &MOC_SDLDISP, OPTEXP_CORE,
    "SDL display screen dimensions",
    "sdl-dims", '\0', "<width>x<height>", "640x480" };

// Used by: SDLdisplay
const ModelOptionDef OPT_SDLdisplayPriority =
  { MODOPT_ARG(int), "SDLdisplayPriority", &MOC_SDLDISP, OPTEXP_CORE,
    "Priority to run at in SCHED_FIFO mode (need to run as root), or 0 for "
    "normal process scheduling and normal priority. Note that in priority 0 "
    "display timing is in no way guaranteed, and timestamps for display "
    "events are unreliable by up to +/- 50ms.",
    "sdl-priority", '\0', "<int>", "80" };

// Used by: SDLdisplay
const ModelOptionDef OPT_SDLdisplayRefreshUsec =
  { MODOPT_ARG(float), "SDLdisplayRefreshUsec", &MOC_SDLDISP, OPTEXP_CORE,
    "Desired refresh delay in microseconds. All screen refresh operations "
    "will enforce that at least 90% of this delay has occurred, but it "
    "could be more depending on the refresh rate of the video mode used.",
    "sdl-refresh", '\0', "<float>", "33333.3333" };

// Used by: SDLdisplay
const ModelOptionDef OPT_SDLdisplayFullscreen =
  { MODOPT_FLAG, "SDLdisplayFullscreen", &MOC_SDLDISP, OPTEXP_CORE,
    "Whether to run the SDLdisplay in a fullscreen window.",
    "fs", '\0', "<bool>", "true" };

// Used by: SDLdisplay
const ModelOptionDef OPT_SDLdisplayVBlankKludge =
  { MODOPT_ARG(uint), "SDLdisplayVBlankKludge", &MOC_SDLDISP, OPTEXP_CORE,
    "On some hardware, waiting for the vertical retrace by polling the "
    "VGA registers just does not work (especially in dual-screen setups). "
    "If non-zero, this parameter will enable a workaround, whereby we just "
    "hard-wait for the exact desired framerate, and just send images "
    "asynchronously to the display at that rate. On nVidia boards this "
    "yields good results where movies play at the exact desired framerate "
    "with no tearing, just we are not guaranteed that each frame will be "
    "exactly synchronized to the vertical refresh. The non-zero value is the "
    "number of microseconds that our busy-waiting code takes to run on your "
    "machine, and has to be adjusted on a case-by-case basis until you obtain "
    "your exact desired framerate. For example, on a 2.26GHz Macbook a "
    "value of 37 yields a solid 30.000 frames/s in the psycho-movie "
    "application displaying 1280x1024 movies. "
    "Larger values will decrease your framerate and vice-versa.",
    "sdl-vblank-kludge", '\0', "<uint>", "0" };


// Used by: SDLdisplay
const ModelOptionDef OPT_SDLdisplaySyncRect  =
  { MODOPT_ARG(Rectangle), "SDLdisplaySyncRect", &MOC_SDLDISP, OPTEXP_CORE,
    "Place a small patch of alternating black and white pixels over the "
    "video frame. This used for syncing with an external system using a "
    "photodiode. Rectangle coordinates define the pixel positions for left, "
    "top, right and bottom corners.", "sdl-sync-rect", '\0', "<Rectangle>", 
    "0,0,0,0"};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_GUIOPTS_C_DEFINED
