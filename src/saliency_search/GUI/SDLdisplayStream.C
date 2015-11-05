/*!@file GUI/SDLdisplayStream.C A FrameOstream class that sends images to SDL windows */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/SDLdisplayStream.C $
// $Id: SDLdisplayStream.C 14290 2010-12-01 21:44:03Z itti $
//

#ifndef GUI_SDLDISPLAYSTREAM_C_DEFINED
#define GUI_SDLDISPLAYSTREAM_C_DEFINED

#ifdef HAVE_SDL_SDL_H

#include "GUI/SDLdisplayStream.H"

#include "Component/GlobalOpts.H" // for OPT_TestMode
#include "GUI/SDLdisplay.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/Assert.H"

#include <unistd.h> // for sleep()

struct SDLdisplayStream::WindowMap
{
  WindowMap()
    :
    win(),
    fakesdl(),
    pressedEscape(false)
  {}

  nub::soft_ref<SDLdisplay> win;
  nub::soft_ref<SDLdisplay> fakesdl; // to export command-line options
  bool pressedEscape;
};

SDLdisplayStream::SDLdisplayStream(OptionManager& mgr)
  :
  FrameOstream(mgr, "Image Display Stream", "SDLdisplayStream"),
  itsTestMode(&OPT_TestMode, this),
  wmap(0)
{
  wmap = new WindowMap;
  wmap->fakesdl.reset(new SDLdisplay(mgr));
  this->addSubComponent(wmap->fakesdl);
}

SDLdisplayStream::~SDLdisplayStream()
{
  delete wmap;
}

void SDLdisplayStream::writeFrame(const GenericFrame& frame,
                                    const std::string& shortname,
                                    const FrameInfo& auxinfo)
{
  if (itsTestMode.getVal() == true)
    // don't put up any windows in test mode
    return;

  ASSERT(wmap != 0);

  if (wmap->pressedEscape == true)
    // don't put up any more windows if the user has already pressed
    // ESC once
    return;

  if (!wmap->win.is_valid())
    {
      wmap->win.reset
        (new SDLdisplay(this->getManager(), shortname, shortname));

      wmap->win->exportOptions(MC_RECURSE);
      wmap->win->setModelParamVal("SDLdisplayDims", frame.getDims());
      wmap->win->start();
      this->addSubComponent(wmap->win);

      if (wmap->win->getModelParamVal<bool>("SDLdisplayFullscreen"))
        // give the fullscreen window a chance to pop up:
        sleep(1);
    }

  ASSERT(wmap->win.is_valid());

  nub::ref<SDLdisplay> win(wmap->win);

  if (win->getDims() != frame.getDims())
    LFATAL("all frames sent to SDLdisplayStream must be the same size "
           "(got %dx%d, expected %dx%d)",
           frame.getDims().w(), frame.getDims().h(),
           win->getDims().w(), win->getDims().h());

  // give checkForKey() a chance to LFATAL() if ESC has been pressed
  win->checkForKey();

  switch (frame.nativeType())
    {
    case GenericFrame::NONE:
      break;

    case GenericFrame::RGB_U8:   /* fall through */
    case GenericFrame::RGBD:     /* fall through */
    case GenericFrame::RGB_F32:  /* fall through */
    case GenericFrame::GRAY_U8:  /* fall through */
    case GenericFrame::GRAY_F32:
      win->displayImage(frame.asRgb(), false, PixRGB<byte>(0,0,0),
                        -1, true);
      break;

    case GenericFrame::VIDEO:
      if (!win->supportsVideoOverlay(frame.asVideo().getMode()))
        {
          win->displayImage(frame.asRgb(), false, PixRGB<byte>(0,0,0),
                            -1, true);
        }
      else
        {
          if (!win->hasYUVoverlay())
            win->createVideoOverlay(frame.asVideo().getMode());
          win->displayVideoOverlay(frame.asVideo(), -1,
                                   SDLdisplay::NEXT_VSYNC);
        }
      break;

    case GenericFrame::RGB_U16:
      break;
    case GenericFrame::GRAY_U16:
      break;
    }
}

// ######################################################################
bool SDLdisplayStream::isVoid() const
{
  return wmap && wmap->pressedEscape;
}

// ######################################################################
void SDLdisplayStream::start1()
{
  // forget about our fake sdl display (which we have only used to
  // export sdl-related command-line options) before it tries to pop
  // up a window:
  this->removeSubComponent(*wmap->fakesdl);
  wmap->fakesdl.reset(0);
}

// ######################################################################
void SDLdisplayStream::closeStream(const std::string& shortname)
{
  if (this->wmap->win.is_valid())
    {
      this->wmap->win->stop();
      this->wmap->win.reset(0);
    }
}

#endif // HAVE_SDL_SDL_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_SDLDISPLAYSTREAM_C_DEFINED
