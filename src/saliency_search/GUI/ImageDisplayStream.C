/*!@file GUI/ImageDisplayStream.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/ImageDisplayStream.C $
// $Id: ImageDisplayStream.C 14376 2011-01-11 02:44:34Z pez $
//

#ifndef GUI_IMAGEDISPLAYSTREAM_C_DEFINED
#define GUI_IMAGEDISPLAYSTREAM_C_DEFINED

#include <map>
#include <vector>

#include "Component/GlobalOpts.H" // for OPT_TestMode
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "GUI/XWinManaged.H"
#include "Image/Image.H"
#include "Image/Layout.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "rutz/shared_ptr.h"
#include "GUI/ImageDisplayStream.H"

// Used by: ImageDisplayStream
static const ModelOptionDef OPT_MaxImageDisplay =
  { MODOPT_ARG(int), "MaxImageDisplay", &MOC_OUTPUT, OPTEXP_CORE,
    "max number of images to display onscreen for any given timestep",
    "max-image-display", '\0', "<integer>", "20" };

// Used by: ImageDisplayStream
static const ModelOptionDef OPT_TraceXEvents =
  { MODOPT_FLAG, "TraceXEvents", &MOC_OUTPUT, OPTEXP_CORE,
    "print some info about each X11 event that is received",
    "trace-x-events", '\0', "", "false" };

typedef std::map<std::string, rutz::shared_ptr<XWinManaged> > map_type;

// ######################################################################
struct ImageDisplayStream::WindowMap
{
  WindowMap()
    :
    windows(),
    pressedCloseButton(false)
  {}

  XWinManaged* getWindowForFrame(const Dims& dims,
                                 const std::string& shortname,
                                 const size_t maxdisplay,
                                 const bool tracexevents)
  {
    if (!this->windows[shortname].is_valid())
      {
        // check if one more window would be too many
        if (this->windows.size() + 1 > maxdisplay)
          {
            LERROR("**** TOO MANY WINDOWS! NOT DISPLAYING IMAGE...");
            return 0;
          }

        rutz::shared_ptr<XWinManaged> win
          (new XWinManaged(dims, -1, -1, shortname.c_str()));

        win->setTraceEvents(tracexevents);

        this->windows[shortname] = win;
      }

    rutz::shared_ptr<XWinManaged> win = this->windows[shortname];

    ASSERT(win.is_valid());

    if (win->pressedCloseButton())
      this->pressedCloseButton = true;

    win->setVisible(true);

    return win.get();
  }

  map_type windows;
  bool pressedCloseButton;
};

// ######################################################################
ImageDisplayStream::ImageDisplayStream(OptionManager& mgr)
  :
  FrameOstream(mgr, "Image Display Stream", "ImageDisplayStream"),
  itsTestMode(&OPT_TestMode, this),
  itsMaxDisplay(&OPT_MaxImageDisplay, this),
  itsTraceXEvents(&OPT_TraceXEvents, this),
  wmap(0)
{}

// ######################################################################
ImageDisplayStream::~ImageDisplayStream()
{
  delete wmap;
}

// ######################################################################
void ImageDisplayStream::writeFrame(const GenericFrame& frame,
                                    const std::string& shortname,
                                    const FrameInfo& auxinfo)
{
  if (itsTestMode.getVal() == true)
    // don't put up any windows in test mode
    return;

  if (wmap == 0)
    {
      wmap = new WindowMap;
    }

  ASSERT(wmap != 0);

  XWinManaged* const win =
    wmap->getWindowForFrame(frame.getDims(), shortname,
                            size_t(itsMaxDisplay.getVal()),
                            itsTraceXEvents.getVal());

  if (win == 0)
    return;

  switch (frame.nativeType())
    {
    case GenericFrame::NONE:                                                          break;
    case GenericFrame::RGB_U8:   win->drawRgbLayout(frame.asRgbU8Layout(),0,0,true);  break;
    case GenericFrame::RGBD:     win->drawRgbLayout(frame.asRgbU8Layout(),0,0,true);  break;
    case GenericFrame::RGB_F32:  win->drawImage(frame.asRgbU8(),0,0,true);            break;
    case GenericFrame::GRAY_U8:  win->drawGrayLayout(frame.asGrayU8Layout(),0,0,true);break;
    case GenericFrame::GRAY_F32: win->drawImage(frame.asGrayU8(),0,0,true);           break;
    case GenericFrame::VIDEO:    win->drawImage(frame.asRgbU8(),0,0,true);            break;
    case GenericFrame::RGB_U16:        break;
    case GenericFrame::GRAY_U16:       break;
    }
}

// ######################################################################
bool ImageDisplayStream::isVoid() const
{
  return wmap && wmap->pressedCloseButton;
}

// ######################################################################
void ImageDisplayStream::closeStream(const std::string& shortname)
{
  if (wmap)
    {
      map_type::iterator itr = wmap->windows.find(shortname);

      if (itr != wmap->windows.end())
        (*itr).second->setVisible(false);
    }
}

// ######################################################################
rutz::shared_ptr<XWinManaged>
ImageDisplayStream::getWindow(const std::string& shortname)
{
  if (wmap)
    {
      map_type::iterator itr = wmap->windows.find(shortname);

      if (itr != wmap->windows.end())
        return (*itr).second;
    }

  return rutz::shared_ptr<XWinManaged>();
}

// ######################################################################
std::vector<rutz::shared_ptr<XWinManaged> > ImageDisplayStream::getWindows()
{
  std::vector<rutz::shared_ptr<XWinManaged> > windows;
  if (wmap)
  {
    map_type::iterator itr;
    for(itr = wmap->windows.begin(); itr != wmap->windows.end(); itr++)
    {
      windows.push_back((*itr).second);
    }
  }

  return windows;

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // GUI_IMAGEDISPLAYSTREAM_C_DEFINED
