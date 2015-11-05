/*!@file GUI/AutomateXWin.C Automate X windows by sending keys and getting images */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/AutomateXWin.C $
// $Id: AutomateXWin.C 9129 2008-01-14 20:27:55Z lior $
//

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "GUI/AutomateXWin.H"


// shift x left by i, where i can be positive or negative
#define SHIFTL(x,i) (((i) >= 0) ? ((x) << (i)) : ((x) >> (-i)))

// ######################################################################
AutomateXWin::AutomateXWin(const char* win_name) :
  itsDisplay(NULL),
  itsScreen(0),
  itsImage(NULL),
  itsWidth(0),
  itsHeight(0),
  itsDepth(0)

{
  char *dispName = XDisplayName(NULL);
  unsigned num_tries;
  int event_base, error_base;
  int major_version, minor_version;

  itsDisplay = XOpenDisplay(dispName);
  if(!itsDisplay)
    LFATAL("XOpenDisplay(%s) failed\n", dispName);

  if (!XTestQueryExtension (itsDisplay, &event_base, &error_base, &major_version, &minor_version)) {
    XCloseDisplay(itsDisplay);
    LFATAL("XTest extension not supported on server");
  }

  LINFO("XTestQueryExtension passed.");
  LINFO("XTest information for server \"%s\":", DisplayString(itsDisplay));
  LINFO("  Major version:       %d", major_version);
  LINFO("  Minor version:       %d", minor_version);
  LINFO("  First event number:  %d", event_base);
  LINFO("  First error number:  %d", error_base);


  itsScreen = DefaultScreen(itsDisplay);

  itsRootWin = RootWindow(itsDisplay, itsScreen);

  for(num_tries=0;;)
  {
    itsWindow = XWindowByName(itsDisplay, itsRootWin, win_name);
    if(itsWindow) break;
    if(++num_tries == 100)
      LFATAL("XWindowByName\n");
    usleep(20000);
  }

  XFlush(itsDisplay);
  XSync(itsDisplay, False);
#ifdef USE_SHM
  if(!XShmQueryExtension(itsDisplay))
    LFATAL("XShmQueryExtension");
#endif
  GetWindowGeometry();

  if(!XMatchVisualInfo(itsDisplay, itsScreen, itsDepth, DirectColor, &itsVInfo))
  {
    return;
  }

}


// ######################################################################
AutomateXWin::~AutomateXWin()
{
  if(!itsImage) return;

#ifdef USE_SHM
  XShmDetach(itsDisplay, &itsShminfo);
  XDestroyImage(itsImage);
  itsImage = NULL;
  shmdt(itsShminfo.shmaddr);
#endif

}

// ######################################################################
void AutomateXWin::setFocus()
{
  XSetInputFocus(itsDisplay, itsWindow, RevertToNone, CurrentTime);
}

// ######################################################################
Image<PixRGB<byte> > AutomateXWin::getImage()
{


  DeleteImage();
#ifdef USE_SHM
  if(!itsImage)
  {
    itsImage = XShmCreateImage(itsDisplay, itsVInfo.visual,
        itsDepth,
        ZPixmap,
        NULL,
        &itsShminfo,
        itsWidth,
        itsHeight);
    if(!itsImage)
    {
      LERROR("XShmCreateImage");
      return Image<PixRGB<byte> >();
    }

    itsShminfo.shmid = shmget(IPC_PRIVATE,
        itsImage->bytes_per_line * itsImage->height,
        IPC_CREAT | 0777);
    if(itsShminfo.shmid < 0)
    {
      LERROR("shmget");
      XDestroyImage(itsImage);
      itsImage=NULL;
      return Image<PixRGB<byte> >();
    }
    itsShminfo.shmaddr = (char *) shmat(itsShminfo.shmid, 0, 0);
    if(itsShminfo.shmaddr == (char *)-1)
    {
      LERROR("shmat");
      XDestroyImage(itsImage);
      return Image<PixRGB<byte> >();
    }
    shmctl(itsShminfo.shmid, IPC_RMID, 0),
      itsImage->data = itsShminfo.shmaddr;
    itsShminfo.readOnly = False;
    XShmAttach(itsDisplay, &itsShminfo);

    XSync(itsDisplay, False);
  }
#endif
  /* If SHM failed or was disabled, try non-SHM way */
  if(!itsImage)
  {
    itsImage = XGetImage(itsDisplay, itsWindow,
        0, 0,
        itsWidth,
        itsHeight,
        AllPlanes,
        XYPixmap);
    if(!itsImage)
      LERROR("XGetImage\n");
  }
#ifdef USE_SHM
  if(!XShmGetImage(itsDisplay, itsWindow, itsImage,
        0, 0,
        AllPlanes))
    LERROR("XSHMGetImage");
#endif

  Image<PixRGB<byte> > img(itsWidth,itsHeight,NO_INIT);
  int rshift = 7 - getHighBitIndex (itsImage->red_mask);
  int gshift = 7 - getHighBitIndex (itsImage->green_mask);
  int bshift = 7 - getHighBitIndex (itsImage->blue_mask);

  for (int y=0; y<itsImage->height; y++) {
    for (int x=0; x<itsImage->width; x++) {
      unsigned long pixel = XGetPixel (itsImage,x,y);
      PixRGB<byte> pixVal(
       SHIFTL(pixel & itsImage->red_mask,rshift),
       SHIFTL(pixel & itsImage->green_mask,gshift),
       SHIFTL(pixel & itsImage->blue_mask,bshift)
      );
      img.setVal(x,y, pixVal);
    }
  }

  return img;

}

void AutomateXWin::DeleteImage(void)
{
  if(!itsImage) return;

#ifndef USE_SHM
  XDestroyImage(itsImage);
  itsImage = NULL;
#endif
}

// ######################################################################
void AutomateXWin::sendKey(const int key)
{
  XTestFakeKeyEvent(itsDisplay, key, True, CurrentTime); //key down
  XTestFakeKeyEvent(itsDisplay, key, False, CurrentTime); //key up
  XSync(itsDisplay, True);
}


// ######################################################################
// return the index of the highest bit
int AutomateXWin::getHighBitIndex (unsigned int x)
{
  int i = 0;
  while (x) {
    i++;
    x >>= 1;
  }
  return i-1;
}

// ######################################################################
void AutomateXWin::GetWindowGeometry()
{
  unsigned border_width;
  int xpos, ypos;
  Window root;

  if(!XGetGeometry(
        itsDisplay, itsWindow, &root,
        &xpos, &ypos,
        &itsWidth, &itsHeight,
        &border_width,
        &itsDepth))
  {
    LERROR("XGetGeometry\n");
  }
  LINFO("width=%u, height=%u, depth=%u\n", itsWidth, itsHeight, itsDepth);

}

// ######################################################################
Window AutomateXWin::XWindowByName(Display *display, const Window rootwin, const char *name)
{
  unsigned int num_children;
  Window *children, child, window;
  XTextProperty windowname;

  if(XGetWMName(display, rootwin, &windowname) != 0)
  {
    LINFO("Window='%s'\n", (const char *)windowname.value);
    if(!strcmp((const char *)windowname.value, name))
      return rootwin;
  }

  window = (Window) NULL;

  if(XQueryTree(display, rootwin, &child, &child, &children, &num_children))
  {
    unsigned i;
    for(i=0; i < num_children; ++i)
    {
      /* Search each child and their children. */
      window = XWindowByName(display, children[i], name);
      if(window != (Window) NULL)
        break;
    }
    if (children != (Window *)NULL)
      XFree((void *)children);
  }
  return window;
}

// ######################################################################
void AutomateXWin::XListWindows(Display *display, const Window rootwin)
{
  unsigned int num_children;
  Window *children, child;
  XTextProperty windowname;

  if(XGetWMName(display, rootwin, &windowname) != 0)
    LINFO("  '%s'\n", (const char *)windowname.value);

  if(XQueryTree(display, rootwin, &child, &child, &children, &num_children))
  {
    unsigned i;
    for(i=0; i < num_children; ++i)
    {
      /* Search each child and their children. */
      XListWindows(display, children[i]);
    }
    if (children != (Window *)NULL)
      XFree((void *)children);
  }
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
