/*!@file GUI/XWinManaged.C A window class with active window management */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/XWinManaged.C $
// $Id: XWinManaged.C 15393 2012-08-15 04:51:24Z kai $
//

#include "GUI/XWinManaged.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/mutex.h"

#include <stdlib.h>
#include <unistd.h>
#include <X11/Xutil.h>

namespace
{
  const int SLEEPTIME = 10000;

  const long g_event_mask = (ExposureMask |
                             ButtonPressMask | KeyPressMask | 
                             ButtonReleaseMask |PointerMotionMask);

  const unsigned int g_max_queue_len = 32;
}

// ######################################################################
void XWinManaged::init()
{
  mousePressed = false;
  this->selectInput(g_event_mask);

  deleteAtom = this->setWMProtocol("WM_DELETE_WINDOW");

  windowPresent.atomic_set(1);
  itsTraceEvents.atomic_set(0);

  if (0 != pthread_mutex_init(&itsEventsMutex, NULL))
    LFATAL("pthread_mutex_init() failed");

  if (0 != pthread_create(&eventLoopThread, NULL, &cEventLoop, this))
    LFATAL("Cannot create thread");
}

// ######################################################################
XWinManaged::XWinManaged(const Dims& dims,
                         const int x, const int y, const char* title)
  :
  XWindow(dims, x, y, title),
  toBeClosed(false)
{
  init();
}


// ######################################################################
XWinManaged::XWinManaged(const Image< PixRGB<byte> >& img,
                         const char* title)
  :
  XWindow(img.getDims(), -1, -1, title),
  toBeClosed(false)
{
  init();
  XWindow::drawImage(img);
}


// ######################################################################
XWinManaged::XWinManaged(const Image<byte>& img, const char* title)
  :
  XWindow(img.getDims(), -1, -1, title),
  toBeClosed(false)
{
  init();
  XWindow::drawImage(img);
}

// ######################################################################
XWinManaged::XWinManaged(const Image<float>& img, const char* title,
                         bool normalize)
  :
  XWindow(img.getDims(), -1, -1, title),
  toBeClosed(false)
{
  init();
  drawImage(img,0,0,normalize);
}


// ######################################################################
XWinManaged::~XWinManaged()
{
  windowPresent.atomic_set(0);

  if (0 != pthread_join(eventLoopThread, NULL))
    LERROR("Thread is not returning");

  if (0 != pthread_mutex_destroy(&itsEventsMutex))
    LERROR("pthread_mutex_destroy() failed");
}

// ######################################################################
void* XWinManaged::cEventLoop(void* xself)
{
  XWinManaged* self = static_cast<XWinManaged*>(xself);

  while (1)
    {
      if (!self->windowPresent.atomic_get())
        break;


      XEvent event;
      const Bool status1 = self->checkMaskEvent(g_event_mask, &event);

      if (True == status1)
        self->eventHandler(&event);

      // NB: XCheckWindowEvent() will NOT return ClientMessage
      // events, since those events are "unmaskable", so we need to
      // do a separate XCheckTypedWindowEvent() to pull in
      // ClientMessage events (ClientMessage events are needed to
      // get close-button notification)
      const Bool status2 = self->checkTypedEvent(ClientMessage, &event);

      if (True == status2)
        self->eventHandler(&event);

      if (False == status1 && False == status2)
        // we didn't get any event this time, so sleep before
        // looking for the next event
        usleep (SLEEPTIME);
    }
  pthread_exit(NULL);
  return 0;
}

// ######################################################################
void XWinManaged::eventHandler(XEvent* event)
{
  if (itsTraceEvents.atomic_get() > 0 || MYLOGVERB >= LOG_DEBUG)
    LINFO("%s in window '%s'...",
          describeEvent(event).c_str(), this->getTitle());

  switch(event->type)
    {

    case ClientMessage:
      {
        if (event->xclient.data.l[0] == (int)deleteAtom)
          {
            // user wants to close window - set the flag
            toBeClosed = true;
          }
      }
      break;

    case Expose:
      {
        while (this->checkMaskEvent(ExposureMask, event))
          { /* empty loop body */ }

        this->redrawImage();
      }
      break;

    case KeyPress:
      {
        GVX_MUTEX_LOCK(&itsEventsMutex);
        itsKeyEvents.push_back(event->xkey);
        if (itsKeyEvents.size() > g_max_queue_len)
          itsKeyEvents.pop_front();
      }
      break;

    case ButtonPress:
      {
        GVX_MUTEX_LOCK(&itsEventsMutex);
        mousePressed = true;

        itsButtonEvents.push_back(event->xbutton);
        if (itsButtonEvents.size() > g_max_queue_len)
          itsButtonEvents.pop_front();

        itsButtonPressedEvents.push_back(event->xbutton);
        if (itsButtonPressedEvents.size() > g_max_queue_len)
          itsButtonPressedEvents.pop_front();
      }
      break;
    case ButtonRelease:
      {
        GVX_MUTEX_LOCK(&itsEventsMutex);
        mousePressed = false;

        itsButtonEvents.push_back(event->xbutton);
        if (itsButtonEvents.size() > g_max_queue_len)
          itsButtonEvents.pop_front();

        itsButtonReleasedEvents.push_back(event->xbutton);
        if (itsButtonReleasedEvents.size() > g_max_queue_len)
          itsButtonReleasedEvents.pop_front();
      }
      break;
    case MotionNotify:
      {
        GVX_MUTEX_LOCK(&itsEventsMutex);

        itsButtonEvents.push_back(event->xbutton);
        if (itsButtonEvents.size() > g_max_queue_len)
          itsButtonEvents.pop_front();

        itsButtonMotionEvents.push_back(event->xbutton);
        if (itsButtonMotionEvents.size() > g_max_queue_len)
          itsButtonMotionEvents.pop_front();
      }
      break;
    }
}

// ######################################################################
void XWinManaged::setTraceEvents(bool on)
{
  itsTraceEvents.atomic_set(on ? 1 : 0);
}

// ######################################################################
int XWinManaged::getLastKeyPress()
{
  XKeyEvent ev;
  if (!this->getLastKeyEvent(&ev))
    return -1;

  return ev.keycode;
}

// ######################################################################
std::string XWinManaged::getLastKeyString()
{
  XKeyEvent ev;
  if (!this->getLastKeyEvent(&ev))
    return std::string();

  char buf[30];
  const int len =
    XLookupString(&ev, &buf[0], sizeof(buf), 0, 0);
  (void) len;
  buf[sizeof(buf)-1] = '\0';

  return std::string(&buf[0]);
}

// ######################################################################
KeySym XWinManaged::getLastKeySym(std::string* s)
{
  XKeyEvent ev;
  if (!this->getLastKeyEvent(&ev))
    return NoSymbol;

  char buf[30];
  KeySym keysym;
  const int len =
    XLookupString(&ev, &buf[0], sizeof(buf), &keysym, 0);
  (void) len;
  buf[sizeof(buf)-1] = '\0';

  if (s != 0)
    *s = buf;

  return keysym;
}

// ######################################################################
bool XWinManaged::getLastKeyEvent(XKeyEvent* ev)
{
  GVX_MUTEX_LOCK(&itsEventsMutex);
  if (itsKeyEvents.empty())
    return false;

  *ev = itsKeyEvents.front();
  itsKeyEvents.pop_front();
  return true;
}

// ######################################################################
Point2D<int> XWinManaged::getLastMouseClick()
{
  XButtonEvent ev;
  if (!this->getLastButtonPressedEvent(&ev))
  {
  	ev.x = -1;ev.y = -1;
  } 
  return Point2D<int>(ev.x, ev.y);
}
// ######################################################################
Point2D<int> XWinManaged::getLastMouseMotion()
{
  XButtonEvent ev;
  if (!this->getLastButtonMotionEvent(&ev))
  {
  	ev.x = -1;ev.y = -1;
  } 
  return Point2D<int>(ev.x, ev.y);
}

// ######################################################################
bool XWinManaged::getLastButtonPressedEvent(XButtonEvent* ev)
{
  GVX_MUTEX_LOCK(&itsEventsMutex);
  if (itsButtonPressedEvents.empty())
    return false;

  *ev = itsButtonPressedEvents.front();
  itsButtonPressedEvents.pop_front();
  return true;
}
// ######################################################################
bool XWinManaged::getLastButtonReleasedEvent(XButtonEvent* ev)
{
  GVX_MUTEX_LOCK(&itsEventsMutex);
  if (itsButtonReleasedEvents.empty())
    return false;

  *ev = itsButtonReleasedEvents.front();
  itsButtonReleasedEvents.pop_front();
  return true;
}
// ######################################################################
bool XWinManaged::getLastButtonMotionEvent(XButtonEvent* ev)
{
  GVX_MUTEX_LOCK(&itsEventsMutex);
  if (itsButtonMotionEvents.empty())
    return false;

  *ev = itsButtonMotionEvents.front();
  //clear all event, only keep last one
  itsButtonMotionEvents.clear();
  return true;
}
// ######################################################################
bool XWinManaged::getLastButtonEvent(XButtonEvent* ev)
{
  GVX_MUTEX_LOCK(&itsEventsMutex);
  if (itsButtonEvents.empty())
    return false;

  *ev = itsButtonEvents.front();
  itsButtonEvents.pop_front();
  return true;
}

// ######################################################################
std::string XWinManaged::describeKeyEvent(const XKeyEvent* ev) const
{
  char buf[30];
  KeySym keysym;
  XKeyEvent evcopy = *ev; // XLookupString() wants a non-const pointer
  const int len =
    XLookupString(&evcopy, &buf[0], sizeof(buf),
                  &keysym, 0);

  const char* symname = XKeysymToString(keysym);
  if (symname == 0)
    symname = "VoidSymbol";

  if (len > 1)
    return sformat("KeyPress code=%d sym=XK_%s (\"%s\")",
                   ev->keycode, symname, &buf[0]);

  // else ...
  return sformat("KeyPress code=%d sym=XK_%s",
                 ev->keycode, symname);
}

// ######################################################################
std::string XWinManaged::describeButtonEvent(const XButtonEvent* ev) const
{
  return sformat("ButtonPress button #%u at (%d,%d) type %d",
                 ev->button, ev->x, ev->y, ev->type);
}

// ######################################################################
std::string XWinManaged::describeEvent(const XEvent* ev) const
{
  switch (ev->type)
    {
    case ClientMessage:
      if (ev->xclient.data.l[0] == (int)deleteAtom)
        return "Close button";
      else
        return "ClientMessage";
      break;

    case Expose:
      return "Expose";
      break;

    case KeyPress:
      return describeKeyEvent(&ev->xkey);
      break;

    case ButtonPress:
      return describeButtonEvent(&ev->xbutton);
      break;
    }

  // default:
  return "other XEvent";
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
