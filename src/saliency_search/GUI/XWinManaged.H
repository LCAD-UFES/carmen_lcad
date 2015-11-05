/*!@file GUI/XWinManaged.H A window class with active window management */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/XWinManaged.H $
// $Id: XWinManaged.H 15393 2012-08-15 04:51:24Z kai $
//

#ifndef XWINMANAGED_H_DEFINED
#define XWINMANAGED_H_DEFINED

#include "GUI/XWindow.H"
#include "rutz/atomic.h" // for rutz::atomic_int_t

#include <X11/Xlib.h>
#include <algorithm>
#include <deque>
#include <pthread.h>
#include <unistd.h>
#include <string>
#include <vector>

#define WRITEIMG(img) { \
                inplaceNormalize(img, 0.0f, 255.0f); \
                Raster::WriteFloat(img, 0, "/tmp/img.ppm"); }


template <class T> class PixRGB;
template <class T> class Image;

//! A window class with active window management
/*! This class is an extension of the XWindow class to allow event
    handling.  In the current implementation, <I>exposure</I> and
    <I>resize</I> events are handled. Instead of destroying the
    window, clicking the <I>close</I> button will set a flag that can
    be accessed by calling pressedCloseButton().
*/
class XWinManaged : public XWindow
{
public:

  //! Constructor: create, display and map the window, start the event handling.
  /*! The call of to the constructor is identical to the parent class
      XWindow. In addition to creating the window, the event handling
      is started.
      @param w Width of usable image area.
      @param h height of usable image area.
      @param x Horizontal position hint for window placement; use -1
      for automatic placement.
      @param y Vertical position hint for window placement; use -1
      for automatic placement.
      @param title String that will appear in the window title. */
  XWinManaged(const Dims& dims,
              const int x = -1, const int y = -1,
              const char* title = "USC Neuromorphic Vision");


  //! Constructor: create a window as large as img and display img
  XWinManaged(const Image< PixRGB<byte> >& img,
              const char* title = "USC Neuromorphic Vision");


  //! Constructor: create a window as large as img and display img
  XWinManaged(const Image<byte>& img,
              const char* title = "USC Neuromorphic Vision");

  //! Constructor: create a window as large as img and display img
  XWinManaged(const Image<float>& img,
              const char* title = "USC Neuromorphic Vision",
              bool normalize = true);

  //! Destructor: destroy the window and stop the event handling
  virtual ~XWinManaged();

  //! returns whether the user has clicked the <I>close</I> button.
  bool pressedCloseButton()
  { return toBeClosed; }

  // returns whether any mouse button is pressed
  bool is_mousePressed()
  { return mousePressed; }

  //! request that event tracing be turned on or off
  void setTraceEvents(bool on);


  //! returns the key press at the front of the keyboard buffer
  /*! returns -1 if no key press has occured since the last call */
  int getLastKeyPress();

  //! get the decoded string of the last key press
  /*! returns an empty string if no key press has occured since the
      last call */
  std::string getLastKeyString();

  //! get the decoded keysym of the last key press
  /*! Returns NoSymbol if no key press has occured since the last
      call. Also return the corresponding string through s if s is
      non-null. */
  KeySym getLastKeySym(std::string* s = 0);

  //! copy the last key event into ev, and pop that event from the queue
  /*! @return false if there is no queued key event */
  bool getLastKeyEvent(XKeyEvent* ev);

  //! returns the last mouse click at the front of the click buffer
  /*! does not distinguish between right and left buttons; returns
      (-1,-1) if no mouse clicks have occured since the last call */
  Point2D<int> getLastMouseClick();

  //! returns last mouse motion location
  Point2D<int> getLastMouseMotion();

  //! copy the last button press event into ev, and pop that event from the queue
  /*! @return false if there is no queued button event */
  bool getLastButtonEvent(XButtonEvent* ev);
  bool getLastButtonPressedEvent(XButtonEvent* ev);
  bool getLastButtonReleasedEvent(XButtonEvent* ev);
  bool getLastButtonMotionEvent(XButtonEvent* ev);

  std::string describeKeyEvent(const XKeyEvent* ev) const;

  std::string describeButtonEvent(const XButtonEvent* ev) const;

  std::string describeEvent(const XEvent* ev) const;


private:

  //! Common initialization code used by the constructors
  void init();

  //! thread function that handles X events
  static void* cEventLoop(void* self);

  //! deal with the events coming in
  /*! NOTE: before you consider turning this into a virtual function
      and overriding it, be sure to consider the multi-threading
      issues involved; currently, eventHandler() must be called with
      itsDisplayMutex locked, so if you override the function, you'll
      need to be sure */
  void eventHandler(XEvent* event);

  Atom deleteAtom;           // XServer atom for window closing action
  pthread_t eventLoopThread; // thread for the event handler loop
  rutz::atomic_int_t windowPresent;// flag whether event handling whould keep running
  bool toBeClosed;           // flag whether the user hashit the close button
  bool mousePressed;         // return true when any mouse button is pressed
  pthread_mutex_t itsEventsMutex;
  std::deque<XKeyEvent> itsKeyEvents;
  std::deque<XButtonEvent> itsButtonPressedEvents;
  std::deque<XButtonEvent> itsButtonReleasedEvents;
  std::deque<XButtonEvent> itsButtonMotionEvents;
  std::deque<XButtonEvent> itsButtonEvents;
  rutz::atomic_int_t itsTraceEvents; // whether to print info about each event as it is received
};


// ######################################################################
// very useful listener for any close button action

//! watch for close button actions with all registered XWinManaged windows
class CloseButtonListener
{
public:

  //! add a window to the list by pointer
  inline void add(XWinManaged* win);

  //! add a window to the list by reference
  inline void add(XWinManaged& win);

  //! remove a window from the list by pointer
  inline void remove(XWinManaged* win);

  //! remove a window from the list by reference
  inline void remove(XWinManaged& win);

  //! has a close button been pressed at any of the windows?
  /*! This function will seg fault if the list contains pointers to
    windows that don't exist anymore. Make sure that you remove windows
    from the CloseButtonListener before you explicitely delete them!*/
  inline bool pressedAnyCloseButton();

  //! wait for a close button to be pressed
  /*! run in an idle loop until a close button is pressed */
  inline void waitForClose();

private:
  std::vector<XWinManaged*> windows;
};

// ######################################################################
// inlined methods

// ######################################################################
inline void CloseButtonListener::add(XWinManaged* win)
{ windows.push_back(win); }

inline void CloseButtonListener::add(XWinManaged& win)
{ add(&win); }

// ######################################################################
inline void CloseButtonListener::remove(XWinManaged* win)
{
  std::vector<XWinManaged*>::iterator it = find(windows.begin(),windows.end(),win);
  if (it != windows.end())
    windows.erase(it);
}

inline void CloseButtonListener::remove(XWinManaged& win)
{ remove(&win); }

// ######################################################################
inline bool CloseButtonListener::pressedAnyCloseButton()
{
  for (uint i = 0; i < windows.size(); i++)
    if (windows[i]->pressedCloseButton()) return true;
  return false;
}

// ######################################################################
inline void CloseButtonListener::waitForClose()
{
  while (!(pressedAnyCloseButton())) usleep(30000);
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
