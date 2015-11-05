/*!@file GUI/XWindow.H Class definition for a simple window */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/XWindow.H $
// $Id: XWindow.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef XWINDOW_H_DEFINED
#define XWINDOW_H_DEFINED

#include "Util/Types.H"
#include "Image/Dims.H"
#include "Image/Point2D.H"
#include "rutz/shared_ptr.h"

#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>

class Dims;
template <class T> class Image;
template <class T> class Layout;
template <class T> class PixRGB;

//! A simple X-window to display images.
/*! Use this class to create an window on your screen and display an
    image. It will use shared memory between us and the X server if
    supported by the X server and the display is local. Resizing is
    not allowed; so any image passed to drawImage() must have
    dimensions that match those specified at onstruction of the
    XWindow. */
class XWindow {
public:
  //! Constructor: create, display and map the window.
  /*!@param dims Dimensions of usable image area.
     @param x Horizontal position hint for window placement; use -1
      for automatic placement.
     @param y Vertical position hint for window placement; use -1
      for automatic placement.
     @param title String that will appear in the window title. */
  XWindow(const Dims& dims = Dims(320, 240),
          const int x = -1, const int y = -1,
          const char* title = "USC Neuromorphic Vision");

  //! Destructor
  virtual ~XWindow();

  //! Change the window's visibility (map or unmap it as needed)
  void setVisible(const bool v);

  //! Check whether the window is currently mapped (i.e. visible on screen)
  bool isVisible(const bool v) const { return itsMapped; }

  //! Set the position of an XWindow
  /*! @param x Horizontal position hint for window placement
    @param y Vertical position hint for window placement
  */
  void setPosition(const int x, const int y);

  //! Get the position of an XWindow
  /* Note: This may give you a bogus value if you used automatic window
     placement */
  Point2D<int> getPosition();

  //! Draw an image into the XWindow
  /*! @param img the image to be drawn in the window

      @param x if the x coordinate where the image should start in the
      window.

      @param y if the y coordinate where the image should start in the
      window.

      @param resizeWindow if true, then the window will be resized (if
      necessary) to precisely fit the dimensions of the image; if
      false (the default), then the dimension of the image must be
      equal to or less than the current dimensions of the window
  */
  void drawImage(const Image< PixRGB<byte> >& img,
                 const int x = 0, const int y = 0,
                 const bool resizeWindow = false);

  //! Draw an image into the XWindow
  /*! @param img the image to be drawn in the window

      @param x if the x coordinate where the image should start in the
      window.

      @param y if the y coordinate where the image should start in the
      window.

      @param resizeWindow if true, then the window will be resized (if
      necessary) to precisely fit the dimensions of the image; if
      false (the default), then the dimension of the image must be
      equal to or less than the current dimensions of the window
  */
  void drawImage(const Image<byte>& img,
                 const int x = 0, const int y = 0,
                 const bool resizeWindow = false);

  //! Draw an image into the XWindow
  /*! @param img the image to be drawn in the window

      @param x if the x coordinate where the image should start in the
      window.

      @param y if the y coordinate where the image should start in the
      window.

      @param normalize if true, inplaceNormalize() the image to
      [0,255]; if false, inplaceClamp() the image to [0,255]

      @param resizeWindow if true, then the window will be resized (if
      necessary) to precisely fit the dimensions of the image; if
      false (the default), then the dimension of the image must be
      equal to or less than the current dimensions of the window
  */
  void drawImage(const Image<float>& img,
                 const int x = 0, const int y = 0,
                 bool normalize = true,
                 const bool resizeWindow = false);

  //! Draw an image layout into the XWindow
  /*! This is equivalent to drawImage(layout.render()), but is
      implemented more efficiently by avoiding an intermediate
      copy rendered of the image.

      @param resizeWindow if true, then the window will be resized (if
      necessary) to precisely fit the dimensions of the image; if
      false (the default), then the dimension of the image must be
      equal to or less than the current dimensions of the window
  */
  void drawRgbLayout(const Layout<PixRGB<byte> >& layout,
                     const int x = 0, const int y = 0,
                     const bool resizeWindow = false);

  //! Draw an image layout into the XWindow
  /*! This is equivalent to drawImage(layout.render()), but is
      implemented more efficiently by avoiding an intermediate
      copy rendered of the image.

      @param resizeWindow if true, then the window will be resized (if
      necessary) to precisely fit the dimensions of the image; if
      false (the default), then the dimension of the image must be
      equal to or less than the current dimensions of the window
  */
  void drawGrayLayout(const Layout<byte>& layout,
                      const int x = 0, const int y = 0,
                      const bool resizeWindow = false);

  //! Get the window title
  const char* getTitle() const;

  //! set the title bar to a new string
  void setTitle(const char* title);

  //! return the dimensions of this window
  Dims getDims() const;

  //! change the dimensions of this window
  void setDims(const Dims& dims);

  /*! @name Wrappers around X11 Xlib calls

      We wrap these calls, rather than exposing a Display* and Window
      directly, so that we can get the threading issues right; namely,
      we need to lock a global mutex each time any window wants to use
      the Display*, since multiple threads may be running and
      accessing the Display*.
   */
  //@{

  //! Calls XSelectInput()
  void selectInput(long event_mask) const;

  //! Calls XInternAtom() followed by XSetWMProtocols()
  Atom setWMProtocol(const char* atomname) const;

  //! Calls XCheckWindowEvent()
  /*! Note that XCheckWindowEvent() is just like XCheckMaskEvent(),
      but restricted to a specific Window) */
  Bool checkMaskEvent(long event_mask, XEvent* event) const;

  //! Calls XCheckTypedWindowEvent()
  Bool checkTypedEvent(int event_type, XEvent* event) const;

  //@}

protected:
  void redrawImage();

private:
  XWindow(const XWindow&); // not implemented
  XWindow& operator=(const XWindow&); // not implemented

  //! MUST BE CALLED with the display mutex already LOCKED!
  void setDimsImpl(const Dims& dims);

  template <class T>
  void drawLayout(const Layout<T>& layout,
                  const int x, const int y,
                  const bool resizeWindow);

  Dims            itsWinDims;   //<! window size
  Window          itsWindow;    //<! our window
  Visual*         itsVisual;    //<! pointer to current X visual
  int             itsDepth;     //<! bit depth of the window
  GC              itsGc;        //<! current graphic context
  std::string     itsTitle;     //!< the window title
  int             itsAttemptShm;//!< whether to attempt to use shared memory buffers
  bool            itsMapped;    //!< whether the window is currently mapped

  struct XWinImage;

  rutz::shared_ptr<XWinImage> itsXimage;
  Point2D<int>         itsImgPos;    //!< position of the image in the window
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
