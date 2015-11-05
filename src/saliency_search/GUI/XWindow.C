/*!@file GUI/XWindow.C Implementation for a simple window class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/XWindow.C $
// $Id: XWindow.C 15444 2012-12-01 04:06:55Z kai $
//

#include "GUI/XWindow.H"

#include "Image/Image.H"
#include "Image/Layout.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/atomic.h"
#include "rutz/mutex.h"
#include "rutz/pipe.h"

#include <X11/Xutil.h>
#include <cerrno>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sys/shm.h>
#include <sys/stat.h>

/* On Mac OSX, the multi-threading approach of using a pthreads mutex
   to guard the global Display connection seems to not work
   consistently and we get intermittent errors like this:

   Xlib: unexpected async reply (sequence 0x23d)!
   Xlib: sequence lost (0x10000 > 0x23d) in reply type 0x0!

   The official X11 way to do threads is to initialize with
   XInitThreads(), then lock/unlock the display with XLockDisplay()
   and XUnlockDisplay(). The man page says that those aren't required
   if an external mutex is used, but in our case that doesn't seem to
   work. Switching to XInitThreads()/XLockDisplay()/XUnlockDisplay()
   does seem to eliminate the Xlib errors in practice. The errors also
   seem to go away if we just do XInitThreads() but then do the
   locking with a pthreads mutex as before. Maybe the problem is that
   we do use some unlocked Xlib calls (though they are calls that
   don't require a Display*).

   For now, we can keep the code for both locking styles around and
   toggle between them at compile time with the
   XWINDOW_PTHREADS_LOCKING macro.
*/

// #define XWINDOW_PTHREADS_LOCKING

namespace
{
  int fourByteAlign(int v)
  {
    const int remainder = v % 4;
    return (remainder == 0) ? v : v + (4-remainder);
  }

  int getByteOrder()
  {
    union
    {
      int i;
      byte b[sizeof(int)];
    } u;
    memset(&u, 0, sizeof(u));
    u.b[0] = 1;
    if (u.i == 1)
      return LSBFirst;

    // else ...
    return MSBFirst;
  }

  Display* g_display = 0;

#ifdef XWINDOW_PTHREADS_LOCKING
  pthread_mutex_t g_display_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

  pthread_once_t display_init_once = PTHREAD_ONCE_INIT;

  void display_init()
  {
    XInitThreads();
#ifdef XWINDOW_PTHREADS_LOCKING
    pthread_mutex_init(&g_display_mutex, 0);
#endif
    const char* displayname = getenv("DISPLAY");
    if (displayname == 0 || displayname[0] == '\0')
      {
        LINFO("the DISPLAY environment variable is not set; "
              "assuming DISPLAY=\":0.0\"");
        displayname = ":0.0";
      }
    g_display = XOpenDisplay(displayname);
  }

#ifdef XWINDOW_PTHREADS_LOCKING
#  define LOCK_DISPLAY GVX_MUTEX_LOCK(&g_display_mutex)
#else

  struct DisplayLock
  {
    DisplayLock(Display* d) : dd(d) { XLockDisplay(d); }

    ~DisplayLock() { XUnlockDisplay(dd); }

  private:
    DisplayLock(const DisplayLock&);
    DisplayLock& operator=(const DisplayLock&);

    Display* const dd;
  };

#  define LOCK_DISPLAY DisplayLock tmplock(g_display)

#endif // !defined(XWINDOW_PTHREADS_LOCKING)
}

// ######################################################################
struct XWindow::XWinImage
{
  XWinImage(Display* dpy, bool attemptShm, const Dims& windims,
            const int depth);

  ~XWinImage();

  void destroy(Display* dpy);

  void destroyImage();

  void createImage(Display* dpy, Visual* vis, const Dims& dims);

  Dims getDims() const { return itsImgDims; }

  byte* getBuf() const { return reinterpret_cast<byte*>(itsBuf); }

  void copyPixelsFrom(const Image<byte>& img, const Point2D<int>& winpos);

  void copyPixelsFrom(const Image<PixRGB<byte> >& img, const Point2D<int>& winpos);

  void redraw(Display* dpy, Window win, GC gc, const Point2D<int>& pos);

private:
  const int       itsBytesPerPixel;

  XImage*         itsImage;     //<! pointer to image data

  XShmSegmentInfo itsShmInfo;   //<! info on shared memory segment
  char*           itsBuf;       //<! shared memory buffer
  bool            itsUsingShm;  //<! true if we can use shared memory
  Dims            itsImgDims;   //<! XImage size
  bool            itsDestroyed;
};

static size_t get_shmmax()
{
  {
    std::ifstream f("/proc/sys/kernel/shmmax");
    if (f.is_open())
      {
        size_t shmmax = 0;
        f >> shmmax;
        return shmmax;
      }
  }

  const char* progs[] = { "/usr/sbin/sysctl", "/sbin/sysctl" };

  const char* args[] = { "kern.sysv.shmmax", "kernel.shmmax" };

  for (size_t i = 0; i < sizeof(progs) / sizeof(progs[0]); ++i)
    {
      try
        {
          rutz::exec_pipe p("r", progs[i], args[i], (const char*) 0);
          std::string w;
          while (p.stream() >> w)
            {
              if (w.size() > 0 && isdigit(w[0]))
                return fromStr<size_t>(w);
            }
        }
      catch (...) { continue; }
    }

  return 0;
}

// ######################################################################
XWindow::XWinImage::XWinImage(Display* dpy, bool attemptShm,
                              const Dims& windims,
                              const int depth)
  :
  itsBytesPerPixel(depth == 16 ? 2 : 4),
  itsImage(0),
  itsShmInfo(),
  itsBuf(0),
  itsUsingShm(false),
  itsImgDims(),
  itsDestroyed(false)
{
  // Get a padded row width that is 4-byte aligned
  const int padded_width = fourByteAlign(windims.w());

  // allocate an image buffer of the size of the window:
  const size_t bufsize = padded_width*windims.h()*itsBytesPerPixel;
  LDEBUG("bufsize=%" ZU , bufsize);

  if (attemptShm)
    {
      itsShmInfo.shmid = shmget(IPC_PRIVATE, bufsize, IPC_CREAT | 0777);
      LDEBUG("shmid=%d", itsShmInfo.shmid);
      if (itsShmInfo.shmid == -1)
        {
          const int errno_save = errno;

          const size_t shmmax = get_shmmax();
          const std::string shmmax_info =
            shmmax > 0
            ? sformat("%" ZU , shmmax)
            : "unknown";

          errno = errno_save;

          const std::string extra_help =
            (errno == ENOMEM
             || (shmmax != 0 && bufsize > shmmax))
            ? ("; try increasing the shmmax value in "
               "/proc/sys/kernel/shmmax, or with "
               "sysctl kern.sysv.shmmax (BSD) "
               "or with sysctl kernel.shmmax (Linux)")
            : "";

          PLERROR("shmget() failed (requested size=%" ZU ", shmmax=%s)%s",
                  bufsize, shmmax_info.c_str(), extra_help.c_str());

          LINFO("switching to slower non-shared memory approach");
        }
      else
        {
          itsBuf = (char*)shmat(itsShmInfo.shmid, NULL, 0);  // attach shared memory
          if (itsBuf == 0) LFATAL("Cannot get shared memory");
          itsShmInfo.shmaddr = itsBuf;        // link buffer to shminfo structure
          itsShmInfo.readOnly = False;
          XShmAttach(dpy, &itsShmInfo);   // now attach to X
          itsUsingShm = true;
        }
    }

  if (!itsUsingShm)
    {
      itsBuf = (char*)malloc(bufsize);
      LDEBUG("buffer: %p", itsBuf);
      if (itsBuf == NULL) LFATAL("Cannot allocate image buffer");
    }
}

// ######################################################################
XWindow::XWinImage::~XWinImage()
{
  // we can't cleanly run the destruction routines that we need here
  // in the destructor, because destroy() requires a Display* with
  // g_display_mutex already locked, and we have no way to pass the
  // Display* into the destructor; therefore, we instead require that
  // XWinImage::destroy() be called explicitly before the destructor
  // is run, and we verify that this has actually been done with an
  // assertion:

  ASSERT(itsDestroyed == true);
}

// ######################################################################
void XWindow::XWinImage::destroy(Display* dpy)
{
  if (itsUsingShm)
    {
      XShmDetach(dpy, &itsShmInfo);
      this->destroyImage();
      shmdt(itsShmInfo.shmaddr);
      shmctl(itsShmInfo.shmid, IPC_RMID, NULL);
    }
  else
    {
      this->destroyImage();
      free(itsBuf);
    }

  itsDestroyed = true;
}

// ######################################################################
void XWindow::XWinImage::destroyImage()
{
  if (itsImage)
    {
      // We have to set the XImage's data to zero before destroying
      // the XImage, because otherwise the XImage will want to free
      // the data as it is destroyed, which means that 'itsBuf' would
      // become a dangling pointer, and that we'd be double-free'ing
      // it when we try to free() it ourselves later.
      itsImage->data = 0;
      XDestroyImage(itsImage);
      itsImage = 0;
    }
}

// ######################################################################
void XWindow::XWinImage::createImage(Display* dpy, Visual* vis,
                                     const Dims& dims)
{
  this->destroyImage();

  uint depth = 0;
  int pad = 0;

  // make sure we use the correct depth values
  if (itsBytesPerPixel == 2)
    { depth = 16; pad = 16;}
  else if (itsBytesPerPixel == 4)
    { depth = 24; pad = 32;}
  else
    LFATAL("bogus bytes-per-pixel value '%d'", itsBytesPerPixel);

  if (itsUsingShm)
    // ok for shared memory; X will allocate the buffer:
    itsImage = XShmCreateImage(g_display, vis, depth, ZPixmap,
                               itsBuf, &itsShmInfo, dims.w(), dims.h());
  else
    // cannot alloc in shared memory... do conventional alloc
    itsImage = XCreateImage(g_display, vis, depth, ZPixmap, 0,
                            itsBuf, dims.w(), dims.h(), pad,
                            fourByteAlign(itsBytesPerPixel * dims.w()));

  // by default, XCreateImage() chooses the byte order according to
  // the machine running X server; however since we are building the
  // image here on the X client, we want to switch the byte order to
  // match the byte order on the client:

  LDEBUG("X server byte order is %s (%d)",
        itsImage->byte_order == LSBFirst ? "LSBFirst" :
        itsImage->byte_order == MSBFirst ? "MSBFirst" : "unknown",
        itsImage->byte_order);

  itsImage->byte_order = getByteOrder();

  LDEBUG("using image byte order %s (%d)",
        itsImage->byte_order == LSBFirst ? "LSBFirst" :
        itsImage->byte_order == MSBFirst ? "MSBFirst" : "unknown",
        itsImage->byte_order);

  itsImgDims = dims;
}

// ######################################################################
void XWindow::XWinImage::copyPixelsFrom(const Image<byte>& img,
                                        const Point2D<int>& winpos)
{
  ASSERT(winpos.i >= 0);
  ASSERT(winpos.i + img.getWidth() <= itsImgDims.w());
  ASSERT(winpos.j >= 0);
  ASSERT(winpos.j + img.getHeight() <= itsImgDims.h());

  const int w = img.getWidth();
  const int h = img.getHeight();

  const byte* im = img.getArrayPtr();
  byte* bu = reinterpret_cast<byte*>(itsBuf)
    + itsBytesPerPixel * (winpos.i + winpos.j * itsImgDims.w());

  if (itsBytesPerPixel == 2)
    {
      const int bytes_per_row = fourByteAlign(itsBytesPerPixel * itsImgDims.w());

      // 16 bit format: 565, lowest byte first
      for (int j = 0; j < h; ++j)
        {
          byte* bu_row = bu + j*bytes_per_row;
          for (int i = 0; i < w; ++i)
            {
              *bu_row++ = ((*im & 0x1C)<<3) | ((*im & 0xF8)>>3);
              *bu_row++ = ((*im & 0xE0)>>5) | (*im & 0xF8);
              im++;
            }
        }
    }
  else if (itsBytesPerPixel == 4)
    {
      const int wskip = itsBytesPerPixel * (itsImgDims.w() - img.getWidth());

      // 24 bit format with extra byte for padding
      switch (getByteOrder())
        {
        case LSBFirst:
          for (int j = 0; j < h; ++j)
            {
              for (int i = 0; i < w; ++i)
                { *bu++ = *im; *bu++ = *im; *bu++ = *im++; *bu++ = 255; }
              bu += wskip;
            }
          break;

        case MSBFirst:
          for (int j = 0; j < h; ++j)
            {
              for (int i = 0; i < w; ++i)
                { *bu++ = 255; *bu++ = *im; *bu++ = *im; *bu++ = *im++; }
              bu += wskip;
            }
          break;

        default:
          LFATAL("invalid byte order %d", getByteOrder());
        }
    }
  else
    LFATAL("bogus bytes-per-pixel value '%d'", itsBytesPerPixel);
}

// ######################################################################
void XWindow::XWinImage::copyPixelsFrom(const Image<PixRGB<byte> >& img,
                                        const Point2D<int>& winpos)
{
  ASSERT(winpos.i >= 0);
  ASSERT(winpos.i + img.getWidth() <= itsImgDims.w());
  ASSERT(winpos.j >= 0);
  ASSERT(winpos.j + img.getHeight() <= itsImgDims.h());

  const int w = img.getWidth();
  const int h = img.getHeight();

  const byte* im = reinterpret_cast<const byte*>(img.getArrayPtr());
  byte* bu = reinterpret_cast<byte*>(itsBuf)
    + itsBytesPerPixel * (winpos.i + winpos.j * itsImgDims.w());

  if (itsBytesPerPixel == 2)
    {
      const int bytes_per_row = fourByteAlign(itsBytesPerPixel * itsImgDims.w());

      // 16 bit format: 565, lowest byte first
      for (int j = 0; j < h; ++j)
        {
          byte* bu_row = bu + j*bytes_per_row;
          for (int i = 0; i < w; ++i)
            {
              *bu_row++ = ((im[1] & 0x1C)<<3) | ((im[2] & 0xF8)>>3);
              *bu_row++ = ((im[1] & 0xE0)>>5) | (im[0] & 0xF8);
              im += 3;
            }
        }
    }
  else if (itsBytesPerPixel == 4)
    {
      const int wskip = itsBytesPerPixel * (itsImgDims.w() - img.getWidth());

      // 24 bit format with extra byte for padding
      switch (getByteOrder())
        {
        case LSBFirst:
          for (int j = 0; j < h; ++j)
            {
              for (int i = 0; i < w; ++i)
                { *bu++ = im[2]; *bu++ = im[1]; *bu++ = *im; *bu++ = 255; im += 3; }
              bu += wskip;
            }
          break;

        case MSBFirst:
          for (int j = 0; j < h; ++j)
            {
              for (int i = 0; i < w; ++i)
                { *bu++ = 255; *bu++ = *im; *bu++ = im[1]; *bu++ = im[2]; im += 3; }
              bu += wskip;
            }
          break;

        default:
          LFATAL("invalid byte order %d", getByteOrder());
        }
    }
  else
    LFATAL("bogus bytes-per-pixel value '%d'", itsBytesPerPixel);
}

// ######################################################################
void XWindow::XWinImage::redraw(Display* dpy, Window win, GC gc,
                                const Point2D<int>& pos)
{
  if (itsImage)
    {
      if (itsUsingShm)
        XShmPutImage(dpy, win, gc, itsImage, 0, 0,
                     pos.i, pos.j,
                     itsImgDims.w(), itsImgDims.h(), 0);
      else
        XPutImage(dpy, win, gc, itsImage, 0, 0,
                  pos.i, pos.j,
                  itsImgDims.w(), itsImgDims.h());
    }
  else
    {
      XClearWindow(dpy,win);
    }
}

// ######################################################################
XWindow::XWindow(const Dims& dims, const int x, const int y,
                 const char* title)
  :
  itsMapped(false)
{
  if (dims.isEmpty())
    LFATAL("window dimensions must be non-empty");

  itsWinDims = dims;

  // connect to X server:
  pthread_once(&display_init_once, &display_init);
  if (g_display == 0)
    LFATAL("Cannot connect to X server");

  LOCK_DISPLAY;

  // get default screen:
  const int screen = DefaultScreen(g_display);
  itsVisual = DefaultVisual(g_display, screen);
  itsDepth = DefaultDepth(g_display, screen);
  LDEBUG("Your screen depth is %d bpp", itsDepth);

  // check if we are using automatic window placement:
  int x0 = x;
  int y0 = y;
  if (x0 == -1 && y0 == -1)
    {
      x0 = 0;
      y0 = 0;
    }

  // open window:
  itsWindow = XCreateSimpleWindow(g_display,
                                  DefaultRootWindow(g_display),
                                  x0, y0, itsWinDims.w(), itsWinDims.h(), 2,
                                  BlackPixel(g_display, screen),
                                  WhitePixel(g_display, screen));
  if (itsWindow <= 0) LFATAL("Cannot open window");

  // set title:
  XStoreName(g_display, itsWindow, (char*)title);
  itsTitle = title;

  // pop this window up on the screen:
  XMapRaised(g_display, itsWindow);
  itsMapped = true;

  // set position of window if not using auto-placement:
  if (x != -1 || y != -1)
    {
      XMoveWindow(g_display, itsWindow, x, y);
    }

  // set events that we are interested in:
  XSelectInput(g_display, itsWindow, NoEventMask); // ExposureMask|ButtonPressMask

  // flush X request queue to server:
  XFlush(g_display);

  // create graphics context for later:
  itsGc = XCreateGC(g_display, itsWindow, 0, NULL);

  XSetWindowBackground(g_display, itsWindow,
                       WhitePixel(g_display, DefaultScreen(g_display)));

  // alloc XImage in shared memory if posible (for faster display):
  itsAttemptShm = XShmQueryExtension(g_display);

  // check if remote display, and disable shared memory if so:
  char* disp = getenv("DISPLAY");
  if (disp != NULL && disp[0] != ':')
    { itsAttemptShm = 0; LDEBUG("Not using shared memory for remote display"); }

  this->setDimsImpl(dims);
}

// ######################################################################
void XWindow::setVisible(const bool v)
{
  if (itsMapped != v)
    {
      Point2D<int> p1 = this->getPosition();
      LDEBUG("old position %d,%d", p1.i, p1.j);

      {
      LOCK_DISPLAY;

      if (v)
        {
          LDEBUG("mapping window %s", itsTitle.c_str());
          XMapWindow(g_display, itsWindow);
        }
      else
        {
          LDEBUG("unmapping window %s", itsTitle.c_str());
          XUnmapWindow(g_display, itsWindow);
        }

      itsMapped = v;
      }

      Point2D<int> p2 = this->getPosition();
      LDEBUG("new position %d,%d", p2.i, p2.j);
    }
}

// ######################################################################
const char* XWindow::getTitle() const
{
  return itsTitle.c_str();
}

// ######################################################################
void XWindow::setTitle(const char* title)
{
  LOCK_DISPLAY;

  XStoreName(g_display, itsWindow, (char*)title);
  itsTitle = title;
}

// ######################################################################
void XWindow::setPosition(const int x, const int y)
{
  LOCK_DISPLAY;

  XMoveWindow(g_display, itsWindow, x, y);
}

// ######################################################################
Point2D<int> XWindow::getPosition()
{
  LOCK_DISPLAY;

  XWindowAttributes xwa;
  XGetWindowAttributes(g_display, itsWindow, &xwa);
  return Point2D<int>(xwa.x, xwa.y);
}

// ######################################################################
void XWindow::drawImage(const Image< PixRGB<byte> >& img,
                        const int x, const int y, const bool resizeWindow)
{
  if (resizeWindow)
    this->setDims(img.getDims());

  ASSERT(x >= 0 && y >= 0 && x < itsWinDims.w() && y < itsWinDims.h());
  const int w = img.getWidth();
  const int h = img.getHeight();
  const Dims d = img.getDims();
  LDEBUG("x %d w %d itsWinDims.W() %d, y %d h %d, itsWinDims.h() %d",
    x,w,itsWinDims.w(),y,h,itsWinDims.h());
  ASSERT(x + w <= itsWinDims.w() && y + h <= itsWinDims.h());
  if (d != itsXimage->getDims())
    {
      LOCK_DISPLAY;
      itsXimage->createImage(g_display, itsVisual, d);
    }

  itsXimage->copyPixelsFrom(img, Point2D<int>(0,0));

  itsImgPos = Point2D<int>(x, y);

  this->redrawImage();
}

// ######################################################################
void XWindow::drawImage(const Image<byte>& img, const int x, const int y,
                        const bool resizeWindow)
{
  if (resizeWindow)
    this->setDims(img.getDims());

  ASSERT(x >= 0 && y >= 0 && x < itsWinDims.w() && y < itsWinDims.h());
  const int w = img.getWidth();
  const int h = img.getHeight();
  const Dims d = img.getDims();
  ASSERT(x + w <= itsWinDims.w() && y + h <= itsWinDims.h());
  if (d != itsXimage->getDims())
    {
      LOCK_DISPLAY;
      itsXimage->createImage(g_display, itsVisual, d);
    }

  itsXimage->copyPixelsFrom(img, Point2D<int>(0,0));

  itsImgPos = Point2D<int>(x, y);

  this->redrawImage();
}

// ######################################################################
void XWindow::drawImage(const Image<float>& img,
                        const int x, const int y,
                        bool normalize, const bool resizeWindow)
{
  if (resizeWindow)
    this->setDims(img.getDims());

  Image<float> image(img);

  if (normalize)
    inplaceNormalize(image, 0.0f, 255.0f);
  else
    inplaceClamp(image, 0.0f, 255.0f);

  this->drawImage(Image<byte>(image), x, y);
}

// ######################################################################
void XWindow::drawRgbLayout(const Layout<PixRGB<byte> >& layout,
                            const int x, const int y,
                            const bool resizeWindow)
{
  this->drawLayout<PixRGB<byte> >(layout, x, y, resizeWindow);
}

// ######################################################################
void XWindow::drawGrayLayout(const Layout<byte>& layout,
                             const int x, const int y,
                             const bool resizeWindow)
{
  this->drawLayout<byte>(layout, x, y, resizeWindow);
}

// ######################################################################
XWindow::~XWindow()
{
  LOCK_DISPLAY;

  LDEBUG("Closing down...");
  itsXimage->destroy(g_display);
  itsXimage.reset(0);

  if (itsGc) XFreeGC(g_display, itsGc);
  if (g_display && itsWindow)
    {
      LDEBUG("XDestroyWindow...");
      XDestroyWindow(g_display, itsWindow);
      XFlush(g_display);
    }

  // NB: We don't call XCloseDisplay() here because other windows
  // (current and future) might still want to use the existing
  // Display*. So we just rely on the display being cleaned up
  // automatically when the program exits.
}

// ######################################################################
void XWindow::redrawImage()
{
  LOCK_DISPLAY;

  itsXimage->redraw(g_display, itsWindow, itsGc, itsImgPos);
  XFlush(g_display);
}

// ######################################################################
Dims XWindow::getDims() const
{
  return itsWinDims;
}

// ######################################################################
void XWindow::setDims(const Dims& dims)
{
  if (dims == itsWinDims)
    return;

  LOCK_DISPLAY;

  this->setDimsImpl(dims);
}

// ######################################################################
void XWindow::selectInput(long event_mask) const
{
  LOCK_DISPLAY;

  XSelectInput(g_display, itsWindow, event_mask);
}

// ######################################################################
Atom XWindow::setWMProtocol(const char* atomname) const
{
  LOCK_DISPLAY;

  Atom a = XInternAtom(g_display, atomname, false);
  if (0 == XSetWMProtocols(g_display, itsWindow, &a, 1))
    LFATAL("Error setting the WM protocol");

  return a;
}

// ######################################################################
Bool XWindow::checkMaskEvent(long event_mask, XEvent* event) const
{
  LOCK_DISPLAY;
  return XCheckWindowEvent(g_display, itsWindow, event_mask, event);
}

// ######################################################################
Bool XWindow::checkTypedEvent(int event_type, XEvent* event) const
{
  LOCK_DISPLAY;
  return XCheckTypedWindowEvent(g_display, itsWindow, event_type, event);
}

// ######################################################################
void XWindow::setDimsImpl(const Dims& dims)
{
  // NOTE: this function MUST be called with g_display_mutex already
  // locked!

  // first release the lock that prevent the window from being resized:
  XSizeHints hints;
  hints.flags = PMinSize | PMaxSize;
  hints.min_width = 0; hints.max_width = 60000;
  hints.min_height = 0; hints.max_height = 60000;
  XSetWMNormalHints(g_display, itsWindow, &hints);

  XResizeWindow(g_display, itsWindow, dims.w(), dims.h());
  itsWinDims = dims;

  if (itsXimage.is_valid())
    itsXimage->destroy(g_display);

  itsXimage.reset(new XWinImage(g_display, itsAttemptShm, itsWinDims,
                                itsDepth));

  // prevent the window from being resized by the use (by fixing both
  // its minimum and maximum size to exactly itsWinDims):
  hints.flags = PMinSize | PMaxSize | PBaseSize;
  hints.min_width = hints.max_width = hints.base_width = itsWinDims.w();
  hints.min_height = hints.max_height = hints.base_height = itsWinDims.h();
  XSetWMNormalHints(g_display, itsWindow, &hints);
}

// ######################################################################
template <class T>
void XWindow::drawLayout(const Layout<T>& layout,
                         const int x, const int y,
                         const bool resizeWindow)
{
  if (resizeWindow)
    this->setDims(layout.getDims());

  ASSERT(x >= 0 && y >= 0 && x < itsWinDims.w() && y < itsWinDims.h());
  const int w = layout.getDims().w();
  const int h = layout.getDims().h();
  const Dims d = layout.getDims();
  ASSERT(x + w <= itsWinDims.w() && y + h <= itsWinDims.h());
  if (d != itsXimage->getDims())
    {
      LOCK_DISPLAY;
      itsXimage->createImage(g_display, itsVisual, d);
    }

  typedef std::pair<const Layout<T>*, Point2D<int> > tile;

  std::vector<tile> q;

  q.push_back(std::make_pair(&layout, Point2D<int>(0,0)));

  while (!q.empty())
    {
      tile t = q.back();
      q.pop_back();

      const Layout<T>* const cur = t.first;
      const Point2D<int> p = t.second;

      const size_t np = cur->numParts();

      if (np == 0)
        {
          if (cur->leafImage().initialized())
            itsXimage->copyPixelsFrom(cur->leafImage(), p);
        }
      else if (cur->getDir() == Layout<T>::H)
        {
          Point2D<int> pp = p;
          for (size_t i = 0; i < np; ++i)
            {
              q.push_back(std::make_pair(&cur->part(i), pp));
              pp.i += cur->part(i).getDims().w();
            }
        }
      else
        {
          Point2D<int> pp = p;
          for (size_t i = 0; i < np; ++i)
            {
              q.push_back(std::make_pair(&cur->part(i), pp));
              pp.j += cur->part(i).getDims().h();
            }
        }
    }

  itsImgPos = Point2D<int>(x, y);

  this->redrawImage();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
