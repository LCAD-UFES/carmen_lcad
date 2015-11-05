/*!@file GUI/SDLdisplay.H Fast full-screen displays using SDL */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/SDLdisplay.H $
// $Id: SDLdisplay.H 14170 2010-10-27 07:04:19Z ilink $
//

#ifndef GUI_SDLDISPLAY_H_DEFINED
#define GUI_SDLDISPLAY_H_DEFINED

#ifdef HAVE_SDL_SDL_H

#include "Component/ModelComponent.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/Timer.H"          // for Timer
#include "Video/VideoFormat.H"
#include "Component/EventLog.H"

#include <list>
#include <SDL/SDL.h>                // for SDL_Screen
#include <SDL/SDL_gfxPrimitives.h>  // for drawing shapes in SDL

class VideoFrame;

//! Class fo do various fast graphics displays
/*! This class is to facilitate the display of various graphics
  stimuli, with an emphasis on strict real-time operation and in
  particular on playing movies at a controlled framerate with
  microsecond accuracy. Programs using this class, such as
  psycho-movie.C, should run as root if SCHED_FIFO scheduling is
  required (and I highly recommend it, as it will make timing
  reliable). The class uses the SDL library to do the displays. This
  class attempts to lock onto the display's vertical blanking for
  reliable timing of movie playing. Note that, as far as I know, SDL
  does not provide a way to select a screen refresh rate; instead, you
  need to specify the refresh rate in your X config. I used the
  modeline calculator at
  http://xtiming.sourceforge.net/cgi-bin/xtiming.pl to figure out good
  modelines for the refresh rates I wanted. For example: for a 120Hz
  640x480 display, I got:

  Modeline "640x480@120" 55.43 640 672 880 912 480 487 497 505

  which I added to the "Monitor" section of /etc/X11/XF86Config-4, and
  I also specified that this mode should be used by giving its name in
  the "Screen" section:

  Subsection "Display"
  Depth 24
  Modes "1280x1024" "1280x960" "1152x864" "1024x768" "800x600" "640x480@120"
  EndSubsection

  Always verify on the OSD info page of your monitor that you are
  actually getting the refresh rate you want.

  For 640x480 @ 60Hz doublescan, I use:
  Modeline "640x480@60d" 48.21 640 672 760 792 480 490 495 505 doublescan

  For 800x600 @ 60Hz (for LCD goggles):
  ModeLine "800x600@60" 40.0 800 840 968 1056 600 601 605 628 +hsync +vsync

  For standard VESA 640x480 @ 60Hz:
  Modeline "psycho"  25.2 640 656 752 800 480 490 492 525 -hsync -vsync
*/

class SDLdisplay : public ModelComponent
{
public:

  /// Different types of inter-frame delay that can be requested
  enum DelayType
    {
      NO_WAIT,       ///< Don't wait at all between frames
      NEXT_VSYNC,    ///< Wait for the next vertical blanking period
      NEXT_FRAMETIME ///< Wait for the next frame time according to itsRefreshDelay
    };

  // ######################################################################
  /*! @name Constructors, destructor and initialization */
  //@{

  //! Constructor
  SDLdisplay(OptionManager& mgr,
             const std::string& descrName = "SDL Display",
             const std::string& tagName = "SDLdisplay");

  //! Destructor
  virtual ~SDLdisplay();

  //! Link us to an EventLog component
  /*! If an EventLog component is registered with us, we will send it
    lots of event messages, each time a frame or overlay is displayed,
    a key pressed, etc. */
  void setEventLog(nub::soft_ref<EventLog> elog);

  //@}

  // ######################################################################
  /*! @name General display property functions */
  //@{

  //! Get bytes per pixel
  inline int getBytesPerPixel() const;

  //! Get display dimensions, in pixels
  inline Dims getDims() const;

  //! Get display width, in pixels
  inline int getWidth() const;

  //! Get display height, in pixels
  inline int getHeight() const;

  //! Show or hide mouse pointer
  inline void showCursor(const bool showit);

  //@}

  // ######################################################################
  /*! @name Pixel-level functions */
  //@{

  //! Get the 32 bit color value corresponding to a given RGB pixel
  inline Uint32 getUint32color(const PixRGB<byte>& col) const;

  //! Get the RGB color triplet corresponding to a given Uint32 color
  inline PixRGB<byte> getRGBcolor(const Uint32 col) const;

  //@}

  // ######################################################################
  /*! @name Basic display and interaction functions */
  //@{

        //! open a SDL display
  void openDisplay();

        //! close a SDL display
  void closeDisplay();

  //! Lock the screen, as required before using some functions
  inline void lockScreen();

  //! Unlock the (previously locked) screen
  inline void unlockScreen();

  //! clear the screen
  /*! @param col the background color to use
      @param vsync will attempt to sync with vertical blanking if true */
  void clearScreen(const PixRGB<byte> col, const bool vsync = true);

  //! clear the video back buffer
  void clearBackBuffer();

  //! Return the pixel value at (x, y)
  // NOTE: The surface must be locked before calling this!
  Uint32 getPixel32(const int x, const int y) const;

  //! Set the pixel at (x, y) to the given value
  // NOTE: The surface must be locked before calling this!
  void putPixel32(const int x, const int y, const Uint32 pixel);

  //! show a text message
   /*! @param ind ind=0 displays the text in the middle of screen, ind =1 displays the text on top or the screen
   and ind = -1 displays the message at the buttom of the display
  */
  void displayText(const std::string& msg, const bool vsync,
                   const PixRGB<byte> txtcol, const PixRGB<byte> bgcol, int ind = 0, 
									 const int = 10);

//! show a text message in a given place
 void displayText(const std::string& msg,Point2D<int> p ,
                             const PixRGB<byte> txtcol,
                             const PixRGB<byte> bgcol,const bool vsync) ;

  //! Wait until start of the next vertical blanking, per our frame rate
  /*! The vertical blanking period is when the monitor's electron gun
    travels from the bottom of the screen back to the top. This
    function updates our itsLastSync data member with current time at
    start of vertical blanking pulse. Most display functions have a
    'vsync' argument and will call this function to perform the sync,
    so you usually never have to call this directly. It is provided
    here just in case. If checkdelay is true, we will log a warning if
    it has been more than one itsRefreshDelay period since the last
    time we were called. No logging is made if quiet is true,
    otherwise start and end times are logged.

    NOTE that this does not necessarily wait for just the next
    hardware vsync; instead we wait for the next hardware vsync that
    just precedes our next desired frame according to
    itsRefreshDelay. So even though your monitor may run at 60Hz or
    120Hz, if itsRefreshDelay is 33333us, your actual displayed frame
    rate will only be 30Hz when you use waitNextRequestedVsync(). */
  void waitNextRequestedVsync(const bool checkdelay = false,
                              const bool quiet = true);

  /// Wait for the next frame time, per our frame rate
  void waitNextRequestedFrameTime(const int frame,
                                  const bool checkdelay = false,
                                  const bool quiet = true);

  //! Wait for a number of frame periods
  /*! This just repeatedly calles waitNextRequestedVsync() */
  void waitFrames(const int n);

  //! Set a desired refresh delay in microseconds, and a fractional tolerance around that delay
  void setDesiredRefreshDelayUsec(float usec, float tol=0.05F);
  //@}

  // ######################################################################
  /*! @name Keyboard functions */
  //@{

  //! wait for a keypress and return character for key pressed
  /*! by default stdin is cleared try waitForKey(false) to keep stdin buffer */
  int waitForKey(bool doWait=true);

  //! wait for a keypress and return character for key pressed
  /*! by default stdin is cleared try waitForKey(false) to keep stdin buffer */
  int waitForKeyTimeout(double timeout = 1000, bool doWait=true);

  //! wait for a mouseclick and return 1 for left button click and 2 for right button click
  /*!by default stdin is cleared try waitForMouseClick(false) to keep stdin buffer */
  int waitForMouseClick(bool doWait=true) ;

	long getTimerValue() ;

	int waitForMouseWheelEvent(bool doWait=true) ;
  //! check whether a mouseclick has occurred and return 1 for key left button clicked and 2 for right button clicked
  /*! a value of -1 is returned if no mouse click event occured */
  int checkForMouseClick() ;

  //! check whether a keypress has occurred and return char for key pressed
  /*! a value of -1 is returned if no keypress occured */
  int checkForKey();

  //! store the string of keypresses input by the user until the terminating character has been reached
  /*!  */
  std::string getString(char terminator);


  //@}

  // ######################################################################
  /*! @name Image blitting functions */
  //@{

  //! Display an image, centered over the screen area
  /*! If you want to display an image that is smaller than the display
    area at a well-defined location on the screen, see
    displayImagePatch(). This just calls makeBlittableSurface()
    followed by displaySurface(). It is often beneficial to use these
    other two functions for real-time applications, first preparing
    your surfaces when you have some extra available time, and then
    getting a faster blit. */
  void displayImage(const Image< PixRGB<byte> >& img,
                    const bool resiz = false,
                    const PixRGB<byte> bgcol = PixRGB<byte>(0, 0, 0),
                    const int frame = -1, const bool vsync = true);

  //! Make a blittable surface from an Image
  /*! The resulting surface can be displayed using displaySurface(). It
    should be freed using SDL_FreeSurface()
    @param img the image to be displayed
    @param resiz if true, the image will be resized such as to fill-up
           at least one dimension of the display (but aspect ratio
           will be maintained, and possible borders filled with bgcol)
    @param bgcol background color to use around image if image has
           different aspect ratio from screen */
  SDL_Surface *makeBlittableSurface(const Image< PixRGB<byte> >& img,
                                    const bool resiz = false,
                                    const PixRGB<byte> bgcol =
                                    PixRGB<byte>(0, 0, 0));

  //! Display an SDL surface that has been pre-formatted for fast blitting
  /*! @param img a pre-formatted image; no rescaling will be attempted
             and it will just be blitted into the screen. Suitable surfaces
             may be obtained by makeBlittableSurface()
      @param frame this is just used for the log messages; if it is -1 then
             no frame number/duration are logged; if it is -2 both the
             start and end of the function execution are logged
      @param vsync will attempt to sync with vertical blanking if true */
  void displaySurface(SDL_Surface *img, const int frame = -1,
                    const bool vsync = true);


  //! Display an image patch
  /*! @param image an image patch
      @param pos the position of the top-left corner of the image
      @param frame this is just used for the log messages; if it is -1 then
             no frame number/duration are logged; if it is -2 both the
             start and end of the function execution are logged
      @param vsync will attempt to sync with vertical blanking if true */
  void displayImagePatch(const Image< PixRGB<byte> >& image,
                         const Point2D<int>& pos, const int frame = -1,
                         const bool vsync = true, const bool flip = true);

  //@}
 // ######################################################################
 //! Display an SDLSurface patch on thescreen
  /*! @param surf a pointer to a SDL_Surface
      @param offset a pointer to a SDL_Rect which encapsulates the offset
      @param clip a pointer to a SDL_Rect which encapsulates the clip to be shown from surf
      @param frame this is just used for the log messages; if it is -1 then
             no frame number/duration are logged; if it is -2 both the
             start and end of the function execution are logged
      @param vsync will attempt to sync with vertical blanking if true
      @param flip will attempt to flip if true
 */
  void displaySDLSurfacePatch(SDL_Surface* surf , SDL_Rect* offset , SDL_Rect* clip , const int frame=-2,
                                   const bool vsync = true, const bool flip = true);

  // ######################################################################
  /*! @name YUV overlay functions */
  //@{

  //! Check whether somebody has already created a YUV overlay for this display
  bool hasYUVoverlay() const;

  //! create a YUV overlay of specified size
  /*! All the other overlay functions will throw a fatal error if no
    overlay has been created. */
  void createYUVoverlay(const Uint32 format, const int w, const int h);

  //! create a YUV overlay
  /*! All the other overlay functions will throw a fatal error if no
    overlay has been created. */
  void createYUVoverlay(const Uint32 format);

  //! destroy a previously-created YUV overlay
  /*! Overlay displays and normal displays don't mix well in SDL. So,
    typically, you would create the overlay just before playing a
    movie in YUV mode, then play the movie, then clear the screen and
    destroy the overlay, do a few normal displays (like a fixation
    cross, some text, etc), and loop. */
  void destroyYUVoverlay();

  //! Lock YUV overlay and get a pointer to it for direct access to pixel data
  /*! And what can you do with the pixel data? see psycho-movie.C and
    pvisionTCP3-master.C for examples of how to fill it from either
    raw MPEG movie frames or from an Image<PixRGB<byte> >. */
  SDL_Overlay* lockYUVoverlay();

  //! Unlock YUV overlay
  void unlockYUVoverlay();

  //! display current YUV overlay
  void displayYUVoverlay(const int frame, const DelayType dly, const int x,
                         const int y, const int w, const int h);

  //! display current YUV overlay
  void displayYUVoverlay(const int frame, const DelayType w);

  //! test whether we can do video overlays in the given frame format
  bool supportsVideoOverlay(const VideoFormat vidformat) const;

  //! create a YUV overlay of specific size from a video format
  /*! All the other overlay functions will throw a fatal error if no
    overlay has been created. */
  void createVideoOverlay(const VideoFormat vidformat,
                            const int w, const int h);

  //! create a YUV overlay from a video format
  /*! All the other overlay functions will throw a fatal error if no
    overlay has been created. */
  void createVideoOverlay(const VideoFormat vidformat);

  //! copy video data to the current YUV overlay, and display it
  void displayVideoOverlay(const VideoFrame& buf,
                           const int frame, const DelayType w);

  //! copy video data to the current YUV overlay, and display it
  //  at location x, y of the screen and new size rw, rh
  void displayVideoOverlay_pos(const VideoFrame& buf,
                           const int frame, const DelayType w,
                           const int x, const int y,
                           const int rw, const int rh);

  //! overlay an image onto a video and copy it to the current YUV
  //  overlay, displaying it. Optionally, the user can specify the
  //  number of threads to use.
  void displayVideoOverlay_image(const VideoFrame& frame,
                                 const int framenum,
                                 const DelayType dly,
                                 const Image<PixRGB<byte> >& img,
                                 const PixRGB<byte>& transpix,
                                 const uint threads = 1);

  //! overlay an small patch onto a video and copy it to the current
  //  YUV overlay, displaying it. This may be faster than
  //  displayVideoOverlay_image() for small patches (like fixation
  //  points), but it probably wont look as clean as there is no color
  //  averaging between frame and image. X,Y position is the upper
  //  left corner of the patch
  void displayVideoOverlay_patch(const VideoFrame& frame,
                                 const int framenum,
                                 const DelayType dly,
                                 const uint x,
                                 const uint y,
                                 const Image<PixRGB<byte> >& img);

  //@}

  // ######################################################################
  /*! @name Event logging functions */
  //@{

  //! Pass-through to EventLog::pushEvent()
  /*! Note that this is a no-op if no EventLog has been registered
    with us through a call to setEventLog(). */
  inline void pushEvent(const std::string& msg);

  //! Pass-through to EventLog::pushEventBegin()
  /*! Note that this is a no-op if no EventLog has been registered
    with us through a call to setEventLog(). */
  inline void pushEventBegin(const std::string& msg);

  //! Pass-through to EventLog::pushEventEnd()
  /*! Note that this is a no-op if no EventLog has been registered
    with us through a call to setEventLog(). */
  inline void pushEventEnd(const std::string& msg);

  //@}

protected:
  OModelParam<Dims> itsDims;      //!< screen resolution
  OModelParam<int> itsPriority;   //!< priority for SCHED_FIFO, or 0 for normal
  OModelParam<float> itsRefreshDelay; //!< desired refresh delay in usec
  OModelParam<bool> itsFullscreen; //!< whether to run in a fullscreen window
  NModelParam<bool> itsSlaveMode;  //!< slave mode (someone else opens screen)
  OModelParam<uint> itsVBlankKludge; //!< workaround for non-working vblank
  OModelParam<Rectangle> itsSyncRect; //!< patch for syncing with photodiode

  void start2();  //!< get started
  void stop1();   //!< get stopped

  Timer itsTimer;           // keep track of time
  uint64 itsLastSync;       // time of last screen sync
  uint64 itsLastOvlDisplay; // time of overlay display

  float itsRefreshTolerance;// fraction of refresh delay that we allow before marking frames as SLOW or FAST

  // update double-buffering and sync to vertical blanking:
  void syncScreen(const bool vsync = true, const bool checkdelay = false,
                  const bool quiet = false);

  // display stuff using SDL:
  SDL_Surface *itsScreen;
  SDL_Overlay *itsOverlay;
  SDL_Overlay *videoOverlay;

private:
  Rectangle itsSync;
  nub::soft_ref<EventLog> itsEventLog;  // log stuff if desired

};

// ######################################################################
// ######################################################################
// ####################      Inlined functions       ####################
// ######################################################################
// ######################################################################

// ######################################################################
inline Dims SDLdisplay::getDims() const
{ return itsDims.getVal(); }

// ######################################################################
inline int SDLdisplay::getWidth() const
{ return itsDims.getVal().w(); }

// ######################################################################
inline int SDLdisplay::getHeight() const
{ return itsDims.getVal().h(); }

// ######################################################################
inline int SDLdisplay::getBytesPerPixel() const
{ return itsScreen->format->BytesPerPixel; }

// ######################################################################
inline Uint32 SDLdisplay::getUint32color(const PixRGB<byte>& col) const
{ return SDL_MapRGB(itsScreen->format, col.red(), col.green(), col.blue()); }

// ######################################################################
inline PixRGB<byte> SDLdisplay::getRGBcolor(const Uint32 col) const
{ LFATAL("unimplemented!"); return PixRGB<byte>(0,0,0); }

// ######################################################################
inline void SDLdisplay::showCursor(const bool showit)
{
  if (showit) SDL_ShowCursor(SDL_ENABLE);
  else SDL_ShowCursor(SDL_DISABLE);
}

// ######################################################################
inline void SDLdisplay::lockScreen()
{
  if (SDL_MUSTLOCK(itsScreen))
    {
      if (SDL_LockSurface(itsScreen) < 0)
        LFATAL("Cannot lock screen: %s", SDL_GetError());
    }
}
// ######################################################################
inline void SDLdisplay::unlockScreen()
{ if (SDL_MUSTLOCK(itsScreen)) SDL_UnlockSurface(itsScreen); }

// ######################################################################
inline void SDLdisplay::pushEvent(const std::string& msg)
{ if (itsEventLog.isValid()) itsEventLog->pushEvent(msg); }

// ######################################################################
inline void SDLdisplay::pushEventBegin(const std::string& msg)
{ if (itsEventLog.isValid()) itsEventLog->pushEventBegin(msg); }

// ######################################################################
inline void SDLdisplay::pushEventEnd(const std::string& msg)
{ if (itsEventLog.isValid()) itsEventLog->pushEventEnd(msg); }


#endif // HAVE_SDL_SDL_H

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
