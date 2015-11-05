/*!@file GUI/SDLdisplay.C Fast full-screen displays using SDL */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/SDLdisplay.C $
// $Id: SDLdisplay.C 14170 2010-10-27 07:04:19Z ilink $
//

#ifdef HAVE_SDL_SDL_H

#include "GUI/SDLdisplay.H"

#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "GUI/GUIOpts.H"
#include "Image/Image.H"
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Util/MathFunctions.H"
#include "Video/VideoFrame.H"
#include "Video/RgbConversion.H"
#include "rutz/unixcall.h"
#include "Util/sformat.H"

#include <ctime>
#include <fstream>
#include <sched.h>
#include <pthread.h>
#ifdef HAVE_SYS_IO_H
#include <sys/io.h>       // for iopl()
#endif
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef MACHINE_OS_DARWIN
#include <dlfcn.h>
#endif

//! Special log function for SDL-related fatal errors
#define SDLFATAL(f, x...) \
  LFATAL("%s", (sformat(f , ## x) + \
                " (" + std::string(SDL_GetError()) + ")").c_str());

namespace
{
  // ######################################################################
  bool tryvidformat2sdlmode(const VideoFormat vidformat,
                            Uint32* result)
  {
    // copied from SDL/SDL_video.h:

    /* The most common video overlay formats.
       For an explanation of these pixel formats, see:
       http://www.webartz.com/fourcc/indexyuv.htm

       For information on the relationship between color spaces, see:
       http://www.neuro.sfc.keio.ac.jp/~aly/polygon/info/color-space-faq.html
    */
    // SDL_YV12_OVERLAY  Planar mode: Y + V + U  (3 planes)
    // SDL_IYUV_OVERLAY  Planar mode: Y + U + V  (3 planes)
    // SDL_YUY2_OVERLAY  Packed mode: Y0+U0+Y1+V0 (1 plane)
    // SDL_UYVY_OVERLAY  Packed mode: U0+Y0+V0+Y1 (1 plane)
    // SDL_YVYU_OVERLAY  Packed mode: Y0+V0+Y1+U0 (1 plane)

    switch (vidformat)
      {
        // NOTE: SDL_IYUV_OVERLAY might seem a more natural choice for
        // VIDFMT_YUV420P, since with SDL_YV12_OVERLAY we have to swap
        // the U and V components (see displayVideoOverlay() below);
        // BUT, it seems that SDL_IYUV_OVERLAY gives bogus results, in
        // that black pixels end up rendered as medium gray, with the
        // rest of the image looking correspondingly washed out.

      case VIDFMT_YUV420P:  *result = SDL_YV12_OVERLAY; return true;

      case VIDFMT_UYVY:     *result = SDL_UYVY_OVERLAY; return true;
      case VIDFMT_YUV422:   *result = SDL_UYVY_OVERLAY; return true;

      case VIDFMT_YUYV:     *result = SDL_YUY2_OVERLAY; return true;

      case VIDFMT_AUTO:     break; /* unsupported mode, fall-through */
      case VIDFMT_GREY:     break; /* unsupported mode, fall-through */
      case VIDFMT_RAW:      break; /* unsupported mode, fall-through */
      case VIDFMT_RGB24:    break; /* unsupported mode, fall-through */
      case VIDFMT_RGB32:    break; /* unsupported mode, fall-through */
      case VIDFMT_RGB555:   break; /* unsupported mode, fall-through */
      case VIDFMT_RGB565:   break; /* unsupported mode, fall-through */
      case VIDFMT_YUV410:   break; /* unsupported mode, fall-through */
      case VIDFMT_YUV410P:  break; /* unsupported mode, fall-through */
      case VIDFMT_YUV411:   break; /* unsupported mode, fall-through */
      case VIDFMT_YUV411P:  break; /* unsupported mode, fall-through */
      case VIDFMT_YUV420:   break; /* unsupported mode, fall-through */
      case VIDFMT_YUV422P:  break; /* unsupported mode, fall-through */
      case VIDFMT_YUV24:    break; /* unsupported mode, fall-through */
      case VIDFMT_YUV444:   break; /* unsupported mode, fall-through */
      case VIDFMT_YUV444P:  break; /* unsupported mode, fall-through */
      case VIDFMT_HM12:  break; /* unsupported mode, fall-through */
      case VIDFMT_BAYER_GB: break;  /* unsupported mode, fall-through */
      case VIDFMT_BAYER_GR: break;  /* unsupported mode, fall-through */
      case VIDFMT_BAYER_RG: break;  /* unsupported mode, fall-through */
      case VIDFMT_BAYER_BG: break;  /* unsupported mode, fall-through */
      case VIDFMT_BAYER_GB12: break;  /* unsupported mode, fall-through */
      case VIDFMT_BAYER_GR12: break;  /* unsupported mode, fall-through */
      case VIDFMT_BAYER_RG12: break;  /* unsupported mode, fall-through */
      case VIDFMT_BAYER_BG12: break;  /* unsupported mode, fall-through */
      case VIDFMT_MJPEG: break;  /* unsupported mode, fall-through */
      }

    return false;
  }

// ######################################################################
  Uint32 vidformat2sdlmode(const VideoFormat vidformat)
  {
    Uint32 res;

    if (tryvidformat2sdlmode(vidformat, &res))
      return res;

    // else... the conversion failed, so give up:
    LFATAL("No SDL overlay mode matching VideoFormat %s (%d)",
           convertToString(vidformat).c_str(), int(vidformat));
    /* can't happen */ return 0;
  }

  //struct to hold video info for a pthread
  struct RgbYuvOverlayStruct
  {
    RgbYuvOverlayStruct(const byte *VidY, const byte *VidU, const byte *VidV,
                        byte *OutY, byte *OutU, byte *OutV,
                        const byte *Rgb,
                        const byte Tr, const byte Tg, const byte Tb,
                        const int W, const int H) :
      vidy(VidY), vidu(VidU), vidv(VidV),
      outy(OutY), outu(OutU), outv(OutV),
      src(Rgb), tr(Tr), tg(Tg), tb(Tb), w(W), h(H) { };

    const byte *vidy, *vidu, *vidv;
    byte  *outy, *outu, *outv;
    const byte *src, tr, tg, tb;
    const int w, h;
  };

// ######################################################################
  void addSyncRect(SDL_Overlay const* ovl, const Dims& dims, 
                   const Rectangle& rect, const int frame, 
                   const Uint32 sdlformat)
  {
    if (rect.isValid())
      {
        if ((sdlformat == SDL_IYUV_OVERLAY) || (sdlformat == SDL_YV12_OVERLAY))
          {

            //pointers to image components
            byte* outy = ovl->pixels[0];
            byte* outu = ovl->pixels[1];
            byte* outv = ovl->pixels[2];
            
            //position and size variables
            const int x(rect.left()), y(rect.top());
            const int w(dims.w());
            const int dw(rect.width()), dh(rect.height());
            const int cpos = x/2+1 + (w/2)*(y/2);
            const int dw2 = dw/2;
            const int w2 = w/2;
            const int val = frame % 2 == 0 ? 255 : 0; //fill value

            //put image data pointers to patches patches position 
            outy += x  + w*y;
            outu += cpos;
            outv += cpos;

            //set the pixel data for each row
            for (int ii = 0; ii < dh; ++ii)
              {
                memset(outy, val, dw);
                if (ii % 2 == 0)
                  {
                    memset(outu, 128, dw2);
                    memset(outv, 128, dw2);
                    outu += w2;
                    outv += w2;
                  }
                outy += w;
              }
          }
        else 
          LFATAL("Cannot add syncing patch to this video format");
      }
  }

// ######################################################################
  void *RgbYuvOverlay(void *arg)
  {
    RgbYuvOverlayStruct *data = (RgbYuvOverlayStruct*)arg;
    int count[data->w], rsum[data->w], gsum[data->w], bsum[data->w];

    //loop through through our macroblocks, rows
    for (int j = 0; j < data->h; j += 2)
      {
        //!loop over macroblock columns
        for (int i = 0; i < data->w; i += 2)
          {
            count[i] = 0;
            rsum[i] = 0;
            gsum[i] = 0;
            bsum[i] = 0;

            //first pixel in macro block
            byte br = *data->src++;
            byte bg = *data->src++;
            byte bb = *data->src++;

            if ( (br == data->tr) && (bg == data->tg) && (bb == data->tb) )
              *data->outy++ = *data->vidy++;
            else
              {
                double r = double(br);
                double g = double(bg);
                double b = double(bb);

                double yf = VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
                *data->outy++ = clamped_rounded_convert<byte>(yf);
                data->vidy++;

                rsum[i] += (int)r;
                gsum[i] += (int)g;
                bsum[i] += (int)b;
                count[i]++;
              }
            //end first element of block


            //second pixel in macro block
            br = *data->src++;
            bg = *data->src++;
            bb = *data->src++;

            if ( (br == data->tr) && (bg == data->tg) && (bb == data->tb) )
              *data->outy++ = *data->vidy++;
            else
              {
                double r = double(br);
                double g = double(bg);
                double b = double(bb);

                double yf = VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
                *data->outy++ = clamped_rounded_convert<byte>(yf);
                data->vidy++;

                rsum[i] += (int)r;
                gsum[i] += (int)g;
                bsum[i] += (int)b;
                count[i]++;
              }
            //end second element of block

          }

        // second row of each block
        for (int i = 0; i < data->w; i += 2)
          {
            //first pixel in second row of macro block
            byte br = *data->src++;
            byte bg = *data->src++;
            byte bb = *data->src++;

            if ( (br == data->tr) && (bg == data->tg) && (bb == data->tb) )
              *data->outy++ = *data->vidy++;
            else
              {
                double r = double(br);
                double g = double(bg);
                double b = double(bb);

                double yf = VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
                *data->outy++ = clamped_rounded_convert<byte>(yf);
                data->vidy++;

                rsum[i] += (int)r;
                gsum[i] += (int)g;
                bsum[i] += (int)b;
                count[i]++;
              }
            //end first element of second row of block


            //second pixel in second row of macro block
            br = *data->src++;
            bg = *data->src++;
            bb = *data->src++;

            if ( (br == data->tr) && (bg == data->tg) && (bb == data->tb) )
              *data->outy++ = *data->vidy++;
            else
              {
                double r = double(br);
                double g = double(bg);
                double b = double(bb);

                double yf = VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
                *data->outy++ = clamped_rounded_convert<byte>(yf);
                data->vidy++;

                rsum[i] += (int)r;
                gsum[i] += (int)g;
                bsum[i] += (int)b;
                count[i]++;
              }

            //end second element of second row of block

            //average color data;
            if (count[i] > 0)
              {
                //get the vide frame color of the macroblock
                double ut = *data->vidu++;
                double vt = *data->vidv++;

                //get the average rgb color in the block and convert to YUV
                double r = rsum[i] / count[i];
                double g = gsum[i] / count[i];
                double b = bsum[i] / count[i];
                double uf = VIDEOYUV_U_R*r + VIDEOYUV_U_G*g
                  + VIDEOYUV_U_B*b + VIDEOYUV_UV_OFFSET;
                double vf = VIDEOYUV_V_R*r + VIDEOYUV_V_G*g
                  + VIDEOYUV_V_B*b + VIDEOYUV_UV_OFFSET;

                uf = uf * count[i]/4.0 + ut * (4-count[i])/4.0;
                vf = vf * count[i]/4.0 + vt * (4-count[i])/4.0;

                *data->outu++ =  clamped_rounded_convert<byte>(uf);
                *data->outv++ =  clamped_rounded_convert<byte>(vf);
              }
            else
              {
                *data->outu++ = *data->vidu++;
                *data->outv++ = *data->vidv++;
              }
          }//end loop over send row of macro block
      }//end loop over vertical pixels
    return NULL;
  }
}

// ######################################################################
SDLdisplay::SDLdisplay(OptionManager& mgr, const std::string& descrName,
                             const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsDims(&OPT_SDLdisplayDims, this), // see Psycho/PsychoOpts.{H,C}
  itsPriority(&OPT_SDLdisplayPriority, this),
  itsRefreshDelay(&OPT_SDLdisplayRefreshUsec, this),
  itsFullscreen(&OPT_SDLdisplayFullscreen, this),
  itsSlaveMode("SDLslaveMode", this, false),
  itsVBlankKludge(&OPT_SDLdisplayVBlankKludge, this),
  itsSyncRect(&OPT_SDLdisplaySyncRect, this),
  itsTimer(1000000),  // use microsecond resolution
  itsLastSync(0ULL), itsLastOvlDisplay(0ULL),
  itsRefreshTolerance(0.05F),
  itsScreen(0), itsOverlay(NULL), 
  itsSync(),
  itsEventLog()
{  }

// ######################################################################
SDLdisplay::~SDLdisplay()
{
  // make sure we closedown SDL (and switch out of full-screen mode if
  // applicable) even in cases our stop1() function is not properly
  // called, for example because we are quitting on an LFATAL or other
  // exception:
  if (itsScreen) { LINFO("Closing down SDL..."); SDL_Quit(); }
}

// ######################################################################
void SDLdisplay::setEventLog(nub::soft_ref<EventLog> elog)
{ itsEventLog = elog; }

// ######################################################################
void SDLdisplay::openDisplay()
{

#ifdef MACHINE_OS_DARWIN
  // on MacOS X we need to get the cocoa framework started before
  // we init SDL. See http://www.ogre3d.org/wiki/index.php/MacLibrary
  void* cocoa_lib = dlopen("/System/Library/Frameworks/Cocoa.framework/Cocoa",RTLD_LAZY);
  void (*nsappload)(void) = (void(*)())dlsym(cocoa_lib, "NSApplicationLoad");
  nsappload();
#endif

  // open a screen and show a black image:
  if (SDL_Init(SDL_INIT_VIDEO) < 0) SDLFATAL("SDL init failed");

  uint32 flags = SDL_SWSURFACE | SDL_HWSURFACE | SDL_DOUBLEBUF | SDL_HWACCEL;
  if (itsFullscreen.getVal()) flags |= SDL_FULLSCREEN;
  itsScreen = SDL_SetVideoMode(itsDims.getVal().w(), itsDims.getVal().h(), 32, flags);

  if (itsScreen == NULL) SDLFATAL("Cannot open screen");

  itsLastSync = 0ULL;
  clearScreen(PixRGB<byte>(0));
  showCursor(false);
}


// ######################################################################
void SDLdisplay::start2()
{
  if (itsRefreshDelay.getVal() <= 0.0f)
    LFATAL("--%s should be strictly greater than zero, but got --%s=%f",
           itsRefreshDelay.getOptionDef()->longoptname,
           itsRefreshDelay.getOptionDef()->longoptname,
           itsRefreshDelay.getVal());

  // switch to SCHED_FIFO scheduling?
  int pri = itsPriority.getVal();
  if (pri)
    {
      // high-priority mode will also trigger hard polling on the VGA registers to sync our displays with the vertical
      // blanking. On recent versions of Linux, this only works when we are in fullscreen mode, otherwise the polling
      // just blocks forever and never finds a vertical blanking. So let's here enforce that we should use fullscreen:
      if (itsFullscreen.getVal() == false) LFATAL("non-zero --sdl-priority only works in fullscreen mode (--fs)");

#ifndef HAVE_SYS_IO_H
      LFATAL("this configuration lacks <sys/io.h>, so it does not support non-zero --sdl-priority");
#else
      // become root:
      if (setuid(0) == -1)
        PLFATAL("I need to run as root when --sdl-priority is non-zero (%d)", pri);

      // set our scheduler policy and priority:
      struct sched_param params;
      int minpri = sched_get_priority_min(SCHED_FIFO);
      int maxpri = sched_get_priority_max(SCHED_FIFO);
      if (pri < minpri || pri > maxpri)
        LFATAL("Invalid priority %d, range [%d..%d]", pri, minpri, maxpri);
      params.sched_priority = pri;

      if (sched_setscheduler(0, SCHED_FIFO, &params) == -1)
        PLFATAL("Cannot switch to SCHED_FIFO scheduling and priority %d", pri);

      // avoid getting swapped around:
      if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        PLFATAL("Cannot lock memory and prevent swapping");

      LINFO("Process priority set to %d and swap locked.", pri);
#endif // HAVE_SYS_IO_H
    }

  if (itsSlaveMode.getVal())
    {
      LINFO("Running in slave mode...");
      itsScreen = SDL_GetVideoSurface();
    }
  else
    {
#ifdef MACHINE_OS_DARWIN
      // on MacOS X we need to get the cocoa framework started before
      // we init SDL. See http://www.ogre3d.org/wiki/index.php/MacLibrary
      void* cocoa_lib = dlopen("/System/Library/Frameworks/Cocoa.framework/Cocoa",RTLD_LAZY);
      void (*nsappload)(void) = (void(*)())dlsym(cocoa_lib, "NSApplicationLoad");
      nsappload();
#endif

      // open a screen and show a flat grey image:
      if (SDL_Init(SDL_INIT_VIDEO) < 0) SDLFATAL("SDL init failed");

      uint32 flags = SDL_SWSURFACE | SDL_HWSURFACE | SDL_DOUBLEBUF | SDL_HWACCEL;
      if (itsFullscreen.getVal()) flags |= SDL_FULLSCREEN;
      itsScreen = SDL_SetVideoMode(itsDims.getVal().w(), itsDims.getVal().h(), 32, flags);
      if (itsScreen == NULL) SDLFATAL("Cannot open screen");

      itsLastSync = 0ULL;
    }

  // get name of video driver and display it:
  char vname[256]; SDL_VideoDriverName(vname, 256);
  LINFO("Using SDL video driver '%s'", vname);

  // Enable keycode translations:
  SDL_EnableUNICODE(1);

  // switch to max I/O privilege for waitNextRequestedVsync() to work, unless
  // using the VBlank kludge, in which case we don't need I/O privileges:
  if (pri) {
    if (itsVBlankKludge.getVal() == 0) {
#ifndef HAVE_SYS_IO_H
      LFATAL("this configuration lacks <sys/io.h>, so it does not support non-zero --sdl-priority");
#else
      if (iopl(3)) PLFATAL("cannot iopl(3)");
#endif
    } else
      LINFO("Using Vertical-Blanking workaround with a delay of %u microseconds", itsVBlankKludge.getVal());
    }

  //check to see if our blinking rectangle used to sync the video is off screen
  if (itsSyncRect.getVal().area() > 0)
    {
      Rectangle r = itsSyncRect.getVal();
      if ( (r.left() < 0) || (r.rightO() > itsDims.getVal().w()) ) 
        LFATAL("The rectangle (%d-%d) set using --sdl-sync-rect is "
               "horizontally off the screen(%d-%d)", r.left(), r.rightO(), 
               0, itsDims.getVal().w());
      if ( (r.top() < 0) || (r.bottomO() > itsDims.getVal().h()) ) 
        LFATAL("The rectangle (%d-%d) set using --sdl-sync-rect is "
               "vertically off the screen(%d-%d)", r.top(), r.bottomO(), 
               0, itsDims.getVal().h());

      itsSync = itsSyncRect.getVal();
    }


  
  // the epoch is now:
  itsTimer.reset();
  char buf1[100], buf2[100];
  time_t ti = time(NULL);
  strncpy(buf1, ctime(&ti), sizeof(buf1));
  buf1[sizeof(buf1)-1] = '\0';
  // kill a trailing '\n' on the ctime() string, if it exists
  if (buf1[strlen(buf1)-1] == '\n') buf1[strlen(buf1)-1] = '\0';
  snprintf(buf2, sizeof(buf2), "##### START %s #####", buf1);
  pushEvent(buf2);

  itsLastSync = 0ULL;
  clearScreen(PixRGB<byte>(0, 0, 0));
  showCursor(false);
}

// ######################################################################
void SDLdisplay::closeDisplay()
{
  // destro any eisting overlay:
  if (itsOverlay) { destroyYUVoverlay(); itsOverlay = NULL; }

  // close the screen:
  if (itsSlaveMode.getVal() == false && itsScreen)
    { LINFO("Closing down SDL..."); SDL_Quit(); itsScreen = NULL; }

  // if running at high priority, do some related cleanup:
  if (itsPriority.getVal())
    {
#ifndef HAVE_SYS_IO_H
      LFATAL("this configuration lacks <sys/io.h>, so it does not support "
             "non-zero --sdl-priority");
#else
      // get back to normal scheduling and zero priority:
      struct sched_param params; params.sched_priority = 0;
      if (sched_setscheduler(0, SCHED_OTHER, &params) == -1)
        PLFATAL("Cannot switch back to SCHED_OTHER");

      // allow swapping:
      if (munlockall() == -1) PLFATAL("Cannot unlock memory");
#endif // HAVE_SYS_IO_H
    }
}

// ######################################################################
void SDLdisplay::stop1()
{
  closeDisplay();
}

// ######################################################################
void SDLdisplay::clearScreen(const PixRGB<byte> col, const bool vsync)
{
  pushEventBegin("clearScreen");

  SDL_Rect rect;
  rect.x = 0; rect.y = 0; rect.w = getWidth(); rect.h = getHeight();

  SDL_FillRect(itsScreen, &rect, getUint32color(col));

  //also paint a black patch if external syncing is requested
  if (itsSync.isValid())
    {
      SDL_Rect r;
      r.x = itsSync.left(); r.y = itsSync.top(); 
      r.w = itsSync.width(); r.h = itsSync.height();
      SDL_FillRect(itsScreen, &r, getUint32color(PixRGB<byte>(0)));
    }

  syncScreen(vsync, false, true);

  pushEventEnd("clearScreen");
}

// ######################################################################
void SDLdisplay::clearBackBuffer()
{
  // Lock the screen for direct access to the pixels
  lockScreen();

  // brutally clear the screen:
  int bpp = getBytesPerPixel();
  memset(itsScreen->pixels, 0, itsDims.getVal().sz() * bpp);

  // unlock the screen:
  unlockScreen();
}

// ######################################################################
int SDLdisplay::waitForKeyTimeout(double timeout, bool doWait)
{
  // clear the input buffer before we get started if doWait is set to true
  if (doWait) while (checkForKey() != -1) ;

  pushEventBegin("waitForKeyTimeout");

	Timer tt;
	double ttime;
	timeout = timeout/1000;
	int key;
  do {
		key = checkForKey();	
		ttime = tt.getSecs();
	} while (key == -1 && ttime < timeout);

  char c; if (key >= 32 && key < 128) c = char(key); else c = '?';
  pushEventEnd(sformat("waitForKeyTimeout - got %d (%c), time %f ms", key, c, ttime*1000));

  // abort if it was ESC:
  if (key == 27) LFATAL("Aborting on [ESC] key");

  return key;
}

// ######################################################################
int SDLdisplay::waitForKey(bool doWait)
{
  // clear the input buffer before we get started if doWait is set to true
  if (doWait) while (checkForKey() != -1) ;

  pushEventBegin("waitForKey");
  SDL_Event event;

  do { SDL_WaitEvent(&event); } while (event.type != SDL_KEYDOWN);

  int key = event.key.keysym.unicode;
  char c; if (key >= 32 && key < 128) c = char(key); else c = '?';
  pushEventEnd(sformat("waitForKey - got %d (%c)", key, c));

  // abort if it was ESC:
  if (key == 27) LFATAL("Aborting on [ESC] key");

  return key;
}

// ######################################################################
int SDLdisplay::waitForMouseClick(bool doWait)
{
  // clear the input buffer before we get started if doWait is set to true
  if (doWait) while (checkForMouseClick() != -1) ;

  pushEventBegin("waitForMouseClick");
  SDL_Event event;

  do { SDL_WaitEvent(&event); } while (event.type != SDL_MOUSEBUTTONDOWN);

  int i = 0;  // will be returned if any other button than left or right
  if (event.button.button == SDL_BUTTON_LEFT) {
    pushEvent(sformat("waitForMouseClick - got left button clicked"));
    i = 1 ;
  }
  if (event.button.button == SDL_BUTTON_RIGHT) {
    pushEvent(sformat("checkForMouseClick - got right button clicked"));
    i = 2 ;
  }

  return i;
}

// ######################################################################
int SDLdisplay::waitForMouseWheelEvent(bool doWait)
{
  // clear the input buffer before we get started if doWait is set to true
  if (doWait) while (checkForMouseClick() != -1) ;

  pushEventBegin("waitForMouseWheelEvent");
  SDL_Event event;

  do { SDL_WaitEvent(&event); } while (event.button.button != SDL_BUTTON_MIDDLE);

  int i = 0;  // will be returned if any other button than left or right
  if (event.type == SDL_MOUSEBUTTONDOWN) {
    pushEvent(sformat("waitForMouseClick - got left button clicked"));
    i = 1 ;
  }
  if (event.type == SDL_MOUSEBUTTONUP) {
    pushEvent(sformat("checkForMouseClick - got right button clicked"));
    i = 2 ;
  }

  return i;
}

// ######################################################################
int SDLdisplay::checkForKey()
{
  SDL_Event event;

  while(SDL_PollEvent(&event))
    {
      if (event.type == SDL_KEYDOWN)
        {
          int key = event.key.keysym.unicode;
          char c; if (key >= 32 && key < 128) c = char(key); else c = '?';
          pushEvent(sformat("checkForKey - got %d (%c)", key, c));

          // abort if it was ESC:
          if (key == 27) LFATAL("Aborting on [ESC] key");

          return key;
        }
      // ignore other events
    }

  // there was no event in the event queue:
  return -1;
}

// ######################################################################
int SDLdisplay::checkForMouseClick()
{
  SDL_Event event;

  while(SDL_PollEvent(&event))
    {
      if (event.type == SDL_MOUSEBUTTONDOWN)
        {
          if(event.button.button == SDL_BUTTON_LEFT) {
            pushEvent(sformat("checkForMouseClick - left button clicked"));
            return 1 ;
          }
          if(event.button.button == SDL_BUTTON_RIGHT) {
            pushEvent(sformat("checkForMouseClick - right button clicked"));
            return 2 ;
          }
        }
      // ignore other events
    }

  // there was no event in the event queue:
  return -1;
}
// #####################################################################
std::string SDLdisplay::getString(char terminator ='\n')
{
  std::string inputString;
  int k;
  do { k = waitForKey();
    inputString += k;
    LINFO("%d, %c %d", k, char(k), int(terminator));
  } while(k != 13);
  LINFO("gotString %s", inputString.c_str());

  return inputString;
}

// ######################################################################
void SDLdisplay::displayImage(const Image< PixRGB<byte> >& img,
                              const bool resiz, const PixRGB<byte> bgcol,
                              const int frame, const bool vsync)
{
  SDL_Surface* surf = makeBlittableSurface(img, resiz, bgcol);
  displaySurface(surf, frame, vsync);
  SDL_FreeSurface(surf);
}

// ######################################################################
SDL_Surface*
SDLdisplay::makeBlittableSurface(const Image< PixRGB<byte> >& img,
                                 const bool resiz, const PixRGB<byte> bgcol)
{
  Image< PixRGB<byte> > image;
  //LINFO("image: w, h : %d, %d",image.getWidth(),image.getHeight());
  //LINFO("img  : w, h : %d, %d",img.getWidth(),img.getHeight());
  //LINFO("pt: (%d,%d)", (getWidth() - img.getWidth()) / 2,
  //                              (getHeight() - img.getHeight()) / 2);

  // resize/embed if necessary:
  if (this->itsDims.getVal() != img.getDims())
    {
      image = Image< PixRGB<byte> >(itsDims.getVal(), NO_INIT);

      if (resiz)
        {
          // stretch and embed:
          inplaceEmbed(image, img, image.getBounds(), bgcol, true);
        }
      else
        {
          // center the image on the screen:
          image.clear(bgcol);
          inplacePaste(image, img,
                       Point2D<int>( (getWidth() - img.getWidth()) / 2,
                                (getHeight() - img.getHeight()) / 2));
        }
    }
  else
    image = img;

  ASSERT(image.getDims() == this->itsDims.getVal());

  if (itsSync.isValid())
    {
      //! Draw a filled rectangle with white
      drawFilledRect(image, itsSync, PixRGB<byte>(255,255,255));
    }

  // create an SDL_Surface suitable for a direct blit:
  SDL_Surface *tmpsurf =
    SDL_CreateRGBSurfaceFrom(image.getArrayPtr(), image.getWidth(),
                             image.getHeight(), 24, 3 * image.getWidth(),
                             0x0000ff, 0x00ff00, 0xff0000, 0x0);
  SDL_Surface *surf = SDL_DisplayFormat(tmpsurf);
  SDL_FreeSurface(tmpsurf);

  return surf;
}

// ######################################################################
void SDLdisplay::displaySurface(SDL_Surface *img, const int frame,
                                const bool vsync)
{
  // special frame -2 is to log both start and end of this function:
  if (frame == -2) pushEventBegin("displayImage");

  // if this is our first frame of a movie, let's start fresh with a
  // sync to vertical blanking:
  if (vsync && frame == 0)
    waitNextRequestedVsync(false, false); // will update itsLastSync

  // let's blit the image into our back buffer:
  if (SDL_BlitSurface(img, NULL, itsScreen, NULL) == -1)
    SDLFATAL("Blit failed");

  // swap the buffers & wait for vertical blanking pulse:
  syncScreen(vsync, (frame >= 0), false); // will update itsLastSync

  if (frame >= 0) pushEvent(sformat("displayImage - frame %d", frame));
  else if (frame == -2) pushEventEnd("displayImage");
}


// ######################################################################
void SDLdisplay::displaySDLSurfacePatch(SDL_Surface* surf, SDL_Rect* offset,
                                        SDL_Rect* clip, const int frame,
                                        const bool vsync, const bool flip)
{
  // special frame -2 is to log both start and end of this function:
  if (frame == -2) pushEventBegin("displaySurfacePatch");

  // if this is our first frame of a movie, let's start fresh with a
  // sync to vertical blanking:
  if (vsync && frame == 0)
    waitNextRequestedVsync(false, false); // will update itsLastSync

  if (SDL_BlitSurface(surf, NULL, itsScreen, offset) == -1)
    SDLFATAL("Blit failed");


  // swap the buffers & wait for vertical blanking pulse:
  if (flip) syncScreen(vsync, (frame >= 0), false); // will update itsLastSync

  if (frame >= 0) pushEvent(sformat("displaySurfacePatch - frame %d", frame));
  else if (frame == -2) pushEventEnd("displaySurfacePatch");
}


// ######################################################################
void SDLdisplay::displayImagePatch(const Image< PixRGB<byte> >& image,
                                   const Point2D<int>& pos, const int frame,
                                   const bool vsync, const bool flip)
{
  // special frame -2 is to log both start and end of this function:
  if (frame == -2) pushEventBegin("displayImagePatch");

  // create an SDL_Surface suitable for a direct blit:
  SDL_Surface *tmpsurf =
    SDL_CreateRGBSurfaceFrom(const_cast<Image< PixRGB<byte> >& >(image).
                             getArrayPtr(), image.getWidth(),
                             image.getHeight(), 24, 3 * image.getWidth(),
                             0x0000ff, 0x00ff00, 0xff0000, 0x0);
  SDL_Surface *surf = SDL_DisplayFormat(tmpsurf);
  SDL_FreeSurface(tmpsurf);

  // if this is our first frame of a movie, let's start fresh with a
  // sync to vertical blanking:
  if (vsync && frame == 0)
    waitNextRequestedVsync(false, false); // will update itsLastSync

  // let's blit the image into our back buffer:
  SDL_Rect r; r.x = pos.i; r.y = pos.j;
  r.w = image.getWidth(); r.h = image.getHeight();
  if (SDL_BlitSurface(surf, NULL, itsScreen, &r) == -1)
    SDLFATAL("Blit failed");
  SDL_FreeSurface(surf);

  // swap the buffers & wait for vertical blanking pulse:
  if (flip) syncScreen(vsync, (frame >= 0), false); // will update itsLastSync

  if (frame >= 0) pushEvent(sformat("displayImagePatch - frame %d", frame));
  else if (frame == -2) pushEventEnd("displayImagePatch");
}

// ######################################################################
bool SDLdisplay::hasYUVoverlay() const
{
  return (itsOverlay != 0);
}

// ######################################################################
void SDLdisplay::createYUVoverlay(const Uint32 format,
                                  const int w, const int h)
{
  if (itsOverlay) destroyYUVoverlay();

  if (format != SDL_YV12_OVERLAY &&
      format != SDL_IYUV_OVERLAY &&
      format != SDL_YUY2_OVERLAY &&
      format != SDL_UYVY_OVERLAY &&
      format != SDL_YVYU_OVERLAY)
    LFATAL("Invalid SDL overlay format");

  itsOverlay = SDL_CreateYUVOverlay(w, h,
                                    format, itsScreen);
  if (itsOverlay == NULL) SDLFATAL("Cannot create YUV overlay");
  itsLastOvlDisplay = 0ULL;
}

// ######################################################################
void SDLdisplay::createYUVoverlay(const Uint32 format)
{
  if (itsOverlay) destroyYUVoverlay();

  if (format != SDL_YV12_OVERLAY &&
      format != SDL_IYUV_OVERLAY &&
      format != SDL_YUY2_OVERLAY &&
      format != SDL_UYVY_OVERLAY &&
      format != SDL_YVYU_OVERLAY)
    LFATAL("Invalid SDL overlay format");


  itsOverlay = SDL_CreateYUVOverlay(getWidth(), getHeight(),
                                    format, itsScreen);
  if (itsOverlay == NULL) SDLFATAL("Cannot create YUV overlay");
  itsLastOvlDisplay = 0ULL;
}

// ######################################################################
void SDLdisplay::destroyYUVoverlay()
{
  if (itsOverlay == NULL) LFATAL("I don't have a YUV overlay");
  SDL_FreeYUVOverlay(itsOverlay);
  itsOverlay = NULL;
}

// ######################################################################
SDL_Overlay* SDLdisplay::lockYUVoverlay()
{
  if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");
  SDL_LockYUVOverlay(itsOverlay);
  return itsOverlay;
}

// ######################################################################
void SDLdisplay::unlockYUVoverlay()
{
  if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");
  SDL_UnlockYUVOverlay(itsOverlay);
}

// ######################################################################
void SDLdisplay::displayYUVoverlay(const int frame, const DelayType dly,
                                   const int x, const int y,
                                   const int w, const int h)
{
  if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");

  SDL_Rect r; r.x = x; r.y = y; r.w = w; r.h = h;

  // sync and log an event:
  switch (dly)
    {
    case NO_WAIT:
      {
        // display the overlay:
        if (SDL_DisplayYUVOverlay(itsOverlay, &r) == -1)
          SDLFATAL("Overlay blit failed");

        // if we are not vsync'ing, probably our timing is controlled
        // by something else, e.g., a video frame grabber in
        // psycho-video.C In this case, let's use itsLastOvlDisplay to
        // check framerate:
        const uint64 tim = itsTimer.get();
        const float usec = float(tim - itsLastOvlDisplay);
        if (frame > 0)
          {
            const float toolong =
              itsRefreshDelay.getVal() * (1.0F + itsRefreshTolerance);
            const float tooshort =
              itsRefreshDelay.getVal() * (1.0F - itsRefreshTolerance);

            pushEvent(sformat("displayYUVoverlay - frame %d - %.3fms%s",
                              frame,
                              usec / 1000.0F,
                              usec > toolong ? " ***** SLOW FRAME? *****"
                              : usec < tooshort ? " ***** FAST FRAME? *****"
                              : ""));
          }
        else
          pushEvent(sformat("displayYUVoverlay - frame %d", frame));
        itsLastOvlDisplay = tim;
      }
      break;

    case NEXT_VSYNC:
      {
        // if this is our first frame of a movie, let's start fresh
        // with a sync to vertical blanking:
        if (frame == 0)
          waitNextRequestedVsync(false, false); // will update itsLastSync

        // display the overlay:
        if (SDL_DisplayYUVOverlay(itsOverlay, &r) == -1)
          SDLFATAL("Overlay blit failed");

        // wait until we enter the vertical blanking:
        waitNextRequestedVsync((frame >= 0), false);

        if (frame >= 0) pushEvent(sformat("displayYUVoverlay - frame %d",frame));
        else pushEvent("displayYUVoverlay");
      }
      break;

    case NEXT_FRAMETIME:
      {
        // wait until we reach the next frame time:
        waitNextRequestedFrameTime(frame, (frame >= 0), false);

        // display the overlay:
        if (SDL_DisplayYUVOverlay(itsOverlay, &r) == -1)
          SDLFATAL("Overlay blit failed");

        if (frame >= 0) pushEvent(sformat("displayYUVoverlay - frame %d",frame));
        else pushEvent("displayYUVoverlay");
      }
    }
}

// ######################################################################
void SDLdisplay::displayYUVoverlay(const int frame, const DelayType dly)
{
  if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");

  SDL_Rect r; r.x = 0; r.y = 0; r.w = getWidth(); r.h = getHeight();

  // sync and log an event:
  switch (dly)
    {
    case NO_WAIT:
      {
        // display the overlay:
        if (SDL_DisplayYUVOverlay(itsOverlay, &r) == -1)
          SDLFATAL("Overlay blit failed");

        // if we are not vsync'ing, probably our timing is controlled
        // by something else, e.g., a video frame grabber in
        // psycho-video.C In this case, let's use itsLastOvlDisplay to
        // check framerate:
        const uint64 tim = itsTimer.get();
        const float usec = float(tim - itsLastOvlDisplay);
        if (frame > 0)
          {
            const float toolong =
              itsRefreshDelay.getVal() * (1.0F + itsRefreshTolerance);
            const float tooshort =
              itsRefreshDelay.getVal() * (1.0F - itsRefreshTolerance);

            pushEvent(sformat("displayYUVoverlay - frame %d - %.3fms%s",
                              frame,
                              usec / 1000.0F,
                              usec > toolong ? " ***** SLOW FRAME? *****"
                              : usec < tooshort ? " ***** FAST FRAME? *****"
                              : ""));
          }
        else
          pushEvent(sformat("displayYUVoverlay - frame %d", frame));
        itsLastOvlDisplay = tim;
      }
      break;

    case NEXT_VSYNC:
      {
        // if this is our first frame of a movie, let's start fresh
        // with a sync to vertical blanking:
        if (frame == 0)
          waitNextRequestedVsync(false, false); // will update itsLastSync

        // display the overlay:
        if (SDL_DisplayYUVOverlay(itsOverlay, &r) == -1)
          SDLFATAL("Overlay blit failed");

        // wait until we enter the vertical blanking:
        waitNextRequestedVsync((frame >= 0), false);

        if (frame >= 0) pushEvent(sformat("displayYUVoverlay - frame %d",frame));
        else pushEvent("displayYUVoverlay");
      }
      break;

    case NEXT_FRAMETIME:
      {
        // wait until we reach the next frame time:
        waitNextRequestedFrameTime(frame, (frame >= 0), false);

        // display the overlay:
        if (SDL_DisplayYUVOverlay(itsOverlay, &r) == -1)
          SDLFATAL("Overlay blit failed");

        if (frame >= 0) pushEvent(sformat("displayYUVoverlay - frame %d",frame));
        else pushEvent("displayYUVoverlay");
      }
    }
}

// ######################################################################
bool SDLdisplay::supportsVideoOverlay(const VideoFormat vidformat) const
{
  Uint32 sdlformat;
  return tryvidformat2sdlmode(vidformat, &sdlformat);
}

// ######################################################################
void SDLdisplay::createVideoOverlay(const VideoFormat vidformat,
                                         const int w, const int h)
{
  const Uint32 sdlformat = vidformat2sdlmode(vidformat);

  this->createYUVoverlay(sdlformat, w, h);
}

// ######################################################################
void SDLdisplay::createVideoOverlay(const VideoFormat vidformat)
{
  const Uint32 sdlformat = vidformat2sdlmode(vidformat);

  this->createYUVoverlay(sdlformat);
}

// ######################################################################
void SDLdisplay::displayVideoOverlay(const VideoFrame& frame,
                                     const int framenum,
                                     const DelayType dly)
{
  if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");

  const Uint32 sdlformat = vidformat2sdlmode(frame.getMode());

  if (sdlformat != itsOverlay->format)
    LFATAL("video frame format does not match SDL overlay format");

  const int w = frame.getDims().w();
  const int h = frame.getDims().h();

  if (w != itsOverlay->w)
    LFATAL("video frame width does not match SDL overlay width");

  if (h != itsOverlay->h)
    LFATAL("video frame height does not match SDL overlay height");

  SDL_Overlay* ovl = this->lockYUVoverlay();
  switch (sdlformat)
    {
    case SDL_IYUV_OVERLAY:  // Planar mode: Y + U + V  (3 planes)
      if (ovl->planes != 3)       LFATAL("IYUV overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");

      memcpy(ovl->pixels[0], frame.getBuffer(), w * h);
      memcpy(ovl->pixels[1], frame.getBuffer() + w * h, ((w+1)/2) * ((h+1)/2));
      memcpy(ovl->pixels[2], frame.getBuffer() + w * h + ((w+1)/2) * ((h+1)/2),
             ((w+1)/2) * ((h+1)/2));
      break;

    case SDL_YV12_OVERLAY:  // Planar mode: Y + V + U  (3 planes)
      if (ovl->planes != 3)       LFATAL("YV12 overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h);
      // NOTE U+V are swapped here, since we're assuming the buffer is
      // in VIDFMT_YUV420P format:
      memcpy(ovl->pixels[2], frame.getBuffer() + w * h, ((w+1)/2) * ((h+1)/2));
      memcpy(ovl->pixels[1], frame.getBuffer() + w * h + ((w+1)/2) * ((h+1)/2),
             ((w+1)/2) * ((h+1)/2));
      break;

    case SDL_YUY2_OVERLAY:  // Packed mode: Y0+U0+Y1+V0 (1 plane)
      if (ovl->planes != 1)       LFATAL("YUY2 overlay must have 1 plane");
      if (ovl->pitches[0] != w*2) LFATAL("bogus pitches[0] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h * 2);
      break;

    case SDL_UYVY_OVERLAY:  // Packed mode: U0+Y0+V0+Y1 (1 plane)
      if (ovl->planes != 1)       LFATAL("UYVY overlay must have 1 plane");
      if (ovl->pitches[0] != w*2) LFATAL("bogus pitches[0] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h * 2);
      break;

    case SDL_YVYU_OVERLAY:  // Packed mode: Y0+V0+Y1+U0 (1 plane)
      if (ovl->planes != 1)       LFATAL("YVYU overlay must have 1 plane");
      if (ovl->pitches[0] != w*2) LFATAL("bogus pitches[0] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h * 2);
      break;

    default:
      LFATAL("Unknown SDL overlay format");
    }

  //if a valid rectangle, add a patch for external syncing with a photodiode
  addSyncRect(ovl, getDims(), itsSync, framenum, sdlformat);
  
  this->unlockYUVoverlay();
  this->displayYUVoverlay(framenum, dly);
}


// ######################################################################
void SDLdisplay::displayVideoOverlay_pos(const VideoFrame& frame,
                                         const int framenum,
                                         const DelayType dly,
                                         const int x, const int y,
                                         const int rw, const int rh)
{
  if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");

  const Uint32 sdlformat = vidformat2sdlmode(frame.getMode());

  if (sdlformat != itsOverlay->format)
    LFATAL("video frame format does not match SDL overlay format");

  const int w = frame.getDims().w();
  const int h = frame.getDims().h();

  if (w != itsOverlay->w)
    LFATAL("video frame width does not match SDL overlay width");

  if (h != itsOverlay->h)
    LFATAL("video frame height does not match SDL overlay height");

  SDL_Overlay* ovl = this->lockYUVoverlay();
  switch (sdlformat)
    {
    case SDL_IYUV_OVERLAY:  // Planar mode: Y + U + V  (3 planes)
      if (ovl->planes != 3)       LFATAL("IYUV overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");

      memcpy(ovl->pixels[0], frame.getBuffer(), w * h);
      memcpy(ovl->pixels[1], frame.getBuffer() + w * h, ((w+1)/2) * ((h+1)/2));
      memcpy(ovl->pixels[2], frame.getBuffer() + w * h + ((w+1)/2) * ((h+1)/2),
             ((w+1)/2) * ((h+1)/2));
      break;

    case SDL_YV12_OVERLAY:  // Planar mode: Y + V + U  (3 planes)
      if (ovl->planes != 3)       LFATAL("YV12 overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h);
      // NOTE U+V are swapped here, since we're assuming the buffer is
      // in VIDFMT_YUV420P format:
      memcpy(ovl->pixels[2], frame.getBuffer() + w * h, ((w+1)/2) * ((h+1)/2));
      memcpy(ovl->pixels[1], frame.getBuffer() + w * h + ((w+1)/2) * ((h+1)/2),
             ((w+1)/2) * ((h+1)/2));
      break;

    case SDL_YUY2_OVERLAY:  // Packed mode: Y0+U0+Y1+V0 (1 plane)
      if (ovl->planes != 1)       LFATAL("YUY2 overlay must have 1 plane");
      if (ovl->pitches[0] != w*2) LFATAL("bogus pitches[0] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h * 2);
      break;

    case SDL_UYVY_OVERLAY:  // Packed mode: U0+Y0+V0+Y1 (1 plane)
      if (ovl->planes != 1)       LFATAL("UYVY overlay must have 1 plane");
      if (ovl->pitches[0] != w*2) LFATAL("bogus pitches[0] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h * 2);
      break;

    case SDL_YVYU_OVERLAY:  // Packed mode: Y0+V0+Y1+U0 (1 plane)
      if (ovl->planes != 1)       LFATAL("YVYU overlay must have 1 plane");
      if (ovl->pitches[0] != w*2) LFATAL("bogus pitches[0] in SDL overlay");
      memcpy(ovl->pixels[0], frame.getBuffer(), w * h * 2);
      break;

    default:
      LFATAL("Unknown SDL overlay format");
    }

  //if a valid rectangle, add a patch for external syncing with a photodiode
  addSyncRect(ovl, getDims(), itsSync, framenum, sdlformat);

  this->unlockYUVoverlay();
  this->displayYUVoverlay(framenum, dly, x, y, rw, rh);
}

// ######################################################################
void SDLdisplay::displayVideoOverlay_image(const VideoFrame& frame,
                                           const int framenum,
                                           const DelayType dly,
                                           const Image<PixRGB<byte> >& img,
                                           const PixRGB<byte>& transpix,
                                           const uint threads)
{
  // NOTE: this code will probably not work with odd image sizes
  if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");

  const Uint32 sdlformat = vidformat2sdlmode(frame.getMode());

  if (sdlformat != itsOverlay->format)
    LFATAL("video frame format does not match SDL overlay format");

  const int w = frame.getDims().w();
  const int h = frame.getDims().h();

  if (w != itsOverlay->w)
    LFATAL("video frame width does not match SDL overlay width");

  if (h != itsOverlay->h)
    LFATAL("video frame height does not match SDL overlay height");

  if (w != img.getWidth())
    LFATAL("video frame width (%d) does not match the overlay image width (%d)",
           w, img.getWidth());

  if (h != img.getHeight())
    LFATAL("video frame height (%d) does not match overlay image height (%d)",
           h, img.getHeight());

  if (h % threads != 0)
    LFATAL("Image height should be a multiple of the number of requested"
           " threads");

  //get our image data
  const byte* src = reinterpret_cast<const byte*>(img.getArrayPtr());

  //lock the display
  SDL_Overlay* ovl = this->lockYUVoverlay();

  //get YUV bytes from the overlay, we will fill these
  byte* outy = NULL;
  byte* outu = NULL;
  byte* outv = NULL;

  switch (sdlformat)
    {
    case SDL_IYUV_OVERLAY:  // Planar mode: Y + U + V  (3 planes)
      if (ovl->planes != 3)       LFATAL("IYUV overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");

      outy = ovl->pixels[0];
      outu = ovl->pixels[1];
      outv = ovl->pixels[2];

      break;

    case SDL_YV12_OVERLAY:  // Planar mode: Y + V + U  (3 planes)
      if (ovl->planes != 3)       LFATAL("YV12 overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");

      outy = ovl->pixels[0];
      outu = ovl->pixels[2];
      outv = ovl->pixels[1];
      break;

    default:
      LFATAL("Unknown/Unsupported SDL format for overlay with image");
    }

  //get indexes into our data array.
  const byte* vidy = frame.getBuffer();
  const byte* vidu = frame.getBuffer() + w * h;
  const byte* vidv = frame.getBuffer() + w * h + ((w+1)/2) * ((h+1)/2);

  const byte tr = transpix.red();
  const byte tg = transpix.green();
  const byte tb = transpix.blue();

  //setup some threads
  pthread_t threadArray[threads];
  const uint rows = h / threads;
  uint py(0), puv(0), prgb(0);
  RgbYuvOverlayStruct *info[threads];

  for (uint i = 0; i < threads; ++i)
    {
      info[i] = new RgbYuvOverlayStruct(&vidy[py], &vidu[puv], &vidv[puv],
                                        &outy[py], &outu[puv], &outv[puv],
                                        &src[prgb], tr, tg, tb, w, rows);

      //create a pthread
      pthread_create(&threadArray[i], NULL, &RgbYuvOverlay, (void*)info[i]);

      //increment array positions
      py += w * rows;
      puv += ((w+1)/2) * ((rows+1)/2);
      prgb += w*rows*3;
    }

  //wait for all our threads to exit
  for (uint i = 0; i < threads; ++i)
    {
      pthread_join(threadArray[i],NULL);
      delete info[i];
    }

  //if a valid rectangle, add a patch for external syncing with a photodiode
  addSyncRect(ovl, getDims(), itsSync, framenum, sdlformat);

  this->unlockYUVoverlay();
  this->displayYUVoverlay(framenum, dly);
}

// ######################################################################
void SDLdisplay::displayVideoOverlay_patch(const VideoFrame& frame,
                                                const int framenum,
                                                const DelayType dly,
                                                const uint x,
                                                const uint y,
                                                const Image<PixRGB<byte> >& img)
{
 if (itsOverlay == NULL) LFATAL("You need to call createYUVoverlay() first");

  const Uint32 sdlformat = vidformat2sdlmode(frame.getMode());

  if (sdlformat != itsOverlay->format)
    LFATAL("video frame format does not match SDL overlay format");

  const int w = frame.getDims().w();
  const int h = frame.getDims().h();

  if (w != itsOverlay->w)
    LFATAL("video frame width does not match SDL overlay width");

  if (h != itsOverlay->h)
    LFATAL("video frame height does not match SDL overlay height");

  if (w < img.getWidth())
    LFATAL("video frame width (%d) is smaller than the patch width (%d)",
           w, img.getWidth());

  if (h < img.getHeight())
    LFATAL("video frame height (%d) is small than the patch height (%d)",
           h, img.getHeight());

  //lock the display
  SDL_Overlay* ovl = this->lockYUVoverlay();

  //get YUV bytes from the overlay, we will fill these in a bit
  byte* outy = NULL;
  byte* outu = NULL;
  byte* outv = NULL;

  //perform the overlay
  switch (sdlformat)
    {
    case SDL_IYUV_OVERLAY:  // Planar mode: Y + U + V  (3 planes)
      if (ovl->planes != 3)       LFATAL("IYUV overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");

      outy = ovl->pixels[0];
      outu = ovl->pixels[1];
      outv = ovl->pixels[2];

      memcpy(outy, frame.getBuffer(), w * h);
      memcpy(outu, frame.getBuffer() + w * h, ((w+1)/2) * ((h+1)/2));
      memcpy(outv, frame.getBuffer() + w * h + ((w+1)/2) * ((h+1)/2),
             ((w+1)/2) * ((h+1)/2));
      break;
    case SDL_YV12_OVERLAY:  // Planar mode: Y + V + U  (3 planes)
      if (ovl->planes != 3)       LFATAL("YV12 overlay must have 3 planes");
      if (ovl->pitches[0] != w)   LFATAL("bogus pitches[0] in SDL overlay");
      if (ovl->pitches[1] != w/2) LFATAL("bogus pitches[1] in SDL overlay");
      if (ovl->pitches[2] != w/2) LFATAL("bogus pitches[2] in SDL overlay");

      outy = ovl->pixels[0];
      outu = ovl->pixels[2];
      outv = ovl->pixels[1];

      memcpy(outy, frame.getBuffer(), w * h);
      // NOTE U+V are swapped here, since we're assuming the buffer is
      // in VIDFMT_YUV420P format:
      memcpy(outu, frame.getBuffer() + w * h, ((w+1)/2) * ((h+1)/2));
      memcpy(outv, frame.getBuffer() + w * h + ((w+1)/2) * ((h+1)/2),
             ((w+1)/2) * ((h+1)/2));
      break;
    default:
      LFATAL("Unknown/Unsupported SDL format for overlay with patch");
    }

  //copy our patch
  outy += x  + w*y;
  outu += x/2+1 + (w/2)*(y/2);
  outv += x/2+1 + (w/2)*(y/2);
  if (x % 2 == 0)
    {
      --outu;
      --outv;
    }

  int wp = img.getWidth();
  int yt = w - wp;
  if (wp % 2 != 0)
    yt-=1;
  const int ystep = yt;
  const int ustep = (ystep+1) / 2;
  const int vstep = (ystep+1) / 2;

  toVideoYUV422(img, outy, outu, outv, ystep,ustep,vstep);

  //if a valid rectangle, add a patch for external syncing with a photodiode
  addSyncRect(ovl, getDims(), itsSync, framenum, sdlformat);
  
  this->unlockYUVoverlay();
  this->displayYUVoverlay(framenum, dly);

}

// ######################################################################
void SDLdisplay::displayText(const std::string& msg, const bool vsync,
                             const PixRGB<byte> txtcol,
                             const PixRGB<byte> bgcol, int ind,
														 const int fontsize)
{
  // let's get a white image:
  Image< PixRGB<byte> > img(itsDims.getVal(), NO_INIT);
  img.clear(bgcol);

  // HACK: Image::writeText uses a font width of 10 pixels:
  Point2D<int> p; p.i = itsDims.getVal().w() / 2 - (msg.length() / 2 * 10);
  switch(ind) {
          case 0 : p.j = itsDims.getVal().h() / 2 - 10; break ;
          case 1 : p.j = 25 ; break ;
          case -1 : p.j = itsDims.getVal().h() - 25; break ;
          case -2 :
            p.j = (rand()%(itsDims.getVal().h()-40))+20;
            p.i = rand()%(itsDims.getVal().w() - (msg.length()-10)) + 5;
            break ;
          default :  p.j = itsDims.getVal().h() / 2 - 10; break ;
  }
  if (p.i < 0) LERROR("Text '%s' does not fit on screen!", msg.c_str());

  writeText(img, p, msg.c_str(), txtcol, bgcol, SimpleFont::FIXED(fontsize));

  // let's convert it to something we can blit:
  SDL_Surface *surf =
    SDL_CreateRGBSurfaceFrom(img.getArrayPtr(), img.getWidth(),
                             img.getHeight(), 24, 3 * img.getWidth(),
                             0x0000ff, 0x00ff00, 0xff0000, 0x0);
  SDL_Surface *surf2 = SDL_DisplayFormat(surf);
  SDL_FreeSurface(surf);
  displaySurface(surf2, -1, vsync);
  SDL_FreeSurface(surf2);
}
// ######################################################################
void SDLdisplay::displayText(const std::string& msg,Point2D<int> p ,
                             const PixRGB<byte> txtcol,
                             const PixRGB<byte> bgcol,const bool vsync)
{
  // let's get a white image:
  Image< PixRGB<byte> > img(itsDims.getVal(), NO_INIT);
  img.clear(bgcol);

  // HACK: Image::writeText uses a font width of 10 pixels:
  writeText(img, p, msg.c_str(), txtcol, bgcol);

  // let's convert it to something we can blit:
  SDL_Surface *surf =
    SDL_CreateRGBSurfaceFrom(img.getArrayPtr(), img.getWidth(),
                             img.getHeight(), 24, 3 * img.getWidth(),
                             0x0000ff, 0x00ff00, 0xff0000, 0x0);
  SDL_Surface *surf2 = SDL_DisplayFormat(surf);
  SDL_FreeSurface(surf);
  displaySurface(surf2, -1, vsync);
  SDL_FreeSurface(surf2);
}

// ######################################################################
void SDLdisplay::syncScreen(const bool vsync, const bool checkdelay,
                            const bool quiet)
{
  // sync to the vertical blanking pulse:
  if (vsync)
    waitNextRequestedVsync(checkdelay, quiet); // will update itsLastSync

  // swap the buffers:
  if (itsScreen->flags & SDL_DOUBLEBUF) SDL_Flip(itsScreen);
  else SDL_UpdateRect(itsScreen, 0, 0, 0, 0);
}
//#####################################################################
long SDLdisplay::getTimerValue()
{
  return itsTimer.get() ;
}

// ######################################################################
void SDLdisplay::waitNextRequestedVsync(const bool checkdelay,
                                           const bool quiet)
{
  if (quiet == false) pushEventBegin("waitVsync");
  uint64 prev = itsLastSync;

  // are we using hardware where vblank cannot be detected by polling
  // the VGA registers? If so, let's just do some hard waiting:
  if (itsVBlankKludge.getVal() > 0)
    {
      const float refresh2 = itsRefreshDelay.getVal() - float(itsVBlankKludge.getVal());
      while(float(itsTimer.get() - itsLastSync) < refresh2)   ;
    }
  else
    {
      // since we typically refresh at 120Hz but play frames at 30Hz, make
      // sure we catch the correct pulse for our given itsRefresh rate; to
      // this end, let's do a slow wait until 5ms before the pulse is
      // expected:
      const float refresh2 = itsRefreshDelay.getVal() - 5000.0F; // fast access to value
      while(float(itsTimer.get() - itsLastSync) < refresh2)   ;

      // we can't access the VGA registers unless we were able to do our
      // iopl(3) during init:
      if (itsPriority.getVal())
        {
#ifndef HAVE_SYS_IO_H
          LFATAL("this configuration lacks <sys/io.h>, so it does not support non-zero --sdl-priority");
#else
          // this code inspired from directfb-dev; see www.directfb.org.
          // Brutal poll of VGA registers; when the bit we poll for is set to
          // 1, we are in the vertical blanking interval (for more info on VGA
          // registers, see http://www.osdever.net/FreeVGA/vga/portidx.htm).
          // Apparently, when in SCHED_FIFO mode, short usleep() calls (10ms
          // or less) are implemented as busy waits; so here we don't even
          // bother calling usleep and do the busy-waiting ourselves:
          if (!(inb(0x3cc) & 1)) // we are in MDA (mono) emulation mode
            {
              // wait until end of blanking in case we are called so fast that
              // we still are in the previous blanking:
              while (!(inb(0x3ba) & 0x8)) ;
              // wait until start of blanking:
              while ((inb(0x3ba) & 0x8)) ;
              // we are in blanking NOW
            }
          else                   // we are in CGA (color) emulation mode
            {
              // wait until end of blanking in case we are called so fast that
              // we still are in the previous blanking:
              while (!(inb(0x3da) & 0x8)) ;
              // wait until start of blanking:
              while ((inb(0x3da) & 0x8)) ;
              // we are in blanking NOW
            }
#endif // HAVE_SYS_IO_H
        }
    }

  // update itsLastSync to current time:
  itsLastSync = itsTimer.get();
  if (quiet == false)
    {
      if (checkdelay)
        {
          const float usec = float(itsLastSync - prev);

          const float toolong = itsRefreshDelay.getVal() * (1.0F + itsRefreshTolerance);
          const float tooshort = itsRefreshDelay.getVal() * (1.0F - itsRefreshTolerance);

          pushEventEnd(sformat("waitVsync - %.3fms%s",
                               usec / 1000.0F,
                               usec > toolong ? " ***** SLOW FRAME? *****"
                               : usec < tooshort ? " ***** FAST FRAME? *****"
                               : ""));
        }
      else
        pushEventEnd("waitVsync");
    }
}

// ######################################################################
void SDLdisplay::waitNextRequestedFrameTime(const int frame,
                                            const bool checkdelay,
                                            const bool quiet)
{
  if (frame < 0)
    LFATAL("frame number must be non-negative");

  if (quiet == false) pushEventBegin("waitFrameTime");

  uint64 prev = itsLastSync;

  if (frame == 0)
    {
      itsTimer.reset();
      itsLastSync = 0;
      prev = 0;
    }
  else
    {
      ASSERT(itsRefreshDelay.getVal() > 0.0f);

      // check how many bits will be used when we multiply
      // itsRefreshDelay by the frame number; we want to make sure we
      // can hold that many bits in a double without rounding, since
      // otherwise we will lose precision:
      const int nbits =
        int(ceil(log(double(itsRefreshDelay.getVal())) / log(2))
            + ceil(log(double(frame)) / log(2)));

      // if double has 53 bits in the mantissa, and the frame numbers
      // run from 0-999999, using ~20 bits, then that leaves ~33 bits
      // for itsRefreshDelay, which should be plenty since typical
      // delays are in the 16000-33000us range, using ~15 bits for the
      // integral portion, and leaving ~18 bits for the fractional
      // portion:
      if (nbits > std::numeric_limits<double>::digits)
        LERROR("Oops! Loss of precision in our frame time calculations!");

      const uint64 target_time =
        uint64(double(itsRefreshDelay.getVal()) * frame);

      const uint64 min_time =
        prev + uint64(0.5 * itsRefreshDelay.getVal());

      const uint64 wait_until = std::max(target_time, min_time);

      // now do a slow wait until the frame delay is finished:
      while (itsTimer.get() < wait_until)
        { /* busy loop */ }

      itsLastSync = itsTimer.get();

      ASSERT(itsLastSync >= target_time);
    }

  if (quiet == false)
    {
      if (checkdelay)
        {
          const double hertz =
            itsLastSync == 0
            ? 0.0
            : ((double(frame) * 1000000.0) / itsLastSync);

          const float usec = float(itsLastSync - prev);

          const float toolong =
            itsRefreshDelay.getVal() * (1.0F + itsRefreshTolerance);
          const float tooshort =
            itsRefreshDelay.getVal() * (1.0F - itsRefreshTolerance);

          pushEventEnd(sformat("waitFrameTime - %.3fms - %.3fHz%s",
                               usec / 1000.0F, hertz,
                               usec > toolong ? " ***** SLOW FRAME? *****"
                               : usec < tooshort ? " ***** FAST FRAME? *****"
                               : ""));
        }
      else
        pushEventEnd("waitFrameTime");
    }
}

// ######################################################################
void SDLdisplay::waitFrames(const int n)
{
  for (int i = 0; i < n; ++i) waitNextRequestedVsync();
}

// ######################################################################
void SDLdisplay::setDesiredRefreshDelayUsec(float usec, float tol)
{
  itsRefreshDelay.setVal(usec);
  itsRefreshTolerance = tol;
}


// ######################################################################
// from SDL manual (http://sdldoc.csn.ul.ie/guidevideo.php#GUIDEVIDEOINTRO)
Uint32 SDLdisplay::getPixel32(const int x, const int y) const
{
  // Compute the address to the pixel we want to retrieve:
  int bpp = getBytesPerPixel();
  Uint8 *p = (Uint8 *)(itsScreen->pixels) + y * itsScreen->pitch + x * bpp;

  switch(bpp)
    {
    case 1:
      return *p;

    case 2:
      return *(Uint16 *)p;

    case 3:
      if (SDL_BYTEORDER == SDL_BIG_ENDIAN)
        return p[0] << 16 | p[1] << 8 | p[2];
      else
        return p[0] | p[1] << 8 | p[2] << 16;

    case 4:
      return *(Uint32 *)p;

    default:
      return 0;       // shouldn't happen, but avoids warnings
    }
}

// ######################################################################
// from SDL manual (http://sdldoc.csn.ul.ie/guidevideo.php#GUIDEVIDEOINTRO)
void SDLdisplay::putPixel32(const int x, const int y, const Uint32 pixel)
{
  // Compute the address to the pixel we want to set:
  int bpp = getBytesPerPixel();
  Uint8 *p = (Uint8 *)(itsScreen->pixels) + y * itsScreen->pitch + x * bpp;

  switch(bpp)
    {
    case 1:
      *p = pixel;
      break;

    case 2:
      *(Uint16 *)p = pixel;
      break;

    case 3:
      if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
        {
          p[0] = (pixel >> 16) & 0xff;
          p[1] = (pixel >> 8) & 0xff;
          p[2] = pixel & 0xff;
        }
      else
        {
          p[0] = pixel & 0xff;
          p[1] = (pixel >> 8) & 0xff;
          p[2] = (pixel >> 16) & 0xff;
        }
      break;

    case 4:
      *(Uint32 *)p = pixel;
      break;

    default:
      LFATAL("Cannot handle pixels with %dbpp", bpp);
    }
}

#endif // HAVE_SDL_SDL_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
