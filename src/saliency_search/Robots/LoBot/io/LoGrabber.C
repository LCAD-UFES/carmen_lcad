/**
   \file  Robots/LoBot/io/Grabber.C
   \brief The bastard step cousin to the IEEE1394grabber.
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoGrabber.C $
// $Id: LoGrabber.C 13037 2010-03-23 01:00:53Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case libdc1394 and other IEEE-1394 libraries are missing
#ifndef HAVE_IEEE1394

#include "Robots/LoBot/io/LoGrabber.H"

namespace lobot {

Grabber::Grabber(int, const Dims&, float)
{
   throw missing_libs(MISSING_LIBDC1394) ;
}

} // end of namespace encapsulating above empty definition

#else

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoGrabber.H"
#include "Robots/LoBot/io/LoFireWireBus.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoMath.H"

// INVT video conversion support
#include "Video/VideoFrame.H"

// Unix headers
#include <unistd.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------- FORWARD DECLARATIONS --------------------------

typedef std::pair<int, VideoFormat> GrabMode ;
static GrabMode grab_mode_enums(const Dims& resolution) ;

static int  frame_rate_enum(float frame_rate) ;
static Dims restrict_grab_size(const Dims& resolution) ;

static int  get_num_cameras() ;
static bool init_camera(int camera_number, int grab_mode, int frame_rate,
                        dc1394_cameracapture*) ;
static int  setup_capture(const raw1394handle_t&,
                          const nodeid_t& camera, int camera_number,
                          int mode, int frame_rate,
                          dc1394_cameracapture*) ;
static bool start_transmission(dc1394_cameracapture*) ;

static void release_camera(dc1394_cameracapture*, int camera_number) ;

//-------------------------- HELPER CLASSES -----------------------------

// Camera parameters "packet"
CameraParams::CameraParams(int bright, int exp, int sharp,
                           int ub, int vr, int hu, int sat,
                           int gam, int shut, int gn)
   : brightness(bright),
     exposure(exp),
     sharpness(sharp),
     white_balance_ub(ub), white_balance_vr(vr),
     hue(hu), saturation(sat),
     gamma(gam),
     shutter(shut),
     gain(gn)
{}

//---------------------- GRABBER INITIALIZATION -------------------------

// On initialization, sanify parameters passed in prior to
// remembering/setting the appropriate internal variables.
Grabber::Grabber(int camera_number,
                 const Dims& resolution, float frame_rate,
                 const CameraParams& params)
   : m_camera_number(clamp(camera_number, 0, get_num_cameras())),
     m_grab_size(restrict_grab_size(resolution)),
     m_grab_mode(grab_mode_enums(m_grab_size)),
     m_frame_rate(frame_rate_enum(frame_rate))
{
   if (! init_camera(m_camera_number,
                     m_grab_mode.first, m_frame_rate, & m_capture))
      throw camera_error(SETUP_FAILED) ;

   setParams(params) ;
   if (! start_transmission(& m_capture)) {
      release_camera(& m_capture, m_camera_number) ;
      throw camera_error(START_TRANSMISSION_FAILED) ;
   }
}

// Setup camera capture over DMA
static bool init_camera(int camera_number, int mode, int frame_rate,
                        dc1394_cameracapture* capture)
{
   const FireWireBus& bus = FireWireBus::instance() ;
   const raw1394handle_t& handle = bus.handle() ;
   const nodeid_t& camera = bus[camera_number] ;

   capture->num_dma_buffers = 8 ;
   capture->drop_frames = 1 ;
   capture->dma_device_file = NULL ;

   int setup = setup_capture(handle, camera, camera_number,
                             mode, frame_rate, capture) ;
   return setup == DC1394_SUCCESS ;
}

// Initiate camera capture mode
static bool start_transmission(dc1394_cameracapture* capture)
{
   const raw1394handle_t& handle = FireWireBus::instance().handle() ;
   int start = dc1394_start_iso_transmission(handle, capture->node) ;
   return start == DC1394_SUCCESS ;
}

// Set various camera parameters
void Grabber::setParams(const CameraParams& params)
{
   const FireWireBus& bus = FireWireBus::instance() ;
   const raw1394handle_t& handle = bus.handle() ;
   const nodeid_t& camera = bus[m_camera_number] ;

   dc1394_set_brightness(handle, camera, params.brightness) ;
   dc1394_set_exposure(handle, camera, params.exposure) ;
   dc1394_set_sharpness(handle, camera, params.sharpness) ;
   dc1394_set_white_balance(handle, camera,
                            params.white_balance_ub,
                            params.white_balance_vr) ;
   dc1394_set_hue(handle, camera, params.hue) ;
   dc1394_set_saturation(handle, camera, params.saturation) ;
   dc1394_set_gamma(handle, camera, params.gamma) ;
   dc1394_set_shutter(handle, camera, params.shutter) ;
   dc1394_set_gain(handle, camera, params.gain) ;
}

//-------------------------- GRABBING FRAMES ----------------------------

ImageType Grabber::grab() const
{
   while (dc1394_dma_single_capture(& m_capture) != DC1394_SUCCESS)
      usleep(1) ;

   const byte* data = m_capture.dma_ring_buffer +
                      m_capture.dma_frame_size * m_capture.dma_last_buffer ;
   VideoFrame V(data, m_capture.dma_frame_size,
                m_grab_size, m_grab_mode.second, false, false) ;
   ImageType I(V.toRgb()) ;

   dc1394_dma_done_with_buffer(& m_capture) ;
   return I ;
}

//--------------------------- GRABBER INFO ------------------------------

float Grabber::frameRate() const
{
   switch (m_frame_rate)
   {
      case FRAMERATE_60:
         return 60 ;
      case FRAMERATE_30:
         return 30 ;
      case FRAMERATE_15:
         return 15 ;
      case FRAMERATE_7_5:
         return 7.5f ;
      case FRAMERATE_3_75:
         return 3.75f ;
      case FRAMERATE_1_875:
         return 1.875f ;
      default:
         return -1 ;
   }
}

//------------------------- GRABBER CLEAN-UP ----------------------------

Grabber::~Grabber()
{
   release_camera(& m_capture, m_camera_number) ;
}

static void release_camera(dc1394_cameracapture* capture, int camera_number)
{
   const raw1394handle_t& handle = FireWireBus::instance().handle() ;

   dc1394_dma_unlisten(handle, capture) ;
   dc1394_stop_iso_transmission(handle, capture->node) ;
   dc1394_dma_release_camera(handle, capture) ;
}

//----------------------- MISCELLANEOUS HELPERS -------------------------

static int get_num_cameras()
{
   return FireWireBus::instance().num_cameras() ;
}

// This grabber will only grab frames sized 160x120 or 320x240 pixels
// (because it was originally written for Unibrain Fire-i cameras and
// they work at 30fps only at these resolutions).
static Dims restrict_grab_size(const Dims& resolution)
{
   if (resolution.w() > 240) // halfway between 160 and 320
      return Dims(320, 240) ;
   return Dims(160, 120) ;
}

// Return appropriate enums for libdc1394 and INVT based on the size of
// frames to be grabbed from cameras.
static GrabMode grab_mode_enums(const Dims& resolution)
{
   if (resolution.w() > 240) // halfway between 160 and 320
      return GrabMode(MODE_320x240_YUV422, VIDFMT_YUV422) ;
   return GrabMode(MODE_160x120_YUV444, VIDFMT_YUV444) ;
}

// The following function returns the libdc1394 enum corresponding to the
// specified frame rate.
static int frame_rate_enum(float r)
{
   if (r >= 45) // halfway between 30 and 60
      return FRAMERATE_60 ;   // r in [45, inf)
   if (r >= 22.5f)
      return FRAMERATE_30 ;   // r in [22.5, 45)
   if (r >= 11.25f)
      return FRAMERATE_15 ;   // r in [11.25, 22.5)
   if (r >= 5.625)
      return FRAMERATE_7_5 ;  // r in [5.625, 11.25)
   if (r >= 2.8125f)
      return FRAMERATE_3_75 ; // r in [2.8125, 5.625)
   return FRAMERATE_1_875 ;   // r in (-inf, 2.8125)
}

// Convenience routine to perform DMA capture setup. libdc1394's API
// comes in two flavours depending on the version of the library.
// Encapsulating this API in a separate function allows us to pass the
// appropriate parameters based on the version of libdc1394 without
// messing up the call site. All of this ugly #ifdef-ing can be localized
// to this portion of the code while the caller remains blissfully
// unaware of the low-level plumbing required to make it work.
#ifdef IEEE1394NEW

// Extra param in new version of DMA capture setup API
static int setup_capture(const raw1394handle_t& handle,
                         const nodeid_t& camera, int camera_number,
                         int mode, int frame_rate,
                         dc1394_cameracapture* capture)
{
   return dc1394_dma_setup_capture(handle, camera, camera_number,
                                   FORMAT_VGA_NONCOMPRESSED, mode,
                                   SPEED_400, frame_rate,
                                   capture->num_dma_buffers,
                                   0, // new param: do_extra_buffering
                                   capture->drop_frames,
                                   capture->dma_device_file, capture) ;
}

#else // use old version of DMA capture setup API

static int setup_capture(const raw1394handle_t& handle,
                         const nodeid_t& camera, int camera_number,
                         int mode, int frame_rate,
                         dc1394_cameracapture* capture)
{
   return dc1394_dma_setup_capture(handle, camera, camera_number,
                                   FORMAT_VGA_NONCOMPRESSED, mode,
                                   SPEED_400, frame_rate,
                                   capture->num_dma_buffers,
                                   capture->drop_frames,
                                   capture->dma_device_file, capture) ;
}

#endif // #ifdef IEEE1394NEW

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef HAVE_IEEE1394

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
