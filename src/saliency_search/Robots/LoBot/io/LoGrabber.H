/**
   \file  Robots/LoBot/io/LoGrabber.H
   \brief The bastard step cousin to the IEEE1394grabber.

   This file defines the lobot::LoGrabber class, which uses the libdc1394
   API directly to perform multi-grabs from multiple FireWire cameras. We
   could do this using the IEEE1394grabber class; unfortunately, that
   class has some weird bug that causes multi-grab operations to crash.

   Specifically, when we have multiple FireWire cameras daisy-chained
   together, their sub-channel IDs will be 0, 1, 2, etc. Creating the
   requisite number of instances of IEEE1394grabber and then setting the
   "FrameGrabberSubChan" option/parameter for each of these instance to
   the appropriate sub-channel ID should work. However, somewhere along
   the line, all the instances end up using the same sub-channel ID
   (e.g., 1 or 2 or 0 or whatever). Then, the libdc1394 library complains
   when trying to initialize the same sub-channel more than once.

   Ideally, we ought to fix IEEE1394grabber so that it works correctly.
   In the mean time, the lobot::LoGrabber class provides a quick
   workaround that allows multi-grabs to function. It implements pretty
   much the same functionality as IEEE1394grabber in almost the same
   manner, except that it isn't part of the ModelComponent hierarchy. Its
   interface is also more direct and vastly simplified. Thus its label as
   "the bastard step cousin to IEEE1394grabber."
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoGrabber.H $
// $Id: LoGrabber.H 11256 2009-05-30 01:46:10Z mviswana $
//

#ifndef LOBOT_GRABBER_DOT_H
#define LOBOT_GRABBER_DOT_H

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case libdc1394 and other IEEE-1394 libraries are missing
#ifndef HAVE_IEEE1394

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoTypes.H"
#include "Image/Dims.H"

namespace lobot {

struct Grabber {
   Grabber(int, const Dims&, float) ;
   ImageType  grab() const {return ImageType() ;}
   Dims  frameSize() const {return Dims(0,0) ;}
   float frameRate() const {return 0 ;}
} ;

} // end of namespace encapsulating above empty definition

#else

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/config/LoDefaults.H"

#include "Robots/LoBot/misc/LoTypes.H"

// INVT video conversion support
#include "Video/VideoFormat.H"

// INVT image support
#include "Image/Dims.H"

// 1394 headers
#include <libdc1394/dc1394_control.h>

// Standard C++ headers
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- HELPER CLASSES -----------------------------

// Defaults for various camera parameters
#ifndef LOCP_DEFAULT_BRIGHTNESS
   #define LOCP_DEFAULT_BRIGHTNESS 32768
#endif
#ifndef LOCP_DEFAULT_EXPOSURE
   #define LOCP_DEFAULT_EXPOSURE 511
#endif
#ifndef LOCP_DEFAULT_SHARPNESS
   #define LOCP_DEFAULT_SHARPNESS 80
#endif
#ifndef LOCP_DEFAULT_WHITE_BALANCE_UB
   #define LOCP_DEFAULT_WHITE_BALANCE_UB 95
#endif
#ifndef LOCP_DEFAULT_WHITE_BALANCE_VR
   #define LOCP_DEFAULT_WHITE_BALANCE_VR 87
#endif
#ifndef LOCP_DEFAULT_HUE
   #define LOCP_DEFAULT_HUE 32768
#endif
#ifndef LOCP_DEFAULT_SATURATION
   #define LOCP_DEFAULT_SATURATION 90
#endif
#ifndef LOCP_DEFAULT_GAMMA
   #define LOCP_DEFAULT_GAMMA 1
#endif
#ifndef LOCP_DEFAULT_SHUTTER
   #define LOCP_DEFAULT_SHUTTER 6
#endif
#ifndef LOCP_DEFAULT_GAIN
   #define LOCP_DEFAULT_GAIN 87
#endif

/// Camera parameters "packet"
struct CameraParams {
   int brightness, exposure, sharpness ;
   int white_balance_ub, white_balance_vr ;
   int hue, saturation ;
   int gamma, shutter, gain ;

   CameraParams(int bright = LOCP_DEFAULT_BRIGHTNESS,
                int exp    = LOCP_DEFAULT_EXPOSURE,
                int sharp  = LOCP_DEFAULT_SHARPNESS,
                int ub     = LOCP_DEFAULT_WHITE_BALANCE_UB,
                int vr     = LOCP_DEFAULT_WHITE_BALANCE_VR,
                int hu     = LOCP_DEFAULT_HUE,
                int sat    = LOCP_DEFAULT_SATURATION,
                int gam    = LOCP_DEFAULT_GAMMA,
                int shut   = LOCP_DEFAULT_SHUTTER,
                int gn     = LOCP_DEFAULT_GAIN) ;
} ;

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Grabber
   \brief Allow multi-grab from multiple FireWire cameras.

   This class is a bastard step cousin to IEEE1394grabber. Its raison
   d'etre is to allow grabbing frames simultaneously from multiple
   FireWire cameras.

   To do this, we would usually just instantiate IEEE1394grabber the
   required number of times. For example, if we have three cameras, we
   would create three IEEE1394grabber instances and reconfigure them to
   use separate sub-channels. That is, one of the instance would use
   sub-channel 0 (first camera on the FireWire bus), the next one would
   use sub-channel 1 (second camera), and the third instance would use
   sub-channel 2. Unfortunately, at this time (circa October 2008), due
   to some strange bug, all the IEEE1394grabber instances end up using
   the same sub-channel (say, 1) even when the "FrameGrabberSubChan"
   option is explicitly reset. This causes the libdc1394 library to
   complain when each IEEE1394grabber tries to (re)initialize the same
   camera (i.e., sub-channel).

   Obviously, the best thing to do is fix IEEE1394grabber, thus, making
   this class obsolete. But that requires a fair amount of familiarity
   with INVT internals (something the author of this class, circa October
   2008, does not yet possess). This class, therefore, serves as an
   interim solution.

   It should be noted that whereas IEEE1394grabber is part of the
   ModelComponent hierarchy, lobot::Grabber is not. Internally, it is
   implemented almost identically to IEEE1394grabber. However, much of
   IEEE1394grabber's pizazz, generality and configurability are missing
   because this class is designed specifically for the Lobot/Robolocust
   project. Its interface is quite direct and simple.
*/
class Grabber {
   // Each grabber needs to remember the sub-channel ID of the camera it
   // is bound to (i.e., the camera number of the camera it is supposed
   // to grab images from).
   int m_camera_number ;

   // For successful conversion between the image format returned by the
   // camera and the desired INVT Image type, we need to remember the
   // grab resolution.
   Dims m_grab_size ;

   // Usually, cameras pack the image data they return based on the grab
   // resolution. For example, the Unibrain Fire-i returns frames in
   // YUV444 format when grabbing at 160x120 and YUV422 at 320x240. The
   // INVT framework provides handy functions/classes to convert such
   // "raw" frames to RGB images. However, the enums used by the
   // libdc1394 library and INVT are different. To keep things in sync,
   // we have to remember the appropriate enums for each.
   //
   // We do that with an STL pair whose first member holds the enum for
   // libdc1394 and whose second member stores the enum for INVT.
   std::pair<int, VideoFormat> m_grab_mode ;

   // We also have to remember the libdc1394 enum for the grab frame
   // rate.
   int m_frame_rate ;

   // Each grabber has its own capture buffer
   mutable dc1394_cameracapture m_capture ;

public :
   /// The constructor expects the sub-channel ID of the camera to grab
   /// from (i.e., the camera number). It can also take the size of the
   /// frame to be grabbed (defaults to 320x240), the frame rate
   /// (defaults to 30fps), and various camera parameters such as
   /// brightness, gamma, etc. (defaults should be okay).
   Grabber(int camera_number,
           const Dims& resolution = LOBOT_DEFAULT_GRAB_SIZE,
           float frame_rate = LOBOT_DEFAULT_GRAB_RATE,
           const CameraParams& params = CameraParams()) ;

   /// Set various camera parameters
   void setParams(const CameraParams&) ;

   /// After instantitation, clients may use this method to retrieve
   /// frames from the camera this grabber is bound to.
   ImageType grab() const ;

   /// Return the size of the frames being grabbed
   Dims frameSize() const {return m_grab_size ;}

   /// Return the rate at which frames are being grabbed from the source
   /// camera.
   float frameRate() const ;

   /// Clean-up
   ~Grabber() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef HAVE_IEEE1394
#endif // #ifndef LOBOT_GRABBER_DOT_H

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
