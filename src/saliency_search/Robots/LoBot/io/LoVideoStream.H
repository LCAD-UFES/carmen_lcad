/**
   \file Robots/LoBot/LoVideoStream.H

   \brief Video stream encapsulation for the Lobot/Robolocust project.

   This file defines the lobot::VideoStream class, which is used to wrap
   around the classes responsible for direct FireWire camera capture and
   MPEG reading. To its client, VideoStream then presents a uniform
   interface that simply provides images so that its clients don't have
   to know or care where those images came from or how they were
   obtained.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoVideoStream.H $
// $Id: LoVideoStream.H 12785 2010-02-06 02:24:05Z irock $
//

#ifndef LOBOT_VIDEO_STREAM_DOT_H
#define LOBOT_VIDEO_STREAM_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoGrabber.H"

#include "Robots/LoBot/config/LoDefaults.H"

#include "Robots/LoBot/misc/LoTypes.H"

// INVT MPEG decoding support
//
// DEVNOTE: Unfortunately, INVT's FfmpegDecoder class doesn't have a
// "default" definition in case the FFmpeg libraries are missing. We
// work around that here by supplying a fake FfmpegDecoder class so that
// lobot builds succeed.
#ifdef INV_HAVE_AVCODEC // FFmpeg libs available

#include "Media/FfmpegDecoder.H"  // use INVT's MPEG decoding support

#else // FFmpeg libs not available

// Fake MPEG decoder to allow builds but fail at run-time...
#include "Robots/LoBot/misc/LoExcept.H"
#include "Raster/GenericFrame.H"

struct FfmpegDecoder {
   FfmpegDecoder(const char*, int, const char*, bool){
      throw lobot::missing_libs(lobot::MISSING_FFMPEG) ;
   }
   lobot::ImageType readRGB() const {return lobot::ImageType() ;}
   GenericFrameSpec peekFrameSpec() const {return GenericFrameSpec() ;}
} ;

#endif

// INVT image support
#include "Image/Dims.H"

// Standard C++ headers
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::VideoStream
   \brief Video stream wrapper for Lobot/Robolocust project.

   This class wraps around lobot::Grabber and INVT's InputMPEGStream so
   that its clients can simply retrieve images either directly from
   multiple FireWire cameras or multiple MPEG files without having to be
   concerned about where or how those images were obtained.

   Ideally, we would not need this class. Unfortunately, we need this
   hack since lobot::Grabber is not implemented within the INVT
   ModelComponent hierarchy to act as an InputFrameSeries.
*/
class VideoStream {
   // The video stream may be coming either directly from FireWire
   // cameras or from MPEG files.
   Grabber* m_grabber ;
   FfmpegDecoder* m_decoder ;

   // The video stream object caches input frames from its source camera
   // or MPEG file during its update cycle and then supplies the cached
   // image to all its clients when they request the next frame.
   ImageType m_image ;

   // Prevent copy and assignment
   VideoStream(const VideoStream&) ;
   VideoStream& operator=(const VideoStream&) ;

public:
   /// This constructor sets up a grabber to read input images directly
   /// from a FireWire camera. It expects to be passed the FireWire
   /// sub-channel ID (i.e., camera number) of the camera it is to be
   /// bound to.
   VideoStream(int camera_number,
               const Dims& resolution = LOBOT_DEFAULT_GRAB_SIZE,
               float frame_rate = LOBOT_DEFAULT_GRAB_RATE) ;

   /// This constructor sets up an input MPEG stream to read input images
   /// from an MPEG file. It should be passed the name of the MPEG file
   /// name it is to read images from.
   VideoStream(const std::string& mpeg_file_name) ;

   /// This method reads the next frame from the input video source
   /// (either camera or MPEG file) and caches it internally. This method
   /// is meant to be called from the main loop of the
   /// application/program rather than directly by clients. Clients
   /// should, instead, use the readFrame() method.
   void update() ;

   /// This method returns the size of the frames being read from the
   /// input video source.
   Dims frameSize() const ;

   /// This method returns the rate at which frames are read from the
   /// input video source. The returned number's units are frames per
   /// second.
   float frameRate() const ;

   /// As far as VideoStream clients are concerned, this method reads the
   /// next frame from the input video source (i.e., either a firewire
   /// camera or MPEG file) and returns it as an Image<PixRGB<byte>>. In
   /// actuality, this method returns the frame read and cached by an
   /// earlier call to the update() method. Clients of VideoStream should
   /// not use update() directly; that is designed to be called from an
   /// application's main loop (or similar structure/place). Instead,
   /// clients should call this method to retrieve frames from the input
   /// video sources.
   ImageType readFrame() const {return m_image ;}

   // Clean-up
   ~VideoStream() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
