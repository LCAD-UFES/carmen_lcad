/**
   \file Robots/LoBot/LoVideoRecorder.H

   \brief Video recorder encapsulation for the Lobot/Robolocust project.

   This file defines the lobot::VideoRecorder class, which is used to
   record input video streams to MPEG movies. This class uses the INVT
   ffmpeg encoder to do its thing.

   The idea here is to be able to drive our robot around and have it
   record movies using its camera inputs. We can then send these movies
   to the Gabbiani team at the Baylor College of Medicine (Houston, TX)
   and have them show these movies to actual locusts and send us back the
   LGMD spike train recordings, which we can use to better test our
   integration algorithms.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoVideoRecorder.H $
// $Id: LoVideoRecorder.H 12785 2010-02-06 02:24:05Z irock $
//

#ifndef LOBOT_VIDEO_RECORDER_DOT_H
#define LOBOT_VIDEO_RECORDER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoVideoStream.H"

// INVT MPEG encoding support
//
// DEVNOTE: Unfortunately, INVT's FfmpegEncoder class doesn't have a
// "default" definition in case the FFmpeg libraries are missing. We
// work around that here by supplying a fake FfmpegEncoder class so that
// lobot builds succeed.
#ifdef INVT_HAVE_AVCODEC // FFmpeg libs available

#include "Media/FfmpegEncoder.H" // use INVT's MPEG encoding support

#else // FFmpeg libs not available

struct FfmpegEncoder {} ; // fake MPEG encoder to allow builds

#endif // #ifdef INV_HAVE_AVCODEC

// Standard C++ headers
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::VideoRecorder
   \brief Video stream recorder for Lobot/Robolocust project.

   This class wraps around INVT's FfmpegEncoder in order to send input
   video streams to an MPEG file. This is useful for exchanging movies of
   our robot moving with the Gabbiani team at Baylor so as to get back
   LGMD spike trains from them.
*/
class VideoRecorder {
   // The recorder gets its input from a video stream (which can be
   // reading either from a IEEE-1394 camera or an MPEG file) and sends
   // the incoming frames to an MPEG file using the INVT FfmpegEncoder
   // class.
   const VideoStream* m_source ;
   FfmpegEncoder      m_sink ;

   // Prevent copy and assignment
   VideoRecorder(const VideoRecorder&) ;
   VideoRecorder& operator=(const VideoRecorder&) ;

public:
   /// This constructor sets up a recorder to read input images from the
   /// provided video stream and store them in an MPEG file with the
   /// given "root" name. The file name extension is appended
   /// automatically (by FfmpegEncoder).
   VideoRecorder(const std::string& mpeg_name, const VideoStream*) ;

   /// This method reads the current frame from its video stream source
   /// and writes it to the MPEG file the recorder is recording to.
   void update() ;

   /// Clean-up
   ~VideoRecorder() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
