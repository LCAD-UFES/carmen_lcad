/**
   \file  Robots/LoBot/io/LoVideoRecorder.C
   \brief Video recorder encapsulation for the Lobot/Robolocust project.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoVideoRecorder.C $
// $Id: LoVideoRecorder.C 12785 2010-02-06 02:24:05Z irock $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case FFmpeg libraries are missing
#ifndef INVT_HAVE_AVCODEC

#include "Robots/LoBot/io/LoVideoRecorder.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

VideoRecorder::VideoRecorder(const std::string&, const VideoStream*)
   : m_source(0), m_sink()
{
   throw missing_libs(MISSING_FFMPEG) ;
}

void VideoRecorder::update(){}
VideoRecorder::~VideoRecorder(){}

} // end of namespace encapsulating above empty definition

#else

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoVideoRecorder.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// Since the recorder mostly wraps around INVT's FfmpegEncoder,
// instantiation mostly involves setting up the ffmpeg encoder. Many of
// the encoder's parameters are read from the config file. Hopefully
// though, the hard-coded defaults specified here are good enough in case
// the user has not setup the config file...
VideoRecorder::
VideoRecorder(const std::string& mpeg_name, const VideoStream* S)
   : m_source(S),
     m_sink(mpeg_name,
            video_conf("recording_codec", std::string("mpeg")),
            video_conf("bit_rate", 400000),
            video_conf("frame_rate", 25),
            video_conf("frame_rate_base", 1),
            S->frameSize(),
            video_conf("buffer_size", 100000))
{}

//-------------------------- MPEG RECORDING -----------------------------

void VideoRecorder::update()
{
   m_sink.writeRGB(m_source->readFrame()) ;
}

//----------------------------- CLEAN-UP --------------------------------

VideoRecorder::~VideoRecorder(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef INV_HAVE_AVCODEC

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
