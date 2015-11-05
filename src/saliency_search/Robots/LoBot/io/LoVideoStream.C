/**
   \file  Robots/LoBot/io/LoVideoStream.C
   \brief Video stream encapsulation for the Lobot/Robolocust project.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoVideoStream.C $
// $Id: LoVideoStream.C 12592 2010-01-18 23:17:34Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoVideoStream.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoExcept.H"

// INVT headers
#include "Raster/GenericFrame.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// Read images directly from FireWire camera
VideoStream::
VideoStream(int camera, const Dims& resolution, float frame_rate)
   : m_grabber(new Grabber(camera, resolution, frame_rate)),
     m_decoder(0)
{}

// Read images from an MPEG
VideoStream::VideoStream(const std::string& mpeg_file_name)
   : m_grabber(0),
     m_decoder(new FfmpegDecoder(video_conf("playback_codec",
                                            std::string("Auto")).c_str(),
                                 video_conf("buffer_size", 100000),
                                 mpeg_file_name.c_str(),
                                 false))
{}

//----------------------------- VIDEO I/O -------------------------------

void VideoStream::update()
{
   if (m_grabber)
      m_image = m_grabber->grab() ;
   else if (m_decoder)
      m_image = m_decoder->readRGB() ;
   else
      throw vstream_error(NO_VIDEOSTREAM_SOURCE) ;
}

//------------------------- VIDEO STREAM INFO ---------------------------

Dims VideoStream::frameSize() const
{
   if (m_grabber)
      return m_grabber->frameSize() ;
   if (m_decoder)
      return m_decoder->peekFrameSpec().dims ;
   throw vstream_error(NO_VIDEOSTREAM_SOURCE) ;
}

float VideoStream::frameRate() const
{
   if (m_grabber)
      return m_grabber->frameRate() ;
   if (m_decoder)
      return m_decoder->peekFrameSpec().frameRate ;
   throw vstream_error(NO_VIDEOSTREAM_SOURCE) ;
}

//----------------------------- CLEAN-UP --------------------------------

VideoStream::~VideoStream()
{
   delete m_grabber ;
   delete m_decoder ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
