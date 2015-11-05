/*!@file Media/HttpEncoder.H Low-level class for using http to output display */

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
// Primary maintainer for this file:Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/HttpEncoder.H $
// $Id: HttpEncoder.H 13505 2010-05-30 16:08:43Z lior $
//

#ifndef MEDIA_HttpEncoder_H_DEFINED
#define MEDIA_HttpEncoder_H_DEFINED

#ifdef INVT_HAVE_AVCODEC

#include "Image/Range.H"
#include "Devices/HTTPServer.H"
#include "Transport/LowLevelEncoder.H"
#include "Util/Types.H" // for byte

#include <cstdio>
#include <pthread.h>

#define attribute_deprecated
extern "C"
{
#ifdef HAVE_LIBAVCODEC_AVCODEC_H
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#else
#include <ffmpeg/avcodec.h>
#include <ffmpeg/avformat.h>
#endif

}
#include <string>
#include <Image/Image.H>
#include <Image/Dims.H>

//class Dims;
class VideoFrame;
template <class T> class Image;
template <class T> class PixRGB;

/// Low-level class for using ffmpeg to decode movie files
/** You probably want to use MpegOutputStream instead, which relies on
	HttpEncoder internally for its implementation but offers a
	richer, higher-level interface. */
class HttpEncoder : public LowLevelEncoder
{
	public:
		HttpEncoder(nub::soft_ref<HttpServer> httpServer,
				const std::string& fname,
				const std::string& codecname,
				const int bitrate,
				const int framerate,
				const int frameratebase,
				const Dims& dims,
				const int bufsz);

		virtual ~HttpEncoder();

		virtual int close();

		void writeRawFrame(const AVFrame* picture);

		void writeRGB(const Image<PixRGB<byte> >& img);

		void writeVideoFrame(const VideoFrame& frame);

		virtual void writeFrame(const GenericFrame& f);

		void sendEncodedFrames(const int clientFd);
		void sendMJPGFrames(const int clientFd);

		void run();

	private:
		void initEncoder();
		void openBuffer();
		void closeFrameBuffer();


		nub::soft_ref<HttpServer>                  itsHttpServer;
		FILE*           itsFile;
		AVCodecContext  itsContext;
		AVFormatContext* itsFormatContext;
		AVStream*                                itsAVStream;
		int             itsFrameNumber;
		const int       itsOutbufSize;
		Range<int>      itsFrameSizeRange;

		std::string itsFname;
		std::string itsCodecname;
		int itsBitrate;
		int itsFramerate;
		int itsFrameratebase;
		Dims itsDims;

		pthread_t                         itsRunThread;
		pthread_mutex_t itsImgFrameLock;
		pthread_mutex_t itsFrameLock;
		pthread_mutex_t itsFrameNumLock;
		unsigned char*  itsEncodedFrameBuffer;
		int             itsEncodedFrameBufferSize;
		unsigned char*  itsEncodedHeader;
		int             itsEncodedHeaderSize;
		Image<PixRGB<byte> > itsCurrentFrame;
		bool itsThreadRunning;
};

#endif // HAVE_FFMPEG_AVCODEC_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_HttpEncoder_H_DEFINED
