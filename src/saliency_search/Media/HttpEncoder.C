/*!@file Media/HttpEncoder.C Low-level class for using http to output display */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/HttpEncoder.C $
// $Id: HttpEncoder.C 15310 2012-06-01 02:29:24Z itti $
//

#ifdef INVT_HAVE_AVCODEC

#include "Media/HttpEncoder.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/JPEGUtil.H"
#include "Image/color_conversions.H" // for rgb24_to_yv12_c()
#include "Raster/GenericFrame.H"
#include "Util/log.H"
#include "Video/FfmpegFrame.H"
#include "Video/VideoFrame.H"
#include "rutz/arrays.h"
#include "rutz/trace.h"

void * HttpEncoder_run(void* r0)
{
	HttpEncoder* r = (HttpEncoder*)r0;
	r->run();
	return NULL;
}

// ######################################################################
HttpEncoder::HttpEncoder(nub::soft_ref<HttpServer> httpServer,
		const std::string& fname,
		const std::string& codecname,
		const int bitrate,
		const int framerate,
		const int frameratebase,
		const Dims& dims,
		const int bufsz)
:
	itsHttpServer(httpServer),
	itsFile(0),
	itsContext(),
	itsFormatContext(0),
	itsFrameNumber(0),
	itsOutbufSize(bufsz),
	itsFrameSizeRange(),
	itsFname(fname),
	itsCodecname(codecname),
	itsBitrate(bitrate),
	itsFramerate(framerate),
	itsFrameratebase(frameratebase),
	itsDims(dims),
	itsThreadRunning(true)
{
	pthread_mutex_init(&itsFrameLock, NULL);
	pthread_mutex_init(&itsImgFrameLock, NULL);
	pthread_mutex_init(&itsFrameNumLock, NULL);
	initEncoder();
	//Start the thread to listen to connections
	pthread_create(&itsRunThread, NULL, &HttpEncoder_run, (void *)this);
}

void HttpEncoder::initEncoder()
{
	GVX_TRACE(__PRETTY_FUNCTION__);

	// no need to guard these functions for being called multiple times;
	// they all have internal guards
	av_register_all();
	avcodec_init();
	avcodec_register_all();

	AVOutputFormat* oformat = NULL;
#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21

	if (itsCodecname.compare("List") == 0) { // list available codecs
      LINFO("##### Available output codecs (not all may work for video):");
      AVOutputFormat* f = av_oformat_next(NULL);
      while(f) {
        LINFO("%s: %s %d", f->name, f->long_name, f->flags);
        f = av_oformat_next(f);
      }
      LFATAL("Please select a codec from this list");
	} else { // format is given
      // no av_find_output_format()?? let's do it by hand...
      AVOutputFormat* f = av_oformat_next(NULL);
      while(f) {
        if (itsCodecname.compare(f->name) == 0) { oformat = f; break; }
        f = av_oformat_next(f);
      }
	}

#else
	if (itsCodecname.compare("List") == 0) { // list available codecs
      LINFO("##### Available output codecs (not all may work for video):");
      for (AVOutputFormat* f = first_oformat; f != NULL; f = f->next)
        LINFO("%s: %s %d", f->name, f->long_name, f->flags);
      LFATAL("Please select a codec from this list");
	} else { // format is given
      // no av_find_output_format()?? let's do it by hand...
      for (AVOutputFormat* f = first_oformat; f != NULL; f = f->next)
        if (itsCodecname.compare(f->name) == 0) { oformat = f; break; }
	}
#endif

	if (oformat == 0)
		LFATAL("No such video codec '%s';\n"
				"try re-running with --output-codec=List to see a list\n"
				"of available codecs", itsCodecname.c_str());

	char ext[100]; ext[0] = '.'; uint i;
	for (i = 0; i < strlen(oformat->extensions); i ++)
		if (oformat->extensions[i] == ',') break;
		else ext[i+1] = oformat->extensions[i];
	ext[i+1] = '\0';
	LINFO("Using output format '%s' (%s), extension %s", oformat->name,
			oformat->long_name, ext);

#ifdef INVT_FFMPEG_HAS_FORMATCONTEXT_FUNCTIONS
	itsFormatContext = av_alloc_format_context();
	if (!itsFormatContext)
		LFATAL("Cannot allocate format context");
	itsFormatContext->oformat = oformat;

	itsAVStream = av_new_stream(itsFormatContext, 0);
	if (!itsAVStream)
		LFATAL("Can not allocate AVStream");
#else
	LFATAL("Need a new version of ffmpeg libs for this option");
	itsFormatContext = NULL;
#endif

	AVCodec* const codec = avcodec_find_encoder(oformat->video_codec);
	if (codec == NULL)  LFATAL("codec not found");

#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
	avcodec_get_context_defaults(&itsContext);
#else
	{
		AVCodecContext* const tmp = avcodec_alloc_context();
		memcpy(&itsContext, tmp, sizeof(AVCodecContext));
		free(tmp);
	}
#endif

	itsContext.bit_rate = itsBitrate;

	// Be sure to set itsContext.pix_fmt -- it may occasionally
	// appear to work to leave pix_fmt unset, because the value we want,
	// PIX_FMT_YUV420P, has the enum value of 0, so if the uninitialized
	// memory for pix_fmt happens to have the value 0, then we'll slip
	// through without setting it explicitly.
	itsContext.pix_fmt = PIX_FMT_YUV420P;

	/* resolution must be a multiple of two */
	itsContext.width = itsDims.w();
	itsContext.height = itsDims.h();
#if defined(INVT_FFMPEG_AVCODECCONTEXT_HAS_TIME_BASE)
	AVRational time_base = { itsFrameratebase, itsFramerate };
	itsContext.time_base = time_base;
	const int frb = itsFrameratebase;
#elif LIBAVCODEC_VERSION_INT >= 0x000406 && LIBAVCODEC_BUILD > 4665
	itsContext.frame_rate = itsFramerate;
	const int frb = itsFrameratebase;
	itsContext.frame_rate_base = frb;
#else
	itsContext.frame_rate = itsFramerate;
	const int frb = FRAME_RATE_BASE;
#endif
	itsContext.gop_size = 10; /* emit one intra frame every ten frames */

	if(codec->id != CODEC_ID_MPEG4 &&
			codec->id != CODEC_ID_MPEG1VIDEO &&
			codec->id != CODEC_ID_MPEG2VIDEO)
		itsContext.max_b_frames = 0;
	else
		itsContext.max_b_frames = 1;

	itsFrameNumber = 0;

	LINFO("using max_b_frames=%i bitrate=%u width=%u height=%u framerate=%u frameratebase=%u",
			itsContext.max_b_frames, itsContext.bit_rate, itsContext.width, itsContext.height, itsFramerate, frb);

	if (avcodec_open(&itsContext, codec) < 0)
		LFATAL("could not open codec\n");

#ifdef INVT_FFMPEG_HAS_FORMATCONTEXT_FUNCTIONS
	AVCodecContext *c = itsAVStream->codec;
	c->codec_id = itsContext.codec_id;
	c->codec_type = CODEC_TYPE_VIDEO;

	/* put sample parameters */
	c->bit_rate = itsContext.bit_rate;
	/* resolution must be a multiple of two */
	c->width = itsContext.width;
	c->height = itsContext.height;
	/* time base: this is the fundamental unit of time (in seconds) in terms
		 of which frame timestamps are represented. for fixed-fps content,
		 timebase should be 1/framerate and timestamp increments should be
		 identically 1. */
#if defined(INVT_FFMPEG_AVCODECCONTEXT_HAS_TIME_BASE)
	c->time_base.den = itsContext.time_base.den;
	c->time_base.num = itsContext.time_base.num;
#endif
	c->gop_size = 12; /* emit one intra frame every twelve frames at most */
	c->pix_fmt = itsContext.pix_fmt;

	/* set the output parameters (must be done even if no
		 parameters). */
	if (av_set_parameters(itsFormatContext, NULL) < 0)
		LFATAL("Invalid output format parameters");

	openBuffer();


	/* write the stream header, if any */
	av_write_header(itsFormatContext);
#else
	LFATAL("Need a new version of FFMPEG for this option");
#endif

	LINFO("EnCoder Inited");

	//Send the stream header to the client
	unsigned char *pb_buffer;
#if defined(INVT_FFMPEG_AVFORMATCONTEXT_BYTEIO_ISPOINTER)
	int len = url_close_dyn_buf(itsFormatContext->pb,
			(unsigned char **)(&pb_buffer));
#else
	int len = url_close_dyn_buf(&itsFormatContext->pb,
			(unsigned char **)(&pb_buffer));
#endif

	pthread_mutex_lock(&itsFrameLock);
	itsEncodedHeaderSize = len;
	itsEncodedHeader = new unsigned char[len];
	memcpy(itsEncodedHeader, pb_buffer, len);
	pthread_mutex_unlock(&itsFrameLock);
}


void HttpEncoder::openBuffer()
{
#if defined(INVT_FFMPEG_URL_OPEN_FUNC_TAKES_SINGLE_POINTER)

#if defined(INVT_FFMPEG_AVFORMATCONTEXT_BYTEIO_ISPOINTER)
	//if (url_fopen(itsFormatContext->pb, oname.c_str(), URL_WRONLY) < 0)
	if(url_open_dyn_buf(itsFormatContext->pb))
		LFATAL("Could not open stream");
#else
	//if (url_fopen(&itsFormatContext->pb, oname.c_str(), URL_WRONLY) < 0)
	if(url_open_dyn_buf(&itsFormatContext->pb))
		LFATAL("Could not open stream");
#endif

#else

#if defined(INVT_FFMPEG_AVFORMATCONTEXT_BYTEIO_ISPOINTER)
	//if (url_fopen(&itsFormatContext->pb, oname.c_str(), URL_WRONLY) < 0)
	if (url_open_dyn_buf(&itsFormatContext->pb))
		LFATAL("Could not open stream");
#else
	LFATAL("Could not open stream");
#endif

#endif //INVT_FFMPEG_URL_OPEN_FUNC_TAKES_SINGLE_POINTER)
}

void HttpEncoder::closeFrameBuffer()
{
	//Send the stream header to the client
	unsigned char *pb_buffer;

	pthread_mutex_lock(&itsFrameLock);
#if defined(INVT_FFMPEG_AVFORMATCONTEXT_BYTEIO_ISPOINTER)
	int len = url_close_dyn_buf(itsFormatContext->pb,
			(unsigned char **)(&pb_buffer));
#else
	int len = url_close_dyn_buf(&itsFormatContext->pb,
			(unsigned char **)(&pb_buffer));
#endif

	itsEncodedFrameBufferSize = len;
	itsEncodedFrameBuffer = pb_buffer;
	pthread_mutex_unlock(&itsFrameLock);
}

HttpEncoder::~HttpEncoder()
{
	close();

	itsThreadRunning = false;

	LINFO("Wait for thread");
	if (0 != pthread_join(itsRunThread, NULL))
		LERROR("Pthread__join() failed");
	LINFO("Disconnecting from stream");

}

int HttpEncoder::close()
{
	GVX_TRACE(__PRETTY_FUNCTION__);

	//if we went through this function already, then all the memory is freed
	if (itsFormatContext  == NULL)
		return 0;

	// (1) write any "delayed frames"
	{
		byte* const outbuf = (byte*) calloc(itsOutbufSize, 1);

		if (outbuf != 0)
		{
			while (true)
			{
				LINFO("pre  frame number %d", itsContext.frame_number);

				const int out_size =
					avcodec_encode_video(&itsContext, outbuf,
							itsOutbufSize, NULL);

				if (out_size <= 0)
					break;

				itsFrameSizeRange.merge(out_size);

#ifdef INVT_FFMPEG_HAS_FORMATCONTEXT_FUNCTIONS
				if (out_size > 0)
				{
					AVPacket pkt;
					av_init_packet(&pkt);

#if defined(INVT_FFMPEG_AVCODECCONTEXT_HAS_TIME_BASE)
					pkt.pts= av_rescale_q(itsContext.coded_frame->pts,
							itsContext.time_base, itsAVStream->time_base);
#endif
					if(itsContext.coded_frame->key_frame)
						pkt.flags |= PKT_FLAG_KEY;
					pkt.stream_index= itsAVStream->index;
					pkt.data= outbuf;
					pkt.size= out_size;

					/* write the compressed frame in the media file */
					openBuffer();
					av_write_frame(itsFormatContext, &pkt);
					closeFrameBuffer();
				}
#else
				LFATAL("Need a new version of ffmpeg for this option");
#endif

				LINFO("post frame number %d", itsContext.frame_number);
				LINFO("delayed frame (out_size=%d)", out_size);
			}

			free(outbuf);
		}
	}

	LINFO("end encoder: wrote %d frames, itsFrameSizeRange=[%d..%d]",
			itsFrameNumber, itsFrameSizeRange.min(), itsFrameSizeRange.max());

#ifdef INVT_FFMPEG_HAS_FORMATCONTEXT_FUNCTIONS
	avcodec_close(&itsContext);

	openBuffer();
	av_write_trailer(itsFormatContext);
	closeFrameBuffer();

	/* free the streams */
	for(uint i = 0; i < (uint)itsFormatContext->nb_streams; i++) {
		av_freep(&itsFormatContext->streams[i]->codec);
		av_freep(&itsFormatContext->streams[i]);
	}

	av_free(itsFormatContext);
	itsFormatContext = NULL;
#else
	LFATAL("Need a new version of ffmpeg for this option");
#endif

	return 0;
}

void HttpEncoder::writeRawFrame(const AVFrame* picture)
{
	GVX_TRACE(__PRETTY_FUNCTION__);

	// FIXME We'd like to have a way to either (1) compute what the
	// maximum necessary itsOutbufSize would be for our given
	// framerate+bitrate, or (2) get a chance to retry writing a given
	// frame if it is truncated. However, we have no programmatic way of
	// knowing whether a given frame gets truncated (all we see is that
	// ffmpeg prints "encoded frame too large" on stderr), but even then
	// the return value from avcodec_encode_video() is less than our
	// itsOutbufSize (although for a "too large" frame we can see
	// that the return value is clearly higher than usual). Also, it's
	// hard to determine a hard upper limit on the bufsize, even given
	// the framerate and bitrate, because the bitrate is only achieved
	// on /average/ -- so, any particular frame might be much larger
	// (e.g., 10x or 100x) than the average frame size. So, given all
	// that, our current approach is just to leave the buffer size up to
	// the user via the --output-mpeg-bufsize command-line option.

	// NOTE: it might seem extravagent to allocate+deallocate these
	// buffers (outbuf, and picture_buf in writeRGB()) for every single
	// frame that is written; however, profiling shows that this
	// accounts for only about 2% of the total time spent in
	// writeFrame(). The alternatives, both with their own
	// disadvantages, would be (1) have separate buffers allocated once
	// per object; however this would be expensive in overall memory
	// usage if we had multiple mpeg streams open at once; or (2) have
	// static buffers shared by all objects; however, this would require
	// some form of between-object synchronization in the case of
	// multi-threading which could be cpu-expensive both for the
	// locking+unlocking and would also waste time waiting to acquire
	// the lock for access to the shared buffers.

	rutz::fixed_block<byte> outbuf(itsOutbufSize);

	const int out_size = avcodec_encode_video(&itsContext,
			&outbuf[0],
			outbuf.size(),
			picture);

	if (out_size < 0)
		LFATAL("error during avcodec_encode_video()");

	if (out_size > 0)
	{
		itsFrameSizeRange.merge(out_size);

#ifdef INVT_FFMPEG_HAS_FORMATCONTEXT_FUNCTIONS
		AVPacket pkt;
		av_init_packet(&pkt);

		pkt.pts= av_rescale_q(itsContext.coded_frame->pts,
				itsContext.time_base, itsAVStream->time_base);
		if(itsContext.coded_frame->key_frame)
			pkt.flags |= PKT_FLAG_KEY;
		pkt.stream_index= itsAVStream->index;
		pkt.data= &outbuf[0];
		pkt.size= out_size;

		/* write the compressed frame in the media file */

		openBuffer();
		av_write_frame(itsFormatContext, &pkt);
		closeFrameBuffer();

#else
		LFATAL("New a new version of ffmpeg for this option");
#endif
	}

	LDEBUG("itsOutbufSize=%d, out_size=%d, frameSizeRange=[%d..%d]",
			itsOutbufSize, out_size,
			itsFrameSizeRange.min(), itsFrameSizeRange.max());

	LDEBUG("encoded frame [zero-based] %d (%d delayed frames pending)",
			itsFrameNumber,
			// to compute the number of pending "delayed frames", we
			// subtract the AVCodecContext's frame number from our own,
			// except that there is an offset of 2 -- one because
			// AVCodecContext counts from 1, while we count from zero, and
			// another because AVCodecContext's counter reports the number
			// of the NEXT frame to be written, while itsFrameNumber is
			// the number of the frame that has just been written
			itsFrameNumber - (itsContext.frame_number - 2));


	pthread_mutex_lock(&itsFrameNumLock);
	++itsFrameNumber;
	pthread_mutex_unlock(&itsFrameNumLock);
}

void HttpEncoder::writeRGB(const Image<PixRGB<byte> >& img)
{
	GVX_TRACE(__PRETTY_FUNCTION__);

	ASSERT(PIX_FMT_YUV420P == itsContext.pix_fmt);

	pthread_mutex_lock(&itsImgFrameLock);
	itsCurrentFrame = img;
	pthread_mutex_unlock(&itsImgFrameLock);

	const int size = itsContext.width * itsContext.height;
	const int size4 =
		((itsContext.width+1)/2) * (itsContext.height/2);

	rutz::fixed_block<byte> picture_buf(size + 2*size4); /* size for YUV 420 */

	AVFrame picture;
#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
	avcodec_get_frame_defaults(&picture);
#else
	{
		AVFrame* tmp = avcodec_alloc_frame();
		memcpy(&picture, tmp, sizeof(AVFrame));
		free(tmp);
	}
#endif

	picture.data[0] = &picture_buf[0];
	picture.data[1] = &picture_buf[0] + size;
	picture.data[2] = &picture_buf[0] + size + size4;
	picture.linesize[0] = itsContext.width;
	picture.linesize[1] = (itsContext.width+1) / 2;
	picture.linesize[2] = (itsContext.width+1) / 2;

	if (img.getWidth() != itsContext.width ||
			img.getHeight() != itsContext.height)
	{
		LFATAL("wrong size mpeg output frame "
				"(expected %dx%d, got %dx%d)",
				itsContext.width, itsContext.height,
				img.getWidth(), img.getHeight());
	}

	rgb24_to_yv12_c(img,
			picture.data[0],
			picture.data[1],
			picture.data[2]);

	this->writeRawFrame(&picture);
}

void HttpEncoder::writeVideoFrame(const VideoFrame& frame)
{
	GVX_TRACE(__PRETTY_FUNCTION__);

	if (frame.getDims().w() != itsContext.width ||
			frame.getDims().h() != itsContext.height)
	{
		LFATAL("wrong size mpeg output frame "
				"(expected %dx%d, got %dx%d)",
				itsContext.width, itsContext.height,
				frame.getDims().w(), frame.getDims().h());
	}

	AVFrame picture;
#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
	avcodec_get_frame_defaults(&picture);
#else
	{
		AVFrame* tmp = avcodec_alloc_frame();
		memcpy(&picture, tmp, sizeof(AVFrame));
		free(tmp);
	}
#endif

	if (convertVideoFrameToAVFrame(frame,
				itsContext.pix_fmt,
				&picture))
	{
		this->writeRawFrame(&picture);
	}
	else
	{
		// OK, we couldn't do a direct conversion from
		// VideoFrame->AVFrame (probably the pixel formats didn't
		// match), so let's just fall back to RGB instead:
		this->writeRGB(frame.toRgb());
	}
}

void HttpEncoder::writeFrame(const GenericFrame& f)
{
	if (f.nativeType() == GenericFrame::VIDEO)
	{
		this->writeVideoFrame(f.asVideo());
	}
	else
	{
		this->writeRGB(f.asRgb());
	}
}

void HttpEncoder::sendEncodedFrames(const int clientFd)
{

	//Send http header
	std::string httpHeader = "HTTP/1.1 200 OK\r\n"
		"CONTENT-TYPE: application/octet-stream\r\n"
		"CONNECTION: close\r\n"
		"\r\n";
	itsHttpServer->writeData(clientFd, httpHeader);

	///send header
	ssize_t bytes_written = ::write(clientFd, itsEncodedHeader, itsEncodedHeaderSize);
	if(bytes_written != itsEncodedHeaderSize)
		LERROR("Only %d/%d header bytes were written.", (int)bytes_written, itsEncodedHeaderSize);

	usleep(10000);

	int ret = 1;
	int lastFrameNumber = 0;
	while(ret > 0 && itsThreadRunning)
	{
		//Send data

		pthread_mutex_lock(&itsFrameNumLock);
		int frameNumber = itsFrameNumber;
		pthread_mutex_unlock(&itsFrameNumLock);

		if (frameNumber > lastFrameNumber)
		{
			pthread_mutex_lock(&itsFrameLock);
			ret = ::write(clientFd, itsEncodedFrameBuffer, itsEncodedFrameBufferSize);
			pthread_mutex_unlock(&itsFrameLock);
			lastFrameNumber = frameNumber;
		}

		usleep(10000);
	}
}

void HttpEncoder::sendMJPGFrames(const int clientFd)
{

	//Send http header

  std::string httpHeader = "HTTP/1.1 200 OK\r\n"
    "Server: INVT-HTTP-Output/0.2\r\n"
    "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n"
    "Pragma: no-cache\r\n"
    "Content-Type: multipart/x-mixed-replace;boundary=boundarydonotcross\r\n"
    "\r\n";
	itsHttpServer->writeData(clientFd, httpHeader);
	usleep(10000);

  std::string jpegHeader =  "--boundarydonotcross\r\n"
		"Content-Type: image/jpeg\r\n"
		"\r\n";

	int ret = 1;
	int lastFrameNumber = 0;
	while(ret > 0 && itsThreadRunning)
	{
		//Send data

		pthread_mutex_lock(&itsFrameNumLock);
		int frameNumber = itsFrameNumber;
		pthread_mutex_unlock(&itsFrameNumLock);

		if (frameNumber > lastFrameNumber)
		{
			pthread_mutex_lock(&itsImgFrameLock);
			Image<PixRGB<byte> > tmp = itsCurrentFrame.deepcopy(); 
			pthread_mutex_unlock(&itsImgFrameLock);

	
			JPEGCompressor jpegCompressor;
      std::vector<unsigned char> jpgRawData = jpegCompressor.compressImage(tmp);

			if (jpgRawData.size() > 0)
			{
				std::vector<unsigned char> jpgData;
				for(uint i=0; i<jpegHeader.size(); i++)
					jpgData.push_back(jpegHeader[i]);

				for(uint i=0; i<jpgRawData.size(); i++)
					jpgData.push_back(jpgRawData[i]);

				ret = ::write(clientFd, &jpgData[0], jpgData.size());
				usleep(100000);
			}

			lastFrameNumber = frameNumber;
		}

		usleep(10000);
	}
}
void HttpEncoder::run()
{
	while(itsThreadRunning)
	{
		int clientFd = itsHttpServer->acceptConn();
		if (clientFd > 0)
		{
			LINFO("New client, send data %i", clientFd);
			//sendEncodedFrames(clientFd);
			sendMJPGFrames(clientFd);
			::close(clientFd);
		}
		usleep(10000);
	}
}

#endif // HAVE_FFMPEG_AVCODEC_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
