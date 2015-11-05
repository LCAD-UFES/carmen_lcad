/*!@file Media/FfmpegDecoder.C Low-level class for using ffmpeg to decode movie files */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FfmpegDecoder.C $
// $Id: FfmpegDecoder.C 15352 2012-07-28 10:43:52Z kai $
//

#ifndef MEDIA_FFMPEGDECODER_C_DEFINED
#define MEDIA_FFMPEGDECODER_C_DEFINED

#ifdef INVT_HAVE_AVCODEC

#include "Media/FfmpegDecoder.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Video/FfmpegFrame.H"
#include "Video/VideoFrame.H"
#include "rutz/trace.h"

#include <cstdlib>
#define attribute_deprecated

extern "C"
{
//These directives are necessary to handle the various places in which
//different versions of ffmpeg install their files. Unfortunately,
//it looks like the cdeps program can't handle the more elegant
//#if defined(XXX) #elif defined(XXX) #endif, so we have to do the following:

#ifdef HAVE_LIBAVCODEC_AVCODEC_H
  #include <libavcodec/avcodec.h>
#else
#ifdef HAVE_FFMPEG_AVCODEC_H
  #include <ffmpeg/avcodec.h>
#endif
#endif

#ifdef HAVE_LIBAVFORMAT_AVFORMAT_H
  #include <libavformat/avformat.h>
#else
#ifdef HAVE_FFMPEG_AVFORMAT_H
  #include <ffmpeg/avformat.h>
#endif
#endif
}

#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>

namespace
{
  bool isNonNegative(int v) { return v >= 0; }
  bool isNonNegative(unsigned int v) { return true; }

  template <class T>
  Image<T> vFlip(const Image<T>& src)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    Image<T> result(src.getDims(), NO_INIT);

    const int w = src.getWidth();
    const int h = src.getHeight();

    const T* sptr = src.getArrayPtr();
    T* dptr = result.getArrayPtr() + (h-1)*w;

    for (int y = 0; y < h; ++y)
      {
        safecopy(dptr, sptr, w);
        sptr += w;
        dptr -= w;
      }

    return result;
  }

  AVCodec* findVideoCodec(const char* fname, AVInputFormat* iformat)
  {
    AVFormatContext* ic;
    int err = av_open_input_file(&ic, fname, iformat, 0, NULL);
    if (err < 0)
      LFATAL("Error opening input file %s: %d", fname, err);

    err = av_find_stream_info(ic);
    if (err < 0)
      LFATAL("Cannot determine stream codec parameters: %d", err);

    LINFO("Using input format '%s' (%s)",
          ic->iformat->name, ic->iformat->long_name);

    AVCodec* result = 0;

    // Prior to ffmpeg svn revision 7556, nb_streams was 'int', but then
    // changed to 'unsigned int'; to allow either type we will later
    // cast it to unsigned int, but first we want to check that it is
    // indeed non-negative:
    ASSERT(isNonNegative(ic->nb_streams));

    for (unsigned int i = 0; i < (unsigned int)(ic->nb_streams); ++i)
      {
#ifdef INVT_FFMPEG_AVSTREAM_CODEC_IS_POINTER
        const AVCodecContext* const cc = ic->streams[i]->codec;
#else
        const AVCodecContext* const cc = ic->streams[i]->codec;
#endif

#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21
        if (cc->codec_type == AVMEDIA_TYPE_VIDEO) //new avcodec
#else
          if (cc->codec_type == CODEC_TYPE_VIDEO) //old avcodec
#endif
          {
            result = avcodec_find_decoder(cc->codec_id);
            if (result == 0)
              LFATAL("codec not found");
            break;
          }
      }

    av_close_input_file(ic);

    if (result == 0)
      LFATAL("Could not find a video stream in input file %s", fname);

    return result;
  }
}

// ######################################################################
FfmpegDecoder::FfmpegDecoder(const char* codecname,
                             const int bufflen,
                             const char* fname,
                             const bool preload)
  :
  itsFile(NULL),
  itsContext(),
  itsPicture(),
  itsFrameNumber(0),
  itsBuf(),
  itsIdxStart(0),
  itsIdxEnd(0),
  itsInputEOF(false),
  itsDimsValid(false),
  itsNextFramePushback(false)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  // no need to guard these functions for being called multiple times;
  // they all have internal guards
  av_register_all();
  avcodec_init();
  avcodec_register_all();

  AVInputFormat* iformat = NULL;
  if (strcmp(codecname, "List") == 0)
    {
      // list available codecs
      LINFO("##### Available input codecs (not all may work for video):");
#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21
      for (AVInputFormat* f = av_iformat_next(NULL); f != NULL; f = av_iformat_next(f))
#else
      for (AVInputFormat* f = first_iformat; f != NULL; f = f->next)
#endif
        LINFO("%s: %s %d", f->name, f->long_name, f->flags);
      LFATAL("Please select a codec from this list");
    }
  else if (strcmp(codecname, "Auto") != 0)
    {
      // format is given
      iformat = av_find_input_format(codecname);
    }

  // ok, let's find a video stream:
  AVCodec* const codec = findVideoCodec(fname, iformat);

  ASSERT(codec != 0);

#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
  avcodec_get_context_defaults(&itsContext);
#else
  {
    AVCodecContext* const tmp = avcodec_alloc_context();
    memcpy(&itsContext, tmp, sizeof(AVCodecContext));
    free(tmp);
  }
#endif

#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
  avcodec_get_frame_defaults(&itsPicture);
#else
  {
    AVFrame* tmp = avcodec_alloc_frame();
    memcpy(&itsPicture, tmp, sizeof(AVFrame));
    free(tmp);
  }
#endif

  if (codec->capabilities & CODEC_CAP_TRUNCATED)
    itsContext.flags |= CODEC_FLAG_TRUNCATED;

  if (avcodec_open(&itsContext, codec) < 0)
    LFATAL("could not open codec\n");

  // open the stream:
  if (itsFile) fclose(itsFile);
  itsFile = fopen(fname, "rb");
  if (itsFile == NULL)
    LFATAL("could not open file! %s", fname);

  // get a read buffer:
  int blen;
  if (preload) // allocate a buffer for the entire movie
    {
      struct stat st;
      const int fd = fileno(itsFile);
      if (fd == -1) PLFATAL("Problem with fileno()");
      if (fstat(fd, &st) == -1) PLFATAL("Cannot stat %s", fname);
      blen = int(st.st_size);
    }
  else
    blen = bufflen; // allocate a buffer for a chunk of movie

  itsBuf.resize(blen);
  itsIdxStart = 0;
  itsIdxEnd = 0;
  itsFrameNumber = 0;
  itsInputEOF = false;
  itsDimsValid = false;

  LINFO("libavcodec build %d (%d.%d.%d)",
        int(LIBAVCODEC_BUILD),
        int((LIBAVCODEC_BUILD & 0xff0000) >> 16),
        int((LIBAVCODEC_BUILD & 0xff00) >> 8),
        int((LIBAVCODEC_BUILD & 0xff) >> 0));

  LINFO("libavformat build %d (%d.%d.%d)",
        int(LIBAVFORMAT_BUILD),
        int((LIBAVFORMAT_BUILD & 0xff0000) >> 16),
        int((LIBAVFORMAT_BUILD & 0xff00) >> 8),
        int((LIBAVFORMAT_BUILD & 0xff) >> 0));

  char buf[512];
  avcodec_string(&buf[0], sizeof(buf), &itsContext, /*encode=*/ 0);
  buf[sizeof(buf)-1] = '\0';
  LINFO("%s [%s]", fname, &buf[0]);

  // if preload, let's load up the entire movie now and close itsFile:
  if (preload)
    {
      const int size = fread(&itsBuf[0], 1, itsBuf.size(), itsFile);
      if (size <= 0) PLFATAL("Read error");
      itsIdxEnd = size_t(size);
      // close the stream since we have all the data already:
      fclose(itsFile);
      itsFile = NULL;
      LINFO("pre-loaded %s", fname);
    }
}

// ######################################################################
FfmpegDecoder::~FfmpegDecoder()
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsFile) { fclose(itsFile); }
  avcodec_close(&itsContext);
}

// ######################################################################
int FfmpegDecoder::apparentFrameNumber() const
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  return
    itsNextFramePushback
    ? itsFrameNumber - 1
    : itsFrameNumber;
}

// ######################################################################
GenericFrameSpec FfmpegDecoder::peekFrameSpec()
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (!itsDimsValid)
    {
      // if we've already peeked at the next frame, then the dims
      // should have already be valid:
      ASSERT(!itsNextFramePushback);

      readRawFrame();
      itsNextFramePushback = true;
    }

  ASSERT(itsDimsValid);

  GenericFrameSpec result;

  result.nativeType = GenericFrame::VIDEO;
  result.videoFormat =
    convertAVPixelFormatToVideoFormat(itsContext.pix_fmt);
  result.videoByteSwap = false;
  result.dims = Dims(itsContext.width, itsContext.height);
  result.floatFlags = 0;

#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD >= 4754) // SVN rev >= 4168
  result.frameRate = static_cast<float>(1/av_q2d(itsContext.time_base)) ;
#else // assume FFmpeg libavcodec build 4753 or earlier (i.e., SVN rev <= 4161)
  result.frameRate = itsContext.frame_rate ;
#endif

  return result;
}

// ######################################################################
VideoFrame FfmpegDecoder::readVideoFrame()
{
  // note that we need to force the peekFrameSpec() call to occur
  // before the convertAVFrameToVideoFrame() call, so that
  // itsContext.{width,height} are properly initialized
  const GenericFrameSpec spec = this->peekFrameSpec();

  return convertAVFrameToVideoFrame(this->readRawFrame(),
                                    itsContext.pix_fmt,
                                    spec.dims);
}

// ######################################################################
Image<PixRGB<byte> > FfmpegDecoder::readRGB()
{
  // note that we need to force the peekFrameSpec() call to occur
  // before the convertAVFrameToRGB() call, so that
  // itsContext.{width,height} are properly initialized
  const GenericFrameSpec spec = this->peekFrameSpec();

  return convertAVFrameToRGB(this->readRawFrame(),
                             itsContext.pix_fmt,
                             spec.dims);
}

// ######################################################################
bool FfmpegDecoder::readAndDiscardFrame()
{
  return (readRawFrame() != 0);
}

// ######################################################################
const AVFrame* FfmpegDecoder::readRawFrame()
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsNextFramePushback)
    {
      itsNextFramePushback = false;
      return &itsPicture;
    }

  if (itsInputEOF) return NULL;  // we have reached end of file already

  int nlen0 = 0;

  while (true)
    {
      bool goteof = false;

      ASSERT(itsIdxEnd >= itsIdxStart);

      // do we need to read more data from file?
      if (itsIdxEnd < itsIdxStart + 16384)
        {
          const size_t size = this->refillBuffer();
          if (size == 0) goteof = true; // end of file
        }

      LDEBUG("buffer range = %" ZU " - %" ZU " of %" ZU ", goteof=%d",
             itsIdxStart, itsIdxEnd, itsBuf.size(), int(goteof));

      // decode contents of our read buffer if any:
      int gotpic = 0;
//from 52.72 to 53.35
#if (LIBAVCODEC_VERSION_MAJOR == 52 && LIBAVCODEC_VERSION_MINOR >= 72 )||LIBAVCODEC_VERSION_MAJOR > 52 
      //for new avcodec
      AVPacket avpkt;
      av_init_packet(&avpkt);
      avpkt.data = (uint8_t *) &itsBuf[itsIdxStart];
      avpkt.size = itsIdxEnd - itsIdxStart;
      avpkt.flags = AV_PKT_FLAG_KEY;
      const int len =
        avcodec_decode_video2(&itsContext, &itsPicture, &gotpic, &avpkt);
#else
      //for old avcodec
      const int len =
        avcodec_decode_video(&itsContext, &itsPicture, &gotpic,
                             &itsBuf[itsIdxStart],
                             itsIdxEnd - itsIdxStart);
#endif



      if (len == 0) ++nlen0;
      else nlen0 = 0;

      LDEBUG("end-start=%" ZU ", len=%d, nlen0=%d, gotpic=%d",
             itsIdxEnd-itsIdxStart, len, nlen0, gotpic);

      if (len < 0)
        LFATAL("Error while decoding frame %d", itsFrameNumber);
      else if (size_t(len) > (itsIdxEnd - itsIdxStart))
        {
          const size_t minsize =
            std::max(4*size_t(len)+4096,
                     4*(itsIdxEnd-itsIdxStart)+4096);

          if (minsize > itsBuf.size())
            // ok, libavcodec needs a bigger buffer in order to be
            // able to hold a full frame, so let's do that now:
            itsBuf.resize(minsize);

          // ok, the decoder wants more data...
          const size_t size = this->refillBuffer();
          if (size == 0)
            LFATAL("libavcodec wanted more data, but we are at eof");
        }
      else
        {
          itsIdxStart += len;
          if ((itsIdxStart == itsIdxEnd) && goteof && (gotpic || nlen0 >= 2))
            itsInputEOF = true; // decoded last frame
          if ((itsIdxStart == itsIdxEnd) && goteof && (nlen0 >= 2))
            return NULL;
          if (gotpic)
            {
              ++itsFrameNumber;
              itsDimsValid = true;
              return &itsPicture;
            }
        }
    }
}

// ######################################################################
size_t FfmpegDecoder::refillBuffer()
{
  // let's move the data we have up to the front of the buffer, and
  // then fill the buffer again
  const size_t nsave = itsIdxEnd - itsIdxStart;
  ASSERT(itsBuf.size() > nsave);

  if (nsave > 0)
    memmove(&itsBuf[0], &itsBuf[itsIdxStart], nsave);
  itsIdxStart = 0;
  itsIdxEnd = nsave;

  // if our file is already closed (e.g. if we preloaded the entire
  // movie), then we can't read any more data, so just return 0:
  if (itsFile == 0)
    return 0;

  const int size = fread(&itsBuf[0] + nsave, 1,
                         itsBuf.size() - nsave, itsFile);
  if (size < 0)
    PLFATAL("read error");
  itsIdxEnd += size;
  return size_t(size);
}

#endif // INVT_HAVE_AVCODEC

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_FFMPEGDECODER_C_DEFINED
