/*!@file Media/FfmpegPacketDecoder.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FfmpegPacketDecoder.C $
// $Id: FfmpegPacketDecoder.C 15350 2012-07-28 02:21:45Z beobot $
//

// Some code in this file is based on ffplay from the ffmpeg
// distribution, with this original copyright notice:

/*
 * FFplay : Simple Media Player based on the ffmpeg libraries
 * Copyright (c) 2003 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef MEDIA_FFMPEGPACKETDECODER_C_DEFINED
#define MEDIA_FFMPEGPACKETDECODER_C_DEFINED

#ifdef INVT_HAVE_AVCODEC

#include "Media/FfmpegPacketDecoder.H"

#include "Raster/GenericFrame.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Video/FfmpegFrame.H"

#include <cerrno>

// see http://dranger.com/ffmpeg/ffmpegtutorial_all.txt for useful info

namespace
{
  // ######################################################################
  static int& eof_reached(AVFormatContext* c)
  {
    // AVFormatContext's pb member changed from a 'ByteIOContext' to a
    // 'ByteIOContext*' with ffmpeg svn rev 11071; see this thread:
    // http://lists.mplayerhq.hu/pipermail/ffmpeg-devel/2007-November/037797.html
#if defined (LIBAVFORMAT_BUILD) && (LIBAVFORMAT_BUILD >= ((52<<16)+(0<<8)+0))
    ASSERT(c->pb != 0);
    return c->pb->eof_reached;
#else
    return c->pb.eof_reached;
#endif
  }

  bool isNonNegative(int v) { return v >= 0; }
  bool isNonNegative(unsigned int v) { return true; }
}

// ######################################################################
FfmpegPacketDecoder::FfmpegPacketDecoder(const char* fname,
                                         const bool preload)
  :
  itsFilename(fname),
  itsFormatContext(0),
  itsCodecContext(0),
  itsStreamID(-1),
  itsFrameNumber(0),
  itsDimsValid(false),
  itsNextFramePushback(false),
  itsPacketsExhausted(false),
  itsPacketQ()
{
  // no need to guard these functions for being called multiple times;
  // they all have internal guards
  av_register_all();
  avcodec_init();
  avcodec_register_all();

  AVFormatParameters params;
  memset(&params, 0, sizeof(params));
#if defined (LIBAVFORMAT_BUILD) && (LIBAVFORMAT_BUILD <= ((50<<16)+(6<<8)))
  // AVImageFormat* disappeared from ffmpeg mainline on 2006-11-02;
  // last version with it was 50.6.0 and first version without it was
  // 51.6.0
  params.image_format = 0;
#endif
#if defined (LIBAVFORMAT_BUILD) && (LIBAVFORMAT_BUILD >= 4610)
  params.initial_pause = 1; /* we force a pause when starting an RTSP
                            stream */
#endif

  int err = av_open_input_file(&itsFormatContext, itsFilename.c_str(),
                               0, 0, &params);
  if (err < 0)
    {
      char errbuf[512];
#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21
      if (av_strerror(err, errbuf, 512))
        LFATAL("%s: error %i while opening file", itsFilename.c_str(), err);
      else
#endif
        LFATAL("%s: error while opening file: %s", itsFilename.c_str(), errbuf);
    }

  err = av_find_stream_info(itsFormatContext);
  if (err < 0)
    {
      LFATAL("%s: could not find codec parameters\n",
             itsFilename.c_str());
    }
  eof_reached(itsFormatContext) = 0; //FIXME hack, ffplay maybe shouldnt use url_feof() to test for the end

  /* now we can begin to play (RTSP stream only) */
#if defined (LIBAVFORMAT_BUILD) && (LIBAVFORMAT_BUILD >= 4610)
  av_read_play(itsFormatContext);
#endif

  LDEBUG("%s: %d streams in file", fname,
         itsFormatContext->nb_streams);

  // Prior to ffmpeg svn revision 7556, nb_streams was 'int', but then
  // changed to 'unsigned int'; to allow either type we will later
  // cast it to unsigned int, but first we want to check that it is
  // indeed non-negative:
  ASSERT(isNonNegative(itsFormatContext->nb_streams));

  for (unsigned int i = 0; i < (unsigned int)(itsFormatContext->nb_streams); ++i)
    {
#ifdef INVT_FFMPEG_AVSTREAM_CODEC_IS_POINTER
      AVCodecContext *enc = itsFormatContext->streams[i]->codec;
#else
      AVCodecContext *enc = itsFormatContext->streams[i]->codec;
#endif

      char buf[512];
      avcodec_string(&buf[0], sizeof(buf), enc, /*encode=*/ 0);
      buf[sizeof(buf)-1] = '\0';

      LDEBUG("%s: stream %u/%u: codec_string=%s",
             fname, i, (unsigned int)(itsFormatContext->nb_streams),
             buf);

#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21
      if (enc->codec_type == AVMEDIA_TYPE_VIDEO)
#else
      if (enc->codec_type == CODEC_TYPE_VIDEO)
#endif
        {
          itsStreamID = i;
          itsCodecContext = enc;
          break;
        }
    }

  if (itsStreamID < 0)
    LFATAL("%s: no video stream", itsFilename.c_str());

  ASSERT(itsCodecContext != 0);
  AVCodec* codec = avcodec_find_decoder(itsCodecContext->codec_id);

  if (codec == 0)
    LFATAL("%s: no codec found for codec_id=%d",
           fname, int(itsCodecContext->codec_id));

#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD >= 4697) // rev 2636
  itsCodecContext->debug_mv = 0;
#endif
  itsCodecContext->debug = 0;
  itsCodecContext->workaround_bugs = 1;
#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD >= 4722) // rev 3507
  itsCodecContext->lowres = 0;
  if (itsCodecContext->lowres)
    itsCodecContext->flags |= CODEC_FLAG_EMU_EDGE;
#endif
  itsCodecContext->idct_algo= FF_IDCT_AUTO;
#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD > 4721) // rev 3429
  if (0)
    itsCodecContext->flags2 |= CODEC_FLAG2_FAST;
#endif
#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD >= 4758) // rev 4440
  itsCodecContext->skip_frame= AVDISCARD_DEFAULT;
  itsCodecContext->skip_idct= AVDISCARD_DEFAULT;
  itsCodecContext->skip_loop_filter= AVDISCARD_DEFAULT;
#endif

#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD > 3410431) // rev 5210
  // no more error_resilience parameter?
#elif defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD > 3276800) // rev 4590
  itsCodecContext->error_resilience= FF_ER_CAREFUL;
#else
  itsCodecContext->error_resilience= FF_ER_CAREFULL;
#endif
  itsCodecContext->error_concealment= 3;
  if (!codec || avcodec_open(itsCodecContext, codec) < 0)
    LFATAL("avcodec_open() failed");
#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD >= 4702) // rev 2772
  itsCodecContext->thread_count= 1;
#endif

#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
  avcodec_get_frame_defaults(&itsPicture);
#else
  {
    AVFrame* tmp = avcodec_alloc_frame();
    memcpy(&itsPicture, tmp, sizeof(AVFrame));
    av_free(tmp);
  }
#endif

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
  avcodec_string(&buf[0], sizeof(buf), itsCodecContext, /*encode=*/ 0);
  buf[sizeof(buf)-1] = '\0';
  LINFO("%s [%s]", fname, &buf[0]);

  if (preload)
    while (getNextPacket() == true)
      { /* empty */ }
}

// ######################################################################
FfmpegPacketDecoder::~FfmpegPacketDecoder()
{
  if (itsCodecContext)
    {
      avcodec_close(itsCodecContext);
      itsCodecContext = 0;
      itsStreamID = -1;
    }

  if (itsFormatContext)
    {
      av_close_input_file(itsFormatContext);
      itsFormatContext = NULL; /* safety */
    }

  while (!itsPacketTrashQ.empty())
    {
      av_free_packet(&(itsPacketTrashQ.back()));
      itsPacketTrashQ.pop_back();
    }
}

// ######################################################################
int FfmpegPacketDecoder::apparentFrameNumber() const
{
  return
    itsNextFramePushback
    ? itsFrameNumber - 1
    : itsFrameNumber;
}

// ######################################################################
GenericFrameSpec FfmpegPacketDecoder::peekFrameSpec()
{
  if (!itsDimsValid)
    {
      // if we've already peeked at the next frame, then the dims
      // should have already be valid:
      ASSERT(!itsNextFramePushback);

      if (readRawFrame() == 0)
        {
          if (itsCodecContext->width % 16 != 0)
            LFATAL("readRawFrame() failed (this may be because the "
                   "movie's width is %d, which is not divisible by 16)",
                   itsCodecContext->width);
          else if (itsCodecContext->height % 16 != 0)
            LFATAL("readRawFrame() failed (this may be because the "
                   "movie's height is %d, which is not divisible by 16)",
                   itsCodecContext->height);
          else
            LFATAL("readRawFrame() failed");
        }

      itsNextFramePushback = true;
    }

  ASSERT(itsDimsValid);

  GenericFrameSpec result;

  result.nativeType = GenericFrame::VIDEO;
  result.videoFormat =
    convertAVPixelFormatToVideoFormat(itsCodecContext->pix_fmt);
  result.videoByteSwap = false;
  result.dims = Dims(itsCodecContext->width, itsCodecContext->height);
  result.floatFlags = 0;

#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD >= 4754) // SVN rev >= 4168
  result.frameRate = static_cast<float>(1/av_q2d(itsCodecContext->time_base)) ;
#else // assume FFmpeg libavcodec build 4753 or earlier (i.e., SVN rev <= 4161)
  result.frameRate = itsCodecContext->frame_rate ;
#endif

  return result;
}

// ######################################################################
VideoFrame FfmpegPacketDecoder::readVideoFrame()
{
  // note that we need to force the peekFrameSpec() call to occur
  // before the convertAVFrameToVideoFrame() call, so that
  // itsCodecContext->{width,height} are properly initialized
  const GenericFrameSpec spec = this->peekFrameSpec();

  return convertAVFrameToVideoFrame(this->readRawFrame(),
                                    itsCodecContext->pix_fmt,
                                    spec.dims);
}

// ######################################################################
Image<PixRGB<byte> > FfmpegPacketDecoder::readRGB()
{
  // note that we need to force the peekFrameSpec() call to occur
  // before the convertAVFrameToRGB() call, so that
  // itsCodecContext->{width,height} are properly initialized
  const GenericFrameSpec spec = this->peekFrameSpec();

  return convertAVFrameToRGB(this->readRawFrame(),
                             itsCodecContext->pix_fmt,
                             spec.dims);
}

// ######################################################################
bool FfmpegPacketDecoder::readAndDiscardFrame()
{
  return (readRawFrame() != 0);
}

// ######################################################################
bool FfmpegPacketDecoder::getNextPacket()
{
#if !(defined(LIBAVFORMAT_BUILD) && (LIBAVFORMAT_BUILD >= 4610))
  LFATAL("you must have <ffmpeg/avformat.h> with "
         "LIBAVFORMAT_BUILD >= 4610 to use FfmpegPacketDecoder");
  /* can't happen */ return false;
#else
  if (eof_reached(itsFormatContext))
    return false;

  while (true)
    {
      AVPacket pkt;
      av_init_packet(&pkt);
      const int ret = av_read_frame(itsFormatContext, &pkt);
      if (ret < 0)
        {
          return false;
        }
      else
        {
          LDEBUG("eof_reached=%d pkt = {data=%p size=%d "
                 "stream_index=%d flags=%d duration=%d}",
                 int(eof_reached(itsFormatContext)),
                 pkt.data, pkt.size, pkt.stream_index,
                 pkt.flags, pkt.duration);

          if (pkt.stream_index == itsStreamID &&
              !eof_reached(itsFormatContext))
            {
              av_dup_packet(&pkt);
              itsPacketQ.push_back(pkt);
              return true;
            }
          else
            {
              av_free_packet(&pkt);
              // no return here; let the loop go around again and try
              // to get another packet
            }
        }
    }
#endif
}

// ######################################################################
AVFrame* FfmpegPacketDecoder::readRawFrame()
{
  if (itsNextFramePushback)
    {
      itsNextFramePushback = false;
      return &itsPicture;
    }

  if (itsPacketsExhausted)
    return 0;

  while (!itsPacketTrashQ.empty())
    {
      av_free_packet(&(itsPacketTrashQ.back()));
      itsPacketTrashQ.pop_back();
    }

  while (true)
    {
      this->getNextPacket();

      if (itsPacketQ.size() == 0)
        {
#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 35
          AVPacket pkt;
          av_init_packet(&pkt);
          pkt.size = 0;
          pkt.data = NULL;
#endif
          
          int got_picture;
          const int len =
            
#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 35
            avcodec_decode_video2(itsCodecContext,
                                  &itsPicture, &got_picture,
                                  &pkt);
          av_free_packet(&pkt);
            
          
#elif LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21
            avcodec_decode_video2(itsCodecContext,
                                  &itsPicture, &got_picture,
                                  NULL);
#else
            avcodec_decode_video(itsCodecContext,
                                 &itsPicture, &got_picture,
                                 NULL, 0);
#endif
          (void) len;

          itsPacketsExhausted = true;

          if (got_picture)
            {
              ++itsFrameNumber;
              itsDimsValid = true;
              return &itsPicture;
            }

          return 0;
        }

      // ok, if we got here then our packet queue is non-empty, so
      // let's pull out the next packet and handle it:

      ASSERT(itsPacketQ.size() > 0);
      AVPacket pkt = itsPacketQ.front();
      itsPacketQ.pop_front();

      ASSERT(pkt.stream_index == itsStreamID);

      int got_picture;
      const int len =
#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21
        avcodec_decode_video2(itsCodecContext,
                             &itsPicture, &got_picture,
                             &pkt);
#else
        avcodec_decode_video(itsCodecContext,
                             &itsPicture, &got_picture,
                             pkt.data, pkt.size);
#endif
      // we can't call av_free_packet(&pkt) just now because
      // itsPicture may refer to pkt.data internally, so if we freed
      // the packet we could have dangling pointers in itsPicture;
      // instead, we just note that the packet must be freed later and
      // then we free it when we start to read the next frame
      itsPacketTrashQ.push_back(pkt);

      if (len < 0)
        LFATAL("avcodec_decode_video() failed");

      if (got_picture)
        {
          ++itsFrameNumber;
          itsDimsValid = true;
          return &itsPicture;
        }
    }
}

#endif // INVT_HAVE_AVCODEC

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_FFMPEGPACKETDECODER_C_DEFINED
