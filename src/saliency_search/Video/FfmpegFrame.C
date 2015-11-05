/*!@file Video/FfmpegFrame.C Conversions between ffmpeg's AVFrame and our VideoFrame */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/FfmpegFrame.C $
// $Id: FfmpegFrame.C 12785 2010-02-06 02:24:05Z irock $
//

#ifndef VIDEO_FFMPEGFRAME_C_DEFINED
#define VIDEO_FFMPEGFRAME_C_DEFINED

#ifdef INVT_HAVE_AVCODEC

#include "Video/FfmpegFrame.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/color_conversions.H" // for yv12_to_rgb24_c
#include "Video/VideoFrame.H"
#include "rutz/trace.h"

#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD > 4718)
#  define HAVE_PIX_FMT_UYVY422
#endif

#if ! PIX_FMT_UYVY411
#define PIX_FMT_UYVY411 PIX_FMT_UYYVYY411
#endif

#if ! PIX_FMT_RGBA32
#define PIX_FMT_RGBA32  PIX_FMT_RGB32
#endif

#if ! PIX_FMT_YUV422
#define PIX_FMT_YUV422  PIX_FMT_YUYV422
#endif

namespace
{
  inline void copy_rows(byte* dst, const int dstlen,
                        const byte* src, const int srclen,
                        int h)
  {
    if (dstlen == srclen)
      {
        memcpy(dst, src, dstlen * h);
      }
    else
      {
        const int minlen = std::min(dstlen, srclen);
        for (int j = 0; j < h; ++j)
          {
            memcpy(dst, src, minlen); src += srclen; dst += dstlen;
          }
      }
  }
}

#ifdef HAVE_PIX_FMT_UYVY422
// ######################################################################
static VideoFrame uyvy_to_VideoFrame(const AVFrame* pic,
                                     const PixelFormat fmt,
                                     const Dims& dims)
{
  ASSERT(fmt == PIX_FMT_UYVY422);

  if (pic == 0 || dims.isEmpty())
    return VideoFrame();

  const int w = dims.w();
  const int h = dims.h();
  ArrayHandle<byte> hdl
    (new ArrayData<byte>(Dims(((w+1)/2)*h*4,1), NO_INIT));

  const int dstlen = ((w+1)/2)*4, srclen = pic->linesize[0];

  copy_rows(hdl.uniq().dataw(), dstlen,
            pic->data[0], srclen, h);

  return VideoFrame(hdl, dims, VIDFMT_UYVY, false);
}
#endif

// ######################################################################
static VideoFrame yuv420p_to_VideoFrame(const AVFrame* pic,
                                        const PixelFormat fmt,
                                        const Dims& dims)
{
  ASSERT(fmt == PIX_FMT_YUV420P);

  if (pic == 0 || dims.isEmpty())
    return VideoFrame();

  const int w = dims.w();
  const int h = dims.h();
  ArrayHandle<byte> hdl
    (new ArrayData<byte>(Dims(w * h + 2 * ( ((w+1) / 2) * ((h+1) / 2) ),
                              1),
                         NO_INIT));
  byte* buf = hdl.uniq().dataw();

  const byte* ysrc = pic->data[0];
  const byte* usrc = pic->data[1];
  const byte* vsrc = pic->data[2];

  byte* ydst = buf;
  byte* udst = ydst + w * h;
  byte* vdst = udst + ((w+1)/2) * ((h+1)/2);

  const int ydstlen = w,         ysrclen = pic->linesize[0];
  const int udstlen = ((w+1)/2), usrclen = pic->linesize[1];
  const int vdstlen = ((w+1)/2), vsrclen = pic->linesize[2];

  copy_rows(ydst, ydstlen, ysrc, ysrclen, h);
  copy_rows(udst, udstlen, usrc, usrclen, h/2);
  copy_rows(vdst, vdstlen, vsrc, vsrclen, h/2);

  return VideoFrame(hdl, dims, VIDFMT_YUV420P, false);
}

// ######################################################################
static VideoFrame yuv422p_to_VideoFrame(const AVFrame* pic,
                                        const PixelFormat fmt,
                                        const Dims& dims)
{
  ASSERT(fmt == PIX_FMT_YUVJ422P);

  if (pic == 0 || dims.isEmpty())
    return VideoFrame();

  const int w = dims.w();
  const int h = dims.h();
  ArrayHandle<byte> hdl
    (new ArrayData<byte>(Dims(w * h + 2 * ( ((w+1) / 2) * h ),
                              1),
                         NO_INIT));
  byte* buf = hdl.uniq().dataw();

  const byte* ysrc = pic->data[0];
  const byte* usrc = pic->data[1];
  const byte* vsrc = pic->data[2];

  byte* ydst = buf;
  byte* udst = ydst + w * h;
  byte* vdst = udst + ((w+1)/2) * h;

  const int ydstlen = w,         ysrclen = pic->linesize[0];
  const int udstlen = ((w+1)/2), usrclen = pic->linesize[1];
  const int vdstlen = ((w+1)/2), vsrclen = pic->linesize[2];

  copy_rows(ydst, ydstlen, ysrc, ysrclen, h);
  copy_rows(udst, udstlen, usrc, usrclen, h);
  copy_rows(vdst, vdstlen, vsrc, vsrclen, h);

  return VideoFrame(hdl, dims, VIDFMT_YUV422P, false);
}

// ######################################################################
static VideoFrame yuv411p_to_VideoFrame(const AVFrame* pic,
                                        const PixelFormat fmt,
                                        const Dims& dims)
{
  ASSERT(fmt == PIX_FMT_YUV411P);

  if (pic == 0 || dims.isEmpty())
    return VideoFrame();

  const int w = dims.w();
  const int h = dims.h();
  ArrayHandle<byte> hdl
    (new ArrayData<byte>(Dims(w * h + 2 * ( ((w+3) / 4) * h ),
                              1),
                         NO_INIT));
  byte* buf = hdl.uniq().dataw();

  const byte* ysrc = pic->data[0];
  const byte* usrc = pic->data[1];
  const byte* vsrc = pic->data[2];

  byte* ydst = buf;
  byte* udst = ydst + w * h;
  byte* vdst = udst + ((w+3)/4) * h;

  const int ydstlen = w,         ysrclen = pic->linesize[0];
  const int udstlen = ((w+3)/4), usrclen = pic->linesize[1];
  const int vdstlen = ((w+3)/4), vsrclen = pic->linesize[2];

  copy_rows(ydst, ydstlen, ysrc, ysrclen, h);
  copy_rows(udst, udstlen, usrc, usrclen, h);
  copy_rows(vdst, vdstlen, vsrc, vsrclen, h);

  return VideoFrame(hdl, dims, VIDFMT_YUV411P, false);
}

// ######################################################################
VideoFormat convertAVPixelFormatToVideoFormat(const PixelFormat fmt)
{
  switch (fmt)
    {
#ifdef HAVE_PIX_FMT_UYVY422
    case PIX_FMT_UYVY422:  return VIDFMT_UYVY;
#endif
    case PIX_FMT_YUV420P:  return VIDFMT_YUV420P;
    case PIX_FMT_YUV422P:  return VIDFMT_YUV422P;
    case PIX_FMT_YUVJ422P: return VIDFMT_YUV422P;
    case PIX_FMT_YUV411P:  return VIDFMT_YUV411P;
    default:
      LFATAL("Oops! I don't know how to convert from "
             "PixelFormat %s (%d) to VideoFormat",
             convertToString(fmt).c_str(), int(fmt));
    }

  /* can't happen */ return VideoFormat(0);
}

// ######################################################################
VideoFrame convertAVFrameToVideoFrame(const AVFrame* pic,
                                      const PixelFormat fmt,
                                      const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  switch (fmt)
    {
#ifdef HAVE_PIX_FMT_UYVY422
    case PIX_FMT_UYVY422:  return uyvy_to_VideoFrame(pic,fmt,dims);
#endif
    case PIX_FMT_YUV420P:  return yuv420p_to_VideoFrame(pic,fmt,dims);
    case PIX_FMT_YUV422P:  return yuv422p_to_VideoFrame(pic,fmt,dims);
    case PIX_FMT_YUVJ422P: return yuv422p_to_VideoFrame(pic,fmt,dims);
    case PIX_FMT_YUV411P:  return yuv411p_to_VideoFrame(pic,fmt,dims);
    default:
      LFATAL("Oops! I don't know how to convert from AVFrame with "
             "PixelFormat %s (%d) to VideoFrame",
             convertToString(fmt).c_str(), int(fmt));
    }

  /* can't happen */ return VideoFrame();
}

// ######################################################################
Image<PixRGB<byte> > convertAVFrameToRGB(const AVFrame* pic,
                                         const PixelFormat fmt,
                                         const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (fmt != PIX_FMT_YUV420P)
    return convertAVFrameToVideoFrame(pic, fmt, dims).toRgb();

  if (pic == 0 || dims.isEmpty())
    return Image<PixRGB<byte> >();

  Image<PixRGB<byte> > result(dims, ZEROS);

  yv12_to_rgb24_c(reinterpret_cast<byte*>(result.getArrayPtr()),
                  dims.w(),
                  pic->data[0],
                  pic->data[1],
                  pic->data[2],
                  pic->linesize[0],
                  pic->linesize[1],
                  dims.w(),
                  dims.h());

  return result;
}

// ######################################################################
bool convertVideoFrameToAVFrame(const VideoFrame& vidframe,
                                const PixelFormat fmt,
                                AVFrame* pic)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // re-initialize the AVFrame
#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
  avcodec_get_frame_defaults(pic);
#else
  {
    AVFrame* tmp = avcodec_alloc_frame();
    memcpy(pic, tmp, sizeof(AVFrame));
    free(tmp);
  }
#endif

  switch (vidframe.getMode())
    {
    case VIDFMT_YUV420P:
      {
        if (fmt != PIX_FMT_YUV420P)
          {
            LDEBUG("Oops! I don't know how to convert from VideoFrame/%s"
                   "to AVFrame with PixelFormat '%d'",
                   convertToString(vidframe.getMode()).c_str(), int(fmt));
            return false;
          }

        const Dims dims = vidframe.getDims();
        byte* buf = const_cast<byte*>(vidframe.getBuffer());

        const int size = dims.w() * dims.h();
        const int size4 = ((dims.w()+1)/2) * (dims.h()/2);

        pic->data[0] = buf;
        pic->data[1] = buf + size;
        pic->data[2] = buf + size + size4;
        pic->linesize[0] = dims.w();
        pic->linesize[1] = (dims.w()+1) / 2;
        pic->linesize[2] = (dims.w()+1) / 2;
      }
      break;

    default:
      {
        LDEBUG("Oops! I don't know how to convert from VideoFormat "
               "'%s' to AVFrame",
               convertToString(vidframe.getMode()).c_str());
        return false;
      }
      break;
    }

  return true;
}

// ######################################################################
std::string convertToString(const PixelFormat fmt)
{
#define PFCASE(X) case X: return std::string(#X); break

  switch (fmt)
    {
      // from ffmpeg/avcodec.h
#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD >= 4753)
      PFCASE(PIX_FMT_NONE);
#endif
      PFCASE(PIX_FMT_YUV420P);   ///< Planar YUV 4:2:0 (1 Cr & Cb sample per 2x2 Y samples)
      PFCASE(PIX_FMT_YUV422);    ///< Packed pixel, Y0 Cb Y1 Cr
      PFCASE(PIX_FMT_RGB24);     ///< Packed pixel, 3 bytes per pixel, RGBRGB...
      PFCASE(PIX_FMT_BGR24);     ///< Packed pixel, 3 bytes per pixel, BGRBGR...
      PFCASE(PIX_FMT_YUV422P);   ///< Planar YUV 4:2:2 (1 Cr & Cb sample per 2x1 Y samples)
      PFCASE(PIX_FMT_YUV444P);   ///< Planar YUV 4:4:4 (1 Cr & Cb sample per 1x1 Y samples)
      PFCASE(PIX_FMT_RGBA32);    ///< Packed pixel, 4 bytes per pixel, BGRABGRA..., stored in cpu endianness
      PFCASE(PIX_FMT_YUV410P);   ///< Planar YUV 4:1:0 (1 Cr & Cb sample per 4x4 Y samples)
      PFCASE(PIX_FMT_YUV411P);   ///< Planar YUV 4:1:1 (1 Cr & Cb sample per 4x1 Y samples)
      PFCASE(PIX_FMT_RGB565);    ///< always stored in cpu endianness
      PFCASE(PIX_FMT_RGB555);    ///< always stored in cpu endianness, most significant bit to 1
      PFCASE(PIX_FMT_GRAY8);
      PFCASE(PIX_FMT_MONOWHITE); ///< 0 is white
      PFCASE(PIX_FMT_MONOBLACK); ///< 0 is black
      PFCASE(PIX_FMT_PAL8);      ///< 8 bit with RGBA palette
      PFCASE(PIX_FMT_YUVJ420P);  ///< Planar YUV 4:2:0 full scale (jpeg)
      PFCASE(PIX_FMT_YUVJ422P);  ///< Planar YUV 4:2:2 full scale (jpeg)
      PFCASE(PIX_FMT_YUVJ444P);  ///< Planar YUV 4:4:4 full scale (jpeg)
      PFCASE(PIX_FMT_XVMC_MPEG2_MC);///< XVideo Motion Acceleration via common packet passing(xvmc_render.h)
      PFCASE(PIX_FMT_XVMC_MPEG2_IDCT);
#ifdef HAVE_PIX_FMT_UYVY422
      PFCASE(PIX_FMT_UYVY422);   ///< Packed pixel, Cb Y0 Cr Y1
#endif
#if defined(LIBAVCODEC_BUILD) && (LIBAVCODEC_BUILD > 4727)
      PFCASE(PIX_FMT_UYVY411);   ///< Packed pixel, Cb Y0 Y1 Cr Y2 Y3
#endif

#if defined(LIBAVUTIL_BUILD) && (LIBAVUTIL_BUILD > 3211264)
      PFCASE(PIX_FMT_BGR32);     ///< Packed RGB 8:8:8, 32bpp, (msb)8A 8B 8G 8R(lsb), in cpu endianness
      PFCASE(PIX_FMT_BGR565);    ///< Packed RGB 5:6:5, 16bpp, (msb)   5B 6G 5R(lsb), in cpu endianness
      PFCASE(PIX_FMT_BGR555);    ///< Packed RGB 5:5:5, 16bpp, (msb)1A 5B 5G 5R(lsb), in cpu endianness most significant bit to 1
      PFCASE(PIX_FMT_BGR8);      ///< Packed RGB 3:3:2,  8bpp, (msb)2B 3G 3R(lsb)
      PFCASE(PIX_FMT_BGR4);      ///< Packed RGB 1:2:1,  4bpp, (msb)1B 2G 1R(lsb)
      PFCASE(PIX_FMT_BGR4_BYTE); ///< Packed RGB 1:2:1,  8bpp, (msb)1B 2G 1R(lsb)
      PFCASE(PIX_FMT_RGB8);      ///< Packed RGB 3:3:2,  8bpp, (msb)2R 3G 3B(lsb)
      PFCASE(PIX_FMT_RGB4);      ///< Packed RGB 1:2:1,  4bpp, (msb)2R 3G 3B(lsb)
      PFCASE(PIX_FMT_RGB4_BYTE); ///< Packed RGB 1:2:1,  8bpp, (msb)2R 3G 3B(lsb)
      PFCASE(PIX_FMT_NV12);      ///< Planar YUV 4:2:0, 12bpp, 1 plane for Y and 1 for UV
      PFCASE(PIX_FMT_NV21);      ///< as above, but U and V bytes are swapped
      PFCASE(PIX_FMT_RGB32_1);   ///< Packed RGB 8:8:8, 32bpp, (msb)8R 8G 8B 8A(lsb), in cpu endianness
      PFCASE(PIX_FMT_BGR32_1);   ///< Packed RGB 8:8:8, 32bpp, (msb)8B 8G 8R 8A(lsb), in cpu endianness
#endif

      PFCASE(PIX_FMT_NB);

    default:
      return std::string("UNKNOWN");
    }

  /* can't happen */ return std::string();
}

#endif // HAVE_FFMPEG_AVCODEC_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_FFMPEGFRAME_C_DEFINED
