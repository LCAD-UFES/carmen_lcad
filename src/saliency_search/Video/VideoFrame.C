/*!@file Video/VideoFrame.C Handle generic video frames in a multitude of formats */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/VideoFrame.C $
// $Id: VideoFrame.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef VIDEO_VIDEOFRAME_C_DEFINED
#define VIDEO_VIDEOFRAME_C_DEFINED

#include "Video/VideoFrame.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/StringUtil.H" // for toLowerCase()
#include "Util/log.H"
#include "Util/sformat.H"
#include "Video/RgbConversion.H"
#include "rutz/mappedfile.h"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"
#include "Raster/DeBayer.H"

#include <fcntl.h>
#include <istream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h> // for fsync()
#include <vector>

// ######################################################################
namespace
{
  void checkDataLength(const size_t actualLength,
                       const VideoFormat mode,
                       const Dims& dims,
                       const bool strict)
  {
    const size_t expectedLength = getFrameSize(mode, dims);

    if (actualLength == expectedLength)
      // OK, the data buffer is the right size:
      return;

    //There is no way to know the jpeg's compression rate until we start
    //decompressing?
    if(mode == VIDFMT_MJPEG) return;

    // OK, the actual buffer size doesn't match the expected buffer
    // size, so let's try to be helpful and guess some possible
    // matching widths and heights that we can suggest to the user

    uint numer = 0, denom = 0;
    getBytesPerPixelForMode(mode, &numer, &denom);
    ASSERT(numer > 0);
    ASSERT(denom > 0);

    const size_t numPix = (actualLength * denom) / numer;

    const uint aspects[][2] =
      {
        // see http://en.wikipedia.org/wiki/Aspect_ratio_(image) for
        // other possible aspect ratios

        { 4, 3 },   // standard TV
        { 16, 9 },  // HDTV
        { 1, 1 },   // square

        // keep this terminal entry last
        { 0, 0 }
      };

    std::string suggestion;

    for (size_t i = 0; aspects[i][0] > 0; ++i)
      {
        const uint aw = aspects[i][0];
        const uint ah = aspects[i][1];

        // w*h = numPix
        // h = (w*ah)/aw

        // w*(w*ah)/aw = numPix
        // w*(w*ah) = numPix*aw
        // w*w = (numPix*aw)/ah
        const uint wsq = (numPix*aw)/ah;

        const uint w = uint(sqrt(double(wsq)));
        const uint h = (w*ah)/aw;

        // LDEBUG("aw=%u, ah=%u, wsq=%u, w=%u, h=%u, w*w=%u, w*h=%u, numPix=%u",
        //        aw, ah, wsq, w, h, w*w, w*h, (uint)numPix);

        if (w*w == wsq && w*h == numPix)
          {
            if (suggestion.length() == 0)
              suggestion = "\n\tpossible matching frame sizes include:";

            suggestion += sformat("\n\t\t%ux%u (%ux%u aspect)",
                                  w, h, aw, ah);
          }
      }

    if (actualLength < expectedLength)
      LFATAL("VideoFrame buffer is too small for %dx%d mode %s\n"
             "\t(expected %" ZU " bytes, got %" ZU " bytes)%s",
             dims.w(), dims.h(), convertToString(mode).c_str(),
             expectedLength, actualLength, suggestion.c_str());

    if (actualLength > expectedLength)
      {
        // NOTE: it's not necessarily a fatal error if actualLength is
        // larger than expectedLength; it might just mean we have some
        // unused space in the data buffer, though it might also mean
        // that the user has specified incorrect yuv dims (or perhaps
        // not specified them at all)
        const std::string msg =
          sformat("VideoFrame buffer is larger than expected for %dx%d mode %s\n"
                  "\t(expected %" ZU " bytes, got %" ZU " bytes; you should\n"
                  "\tdouble-check the width and height settings on your\n"
                  "\tframegrabber, mpeg file, etc. or try --yuv-dims)%s",
                  dims.w(), dims.h(), convertToString(mode).c_str(),
                  expectedLength, actualLength, suggestion.c_str());

        if (strict)
          LFATAL("%s", msg.c_str());
        else
          LDEBUG("%s", msg.c_str());
      }
  }

  template <class T>
  class ArrayHandleStorage : public VideoFrame::Storage
  {
  public:
    ArrayHandleStorage(const ArrayHandle<T>& hdl) : itsHdl(hdl) {}

    virtual ~ArrayHandleStorage() {}

    virtual Storage* clone() const
    {
      // copy-on-write in effect here
      return new ArrayHandleStorage<T>(this->itsHdl);
    }

    ArrayHandle<T> itsHdl;
  };

  template <class T>
  class ImageStorage : public VideoFrame::Storage
  {
  public:
    ImageStorage(const Image<T>& img) : itsImg(img) {}

    virtual ~ImageStorage() {}

    virtual Storage* clone() const
    {
      // copy-on-write in effect here
      return new ImageStorage<T>(this->itsImg);
    }

    Image<T> itsImg;
  };

  class MappedFileStorage : public VideoFrame::Storage
  {
  public:
    MappedFileStorage(rutz::shared_ptr<rutz::mapped_infile> f) : itsFile(f) {}

    virtual ~MappedFileStorage() {}

    virtual Storage* clone() const
    {
      return new MappedFileStorage(this->itsFile);
    }

    rutz::shared_ptr<rutz::mapped_infile> itsFile;
  };
}

// ######################################################################
VideoFrame::Storage::~Storage() {}

// ######################################################################
VideoFrame::VideoFrame()
  :
  itsStorage(),
  itsData(0),
  itsDims(),
  itsMode(VIDFMT_AUTO),
  itsByteSwap(false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
VideoFrame::VideoFrame(const byte* data, const size_t length, const Dims& dims,
                       const VideoFormat mode, const bool byteswap,
                       const bool strictLength)
  :
  itsStorage(), // don't initialize this unless someone requests it
  itsData(data),
  itsDataLength(length),
  itsDims(dims),
  itsMode(mode),
  itsByteSwap(byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  checkDataLength(itsDataLength, itsMode, itsDims, strictLength);
}


// ######################################################################
VideoFrame::VideoFrame(const ArrayHandle<byte>& hdl, const Dims& dims,
                       const VideoFormat mode, const bool byteswap)
  :
  itsStorage(new ArrayHandleStorage<byte>(hdl)),
  itsData(hdl.get().data()),
  itsDataLength(hdl.get().dims().sz()),
  itsDims(dims),
  itsMode(mode),
  itsByteSwap(byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  checkDataLength(itsDataLength, itsMode, itsDims, true);
}

// ######################################################################
VideoFrame::VideoFrame(const Image<byte>& gray)
  :
  itsStorage(new ImageStorage<byte>(gray)),
  itsData(gray.getArrayPtr()),
  itsDataLength(gray.getSize() * sizeof(byte)),
  itsDims(gray.getDims()),
  itsMode(VIDFMT_GREY),
  itsByteSwap(false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  checkDataLength(itsDataLength, itsMode, itsDims, true);
}

// ######################################################################
VideoFrame::VideoFrame(const Image<PixRGB<byte> >& rgb)
  :
  itsStorage(new ImageStorage<PixRGB<byte> >(rgb)),
  itsData(reinterpret_cast<const byte*>(rgb.getArrayPtr())),
  itsDataLength(rgb.getSize() * sizeof(PixRGB<byte>)),
  itsDims(rgb.getDims()),
  itsMode(VIDFMT_RGB24),
  itsByteSwap(false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(sizeof(PixRGB<byte>) == 3);
  checkDataLength(itsDataLength, itsMode, itsDims, true);
}

// ######################################################################
VideoFrame::VideoFrame(const Image<PixVideoYUV<byte> >& yuv)
  :
  itsStorage(new ImageStorage<PixVideoYUV<byte> >(yuv)),
  itsData(reinterpret_cast<const byte*>(yuv.getArrayPtr())),
  itsDataLength(yuv.getSize() * sizeof(PixVideoYUV<byte>)),
  itsDims(yuv.getDims()),
  itsMode(VIDFMT_YUV24),
  itsByteSwap(false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(sizeof(PixVideoYUV<byte>) == 3);
  checkDataLength(itsDataLength, itsMode, itsDims, true);
}

// ######################################################################
VideoFrame::~VideoFrame()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
VideoFrame::VideoFrame(const VideoFrame& that)
  :
  itsStorage(that.itsStorage.get()
             ? that.itsStorage->clone()
             : 0),
  itsData(that.itsData),
  itsDataLength(that.itsDataLength),
  itsDims(that.itsDims),
  itsMode(that.itsMode),
  itsByteSwap(that.itsByteSwap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
VideoFrame& VideoFrame::operator=(const VideoFrame& that)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  itsStorage.reset(that.itsStorage.get()
                   ? that.itsStorage->clone()
                   : 0);
  itsData = that.itsData;
  itsDataLength = that.itsDataLength;
  itsDims = that.itsDims;
  itsMode = that.itsMode;
  itsByteSwap = that.itsByteSwap;

  return *this;
}

// ######################################################################
VideoFrame VideoFrame::fromFile(const char* fname, const Dims& dims,
                                const VideoFormat mode,
                                const bool byteswap,
                                const bool strict)
{
  rutz::shared_ptr<rutz::mapped_infile> fmap(new rutz::mapped_infile(fname));

  VideoFrame result;
  result.itsStorage.reset(new MappedFileStorage(fmap));
  result.itsData = static_cast<const byte*>(fmap->memory());
  result.itsDataLength = fmap->length();
  result.itsDims = dims;
  result.itsMode = mode;
  result.itsByteSwap = byteswap;

  checkDataLength(result.itsDataLength, result.itsMode, result.itsDims,
                  strict);

  return result;
}

// ######################################################################
VideoFrame VideoFrame::fromStream(std::istream& strm,
                                  const Dims& dims,
                                  const VideoFormat mode,
                                  const bool byteswap,
                                  const bool fail_is_fatal)
{
  const size_t sz = getFrameSize(mode, dims);

  ArrayHandle<byte> data
    (new ArrayData<byte>(Dims(sz, 1), NO_INIT));

  strm.read(reinterpret_cast<char*>(data.uniq().dataw()), sz);

  if (strm.fail())
    {
      if (fail_is_fatal)
        LFATAL("error while reading data from input stream");
      else
        return VideoFrame();
    }

  return VideoFrame(data, dims, mode, byteswap);
}

// ######################################################################
VideoFrame VideoFrame::deepCopyOf(const VideoFrame& original)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!original.initialized())
    // original was an invalid (empty) frame, so just return another
    // empty frame as the result:
    return VideoFrame();

  if (original.itsStorage.get() != 0)
    // original already has a persistent copy of the data, so just
    // return a (copy-on-write) copy of that:
    return original;

  // otherwise, make a fresh copy of original's data:

  ArrayHandle<byte> newhdl
    (new ArrayData<byte>(Dims(original.getBufSize(), 1),
                         original.itsData));

  return VideoFrame(newhdl, original.itsDims, original.itsMode,
                    original.itsByteSwap);
}

// ######################################################################
size_t VideoFrame::getBufSize() const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const size_t expectedLength = getFrameSize(itsMode, itsDims);

  if(expectedLength <= 0) return itsDataLength;

  // This is a hard ASSERT, not an LFATAL, because we should have
  // already caught any size mismatch in the VideoFrame constructor,
  // so this ASSERT just serves as a double-check:
  ASSERT(itsDataLength >= expectedLength);

  // NOTE: We return the expectedLength, even though itsDataLength may
  // be larger; that's because external clients should only count on
  // having expectedLength of space, and not more.

  // Also make sure that we always return 0 if our data buffer is null
  // (otherwise callers would try to index into a null pointer)
  ASSERT(itsData != 0 || expectedLength == 0);

  return expectedLength;
}

// ######################################################################
namespace
{
  void bobDeinterlace(const byte* src, const byte* const srcend,
                      byte* dest, byte* const destend,
                      const int height, const int stride,
                      const bool in_bottom_field)
  {
    // NOTE: this deinterlacing code was derived from and/or inspired by
    // code from tvtime (http://tvtime.sourceforge.net/); original
    // copyright notice here:

    /**
     * Copyright (c) 2001, 2002, 2003 Billy Biggs <vektor@dumbterm.net>.
     *
     * This program is free software; you can redistribute it and/or modify
     * it under the terms of the GNU General Public License as published by
     * the Free Software Foundation; either version 2, or (at your option)
     * any later version.
     *
     * This program is distributed in the hope that it will be useful,
     * but WITHOUT ANY WARRANTY; without even the implied warranty of
     * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     * GNU General Public License for more details.
     *
     * You should have received a copy of the GNU General Public License
     * along with this program; if not, write to the Free Software Foundation,
     * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
     */

    ASSERT(height > 0);
    ASSERT(stride > 0);

    // NOTE: on x86 machines with glibc 2.3.6 and g++ 3.4.4, it looks
    // like std::copy is faster than memcpy, so we use that to do the
    // copying here:

#if 1
#  define DOCOPY(dst,src,n) std::copy((src),(src)+(n),(dst))
#else
#  define DOCOPY(dst,src,n) memcpy((dst),(src),(n))
#endif

    if (in_bottom_field)
      {
        src += stride;

        DOCOPY(dest, src, stride);

        dest += stride;
      }

    DOCOPY(dest, src, stride);

    dest += stride;

    const int N = (height / 2) - 1;
    for (int i = 0; i < N; ++i)
      {
        const byte* const src2 = src + (stride*2);

        for (int k = 0; k < stride; ++k)
          dest[k] = (src[k] + src2[k]) / 2;

        dest += stride;

        DOCOPY(dest, src2, stride);

        src += stride*2;
        dest += stride;
      }

    if (!in_bottom_field)
      {
        DOCOPY(dest, src, stride);

        src += stride*2;
        dest += stride;
      }
    else
      src += stride;

    // consistency check: make sure we've done all our counting right:

    if (src != srcend)
      LFATAL("deinterlacing consistency check failed: %d src %p-%p=%d",
             int(in_bottom_field), src, srcend, int(src-srcend));

    if (dest != destend)
      LFATAL("deinterlacing consistency check failed: %d dst %p-%p=%d",
             int(in_bottom_field), dest, destend, int(dest-destend));
  }

#undef DOCOPY

}

// ######################################################################
VideoFrame VideoFrame::makeBobDeinterlaced(int in_bottom_field) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // Allocate a new memory area for storing the deinterlaced
  // frame. (Note, although it may seem expensive to allocate a new
  // buffer for every frame, in practice this is cheap because
  // ArrayData uses a caching allocator (invt_allocate(), see
  // Util/alloc.C) which caches commonly requested chunk sizes.)
  ArrayHandle<byte> hdl
    (new ArrayData<byte>(Dims(getFrameSize(itsMode, itsDims), 1),
                         NO_INIT));

  byte* const dest = hdl.uniq().dataw();
  const byte* const src = this->getBuffer();
  const int w = this->getDims().w();
  const int h = this->getDims().h();
  const int stride = getScanlineWidth(this->getMode(), w);

  if (stride > 0)
    {
      bobDeinterlace(src, src+h*stride, dest, dest+h*stride,
                     h, stride, in_bottom_field);
    }
  else if (this->getMode() == VIDFMT_YUV420P)
    {
      // in this planar format, each plane is interlaced separately,
      // so we just deinterlace each separately as well:

      // y plane:
      bobDeinterlace(src, src+w*h, dest, dest+w*h,
                     h, w, in_bottom_field);

      // we have to do (w+1)/2 instead of just w/2, because if
      // e.g. the y array has 5 pixels, then we want the u and v
      // arrays to have 3 pixels, not 2:
      const int w2 = (w+1)/2;
      const int h2 = (h+1)/2;

      // u plane:
      bobDeinterlace(src + w*h, src + w*h + w2*h2,
                     dest + w*h, dest + w*h + w2*h2,
                     h2, w2, in_bottom_field);
      // v plane:
      bobDeinterlace(src + w*h + w2*h2, src + w*h + 2*w2*h2,
                     dest + w*h + w2*h2, dest + w*h + 2*w2*h2,
                     h2, w2, in_bottom_field);
    }

  return VideoFrame(hdl, this->getDims(), this->getMode(),
                    this->getByteSwap());
}

// ######################################################################
VideoFrame VideoFrame::getFlippedHoriz() const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // this only works in YUV420P:
  if (this->getMode() != VIDFMT_YUV420P)
    LFATAL("Sorry, only VIDFMT_YUV420P is currently supported");

  // prepare a new frame:
  ArrayHandle<byte> hdl
    (new ArrayData<byte>(Dims(getFrameSize(itsMode, itsDims), 1), NO_INIT));

  byte* dest = hdl.uniq().dataw();
  const byte* src = this->getBuffer();
  const int w = this->getDims().w();
  const int h = this->getDims().h();

  const byte *ysrc = src;
  const byte *usrc = ysrc + w * h;
  const byte *vsrc = usrc + ((w+1)/2) * ((h+1)/2);

  // do the luminance:
  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
      *dest++ = ysrc[(j+1)*w - i - 1];

  // do U and V:
  for (int j = 0; j < (h+1)/2; j ++)
    for (int i = 0; i < (w+1)/2; i ++)
      *dest++ = usrc[(j+1)*((w+1)/2) - i - 1];

  for (int j = 0; j < (h+1)/2; j ++)
    for (int i = 0; i < (w+1)/2; i ++)
      *dest++ = vsrc[(j+1)*((w+1)/2) - i - 1];

  return VideoFrame(hdl, this->getDims(), VIDFMT_YUV420P, this->getByteSwap());
}

// ######################################################################
namespace
{
  std::string extensionFor(const Dims& dims, VideoFormat mode)
  {
    return sformat(".%dx%d.%s",
                   dims.w(), dims.h(),
                   toLowerCase(convertToString(mode)).c_str());
  }

  std::string addExtension(const char* fstem,
                           const Dims& dims, VideoFormat mode)
  {
    const std::string ext = extensionFor(dims, mode);

    std::string result(fstem);

    if (result.size() < ext.size() ||
        result.compare(result.size() - ext.size(), result.npos,
                       ext) != 0)
      result += ext;

    ASSERT(result.compare(result.size() - ext.size(), result.npos,
                          ext) == 0);

    return result;
  }
}

// ######################################################################
std::string VideoFrame::diskDumpMmap(const char* fstem, bool flush) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const std::string fname = addExtension(fstem, itsDims, itsMode);

  const int fd =
    open(fname.c_str(),
         O_CREAT | O_RDWR | O_TRUNC,
         S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

  if (fd == -1)
    PLFATAL("Cannot open %s for writing", fname.c_str());

  const size_t siz = this->getBufSize();

  if (siz != 0)
    {
      // seek to the desired file size and write a single byte
      // there, in order to force the file to be the right size
      lseek(fd, siz-1, SEEK_SET);
      int ret = write(fd, "", 1);
      ASSERT(ret > 0);
      lseek(fd, 0, SEEK_SET);

      void* const mem = mmap(0, siz, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
      if (mem == MAP_FAILED)
        { close(fd); PLFATAL("mmap() failed for %s", fname.c_str()); }

      memcpy(mem, this->getBuffer(), siz);

      if (munmap(mem, siz) != 0)
        { close(fd); PLFATAL("munmap() failed for %s", fname.c_str()); }
    }

  if (flush)
    fsync(fd);

  close(fd);

  return fname;
}

// ######################################################################
std::string VideoFrame::diskDumpStdio(const char* fstem, bool flush) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const std::string fname = addExtension(fstem, itsDims, itsMode);

  const int fd =
    open(fname.c_str(),
         O_CREAT | O_RDWR | O_TRUNC,
         S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

  if (fd == -1)
    PLFATAL("Cannot open %s for writing", fname.c_str());

  const size_t siz = this->getBufSize();

  if (write(fd, this->getBuffer(), siz) != ssize_t(siz))
    { close(fd); PLFATAL("write() failed for %s", fname.c_str()); }

  if (flush)
    fsync(fd);

  close(fd);

  return fname;
}

// ######################################################################
Image<PixRGB<byte> > VideoFrame::toRgb() const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsData == 0)
    LFATAL("Eeek! I have no data!");

  // short names so the switch statement below fits nicely :-)

  const byte*  d = itsData;
  const size_t s = this->getBufSize();
  const Dims   m = itsDims;
  const bool   p = itsByteSwap;

  switch (itsMode)
    {
    case VIDFMT_GREY:        return fromMono         (d,s,m);
    case VIDFMT_RGB555:      return fromRGB555       (d,s,m,p);
    case VIDFMT_RGB565:      return fromRGB565       (d,s,m,p);
    case VIDFMT_RGB24:       return fromRGB          (d,s,m,p);
    case VIDFMT_RGB32:       return fromARGB         (d,s,m,p);
    case VIDFMT_YUV24:       return fromVideoYUV24   (d,s,m,p);
    case VIDFMT_UYVY:        return fromVideoYUV422  (d,s,m,p); // same as YUV422
    case VIDFMT_YUYV:        return fromVideoYUV422  (d,s,m,!p); // same as YUV422
    case VIDFMT_YUV444:      return fromVideoYUV444  (d,s,m,p);
    case VIDFMT_YUV422:      return fromVideoYUV422  (d,s,m,p);
    case VIDFMT_YUV411:      return fromVideoYUV411  (d,s,m,p);
    case VIDFMT_YUV420:      return fromVideoYUV420P (d,s,m); // same as YUV420P?
    case VIDFMT_YUV410:      return fromVideoYUV410P (d,s,m); // same as YUV410P?
    case VIDFMT_YUV444P:     return fromVideoYUV444P (d,s,m);
    case VIDFMT_YUV422P:     return fromVideoYUV422P (d,s,m);
    case VIDFMT_YUV411P:     return fromVideoYUV411P (d,s,m);
    case VIDFMT_YUV420P:     return fromVideoYUV420P (d,s,m);
    case VIDFMT_YUV410P:     return fromVideoYUV410P (d,s,m);
    case VIDFMT_HM12:        return fromVideoHM12 (d,s,m);
    case VIDFMT_MJPEG:       return fromVideoMJPEG (d,s,m,p);
    case VIDFMT_BAYER_GB:    return fromBayer(d,s,m, BAYER_GBRG);
    case VIDFMT_BAYER_BG:    return fromBayer(d,s,m, BAYER_BGGR);
    case VIDFMT_BAYER_GR:    return fromBayer(d,s,m, BAYER_GRBG);
    case VIDFMT_BAYER_RG:    return fromBayer(d,s,m, BAYER_RGGB);
    default:
      LFATAL("Sorry, conversion from selected grabmode to RGB not implemented!");
    }
  /* can't happen */ return Image< PixRGB<byte> >();
}

// ######################################################################
Image<PixRGB<uint16> > VideoFrame::toRgbU16() const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsData == 0)
    LFATAL("Eeek! I have no data!");

  // short names so the switch statement below fits nicely :-)

  const uint16*  d = (uint16*)itsData;
  //const unsigned short*  d = (unsigned short*)itsData;
  const size_t s = this->getBufSize();
  const Dims   m = itsDims;

  switch (itsMode)
    {
    case VIDFMT_BAYER_GB12:  return fromBayerU16 (d,s,m, BAYER_GBRG12);
    case VIDFMT_BAYER_BG12:  return fromBayerU16 (d,s,m, BAYER_BGGR12);
    case VIDFMT_BAYER_GR12:  return fromBayerU16 (d,s,m, BAYER_GRBG12);
    case VIDFMT_BAYER_RG12:  return fromBayerU16 (d,s,m, BAYER_RGGB12);
    default:
      LFATAL("Sorry, conversion from selected grabmode to RGB not implemented!");
    }
  /* can't happen */ return Image< PixRGB<uint16> >();
}

// ######################################################################
void VideoFrame::toYuvComponents(Image<byte>& y,
                                 Image<byte>& u,
                                 Image<byte>& v) const
{
  const byte* buf = this->getBuffer();
  const Dims dims = this->getDims();

  switch (this->getMode())
    {
    case VIDFMT_YUV420P:
      y = Image<byte>(buf, dims);
      u = Image<byte>(buf + dims.sz(), (dims + 1) / 2);
      v = Image<byte>(buf + dims.sz() + ((dims + 1) / 2).sz(),
                      (dims + 1) / 2);
      break;

    // FIXME add code here for other YUV VideoFormat modes

    default:
      LFATAL("toYuvComponents() not supported for VideoFormat %s",
             convertToString(this->getMode()).c_str());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_VIDEOFRAME_C_DEFINED
