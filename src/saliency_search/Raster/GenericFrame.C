/*!@file Raster/GenericFrame.C Discriminated union of rgb, grayscale, floating-point, and video-yuv images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/GenericFrame.C $
// $Id: GenericFrame.C 14290 2010-12-01 21:44:03Z itti $
//

#ifndef RASTER_GENERICFRAME_C_DEFINED
#define RASTER_GENERICFRAME_C_DEFINED

#include "Raster/GenericFrame.H"

#include "Image/ColorOps.H"
#include "Image/Normalize.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"

#include <algorithm> // for std::equal()

// ######################################################################
GenericFrame::MetaData::MetaData()
{}

// ######################################################################
GenericFrame::MetaData::~MetaData()
{}

// ######################################################################
GenericFrame::GenericFrame()
  :
  itsNativeType(GenericFrame::NONE),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Image<PixRGB<byte> >& rgbimg)
  :
  itsNativeType(rgbimg.initialized()
                ? GenericFrame::RGB_U8
                : GenericFrame::NONE),
  itsRgbU8(rgbimg),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Image<PixRGB<byte> >& rgbimg, const Image<uint16>& dimg)
  :
  itsNativeType((rgbimg.initialized() && dimg.initialized())
                ? GenericFrame::RGBD
                : GenericFrame::NONE),
  itsRgbU8(rgbimg),
  itsGrayU16(dimg),
  itsFloatFlags(0)
{
  ASSERT(rgbimg.isSameSize(dimg));
}

// ######################################################################
GenericFrame::GenericFrame(const Image<PixRGB<uint16> >& rgbimg)
  :
  itsNativeType(rgbimg.initialized()
                ? GenericFrame::RGB_U16
                : GenericFrame::NONE),
  itsRgbU8(rgbimg),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Layout<PixRGB<byte> >& rgbimg)
  :
  itsNativeType(rgbimg.initialized()
                ? GenericFrame::RGB_U8
                : GenericFrame::NONE),
  itsRgbU8(rgbimg),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Image<PixRGB<float> >& rgbimg,
                           const int flags)
  :
  itsNativeType(rgbimg.initialized()
                ? GenericFrame::RGB_F32
                : GenericFrame::NONE),
  itsRgbF32(rgbimg),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Image<byte>& grayimg)
  :
  itsNativeType(grayimg.initialized()
                ? GenericFrame::GRAY_U8
                : GenericFrame::NONE),
  itsGrayU8(grayimg),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Image<uint16>& grayimg)
  :
  itsNativeType(grayimg.initialized()
                ? GenericFrame::GRAY_U16
                : GenericFrame::NONE),
  itsGrayU16(grayimg),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Layout<byte>& grayimg)
  :
  itsNativeType(grayimg.initialized()
                ? GenericFrame::GRAY_U8
                : GenericFrame::NONE),
  itsGrayU8(grayimg),
  itsFloatFlags(0)
{}

// ######################################################################
GenericFrame::GenericFrame(const Image<float>& floatimg,
                           const int flags)
  :
  itsNativeType(floatimg.initialized()
                ? GenericFrame::GRAY_F32
                : GenericFrame::NONE),
  itsGrayF32(floatimg),
  itsFloatFlags(flags)
{}

// ######################################################################
GenericFrame::GenericFrame(const VideoFrame& vidframe)
  :
  itsNativeType(vidframe.getDims().isNonEmpty()
                ? GenericFrame::VIDEO
                : GenericFrame::NONE),
  itsVideo(vidframe)
{}

// ######################################################################
bool GenericFrame::initialized() const
{
  if (itsNativeType != NONE)
    {
      ASSERT(this->getDims().isNonEmpty());
      return true;
    }

  return false;
}

// ######################################################################
GenericFrame GenericFrame::deepCopyOf(const GenericFrame& f)
{
  GenericFrame result;
  result.itsNativeType = f.itsNativeType;
  result.itsRgbU8 = f.itsRgbU8;
  result.itsRgbU16 = f.itsRgbU16;
  result.itsRgbF32 = f.itsRgbF32;
  result.itsGrayU8 = f.itsGrayU8;
  result.itsGrayU16 = f.itsGrayU16;
  result.itsGrayF32 = f.itsGrayF32;
  result.itsVideo = VideoFrame::deepCopyOf(f.itsVideo);
  result.itsFloatFlags = f.itsFloatFlags;
  return result;
}

// ######################################################################
GenericFrameSpec GenericFrame::frameSpec() const
{
  GenericFrameSpec result;
  result.nativeType = this->itsNativeType;
  result.videoFormat = this->itsVideo.getMode();
  result.videoByteSwap = this->itsVideo.getByteSwap();
  result.dims = this->getDims();
  result.floatFlags = this->itsFloatFlags;
  return result;
}

// ######################################################################
std::string GenericFrame::nativeTypeName() const
{
  return this->frameSpec().nativeTypeName();
}

// ######################################################################
Dims GenericFrame::getDims() const
{
  switch (itsNativeType)
    {
    case NONE:     return Dims();
    case RGB_U8:   return itsRgbU8.getDims();
    case RGB_U16:  return itsRgbU16.getDims();
    case RGB_F32:  return itsRgbF32.getDims();
    case GRAY_U8:  return itsGrayU8.getDims();
    case GRAY_U16: return itsGrayU16.getDims();
    case GRAY_F32: return itsGrayF32.getDims();
    case VIDEO:    return itsVideo.getDims();
    case RGBD:     return itsRgbU8.getDims();
    }

  ASSERT(0); /* can't happen */ return Dims();
}

// ######################################################################
void GenericFrame::setFloatFlags(int v)
{
  if (itsFloatFlags != v)
    {
      itsFloatFlags = v;

      // now, if our native type is floating-point, then we need to
      // kill any cached byte versions since they will need to be
      // recomputed now that our float-conversion flags have changed
      if (itsNativeType == RGB_F32 || itsNativeType == GRAY_F32)
        {
          itsRgbU8 = Layout<PixRGB<byte> >();
          itsGrayU8 = Layout<byte>();
          itsVideo = VideoFrame();
        }
    }
}

// ######################################################################
const Layout<PixRGB<byte> >& GenericFrame::asRgbU8Layout() const
{
  if (!itsRgbU8.initialized())
    {
      switch (itsNativeType)
        {
        case NONE:
          // leave itsRgbU8 uninitialized
          break;

        case RGB_U8:
        case RGBD:
          break;

        case RGB_U16:
          break;

        case RGB_F32:
          // this will possibly do 0..255 normalization (if
          // flags&FLOAT_NORM_0_255), possibly add a text indication
          // of the original scale (if flags&FLOAT_NORM_WITH_SCALE),
          // depending on the user's request:
          itsRgbU8 = Layout<PixRGB<byte> >(Image<PixRGB<byte> >(normalizeFloatRgb(itsRgbF32, itsFloatFlags)));
          break;

        case GRAY_U8:
          itsRgbU8 = Layout<PixRGB<byte> >(Image<PixRGB<byte> >(itsGrayU8.render()));
          break;

        case GRAY_U16:
          break;

        case GRAY_F32:
          // this will possibly do 0..255 normalization (if
          // flags&FLOAT_NORM_0_255), possibly add a text indication
          // of the original scale (if flags&FLOAT_NORM_WITH_SCALE),
          // depending on the user's request:
          itsRgbU8 = Layout<PixRGB<byte> >(Image<PixRGB<byte> >(normalizeFloat(itsGrayF32, itsFloatFlags)));
          break;

        case VIDEO:
          itsRgbU8 = Layout<PixRGB<byte> >(itsVideo.toRgb());
          break;
        }
    }

  return itsRgbU8;
}

// ######################################################################
Image<PixRGB<uint16> > GenericFrame::asRgbU16() const
{
  if (!itsRgbU16.initialized())
    {
      switch (itsNativeType)
        {
        case NONE:
          break;
        case RGB_U8:
        case RGBD:
          break;
        case RGB_U16:
          break;
        case RGB_F32:
          break;
        case GRAY_U8:
          break;
        case GRAY_F32:
          break;
        case GRAY_U16:
          break;
        case VIDEO:
          itsRgbU16 = itsVideo.toRgbU16();
        }
    }
  return itsRgbU16;
}

// ######################################################################
Image<PixRGB<float> > GenericFrame::asRgbF32() const
{
  if (!itsRgbF32.initialized())
    {
      switch (itsNativeType)
        {
        case NONE:
          // leave itsRgbF32 uninitialized
          break;

        case RGB_U8:
        case RGBD:
          itsRgbF32 = itsRgbU8.render();
          break;

        case RGB_U16:
          break;

        case RGB_F32:
          break;

        case GRAY_U8:
          itsRgbF32 = luminance(this->asGrayF32());
          break;

        case GRAY_U16:
          break;

        case GRAY_F32:
          itsRgbF32 = itsGrayF32;
          break;

        case VIDEO:
          itsRgbF32 = this->asRgbU8();
          break;
        }
    }

  return itsRgbF32;
}

// ######################################################################
const Layout<byte>& GenericFrame::asGrayU8Layout() const
{
  if (!itsGrayU8.initialized())
    {
      switch (itsNativeType)
        {
        case NONE:
          // leave itsGrayU8 uninitialized
          break;

        case RGB_U8:
          itsGrayU8 = Layout<byte>(luminance(itsRgbU8.render()));
          break;

        case RGB_U16:
        case RGBD:
          break;

        case RGB_F32:
          itsGrayU8 = Layout<byte>(luminance(this->asRgbU8()));
          break;

        case GRAY_U8:
          break;

        case GRAY_U16:
          break;

        case GRAY_F32:
          // this will possibly do 0..255 normalization (if
          // flags&FLOAT_NORM_0_255), possibly add a text indication
          // of the original scale (if flags&FLOAT_NORM_WITH_SCALE),
          // depending on the user's request:
          itsGrayU8 = Layout<byte>(Image<byte>(normalizeFloat(itsGrayF32, itsFloatFlags)));
          break;

        case VIDEO:
          if(itsVideo.getMode() == VIDFMT_BAYER_GB ||
             itsVideo.getMode() == VIDFMT_BAYER_BG ||
             itsVideo.getMode() == VIDFMT_BAYER_GR ||
             itsVideo.getMode() == VIDFMT_BAYER_RG)
            {
              itsGrayU8 = Image<byte>(itsVideo.getBuffer(),itsVideo.getDims());
            }
          else
            {
              // FIXME: make a VideoFrame::asGray() function to return
              // just the Y component from a YUV image
              itsGrayU8 = Layout<byte>(luminance(this->asRgbU8()));
            }
          break;
        }
    }

  return itsGrayU8;
}


// ######################################################################
const Layout<byte>& GenericFrame::asGrayU8NTSCLayout() const
{
  if (!itsGrayU8.initialized())
    {
      switch (itsNativeType)
        {
        case NONE:
          // leave itsGrayU8 uninitialized
          break;

        case RGB_U8:
          itsGrayU8 = Layout<byte>(luminanceNTSC(itsRgbU8.render()));
          break;

        case RGB_U16:
        case RGBD:
          break;

        case RGB_F32:
          itsGrayU8 = Layout<byte>(luminanceNTSC(this->asRgbU8()));
          break;

        case GRAY_U8:
          break;

        case GRAY_U16:
          break;

        case GRAY_F32:
          // this will possibly do 0..255 normalization (if
          // flags&FLOAT_NORM_0_255), possibly add a text indication
          // of the original scale (if flags&FLOAT_NORM_WITH_SCALE),
          // depending on the user's request:
          itsGrayU8 = Layout<byte>(Image<byte>(normalizeFloat(itsGrayF32, itsFloatFlags)));
          break;

        case VIDEO:
          if(itsVideo.getMode() == VIDFMT_BAYER_GB ||
             itsVideo.getMode() == VIDFMT_BAYER_BG ||
             itsVideo.getMode() == VIDFMT_BAYER_GR ||
             itsVideo.getMode() == VIDFMT_BAYER_RG)
            {
              itsGrayU8 = Image<byte>(itsVideo.getBuffer(),itsVideo.getDims());
            }
          else
            {
              // FIXME: make a VideoFrame::asGray() function to return
              // just the Y component from a YUV image
              itsGrayU8 = Layout<byte>(luminanceNTSC(this->asRgbU8()));
            }
          break;
        }
    }

  return itsGrayU8;
}


// ######################################################################
Image<uint16> GenericFrame::asGrayU16() const
{
  if (!itsGrayU16.initialized())
    {
      switch(itsNativeType)
        {
        case NONE:
          break;
        case RGB_U8:
          break;
        case RGB_U16:
          break;
        case RGB_F32:
          break;
        case GRAY_U8:
          break;
        case GRAY_U16:
        case RGBD:
          break;
        case GRAY_F32:
          break;
        case VIDEO:
          if(itsVideo.getMode() == VIDFMT_BAYER_GB12 ||
             itsVideo.getMode() == VIDFMT_BAYER_BG12 ||
             itsVideo.getMode() == VIDFMT_BAYER_GR12 ||
             itsVideo.getMode() == VIDFMT_BAYER_RG12)
            {
              itsGrayU16 = Image<uint16>((uint16*)itsVideo.getBuffer(),
                                         itsVideo.getDims());
            }
          break;
        }
    }
  return itsGrayU16;
}

// ######################################################################
Image<float> GenericFrame::asGrayF32() const
{
  if (!itsGrayF32.initialized())
    {
      switch (itsNativeType)
        {
        case NONE:
          break;

        case RGB_U8:
          itsGrayF32 = this->asGray();
          break;

        case RGB_U16:
          break;

        case RGB_F32:
          itsGrayF32 = luminance(itsRgbF32);
          break;

        case GRAY_U8:
          itsGrayF32 = itsGrayU8.render();
          break;

        case GRAY_U16:
        case RGBD:
          break;

        case GRAY_F32:
          break;

        case VIDEO:
          itsGrayF32 = this->asGray();
          break;
        }
    }

  return itsGrayF32;
}

// ######################################################################
VideoFrame GenericFrame::asVideo() const
{
  if (!itsVideo.initialized())
    {
      switch (itsNativeType)
        {
        case NONE:
          break;

        case RGB_U8:
        case RGBD:
          itsVideo = VideoFrame(itsRgbU8.render());
          break;

        case RGB_U16:
          break;

        case RGB_F32:
          itsVideo = VideoFrame(this->asRgbU8());
          break;

        case GRAY_U8:
          itsVideo = VideoFrame(itsGrayU8.render());
          break;

        case GRAY_U16:
          break;

        case GRAY_F32:
          // force a conversion from float to gray, then to VideoFrame:
          itsVideo = VideoFrame(this->asGray());
          break;

        case VIDEO:
          break;
        }
    }

  return itsVideo;
}

// ######################################################################
bool GenericFrame::hasMetaData(const std::string& tag) const
{
  if (itsMetaDataMap.get() == 0)
    return false;

  return itsMetaDataMap->find(tag) != itsMetaDataMap->end();
}
// ######################################################################
rutz::shared_ptr<GenericFrame::MetaData> GenericFrame::getMetaData(const std::string& tag)
{
  if (itsMetaDataMap.get() != 0)
  {
    MetaDataMap::iterator itr = itsMetaDataMap->find(tag);

    if (itr != itsMetaDataMap->end())
      return (*itr).second;
  }

  LINFO("Oops: No meta-data with tag '%s'", tag.c_str());
  return rutz::shared_ptr<GenericFrame::MetaData>(); 

}

// ######################################################################
void GenericFrame::addMetaData(const std::string& tag,
    rutz::shared_ptr<MetaData> d)
{
  if (itsMetaDataMap.get() == 0)
    itsMetaDataMap.reset(new MetaDataMap);

  itsMetaDataMap->insert(MetaDataMap::value_type(tag, d));
}

// ######################################################################
GenericFrameSpec::GenericFrameSpec()
  :
  nativeType(GenericFrame::NONE),
  videoFormat(VIDFMT_AUTO),
  videoByteSwap(false),
  dims(),
  floatFlags(0),
  frameRate(0)
{}

// ######################################################################
bool GenericFrameSpec::operator==(const GenericFrameSpec& that) const
{
  return
    this->nativeType == that.nativeType
    && (this->nativeType != GenericFrame::VIDEO
        // consider the video format only if it's actually a
        // raw-video frame
        || (this->videoFormat == that.videoFormat
            && this->videoByteSwap == that.videoByteSwap))
    && this->dims == that.dims
    && ((this->nativeType != GenericFrame::RGB_F32
         && this->nativeType != GenericFrame::GRAY_F32)
        // consider the float flags only if it's actually a
        // floating-point format
        || this->floatFlags == that.floatFlags);
}

// ######################################################################
std::string GenericFrameSpec::getDescription() const
{
  return sformat("%dx%d %s",
                 this->dims.w(), this->dims.h(),
                 this->nativeTypeName().c_str());
}

// ######################################################################
std::string GenericFrameSpec::nativeTypeName() const
{
  switch (this->nativeType)
    {
    case GenericFrame::NONE:     return "(empty)";
    case GenericFrame::RGB_U8:   return "Image<PixRGB<byte>>";
    case GenericFrame::RGB_U16:  return "Image<PixRGB<uint16>>";
    case GenericFrame::RGB_F32:  return "Image<PixRGB<float>>";
    case GenericFrame::GRAY_U8:  return "Image<byte>";
    case GenericFrame::GRAY_U16: return "Image<uint16>";
    case GenericFrame::GRAY_F32: return "Image<float>";
    case GenericFrame::VIDEO:    return sformat("VideoFrame (%s%s)",
                                                convertToString(this->videoFormat).c_str(),
                                                this->videoByteSwap
                                                ? "+byteswap" : "");
    case GenericFrame::RGBD: return "Image<PixRGB<byte>>+Image<uint16>";
    }
  ASSERT(0); /* can't happen */; return 0;
}

// ######################################################################
VideoFormat GenericFrameSpec::getActualVideoFormat() const
{
  switch (this->nativeType)
    {
    case GenericFrame::NONE:     return VIDFMT_GREY;
    case GenericFrame::RGB_U8:   return VIDFMT_RGB24;
    case GenericFrame::RGBD:     return VIDFMT_RGB24;
    case GenericFrame::RGB_F32:  return VIDFMT_RGB24;
    case GenericFrame::GRAY_U8:  return VIDFMT_GREY;
    case GenericFrame::GRAY_F32: return VIDFMT_GREY;
    case GenericFrame::VIDEO:    return this->videoFormat;

      //undefined byte for the 16 bits RGB or gray scale image
    case GenericFrame::RGB_U16:  return VIDFMT_AUTO;
    case GenericFrame::GRAY_U16: return VIDFMT_AUTO;
    }
  ASSERT(0); /* can't happen */; return VIDFMT_AUTO;
}

// ######################################################################
bool operator==(const GenericFrame& f1, const GenericFrame& f2)
{
  if (f1.nativeType() == f2.nativeType())
    {
      switch (f1.nativeType())
        {
        case GenericFrame::NONE: return true;
        case GenericFrame::RGB_U8: return f1.asRgbU8() == f2.asRgbU8();
        case GenericFrame::RGBD: return ((f1.asRgbU8() == f2.asRgbU8()) && (f1.asGrayU16() == f2.asGrayU16()));
        case GenericFrame::RGB_F32: return f1.asRgbF32() == f2.asRgbF32();
        case GenericFrame::GRAY_U8: return f1.asGrayU8() == f2.asGrayU8();
        case GenericFrame::GRAY_F32: return f1.asGrayF32() == f2.asGrayF32();
        case GenericFrame::VIDEO:
          {
            const VideoFrame v1 = f1.asVideo();
            const VideoFrame v2 = f2.asVideo();

            if (v1.getMode() == v2.getMode())
              return std::equal(v1.getBuffer(),
                                v1.getBuffer() + v1.getBufSize(),
                                v2.getBuffer());
            else
              return v1.toRgb() == v2.toRgb();
          }
        case GenericFrame::RGB_U16:        return f1.asRgbU16() == f2.asRgbU16();
        case GenericFrame::GRAY_U16:       return f1.asGrayU16() == f2.asGrayU16();
        }
    }

  return f1.asRgbF32() == f2.asRgbF32();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_GENERICFRAME_C_DEFINED
