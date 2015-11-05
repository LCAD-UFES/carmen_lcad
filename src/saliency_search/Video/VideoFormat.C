/*!@file Video/VideoFormat.C definitions of possible image grabbing modes */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/VideoFormat.C $
// $Id: VideoFormat.C 15147 2012-02-06 23:22:31Z kai $
//

#include "Video/VideoFormat.H"

#include "Image/Dims.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H" // for toUpperCase()
#include "Util/log.H"

// ######################################################################
std::string convertToString(const VideoFormat val)
{
  switch (val)
    {
    case VIDFMT_GREY:         return "GREY";
    case VIDFMT_RAW:          return "RAW";
    case VIDFMT_RGB555:       return "RGB555";
    case VIDFMT_RGB565:       return "RGB565";
    case VIDFMT_RGB24:        return "RGB24";
    case VIDFMT_RGB32:        return "RGB32";
    case VIDFMT_YUV24:        return "YUV24";
    case VIDFMT_YUYV:         return "YUYV";
    case VIDFMT_UYVY:         return "UYVY";
    case VIDFMT_YUV444:       return "YUV444";
    case VIDFMT_YUV422:       return "YUV422";
    case VIDFMT_YUV411:       return "YUV411";
    case VIDFMT_YUV420:       return "YUV420";
    case VIDFMT_YUV410:       return "YUV410";
    case VIDFMT_YUV444P:      return "YUV444P";
    case VIDFMT_YUV422P:      return "YUV422P";
    case VIDFMT_YUV411P:      return "YUV411P";
    case VIDFMT_YUV420P:      return "YUV420P";
    case VIDFMT_YUV410P:      return "YUV410P";
    case VIDFMT_HM12:               return "HM12";
    case VIDFMT_BAYER_GB:     return "BAYER_GB";
    case VIDFMT_BAYER_GR:     return "BAYER_GR";
    case VIDFMT_BAYER_RG:     return "BAYER_RG";
    case VIDFMT_BAYER_BG:     return "BAYER_BG";
    case VIDFMT_BAYER_GB12:   return "BAYER_GB12";
    case VIDFMT_BAYER_GR12:   return "BAYER_GR12";
    case VIDFMT_BAYER_RG12:   return "BAYER_RG12";
    case VIDFMT_BAYER_BG12:   return "BAYER_BG12";
    case VIDFMT_MJPEG:        return "MJPEG";
    case VIDFMT_AUTO:    return "Auto";
    }

  LFATAL("Invalid VideoFormat value %d (valid range is 0-%d inclusive)",
         int(val), int(VIDFMT_AUTO));

  /* can't happen */ return std::string();
}

// ######################################################################
void convertFromString(const std::string& orig, VideoFormat& val)
{
  const std::string upper = toUpperCase(orig);

  for (int i = 0; i <= VIDFMT_AUTO; ++i)
    if (upper.compare(toUpperCase(convertToString(VideoFormat(i)))) == 0)
      { val = VideoFormat(i); return; }

  conversion_error::raise<VideoFormat>(orig);
}

// ######################################################################
bool isSimplePackedMode(const VideoFormat vidformat)
{
  switch (vidformat)
    {
    case VIDFMT_GREY:    return true;
    case VIDFMT_RAW:                   return false; // unknown format?
    case VIDFMT_RGB555:                return false; // not a simple bytewise format, so not easily deinterlaceable
    case VIDFMT_RGB565:                return false; // not a simple bytewise format, so not easily deinterlaceable
    case VIDFMT_RGB24:   return true;
    case VIDFMT_RGB32:   return true;
    case VIDFMT_YUV24:   return true;
    case VIDFMT_YUYV:    return true;
    case VIDFMT_UYVY:    return true;
    case VIDFMT_YUV444:  return true;
    case VIDFMT_YUV422:  return true;
    case VIDFMT_YUV411:                return false;
    case VIDFMT_YUV420:                return false;
    case VIDFMT_YUV410:                return false;
    case VIDFMT_YUV444P:               return false;
    case VIDFMT_YUV422P:               return false;
    case VIDFMT_YUV411P:               return false;
    case VIDFMT_YUV420P:               return false;
    case VIDFMT_YUV410P:               return false;
    case VIDFMT_HM12:                  return false;
    case VIDFMT_BAYER_GB:              return false;
    case VIDFMT_BAYER_GR:              return false;
    case VIDFMT_BAYER_RG:              return false;
    case VIDFMT_BAYER_BG:              return false;
    case VIDFMT_BAYER_GB12:            return false;
    case VIDFMT_BAYER_GR12:            return false;
    case VIDFMT_BAYER_RG12:            return false;
    case VIDFMT_BAYER_BG12:            return false;
    case VIDFMT_MJPEG:                 return true;

    case VIDFMT_AUTO:                  return false;
    }

  LFATAL("invalid VideoFormat %d", int(vidformat));
  /* can't happen */ return -1;
}

// ######################################################################
int getScanlineWidth(const VideoFormat vidformat,
                     const int imgwidth)
{
  // we return -1 if vidformat is not a packed pixel format:

  switch (vidformat)
    {
    case VIDFMT_GREY:    return imgwidth;
    case VIDFMT_RAW:                           return -1; // unknown format?
    case VIDFMT_RGB555:                        return -1; // not a simple bytewise format, so not easily deinterlaceable
    case VIDFMT_RGB565:                        return -1; // not a simple bytewise format, so not easily deinterlaceable
    case VIDFMT_RGB24:   return imgwidth * 3;
    case VIDFMT_RGB32:   return imgwidth * 4;
    case VIDFMT_YUV24:   return imgwidth * 3;
    case VIDFMT_YUYV:    return imgwidth * 2;
    case VIDFMT_UYVY:    return imgwidth * 2;
    case VIDFMT_YUV444:  return imgwidth * 3;
    case VIDFMT_YUV422:  return imgwidth * 2;
    case VIDFMT_YUV411:                        return -1;
    case VIDFMT_YUV420:                        return -1;
    case VIDFMT_YUV410:                        return -1;
    case VIDFMT_YUV444P:                       return -1;
    case VIDFMT_YUV422P:                       return -1;
    case VIDFMT_YUV411P:                       return -1;
    case VIDFMT_YUV420P:                       return -1;
    case VIDFMT_YUV410P:                       return -1;
    case VIDFMT_HM12:                          return -1;
    case VIDFMT_BAYER_GB:                      return imgwidth;
    case VIDFMT_BAYER_GR:                      return imgwidth;
    case VIDFMT_BAYER_RG:                      return imgwidth;
    case VIDFMT_BAYER_BG:                      return imgwidth;
    case VIDFMT_BAYER_GB12:                    return imgwidth;
    case VIDFMT_BAYER_GR12:                    return imgwidth;
    case VIDFMT_BAYER_RG12:                    return imgwidth;
    case VIDFMT_BAYER_BG12:                    return imgwidth;
    case VIDFMT_MJPEG:                         return imgwidth;
    case VIDFMT_AUTO:                          return -1;
    }

  LFATAL("invalid VideoFormat %d", int(vidformat));
  /* can't happen */ return -1;
}

// ######################################################################
void getBytesPerPixelForMode(const VideoFormat vidformat,
                             unsigned int* numer,
                             unsigned int* denom)
{
  ASSERT(numer != 0);
  ASSERT(denom != 0);

  switch (vidformat)
    {
    case VIDFMT_GREY:        *numer=1; *denom=1; break;
    case VIDFMT_RAW:         *numer=4; *denom=1; break; // FIXME is this right?
    case VIDFMT_RGB555:      *numer=2; *denom=1; break;
    case VIDFMT_RGB565:      *numer=2; *denom=1; break;
    case VIDFMT_RGB24:       *numer=3; *denom=1; break;
    case VIDFMT_RGB32:       *numer=4; *denom=1; break;
    case VIDFMT_YUV24:       *numer=3; *denom=1; break;
    case VIDFMT_YUYV:        *numer=2; *denom=1; break; // (Y=1,  U=1/2,  V=1/2)
    case VIDFMT_UYVY:        *numer=2; *denom=1; break; // (Y=1,  U=1/2,  V=1/2)
    case VIDFMT_YUV444:      *numer=3; *denom=1; break; // (Y=1,  U=1,    V=1)
    case VIDFMT_YUV422:      *numer=2; *denom=1; break; // (Y=1,  U=1/2,  V=1/2)
    case VIDFMT_YUV411:      *numer=3; *denom=2; break; // (Y=1,  U=1/4,  V=1/4)
    case VIDFMT_YUV420:      *numer=3; *denom=2; break; // (Y=1,  U=1/4,  V=1/4)
    case VIDFMT_YUV410:      *numer=9; *denom=8; break; // (Y=1,  U=1/16, V=1/16)
    case VIDFMT_YUV444P:     *numer=3; *denom=1; break; // (Y=1,  U=1,    V=1)
    case VIDFMT_YUV422P:     *numer=2; *denom=1; break; // (Y=1,  U=1/2,  V=1/2)
    case VIDFMT_YUV411P:     *numer=3; *denom=2; break; // (Y=1,  U=1/4,  V=1/4)
    case VIDFMT_YUV420P:     *numer=3; *denom=2; break; // (Y=1,  U=1/4,  V=1/4)
    case VIDFMT_YUV410P:     *numer=9; *denom=8; break; // (Y=1,  U=1/16, V=1/16)
    case VIDFMT_HM12:        *numer=3; *denom=2; break; // (Y=1,  U=1/4, V=1/4)
    case VIDFMT_BAYER_GB:    *numer=1; *denom=1; break;
    case VIDFMT_BAYER_GR:    *numer=1; *denom=1; break;
    case VIDFMT_BAYER_RG:    *numer=1; *denom=1; break;
    case VIDFMT_BAYER_BG:    *numer=1; *denom=1; break;
    case VIDFMT_BAYER_GB12:  *numer=2; *denom=1; break;
    case VIDFMT_BAYER_GR12:  *numer=2; *denom=1; break;
    case VIDFMT_BAYER_RG12:  *numer=2; *denom=1; break;
    case VIDFMT_BAYER_BG12:  *numer=2; *denom=1; break;
    case VIDFMT_MJPEG:       *numer=2; *denom=1; break;

    case VIDFMT_AUTO:    LFATAL("can't determine bytes per pixel for VIDFMT_AUTO");
    default:             LFATAL("invalid VideoFormat %d", int(vidformat));
    }
}

// ######################################################################
unsigned int getFrameSize(const VideoFormat vidformat,
                          const Dims& imgdims)
{
  const unsigned int sz = imgdims.sz();

  unsigned int numer=0, denom=0;

  getBytesPerPixelForMode(vidformat, &numer, &denom);
  ASSERT(numer > 0);
  ASSERT(denom > 0);

  return (sz * numer) / denom;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
