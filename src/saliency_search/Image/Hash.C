/*!@file Image/Hash.C hash/message-digest functions for Image objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Hash.C $
// $Id: Hash.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef IMAGE_HASH_C_DEFINED
#define IMAGE_HASH_C_DEFINED

#include "Image/Hash.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/md5.H"
#include "Util/sha1.H"
#include "Util/sha2.H"
#include "rutz/compat_snprintf.h"
#include "rutz/trace.h"

#include <cctype>

namespace
{
  template <class T>
  Digest<16> md5helper(const Image<T>* img, const void* extra, size_t nextra)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);
    md5_context ctx;
    md5_starts(&ctx);
    md5_update(&ctx,
               reinterpret_cast<const byte*>(img->getArrayPtr()),
               img->getSize() * sizeof(T));
    if (extra != 0 && nextra > 0)
      md5_update(&ctx, static_cast<const byte*>(extra), nextra);
    Digest<16> result;
    md5_finish(&ctx, result.buffer);
    return result;
  }

  template <class T>
  Digest<20> sha1helper(const Image<T>* img, const void* extra, size_t nextra)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);
    sha1_context ctx;
    sha1_starts(&ctx);
    sha1_update(&ctx,
                reinterpret_cast<const byte*>(img->getArrayPtr()),
                img->getSize() * sizeof(T));
    if (extra != 0 && nextra > 0)
      sha1_update(&ctx, static_cast<const byte*>(extra), nextra);
    Digest<20> result;
    sha1_finish(&ctx, result.buffer);
    return result;
  }

  template <class T>
  Digest<32> sha256helper(const Image<T>* img, const void* extra, size_t nextra)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);
    sha256_context ctx;
    sha256_starts(&ctx);
    sha256_update(&ctx,
                  reinterpret_cast<const byte*>(img->getArrayPtr()),
                  img->getSize() * sizeof(T));
    if (extra != 0 && nextra > 0)
      sha256_update(&ctx, static_cast<const byte*>(extra), nextra);
    Digest<32> result;
    sha256_finish(&ctx, result.buffer);
    return result;
  }
}

std::string digestFormatHelper(const byte* buf, unsigned int n)
{
  char fmt[n*2 + 1];
  for (unsigned int i = 0; i < n; ++i)
    {
      // we pass 3 instead of 2 for the buffer length because
      // snprintf() needs 2 characters for the 2 hexadecimal digits,
      // plus 1 character for the null terminator
      snprintf(&fmt[i*2], 3, "%02x", int(buf[i]));
    }
  return std::string(&fmt[0]);
}

void digestScanHelper(const std::string& s, byte* buf, unsigned int n)
{
  if (s.length() != n*2)
    LFATAL("expected string of length %u in order to generate a "
           "Digest<%u>, but got '%s' of length %" ZU ,
           n*2, n, s.c_str(), s.length());

  for (unsigned int i = 0; i < n; ++i)
    {
      buf[i] = 0;

      for (unsigned int j = 0; j < 2; ++j)
        {
          buf[i] *= 16;

          switch (tolower(s[i*2+j]))
            {
            case '0': buf[i] += 0;  break;
            case '1': buf[i] += 1;  break;
            case '2': buf[i] += 2;  break;
            case '3': buf[i] += 3;  break;
            case '4': buf[i] += 4;  break;
            case '5': buf[i] += 5;  break;
            case '6': buf[i] += 6;  break;
            case '7': buf[i] += 7;  break;
            case '8': buf[i] += 8;  break;
            case '9': buf[i] += 9;  break;
            case 'a': buf[i] += 10; break;
            case 'b': buf[i] += 11; break;
            case 'c': buf[i] += 12; break;
            case 'd': buf[i] += 13; break;
            case 'e': buf[i] += 14; break;
            case 'f': buf[i] += 15; break;
            default:
              LFATAL("invalid hex digit '%c' at position %u in hash string '%s'",
                     s[i*2+j], i*2+j, s.c_str());
            }
        }
    }
}


Digest<16> md5byte(const Image<byte>* img, const void* extra, size_t nextra)
{ return md5helper<byte>(img, extra, nextra); }

Digest<16> md5float(const Image<float>* img, const void* extra, size_t nextra)
{ return md5helper<float>(img, extra, nextra); }

Digest<16> md5rgb(const Image<PixRGB<byte> >* img, const void* extra, size_t nextra)
{ return md5helper<PixRGB<byte> >(img, extra, nextra); }


Digest<20> sha1byte(const Image<byte>* img, const void* extra, size_t nextra)
{ return sha1helper<byte>(img, extra, nextra); }

Digest<20> sha1float(const Image<float>* img, const void* extra, size_t nextra)
{ return sha1helper<float>(img, extra, nextra); }

Digest<20> sha1rgb(const Image<PixRGB<byte> >* img, const void* extra, size_t nextra)
{ return sha1helper<PixRGB<byte> >(img, extra, nextra); }


Digest<32> sha256byte(const Image<byte>* img, const void* extra, size_t nextra)
{ return sha256helper<byte>(img, extra, nextra); }

Digest<32> sha256float(const Image<float>* img, const void* extra, size_t nextra)
{ return sha256helper<float>(img, extra, nextra); }

Digest<32> sha256rgb(const Image<PixRGB<byte> >* img, const void* extra, size_t nextra)
{ return sha256helper<PixRGB<byte> >(img, extra, nextra); }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_HASH_C_DEFINED
