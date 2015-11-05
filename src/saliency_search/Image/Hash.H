/*!@file Image/Hash.H hash/message-digest functions for Image objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Hash.H $
// $Id: Hash.H 8296 2007-04-24 23:42:40Z rjpeters $
//

#ifndef IMAGE_HASH_H_DEFINED
#define IMAGE_HASH_H_DEFINED

#include "Image/Image.H"
#include "Util/Assert.H"
#include "Util/Types.H"

#include <string>

template <class T> class PixRGB;

//! Helper function for Digest::format()
std::string digestFormatHelper(const byte* buf, unsigned int n);

//! Helper function for Digest::fromString()
void digestScanHelper(const std::string& s, byte* buf, unsigned int n);

//! Template message digest class represting a digest with N bytes.
template <unsigned int N>
struct Digest
{
  // default ctor, dtor, copy-ctor, assignment operator OK

  //! Comparison operator for sorting e.g. in std::set or std::map
  bool operator<(const Digest<N>& other) const
  {
    for (unsigned int i = 0; i < N; ++i)
      {
        if (this->buffer[i] < other.buffer[i])
          return true;
        else if (this->buffer[i] > other.buffer[i])
          return false;

        // else, if this->buffer[i] == other.buffer[i] then we'll
        // continue into the next iteration and let the
        // greater-than/less-than decision be made by the next element
      }

    // here, none of this->buffer was greater than other.buffer, nor
    // was it less than; ergo the two arrays are equal and since we
    // are computing strictly-less-than:
    return false;
  }

  //! Return a string with N*2 hexadecimal chars representing the digest
  std::string asString() const
  { return digestFormatHelper(&buffer[0], N); }

  //! Return an Nx1 Image containing the digest
  Image<byte> asImage() const
  { return Image<byte>(&buffer[0], N, 1); }

  //! Pseudo-constructor to create a digest from a byte array
  /*! NOTE! This is NOT a message digest OF the byte array; rather we
      interpreting the bytes in the array as a message digest
      itself. This might be the case if e.g. a bunch of message
      digests have been saved to disk as byte images. */
  static Digest<N> asDigest(const byte* d, const unsigned int len)
  {
    ASSERT(len == N);

    Digest<N> result;
    for (size_t i = 0; i < N; ++i)
      {
        result.buffer[i] = d[i];
      }

    return result;
  }

  //! Generate a digest by parsing hex digits from a string
  /*! @param s digest string must have exactly 2*N characters, each of
      which must be a valid hex digit (0-9, a-f, A-F) */
  static Digest<N> fromString(const std::string& s)
  {
    Digest<N> result;
    digestScanHelper(s, &result.buffer[0], N);
    return result;
  }

  byte buffer[N];
};

// ############################################################
/*! @name Image hash digest functions

    NOTE that these hash digest functions ARE NOT overloads -- that
    is, we don't have sha1sum() for Image<byte>, another sha1sum() for
    Image<float>, etc. That's because in the case of checksums, we
    want to be very clear that there shouldn't be any implicit
    conversions going on -- a checksum for an Image<byte> will be very
    different from the checksum for an Image<float> even if those two
    images contain logically identical pixels. To further avoid
    implicit conversions, the digest functions take a pointer, not a
    reference, to the Image object -- although there is an implicit
    conversion from Image<byte> to Image<float>, there is no
    conversion from Image<byte>* to Image<float>*.

    NOTE also that these functions are NOT intended to be portable
    across machines with different endianness. However, the digests
    ARE reproducible across multiple runs on different machines with
    the same endianness.

    In all of these functions, there is an optional void* parameter
    plus number of bytes, which, if non-null, will also be hashed into
    the digest. This affords the opportunity to hash some additional
    piece of information along with the image. For example, if you
    want to hash some integer 'x' along with an image, you could call
    md5byte(&img, &x, sizeof(int)).
*/
//@{

//! Compute the 128-bit md5 digest of a byte image.
Digest<16> md5byte(const Image<byte>* img,
                   const void* extra = 0, size_t nextra = 0);

//! Compute the 128-bit md5 digest of a float image.
Digest<16> md5float(const Image<float>* img,
                    const void* extra = 0, size_t nextra = 0);

//! Compute the 128-bit md5 digest of an rgb image.
Digest<16> md5rgb(const Image<PixRGB<byte> >* img,
                  const void* extra = 0, size_t nextra = 0);


//! Compute the 160-bit sha1 digest of a byte image.
Digest<20> sha1byte(const Image<byte>* img,
                    const void* extra = 0, size_t nextra = 0);

//! Compute the 160-bit sha1 digest of a float image.
Digest<20> sha1float(const Image<float>* img,
                     const void* extra = 0, size_t nextra = 0);

//! Compute the 160-bit sha1 digest of an rgb image.
Digest<20> sha1rgb(const Image<PixRGB<byte> >* img,
                   const void* extra = 0, size_t nextra = 0);


//! Compute the 256-bit sha2 digest of a byte image.
Digest<32> sha256byte(const Image<byte>* img,
                      const void* extra = 0, size_t nextra = 0);

//! Compute the 256-bit sha2 digest of a float image.
Digest<32> sha256float(const Image<float>* img,
                       const void* extra = 0, size_t nextra = 0);

//! Compute the 256-bit sha2 digest of an rgb image.
Digest<32> sha256rgb(const Image<PixRGB<byte> >* img,
                     const void* extra = 0, size_t nextra = 0);

//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_HASH_H_DEFINED
