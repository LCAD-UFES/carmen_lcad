/*!@file Raster/PnmParser.C Parse pbm/pgm/ppm image files. */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PnmParser.C $
// $Id: PnmParser.C 8790 2007-09-28 22:24:10Z rjpeters $
//

#include "Raster/PnmParser.H"

#include "Util/Assert.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/Assert.H"
#include "Util/FileUtil.H"
#include "Util/log.H"
#include "rutz/shared_ptr.h"

#include <cctype>
#include <istream>
#include <limits>
#include <string>

// ######################################################################
struct PnmParser::Rep
{
  Rep(const std::string& fname_) :
    fname(fname_),
    owned_strm(openMaybeCompressedFile(fname_)),
    strm(0),
    mode(-1),
    w(-1),
    h(-1),
    maxGrey(1),
    comments("")
  {
    ASSERT(owned_strm.get() != 0);
    strm = owned_strm.get();
  }

  Rep(std::istream& strm_) :
    fname("(anonymous stream)"),
    owned_strm(),
    strm(&strm_),
    mode(-1),
    w(-1),
    h(-1),
    maxGrey(1),
    comments("")
  {}

  void init();

  /// Read the actual pixels out of the file, for 1-byte-per-value images
  void readPixels8(byte* space, int nvals);

  /// Read the actual pixels out of the file, for 2-bytes-per-value images
  void readPixels16(uint16* space, int nvals);

  /// Parse a 1-bit black+white raw or ascii image file into an 8-bit image.
  /** The underlying file <i>must</i> be a black_white file (mode 1 or
      4). */
  Image<byte> parseBW();

  /// Parse an 8-bit grayscale image in either raw or ascii mode.
  /** The underlying file <i>must</i> be a grayscale file (mode 2 or 5). */
  Image<byte> parseGrayU8();

  /// Parse a 16-bit grayscale image in either raw or ascii mode.
  /** The underlying file <i>must</i> be a grayscale file (mode 2 or 5). */
  Image<uint16> parseGrayU16();

  /// Parse an 8-bit rgb image in either raw or ascii mode
  /** The underlying file <i>must</i> be a RGB file (mode 3 or 6). */
  Image<PixRGB<byte> > parseRgbU8();

  /// Parse a 16-bit rgb image in either raw or ascii mode
  /** The underlying file <i>must</i> be a RGB file (mode 3 or 6). */
  Image<PixRGB<uint16> > parseRgbU16();

  std::string fname;
  rutz::shared_ptr<std::istream> owned_strm;
  std::istream* strm;
  int mode;
  int w, h;
  int maxGrey;
  std::string comments;
};

// ######################################################################
void PnmParser::Rep::init()
{
  int c = this->strm->get();
  if (c != 'P')
    LFATAL("Missing magic number in pnm file '%s'"
           "(got '%c' [%d], expected '%c' [%d]).",
           fname.c_str(), c, c, 'P', 'P');

  *(this->strm) >> this->mode;

  *(this->strm) >> std::ws;
  // copy and concatenate optional comment line(s) starting with '#'
  // into comments string

  std::string temp;

  while ( this->strm->peek() == '#' )
    {
      //get the full line
      std::getline( *(this->strm), temp, '\n');
      this->comments += temp;

      //This is what we did before we were interested in the comments
      //this->strm->ignore(std::numeric_limits<int>::max(), '\n');
    }

  *(this->strm) >> this->w >> this->h;

  if (this->mode != 1 && this->mode != 4)
    {
      *(this->strm) >> this->maxGrey;
    }

  // read one more character of whitespace from the stream after maxGrey
  c = this->strm->get();
  if ( !isspace(c) )
    LFATAL("Missing whitespace after maxGrey in pbm file '%s'.",
           fname.c_str());
}

// ######################################################################
void PnmParser::Rep::readPixels8(byte* p, int nvals)
{
  ASSERT(this->mode == 2 || this->mode == 3 || this->mode == 5 || this->mode == 6);

  ASSERT(this->maxGrey > 0);
  ASSERT(this->maxGrey <= 255);

  const double scale = 255.0/double(this->maxGrey);

  if (this->mode == 2 || this->mode == 3)
    {
      // Parse ascii:
      int position = 0;
      int val = 0;

      while (this->strm->peek() != EOF && position < nvals)
        {
          *(this->strm) >> val;
          p[position++] = static_cast<byte>(val * scale);
        }
    }
  else
    {
      // Parse raw:
      this->strm->read(reinterpret_cast<char*>(p), nvals);

      if (this->maxGrey != 255)
        for (int i = 0; i < nvals; ++i)
          {
            if (p[i] > this->maxGrey)
              {
                LERROR("%s: value %d was %d but maxgrey "
                       "is claimed to be %d",
                       this->fname.c_str(), i, p[i], this->maxGrey);
                p[i] = this->maxGrey;
              }
            p[i] = byte(p[i] * scale);
          }
    }
}

// ######################################################################
void PnmParser::Rep::readPixels16(uint16* p, int nvals)
{
  ASSERT(this->mode == 2 || this->mode == 3 || this->mode == 5 || this->mode == 6);

  ASSERT(this->maxGrey >= 256);
  ASSERT(this->maxGrey <= 65535);

  const double scale = 65535.0/double(this->maxGrey);

  if (this->mode == 2 || this->mode == 3)
    {
      // Parse ascii:
      int position = 0;
      int val = 0;

      while (this->strm->peek() != EOF && position < nvals)
        {
          *(this->strm) >> val;
          p[position++] = static_cast<uint16>(val * scale);
        }
    }
  else
    {
      // Parse raw:
      this->strm->read(reinterpret_cast<char*>(p), nvals * 2);

      union { byte b[2]; uint16 u16; } u;
      u.b[0] = 0; u.b[1] = 1;

      if (u.u16 != 1) // need to swap bytes
        {
          for (int i = 0; i < nvals; ++i)
            p[i] = ((p[i] & 0xff00) >> 8) | ((p[i] & 0x00ff) << 8);
        }

      if (this->maxGrey != 65535)
        for (int i = 0; i < nvals; ++i)
          {
            if (p[i] > this->maxGrey)
              {
                LERROR("%s: value %d was %d but maxgrey "
                       "is claimed to be %d",
                       this->fname.c_str(), i, p[i], this->maxGrey);
                p[i] = this->maxGrey;
              }
            p[i] = uint16(p[i] * scale);
          }
    }
}

// ######################################################################
Image<byte> PnmParser::Rep::parseBW()
{
  ASSERT(this->mode == 1 || this->mode == 4);

  Image<byte> img(this->w, this->h, NO_INIT);

  if (this->mode == 1)
    {
      Image<byte>::iterator itr = img.beginw(), stop = img.endw();

      while (itr != stop)
        {
          const int c = this->strm->get();
          if (c == EOF)
            LFATAL("while parsing %s: premature EOF before pixel #%d",
                   this->fname.c_str(), int(stop - itr));

          if (isspace(c))
            continue;

          if (c == '0') // "0" ==> white pixel
            *itr++ = 255;
          else if (c == '1') // "1" ==> black pixel
            *itr++ = 0;
          else
            LFATAL("while parsing %s: "
                   "invalid pixel value '%c' at pixel #%d",
                   this->fname.c_str(), c, int(stop - itr));
        }
    }
  else if (this->mode == 4)
    {
      Image<byte>::iterator itr = img.beginw(), stop = img.endw();

      int c = 0;
      int pos = 7;

      while (itr != stop)
        {
          if (pos == 7)
            {
              c = this->strm->get();
              if (c == EOF)
                LFATAL("while parsing %s: premature EOF before pixel #%d",
                       this->fname.c_str(), int(stop - itr));
            }

          if (c & (1 << pos))
            *itr++ = 0;
          else
            *itr++ = 255;

          if (pos == 0) pos = 7;
          else --pos;
        }
    }

  return img;
}

// ######################################################################
Image<byte> PnmParser::Rep::parseGrayU8()
{
  ASSERT(this->mode == 2 || this->mode == 5);

  Image<byte> img(this->w, this->h, NO_INIT);
  this->readPixels8(img.getArrayPtr(), img.getSize());
  return img;
}

// ######################################################################
Image<uint16> PnmParser::Rep::parseGrayU16()
{
  ASSERT(this->mode == 2 || this->mode == 5);

  Image<uint16> img(this->w, this->h, NO_INIT);
  this->readPixels16(img.getArrayPtr(), img.getSize());
  return img;
}

// ######################################################################
Image<PixRGB<byte> > PnmParser::Rep::parseRgbU8()
{
  ASSERT(this->mode == 3 || this->mode == 6);

  Image<PixRGB<byte> > img(this->w, this->h, NO_INIT);
  this->readPixels8(reinterpret_cast<byte*>(img.getArrayPtr()), img.getSize()*3);
  return img;
}

// ######################################################################
Image<PixRGB<uint16> > PnmParser::Rep::parseRgbU16()
{
  ASSERT(this->mode == 3 || this->mode == 6);

  Image<PixRGB<uint16> > img(this->w, this->h, NO_INIT);
  this->readPixels16(reinterpret_cast<uint16*>(img.getArrayPtr()), img.getSize()*3);
  return img;
}

// ######################################################################
PnmParser::PnmParser(const char* fname) :
  rep(new Rep(fname))
{
  rep->init();
}

// ######################################################################
PnmParser::PnmParser(std::istream& strm) :
  rep(new Rep(strm))
{
  rep->init();
}

// ######################################################################
PnmParser::~PnmParser()
{
  delete rep;
}

// ######################################################################
GenericFrameSpec PnmParser::getFrameSpec() const
{
  GenericFrameSpec result;

  switch (rep->mode)
    {
    case 1: case 4: // 1-bit bitmap file, but we parse it into an 8-bit image:
      result.nativeType = GenericFrame::GRAY_U8;
      break;

    case 2: case 5: // 8-bit grayscale
      result.nativeType =
        rep->maxGrey <= 255
        ? GenericFrame::GRAY_U8
        : GenericFrame::GRAY_F32;
      break;

    case 3: case 6: // 24-bit RGB
      result.nativeType =
        rep->maxGrey <= 255
        ? GenericFrame::RGB_U8
        : GenericFrame::RGB_F32;
      break;

    default:
      LFATAL("invalid PNM mode '%d'", rep->mode);
    }

  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = Dims(rep->w, rep->h);
  result.floatFlags = 0;

  return result;
}

// ######################################################################
std::string PnmParser::getComments() const
{ return rep->comments; }

// ######################################################################
uint PnmParser::getTagCount() const
{ return 0; }

// ######################################################################
bool PnmParser::getTag(uint tag, std::string &name, std::string &value) const
{ return false; }

// ######################################################################
GenericFrame PnmParser::getFrame()
{
  switch (rep->mode)
    {
    case 1: case 4: // black+white
      return GenericFrame(rep->parseBW());
      break;

    case 2: case 5: // grayscale
      if (rep->maxGrey <= 255)
        return GenericFrame(rep->parseGrayU8());
      else
        {
          const Image<uint16> gray16(rep->parseGrayU16());
          Image<float> ret(gray16.getDims(), NO_INIT);
          Image<uint16>::const_iterator sptr = gray16.begin();
          Image<float>::iterator fptr = ret.beginw();
          Image<float>::iterator const stop = ret.endw();
          // scale the result so its range is still [0.0,256.0[, just
          // as if the input image file had been 8-bit, but now with
          // extra precision:
          while (fptr != stop)
            *fptr++ = *sptr++ / 256.0f;
          return GenericFrame(ret, /* floatFlags = */ 0);
        }
      break;

    case 3: case 6: // rgb
      if (rep->maxGrey <= 255)
        return GenericFrame(rep->parseRgbU8());
      else
        {
          const Image<PixRGB<uint16> > rgb16(rep->parseRgbU16());
          Image<PixRGB<float> > ret(rgb16.getDims(), NO_INIT);
          Image<PixRGB<uint16> >::const_iterator sptr = rgb16.begin();
          Image<PixRGB<float> >::iterator fptr = ret.beginw();
          Image<PixRGB<float> >::iterator const stop = ret.endw();
          // scale the result so its range is still [0.0,256.0[, just
          // as if the input image file had been 8-bit, but now with
          // extra precision:
          while (fptr != stop)
            *fptr++ = PixRGB<float>(*sptr++) / 256.0f;
          return GenericFrame(ret, /* floatFlags = */ 0);
        }
      break;
    }

  LFATAL("invalid PNM mode '%d'", rep->mode);
  /* can't happen */ return GenericFrame();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
