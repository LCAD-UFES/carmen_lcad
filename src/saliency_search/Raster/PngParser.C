/*!@file Raster/PngParser.C Parse png image files. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PngParser.C $
// $Id: PngParser.C 11208 2009-05-20 02:03:21Z itti $
//

#ifdef INVT_HAVE_LIBPNG

#include "Raster/PngParser.H"

#include "Util/Assert.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/log.H"
#include "Util/sformat.H"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <png.h>

// ######################################################################
struct PngParser::Rep
{
  Rep(const char* nm) :
    filename(nm), file(0), pngPtr(0), infoPtr(0), endPtr(0)
  {}

  void close()
  {
    if (this->pngPtr != 0)
      {
        png_destroy_read_struct(this->pngPtr ? &this->pngPtr : 0,
                                this->infoPtr ? &this->infoPtr : 0,
                                this->endPtr ? &this->endPtr : 0);
      }

    if (this->file != 0)
      {
        fclose(this->file);
        this->file = 0;
      }
  }

  bool closed() const
  {
    return this->file == 0;
  }

  void onError(const std::string& msg)
  {
    this->close();
    if (errno == 0)
      {
        LFATAL("with file %s: %s\n", this->filename, msg.c_str());
      }
    else
      {
        LFATAL("with file %s: %s\n  [errno %d: %s]\n",
               this->filename, msg.c_str(), errno, strerror(errno));
      }
  }

  bool isGray() const
  {
    return this->colorType == PNG_COLOR_TYPE_GRAY
      || this->colorType == PNG_COLOR_TYPE_GRAY_ALPHA;
  }

  bool isColor() const
  {
    return this->colorType == PNG_COLOR_TYPE_RGB
      || this->colorType == PNG_COLOR_TYPE_RGB_ALPHA
      || this->colorType == PNG_COLOR_TYPE_PALETTE;
  }

  const char* filename;
  FILE* file;
  png_structp pngPtr;
  png_infop infoPtr;
  png_infop endPtr;
  int width;
  int height;
  int bitDepth;
  png_byte colorType;
  int rowBytes;
  int numChannels;

};

// ######################################################################
PngParser::PngParser(const char* filename)
  :
  rep(new Rep(filename))
{
  errno = 0;
  rep->file = fopen(filename, "rb");
  if (rep->file == 0) rep->onError("couldn't open file for png reading");

  const size_t nheader = 8;
  png_byte header[nheader];

  if (fread(header, 1, nheader, rep->file) != nheader) rep->onError("short read on png file header");

  int is_png = !png_sig_cmp(header, 0, nheader);
  if (!is_png) rep->onError("file was not a png image file");

  rep->pngPtr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
  if (rep->pngPtr == 0) rep->onError("png_create_read_struct failed");

  rep->infoPtr = png_create_info_struct(rep->pngPtr);
  if (rep->infoPtr == 0) rep->onError("png_create_info_struct failed");

  rep->endPtr = png_create_info_struct(rep->pngPtr);
  if (rep->endPtr == 0) rep->onError("png_create_info_struct failed");

  png_init_io(rep->pngPtr, rep->file);

  png_set_sig_bytes(rep->pngPtr, nheader);

  png_read_info(rep->pngPtr, rep->infoPtr);

  rep->bitDepth = png_get_bit_depth(rep->pngPtr, rep->infoPtr);

  rep->colorType = png_get_color_type(rep->pngPtr, rep->infoPtr);

  if (rep->bitDepth == 16)
    png_set_strip_16(rep->pngPtr);
  else if (rep->bitDepth < 8 && (rep->colorType == PNG_COLOR_TYPE_GRAY))
    png_set_gray_1_2_4_to_8(rep->pngPtr);
  else if (rep->bitDepth != 8 && (rep->colorType != PNG_COLOR_TYPE_PALETTE))
    rep->onError(sformat("invalid bit-depth(%d)/color-mode(%d) "
                         "combination", rep->bitDepth, rep->colorType));

  // Strip out the alpha channel, if present
  if (rep->colorType & PNG_COLOR_MASK_ALPHA)
    png_set_strip_alpha(rep->pngPtr);

  if (rep->colorType & PNG_COLOR_MASK_PALETTE)
    png_set_palette_to_rgb(rep->pngPtr);

  // This must come after any+all transformations are specified
  png_read_update_info(rep->pngPtr, rep->infoPtr);

  // These calls must come after png_read_update_info, so that we get
  // values that reflect any transformations
  rep->width = png_get_image_width(rep->pngPtr, rep->infoPtr);
  rep->height = png_get_image_height(rep->pngPtr, rep->infoPtr);

  rep->rowBytes = png_get_rowbytes(rep->pngPtr, rep->infoPtr);

  rep->numChannels = png_get_channels(rep->pngPtr, rep->infoPtr);

  ASSERT(rep->rowBytes = rep->width*rep->numChannels);

// run-time mmx PNG optimizations (nate)
#if defined(HAVE_PNG_ASM_FLAGS)
  const png_uint_32 flags = png_get_asm_flags(rep->pngPtr);
  const png_uint_32 mask =
    png_get_asm_flagmask(PNG_SELECT_READ | PNG_SELECT_WRITE);
  png_set_asm_flags(rep->pngPtr, flags | mask);
#endif
}

// ######################################################################
PngParser::~PngParser()
{
  rep->close();

  delete rep;
}

// ######################################################################
GenericFrameSpec PngParser::getFrameSpec() const
{
  GenericFrameSpec result;

  if (rep->isGray())       result.nativeType = GenericFrame::GRAY_U8;
  else if (rep->isColor()) result.nativeType = GenericFrame::RGB_U8;
  else rep->onError("unsupported image type (neither grayscale nor RGB)");

  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = Dims(rep->width, rep->height);
  result.floatFlags = 0;

  return result;
}

// ######################################################################
std::string PngParser::getComments() const
{ return "PNG comments not currently supported"; }

// ######################################################################
uint PngParser::getTagCount() const
{ return 0; }

// ######################################################################
bool PngParser::getTag(uint tag, std::string &name, std::string &value) const
{ return false; }

// ######################################################################
GenericFrame PngParser::getFrame()
{
  if (rep->isGray())       return GenericFrame(parseGray());
  else if (rep->isColor()) return GenericFrame(parseRGB());
  // else...
  rep->onError("unsupported image type (neither grayscale nor RGB)");
  /* can't happen */ return GenericFrame();
}

// ######################################################################
Image<byte> PngParser::parseGray()
{
  ASSERT(!rep->closed());
  ASSERT(rep->isGray());
  ASSERT(rep->rowBytes == rep->width);

  errno = 0;

  Image<byte> result(rep->width, rep->height, NO_INIT);

  png_bytep* row_pointers = new png_bytep[rep->height];

  for (int i = 0; i < rep->height; ++i)
    {
      row_pointers[i] = (png_bytep) (result.getArrayPtr() + rep->width*i);
    }

  png_read_image(rep->pngPtr, row_pointers);

  png_read_end(rep->pngPtr, rep->endPtr);

  delete [] row_pointers;

  rep->close();

  return result;
}

// ######################################################################
Image<PixRGB<byte> > PngParser::parseRGB()
{
  ASSERT(!rep->closed());
  ASSERT(rep->isColor());
  ASSERT(rep->rowBytes == rep->width*3);

  errno = 0;

  Image<PixRGB<byte> > result(rep->width, rep->height, NO_INIT);

  png_bytep* row_pointers = new png_bytep[rep->height];

  for (int i = 0; i < rep->height; ++i)
    {
      row_pointers[i] = (png_bytep) (result.getArrayPtr() + rep->width*i);
    }

  png_read_image(rep->pngPtr, row_pointers);

  png_read_end(rep->pngPtr, rep->endPtr);

  delete [] row_pointers;

  rep->close();

  return result;
}

#endif // INVT_HAVE_LIBPNG

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
