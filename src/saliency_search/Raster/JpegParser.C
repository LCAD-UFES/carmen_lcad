/*!@file Raster/JpegParser.C Parser for jpeg image files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/JpegParser.C $
// $Id: JpegParser.C 10439 2008-11-17 06:31:45Z itti $
//

#ifndef RASTER_JPEGPARSER_C_DEFINED
#define RASTER_JPEGPARSER_C_DEFINED

#include "Raster/JpegParser.H"

#include "Raster/GenericFrame.H"
#include "rutz/error.h"
#include "rutz/sfmt.h"
#include "rutz/trace.h"

#include <csetjmp>
#include <cstdio>

#ifdef INVT_HAVE_LIBJPEG

// WARNING: jpeglib.h is braindead in a couple ways -- (1) it doesn't
// have the extern "C" to be c++-friendly, like every other modern C
// header does, so we have to provide the extern "C" ourselves, and
// (2) it's not self-contained -- it will bomb out if we don't
// explicitly #include <stdio.h> and <stdlib.h> before jpeglib.h; (3)
// on darwin it defines HAVE_STDLIB_H which conflicts with our
// definition:
#include <stdlib.h>
#include <stdio.h>
#undef HAVE_STDLIB_H
extern "C"
{
#include <jpeglib.h>
}

namespace
{
  struct jpeg_aux
  {
    FILE* infile;
    jmp_buf* jmp_state;
  };

  void cleanup(jpeg_decompress_struct* cinfo)
  {
    jpeg_aux* aux = static_cast<jpeg_aux*>(cinfo->client_data);
    if (aux->infile != 0)
      fclose(aux->infile);
    jpeg_destroy_decompress(cinfo);
  }

  void jpeg_error_exit(j_common_ptr cinfo)
  {
    // Since we longjmp out of here, DON'T use any C++ objects that
    // need to have destructors run!

    cinfo->err->output_message(cinfo);

    jpeg_aux* aux = static_cast<jpeg_aux*>(cinfo->client_data);
    longjmp(*(aux->jmp_state), 1);
  }
}

#define SETJMP_TRY(statement)                            \
do {                                                     \
  if (setjmp(state) == 0)                                \
    {                                                    \
      statement;                                         \
    }                                                    \
  else                                                   \
    {                                                    \
      LFATAL("%s failed for file %s", #statement, filename);    \
    }                                                    \
} while (0)

#endif

struct JpegParser::Impl
{
  Impl(const char* fname);

  Image<PixRGB<byte> > rgb;
  Image<byte> gray;
};

// ######################################################################

JpegParser::Impl::Impl(const char* filename)
{
#ifndef INVT_HAVE_LIBJPEG
  LFATAL("jpeg support is disabled since libjpeg was not found at configure time");
#else
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (BITS_IN_JSAMPLE != 8)
    {
      throw rutz::error("jpeg library must be built for 8 bits-per-sample",
                        SRC_POS);
    }

  jmp_buf state;

  // 1. Allocate and initialize the JPEG decompression object
  jpeg_decompress_struct cinfo;
  jpeg_error_mgr jerr;

  jpeg_aux aux;
  aux.infile = 0;
  aux.jmp_state = &state;
  cinfo.client_data = static_cast<void*>(&aux);

  cinfo.err = jpeg_std_error(&jerr);
  cinfo.err->error_exit = &jpeg_error_exit;

  SETJMP_TRY(jpeg_create_decompress(&cinfo));

  // 2. Specify the source of the compressed data (i.e., the input file)
  aux.infile = fopen(filename, "rb");

  if (aux.infile == 0)
    {
      throw rutz::error(rutz::sfmt("couldn't open '%s' for reading",
                                   filename), SRC_POS);
    }


  SETJMP_TRY(jpeg_stdio_src(&cinfo, aux.infile));

  // 3. Call jpeg_read_header() to obtain image info
  SETJMP_TRY(jpeg_read_header(&cinfo, TRUE));

  // 4. Set parameters for decompression

  // 5. Start decompression
  SETJMP_TRY(jpeg_start_decompress(&cinfo));

  // cinfo.output_width
  // cinfo.output_height
  // cinfo.out_color_components
  // cinfo.output_components

  // 6. Read scanlines

  switch (cinfo.output_components)
    {
    case 1:
      {
        this->gray = Image<byte>(cinfo.output_width,
                                 cinfo.output_height,
                                 NO_INIT);

        while (cinfo.output_scanline < cinfo.output_height)
          {
            JSAMPLE* dest =
              this->gray.getArrayPtr()
              + cinfo.output_scanline * this->gray.getWidth();

            SETJMP_TRY(jpeg_read_scanlines(&cinfo,
                                           &dest,
                                           /* max lines */ 1));
          }
      }
      break;
    case 3:
      {
        this->rgb = Image<PixRGB<byte> >(cinfo.output_width,
                                         cinfo.output_height,
                                         NO_INIT);

        while (cinfo.output_scanline < cinfo.output_height)
          {
            JSAMPLE* dest =
              reinterpret_cast<JSAMPLE*>
              (this->rgb.getArrayPtr()
               + cinfo.output_scanline * this->rgb.getWidth());

            SETJMP_TRY(jpeg_read_scanlines(&cinfo,
                                           &dest,
                                           /* max lines */ 1));
          }
      }
      break;
    default:
      LFATAL("invalid number of jpeg components (%d)",
             cinfo.output_components);
      break;
    }

  // 7. finish decompression
  SETJMP_TRY(jpeg_finish_decompress(&cinfo));

  // 8. cleanup
  cleanup(&cinfo);
#endif
}

// ######################################################################

JpegParser::JpegParser(const char* fname)
  :
  rep(new Impl(fname))
{
  ASSERT(rep->rgb.initialized() || rep->gray.initialized());
}

// ######################################################################

JpegParser::~JpegParser()
{
  delete rep;
}

GenericFrameSpec JpegParser::getFrameSpec() const
{
  GenericFrameSpec result;

  if      (rep->rgb.initialized())
    {
      result.nativeType = GenericFrame::RGB_U8;
      result.dims = rep->rgb.getDims();
    }
  else if (rep->gray.initialized())
    {
      result.nativeType = GenericFrame::GRAY_U8;
      result.dims = rep->gray.getDims();
    }
  else
    ASSERT(0);

  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.floatFlags = 0;

  return result;
}

// ######################################################################

std::string JpegParser::getComments() const
{
  return std::string();
}

// ######################################################################

uint JpegParser::getTagCount() const
{
  return 0;
}

// ######################################################################

bool JpegParser::getTag(uint tag, std::string &name, std::string &value) const
{
  return false;
}

// ######################################################################

GenericFrame JpegParser::getFrame()
{
  if (rep->rgb.initialized())
    return GenericFrame(rep->rgb);

  // else...
  ASSERT(rep->gray.initialized());
  return GenericFrame(rep->gray);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_JPEGPARSER_C_DEFINED
