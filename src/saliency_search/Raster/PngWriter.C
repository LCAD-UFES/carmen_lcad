/*!@file Raster/PngWriter.C Write PNG image files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PngWriter.C $
// $Id: PngWriter.C 15424 2012-11-02 08:02:51Z kai $
//

#ifdef INVT_HAVE_LIBPNG

#include "Raster/PngWriter.H"

#include "Image/Image.H"
#include "Image/Normalize.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "rutz/fstring.h"
#include "rutz/sfmt.h"
#include "Util/FileUtil.H"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <png.h>
#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>


namespace
{
  rutz::fstring errnoMessage()
  {
    if (errno != 0)
      return rutz::sfmt("\n  [errno %d: %s]", errno, strerror(errno));

    return rutz::fstring();
  }

  /// Helper class for PNG-writing functions.
  /** Essentially this is here to make proper error cleanup easier,
      e.g. in the presence of exceptions. */
  class PngWriterHelper
  {
  public:
    PngWriterHelper(const char* filename);

    ~PngWriterHelper();

    /// Return the error status given by ::close().
    /** If possible, we want to check and report that error status;
        however if close() is being called from ~PngWriterHelper(),
        then it's already too late (we can't/shouldn't throw an
        exception from a destructor) and we'll have to just ignore the
        error. */
    int close();

    /// Write an 8-bit grayscale PNG image file.
    void writeGray(const Image<byte>& img);

    /// Write an 16-bit grayscale PNG image file.
    void writeGray16(const Image<uint16>& img);

    /// Write an 8-bit color PNG image file.
    void writeRGB(const Image<PixRGB<byte> >& img);

  private:
    PngWriterHelper(const PngWriterHelper&);
    PngWriterHelper& operator=(const PngWriterHelper&);

    void onError(const rutz::fstring& msg);

    const rutz::fstring itsFilename;
    FILE* itsFile;
    png_structp itsPngPtr;
    png_infop itsInfoPtr;
  };

  PngWriterHelper::PngWriterHelper(const char* filename)
    :
    itsFilename(filename),
    itsFile(0),
    itsPngPtr(0),
    itsInfoPtr(0)
  {
    errno = 0;

    itsFile = fopen(filename, "wb");

    if (itsFile == 0)
      {
        onError(rutz::sfmt("couldn't open png file '%s' for writing%s",
                           filename, errnoMessage().c_str()));
      }

    const int lockstat = flock(fileno(itsFile),LOCK_EX);

    if (lockstat == EBADF)
      {
        onError(rutz::sfmt("couldn't open png file '%s' for writing%s",
                           filename, errnoMessage().c_str()));
      }
    if (lockstat == EINTR)
      {
        onError(rutz::sfmt("Lock on png file '%s' interupted %s",
                           filename, errnoMessage().c_str()));
      }

    itsPngPtr = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
    if (itsPngPtr == 0)
      {
        onError(rutz::sfmt("png_create_write_struct() failed for file '%s'%s",
                           filename, errnoMessage().c_str()));
      }

    itsInfoPtr = png_create_info_struct(itsPngPtr);
    if (itsInfoPtr == 0)
      {
        onError(rutz::sfmt("png_create_info_struct() failed for file '%s'%s",
                           filename, errnoMessage().c_str()));
      }

    png_init_io(itsPngPtr, itsFile);

    png_set_compression_level(itsPngPtr, 9);
  }

  PngWriterHelper::~PngWriterHelper()
  {
    const int status = this->close();
    if (status != 0)
      // NOTE: Don't use PLFATAL here since then we'd end up with
      // double-error-reporting.
      PLERROR("Error during fclose() while writing %s",
              itsFilename.c_str());
  }

  int PngWriterHelper::close()
  {
    if (itsPngPtr != 0)
      {
        png_destroy_write_struct(itsPngPtr ? &itsPngPtr : 0,
                                 itsInfoPtr ? &itsInfoPtr : 0);

        itsPngPtr = 0;
        itsInfoPtr = 0;
      }

    if (itsFile != 0)
      {
        // force a FULL sync before we unlock this file
        // fclose will not wait for the file to get onto
        // the HD so we can still remove a lock on a file
        // being written.
        fsync(fileno(itsFile));
        // copy since fclose may delete the reference before flock is called
        const int file        = fileno(itsFile);
        const int status      = fclose(itsFile);
        /*const int lockstat =*/ flock(file,LOCK_UN);
        itsFile = 0;
        return status;
      }

    return 0;
  }

  void PngWriterHelper::writeGray16(const Image<uint16>& img)
  {
    // run-time mmx PNG optimizations (nate)
#if defined(HAVE_PNG_ASM_FLAGS)
    const png_uint_32 flags = png_get_asm_flags(itsPngPtr);
    const png_uint_32 mask =
      png_get_asm_flagmask(PNG_SELECT_READ | PNG_SELECT_WRITE);
    png_set_asm_flags(itsPngPtr, flags | mask);
#endif

    png_set_IHDR(itsPngPtr, itsInfoPtr,
                 img.getWidth(), img.getHeight(),
                 16, /* bits per pixel element */
                 PNG_COLOR_TYPE_GRAY,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(itsPngPtr, itsInfoPtr);

    const int height = img.getHeight();

    png_bytep row_pointers[height];

    const uint16* p = img.getArrayPtr();

    for (int i = 0; i < height; ++i)
      {
        row_pointers[i] =
          static_cast<png_bytep>
          (static_cast<void*>
           (const_cast<uint16*>(p + i*img.getWidth())));
      }

    png_write_image(itsPngPtr, &row_pointers[0]);
    png_write_end(itsPngPtr, itsInfoPtr);

    const int status = this->close();
    if (status != 0)
      PLFATAL("Error during fclose() while writing %s",
              itsFilename.c_str());
  }
  void PngWriterHelper::writeGray(const Image<byte>& img)
  {
    // run-time mmx PNG optimizations (nate)
#if defined(HAVE_PNG_ASM_FLAGS)
    const png_uint_32 flags = png_get_asm_flags(itsPngPtr);
    const png_uint_32 mask =
      png_get_asm_flagmask(PNG_SELECT_READ | PNG_SELECT_WRITE);
    png_set_asm_flags(itsPngPtr, flags | mask);
#endif

    png_set_IHDR(itsPngPtr, itsInfoPtr,
                 img.getWidth(), img.getHeight(),
                 8, /* bits per pixel element */
                 PNG_COLOR_TYPE_GRAY,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(itsPngPtr, itsInfoPtr);

    const int height = img.getHeight();

    png_bytep row_pointers[height];

    const byte* p = img.getArrayPtr();

    for (int i = 0; i < height; ++i)
      {
        row_pointers[i] =
          static_cast<png_bytep>
          (static_cast<void*>
           (const_cast<byte*>(p + i*img.getWidth())));
      }

    png_write_image(itsPngPtr, &row_pointers[0]);
    png_write_end(itsPngPtr, itsInfoPtr);

    const int status = this->close();
    if (status != 0)
      PLFATAL("Error during fclose() while writing %s",
              itsFilename.c_str());
  }

  void PngWriterHelper::writeRGB(const Image<PixRGB<byte> >& img)
  {

    // run-time mmx PNG optimizations (nate)
#if defined(HAVE_PNG_ASM_FLAGS)
    const png_uint_32 flags = png_get_asm_flags(itsPngPtr);
    const png_uint_32 mask =
      png_get_asm_flagmask(PNG_SELECT_READ | PNG_SELECT_WRITE);
    png_set_asm_flags(itsPngPtr, flags | mask);
#endif

    png_set_IHDR(itsPngPtr, itsInfoPtr,
                 img.getWidth(), img.getHeight(),
                 8, /* bits per pixel element */
                 PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(itsPngPtr, itsInfoPtr);

    const int height = img.getHeight();

    png_bytep row_pointers[height];

    const PixRGB<byte>* p = img.getArrayPtr();

    for (int i = 0; i < height; ++i)
      {
        row_pointers[i] =
          static_cast<png_bytep>
          (static_cast<void*>
           (const_cast<PixRGB<byte>*>(p + i*img.getWidth())));
      }

    png_write_image(itsPngPtr, &row_pointers[0]);
    png_write_end(itsPngPtr, itsInfoPtr);

    const int status = this->close();
    if (status != 0)
      PLFATAL("Error during fclose() while writing %s",
              itsFilename.c_str());
  }

  void PngWriterHelper::onError(const rutz::fstring& msg)
  {
    const int status = this->close();
    if (status != 0)
      // NOTE: Don't use PLFATAL here since then we'd end up with
      // double-error-reporting.
      PLERROR("Error during fclose() while writing %s",
              itsFilename.c_str());

    LFATAL("%s", msg.c_str());
  }

} // end anonymous namespace

PngWriter::PngWriter() {}

PngWriter::~PngWriter() {}

std::string PngWriter::writeFrame(const GenericFrame& image,
                                  const std::string& fname)
{
  switch (image.nativeType())
    {
    case GenericFrame::NONE:
      LFATAL("cannot write an empty image to '%s'", fname.c_str());
      break;

    case GenericFrame::RGB_U8:
    case GenericFrame::RGBD:
      PngWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::RGB_U16:
      PngWriter::writeRGB(image.asRgbU16(), fname);
      break;

    case GenericFrame::RGB_F32:
      PngWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::GRAY_U8:
      PngWriter::writeGray(image.asGrayU8(), fname);
      break;

    case GenericFrame::GRAY_U16:
      PngWriter::writeGray(image.asGrayU16(), fname);
      break;

    case GenericFrame::GRAY_F32:
      if ((image.floatFlags() & FLOAT_NORM_PRESERVE)
          && !(image.floatFlags() & FLOAT_NORM_0_255))
        {
          // the user wanted to preserve the full floating-point
          // precision without normalizing to 0..255; however for png
          // output we don't have any way to accomodate that request
          // so let's just go ahead and normalize to 0..255 so that
          // the output will be viewable:
          const GenericFrame image2(image.asGrayF32(),
                                    image.floatFlags() | FLOAT_NORM_0_255);
          PngWriter::writeGray(image2.asGrayU8(), fname);
        }
      else
        PngWriter::writeGray(image.asGrayU8(), fname);
      break;

    case GenericFrame::VIDEO:
      PngWriter::writeRGB(image.asRgbU8(), fname);
      break;
    }

  return fname;
}

void PngWriter::writeRGB(const Image<PixRGB<byte> >& image,
                         const std::string& fname)
{
  PngWriterHelper writer(fname.c_str());
  writer.writeRGB(image);
}

void PngWriter::writeGray(const Image<byte>& image,
                          const std::string& fname)
{
  PngWriterHelper writer(fname.c_str());
  writer.writeGray(image);
}
void PngWriter::writeGray(const Image<uint16>& image,
                          const std::string& fname)
{
  PngWriterHelper writer(fname.c_str());
  writer.writeGray16(image);
}

#endif // INVT_HAVE_LIBPNG

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
