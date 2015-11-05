/*!@file Raster/JpegWriter.C Write Jpeg image files */

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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/JpegWriter.C $
// $Id: JpegWriter.C 8141 2012-04-30 06:43:45Z kai $
//


#include "Raster/JpegWriter.H"

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
#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>


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
#define OUTPUT_BUF_SIZE  100000
JpegWriter::JpegWriter() {}

JpegWriter::~JpegWriter() {}

std::string JpegWriter::writeFrame(const GenericFrame& image,
                                  const std::string& fname)
{
  switch (image.nativeType())
    {
    case GenericFrame::NONE:
      LFATAL("cannot write an empty image to '%s'", fname.c_str());
      break;

    case GenericFrame::RGB_U8:
    case GenericFrame::RGBD:
      JpegWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::RGB_U16:
      JpegWriter::writeRGB(image.asRgbU16(), fname);
      break;

    case GenericFrame::RGB_F32:
      JpegWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::GRAY_U8:
      JpegWriter::writeGray(image.asGrayU8(), fname);
      break;

    case GenericFrame::GRAY_U16:
      JpegWriter::writeGray(image.asGrayU16(), fname);
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
          JpegWriter::writeGray(image2.asGrayU8(), fname);
        }
      else
        JpegWriter::writeGray(image.asGrayU8(), fname);
      break;

    case GenericFrame::VIDEO:
      JpegWriter::writeRGB(image.asRgbU8(), fname);
      break;
    }

  return fname;
}

// ######################################################################
void JpegWriter::writeRGB(const Image<PixRGB<byte> >& image,
                         const std::string& fname)
{
	JPEGCompressor jcomp;
	Image<PixRGB<byte> > img = Image<PixRGB<byte> >(image);
	jcomp.saveImage(img,fname);
}

void JpegWriter::writeGray(const Image<byte>& image,
                          const std::string& fname)
{
	JPEGCompressor jcomp;
	Image<byte> img = Image<byte>(image);
	jcomp.saveImage(img,fname);
}

#endif // INVT_HAVE_LIBJPEG

//namespace
//{
//  rutz::fstring errnoMessage()
//  {
//    if (errno != 0)
//      return rutz::sfmt("\n  [errno %d: %s]", errno, strerror(errno));
//
//    return rutz::fstring();
//  }
//
//
//  /// Helper class for PNG-writing functions.
//  /** Essentially this is here to make proper error cleanup easier,
//      e.g. in the presence of exceptions. */
//  class JpegWriterHelper
//  {
//		typedef struct {
//			//The public fields common to destination managers
//			struct jpeg_destination_mgr pub;
//
//			//The input buffer pointer
//			JOCTET* in_buffer;
//
//			//The destination queue of
//			std::vector<unsigned char>* out_buffer;
//
//		} buff_dest_mgr;
//  public:
//    JpegWriterHelper(const char* filename);
//
//    ~JpegWriterHelper();
//
//    /// Return the error status given by ::close().
//    /** If possible, we want to check and report that error status;
//        however if close() is being called from ~JpegWriterHelper(),
//        then it's already too late (we can't/shouldn't throw an
//        exception from a destructor) and we'll have to just ignore the
//        error. */
//    int close();
//
//    /// Write an 8-bit grayscale PNG image file.
//    void writeGray(const Image<byte>& img);
//
//    /// Write an 8-bit color PNG image file.
//    void writeRGB(const Image<PixRGB<byte> >& img);
//
//  private:
//    JpegWriterHelper(const JpegWriterHelper&);
//    JpegWriterHelper& operator=(const JpegWriterHelper&);
//
//    void onError(const rutz::fstring& msg);
//
//    const rutz::fstring itsFilename;
//    FILE* itsFile;
//
//    //The public fields common to destination managers
//    struct jpeg_destination_mgr pub;
//
//    //The input buffer pointer
//    JOCTET* in_buffer;
//
//    //The destination queue of
//    std::vector<unsigned char>* out_buffer;
//
//  };
//  ////////////////////////////////////////////////////////////////
//  // Initialize destination --- called by jpeg_start_compress
//  // before any data is actually written.
//  ////////////////////////////////////////////////////////////////
//  void JpegWriterHelper::init_destination(j_compress_ptr cinfo)
//  {
//    //Get the pointer to our cinfo's destination manager
//    buff_dest_mgr* dest = (buff_dest_mgr*) cinfo->dest;
//
//    //Allocate the input buffer --- it will be released when done with image
//    dest->in_buffer = (JOCTET *)
//      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
//          OUTPUT_BUF_SIZE * sizeof(JOCTET));
//
//    //Reset the input buffer
//    dest->pub.next_output_byte = dest->in_buffer;
//    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
//  }
//  ////////////////////////////////////////////////////////////////
//  // Empty the output buffer --- called whenever buffer fills up.
//  ////////////////////////////////////////////////////////////////
//  bool JpegWriterHelper::empty_output_buffer (j_compress_ptr cinfo)
//  {
//    //Get the pointer to our cinfo's destination manager
//    buff_dest_mgr* dest = (buff_dest_mgr*) cinfo->dest;
//
//    //Copy the rest of the input buffer onto the end of our output buffer
//    dest->out_buffer->insert(dest->out_buffer->end(),
//        dest->in_buffer,
//        dest->in_buffer+OUTPUT_BUF_SIZE);
//
//    //Reset the input buffer
//    dest->pub.next_output_byte = dest->in_buffer;
//    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
//
//    return TRUE;
//  }
//
//  ////////////////////////////////////////////////////////////////
//  // Terminate destination --- called by jpeg_finish_compress
//  // after all data has been written.  Usually needs to flush buffer.
//  ////////////////////////////////////////////////////////////////
//  void JpegWriterHelper::term_destination (j_compress_ptr cinfo)
//  {
//    //Get the pointer to our cinfo's destination manager
//    buff_dest_mgr* dest = (buff_dest_mgr*) cinfo->dest;
//
//    //Calculate the number of bytes left to be written
//    size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;
//
//    //Copy the rest of the input buffer onto the end of our output buffer
//    dest->out_buffer->insert(dest->out_buffer->end(),
//        dest->in_buffer,
//        dest->in_buffer+datacount);
//  }
//
//  JpegWriterHelper::JpegWriterHelper(const char* filename)
//    :
//    itsFilename(filename),
//    itsFile(0)
//  {
//    errno = 0;
//
//    itsFile = fopen(filename, "wb");
//		if(!itsFile) LFATAL("Error opening output jpeg file [%s]", itsFile);
//
//
//
//  }
//
//  JpegWriterHelper::~JpegWriterHelper()
//  {
//  }
//
//
// // ######################################################################
//  void JpegWriterHelper::doJpegCompressionGray(jpeg_compress_struct &cinfo,const Image<byte>& img, int quality = 75)
//	{
//    cinfo.image_width      = img.getWidth();	
//    cinfo.image_height     = img.getHeight();
//
//		cinfo.in_color_space   = JCS_GRAYSCALE;
//		cinfo.input_components = 1;
//    const byte* raw_image = img.getArrayPtr();
//
//		/* default compression parameters, we shouldn't be worried about these */
//		jpeg_set_defaults( &cinfo );
//
//		jpeg_set_quality(&cinfo, quality, TRUE);
//
//		/* Now do the compression .. */
//		jpeg_start_compress( &cinfo, TRUE );
//
//		/* like reading a file, this time write one row at a time */
//		JSAMPROW row_pointer[1];
//		while( cinfo.next_scanline < cinfo.image_height )
//		{
//			row_pointer[0] = &raw_image[ cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
//			jpeg_write_scanlines( &cinfo, row_pointer, 1 );
//		}
//		    jpeg_finish_compress( &cinfo );
//    jpeg_destroy_compress( &cinfo );
//	}
// // ######################################################################
//  void JpegWriterHelper::doJpegCompressionRGB(jpeg_compress_struct &cinfo,const Image<PixelRGB<byte> >& img, int quality = 75)
//  {
//    cinfo.image_width      = img.width();	
//    cinfo.image_height     = img.height();
//
//      cinfo.in_color_space   = JCS_RGB;
//      cinfo.input_components = 3;
//			const PixRGB<byte>* raw_image = img.getArrayPtr();
//
//      /* default compression parameters, we shouldn't be worried about these */
//      jpeg_set_defaults( &cinfo );
//
//			jpeg_set_quality(&cinfo, quality, TRUE);
//			/* Now do the compression .. */
//			jpeg_start_compress( &cinfo, TRUE );
//
//			/* like reading a file, this time write one row at a time */
//			JSAMPROW row_pointer[1];
//			while( cinfo.next_scanline < cinfo.image_height )
//			{
//				row_pointer[0] = &raw_image[ cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
//				jpeg_write_scanlines( &cinfo, row_pointer, 1 );
//			}
//
//    jpeg_finish_compress( &cinfo );
//    jpeg_destroy_compress( &cinfo );
//	}
// // ######################################################################
//  void JpegWriterHelper::writeRGB(const Image<PixRGB<byte> >& img)
//  {
//		struct jpeg_compress_struct cinfo;
//		struct jpeg_error_mgr jerr;
//		cinfo.err = jpeg_std_error( &jerr );
//		jpeg_create_compress(&cinfo);
//		jpeg_stdio_dest(&cinfo, itsFile);
//
//		doJpegCompressionRGB(cinfo, img, quality);
//
//		fclose(itsFile);
//	}
// // ######################################################################
//
//  void JpegWriterHelper::writeGray(const Image<byte>& img)
//  {
//		struct jpeg_compress_struct cinfo;
//		struct jpeg_error_mgr jerr;
//		cinfo.err = jpeg_std_error( &jerr );
//		jpeg_create_compress(&cinfo);
//		jpeg_stdio_dest(&cinfo, itsFile);
//
//		doJpegCompressionGray(cinfo, img, quality);
//
//		fclose(itsFile);
//
//  }
//
//} // end anonymous namespace

