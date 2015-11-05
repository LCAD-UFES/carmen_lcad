#ifndef JPEGUTIL_H
#define JPEGUTIL_H

#include "Image/Image.H"
#include "Image/PixelsTypes.H"

#undef HAVE_STDLIB_H  // needed on OSX

#include <jpeglib.h>
#include <jerror.h>

class JPEGDecompressor
{
  public:
    JPEGDecompressor();

    Image<PixRGB<byte> > DecompressImage(std::vector<unsigned char> &img_buffer);

  private:

    //Structure to hold all of our jpeg compression parameters
    struct jpeg_decompress_struct cinfo;

    //Structure to handle all of our errors
    struct jpeg_error_mgr jerr;

    GLOBAL(void) InitImageSource(std::vector<unsigned char>* source_buffer);
};

class JPEGCompressor
{
public:
  JPEGCompressor();
  
  std::vector<unsigned char> compressImage(Image<PixRGB<byte> >& input);
  
	//! Save color image to file
  void saveImage (Image<PixRGB<byte> >& input, const std::string& fname);

  //! Save grayscale image to file
	void saveImage (Image<byte>& input, const std::string& fname);
    
  private:

    //Structure to hold all of our jpeg compression parameters
    struct jpeg_compress_struct cinfo;

    //Structure to handle all of our errors
    struct jpeg_error_mgr jerr;

    GLOBAL(void) InitImageDest(std::vector<unsigned char>* destination_buffer);
};

#endif //JPEGUTIL_H

