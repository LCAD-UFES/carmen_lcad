#include "Image/JPEGUtil.H"


#include <vector>
#include <math.h>
#include <cstdio>

#define INPUT_BUF_SIZE  4096LU
//The output buffer size... I believe this should be as large as possible
//so that empty_output_buffer will never need to be called as an intermediary
//step, and we can just rely on term_destination to write all of our data
#define OUTPUT_BUF_SIZE  100000


namespace IIDC_NS
{
  typedef struct
  {
    //The public fields common to source managers
    struct jpeg_source_mgr pub;

    //The compressed image source base pointer
    std::vector<unsigned char>* src_vec;
    int curr_src_pos;

    //The output buffer pointer
    JOCTET* buffer;

  } buff_src_mgr;


  ////////////////////////////////////////////////////////////////
  // Initialize the source -- called by jpeg_read_header before
  // any data is actually read
  ////////////////////////////////////////////////////////////////
  METHODDEF(void)
    init_source (j_decompress_ptr cinfo)
    {
      buff_src_mgr* src = (buff_src_mgr*) cinfo->src;
      src->curr_src_pos = 0;
    }

  ////////////////////////////////////////////////////////////////
  // Fill the input buffer -- called whenever bytes_in_buffer is 0
  ////////////////////////////////////////////////////////////////
  METHODDEF(boolean)
    fill_input_buffer (j_decompress_ptr cinfo)
    {

      buff_src_mgr* src = (buff_src_mgr*) cinfo->src;

      size_t nbytes;

      nbytes = std::min(INPUT_BUF_SIZE, (long unsigned int)(src->src_vec->size() - src->curr_src_pos));
      unsigned char* src_base = &((*(src->src_vec))[0]);


      src->pub.next_input_byte =  src_base + src->curr_src_pos;
      src->pub.bytes_in_buffer = nbytes;
      src->curr_src_pos += nbytes;

      return TRUE;
    }

  ////////////////////////////////////////////////////////////////
  // Skip input data -- Skips num_bytes worth of data without
  // putting it into the buffer
  ////////////////////////////////////////////////////////////////
  METHODDEF(void)
    skip_input_data (j_decompress_ptr cinfo, long num_bytes)
    {
      buff_src_mgr* src = (buff_src_mgr*) cinfo->src;

      src->curr_src_pos += num_bytes;

      src->pub.next_input_byte += (size_t) num_bytes;
      src->pub.bytes_in_buffer -= (size_t) num_bytes;
    }

  ////////////////////////////////////////////////////////////////
  // Terminate source -- Nothing to do here
  ////////////////////////////////////////////////////////////////
  METHODDEF(void)
    term_source (j_decompress_ptr cinfo)
    {
      /* no work necessary here */
    }

}

namespace IIC_NS
{
  typedef struct {
    //The public fields common to destination managers
    struct jpeg_destination_mgr pub;

    //The input buffer pointer
    JOCTET* in_buffer;

    //The destination queue of
    std::vector<unsigned char>* out_buffer;

  } buff_dest_mgr;

  ////////////////////////////////////////////////////////////////
  // Initialize destination --- called by jpeg_start_compress
  // before any data is actually written.
  ////////////////////////////////////////////////////////////////
  METHODDEF(void) init_destination(j_compress_ptr cinfo)
  {
    //Get the pointer to our cinfo's destination manager
    buff_dest_mgr* dest = (buff_dest_mgr*) cinfo->dest;

    //Allocate the input buffer --- it will be released when done with image
    dest->in_buffer = (JOCTET *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
          OUTPUT_BUF_SIZE * sizeof(JOCTET));

    //Reset the input buffer
    dest->pub.next_output_byte = dest->in_buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
  }

  ////////////////////////////////////////////////////////////////
  // Empty the output buffer --- called whenever buffer fills up.
  ////////////////////////////////////////////////////////////////
  METHODDEF(boolean) empty_output_buffer (j_compress_ptr cinfo)
  {
    //Get the pointer to our cinfo's destination manager
    buff_dest_mgr* dest = (buff_dest_mgr*) cinfo->dest;

    //Copy the rest of the input buffer onto the end of our output buffer
    dest->out_buffer->insert(dest->out_buffer->end(),
        dest->in_buffer,
        dest->in_buffer+OUTPUT_BUF_SIZE);

    //Reset the input buffer
    dest->pub.next_output_byte = dest->in_buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;

    return TRUE;
  }

  ////////////////////////////////////////////////////////////////
  // Terminate destination --- called by jpeg_finish_compress
  // after all data has been written.  Usually needs to flush buffer.
  ////////////////////////////////////////////////////////////////
  METHODDEF(void) term_destination (j_compress_ptr cinfo)
  {
    //Get the pointer to our cinfo's destination manager
    buff_dest_mgr* dest = (buff_dest_mgr*) cinfo->dest;

    //Calculate the number of bytes left to be written
    size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;

    //Copy the rest of the input buffer onto the end of our output buffer
    dest->out_buffer->insert(dest->out_buffer->end(),
        dest->in_buffer,
        dest->in_buffer+datacount);
  }


}

///////////////////////////////////////////////////////////////////////////////

// constuctor does nothing, maybe that needs FIXME
JPEGDecompressor::JPEGDecompressor()
{ }

////////////////////////////////////////////////////////////////
// Decompress an Image -- Pass this function an std::vector
// filled with data from a compressed jpeg image, and it will
// return to you an uncompressed Image< PixRGB<byte> >
////////////////////////////////////////////////////////////////
Image<PixRGB<byte> > JPEGDecompressor::DecompressImage(std::vector<unsigned char> &source_buffer)
{
  //Initialize our jpeg error handler to the default
  //(spit out non-fatal error messages on cerr, and
  // exit() on fatal errors)
  cinfo.err = jpeg_std_error(&jerr);

  //Create our jpeg decompression object
  jpeg_create_decompress(&cinfo);

  InitImageSource(&source_buffer);

  int ret =  jpeg_read_header(&cinfo, TRUE);
  if (ret != JPEG_HEADER_OK)
    return Image<PixRGB<byte> >();

  (void) jpeg_start_decompress(&cinfo);


  assert(cinfo.output_components == 3);
  Image<PixRGB<byte> > outputImg(cinfo.output_width, cinfo.output_height, NO_INIT);
  JSAMPLE* img_ptr = reinterpret_cast<JSAMPLE*>(outputImg.getArrayPtr());

  int row_stride = cinfo.output_width * cinfo.output_components;

  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)
    ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

  while(cinfo.output_scanline < cinfo.output_height)
  {
    (void) jpeg_read_scanlines(&cinfo, buffer, 1);
    std::memcpy(img_ptr, *buffer, row_stride);
    img_ptr+=row_stride;
  }


  (void) jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

  return outputImg;
}


////////////////////////////////////////////////////////////////
// Initialize image source -- Just set up some variables and
// allocate some buffer space (if necessary) for the
// decompressor
////////////////////////////////////////////////////////////////
GLOBAL(void) JPEGDecompressor::InitImageSource(std::vector<unsigned char>* source_buffer)
{
  IIDC_NS::buff_src_mgr* src;

  //Allocate the source buffer
  if (cinfo.src == NULL)
  {        /* first time for this JPEG object? */

    cinfo.src = (struct jpeg_source_mgr *)
      (*cinfo.mem->alloc_small) ((j_common_ptr) &cinfo, JPOOL_PERMANENT,
          sizeof(IIDC_NS::buff_src_mgr));

    src = (IIDC_NS::buff_src_mgr*) cinfo.src;

    src->buffer = (JOCTET *)
      (*cinfo.mem->alloc_small) ((j_common_ptr) &cinfo, JPOOL_PERMANENT,
          INPUT_BUF_SIZE * sizeof(JOCTET));
  }


  src = (IIDC_NS::buff_src_mgr*) cinfo.src;
  src->pub.init_source = IIDC_NS::init_source;
  src->pub.fill_input_buffer = IIDC_NS::fill_input_buffer;
  src->pub.skip_input_data = IIDC_NS::skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
  src->pub.term_source = IIDC_NS::term_source;
  src->pub.bytes_in_buffer = 0; /* forces fill_input_buffer on first read */
  src->pub.next_input_byte = NULL; /* until buffer loaded */

  src->src_vec      = source_buffer;
  src->curr_src_pos = 0;

}
///////////////////////////////////////////////////////////////////////////////

JPEGCompressor::JPEGCompressor()
{

	//Initialize our jpeg error handler to the default
  //(spit out non-fatal error messages on cerr, and
  // exit() on fatal errors)
  cinfo.err = jpeg_std_error(&jerr);

  //Create our jpeg compression object
  jpeg_create_compress(&cinfo);

}

 // ######################################################################
std::vector<unsigned char> JPEGCompressor::compressImage(Image<PixRGB<byte> >& input)
{
	std::vector<unsigned char> jpeg_buffer;

  //Initialize our data destination
  InitImageDest(&jpeg_buffer);

  //Set our image and compression parameters
  cinfo.image_width   = input.getWidth();
  cinfo.image_height  = input.getHeight();
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);

  //Begin the compression
  jpeg_start_compress(&cinfo, TRUE);

  //Get a pointer to the start of the image's raw data array. This assumes that
  //all of the image data is layed out as R G B R G B, with each element as a
  //byte in contiguous memory. This should be a valid assumption for any
  //Image<PixRGB<byte> >
  JSAMPLE* p_start = reinterpret_cast<JSAMPLE*>(input.getArrayPtr());

  //Pass a pointer to each row of the image to the jpeg compressor.  It would
  //be nice to do this all in one shot, but libjpeg segfaults when I try.
  for(int row_idx=0; row_idx<input.getHeight(); row_idx++)
  {
    JSAMPLE* p = (p_start + (row_idx * input.getWidth()*3) );
    jpeg_write_scanlines(&cinfo, &p, 1);
  }

  //Clean up the compression, and finish writing all bytes to the output buffer
  jpeg_finish_compress(&cinfo);

  return jpeg_buffer;

}
 // ######################################################################
void JPEGCompressor::saveImage
(Image<PixRGB<byte> >& input, const std::string& fname)
{

	FILE* file = fopen(fname.c_str(), "wb");
	if(!file) LFATAL("Error opening output jpeg file [%s]", fname.c_str());
  //Initialize our data destination
  jpeg_stdio_dest(&cinfo, file); 
  //InitImageDest(&jpeg_buffer);

  //Set our image and compression parameters
  cinfo.image_width   = input.getWidth();
  cinfo.image_height  = input.getHeight();
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);

  //Begin the compression
  jpeg_start_compress(&cinfo, TRUE);

  //Get a pointer to the start of the image's raw data array. This assumes that
  //all of the image data is layed out as R G B R G B, with each element as a
  //byte in contiguous memory. This should be a valid assumption for any
  //Image<PixRGB<byte> >
  JSAMPLE* p_start = reinterpret_cast<JSAMPLE*>(input.getArrayPtr());

  //Pass a pointer to each row of the image to the jpeg compressor.  It would
  //be nice to do this all in one shot, but libjpeg segfaults when I try.
  for(int row_idx=0; row_idx<input.getHeight(); row_idx++)
  {
    JSAMPLE* p = (p_start + (row_idx * input.getWidth()*3) );
    jpeg_write_scanlines(&cinfo, &p, 1);
  }

  //Clean up the compression, and finish writing all bytes to the output buffer
  jpeg_finish_compress(&cinfo);

}

 // ######################################################################
void JPEGCompressor::saveImage
(Image<byte >& input, const std::string& fname)
{

  //Initialize our data destination
	FILE* file = fopen(fname.c_str(), "wb");
	if(!file) LFATAL("Error opening output jpeg file [%s]", fname.c_str());
  jpeg_stdio_dest(&cinfo, file); 

  //Set our image and compression parameters
  cinfo.image_width   = input.getWidth();
  cinfo.image_height  = input.getHeight();
  cinfo.input_components = 1;
  cinfo.in_color_space = JCS_GRAYSCALE;
  jpeg_set_defaults(&cinfo);

  //Begin the compression
  jpeg_start_compress(&cinfo, TRUE);

  //Get a pointer to the start of the image's raw data array. This assumes that
  //all of the image data is layed out as R G B R G B, with each element as a
  //byte in contiguous memory. This should be a valid assumption for any
  //Image<PixRGB<byte> >
  JSAMPLE* p_start = reinterpret_cast<JSAMPLE*>(input.getArrayPtr());

  //Pass a pointer to each row of the image to the jpeg compressor.  It would
  //be nice to do this all in one shot, but libjpeg segfaults when I try.
  for(int row_idx=0; row_idx<input.getHeight(); row_idx++)
  {
    JSAMPLE* p = (p_start + (row_idx * input.getWidth()) );
    jpeg_write_scanlines(&cinfo, &p, 1);
  }

  //Clean up the compression, and finish writing all bytes to the output buffer
  jpeg_finish_compress(&cinfo);

}
 // ######################################################################
GLOBAL(void) JPEGCompressor::InitImageDest(std::vector<unsigned char>* destination_buffer)
{

 IIC_NS::buff_dest_mgr* dest;

  if(cinfo.dest == NULL)
  {
    //Allocate some memory space for our destination manager.
    cinfo.dest = (struct jpeg_destination_mgr *)
      (*(cinfo.mem->alloc_small)) ((j_common_ptr) &cinfo, JPOOL_PERMANENT, sizeof(IIC_NS::buff_dest_mgr));
  }

  //Initialize our destination manager by filling in all of the appropriate
  //function pointers, and assigning the output buffer.
  dest = (IIC_NS::buff_dest_mgr*) cinfo.dest;
  dest->pub.init_destination    = IIC_NS::init_destination;
  dest->pub.empty_output_buffer = IIC_NS::empty_output_buffer;
  dest->pub.term_destination    = IIC_NS::term_destination;
  dest->out_buffer              = destination_buffer;

}



