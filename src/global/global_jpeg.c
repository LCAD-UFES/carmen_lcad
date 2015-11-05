/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include "global.h"
#include <carmen/map.h>
#include "global_graphics.h"
#include <jpeglib.h>

void
carmen_graphics_write_map_as_jpeg(char *filename, carmen_map_p map, int flags)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE * outfile;
  JSAMPROW row_pointer[1];        /* pointer to a single row */
  int row_stride;                 /* physical row width in buffer */

  unsigned char *image_data;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  
  if ((outfile = fopen(filename, "wb")) == NULL) {
    carmen_perror("Can't open %s", filename);
    return;
  }
  jpeg_stdio_dest(&cinfo, outfile);
  cinfo.image_width = map->config.x_size; /* image width & height, in pixels */
  cinfo.image_height = map->config.y_size;
  cinfo.input_components = 3;     /* # of color components per pixel */
  cinfo.in_color_space = JCS_RGB; /* colorspace of input image */

  jpeg_set_defaults(&cinfo);
  jpeg_start_compress(&cinfo, TRUE);
    
  image_data = 
    carmen_graphics_convert_to_image(map, flags | CARMEN_GRAPHICS_ROTATE);

  row_stride = map->config.x_size * 3;   /* JSAMPLEs per row in image_buffer */

  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = image_data + cinfo.next_scanline * row_stride;
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }
  jpeg_finish_compress(&cinfo);
  fclose(outfile);
  jpeg_destroy_compress(&cinfo);
}
