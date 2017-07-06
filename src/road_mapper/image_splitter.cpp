/*
 * image_splitter.cpp
 *
 *  Created on: Jun 7, 2017
 *      Author: rcarneiro
 */
/*
 * Command-line: image_splitter <split_rows> <split_columns> <file_with_png_image_list.txt> <path_of_output_images>
 *
 * Input:	1)	Number of split rows
 * 			2)	Number of split columns
 * 			3)	Text filename which contains a list of PNG image filenames to be split.
 * 				Each PNG filename must be in the following format: i<x_origin>_<y_origin>.00.png.
 * 				X and Y origins are world coordinates in meters and may be positive or negative up to 8 digits.
 * 			4)	Path where the split PNG image files will be stored.
 *
 * Output: each PNG file will be split in <split_rows> * <split_columns> part files and stored in the output directory.
 *
 */

#include <carmen/carmen.h>
#include <png.h>

int width, height;
png_byte color_type;
png_byte bit_depth;
png_bytep *row_pointers;

/*
int read_string(char *buf, int size, carmen_FILE *fp)
{
  int i;
  int result;
  char c;

  for (i = 0; i < size || size < 0; i++) {
    result = carmen_fgetc(fp);
    c = (char) result;
    if (result == EOF)
      return -1;
    if (c == '\0') {
      if (size >= 0)
    	buf[i] = '\0';
      return i;
    }
    if (size >= 0)
      buf[i] = c;
  }

  buf[size-1] = '\0';
  return -1;
}
*/

int read_png_file(char *filename)
{
	FILE *fp = fopen(filename, "rb");
	if(!fp) {
		printf("Cannot open file for reading: %s\n", filename);
		return false;
	}

	png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png) {
		printf("Cannot create PNG read struct for:%s\n", filename);
		return false;
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
		printf("Cannot create PNG info struct for:%s\n", filename);
		return false;
	}

	if(setjmp(png_jmpbuf(png))) {
		printf("Cannot set PNG JMP buf for:%s\n", filename);
		return false;
	}

	png_init_io(png, fp);

	png_read_info(png, info);

	width      = png_get_image_width(png, info);
	height     = png_get_image_height(png, info);
	color_type = png_get_color_type(png, info);
	bit_depth  = png_get_bit_depth(png, info);

	// Read any color_type into 8bit depth, RGBA format.
	// See http://www.libpng.org/pub/png/libpng-manual.txt

	if(bit_depth == 16)
		png_set_strip_16(png);

	if(color_type == PNG_COLOR_TYPE_PALETTE)
		png_set_palette_to_rgb(png);

	// PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
	if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
		png_set_expand_gray_1_2_4_to_8(png);

	if(png_get_valid(png, info, PNG_INFO_tRNS))
		png_set_tRNS_to_alpha(png);

	// These color_types don't have an alpha channel then fill it with 0xff.
 	if(color_type == PNG_COLOR_TYPE_RGB ||
	   color_type == PNG_COLOR_TYPE_GRAY ||
       color_type == PNG_COLOR_TYPE_PALETTE)
 		png_set_filler(png, 0xFF, PNG_FILLER_AFTER);

	if(color_type == PNG_COLOR_TYPE_GRAY ||
       color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
		png_set_gray_to_rgb(png);

	png_read_update_info(png, info);

	row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
	for(int y = 0; y < height; y++) {
		row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
	}

	png_read_image(png, row_pointers);

	png_destroy_read_struct(&png, &info, NULL);

	fclose(fp);

	return true;
}

/*
void write_png_file(char *filename) {
  int y;

  FILE *fp = fopen(filename, "wb");
  if(!fp)
	  abort();

  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png)
	  abort();

  png_infop info = png_create_info_struct(png);
  if (!info)
	  abort();

  if (setjmp(png_jmpbuf(png)))
	  abort();

  png_init_io(png, fp);

  // Output is 8bit depth, RGBA format.
  png_set_IHDR(
    png,
    info,
    width, height,
    8,
    PNG_COLOR_TYPE_RGBA,
    PNG_INTERLACE_NONE,
    PNG_COMPRESSION_TYPE_DEFAULT,
    PNG_FILTER_TYPE_DEFAULT
  );
  png_write_info(png, info);

  // To remove the alpha channel for PNG_COLOR_TYPE_RGB format,
  // Use png_set_filler().
  //png_set_filler(png, 0, PNG_FILLER_AFTER);

  png_write_image(png, row_pointers);
  png_write_end(png, NULL);

  for(int y = 0; y < height; y++) {
    free(row_pointers[y]);
  }
  free(row_pointers);

  if (png && info)
	png_destroy_write_struct(&png, &info);

  fclose(fp);
}
*/

int write_part_png_file(char *filename, int init_column, int init_row, int part_width, int part_height)
{

	if(init_column < 0 || init_column >= width || init_row < 0 || init_row >= height) {
		printf("Invalid range: column=%d (0:%d) row=%d (0:%d)\n", init_column, width - 1, init_row, height - 1);
		return false;
	}
	if(part_width < 1 || part_width > (width - init_column) || part_height < 1 || part_height > (height - init_row)) {
		printf("Invalid range: width=%d (1:%d) height=%d (1:%d)\n", part_width, (width - init_column), part_height, (height - init_row));
		return false;
	}

	FILE *fp = fopen(filename, "wb");
	if(!fp) {
		printf("Cannot open file for writing: %s\n", filename);
		return false;
	}

	png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png) {
		printf("Cannot create PNG write struct for:%s\n", filename);
		return false;
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
		printf("Cannot create PNG info struct for:%s\n", filename);
		return false;
	}

	if (setjmp(png_jmpbuf(png))) {
		printf("Cannot set PNG JMP buf for:%s\n", filename);
		return false;
	}

	png_init_io(png, fp);

	// Output is 8bit depth, RGBA format.
	png_set_IHDR(
			png,
			info,
			part_width, part_height,
			8,
			PNG_COLOR_TYPE_RGBA,
			PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_DEFAULT,
			PNG_FILTER_TYPE_DEFAULT
	);
	png_write_info(png, info);

	png_bytep *part_row_pointers;
	part_row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * part_height);
	for(int y = 0; y < part_height; y++) {
		part_row_pointers[y] = &(row_pointers[y + init_row][init_column * 4]);
	}

	png_write_image(png, part_row_pointers);
	png_write_end(png, NULL);

	free(part_row_pointers);

	if (png && info)
		png_destroy_write_struct(&png, &info);

	fclose(fp);

	return true;
}

/*
void process_png_file() {
  for(int y = 0; y < height; y++) {
    png_bytep row = row_pointers[y];
    for(int x = 0; x < width; x++) {
      png_bytep px = &(row[x * 4]);
      // Do something awesome for each pixel here...
      //printf("%4d, %4d = RGBA(%3d, %3d, %3d, %3d)\n", x, y, px[0], px[1], px[2], px[3]);
    }
  }
}
*/

int get_coordinates(const char *fname, int *x, int *y)
{
	// PNG filename must be in format: <path>/i<x_origin>_<y_origin>.00.png

	int n;

	for(n = strlen(fname); n > 0 && fname[n - 1] != '/'; n--);

	return (sscanf(&fname[n],"i%d_%d.00.png", x, y) == 2);
}

int main(int argc, char **argv)
{
	#define flist_name	argv[3]
	#define opath		argv[4]
	#define x_pixels_per_meter		5
	#define y_pixels_per_meter		5
	const char usage[] =
			"Usage: image_splitter <split_rows> <split_columns> <file_with_png_image_list.txt> <path_to_output_images>";
	int split_rows, split_columns;
	FILE *flist;

	char fname_in[2000];
	char fname_out[2000];

	if (argc != 5) {
		printf("%s\nInvalid number of arguments\n", usage);
		return false;
	}

	split_rows = atoi(argv[1]);
	split_columns = atoi(argv[2]);
	if (split_rows <= 0 || split_columns <= 0 || (split_rows * split_columns) <= 1) {
		printf("%s\nInvalid range: rows=%d (>0) columns=%d (>0) rows*columns=%d (>1)\n", usage,
				split_rows, split_columns, (split_rows * split_columns));
		return false;
	}

	flist = fopen(flist_name, "r");
	if (!flist) {
		printf("%s\nCannot open file for reading: %s\n", usage, flist_name);
		return false;
	}

	int in_count = 0, ignore_count = 0,  out_count = 0, duplicate_count = 0, error_out_count = 0;

	while (fscanf(flist, "%s\n", fname_in) != EOF)
	{
		int x_origin, y_origin;

		printf("#%d In: %s\n", ++in_count, fname_in);

		if(!get_coordinates(fname_in, &x_origin, &y_origin)) {
			printf("PNG filename must be in format: <path>/i<x_origin>_<y_origin>.00.png\n");
			printf("Ignore #%d: %s\n", ++ignore_count, fname_in);
			continue;
		}

		if(!read_png_file(fname_in)) {
			printf("Ignore #%d: %s\n", ++ignore_count, fname_in);
			continue;
		}

		int part_width = (width / split_columns) + ((width % split_columns) != 0);
		int part_height = (height / split_rows) + ((height % split_rows) != 0);

		for(int x = 0; x < split_columns; x++) {

			int init_x = x * part_width;
			int init_x_meters = x_origin + (init_x / x_pixels_per_meter);
			if((part_width + init_x) > width)
				part_width = width - init_x;

			for(int y = 0; y < split_rows; y++) {

				int init_y = y * part_height;
				int init_y_meters = y_origin + (init_y / y_pixels_per_meter);
				init_y = height - part_height - init_y;
				if(init_y < 0) {
					part_height += init_y;
					init_y = 0;
				}

				sprintf(fname_out, "%s/i%d_%d.00.png", opath, init_x_meters, init_y_meters);
				printf("#%d Out: %s\n", ++out_count, fname_out);

			    if (FILE *fp = fopen(fname_out, "r")) {
			        fclose(fp);
					printf("Duplicate #%d: %s\n", ++duplicate_count, fname_out);
					continue;
			    }

				if(!write_part_png_file(fname_out, init_x, init_y, part_width, part_height)) {
					printf("Error Out #%d: %s\n", ++error_out_count, fname_out);
					continue;
				}
			}
		}

		if (row_pointers) {
			for(int y = 0; y < height; y++) {
				free(row_pointers[y]);
			}
			free(row_pointers);
		}
	}

	printf("\n\nin_count = %d, ignore_count = %d, actual_in_count = %d\n",
			in_count, ignore_count, (in_count - ignore_count));
	printf("\nout_count = %d, duplicate_count = %d, error_out_count = %d, actual_out_count = %d\n\n",
			out_count, duplicate_count, error_out_count, (out_count - duplicate_count - error_out_count));

	return true;
}
