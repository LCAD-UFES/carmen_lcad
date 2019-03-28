
#include <cstdio>
#include <cstdlib>
#include <opencv2/imgproc.hpp>
#include <carmen/synchronized_data_package.h>
#include <carmen/carmen_image_reader.h>
#include <carmen/util_io.h>

using namespace cv;


cv::Mat
load_image(DataSample *sample)
{
	static int image_size = 0.;
	static unsigned char *raw_right = NULL;
	static Mat img_r;

	int sample_image_size = sample->image_height * sample->image_width * 3;

	// realloc only if necessary
	if (sample_image_size != image_size)
	{
		image_size = sample_image_size;

		if (raw_right != NULL) free(raw_right);

		raw_right = (unsigned char*) calloc (image_size, sizeof(unsigned char));
		img_r = Mat(sample->image_height, sample->image_width, CV_8UC3, raw_right, 0);
	}

	FILE *image_file = fopen(sample->image_path.c_str(), "rb");

	if (image_file != NULL)
	{
		// jump the left image
		fseek(image_file, image_size * sizeof(unsigned char), SEEK_SET);
		fread(raw_right, image_size, sizeof(unsigned char), image_file);
		fclose(image_file);
		// carmen images are stored as rgb
		cvtColor(img_r, img_r, COLOR_RGB2BGR);
	}
	else
		fprintf(stderr, "Warning: image '%s' not found. Returning previous image.\n", sample->image_path.c_str());

	return img_r;
}
