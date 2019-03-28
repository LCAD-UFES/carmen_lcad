
#include <cstdio>
#include <cstdlib>
#include <opencv2/imgproc.hpp>
#include <carmen/synchronized_data_package.h>
#include <carmen/carmen_image_reader.h>
#include <carmen/util_io.h>

using namespace cv;

CarmenImageLoader::CarmenImageLoader(CarmenImageLoader::CamSide side)
{
	_side = side;
	_image_size = 0.;
	_raw = NULL;
}


CarmenImageLoader::~CarmenImageLoader()
{
	if (_raw != NULL)
		free(_raw);
}


cv::Mat
CarmenImageLoader::load(DataSample *sample)
{
	_recreate_image_if_necessary(sample);

	FILE *fptr = fopen(sample->image_path.c_str(), "rb");

	if (fptr != NULL)
	{
		// skip left image if we are interested in the right one.
		if (_side == RIGHT_CAM)
			fseek(fptr, _image_size * sizeof(unsigned char), SEEK_SET);

		fread(_raw, _image_size, sizeof(unsigned char), fptr);
		fclose(fptr);

		// carmen images are stored as rgb
		cvtColor(_img, _img, COLOR_RGB2BGR);
	}
	else
		fprintf(stderr, "Warning: image '%s' not found. Returning previous image.\n", sample->image_path.c_str());

	return _img;
}


void
CarmenImageLoader::_recreate_image_if_necessary(DataSample *sample)
{
	int sample_image_size = sample->image_height * sample->image_width * 3;

	if ((sample_image_size > 0) && (sample_image_size != _image_size))
	{
		_image_size = sample_image_size;

		if (_raw != NULL)
			free(_raw);

		_raw = (unsigned char*) calloc (_image_size, sizeof(unsigned char));
		_img = Mat(sample->image_height, sample->image_width, CV_8UC3, _raw, 0);
	}
}

