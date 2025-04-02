/*********************************************************
 * Bumblebee2 Camera Module
 **********************************************************/

#include <carmen/carmen.h>
#include <carmen/stereo_interface.h>
#include <carmen/stereo_messages.h>

#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>

#include <qx_csbp.h>
#include <sys/time.h>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>

#include <vg_ram.h>

//#include "elas.h"

static int max_disparity_rescale;
static double lambda = 1024;

static int stereo_width;
static int stereo_height;
static int max_disparity;
static int smoothing_kernel_size = 0;
static int stereo_scalew;
static int stereo_scaleh;
static char *algorithm;
static double gaussian_radius;
static int synapses;
static int wintakeiteration;
static int numdisparity;
static int numthreads;
static int inc_width;
static int inc_height;

static int vertical_ROI_ini;
static int vertical_ROI_end;
static int horizontal_ROI_ini;
static int horizontal_ROI_end;

static int ROI_width;
static int ROI_height;

static int camera;

static int bumblebee_basic_width;
static int bumblebee_basic_height;

IplImage *image_left_copy;
IplImage *image_left_rescale;
unsigned char *raw_image_left_rescale;

IplImage *image_right_copy;
IplImage *image_right_rescale;
unsigned char *raw_image_right_rescale;

IplImage *depth_image;
IplImage *depth_image_rescale;
unsigned char *raw_disparity_rescale;

IplConvKernel *kernel;

static float *left_disparity;
static float *right_disparity;
static unsigned char *gray_left_roi;
static unsigned char *gray_right_roi;

carmen_simple_stereo_disparity_message disparity_message;


void
copy_RGB_image_to_BGR_image(unsigned char *original, IplImage *copy, int nchannels)
{
	int i, j;

	for (i = 0; i < copy->height; i++)
	{
		unsigned char* data = (unsigned char*) copy->imageData + (i * copy->widthStep);
		if (nchannels == 3)
		{
			for (j = 0; j < copy->width; j++)
			{
				data[nchannels * j + 2] = original[nchannels * (i * copy->width + j) + 0];
				data[nchannels * j + 1] = original[nchannels * (i * copy->width + j) + 1];
				data[nchannels * j + 0] = original[nchannels * (i * copy->width + j) + 2];
			}
		}
		else
		{
			for (j = 0; j < copy->width; j++)
				data[j] = original[i * copy->width + j];
		}
	}
}


void
copy_BGR_image_to_RGB_image(IplImage *original, unsigned char *copy, int nchannels)
{
	int i, j;

	for (i = 0; i < original->height; i++)
	{
		unsigned char *data = (unsigned char *) original->imageData + (i * original->widthStep);
		if (nchannels == 3)
		{
			for (j = 0; j < original->width; j++)
			{
				copy[nchannels * (i * original->width + j) + 2] = data[nchannels * j + 0];
				copy[nchannels * (i * original->width + j) + 1] = data[nchannels * j + 1];
				copy[nchannels * (i * original->width + j) + 0] = data[nchannels * j + 2];
			}
		}
		else
		{
			for (j = 0; j < original->width; j++)
				copy[i * original->width + j] = data[j];
		}
	}
}


void
copy_raw_image_to_opencv_image(unsigned char *original, IplImage *copy, int nchannels)
{
	int i, j;

	for (i = 0; i < copy->height; i++)
	{
		unsigned char *data = (unsigned char *) copy->imageData + (i * copy->widthStep);
		if (nchannels == 3)
		{
			for (j = 0; j < copy->width; j++)
			{
				data[nchannels * j] = original[nchannels * (i * copy->width + j)];
				data[nchannels * j + 1] = original[nchannels * (i * copy->width + j) + 1];
				data[nchannels * j + 2] = original[nchannels * (i * copy->width + j) + 2];
			}
		}
		else
		{
			for (j = 0; j < copy->width; j++)
				data[j] = original[i * copy->width + j];
		}
	}
}


void
copy_opencv_image_to_raw_image(IplImage *original, unsigned char *copy, int nchannels)
{
	int i, j;

	for (i = 0; i < original->height; i++)
	{
		unsigned char *data = (unsigned char *) original->imageData + (i * original->widthStep);
		if (nchannels == 3)
		{
			for (j = 0; j < original->width; j++)
			{
				copy[nchannels * (i * original->width + j)] = data[nchannels * j];
				copy[nchannels * (i * original->width + j) + 1] = data[nchannels * j + 1];
				copy[nchannels * (i * original->width + j) + 2] = data[nchannels * j + 2];
			}
		}
		else
		{
			for (j = 0; j < original->width; j++)
				copy[i * original->width + j] = data[j];
		}
	}
}


void
adjust_disparity_scale_from_opencv_image(IplImage *original, float *adjusted_disparity)
{
	int i, j;
	double scale;

	scale = 1.0f; //(double) bumblebee_basic_width / (double) stereo_scalew;
	for (i = 0; i < original->height; i++)
	{
		unsigned char *data = (unsigned char *) original->imageData + (i * original->widthStep);
		for (j = horizontal_ROI_ini; j < horizontal_ROI_end; j++)
			adjusted_disparity[i * original->width + j] = data[j] * scale;
	}
}


void
convert_disparity_from_raw_image_to_float(unsigned char *raw_image, float *disparity)
{
	int i = vertical_ROI_ini, j = 0, x = 0, y = 0;

	for (y = vertical_ROI_ini; i < vertical_ROI_end; y++, i++)
	{
		for (j = horizontal_ROI_ini, x = horizontal_ROI_ini; j < horizontal_ROI_end; x++, j++)
			disparity[y * stereo_width + x] = ((float) raw_image[i * bumblebee_basic_width + j]);
	}
}


void
rescale_image(unsigned char *raw_image)
{
	int i = 0, j = 0, x = 0, y = 0;

	for (y = 0; i < bumblebee_basic_height; y++, i += inc_height)
	{
		for (j = horizontal_ROI_ini, x = horizontal_ROI_ini; j < horizontal_ROI_end; x++, j += inc_width)
			depth_image_rescale->imageData[y * depth_image_rescale->widthStep + x] = (raw_image[i * bumblebee_basic_width + j]);
	}

	cvErode(depth_image_rescale, depth_image_rescale, kernel, 1);
	cvDilate(depth_image_rescale, depth_image_rescale, kernel, 1);

	//cvSmooth(depth_image_rescale, depth_image_rescale, CV_GAUSSIAN, 5, 0, 0, 0);

	cvResize(depth_image_rescale, depth_image, CV_INTER_LINEAR);

	copy_opencv_image_to_raw_image(depth_image, raw_image, 1);
}


void
compute_depth_map_rescale(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	unsigned char *right_roi_image, *left_roi_image, *disparity_roi_map;

	copy_raw_image_to_opencv_image(stereo_image->raw_left, image_left_copy, 3);

	cvResize(image_left_copy, image_left_rescale, CV_INTER_LINEAR);
	copy_opencv_image_to_raw_image(image_left_rescale, raw_image_left_rescale, 3);

	copy_raw_image_to_opencv_image(stereo_image->raw_right, image_right_copy, 3);

	cvResize(image_right_copy, image_right_rescale, CV_INTER_LINEAR);
	copy_opencv_image_to_raw_image(image_right_rescale, raw_image_right_rescale, 3);

	left_roi_image = raw_image_left_rescale + vertical_ROI_ini * 3 * stereo_scalew;
	right_roi_image = raw_image_right_rescale + vertical_ROI_ini * 3 * stereo_scalew;
	disparity_roi_map = raw_disparity_rescale + vertical_ROI_ini * stereo_scalew;

	switch (algorithm[0])
	{
		case 'V':
			disparityVgRamWNN(left_roi_image, right_roi_image, ROI_height, ROI_width, max_disparity_rescale, disparity_message.disparity, synapses, numthreads,
					wintakeiteration, inc_width, inc_height);
			break;
		case 'C':
			disparityCSBP(left_roi_image, right_roi_image, ROI_height, ROI_width, max_disparity_rescale, disparity_roi_map);
			break;
	}

	copy_raw_image_to_opencv_image(raw_disparity_rescale, depth_image_rescale, 1);
	cvSaveImage("dips.png", depth_image_rescale, 0);
	cvResize(depth_image_rescale, depth_image, CV_INTER_LINEAR);
	adjust_disparity_scale_from_opencv_image(depth_image, disparity_message.disparity);
}


void
rgb_to_gray(unsigned char *src, unsigned char *dst, int width, int height)
{
	int i;
	int n = width * height;

	for (i = 0; i < n; i++)
		dst[i] = round((src[3 * i] + src[3 * i + 1] + src[3 * i + 2]) / 3.0);
}


float
smooth_filter(float *right_disparity, int i, int j,
		int ROI_width, int vertical_ROI_ini, int vertical_ROI_end, int horizontal_ROI_ini, int horizontal_ROI_end,
		int kernel_size)
{
	float sum = 0.0;
	float count = 0.0;
	for (int y = -kernel_size / 2; y < kernel_size / 2; y++)
	{
		for (int x = -kernel_size / 2; x < kernel_size / 2; x++)
		{
			int ii = i + y;
			int jj = j + x;
			if ((right_disparity[ii * ROI_width + jj] > 0.0) && (ii >= vertical_ROI_ini) && (ii < vertical_ROI_end) && (jj >= horizontal_ROI_ini) && (jj < horizontal_ROI_end))
			{
				sum += right_disparity[ii * ROI_width + jj];
				count += 1.0;
			}
		}
	}

	if (count != 0.0)
	{
		float filtered_disparity = sum / count;
		return (filtered_disparity);
	}
	else
		return (0.0);
}


void
compute_depth_map(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	unsigned char *right_roi_image, *left_roi_image;
	float *disparity_roi_map;

	/*  copy_raw_image_to_opencv_image(stereo_image->raw_left, image_left_copy, 3);

	 cvSmooth(image_left_copy, image_left_copy, CV_GAUSSIAN, 11, 0, 0, 0);

	 copy_opencv_image_to_raw_image(image_left_copy, stereo_image->raw_left, 3);
	 copy_raw_image_to_opencv_image(stereo_imastereo_main.cppge->raw_right, image_right_copy, 3);

	 cvSmooth(image_right_copy, image_right_copy, CV_GAUSSIAN, 11, 0, 0, 0);

	 copy_opencv_image_to_raw_image(image_right_copy, stereo_image->raw_right , 3);
	 */
	left_roi_image = stereo_image->raw_left + vertical_ROI_ini * 3 * bumblebee_basic_width;
	right_roi_image = stereo_image->raw_right + vertical_ROI_ini * 3 * bumblebee_basic_width;
	disparity_roi_map = disparity_message.disparity + vertical_ROI_ini * bumblebee_basic_width;

	switch (algorithm[0])
	{
		case 'V':
			disparityVgRamWNN(left_roi_image, right_roi_image, ROI_height, ROI_width, max_disparity, disparity_roi_map, synapses, numthreads, wintakeiteration,
					inc_width, inc_height);
			break;
		case 'C':
			disparityCSBP(left_roi_image, right_roi_image, ROI_height, ROI_width, max_disparity, raw_disparity_rescale);
			convert_disparity_from_raw_image_to_float(raw_disparity_rescale, disparity_message.disparity);
			break;
/*		case 'E':
			rgb_to_gray(left_roi_image, gray_left_roi, ROI_width, ROI_height);
			rgb_to_gray(right_roi_image, gray_right_roi, ROI_width, ROI_height);

			const int32_t dims[3] = { ROI_width, ROI_height, ROI_width };
			Elas::parameters param;
			param.postprocess_only_left = false;
			Elas elas(param);
			elas.process(gray_left_roi, gray_right_roi, left_disparity, right_disparity, dims);

//			FILE *disp_right = fopen("/home/alberto/carmen_lcad/sharedlib/libelas/disp_rigth.bin", "r");
//			fread(right_disparity, sizeof(float), ROI_width*ROI_height, disp_right);
//			fclose(disp_right);

			float disparity;
			int i, j, x, y;
			for (i = 0, y = vertical_ROI_ini; y < vertical_ROI_end; y++, i++)
			{
				for (j = 0, x = horizontal_ROI_ini; x < horizontal_ROI_end; x++, j++)
				{
					if (smoothing_kernel_size  == 0)
						disparity = right_disparity[i * ROI_width + j];
					else
						disparity = smooth_filter(right_disparity, i, j, ROI_width, vertical_ROI_ini, vertical_ROI_end, horizontal_ROI_ini, horizontal_ROI_end,
								smoothing_kernel_size);

					disparity_message.disparity[y * bumblebee_basic_width + x] = MAX(disparity, 0.0);
				}
			}
			break;
*/	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_stereo_message()
{
	carmen_stereo_publish_message(camera, &disparity_message);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	if (stereo_scalew > bumblebee_basic_width && stereo_scalew > bumblebee_basic_height)
		compute_depth_map_rescale(stereo_image);
	else
		compute_depth_map(stereo_image);

	disparity_message.reference_image_size = stereo_image->image_size;

	disparity_message.reference_image = stereo_image->raw_right;
	disparity_message.timestamp = stereo_image->timestamp;

	publish_stereo_message();
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
stereo_algorithm_initialization()
{
	disparity_message.disparity_size = bumblebee_basic_height * bumblebee_basic_width;
	disparity_message.disparity = (float *) calloc((disparity_message.disparity_size), sizeof(float));
	disparity_message.host = carmen_get_host();

	inc_width = 1;
	inc_height = 1;

	kernel = cvCreateStructuringElementEx(2, 2, 0, 0, CV_SHAPE_RECT, NULL);

	if (stereo_scalew > bumblebee_basic_width && stereo_scaleh > bumblebee_basic_height)
	{
		vertical_ROI_ini = vertical_ROI_ini * ((float) stereo_scaleh / bumblebee_basic_height);
		vertical_ROI_end = vertical_ROI_end * ((float) stereo_scaleh / bumblebee_basic_height);

		horizontal_ROI_ini = horizontal_ROI_ini * ((float) stereo_scalew / bumblebee_basic_width);
		horizontal_ROI_end = horizontal_ROI_end * ((float) stereo_scalew / bumblebee_basic_width);

		ROI_height = vertical_ROI_end - vertical_ROI_ini;
		ROI_width = stereo_scalew;

		if (vertical_ROI_end > stereo_scaleh)
			vertical_ROI_end = stereo_scaleh;
		if (ROI_height < 0)
		{
			ROI_height = stereo_scaleh;
			printf("Warnning: The stereo_vertical_ROI_ini is bigger than stereo_vertical_ROI_end\n");
			sleep(3);
		}

		image_left_copy = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 3);
		image_right_copy = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 3);
		image_left_rescale = cvCreateImage(cvSize(stereo_scalew, stereo_scaleh), IPL_DEPTH_8U, 3);
		image_right_rescale = cvCreateImage(cvSize(stereo_scalew, stereo_scaleh), IPL_DEPTH_8U, 3);
		raw_image_left_rescale = (unsigned char *) calloc((stereo_scalew * stereo_scaleh) * 3, sizeof(unsigned char));
		raw_image_right_rescale = (unsigned char *) calloc((stereo_scalew * stereo_scaleh) * 3, sizeof(unsigned char));
		depth_image_rescale = cvCreateImage(cvSize(stereo_scalew, stereo_scaleh), IPL_DEPTH_8U, 1);
		raw_disparity_rescale = (unsigned char *) calloc((stereo_scalew * stereo_scaleh), sizeof(unsigned char));
		depth_image = cvCreateImage(cvSize(stereo_width, stereo_height),
		IPL_DEPTH_8U, 1);

		max_disparity_rescale = (int) (max_disparity);  // * (stereo_scalew  / (double) bumblebee_basic_width));

		switch (algorithm[0])
		{
			case 'V':
				WNNInitialize(stereo_scaleh, stereo_scalew, max_disparity, numdisparity, numthreads, synapses, gaussian_radius, horizontal_ROI_ini,
						horizontal_ROI_end, lambda);
				break;
			case 'C':
				init_stereo(stereo_scaleh, stereo_scalew, max_disparity,
				QX_DEF_BP_NR_PLANE,
				QX_DEF_BP_COST_DISCONTINUITY_SINGLE_JUMP,
				QX_DEF_BP_COST_MAX_DATA_TERM, QX_DEF_BP_MAX_NR_JUMP,
				QX_DEF_BP_NR_SCALES, NULL);
				break;
		}
	}
	else
	{
		inc_width = (int) (((double) bumblebee_basic_width / stereo_scalew));
		inc_height = (int) (((double) bumblebee_basic_height / stereo_scaleh));
		raw_disparity_rescale = (unsigned char *) calloc((bumblebee_basic_width * bumblebee_basic_height), sizeof(unsigned char));
		printf("stereo_scalew = %d \n, ", inc_width);

		image_left_copy = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 3);
		image_right_copy = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 3);

		depth_image_rescale = cvCreateImage(cvSize(stereo_scalew, stereo_scaleh),
		IPL_DEPTH_8U, 1);

		depth_image = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 1);

		ROI_height = vertical_ROI_end - vertical_ROI_ini;
		ROI_width = bumblebee_basic_width;

		if (ROI_height < 0)
		{
			ROI_height = bumblebee_basic_height;
			printf("Warnning: The stereo_vertical_ROI_ini is bigger than stereo_vertical_ROI_end\n");
			sleep(3);
		}

		switch (algorithm[0])
		{
			case 'V':
				WNNInitialize(bumblebee_basic_height, bumblebee_basic_width, max_disparity, numdisparity, numthreads, synapses, gaussian_radius,
						horizontal_ROI_ini, horizontal_ROI_end, lambda);
				break;
			case 'C':

				init_stereo(bumblebee_basic_height, bumblebee_basic_width, max_disparity, QX_DEF_BP_NR_PLANE,
				QX_DEF_BP_COST_DISCONTINUITY_SINGLE_JUMP,
				QX_DEF_BP_COST_MAX_DATA_TERM, QX_DEF_BP_MAX_NR_JUMP,
				QX_DEF_BP_NR_SCALES, NULL);
				break;
			case 'E':
				left_disparity = (float*) malloc(ROI_width * ROI_height * sizeof(float));
				right_disparity = (float*) malloc(ROI_width * ROI_height * sizeof(float));
				gray_left_roi = (unsigned char*) malloc(ROI_width * ROI_height * sizeof(unsigned char));
				gray_right_roi = (unsigned char*) malloc(ROI_width * ROI_height * sizeof(unsigned char));
				break;
		}
	}
}


void
subscribe_bumblebee_basic_messages(int camera_organization)
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera_organization, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
read_parameters(int argc, char **argv)
{
	int num_items;

	char bumblebee_string[256];
	char stereo_string[256];

	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", atoi(argv[1]));
	sprintf(stereo_string, "%s%d", "stereo", atoi(argv[1]));

	carmen_param_t param_list[] = {
		{ stereo_string, (char*) "width", CARMEN_PARAM_INT, &stereo_width, 0, NULL },
		{ stereo_string, (char*) "height", CARMEN_PARAM_INT, &stereo_height, 0, NULL },
		{ stereo_string, (char*) "max_disparity", CARMEN_PARAM_INT, &max_disparity, 0, NULL },
		{ stereo_string, (char*) "smoothing_kernel_size", CARMEN_PARAM_INT, &smoothing_kernel_size, 0, NULL },
		{ stereo_string, (char*) "algorithm", CARMEN_PARAM_STRING, &algorithm, 0, NULL },
		{ stereo_string, (char*) "gaussian_radius", CARMEN_PARAM_DOUBLE, &gaussian_radius, 0, NULL },
		{ stereo_string, (char*) "synapses", CARMEN_PARAM_INT, &synapses, 0, NULL },
		{ stereo_string, (char*) "wintakeiteration", CARMEN_PARAM_INT, &wintakeiteration, 0, NULL },
		{ stereo_string, (char*) "numdisparity", CARMEN_PARAM_INT, &numdisparity, 0, NULL },
		{ stereo_string, (char*) "numthreads", CARMEN_PARAM_INT, &numthreads, 0, NULL },
		{ stereo_string, (char*) "scalew", CARMEN_PARAM_INT, &stereo_scalew, 0, NULL },
		{ stereo_string, (char*) "scaleh", CARMEN_PARAM_INT, &stereo_scaleh, 0, NULL },
		{ stereo_string, (char*) "vertical_ROI_ini", CARMEN_PARAM_INT, &vertical_ROI_ini, 0, NULL },
		{ stereo_string, (char*) "vertical_ROI_end", CARMEN_PARAM_INT, &vertical_ROI_end, 0, NULL },
		{ stereo_string, (char*) "horizontal_ROI_ini", CARMEN_PARAM_INT, &horizontal_ROI_ini, 0, NULL },
		{ stereo_string, (char*) "horizontal_ROI_end", CARMEN_PARAM_INT, &horizontal_ROI_end, 0, NULL },
		{ bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL },
		{ bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL }
	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return (0);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	if ((argc != 2) && (argc != 3))
		carmen_die(
				"%s: Wrong number of parameters. stereo requires either 1 or 2 parameters and received %d parameter(s). \nUsage:\n %s <camera_number> or\n %s <camera_number[LR]> <camera_number[LR]>",
				argv[0], argc - 1, argv[0], argv[0]);

	camera = atoi(argv[1]);

	read_parameters(argc, argv);

	carmen_stereo_define_messages(camera);

	subscribe_bumblebee_basic_messages(camera);

	stereo_algorithm_initialization();

	carmen_ipc_dispatch();

	return (0);
}
