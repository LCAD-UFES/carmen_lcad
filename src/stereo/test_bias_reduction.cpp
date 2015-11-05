#include <carmen/carmen.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_interface.h>
#include <carmen/bumblebee_basic_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

char *left_image_filename, *right_image_filename;
double baseline, focal_length;
int camera_index, stereo_message_received;
IplImage *left_image, *right_image, *disparity_image;
float **disparity;
int num_points_filled;
int pixels[2][2];
double bias_std_deviation;

typedef struct
{
	IplImage *left_image_reduced;
	IplImage *right_image_reduced;
}ProgramDataStruct;

ProgramDataStruct program_data;

int
parse_arguments(int argc, char **argv)
{
	if (argc < 7)
		return 0;
	else
	{
		// read the arguments
		left_image_filename = argv[1];
		right_image_filename = argv[2];

		baseline = atof(argv[3]);
		focal_length = atof(argv[4]);

		camera_index = atoi(argv[5]);
		bias_std_deviation = atof(argv[6]);

		// try open the images
		left_image = cvLoadImage(left_image_filename, CV_LOAD_IMAGE_COLOR);
		right_image = cvLoadImage(right_image_filename, CV_LOAD_IMAGE_COLOR);

		if (left_image == NULL)
			exit(printf("image '%s' not found", left_image_filename));

		if (right_image == NULL)
			exit(printf("image '%s' not found", left_image_filename));

		return 1;
	}
}


void
show_usage_information_and_exit(int argc, char **argv)
{
	// this parameter is not used here...
	(void) argc;

	printf("\n");
	printf("\nUse %s <left-image-filename> <right-image-filename> <baseline> <focal-lenght> <camera-number> <std-dev-bias>\n", argv[0]);
	printf("\n");

	exit(-1);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("stereo test: disconnected.\n");

		exit(0);
	}
}


double
get_disparity(carmen_simple_stereo_disparity_message *message, int i, int j)
{
	int s = 10;
	double disp = 0;
	int num_points_visited = 0;

	for(int m = (i - s); m < (i + s); m++)
	{
		if (m < 0 || m >= right_image->height)
			continue;

		for(int n = (j - s); n < (j + s); n++)
		{
			if (n < 0 || n >= right_image->width)
				continue;

			int p = m * right_image->width + n;
			disp += (double) message->disparity[p];

			num_points_visited++;
		}
	}

	return disp / (double) num_points_visited;
}


void
stereo_handler(carmen_simple_stereo_disparity_message *message)
{
//	int disparity_size;   		/* disparity_width * disparity_height */
//	float *disparity;
//	int reference_image_size;		/* input_image_width * input_image_height */
//	unsigned char *reference_image;
//	double timestamp;
//	char *host;

	double max_disp = 0, min_disp = 0, disp;
	int first = 1;

	disparity_image = cvCreateImage(cvSize(right_image->width, right_image->height), IPL_DEPTH_8U, 1);
	disparity = (float **) calloc (right_image->height, sizeof(float *));

	for(int i = 0; i < right_image->height; i++)
	{
		disparity[i] = (float *) calloc (right_image->width, sizeof(float *));

		for(int j = 0; j < right_image->width; j++)
		{
			int p2 = i * disparity_image->widthStep + j;

			//disp = get_disparity(message, i, j);
			disp = message->disparity[i * right_image->width + j];

			disparity_image->imageData[p2] = ((disp / 36) * 255.0);
			disparity[i][j] = disp;

			if (disp > max_disp || first)
			{
				max_disp = disp;
				first = 0;
			}
			if (disp < min_disp || first)
			{
				min_disp = disp;
				first = 0;
			}
		}
	}

	printf("max_disp: %lf min_disp: %lf\n", max_disp, min_disp);
	stereo_message_received = 1;
}


void
initialize_carmen_module(int argc, char **argv, int camera)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_stereo_define_messages(camera);
	carmen_bumblebee_basic_define_messages(camera);
	carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) stereo_handler, CARMEN_SUBSCRIBE_ALL);
}


carmen_bumblebee_basic_stereoimage_message*
generate_bumblebee_message()
{
//	typedef struct {
//	  int width;                    /**<The x dimension of the image in pixels. */
//	  int height;                   /**<The y dimension of the image in pixels. */
//	  int image_size;              /**<width*height*bytes_per_pixel. */
//	  int isRectified;
//	  unsigned char *raw_left;
//	  unsigned char *raw_right;
//	  double timestamp;
//	  char *host;
//	} carmen_bumblebee_basic_stereoimage_message;

	carmen_bumblebee_basic_stereoimage_message *bumblebee_message = (carmen_bumblebee_basic_stereoimage_message *) calloc (1, sizeof(carmen_bumblebee_basic_stereoimage_message));
	carmen_test_alloc(bumblebee_message);

	bumblebee_message->width = left_image->width;
	bumblebee_message->height = left_image->height;
	bumblebee_message->image_size = left_image->imageSize;
	bumblebee_message->isRectified = 1;
	bumblebee_message->raw_left = (unsigned char *) left_image->imageData;
	bumblebee_message->raw_right = (unsigned char *) right_image->imageData;
	bumblebee_message->timestamp = carmen_get_time();
	bumblebee_message->host = carmen_get_host();

	return bumblebee_message;
}


void
publish_bumblebee_message(int camera, carmen_bumblebee_basic_stereoimage_message* bumblebee_message)
{
	carmen_bumblebee_basic_publish_message(camera, bumblebee_message);
}


void
perform_stereo(int camera)
{
	stereo_message_received = 0;

	carmen_bumblebee_basic_stereoimage_message* stereo_message = generate_bumblebee_message();
	publish_bumblebee_message(camera, stereo_message);

	while(!stereo_message_received)
		carmen_ipc_sleep(0.1);
}


void
draw_pixel(IplImage *img, int x, int y)
{
	cvCircle(img, cvPoint(x, y), 3, cvScalar(0, 0, 255, 0), 1, 1, 0);
}


void
compute_3D_points_with_bias()
{
	carmen_position_t left_pixel, right_pixel;

	left_pixel.x = pixels[0][0] + disparity[pixels[0][0]][pixels[0][1]];
	left_pixel.y = pixels[0][1];
	right_pixel.x = pixels[0][0];
	right_pixel.y = pixels[0][1];

	carmen_vector_3D_p point_3D_1 = reproject_single_point_to_3D_in_left_camera_frame(
		left_pixel,
		right_pixel,
		baseline, focal_length
	);

	left_pixel.x = pixels[1][0] + disparity[pixels[1][0]][pixels[1][1]];
	left_pixel.y = pixels[1][1];
	right_pixel.x = pixels[1][0];
	right_pixel.y = pixels[1][1];

	carmen_vector_3D_p point_3D_2 = reproject_single_point_to_3D_in_left_camera_frame(
		left_pixel,
		right_pixel,
		baseline, focal_length
	);

	if ((point_3D_1 != NULL) && (point_3D_2 != NULL))
	{
		double dist = sqrt(
			pow(point_3D_1->x - point_3D_2->x,2) +
			pow(point_3D_1->y - point_3D_2->y,2) +
			pow(point_3D_1->z - point_3D_2->z,2)
		);

		printf("dist with bias: %lf\n", dist);
//		printf("\tpoint_3D_1\tdisp\t%lf\t%lf\t%lf\t%lf\n", disparity[pixels[0][0]][pixels[0][1]], point_3D_1->x, point_3D_1->y, point_3D_1->z);
//		printf("\tpoint_3D_2\tdisp\t%lf\t%lf\t%lf\t%lf\n", disparity[pixels[1][0]][pixels[1][1]], point_3D_2->x, point_3D_2->y, point_3D_2->z);

		free(point_3D_1);
		free(point_3D_2);
	}
	else
		printf("Error: point_3D_1 = %p and point_3D_2 = %p\n", point_3D_1, point_3D_2);

}


void
compute_3D_points_without_bias(double bias_std_deviation)
{
	carmen_position_t left_pixel, right_pixel;

	left_pixel.x = pixels[0][0] + disparity[pixels[0][0]][pixels[0][1]];
	left_pixel.y = pixels[0][1];
	right_pixel.x = pixels[0][0];
	right_pixel.y = pixels[0][1];

	carmen_vector_3D_p point_3D_1 = reproject_single_point_to_3D_in_left_camera_frame_with_bias_reduction(
		left_pixel,
		right_pixel,
		baseline, focal_length,
		bias_std_deviation
	);

	left_pixel.x = pixels[1][0] + disparity[pixels[1][0]][pixels[1][1]];
	left_pixel.y = pixels[1][1];
	right_pixel.x = pixels[1][0];
	right_pixel.y = pixels[1][1];

	carmen_vector_3D_p point_3D_2 = reproject_single_point_to_3D_in_left_camera_frame_with_bias_reduction(
		left_pixel,
		right_pixel,
		baseline, focal_length,
		bias_std_deviation
	);

	if ((point_3D_1 != NULL) && (point_3D_2 != NULL))
	{
		double dist_without_bias = sqrt(
			pow(point_3D_1->x - point_3D_2->x,2) +
			pow(point_3D_1->y - point_3D_2->y,2) +
			pow(point_3D_1->z - point_3D_2->z,2)
		);

		printf("dist without bias: %lf\n", dist_without_bias);
//		printf("\tpoint_3D_1\tdisp\t%lf\t%lf\t%lf\t%lf\n", disparity[pixels[0][0]][pixels[0][1]], point_3D_1->x, point_3D_1->y, point_3D_1->z);
//		printf("\tpoint_3D_2\tdisp\t%lf\t%lf\t%lf\t%lf\n", disparity[pixels[1][0]][pixels[1][1]], point_3D_2->x, point_3D_2->y, point_3D_2->z);

		free(point_3D_1);
		free(point_3D_2);
	}
	else
		printf("Error: point_3D_1 = %p and point_3D_2 = %p\n", point_3D_1, point_3D_2);
}


void
compute_3D_from_carmen()
{
	stereo_util stereo_utility = get_stereo_instance(camera_index, right_image->width, right_image->height);
	carmen_position_t right_pixel;

	right_pixel.x = pixels[0][0];
	right_pixel.y = pixels[0][1];

	carmen_vector_3D_p point_3D_1 = reproject_single_point_to_3D(
		&stereo_utility,
		right_pixel,
		disparity[pixels[0][0]][pixels[0][1]]
	);

	right_pixel.x = pixels[1][0];
	right_pixel.y = pixels[1][1];

	carmen_vector_3D_p point_3D_2 = reproject_single_point_to_3D(
		&stereo_utility,
		right_pixel,
		disparity[pixels[1][0]][pixels[1][1]]
	);

	if ((point_3D_1 != NULL) && (point_3D_2 != NULL))
	{
		double dist_from_carmen = sqrt(
			pow(point_3D_1->x - point_3D_2->x,2) +
			pow(point_3D_1->y - point_3D_2->y,2) +
			pow(point_3D_1->z - point_3D_2->z,2)
		);

		printf("dist from carmen: %lf\n", dist_from_carmen);
//		printf("\tpoint_3D_1\tdisp\t%lf\t%lf\t%lf\t%lf\n", disparity[pixels[0][0]][pixels[0][1]], point_3D_1->x, point_3D_1->y, point_3D_1->z);
//		printf("\tpoint_3D_2\tdisp\t%lf\t%lf\t%lf\t%lf\n", disparity[pixels[1][0]][pixels[1][1]], point_3D_2->x, point_3D_2->y, point_3D_2->z);

		free(point_3D_1);
		free(point_3D_2);
	}
	else
		printf("Error: point_3D_1 = %p and point_3D_2 = %p\n", point_3D_1, point_3D_2);
}


void
compute_distance_between_points_and_draw()
{
	compute_3D_from_carmen();
	compute_3D_points_with_bias();
	compute_3D_points_without_bias(bias_std_deviation);

	cvLine(program_data.right_image_reduced,
		cvPoint(pixels[0][0], pixels[0][1]),
		cvPoint(pixels[1][0], pixels[1][1]),
		cvScalar(0, 0, 255, 0), 1, 1, 0
	);

	cvLine(program_data.left_image_reduced,
		cvPoint(pixels[0][0] + disparity[pixels[0][0]][pixels[0][1]], pixels[0][1]),
		cvPoint(pixels[1][0] + disparity[pixels[1][0]][pixels[1][1]], pixels[1][1]),
		cvScalar(0, 0, 255, 0), 1, 1, 0
	);
}


void
mouse_callback_function(int event, int x, int y, int flags, void *param)
{
	// this parameters are not used here...
	(void) flags;
	(void) param;

	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:
		{
			if (num_points_filled == 2)
			{
				cvResize(left_image, program_data.left_image_reduced, CV_INTER_CUBIC);
				cvResize(right_image, program_data.right_image_reduced, CV_INTER_CUBIC);

				num_points_filled = 0;
			}

			pixels[num_points_filled][0] = x;
			pixels[num_points_filled][1] = y;

			draw_pixel(program_data.right_image_reduced, x, y);
			draw_pixel(program_data.left_image_reduced, x + disparity[x][y], y);

			num_points_filled++;

			if (num_points_filled == 2)
				compute_distance_between_points_and_draw();

			break;
		}
		default:
			break;
	}
}


void
point_calculation_loop()
{
	num_points_filled = 0;

	IplImage *left_reduced = cvCreateImage(cvSize(left_image->width/2, left_image->height/2), left_image->depth, left_image->nChannels);
	IplImage *right_reduced = cvCreateImage(cvSize(right_image->width/2, right_image->height/2), right_image->depth, right_image->nChannels);
	IplImage *disparity_reduced = cvCreateImage(cvSize(disparity_image->width/2, disparity_image->height/2), disparity_image->depth, disparity_image->nChannels);

	cvResize(left_image, left_reduced, CV_INTER_CUBIC);
	cvResize(right_image, right_reduced, CV_INTER_CUBIC);
	cvResize(disparity_image, disparity_reduced, CV_INTER_CUBIC);

	program_data.left_image_reduced = left_reduced;
	program_data.right_image_reduced = right_reduced;

	cvNamedWindow("disparity", 1);
	cvSetMouseCallback("disparity", mouse_callback_function, (void*) NULL);

	while(1)
	{
		cvShowImage("left", left_reduced);
		cvShowImage("right", right_reduced);
		cvShowImage("disparity", disparity_reduced);

		if ((cvWaitKey(10) & 255) == 27) break;
	}
}


int
main(int argc, char **argv)
{
	if (!parse_arguments(argc, argv))
		show_usage_information_and_exit(argc, argv);

	initialize_carmen_module(argc, argv, camera_index);
	perform_stereo(camera_index);
	point_calculation_loop();

	return 0;
}

