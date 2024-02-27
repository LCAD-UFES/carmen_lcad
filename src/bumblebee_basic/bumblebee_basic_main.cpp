/*********************************************************
 * Bumblebee2 Camera Module
 **********************************************************/

#include <carmen/carmen.h>
#include <libbee.hpp>
#include <carmen/bumblebee_basic_interface.h>
#include <sys/time.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#define LEFT 0
#define RIGHT 1

static carmen_bumblebee_basic_stereoimage_message msg;

static int bumblebee_basic_width;
static int bumblebee_basic_height;
static int bumblebee_basic_is_rectified;
static int bumblebee_basic_is_legacy;

void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int bytes_per_pixel, int isRectified, int camera)
{
  msg.host = carmen_get_host();
  msg.image_size = width*height*bytes_per_pixel;
  msg.width = width;
  msg.isRectified = isRectified;
  msg.height = height;
  msg.raw_left = rawLeft;
  msg.raw_right = rawRight;

  carmen_bumblebee_basic_publish_message(camera, &msg);
}

unsigned long long int CameraContextByGuid(int camera)
{
  unsigned long long int guid = 0;

  switch(camera)
  {
  case 1:
    guid = 49712223527926590ll;
    break;

  case 2:
    guid = 49712223531755284ll;
    break;

  case 3:
    guid = 49712223533115246ll;
    break;

  case 4:
    guid = 49712223533115251ll;
    break;

  case 5:
    guid = 49712223533115245ll;
    break;

  case 6:
    guid = 49712223533115244ll;
    break;

  case 7:
    guid = 49712223533115250ll;
    break;

  case 8:
    guid = 49712223533115248ll;
    break;

  case 9:
    guid = 49712223532964068ll;
    break;

  default:
    guid = -1;
    break;
  }

  return guid;
}


void InterpolateTriclopsStereoImage(unsigned char* outImageLeft, TriclopsColorImage *inputImageLeft,
							   unsigned char* outImageRight, TriclopsColorImage *inputImageRight,
								int width, int height)
{
     for (int i = 0; i < width*height; i++)
     {
        outImageLeft[3*i+0] = inputImageLeft->red[i];
        outImageLeft[3*i+1] = inputImageLeft->green[i];
        outImageLeft[3*i+2] = inputImageLeft->blue[i];

        outImageRight[3*i+0] = inputImageRight->red[i];
		outImageRight[3*i+1] = inputImageRight->green[i];
		outImageRight[3*i+2] = inputImageRight->blue[i];
     }
}


unsigned char *
crop_raw_image(int image_width, int image_height, unsigned char *raw_image, int displacement_x, int displacement_y, int crop_width, int crop_height)
{
	unsigned char *cropped_image = (unsigned char *) malloc (crop_width * crop_height * 3 * sizeof(unsigned char));  // Only works for 3 channels image

	displacement_x = (displacement_x - 2) * 3;
	displacement_y = (displacement_y - 2) * image_width * 3;
	crop_width     = displacement_x + ((crop_width + 1) * 3);
	crop_height    = displacement_y + ((crop_height + 1) * image_width * 3);
	image_height   = image_height * image_width * 3;
	image_width   *= 3;

	for (int line = 0, index = 0; line < image_height; line += image_width)
	{
		for (int column = 0; column < image_width; column += 3)
		{
			if (column > displacement_x && column < crop_width && line > displacement_y && line < crop_height)
			{
				cropped_image[index]     = raw_image[line + column];
				cropped_image[index + 1] = raw_image[line + column + 1];
				cropped_image[index + 2] = raw_image[line + column + 2];

				index += 3;
			}
		}
	}

	return (cropped_image);
}


static int
read_parameters(int argc, char **argv, int camera)
{
  int num_items;
  char bumblebee_string[256];

  sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

  carmen_param_t param_list[] = {
    {bumblebee_string, (char*)"width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
    {bumblebee_string, (char*)"height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
    {bumblebee_string, (char*)"is_rectified", CARMEN_PARAM_ONOFF, &bumblebee_basic_is_rectified, 0, NULL},
    {bumblebee_string, (char*)"is_legacy", CARMEN_PARAM_ONOFF, &bumblebee_basic_is_legacy, 0, NULL},
    };
	

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  return 0;
}

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
     carmen_ipc_disconnect();
     printf("bumblebee_basic was disconnected.\n");
     libbbee_terminate_camera_context();
     exit(0);
  }
}

int main(int argc, char **argv)
{
  TriclopsColorImage *imageLeft = new(TriclopsColorImage);
  TriclopsColorImage *imageRight = new(TriclopsColorImage);
  int camera = 0;
  int fps = 0;
  double timestamp_fps = 0.0;

  /* connect to IPC server */
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  if (argc != 2)
    carmen_die("%s: Wrong number of parameters. %s requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number>\n", argv[0], argv[0], argc-1, argv[0]);

  camera = atoi(argv[1]);

  printf("%d \n", camera);

  if (camera == 2)
    sleep(2);

  read_parameters(argc, argv, camera);

  signal(SIGINT, shutdown_module);

  carmen_bumblebee_basic_define_messages(camera);

  carmen_param_allow_unfound_variables(0);

  libbbee_initialize_camera_context(CameraContextByGuid(camera), bumblebee_basic_width, bumblebee_basic_height, bumblebee_basic_is_rectified, bumblebee_basic_is_legacy);

  unsigned char *rawLeft = (unsigned char *) malloc ((bumblebee_basic_width * bumblebee_basic_height * 3) * sizeof(unsigned char));
  unsigned char *rawRight = (unsigned char *) malloc ((bumblebee_basic_width * bumblebee_basic_height * 3) * sizeof(unsigned char));

  if (bumblebee_basic_is_rectified)
  {
		while(1)
		{
			memset(rawLeft, 0, (bumblebee_basic_width * bumblebee_basic_height * 3) * sizeof(unsigned char));
			memset(rawRight, 0, (bumblebee_basic_width * bumblebee_basic_height * 3) * sizeof(unsigned char));

			libbee_get_rectified_images(imageLeft, imageRight, &(msg.timestamp));

			InterpolateTriclopsStereoImage(rawLeft, imageLeft,rawRight, imageRight, bumblebee_basic_width, bumblebee_basic_height);

			fps++;

			if (fabs(msg.timestamp - timestamp_fps) >= 1.0)
			{
				printf("FPS (Rectified): %d %lf\n", fps, msg.timestamp - timestamp_fps);
				timestamp_fps = msg.timestamp;
				fps = 0.0;
			}

//			char nome[256];
//			static int count = 0;
//			cv::Mat frame = cv::Mat(cv::Size(bumblebee_basic_width, bumblebee_basic_height), CV_8UC3, rawLeft, bumblebee_basic_width * 3);
//			sprintf(nome, "%dimage.png", count);
//			count++;
//			cv::imwrite(nome, frame);

			//usleep(5000000);
		//	static double time_lastmsg = carmen_get_time();
		//	if (fabs(time_lastmsg - carmen_get_time()) > 1.0)
		//	{
		//		time_lastmsg = carmen_get_time();

				if (imageLeft->green != NULL && imageRight->green != NULL) {
					carmen_bumblebee_publish_stereoimage_message(rawLeft, rawRight, bumblebee_basic_width, bumblebee_basic_height, 3, 1, camera);
				}

		//	}  
	

		}
  }
  else
  {
		while(1)
		{
			libbee_get_raw_images(imageLeft, imageRight, &(msg.timestamp));
			fps++;

			if(fabs(msg.timestamp - timestamp_fps) >= 1.0)
			{
				printf("FPS: %d\n", fps);
				timestamp_fps = msg.timestamp;
				fps = 0.0;
			}

			InterpolateTriclopsStereoImage(rawLeft, imageLeft,rawRight, imageRight, bumblebee_basic_width, bumblebee_basic_height);

			if (imageLeft->green != NULL && imageRight->green != NULL) {
				carmen_bumblebee_publish_stereoimage_message(rawLeft, rawRight, bumblebee_basic_width, bumblebee_basic_height, 3, 1, camera);
			}

			free(imageLeft->blue);
			free(imageLeft->green);
			free(imageLeft->red);

			free(imageRight->blue);
			free(imageRight->green);
			free(imageRight->red);
		}
  }

  return 0;
}
