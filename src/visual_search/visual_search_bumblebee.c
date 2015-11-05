#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("Visual Tracker Bumblebee: disconnected.\n");
    exit(0);
  }
}

/*void
read_parameters(int argc, char** argv)
{
}
*/

void
publish_message(carmen_bumblebee_basic_stereoimage_message *message)
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE2_NAME);
}

void
copy_message(unsigned char *message, const IplImage *image, int nchannels)
{
	int i, j;

	for(i = 0; i < image->height; i++)
	{
		unsigned char* image_data = (unsigned char*)image->imageData + (i*image->widthStep);
		if(nchannels==3)
			for(j = 0; j < image->width; j++)
			{
				message[nchannels*(i*image->width+j)+0] = image_data[nchannels*j+2];
				message[nchannels*(i*image->width+j)+1] = image_data[nchannels*j+1];
				message[nchannels*(i*image->width+j)+2] = image_data[nchannels*j+0];
			}
		else
			for(j = 0; j < image->width; j++)
			{
				message[i*image->width+j] = image_data[j];
			}
	}
}

void
format_message(carmen_bumblebee_basic_stereoimage_message *message, const IplImage *image)
{
	message->timestamp = carmen_get_time();
	message->host = carmen_get_host();

	message->height = image->height;
	message->width = image->width;

	message->image_size = image->height * image->width * image->nChannels;

	if (message->raw_left == NULL)
		message->raw_left = calloc(message->image_size, sizeof(unsigned char));

	if (message->raw_right == NULL)
		message->raw_right = calloc(message->image_size, sizeof(unsigned char));

	copy_message(message->raw_left, image, image->nChannels);
}

void
format_message_right(carmen_bumblebee_basic_stereoimage_message *message, const IplImage *image)
{
	copy_message(message->raw_right, image, image->nChannels);
}

void
define_messages()
{
	carmen_bumblebee_basic_define_messages(2);
}

/**
 * Command line parameters:
 * <IMAGE PATH>
 * /media/storage/lcad/logs/TLD/06_car/%05d.jpg (320x240)
 * /media/storage/lcad/logs/TLD/09_carchase/%05d.jpg (290x217)
 */
int
main(int argc, char **argv)
{
	float FPS = 30.0;
	int current_frame = 0;
	char current_path[255];
	carmen_bumblebee_basic_stereoimage_message bumblebee;
	bumblebee.raw_left = NULL;
	bumblebee.raw_right = NULL;

	if (argc < 2)
		carmen_die("%s: Wrong number of parameters. \nUsage:\n %s path/to/image-%%05d.bmp \n", argv[0], argv[0]);

	char *image_path = argv[1];


	if (argc > 2)
		FPS = atof(argv[2]);

	carmen_ipc_initialize(argc, argv);

	//carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	//read_parameters(argc, argv);

	define_messages();

	while(1)
	{
		sprintf(current_path, image_path, ++current_frame);
		printf("CURRENT_PATH: %s\n", current_path);

//		IplImage *image_right = cvLoadImage(argv[4], 1);
//		IplImage *image_left = cvLoadImage(argv[3], 1);
		IplImage *image_right = cvLoadImage(current_path, 1);
		IplImage *image_left = cvLoadImage(current_path, 1);

		if (image_right == NULL)
			break;

		format_message(&bumblebee, image_left);

		format_message_right(&bumblebee, image_right);

		publish_message(&bumblebee);

		cvReleaseImage(&image_right);
		cvReleaseImage(&image_left);

		usleep((useconds_t)(1000000 * 1.0/FPS));
	}

	return (0);
}
