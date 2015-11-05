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
    printf("Vergence Bumblebee: disconnected.\n");
    exit(0);
  }
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
format_message(carmen_bumblebee_basic_stereoimage_message *message, int image_width, int image_height, int image_channels)
{
	message->timestamp = carmen_get_time();
	message->host = carmen_get_host();

	message->height = image_height;
	message->width = image_width;

	message->image_size = image_height * image_width * image_channels;

	if (message->raw_left == NULL)
		message->raw_left = calloc(message->image_size, sizeof(unsigned char));

	if (message->raw_right == NULL)
		message->raw_right = calloc(message->image_size, sizeof(unsigned char));

}


/**
 * Command line parameters:
 * <camera>
 * <image_left>
 * <image_right>
 */
int
main(int argc, char **argv)
{
	int camera = 0;

	carmen_bumblebee_basic_stereoimage_message bumblebee;
	bumblebee.raw_left = NULL;
	bumblebee.raw_right = NULL;

	if (argc < 3)
		carmen_die("%s: Wrong number of parameters. \nUsage:\n %s <camera> path/to/image-left path/to/image-right \n", argv[0], argv[0]);

	if (argc > 2)
		camera = atof(argv[1]);

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	carmen_bumblebee_basic_define_messages(camera);

	IplImage *image_left = cvLoadImage(argv[2], 1);
	IplImage *image_right = cvLoadImage(argv[3], 1);

	format_message(&bumblebee, image_left->width, image_left->height, image_left->nChannels);

	copy_message(bumblebee.raw_left, image_left, image_left->nChannels);
	copy_message(bumblebee.raw_right, image_right, image_right->nChannels);

	while(1)
	{
		carmen_bumblebee_basic_publish_message(camera, &bumblebee);
		sleep(1);
	}

	cvReleaseImage(&image_right);
	cvReleaseImage(&image_left);

	return (0);
}
