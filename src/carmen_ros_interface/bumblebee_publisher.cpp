#include <carmen/carmen.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/bumblebee_basic_interface.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "bumblebee_basic_stereoimage.h"

using namespace sensor_msgs;

static bumblebee_basic::bumblebee_basic_stereoimage msg;
static Image left_raw_msg;
static Image right_raw_msg;

ros::Publisher bumblebee_basic_publisher;
ros::Publisher left_image_publisher;
ros::Publisher right_image_publisher;

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		ros::shutdown();

		printf("bumblebee_publisher: disconnected.\n");
		exit(0);
	}
}


void
copy_raw_image(unsigned char *raw_image_copy, unsigned char *raw_image, int bumblebee_basic_width, int bumblebee_basic_height)
{
	int x, y, r, g, b;

	for (y = 0; y < bumblebee_basic_height; y++)
	{
		for (x = 0; x < bumblebee_basic_width; x++)
		{
			r = raw_image[3 * (y * bumblebee_basic_width + x) + 0];
			g = raw_image[3 * (y * bumblebee_basic_width + x) + 1];
			b = raw_image[3 * (y * bumblebee_basic_width + x) + 2];

			raw_image_copy[3 * (y * bumblebee_basic_width + x) + 0] = r;
			raw_image_copy[3 * (y * bumblebee_basic_width + x) + 1] = g;
			raw_image_copy[3 * (y * bumblebee_basic_width + x) + 2] = b;
		}
	}
}


void
carmen_bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	printf("bumblebee message received... ");

	int width = message->width;
	int height = message->height;
	int bytes_per_pixel = (message->image_size) / (width * height);

	unsigned char *rawLeft = message->raw_left;
	unsigned char *rawRight = message->raw_right;

	msg.header.frame_id.assign("bumblebee_stereo_images");
	left_raw_msg.header.frame_id.assign("bumblebee_left_raw_image");
	right_raw_msg.header.frame_id.assign("bumblebee_right_raw_image");

	msg.header.stamp = ros::Time::now();
	left_raw_msg.header.stamp = ros::Time::now();
	right_raw_msg.header.stamp = ros::Time::now();

	msg.image_size = width*height*bytes_per_pixel;

	msg.width = width;
	left_raw_msg.width = width;
	right_raw_msg.width = width;

	left_raw_msg.encoding = "rgb8";
	right_raw_msg.encoding = "rgb8";

	msg.isRectified = 1;

	msg.height = height;
	left_raw_msg.height = height;
	right_raw_msg.height = height;

	left_raw_msg.is_bigendian = 1;
	right_raw_msg.is_bigendian = 1;

	left_raw_msg.step = width * bytes_per_pixel;
	right_raw_msg.step = width * bytes_per_pixel;

	msg.raw_left.resize(width*height*bytes_per_pixel);
	msg.raw_right.resize(width*height*bytes_per_pixel);
	left_raw_msg.data.resize(width * height * bytes_per_pixel);
	right_raw_msg.data.resize(width * height * bytes_per_pixel);

	copy_raw_image(msg.raw_left.data(), rawLeft, width, height);
	copy_raw_image(msg.raw_right.data(), rawRight, width, height);
	copy_raw_image(left_raw_msg.data.data(), rawLeft, width, height);
	copy_raw_image(right_raw_msg.data.data(), rawRight, width, height);

	bumblebee_basic_publisher.publish(msg);
	left_image_publisher.publish(left_raw_msg);
	right_image_publisher.publish(right_raw_msg);

	printf("Published\n");
}


int
main(int argc, char **argv)
{
	int witch_camera;

	if (argc < 2)
		exit(printf("Use %s <camera-id>\n", argv[0]));

	witch_camera = atoi(argv[1]);

	IPC_initialize();
	IPC_connect(argv[0]);

	signal(SIGINT, shutdown_module);

	ros::init(argc, argv, "bumblebee_publisher", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	left_image_publisher = nh.advertise<sensor_msgs::Image>("/bumblebee_basic/left/image_raw", 1);
	right_image_publisher = nh.advertise<sensor_msgs::Image>("/bumblebee_basic/right/image_raw", 1);
	bumblebee_basic_publisher = nh.advertise<bumblebee_basic::bumblebee_basic_stereoimage>("/bumblebee_basic/stereo_images", 1);

	carmen_bumblebee_basic_subscribe_stereoimage(witch_camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();
	return 0;
}


