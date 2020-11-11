#include <carmen/carmen.h>
#include <carmen/mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/global_graphics.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>


double last_timestamp = 0.0;
carmen_map_server_offline_map_message *offline_map_msg = NULL;
carmen_map_t map;
cv::Mat *map_img = NULL;


void
map_to_image(carmen_map_t *map, cv::Mat *offline_map_img)
{
	unsigned char *image_data = carmen_graphics_convert_to_image(map, 0);

	int i = 0, j = 0;
	for (i = 0; i < map->config.x_size; i++)
	{
		for (j = 0; j < map->config.y_size; j++)
		{
			uchar red   = (uchar) image_data[((i * map->config.y_size + j) * 3)];
			uchar green = (uchar) image_data[((i * map->config.y_size + j) * 3) + 1];
			uchar blue  = (uchar) image_data[((i * map->config.y_size + j) * 3) + 2];
			cv::Vec3b color = cv::Vec3b(blue, green, red);
			offline_map_img->at<cv::Vec3b>(map->config.y_size - 1 - j, i) = color;
		}
	}
	free(image_data);
}


void
display_online_map(carmen_mapper_diff_map_message *msg)
{
	map.config = offline_map_msg->config;
	memcpy(map.complete_map, offline_map_msg->complete_map, offline_map_msg->size * sizeof(double));

	for (int i = 0; i < msg->size; i++)
	{
		int index = msg->coord_x[i] * msg->config.y_size + msg->coord_y[i];
		map.complete_map[index] = (double) msg->value[i] / 255.0;
	}

	map_to_image(&map, map_img);
	cv::imshow("Map", *map_img);
	cv::waitKey(2);
}


void
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	static int first_time = 1;
	if (first_time)
	{
		map.complete_map = (double *) malloc(msg->size * sizeof(double));
		carmen_test_alloc(map.complete_map);
		map_img = new cv::Mat(msg->config.y_size, msg->config.x_size, CV_8UC3, cv::Scalar::all(0));
		first_time = 0;
	}

	offline_map_msg = msg;
}


void
diff_map_handler(carmen_mapper_diff_map_message *msg)
{
	if (offline_map_msg != NULL &&
		offline_map_msg->config.x_origin == msg->config.x_origin && offline_map_msg->config.y_origin == msg->config.y_origin)
		display_online_map(msg);

	printf("\nReceived carmen_mapper_diff_map_message:\n");
	printf("origin = { %.2lf , %.2lf }\n", msg->config.x_origin, msg->config.y_origin);
	printf("size = %d\n", msg->size);
	for (int i = 0; i < msg->size; i++)
	{
		printf("  i = %3d", i);
		printf("  coord = { %4d , %4d }", msg->coord_x[i], msg->coord_y[i]);
		printf("  value = %3d\n", msg->value[i]);
		if (i == 10)
		{
			printf("  ...\n");
			break;
		}
	}
	printf("timestamp = %lf (%.2lf)\n", msg->timestamp, msg->timestamp - last_timestamp);
	last_timestamp = msg->timestamp;
}


static void
shutdown_module(int sig)
{
	(void) sig;

	carmen_ipc_disconnect();
	fprintf(stderr, "\nModule exited by soft shutdown\n\n");
	exit(0);
}


void
subscribe_messages()
{
	carmen_mapper_subscribe_diff_map_message(NULL, (carmen_handler_t) diff_map_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	subscribe_messages();
	last_timestamp = carmen_get_time();
	carmen_ipc_dispatch();
	return 0;
}
