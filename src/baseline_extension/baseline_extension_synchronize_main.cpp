#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <string.h>
#include <vector>

using namespace std;

int camera;
char *output_file;
FILE *output_file_ptr;
vector<carmen_bumblebee_basic_stereoimage_message*> image_queue;

void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		fflush(output_file_ptr);
		fclose(output_file_ptr);

		carmen_ipc_disconnect();
		printf("baseline_extension: disconnected.\n");

		exit(0);
	}
}


carmen_bumblebee_basic_stereoimage_message *
carmen_baseline_extension_synchronize_create_message_copy(carmen_bumblebee_basic_stereoimage_message *stereo_message)
{
	carmen_bumblebee_basic_stereoimage_message *message = (carmen_bumblebee_basic_stereoimage_message *) calloc (sizeof(carmen_bumblebee_basic_stereoimage_message), 1);

	message->raw_left = (unsigned char *) calloc (stereo_message->image_size, sizeof(unsigned char));
	message->raw_right = (unsigned char *) calloc (stereo_message->image_size, sizeof(unsigned char));

	message->width = stereo_message->width;
	message->height = stereo_message->height;
	message->image_size = stereo_message->image_size;
	message->isRectified = stereo_message->isRectified;
	message->timestamp = stereo_message->timestamp;
	message->host = stereo_message->host;

	memcpy(message->raw_left, stereo_message->raw_left, stereo_message->image_size * sizeof(unsigned char));
	memcpy(message->raw_right, stereo_message->raw_right, stereo_message->image_size * sizeof(unsigned char));

	return message;
}


void
carmen_baseline_extension_synchronize_save_image_and_pose(carmen_bumblebee_basic_stereoimage_message *stereo_message, carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	int i;

	fprintf(output_file_ptr, "%d %d %d %d %lf ", stereo_message->height, stereo_message->width, stereo_message->image_size, stereo_message->isRectified, stereo_message->timestamp);

	for (i = 0; i < stereo_message->image_size; i++)
		fprintf(output_file_ptr, "%d ", (int) stereo_message->raw_left[i]);

	for (i = 0; i < stereo_message->image_size; i++)
		fprintf(output_file_ptr, "%d ", (int) stereo_message->raw_right[i]);

	fprintf(output_file_ptr, "\n");

//	fprintf(output_file_ptr, "%lf %lf %lf %lf %lf %lf\n",
//		globalpos_message->pose.position.x, globalpos_message->pose.position.y, globalpos_message->pose.position.z,
//		globalpos_message->pose.orientation.roll, globalpos_message->pose.orientation.pitch, globalpos_message->pose.orientation.yaw
//	);
}


void
carmen_baseline_extension_bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_message)
{
//	printf("bumblebee handler\n");
//	carmen_baseline_extension_synchronize_save_image_and_pose(stereo_message, NULL);
//	image_queue.push_back(carmen_baseline_extension_synchronize_create_message_copy(stereo_message));
}


carmen_bumblebee_basic_stereoimage_message *
carmen_baseline_extension_synchronize_get_synchronized_image_message(double globalpos_timestamp)
{
	uint i;
	int min_time_diff_message_index = -1;
	double min_time_diff = DBL_MAX, time_diff;

	for (i = 0; i < image_queue.size(); i++)
	{
		time_diff = fabs(globalpos_timestamp - image_queue[i]->timestamp);

		if (time_diff < min_time_diff)
		{
			min_time_diff = time_diff;
			min_time_diff_message_index = i;
		}
	}

	if (min_time_diff_message_index == -1)
		exit(printf("Error: No images to synchronize with pose\n"));

	return image_queue[min_time_diff_message_index];
}


void
carmen_baseline_extension_synchronize_clear_image_queue()
{
	uint i;

	for (i = 0; i < image_queue.size(); i++)
	{
		free(image_queue[i]->raw_left);
		free(image_queue[i]->raw_right);
		free(image_queue[i]);
	}

	image_queue.clear();
}


void
carmen_baseline_extension_localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
		globalpos_message->pose.position.x,
		globalpos_message->pose.position.y,
		globalpos_message->pose.position.z,
		globalpos_message->pose.orientation.roll,
		globalpos_message->pose.orientation.pitch,
		globalpos_message->pose.orientation.yaw,
		globalpos_message->timestamp
	);

//	carmen_bumblebee_basic_stereoimage_message *stereo_message = carmen_baseline_extension_synchronize_get_synchronized_image_message(globalpos_message->timestamp);
//	carmen_baseline_extension_synchronize_save_image_and_pose(stereo_message, globalpos_message);
//	carmen_baseline_extension_synchronize_clear_image_queue();
}


void
carmen_baseline_extension_synchronize_initialize()
{
	output_file_ptr = fopen(output_file, "w");

	if (!output_file_ptr)
	{
		printf("Error: can not open the file '%s'\n", output_file);
		exit(-1);
	}
}


int 
parse_arguments(int argc, char **argv)
{
	if (argc < 3)
		return 0;

	camera = atoi(argv[1]);
	output_file = argv[2];

	return 1;
}


void
show_usage_information_and_exit(int argc __attribute__ ((unused)), char **argv)
{
	printf("\n");
	printf("Use %s <camera-index> <output-synchronization-file>\n", argv[0]);
	printf("\n");

	exit(-1);
}


int
main(int argc, char **argv) 
{
	if (!parse_arguments(argc, argv))
		show_usage_information_and_exit(argc, argv);

	carmen_baseline_extension_synchronize_initialize();
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_baseline_extension_bumblebee_basic_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_baseline_extension_localize_ackerman_handler, CARMEN_SUBSCRIBE_ALL);

	carmen_ipc_dispatch();
	return (0);
}
