#include <carmen/carmen.h>
#include <carmen/global.h>
#include "kinect_wrapper/kinect_wrapper.h"
#include "kinect_messages.h"
#include "kinect_interface.h"
#include "kinect_util.h"
#include <pthread.h>
#include <signal.h>
#include <bits/sigaction.h>

static int kinect_depth_width = 0;
static int kinect_depth_height = 0;
static int kinect_video_width = 0;
static int kinect_video_height = 0;

static double timestamp_video = 0.0;
static double timestamp_depth = 0.0;

static carmen_kinect_video_message msg_video[10];
static carmen_kinect_depth_message msg_depth[10];

int kinect_count = 0;

int is_new_msg(double timestamp_new, double timestamp_old){
	return (timestamp_new - timestamp_old) > 0.0 ? 1 : 0;
}

void copy_video(int kinect_id, uint8_t* data, double timestamp, int size){

	if (size > 0 && is_new_msg(timestamp, timestamp_video)) {
		msg_video[kinect_id].id = kinect_id;
		msg_video[kinect_id].width = kinect_video_width;
		msg_video[kinect_id].height = kinect_video_height;
		msg_video[kinect_id].size = size;
		msg_video[kinect_id].timestamp = timestamp;
		msg_video[kinect_id].host = carmen_get_host();
		memcpy(msg_video[kinect_id].video, data, size*sizeof(unsigned char));

#ifdef DEBUG
		printf("copying video timestamp: %f\n", timestamp);
		fflush(stdout);
#endif
	}
}

void publish_video(int kinect_id){
	IPC_RETURN_TYPE err = IPC_OK;
	char* msg_name=carmen_kinect_get_video_messagename(kinect_id);

	if (msg_video[kinect_id].size > 0 && is_new_msg(msg_video[kinect_id].timestamp, timestamp_video)) {
		timestamp_video = msg_video[kinect_id].timestamp;

		err = IPC_publishData(msg_name, &(msg_video[kinect_id]));

		carmen_test_ipc_exit(err, "Could not publish", msg_name);

#ifdef DEBUG
		printf("publishing video timestamp: %f\n", timestamp_video);
		fflush(stdout);
#endif
	}
}

void copy_depth(int kinect_id, uint16_t* data, double timestamp, int size){
	int i;
	if (size > 0 && is_new_msg(timestamp, timestamp_depth)) {
		msg_depth[kinect_id].id = kinect_id;
		msg_depth[kinect_id].width = kinect_depth_width;
		msg_depth[kinect_id].height = kinect_depth_height;
		msg_depth[kinect_id].size = size;
		msg_depth[kinect_id].timestamp = timestamp;
		msg_depth[kinect_id].host = carmen_get_host();
		for(i=0; i<size; i++)
			msg_depth[kinect_id].depth[i] = convert_kinect_depth_raw_to_meters(data[i]);

#ifdef DEBUG
		printf("copying depth timestamp: %f\n", timestamp);
		fflush(stdout);
#endif
	}
}

void publish_depth(int kinect_id){
	IPC_RETURN_TYPE err = IPC_OK;
	char* msg_name=carmen_kinect_get_depth_messagename(kinect_id);

	if (msg_depth[kinect_id].size > 0 && is_new_msg(msg_depth[kinect_id].timestamp, timestamp_depth)) {
		timestamp_depth = msg_depth[kinect_id].timestamp;

		err = IPC_publishData(msg_name, &(msg_depth[kinect_id]));

		carmen_test_ipc_exit(err, "Could not publish", msg_name);

#ifdef DEBUG
		printf("publishing depth timestamp: %f\n", timestamp_depth);
		fflush(stdout);
#endif
	}
}

void shutdown_kinect(int signal __attribute__ ((unused)))
{
	int i;
	for(i=0; i<kinect_count; i++)
	{
		kinect_set_led_off(0);
		kinect_close_device(i);
	}
	carmen_ipc_disconnect();

	exit(0);
}

void video_event_handler(int kinect_id, uint8_t* data, uint32_t timestamp __attribute__ ((unused)), int size){
	copy_video(kinect_id, data, carmen_get_time(), size);
}

void depth_event_handler(int kinect_id, uint16_t* data, uint32_t timestamp __attribute__ ((unused)), int size){
	copy_depth(kinect_id, data, carmen_get_time(), size);
}

int main(int argc __attribute__ ((unused)),
		char *argv[] __attribute__ ((unused)))
{
	int i;

	/* initialize carmen */
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);

	/* Setup exit handler */
	signal(SIGINT, shutdown_kinect);
	signal(SIGTERM, shutdown_kinect);

	kinect_count = kinect_device_count();

	if (kinect_count<1)
		carmen_die("You have to specify at least one kinect device to run kinect service.\n");

	for(i=0; i<kinect_count; i++)
	{
		carmen_kinect_define_kinect_messages(i);

		kinect_close_device(i);
		kinect_open_device(i);
		kinect_set_led_green(i);

		kinect_depth_width = kinect_get_depth_width();
		kinect_depth_height = kinect_get_depth_height();
		kinect_video_width = kinect_get_video_width();
		kinect_video_height = kinect_get_video_height();

		msg_depth[i].depth = (float*)malloc(kinect_depth_width*kinect_depth_height*sizeof(float));
		msg_video[i].video = (unsigned char*)malloc(3*kinect_video_width*kinect_video_height*sizeof(unsigned char));

		kinect_set_on_video_event(i, video_event_handler);
		kinect_set_on_depth_event(i, depth_event_handler);

		kinect_start_video_capture(i);
		kinect_start_depth_capture(i);
	}

	carmen_warn("Kinect initialized.\n");

	while(1)
	{
		IPC_listen(1);

		for(i=0; i<kinect_count; i++)
		{
			publish_video(i);
			publish_depth(i);
		}
	}

	return 0;
}
