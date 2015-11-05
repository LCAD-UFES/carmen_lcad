#include <carmen/carmen.h>
#include <bits/sigaction.h>
#include <signal.h>
#include <stdint.h>
#include "kinect_interface.h"
#include "kinect_messages.h"
#include "kinect_util.h"

static double timestamp_video = 0.0;
static double timestamp_depth = 0.0;

static carmen_kinect_video_message msg_video[1];
static carmen_kinect_depth_message msg_depth[1];

static int
is_new_msg(double timestamp_new, double timestamp_old)
{
	return (timestamp_new - timestamp_old) > 0.0 ? 1 : 0;
}

void copy_video(int kinect_id, unsigned char* data, double timestamp, int size,
		int kinect_video_width, int kinect_video_height)
{
	if (size > 0 && is_new_msg(timestamp, timestamp_video))
	{
		msg_video[kinect_id].id = kinect_id;
		msg_video[kinect_id].width = kinect_video_width;
		msg_video[kinect_id].height = kinect_video_height;
		msg_video[kinect_id].size = size;
		msg_video[kinect_id].timestamp = timestamp;
		msg_video[kinect_id].host = carmen_get_host();
		if (msg_video[kinect_id].video == 0)
			msg_video[kinect_id].video = (unsigned char *)malloc(size * sizeof(unsigned char));
		memcpy(msg_video[kinect_id].video, data, size * sizeof(unsigned char));
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

static void
kinect_video_mrds_handler(carmen_kinect_video_mrds_message *message)
{
	copy_video(message->id, message->video, message->timestamp, message->size, message->width, message->height);
	publish_video(message->id);
}

void copy_depth(int kinect_id, short* data, double timestamp, int size,
		int kinect_depth_width, int kinect_depth_height)
{
	int i;
	if (size > 0 && is_new_msg(timestamp, timestamp_depth))
	{
		msg_depth[kinect_id].id = kinect_id;
		msg_depth[kinect_id].width = kinect_depth_width;
		msg_depth[kinect_id].height = kinect_depth_height;
		msg_depth[kinect_id].size = size;
		msg_depth[kinect_id].timestamp = timestamp;
		msg_depth[kinect_id].host = carmen_get_host();
		if (msg_depth[kinect_id].depth == 0)
			msg_depth[kinect_id].depth = (float *)malloc(size * sizeof(float));
		for(i=0; i<size; i++)
			msg_depth[kinect_id].depth[i] = convert_kinect_depth_mm_to_meters(data[i]);

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

static void
kinect_depth_mrds_handler(carmen_kinect_depth_mrds_message *message)
{
	copy_depth(message->id, message->depth, message->timestamp, message->size, message->width, message->height);
	publish_depth(message->id);
}

void shutdown_kinect(int signal __attribute__ ((unused)))
{
	carmen_ipc_disconnect();

	exit(0);
}

int main(int argc, char *argv[])
{
	int kinect_id = 0;

	msg_depth[kinect_id].depth = 0;
	msg_video[kinect_id].video = 0;

	/* initialize carmen */
	carmen_randomize(&argc, &argv);
	carmen_ipc_initialize(argc, argv);

	/* Setup exit handler */
	signal(SIGINT, shutdown_kinect);
	signal(SIGTERM, shutdown_kinect);

	carmen_kinect_define_kinect_messages(kinect_id);

	carmen_kinect_subscribe_depth_mrds_message0(NULL,
			(carmen_handler_t) kinect_depth_mrds_handler,
			CARMEN_SUBSCRIBE_LATEST );

	carmen_kinect_subscribe_video_mrds_message0(NULL,
			(carmen_handler_t) kinect_video_mrds_handler,
			CARMEN_SUBSCRIBE_LATEST );

	carmen_warn("Kinect MRDS initialized.\n");

	carmen_ipc_dispatch();

	return 0;
}




