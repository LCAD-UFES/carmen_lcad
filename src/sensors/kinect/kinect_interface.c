#include <carmen/carmen.h>
#include <carmen/kinect_messages.h>

static char* depth_msgnames[10]={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static char* video_msgnames[10]={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};

char* carmen_kinect_get_depth_messagename(int kinect_id) {
	if (depth_msgnames[kinect_id])
		return depth_msgnames[kinect_id];
	char name_result[1024];
	sprintf(name_result, "carmen_kinect_depth_msg_%d", kinect_id);
	depth_msgnames[kinect_id]=malloc(strlen(name_result)+1);
	strcpy(depth_msgnames[kinect_id],name_result);
	return depth_msgnames[kinect_id];
}

char* carmen_kinect_get_video_messagename(int kinect_id) {
	if (video_msgnames[kinect_id])
		return video_msgnames[kinect_id];
	char name_result[1024];
	sprintf(name_result, "carmen_kinect_video_msg_%d", kinect_id);
	video_msgnames[kinect_id]=malloc(strlen(name_result)+1);
	strcpy(video_msgnames[kinect_id],name_result);
	return video_msgnames[kinect_id];
}

void
carmen_kinect_subscribe_depth_message(int kinect_id,
		carmen_kinect_depth_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	char* msg_name=carmen_kinect_get_depth_messagename(kinect_id);

	carmen_subscribe_message(msg_name,
			CARMEN_KINECT_DEPTH_FMT,
			message, sizeof(carmen_kinect_depth_message), handler, subscribe_how);
}

void
carmen_kinect_unsubscribe_depth_message(int kinect_id, carmen_handler_t handler)
{
	char* msg_name=carmen_kinect_get_depth_messagename(kinect_id);

	carmen_unsubscribe_message(msg_name, handler);
}

void
carmen_kinect_subscribe_video_message(int kinect_id,
		carmen_kinect_video_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	char* msg_name=carmen_kinect_get_video_messagename(kinect_id);

	carmen_subscribe_message(msg_name,
			CARMEN_KINECT_VIDEO_FMT,
			message, sizeof(carmen_kinect_video_message), handler, subscribe_how);
}

void
carmen_kinect_unsubscribe_video_message(int kinect_id, carmen_handler_t handler)
{
	char* msg_name=carmen_kinect_get_video_messagename(kinect_id);

	carmen_unsubscribe_message(msg_name, handler);
}

IPC_RETURN_TYPE
carmen_kinect_define_kinect_messages(int kinect_id)
{
	IPC_RETURN_TYPE err;
	char* depth_msg_name=carmen_kinect_get_depth_messagename(kinect_id);
	char* video_msg_name=carmen_kinect_get_video_messagename(kinect_id);

	err = IPC_defineMsg(depth_msg_name, IPC_VARIABLE_LENGTH, CARMEN_KINECT_DEPTH_FMT);
	carmen_test_ipc_exit(err, "Could not define", depth_msg_name);

	err = IPC_defineMsg(video_msg_name, IPC_VARIABLE_LENGTH, CARMEN_KINECT_VIDEO_FMT);
	carmen_test_ipc_exit(err, "Could not define", video_msg_name);

	return err;
}

IPC_RETURN_TYPE
carmen_kinect_define_kinect_mrds_messages(int kinect_id)
{
	IPC_RETURN_TYPE err;

	if (kinect_id > 0)
		return IPC_Error;

	err = carmen_kinect_define_video_mrds_message0();

	err = carmen_kinect_define_depth_mrds_message0();

	return err;
}

IPC_RETURN_TYPE
carmen_kinect_define_video_mrds_message0()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_KINECT_VIDEO_MRDS_MSG_0_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_VIDEO_MRDS_MSG_0_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_VIDEO_MRDS_MSG_0_NAME);

	return err;
}

IPC_RETURN_TYPE
carmen_kinect_define_depth_mrds_message0()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_KINECT_DEPTH_MRDS_MSG_0_NAME, IPC_VARIABLE_LENGTH, CARMEN_KINECT_DEPTH_MRDS_MSG_0_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_KINECT_DEPTH_MRDS_MSG_0_NAME);

	return err;
}

void
carmen_kinect_subscribe_depth_mrds_message0(
		carmen_kinect_depth_mrds_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{

	carmen_subscribe_message(CARMEN_KINECT_DEPTH_MRDS_MSG_0_NAME,
			CARMEN_KINECT_DEPTH_MRDS_MSG_0_FMT,
			message, sizeof(carmen_kinect_depth_mrds_message), handler, subscribe_how);
}

void
carmen_kinect_unsubscribe_depth_mrds_message0(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_KINECT_DEPTH_MRDS_MSG_0_NAME, handler);
}

void
carmen_kinect_subscribe_video_mrds_message0(
		carmen_kinect_video_mrds_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{

	carmen_subscribe_message(CARMEN_KINECT_VIDEO_MRDS_MSG_0_NAME,
			CARMEN_KINECT_DEPTH_MRDS_MSG_0_FMT,
			message, sizeof(carmen_kinect_video_mrds_message), handler, subscribe_how);
}

void
carmen_kinect_unsubscribe_video_mrds_message0(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_KINECT_VIDEO_MRDS_MSG_0_NAME, handler);
}
