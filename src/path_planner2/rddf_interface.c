#include <carmen/carmen.h>
#include "rddf_interface.h"

char*
rddf_get_annotation_description_by_type(int type)
{
    switch (type)
    {
    case RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT:
        return "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT";
    case RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN:
        return "RDDF_ANNOTATION_TYPE_TRAFFIC_SIGN";
    case RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK:
        return "RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK";
    case RDDF_ANNOTATION_TYPE_STOP:
        return "RDDF_ANNOTATION_TYPE_STOP";
    case RDDF_ANNOTATION_TYPE_BARRIER:
        return "RDDF_ANNOTATION_TYPE_BARRIER";
    case RDDF_ANNOTATION_TYPE_BUMP:
        return "RDDF_ANNOTATION_TYPE_BUMP";
    case RDDF_ANNOTATION_TYPE_SPEED_LIMIT:
        return "RDDF_ANNOTATION_TYPE_SPEED_LIMIT";
    default:
        return "";
    }

    return "";
}

void
carmen_rddf_subscribe_road_profile_message(carmen_rddf_road_profile_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_road_profile_message), handler, subscribe_how);
}

void
carmen_rddf_subscribe_end_point_message(carmen_rddf_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_RDDF_END_POINT_MESSAGE_NAME, CARMEN_RDDF_END_POINT_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_end_point_message), handler, subscribe_how);
}

void
carmen_rddf_subscribe_waypoints_around_end_point_message(carmen_rddf_waypoints_around_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME, CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_waypoints_around_end_point_message), handler, subscribe_how);
}

void
carmen_rddf_subscribe_add_annotation_message(carmen_rddf_add_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_add_annotation_message), handler, subscribe_how);
}

void
carmen_rddf_subscribe_dynamic_annotation_message(carmen_rddf_dynamic_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_dynamic_annotation_message), handler, subscribe_how);
}

void
carmen_rddf_subscribe_annotation_message(carmen_rddf_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, CARMEN_RDDF_ANNOTATION_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_annotation_message), handler, subscribe_how);
}

void
carmen_rddf_subscribe_traffic_sign_message(carmen_rddf_traffic_sign_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_traffic_sign_message), handler, subscribe_how);
}

void
carmen_rddf_subscribe_multi_path_message(carmen_rddf_multi_path_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_RDDF_MULTI_PATH_MESSAGE_NAME, CARMEN_RDDF_MULTI_PATH_MESSAGE_FMT,
	                             message, sizeof (carmen_rddf_multi_path_message), handler, subscribe_how);
}


void
carmen_rddf_unsubscribe_road_profile_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, handler);
}

void
carmen_rddf_unsubscribe_end_point_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_END_POINT_MESSAGE_NAME, handler);
}

void
carmen_rddf_unsubscribe_waypoints_around_end_point_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME, handler);
}

void
carmen_rddf_unsubscribe_add_annotation_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, handler);
}

void
carmen_rddf_unsubscribe_dynamic_annotation_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, handler);
}

void
carmen_rddf_unsubscribe_annotation_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, handler);
}

void
carmen_rddf_unsubscribe_traffic_sign_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, handler);
}

void
carmen_rddf_unsubscribe_multi_path_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_MULTI_PATH_MESSAGE_NAME, handler);
}


void
carmen_rddf_define_messages()
{
    IPC_RETURN_TYPE err;

    //
    // define the road profile message
    //
    err = IPC_defineMsg(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME);

    //
    // define the end point message
    //
    err = IPC_defineMsg(CARMEN_RDDF_END_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_END_POINT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_END_POINT_MESSAGE_NAME);

    //
    // define the road profile message around to the end point
    //
    err = IPC_defineMsg(CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME);

    //
    // define the add annotation message
    //
    err = IPC_defineMsg(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME);

    //
    // define the dynamic annotation message
    //
    err = IPC_defineMsg(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME);

    //
    // define the annotation message
    //
    err = IPC_defineMsg(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_ANNOTATION_MESSAGE_NAME);

    //
    // define the traffic sign message
    //
    err = IPC_defineMsg(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME);

    //
	// define the multi path message
	//
	err = IPC_defineMsg(CARMEN_RDDF_MULTI_PATH_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_MULTI_PATH_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_MULTI_PATH_MESSAGE_NAME);
}

void
carmen_rddf_publish_end_point_message(int number_of_poses, carmen_point_t point)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_end_point_message rddf_end_point_message;

    rddf_end_point_message.number_of_poses = number_of_poses;
    rddf_end_point_message.point = point;
    rddf_end_point_message.timestamp = carmen_get_time();
    rddf_end_point_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_END_POINT_MESSAGE_NAME, &rddf_end_point_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_END_POINT_MESSAGE_NAME);
}

void
carmen_rddf_publish_road_profile_message(carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int num_poses, int num_poses_back, int *annotations, int * annotations_codes)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_road_profile_message rddf_road_profile_message;

    rddf_road_profile_message.poses = poses_ahead;
    rddf_road_profile_message.poses_back = poses_back;
    rddf_road_profile_message.number_of_poses = num_poses;
    rddf_road_profile_message.number_of_poses_back = num_poses_back;
    rddf_road_profile_message.annotations = annotations;
    rddf_road_profile_message.annotations_codes = annotations_codes;
    rddf_road_profile_message.timestamp = carmen_get_time();
    rddf_road_profile_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, &rddf_road_profile_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME);
}

void
carmen_rddf_publish_road_profile_around_end_point_message(carmen_ackerman_traj_point_t *poses_around_end_point, int num_poses)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_waypoints_around_end_point_message rddf_road_profile_around_end_point_message;

    rddf_road_profile_around_end_point_message.poses = poses_around_end_point;
    rddf_road_profile_around_end_point_message.number_of_poses = num_poses;
    rddf_road_profile_around_end_point_message.timestamp = carmen_get_time();
    rddf_road_profile_around_end_point_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME, &rddf_road_profile_around_end_point_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME);
}

void
carmen_rddf_publish_add_annotation_message(carmen_vector_3D_t annotation_point, double orientation, char *annotation_description, int annotation_type, int annotation_code)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_add_annotation_message rddf_add_annotation_message;

    rddf_add_annotation_message.annotation_point = annotation_point;
    rddf_add_annotation_message.annotation_orientation = orientation;
    rddf_add_annotation_message.annotation_description = annotation_description;
    rddf_add_annotation_message.annotation_type = annotation_type;
    rddf_add_annotation_message.annotation_code = annotation_code;
    rddf_add_annotation_message.timestamp = carmen_get_time();
    rddf_add_annotation_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, &rddf_add_annotation_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME);
}

void
carmen_rddf_publish_dynamic_annotation_message(carmen_vector_3D_t annotation_point, double orientation, char *annotation_description,
		int annotation_type, int annotation_code, double timestamp)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_dynamic_annotation_message rddf_dynamic_annotation_message;

    rddf_dynamic_annotation_message.annotation_point = annotation_point;
    rddf_dynamic_annotation_message.annotation_orientation = orientation;
    rddf_dynamic_annotation_message.annotation_description = annotation_description;
    rddf_dynamic_annotation_message.annotation_type = annotation_type;
    rddf_dynamic_annotation_message.annotation_code = annotation_code;
    rddf_dynamic_annotation_message.timestamp = timestamp;
    rddf_dynamic_annotation_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, &rddf_dynamic_annotation_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME);
}

void
carmen_rddf_publish_annotation_message(carmen_annotation_t *annotations, int num_annotations)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_annotation_message rddf_annotation_message;

    rddf_annotation_message.num_annotations = num_annotations;
    rddf_annotation_message.annotations = annotations;
    rddf_annotation_message.timestamp = carmen_get_time();
    rddf_annotation_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &rddf_annotation_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_NAME);
}

void
carmen_rddf_publish_traffic_sign_message(int traffic_sign_state, double traffic_sign_data)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_traffic_sign_message rddf_traffic_sign_message;

    rddf_traffic_sign_message.traffic_sign_state = traffic_sign_state;
    rddf_traffic_sign_message.traffic_sign_data = traffic_sign_data;
    rddf_traffic_sign_message.timestamp = carmen_get_time();
    rddf_traffic_sign_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, &rddf_traffic_sign_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME);
}

void
carmen_rddf_publish_multi_path_message(carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int num_poses, int num_poses_back,
		int *annotations, int *annotations_codes, int tree_path_num, int *tree_path_sizes, double *tree_path_costs, carmen_ackerman_traj_point_t **tree_path)
{
	IPC_RETURN_TYPE err;
	carmen_rddf_multi_path_message rddf_multi_path_message;

	int sum = 0;
	for (int i = 0; i < tree_path_num; i++)
		sum += tree_path_sizes[i];

	carmen_ackerman_traj_point_t flat_tree_path [sum];
	for (int i = 0, j = 0; i < tree_path_num; j += tree_path_sizes[i++])
		memcpy(&flat_tree_path[j],tree_path[i], sizeof(carmen_ackerman_traj_point_t) * tree_path_sizes[i]);

	rddf_multi_path_message.poses = poses_ahead;
	rddf_multi_path_message.poses_back = poses_back;
    rddf_multi_path_message.number_of_poses = num_poses;
    rddf_multi_path_message.number_of_poses_back = num_poses_back;
    rddf_multi_path_message.annotations = annotations;
    rddf_multi_path_message.annotations_codes = annotations_codes;


	rddf_multi_path_message.tree_path_costs = tree_path_costs;
	rddf_multi_path_message.tree_path_num = tree_path_num;
	rddf_multi_path_message.tree_path_total_points = sum;
	rddf_multi_path_message.tree_path_sizes = tree_path_sizes;
	rddf_multi_path_message.flat_tree_path = &flat_tree_path[0];
	rddf_multi_path_message.timestamp = carmen_get_time();
	rddf_multi_path_message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_RDDF_MULTI_PATH_MESSAGE_NAME, &rddf_multi_path_message);

	carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_MULTI_PATH_MESSAGE_NAME);
}


char *
get_traffic_sign_state_name(int state)
{
	if (state == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT) 	return ((char *) "Turn_Right");
	if (state == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT) 	return ((char *) "Turn_Left");
	if (state == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_GO_STRAIGHT)	return ((char *) "Go_Straight");
	if (state == RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_OFF) 		return ((char *) "Off");

	return ((char *) " ");
}
