#include "global_planning_interface.h"



void define_msg(int *activate, char *name, char *fmt)
{
    IPC_RETURN_TYPE err; 
    err = IPC_defineMsg(name,  IPC_VARIABLE_LENGTH, fmt);
    carmen_test_ipc_exit( err, "Could not define", name);
    *activate = 1;
}


/***************************** Route Planner Route *****************************/
//============== subscribe de backwards_annotations ===================== //

/*
void 
carmen_route_planner_subscribe_backwards_annotations(carmen_rddf_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how )
{
	IPC_RETURN_TYPE err;
    static int initialized = 0;
    if (!initialized)
    {
        err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME);
        initialized = 1;
    }
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME, CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_FMT,
                             message, sizeof(carmen_rddf_annotation_message), handler, subscribe_how);
}

void 
carmen_route_planner_publish_backwards_annotations(carmen_rddf_annotation_message *msg)
{
	IPC_RETURN_TYPE err;
    static int initialized = 0;

    if (!initialized)
    {
        err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME);
        initialized = 1;
    }

    err = IPC_publishData(CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME, msg);
    carmen_test_ipc_exit(err, "Could not publish message", CARMEN_ROUTE_PLANNER_INITIAL_ANNOTATION_MESSAGE_NAME);
}
*/
//==================================================================================== //

void
carmen_route_planner_subscribe_destination_message(carmen_route_planner_destination_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME, CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_FMT,
                             message, sizeof (carmen_route_planner_destination_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_destination_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME, handler);
}

void
carmen_route_planner_publish_destination_message(char *destination, carmen_point_t destination_point)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_destination_message msg;

	msg.destination = destination;
	msg.destination_point = destination_point;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_DESTINATION_MESSAGE_NAME);
}



void
carmen_route_planner_subscribe_pallet_and_destination_message(carmen_route_planner_pallet_and_destination_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;

    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME, CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_FMT,
                             message, sizeof (carmen_route_planner_pallet_and_destination_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_pallet_and_destination_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME, handler);
}

void
carmen_route_planner_publish_pallet_and_destination(char *pallet, carmen_point_t pallet_point, char *destination, carmen_point_t destination_point)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_pallet_and_destination_message msg;

	

	msg.pallet = pallet;
	msg.pallet_point = pallet_point;
	msg.destination = destination;
	msg.destination_point = destination_point;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_PALLET_AND_DESTINATION_MESSAGE_NAME);
} 



void
carmen_route_planner_subscribe_road_network_message(carmen_route_planner_road_network_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME, CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_FMT,
                             message, sizeof (carmen_route_planner_road_network_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_road_network_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME, handler);
}

void
carmen_route_planner_publish_road_network_message(carmen_route_planner_road_network_message *route_planner_road_network_message)
{
    IPC_RETURN_TYPE err;

    // static int *nearby_lanes_node_ids = NULL;	// Size == nearby_lanes_size. Ids dos nós (poses) de todas as lanes.

	// static int *nearby_lanes_merges_indexes = NULL;	// Size == number_of_nearby_lanes. O ponto em nearby_lanes_merges onde começam os merges de cada lane.
	// static int *nearby_lanes_merges_sizes = NULL;		// Size == number_of_nearby_lanes. O número de merges de cada lane.

	// static int *nearby_lanes_forks_indexes = NULL;	// Size == number_of_nearby_lanes. O ponto em nearby_lanes_forks onde começam os forks de cada lane.
	// static int *nearby_lanes_forks_sizes = NULL;		// Size == number_of_nearby_lanes. O número de forks de cada lane.

    // nearby_lanes_node_ids = realloc(nearby_lanes_node_ids, route_planner_road_network_message->nearby_lanes_size * sizeof(int));
    // route_planner_road_network_message->nearby_lanes_node_ids = nearby_lanes_node_ids;

    // nearby_lanes_merges_indexes = realloc(nearby_lanes_merges_indexes, route_planner_road_network_message->number_of_nearby_lanes * sizeof(int));
    // route_planner_road_network_message->nearby_lanes_merges_indexes = nearby_lanes_merges_indexes;
    // nearby_lanes_merges_sizes = realloc(nearby_lanes_merges_sizes, route_planner_road_network_message->number_of_nearby_lanes * sizeof(int));
    // route_planner_road_network_message->nearby_lanes_merges_sizes = nearby_lanes_merges_sizes;

    // nearby_lanes_forks_indexes = realloc(nearby_lanes_forks_indexes, route_planner_road_network_message->number_of_nearby_lanes * sizeof(int));
    // route_planner_road_network_message->nearby_lanes_forks_indexes = nearby_lanes_forks_indexes;
    // nearby_lanes_forks_sizes = realloc(nearby_lanes_forks_sizes, route_planner_road_network_message->number_of_nearby_lanes * sizeof(int));
    // route_planner_road_network_message->nearby_lanes_forks_sizes = nearby_lanes_forks_sizes;

    err = IPC_publishData(CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME, route_planner_road_network_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_ROUTE_PLANNER_ROAD_NETWORK_MESSAGE_NAME);
}



void
carmen_route_planner_subscribe_locked_roads_message(carmen_route_planner_locked_roads_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	IPC_RETURN_TYPE err;

    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_LOCKED_ROADS_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_LOCKED_ROADS_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_LOCKED_ROADS_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_LOCKED_ROADS_NAME, CARMEN_ROUTE_PLANNER_LOCKED_ROADS_FMT,
                             message, sizeof (carmen_route_planner_locked_roads_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_locked_roads_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_LOCKED_ROADS_NAME, handler);
} 

void
carmen_route_planner_publish_locked_roads_message(int number_of_locked_node_ids, int *locked_graph_node_ids, int status, char *graph_id)
{
    static char empty_string[] = "";
    
	IPC_RETURN_TYPE err;
	carmen_route_planner_locked_roads_message msg;

	

	msg.number_of_locked_node_ids = number_of_locked_node_ids;
	msg.locked_graph_node_ids     = locked_graph_node_ids;
	msg.status                    = status;
    msg.graph_id                  = graph_id ? graph_id : empty_string;
	msg.timestamp                 = carmen_get_time();
	msg.host                      = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_LOCKED_ROADS_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_LOCKED_ROADS_NAME);
}



void
carmen_route_planner_subscribe_predefined_route_message(carmen_route_planner_predefined_route_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	IPC_RETURN_TYPE err;

    static int initialized = 0;
	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_NAME);
		initialized = 1;
	}

    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_NAME, CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_FMT,
                             message, sizeof (carmen_route_planner_predefined_route_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_predefined_route_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_NAME, handler);
}

void
carmen_route_planner_publish_predefined_route_message(char *predefined_route, int code)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_predefined_route_message msg;

	
	msg.predefined_route = predefined_route;
	msg.code             = code;
	msg.timestamp        = carmen_get_time();
	msg.host             = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_PREDEFINED_ROUTE_MESSAGE_NAME);
}



void
carmen_route_planner_subscribe_route_status_change_message(carmen_route_planner_route_status_change_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;

    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME, CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_FMT,
                             message, sizeof (carmen_route_planner_route_status_change_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_route_status_change_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME, handler);
}

void
carmen_route_planner_publish_route_status_change_message(int route_id, int status)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_route_status_change_message msg;

	
	msg.route_id = route_id;
	msg.status = status;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_ROUTE_STATUS_CHANGE_NAME);
}



void
carmen_route_planner_subscribe_route_reload_message(carmen_route_planner_route_reload_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME, CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_FMT,
                             message, sizeof (carmen_route_planner_route_reload_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_route_reload_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME, handler);
}

void
carmen_route_planner_publish_route_reload_message(char *road_netword_id)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_route_reload_message msg;

	

	msg.road_netword_id = road_netword_id;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_ROUTE_RELOAD_NAME);
}



void
carmen_route_planner_subscribe_route_list_request_message(carmen_route_planner_route_list_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME, CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_FMT,
                             message, sizeof (carmen_route_planner_route_list_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_route_list_request_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME, handler);
}

void
carmen_route_planner_publish_route_list_request_message(carmen_position_t center, double range)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_route_list_message msg;

	

	msg.center = center;
	msg.range = range;
	msg.number_of_routes = 0;
	msg.routes = NULL;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_ROUTE_LIST_REQUEST_NAME);
}



void
carmen_route_planner_subscribe_route_list_response_message(carmen_route_planner_route_list_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME);
		initialized = 1;
	}

    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME, CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_FMT,
                             message, sizeof (carmen_route_planner_route_list_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_route_list_response_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME, handler);
}

void
carmen_route_planner_publish_route_list_response_message(carmen_position_t center, double range, int number_of_routes, route_t *routes)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_route_list_message msg;

	
	msg.center = center;
	msg.range = range;
	msg.number_of_routes = number_of_routes;
	msg.routes = routes;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_ROUTE_LIST_RESPONSE_NAME);
}



void
carmen_route_planner_subscribe_node_ids_ahead_request_message(carmen_route_planner_node_ids_ahead_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_NAME, CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_FMT,
                             message, sizeof (carmen_route_planner_node_ids_ahead_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_node_ids_ahead_request_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_NAME, handler);
}

void
carmen_route_planner_publish_node_ids_ahead_request_message(carmen_point_t initial_point, double meters_ahead)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_node_ids_ahead_message msg;

	

	msg.initial_point = initial_point;
	msg.meters_ahead = meters_ahead;
	msg.num_of_node_ids_ahead = 0;
	msg.node_ids_ahead = NULL;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_REQUEST_NAME);
}



void
carmen_route_planner_subscribe_node_ids_ahead_response_message(carmen_route_planner_node_ids_ahead_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_NAME);
		initialized = 1;
	}

    carmen_subscribe_message(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_NAME, CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_FMT,
                             message, sizeof (carmen_route_planner_node_ids_ahead_message), handler, subscribe_how);
}

void
carmen_route_planner_unsubscribe_node_ids_ahead_response_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_NAME, handler);
}

void
carmen_route_planner_publish_node_ids_ahead_response_message(carmen_point_t initial_point, double meters_ahead, int num_of_node_ids_ahead, int *node_ids_ahead)
{
	IPC_RETURN_TYPE err;
	carmen_route_planner_node_ids_ahead_message msg;

	
	msg.initial_point = initial_point;
	msg.meters_ahead = meters_ahead;
	msg.num_of_node_ids_ahead = num_of_node_ids_ahead;
	msg.node_ids_ahead = node_ids_ahead;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROUTE_PLANNER_NODE_IDS_AHEAD_RESPONSE_NAME);
}



/************************** Route Planner Annotation ***************************/

void
carmen_rddf_subscribe_dynamic_annotation_message(carmen_rddf_dynamic_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
        err = IPC_defineMsg(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_dynamic_annotation_message), handler, subscribe_how);
}

void
carmen_rddf_unsubscribe_dynamic_annotation_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, handler);
}
/*
void
carmen_rddf_publish_dynamic_annotation_message(carmen_vector_3D_t annotation_point, double orientation, char *annotation_description,
		int annotation_type, int annotation_code, int annotation_id, double timestamp)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_dynamic_annotation_message rddf_dynamic_annotation_message;

    rddf_dynamic_annotation_message.annotation_point = annotation_point;
    rddf_dynamic_annotation_message.annotation_orientation = orientation;
    rddf_dynamic_annotation_message.annotation_description = annotation_description;
    rddf_dynamic_annotation_message.annotation_type = annotation_type;
    rddf_dynamic_annotation_message.annotation_code = annotation_code;
    rddf_dynamic_annotation_message.annotation_id = annotation_id;
    rddf_dynamic_annotation_message.timestamp = timestamp;
    rddf_dynamic_annotation_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME, &rddf_dynamic_annotation_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_DYNAMIC_ANNOTATION_MESSAGE_NAME);
}
*/


void
carmen_rddf_subscribe_waypoints_around_end_point_message(carmen_rddf_waypoints_around_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
        err = IPC_defineMsg(CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME, CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_waypoints_around_end_point_message), handler, subscribe_how);
}

void
carmen_rddf_unsubscribe_waypoints_around_end_point_message(carmen_handler_t handler)
{
        carmen_unsubscribe_message(CARMEN_RDDF_WAYPOINTS_AROUND_END_POINT_MESSAGE_NAME, handler);

}

void
carmen_rddf_publish_waypoints_around_end_point_message(carmen_robot_and_trailers_traj_point_t *poses_around_end_point, int num_poses)
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

static int initialized_traffic_sign = 0;


void
carmen_rddf_subscribe_traffic_sign_message(carmen_rddf_traffic_sign_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;

	if (!initialized_traffic_sign)
	{
        err = IPC_defineMsg(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME);
		initialized_traffic_sign = 1;
	}
    carmen_subscribe_message(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_traffic_sign_message), handler, subscribe_how);
}

void
carmen_rddf_publish_traffic_sign_message(int traffic_sign_state, double traffic_sign_data)
{
    IPC_RETURN_TYPE err;
    if (!initialized_traffic_sign)
	{
        err = IPC_defineMsg(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME);
		initialized_traffic_sign = 1;
	}
    carmen_rddf_traffic_sign_message rddf_traffic_sign_message;

    rddf_traffic_sign_message.traffic_sign_state = traffic_sign_state;
    rddf_traffic_sign_message.traffic_sign_data = traffic_sign_data;
    rddf_traffic_sign_message.timestamp = carmen_get_time();
    rddf_traffic_sign_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME, &rddf_traffic_sign_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_TRAFFIC_SIGN_MESSAGE_NAME);
}


void
carmen_rddf_subscribe_road_profile_message(carmen_rddf_road_profile_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME);
		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_RDDF_ROAD_PROFILE_MESSAGE_NAME, CARMEN_RDDF_ROAD_PROFILE_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_road_profile_message), handler, subscribe_how);
}

void
carmen_rddf_publish_road_profile_message(carmen_robot_and_trailers_traj_point_t *poses_ahead,
		carmen_robot_and_trailers_traj_point_t *poses_back, int num_poses, int num_poses_back, int *annotations, int * annotations_codes)
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
carmen_rddf_subscribe_end_point_message(carmen_rddf_end_point_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;

	if (!initialized)
	{
        err = IPC_defineMsg(CARMEN_RDDF_END_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_END_POINT_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_END_POINT_MESSAGE_NAME);

		initialized = 1;
	}
    carmen_subscribe_message(CARMEN_RDDF_END_POINT_MESSAGE_NAME, CARMEN_RDDF_END_POINT_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_end_point_message), handler, subscribe_how);
}

void
carmen_rddf_publish_end_point_message(int half_meters_to_final_goal, carmen_robot_and_trailers_pose_t point)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_end_point_message rddf_end_point_message;

    rddf_end_point_message.half_meters_to_final_goal = half_meters_to_final_goal;

    carmen_robot_and_trailers_pose_t end_point;
    end_point.x = point.x;
    end_point.y = point.y;
    end_point.theta = point.theta;
    end_point.num_trailers = point.num_trailers;
    for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
    	end_point.trailer_theta[z] = point.trailer_theta[z];

    rddf_end_point_message.point = end_point;
    rddf_end_point_message.timestamp = carmen_get_time();
    rddf_end_point_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_END_POINT_MESSAGE_NAME, &rddf_end_point_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_END_POINT_MESSAGE_NAME);
}



void
carmen_rddf_subscribe_add_annotation_message(carmen_rddf_add_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;
    if (!initialized)
    {
        err = IPC_defineMsg(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME);
        initialized = 1;
    }
    carmen_subscribe_message(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_add_annotation_message), handler, subscribe_how);
}

void
carmen_rddf_unsubscribe_add_annotation_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, handler);
}
/*
void
carmen_rddf_publish_add_annotation_message(carmen_vector_3D_t annotation_point, double orientation, char *annotation_description, int annotation_type, int annotation_code, int annotation_id)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_add_annotation_message rddf_add_annotation_message;

    rddf_add_annotation_message.annotation_point = annotation_point;
    rddf_add_annotation_message.annotation_orientation = orientation;
    rddf_add_annotation_message.annotation_description = annotation_description;
    rddf_add_annotation_message.annotation_type = annotation_type;
    rddf_add_annotation_message.annotation_code = annotation_code;
    rddf_add_annotation_message.annotation_id = annotation_id;
    rddf_add_annotation_message.timestamp = carmen_get_time();
    rddf_add_annotation_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME, &rddf_add_annotation_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ADD_ANNOTATION_MESSAGE_NAME);
}

*/

void
carmen_rddf_subscribe_update_annotation_message(carmen_rddf_update_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int initialized = 0;
    if (!initialized)
    {
        err = IPC_defineMsg(CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_NAME);
        carmen_subscribe_message(CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_NAME, CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_update_annotation_message), handler, subscribe_how);
        initialized = 1;
    }
}

void
carmen_rddf_publish_update_annotation_message(crud_t action, carmen_annotation_t old_annotation, carmen_annotation_t new_annotation)
{
    IPC_RETURN_TYPE err;
    carmen_rddf_update_annotation_message rddf_update_annotation_message;

    rddf_update_annotation_message.action = action;
    rddf_update_annotation_message.old_annotation = old_annotation;
    rddf_update_annotation_message.new_annotation = new_annotation;
    rddf_update_annotation_message.timestamp = carmen_get_time();
    rddf_update_annotation_message.host = carmen_get_host();

    err = IPC_publishData(CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_NAME, &rddf_update_annotation_message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_UPDATE_ANNOTATION_MESSAGE_NAME);
}



void
carmen_rddf_subscribe_annotation_message(carmen_rddf_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    IPC_RETURN_TYPE err;
    static int inicialized = 0;
    if (!inicialized)
    {
        err = IPC_defineMsg(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);
        carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_ANNOTATION_MESSAGE_NAME);
        inicialized = 1;
    }
    carmen_subscribe_message(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, CARMEN_RDDF_ANNOTATION_MESSAGE_FMT,
                             message, sizeof (carmen_rddf_annotation_message), handler, subscribe_how);
}
void
carmen_rddf_unsubscribe_annotation_message(carmen_handler_t handler)
{
    carmen_unsubscribe_message(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, handler);
}

/*
void
carmen_rddf_publish_annotation_message(carmen_rddf_annotation_message message)
{
	IPC_RETURN_TYPE err;
    
    err = IPC_publishData( CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &message );
	carmen_test_ipc_exit( err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_NAME );
}
*/

void 
carmen_route_planner_subscribe_change_weight_node_message(carmen_route_planner_change_weight_node_message *msg, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	IPC_RETURN_TYPE err;
    static int inicialized = 0;
    if (!inicialized)
    {
		err = IPC_defineMsg(CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_NAME);
		inicialized = 1;
	}
	carmen_subscribe_message(CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_NAME, CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_FMT, 
							msg, sizeof(carmen_route_planner_change_weight_node_message), handler, subscribe_how);
}

void 
carmen_route_planner_publish_change_weight_node_message(carmen_route_planner_change_weight_node_message *msg)
{
	IPC_RETURN_TYPE err;
    
    err = IPC_publishData( CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_NAME, msg );
	carmen_test_ipc_exit( err, "Could not publish", CARMEN_ROUTE_PLANNER_CHANGE_WEIGHT_NODE_NAME);
}


//*****************************************  Obstacle Avoider  ******************************************* *//

void
carmen_robot_ackerman_subscribe_road_velocity_control_message(carmen_robot_ackerman_road_velocity_control_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_NAME,
			CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_FMT,
			msg, sizeof(carmen_robot_ackerman_road_velocity_control_message),
			handler, subscribe_how);
}

void
carmen_obstacle_avoider_subscribe_robot_hit_obstacle_message(carmen_obstacle_avoider_robot_will_hit_obstacle_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME,
			CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_FMT,
			msg, sizeof(carmen_obstacle_avoider_robot_will_hit_obstacle_message),
			handler, subscribe_how);
}



//*****************************************  mpp/Model Predictive  ******************************************* *//

void
carmen_model_predictive_planner_define_motion_plan_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME);
}

void
carmen_model_predictive_planner_define_all()
{
	carmen_model_predictive_planner_define_motion_plan_message();
}

void
carmen_model_predictive_planner_subscribe_motion_plan_message(carmen_model_predictive_planner_motion_plan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_FMT,
                             message, sizeof (carmen_model_predictive_planner_motion_plan_message), handler, subscribe_how);
}

static int release_inicialized = 0;
/*
void carmen_release_annotation_subscribe_message(carmen_release_annotation_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    if (!release_inicialized)
        define_msg(&release_inicialized, CARMEN_RELEASE_ANNOTATION_MESSAGE_NAME, CARMEN_RELEASE_ANNOTATION_MESSAGE_FMT);

    carmen_subscribe_message(CARMEN_RELEASE_ANNOTATION_MESSAGE_NAME, CARMEN_RELEASE_ANNOTATION_MESSAGE_FMT, message, sizeof( carmen_release_annotation_message ), handler, subscribe_how);
}

void carmen_release_annotation_publish_message( carmen_release_annotation_message *message )
{
    IPC_RETURN_TYPE err; 

    if (!release_inicialized)
        define_msg(&release_inicialized, CARMEN_RELEASE_ANNOTATION_MESSAGE_NAME, CARMEN_RELEASE_ANNOTATION_MESSAGE_FMT);

    err = IPC_publishData(CARMEN_RELEASE_ANNOTATION_MESSAGE_NAME, message);
    carmen_test_ipc_exit( err, "Could not publish", CARMEN_RELEASE_ANNOTATION_MESSAGE_NAME);
}
*/

static int offroad_activate_inicialized = 0;

/*
void carmen_offroad_activate_subscribe_message(carmen_offroad_activate_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    if (!offroad_activate_inicialized)
        define_msg(&offroad_activate_inicialized, CARMEN_OFFROAD_ACTIVATE_MESSAGE_NAME, CARMEN_OFFROAD_ACTIVATE_MESSAGE_FMT);

    carmen_subscribe_message(CARMEN_OFFROAD_ACTIVATE_MESSAGE_NAME, CARMEN_OFFROAD_ACTIVATE_MESSAGE_FMT, message, sizeof( carmen_release_annotation_message ), handler, subscribe_how);
}

void carmen_offroad_activate_publish_message( carmen_offroad_activate_message message )
{
    IPC_RETURN_TYPE err; 

    if (!offroad_activate_inicialized)
        define_msg(&offroad_activate_inicialized, CARMEN_OFFROAD_ACTIVATE_MESSAGE_NAME, CARMEN_OFFROAD_ACTIVATE_MESSAGE_FMT);

    err = IPC_publishData(CARMEN_OFFROAD_ACTIVATE_MESSAGE_NAME, &message);
    carmen_test_ipc_exit( err, "Could not publish", CARMEN_OFFROAD_ACTIVATE_MESSAGE_NAME);
}
*/
void
carmen_model_predictive_planner_unsubscribe_motion_plan_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, handler);
}

void
carmen_model_predictive_planner_publish_motion_plan_message(carmen_robot_and_trailers_traj_point_t *plan, int plan_length)
{
    IPC_RETURN_TYPE err;
	carmen_model_predictive_planner_motion_plan_message motion_plan_msg;

	static int first_time = 1;

	if (first_time)
	{
		first_time = 0;
		carmen_model_predictive_planner_define_motion_plan_message();
	}

	motion_plan_msg.plan = plan;
	motion_plan_msg.plan_length = plan_length;
	motion_plan_msg.timestamp = carmen_get_time();
	motion_plan_msg.host = carmen_get_host();

    err = IPC_publishData(CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME, &motion_plan_msg);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_MODEL_PREDICTIVE_PLANNER_MOTION_PLAN_MESSAGE_NAME);
}