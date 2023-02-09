/*
 * behavior_selector_interface.c
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */
#include "behavior_selector_interface.h"


void carmen_behavior_selector_subscribe_current_state_message(
		carmen_behavior_selector_state_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME,
			CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT,
			msg, sizeof(carmen_behavior_selector_state_message),
			handler, subscribe_how);
}

void carmen_behavior_selector_set_task(carmen_behavior_selector_task_t task, double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_set_task_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_SET_TASK_NAME, IPC_VARIABLE_LENGTH, CARMEN_BEHAVIOR_SELECTOR_SET_TASK_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_SET_TASK_NAME);
		initialized = 1;
	}

	msg.task = task;
	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_SET_TASK_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_SET_TASK_NAME);
}

void carmen_behavior_selector_add_goal(carmen_point_t goal, double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_add_goal_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME);
		initialized = 1;
	}

	msg.goal = goal;
	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME);
}

void carmen_behavior_selector_clear_goal_list(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_clear_goal_list_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME);
		initialized = 1;
	}

	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME);
}

void carmen_behavior_selector_remove_goal(double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_remove_goal_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME);
		initialized = 1;
	}

	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME);
}

void
carmen_behavior_selector_set_algorithm(carmen_behavior_selector_algorithm_t algorithm,  carmen_behavior_selector_task_t task, double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_set_algorithm_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME, IPC_VARIABLE_LENGTH, CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME);
		initialized = 1;
	}

	msg.algorithm = algorithm;
	msg.task = task;
	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME);
}


char *
get_low_level_state_name(carmen_behavior_selector_low_level_state_t state)
{
	if (state == Initializing) 						return ((char *) "Initializing");
	if (state == Stopped) 							return ((char *) "Stopped");
	if (state == Free_Running) 						return ((char *) "Free_Running");
	if (state == Free_Reverse_Running) 				return ((char *) "Free_Reverse_Running");
	if (state == Stopping_Behind_Moving_Object) 	return ((char *) "Stopping_Behind_Moving_Object");
	if (state == Stopped_Behind_Moving_Object_S0) 	return ((char *) "Stopped_Behind_Moving_Object_S0");
	if (state == Stopped_Behind_Moving_Object_S1) 	return ((char *) "Stopped_Behind_Moving_Object_S1");
	if (state == Stopped_Behind_Moving_Object_S2) 	return ((char *) "Stopped_Behind_Moving_Object_S2");
	if (state == Stopping_At_Red_Traffic_Light)     return ((char *) "Stopping_At_Red_Traffic_Light");
	if (state == Stopped_At_Red_Traffic_Light_S0) 	return ((char *) "Stopped_At_Red_Traffic_Light_S0");
	if (state == Stopped_At_Red_Traffic_Light_S1) 	return ((char *) "Stopped_At_Red_Traffic_Light_S1");
	if (state == Stopped_At_Red_Traffic_Light_S2) 	return ((char *) "Stopped_At_Red_Traffic_Light_S2");
	if (state == Stopping_At_Busy_Pedestrian_Track)     return ((char *) "Stopping_At_Busy_Pedestrian_Track");
	if (state == Stopped_At_Busy_Pedestrian_Track_S0) 	return ((char *) "Stopped_At_Busy_Pedestrian_Track_S0");
	if (state == Stopped_At_Busy_Pedestrian_Track_S1) 	return ((char *) "Stopped_At_Busy_Pedestrian_Track_S1");
	if (state == Stopped_At_Busy_Pedestrian_Track_S2) 	return ((char *) "Stopped_At_Busy_Pedestrian_Track_S2");
	if (state == Stopping_At_Yield)     			return ((char *) "Stopping_At_Yield");
	if (state == Stopped_At_Yield_S0) 				return ((char *) "Stopped_At_Yield_S0");
	if (state == Stopped_At_Yield_S1) 				return ((char *) "Stopped_At_Yield_S1");
	if (state == Stopped_At_Yield_S2) 				return ((char *) "Stopped_At_Yield_S2");
	if (state == Stopping_At_Stop_Sign) 			return ((char *) "Stopping_At_Stop_Sign");
	if (state == Stopped_At_Stop_Sign_S0) 			return ((char *) "Stopped_At_Stop_Sign_S0");
	if (state == Stopped_At_Stop_Sign_S1) 			return ((char *) "Stopped_At_Stop_Sign_S1");
	if (state == Stopped_At_Stop_Sign_S2) 			return ((char *) "Stopped_At_Stop_Sign_S2");
	if (state == Stopping_To_Reverse) 				return ((char *) "Stopping_To_Reverse");
	if (state == Stopped_At_Reverse_S0) 			return ((char *) "Stopped_At_Reverse_S0");
	if (state == Stopped_At_Reverse_S1) 			return ((char *) "Stopped_At_Reverse_S1");
	if (state == Stopped_At_Reverse_S2) 			return ((char *) "Stopped_At_Reverse_S2");
	if (state == Stopping_To_Go_Forward) 			return ((char *) "Stopping_To_Go_Forward");
	if (state == Stopped_At_Go_Forward_S0) 			return ((char *) "Stopped_At_Go_Forward_S0");
	if (state == Stopped_At_Go_Forward_S1) 			return ((char *) "Stopped_At_Go_Forward_S1");
	if (state == Stopped_At_Go_Forward_S2) 			return ((char *) "Stopped_At_Go_Forward_S2");
	if (state == Stopping_To_Pedestrian) 			return ((char *) "Stopping_To_Pedestrian");
	if (state == Stopped_At_Pedestrian_S0) 			return ((char *) "Stopped_At_Pedestrian_S0");
	if (state == Stopped_At_Pedestrian_S1) 			return ((char *) "Stopped_At_Pedestrian_S1");
	if (state == Stopped_At_Pedestrian_S2) 			return ((char *) "Stopped_At_Pedestrian_S2");
	if (state == Stopping_At_Busy_Queue) 			return ((char *) "Stopping_At_Busy_Queue");
	if (state == Stopped_At_Busy_Queue_S0) 			return ((char *) "Stopped_At_Busy_Queue_S0");
	if (state == Stopped_At_Busy_Queue_S1) 			return ((char *) "Stopped_At_Busy_Queue_S1");
	if (state == Stopped_At_Busy_Queue_S2) 			return ((char *) "Stopped_At_Busy_Queue_S2");
	if (state == Stopping_At_Unavoidable_Obstacle) 			return ((char *) "Stopping_At_Unavoidable_Obstacle");
	if (state == Stopped_At_Unavoidable_Obstacle_S0) 			return ((char *) "Stopped_At_Unavoidable_Obstacle_S0");
	if (state == End_Of_Path_Reached) 				return ((char *) "End_Of_Path_Reached");
	if (state == End_Of_Path_Reached2) 				return ((char *) "End_Of_Path_Reached2");
	if (state == Recovering_From_Error) 			return ((char *) "Recovering_From_Error");

	return ((char *) " ");
}


char *
get_low_level_state_flag_name(int flag)
{
	static char str_flags[2048];

	str_flags[0] = '\0';

	if (flag & CARMEN_BEHAVIOR_SELECTOR_GOING_BACKWARDS)		strcat(str_flags, (char *) ", GOING_BACKWARDS");
	if (flag & CARMEN_BEHAVIOR_SELECTOR_WITHIN_NARROW_PASSAGE)	strcat(str_flags, (char *) ", NARROW_PASSAGE");

	return (str_flags);
}


void
carmen_behavior_selector_subscribe_path_goals_and_annotations_message(carmen_behavior_selector_path_goals_and_annotations_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_NAME,
			CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_FMT,
			msg, sizeof(carmen_behavior_selector_path_goals_and_annotations_message),
			handler, subscribe_how);
}


void
visit_message_fields(carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message)
{
	carmen_behavior_selector_path_goals_and_annotations_message *m = path_goals_and_annotations_message;
	for (int i = 0; i < m->number_of_poses; i++)
	{
		printf("poses %lf %lf %lf %lf %lf\n", m->poses[i].x, m->poses[i].y, m->poses[i].theta, m->poses[i].v, m->poses[i].phi);
		printf("annotations %d\n", m->annotations[i]);
		printf("annotations_codes %d\n", m->annotations_codes[i]);
	}
	for (int i = 0; i < m->number_of_poses_back; i++)
	{
		printf("poses_back %lf %lf %lf %lf %lf\n",
				m->poses_back[i].x, m->poses_back[i].y, m->poses_back[i].theta, m->poses_back[i].v, m->poses_back[i].phi);
	}
	for (int i = 0; i < m->goal_list_size; i++)
	{
		printf("goal_list %lf %lf %lf %lf %lf\n",
				m->goal_list[i].x, m->goal_list[i].y, m->goal_list[i].theta, m->goal_list[i].v, m->goal_list[i].phi);
	}
	printf("%lf %s\n", m->timestamp, m->host);
}


void
carmen_behavior_selector_publish_path_goals_and_annotations_message(carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations_message)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static int initialized = 0;

//	visit_message_fields(path_goals_and_annotations_message);
	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_NAME);
		initialized = 1;
	}

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_NAME, path_goals_and_annotations_message);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_PATH_GOALS_AND_ANNOTATIONS_MESSAGE_NAME);
}


carmen_annotation_t *
carmen_behavior_selector_get_nearest_specified_annotation(int annotation, carmen_rddf_annotation_message annotation_message, carmen_robot_and_trailers_traj_point_t *current_robot_pose_v_and_phi)
{
	int nearest_annotation_index = -1;
	double distance_to_nearest_annotation = 1000.0;

	for (int i = 0; i < annotation_message.num_annotations; i++)
	{
		double distance_to_annotation = DIST2D_P(&annotation_message.annotations[i].annotation_point, current_robot_pose_v_and_phi);

		if ((annotation_message.annotations[i].annotation_type == annotation) &&
			(distance_to_annotation < distance_to_nearest_annotation))
		{
			distance_to_nearest_annotation = distance_to_annotation;
			nearest_annotation_index = i;
		}
	}

	if (nearest_annotation_index != -1)
		return (&(annotation_message.annotations[nearest_annotation_index]));
	else
		return (NULL);
}
