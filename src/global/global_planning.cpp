#include "global_planning.h"
bool
annotations_target_point_is_forward(carmen_point_t robot_pose, carmen_point_t annotation_point)
{
	double cos_theta = cos(-robot_pose.theta);
	double sin_theta = sin(-robot_pose.theta);

	carmen_point_t robot_pose_inverse = {-(robot_pose.x * cos_theta - robot_pose.y * sin_theta),
						  -(robot_pose.x * sin_theta + robot_pose.y * cos_theta),
						  -robot_pose.theta};



	cos_theta = cos(robot_pose_inverse.theta);
	sin_theta = sin(robot_pose_inverse.theta);
	double new_x = robot_pose_inverse.x + annotation_point.x * cos_theta - annotation_point.y * sin_theta;
	double new_y = robot_pose_inverse.y + annotation_point.x * sin_theta + annotation_point.y * cos_theta;
	double new_theta = robot_pose_inverse.theta + annotation_point.theta;
	carmen_point_t annotation_in_car_reference = {new_x, new_y, new_theta};

	return (annotation_in_car_reference.x > 0.0);
}


int 
carmen_global_planning_pose_passed_through_annotation(int annotation_type, carmen_point_t pose, carmen_rddf_annotation_message *annotation_message)
{
	static annotation_state_transition vector_annotation_transition[NUM_RDDF_ANNOTATION_TYPES];
	static int first = 1;

	if (first == 1)
	{
		for (int i = 0; i < NUM_RDDF_ANNOTATION_TYPES; i++)
		{
			vector_annotation_transition[i] = NOT_IN_FRONT;
		}

		first = 0;
	}

	double distance_to_nearest_annotation = 1000.0;
	int index_of_nearest_annotation = -1;

	for (int i = 0; i < annotation_message->num_annotations; i++)
	{
		if (annotation_message->annotations[i].annotation_type != annotation_type)
		{
			continue;
		}

		double distance_to_annotation = DIST2D(annotation_message->annotations[i].annotation_point, pose);

		if (distance_to_annotation < distance_to_nearest_annotation)
		{
			distance_to_nearest_annotation = distance_to_annotation;
			index_of_nearest_annotation = i;
		}
	}

	if (index_of_nearest_annotation < 0)
	{
		return (-3); // Anotação não encontrada
	}

	carmen_point_t annotation_point = {annotation_message->annotations[index_of_nearest_annotation].annotation_point.x, annotation_message->annotations[index_of_nearest_annotation].annotation_point.y, 0.0};
	if (annotations_target_point_is_forward(pose, annotation_point) == true)
	{
		vector_annotation_transition[annotation_type] = IS_IN_FRONT;
		return (-2); // Anotação está IN_FRONT
	}
	else
	{
		if (vector_annotation_transition[annotation_type] == IS_IN_FRONT)
		{
			vector_annotation_transition[annotation_type] = NOT_IN_FRONT;
			return (index_of_nearest_annotation); // Anotação acabou de deixar de ser in_front (JUST_STOPPED_BEING_IN_FRONT)
		}
		else
		{
			vector_annotation_transition[annotation_type] = NOT_IN_FRONT;
			return (-1); // Anotação não está in_front
		}
	}

	return (-99); // Não entrou nas condições
}
