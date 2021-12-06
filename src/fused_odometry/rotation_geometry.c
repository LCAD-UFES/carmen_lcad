#include <carmen/carmen.h>

#include "rotation_geometry.h"


//	Let roll = r, pitch = p, yaw = y
//
//	This matrix is the result of first rotating the z axis by yaw, then the y axis by pitch and then the x axis by roll
//	assuming the frame of reference rotates with the object.
//
//  See Xsens MTIG manual, pg 31
//
//	| 	cos(p)*cos(y)		-cos(r)*sin(y)+sin(r)*sin(p)*cos(y)		sin(r)*sin(y)+cos(r)*sin(p)*cos(y)	|
//	| 	cos(p)*sin(y)		cos(r)*cos(y)+sin(r)*sin(p)*sin(y)		-sin(r)*cos(y)+cos(r)*sin(p)*sin(y)	|
//	| 		-sin(p)		 			sin(r)*cos(p)								cos(r)*cos(p)			|
//

rotation_matrix *
create_rotation_matrix(carmen_orientation_3D_t orientation)
{
	rotation_matrix *r_matrix = malloc(sizeof(rotation_matrix));
	
	double sinRoll 	= sin(orientation.roll);
	double cosRoll	= cos(orientation.roll);
	double sinPitch = sin(orientation.pitch);
	double cosPitch = cos(orientation.pitch);
	double sinYaw 	= sin(orientation.yaw);
	double cosYaw 	= cos(orientation.yaw);

	// The first parameter is the line and the second the column, i.e. [0 + 3*1] is line 0 column 1.
	r_matrix->matrix[0 + 3*0] = cosPitch*cosYaw;
	r_matrix->matrix[0 + 3*1] = -cosRoll*sinYaw + sinRoll*sinPitch*cosYaw;
	r_matrix->matrix[0 + 3*2] = sinRoll*sinYaw + cosRoll*sinPitch*cosYaw;
	r_matrix->matrix[1 + 3*0] = cosPitch*sinYaw;
	r_matrix->matrix[1 + 3*1] = cosRoll*cosYaw + sinRoll*sinPitch*sinYaw;
	r_matrix->matrix[1 + 3*2] = -sinRoll*cosYaw + cosRoll*sinPitch*sinYaw;
	r_matrix->matrix[2 + 3*0] = -sinPitch;
	r_matrix->matrix[2 + 3*1] = sinRoll*cosPitch;
	r_matrix->matrix[2 + 3*2] = cosRoll*cosPitch;

	return (r_matrix);
}


rotation_matrix *
compute_rotation_matrix(rotation_matrix *r_matrix, carmen_orientation_3D_t orientation)
{
	if (!r_matrix)
		r_matrix = (rotation_matrix *) malloc(sizeof(rotation_matrix));
	
	double sinRoll 	= sin(orientation.roll);
	double cosRoll	= cos(orientation.roll);
	double sinPitch = sin(orientation.pitch);
	double cosPitch = cos(orientation.pitch);
	double sinYaw 	= sin(orientation.yaw);
	double cosYaw 	= cos(orientation.yaw);

	r_matrix->matrix[0 + 3*0] = cosPitch*cosYaw;
	r_matrix->matrix[0 + 3*1] = -cosRoll*sinYaw + sinRoll*sinPitch*cosYaw;
	r_matrix->matrix[0 + 3*2] = sinRoll*sinYaw + cosRoll*sinPitch*cosYaw;
	r_matrix->matrix[1 + 3*0] = cosPitch*sinYaw;
	r_matrix->matrix[1 + 3*1] = cosRoll*cosYaw + sinRoll*sinPitch*sinYaw;
	r_matrix->matrix[1 + 3*2] = -sinRoll*cosYaw + cosRoll*sinPitch*sinYaw;
	r_matrix->matrix[2 + 3*0] = -sinPitch;
	r_matrix->matrix[2 + 3*1] = sinRoll*cosPitch;
	r_matrix->matrix[2 + 3*2] = cosRoll*cosPitch;

	return (r_matrix);
}


rotation_matrix *
create_rotation_matrix_inverse(carmen_orientation_3D_t orientation)
{
	rotation_matrix *r_matrix = malloc(sizeof(rotation_matrix));
	
	double sinRoll 	= sin(orientation.roll);
	double cosRoll	= cos(orientation.roll);
	double sinPitch = sin(orientation.pitch);
	double cosPitch = cos(orientation.pitch);
	double sinYaw 	= sin(orientation.yaw);
	double cosYaw 	= cos(orientation.yaw);

	r_matrix->matrix[0 + 3*0] = cosPitch*cosYaw;
	r_matrix->matrix[1 + 3*0] = -cosRoll*sinYaw + sinRoll*sinPitch*cosYaw;
	r_matrix->matrix[2 + 3*0] = sinRoll*sinYaw + cosRoll*sinPitch*cosYaw;
	r_matrix->matrix[0 + 3*1] =	cosPitch*sinYaw;
	r_matrix->matrix[1 + 3*1] = cosRoll*cosYaw + sinRoll*sinPitch*sinYaw;
	r_matrix->matrix[2 + 3*1] = -sinRoll*cosYaw + cosRoll*sinPitch*sinYaw;
	r_matrix->matrix[0 + 3*2] = -sinPitch;
	r_matrix->matrix[1 + 3*2] = sinRoll*cosPitch;
	r_matrix->matrix[2 + 3*2] = cosRoll*cosPitch;

	return r_matrix;	
}


rotation_matrix *
create_rotation_matrix_from_matrix(double matrix[3][3])
{
	rotation_matrix *r_matrix = malloc(sizeof(rotation_matrix));
		
	r_matrix->matrix[0 + 3*0] = matrix[0][0];
	r_matrix->matrix[1 + 3*0] = matrix[1][0];
	r_matrix->matrix[2 + 3*0] = matrix[2][0];
	r_matrix->matrix[0 + 3*1] =	matrix[0][1];
	r_matrix->matrix[1 + 3*1] = matrix[1][1];
	r_matrix->matrix[2 + 3*1] = matrix[2][1];
	r_matrix->matrix[0 + 3*2] = matrix[0][2];
	r_matrix->matrix[1 + 3*2] = matrix[1][2];
	r_matrix->matrix[2 + 3*2] = matrix[2][2];

	return (r_matrix);
}


rotation_matrix *
create_rotation_matrix_from_matrix_inverse(double matrix[3][3])
{
	rotation_matrix *r_matrix = malloc(sizeof(rotation_matrix));
		
	r_matrix->matrix[0 + 3*0] = matrix[0][0];
	r_matrix->matrix[1 + 3*0] = matrix[0][1];
	r_matrix->matrix[2 + 3*0] = matrix[0][2];
	r_matrix->matrix[0 + 3*1] =	matrix[1][0];
	r_matrix->matrix[1 + 3*1] = matrix[1][1];
	r_matrix->matrix[2 + 3*1] = matrix[1][2];
	r_matrix->matrix[0 + 3*2] = matrix[2][0];
	r_matrix->matrix[1 + 3*2] = matrix[2][1];
	r_matrix->matrix[2 + 3*2] = matrix[2][2];

	return (r_matrix);
}


rotation_matrix *
create_rotation_matrix_from_vector(double *matrix)
{
	rotation_matrix *r_matrix = malloc(sizeof(rotation_matrix));
		
	r_matrix->matrix[0 + 3*0] = matrix[0 + 3*0];
	r_matrix->matrix[1 + 3*0] = matrix[1 + 3*0];
	r_matrix->matrix[2 + 3*0] = matrix[2 + 3*0];
	r_matrix->matrix[0 + 3*1] =	matrix[0 + 3*1];
	r_matrix->matrix[1 + 3*1] = matrix[1 + 3*1];
	r_matrix->matrix[2 + 3*1] = matrix[2 + 3*1];
	r_matrix->matrix[0 + 3*2] = matrix[0 + 3*2];
	r_matrix->matrix[1 + 3*2] = matrix[1 + 3*2];
	r_matrix->matrix[2 + 3*2] = matrix[2 + 3*2];

	return (r_matrix);
}


rotation_matrix *
create_rotation_matrix_from_vector_inverse(double *matrix)
{
	rotation_matrix* r_matrix = malloc(sizeof(rotation_matrix));
		
	r_matrix->matrix[0 + 3*0] = matrix[0 + 3*0];
	r_matrix->matrix[1 + 3*0] = matrix[0 + 3*1];
	r_matrix->matrix[2 + 3*0] = matrix[0 + 3*2];
	r_matrix->matrix[0 + 3*1] =	matrix[1 + 3*0];
	r_matrix->matrix[1 + 3*1] = matrix[1 + 3*1];
	r_matrix->matrix[2 + 3*1] = matrix[1 + 3*2];
	r_matrix->matrix[0 + 3*2] = matrix[2 + 3*0];
	r_matrix->matrix[1 + 3*2] = matrix[2 + 3*1];
	r_matrix->matrix[2 + 3*2] = matrix[2 + 3*2];

	return (r_matrix);
}


rotation_matrix *
create_rotation_matrix_from_quaternions(carmen_quaternion_t quat)
{
	double q0 = quat.q0;
	double q1 = quat.q1;
	double q2 = quat.q2;
	double q3 = quat.q3;

	rotation_matrix *r_matrix = malloc(sizeof(rotation_matrix));
		
	r_matrix->matrix[0 + 3*0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	r_matrix->matrix[0 + 3*1] = 2*(q1*q2 - q0*q3);
	r_matrix->matrix[0 + 3*2] = 2*(q0*q2 + q1*q3);
	r_matrix->matrix[1 + 3*0] =	2*(q1*q2 + q0*q3);
	r_matrix->matrix[1 + 3*1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	r_matrix->matrix[1 + 3*2] = 2*(q2*q3 - q0*q1);
	r_matrix->matrix[2 + 3*0] = 2*(q1*q3 - q0*q2);
	r_matrix->matrix[2 + 3*1] = 2*(q0*q1 + q2*q3);
	r_matrix->matrix[2 + 3*2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	return (r_matrix);
}


rotation_matrix *
transpose_matrix(rotation_matrix *matrix_original)
{
	rotation_matrix *matrix_transpose = malloc(sizeof(rotation_matrix));

	matrix_transpose->matrix[0 + 3*0] = matrix_original->matrix[0 + 3*0];
	matrix_transpose->matrix[0 + 3*1] = matrix_original->matrix[1 + 3*0];
	matrix_transpose->matrix[0 + 3*2] = matrix_original->matrix[2 + 3*0];
	matrix_transpose->matrix[1 + 3*0] = matrix_original->matrix[0 + 3*1];
	matrix_transpose->matrix[1 + 3*1] = matrix_original->matrix[1 + 3*1];
	matrix_transpose->matrix[1 + 3*2] = matrix_original->matrix[2 + 3*1];
	matrix_transpose->matrix[2 + 3*0] = matrix_original->matrix[0 + 3*2];
	matrix_transpose->matrix[2 + 3*1] = matrix_original->matrix[1 + 3*2];
	matrix_transpose->matrix[2 + 3*2] = matrix_original->matrix[2 + 3*2];

	return (matrix_transpose);
}


void
destroy_rotation_matrix(rotation_matrix *r_matrix)
{
	free(r_matrix);
}


carmen_vector_3D_t
multiply_matrix_vector(rotation_matrix *r_matrix, carmen_vector_3D_t vector)
{
	carmen_vector_3D_t result;	
	
	result.x = r_matrix->matrix[0 + 3*0]*vector.x + r_matrix->matrix[0 + 3*1]*vector.y + r_matrix->matrix[0 + 3*2]*vector.z;
	result.y = r_matrix->matrix[1 + 3*0]*vector.x + r_matrix->matrix[1 + 3*1]*vector.y + r_matrix->matrix[1 + 3*2]*vector.z;
	result.z = r_matrix->matrix[2 + 3*0]*vector.x + r_matrix->matrix[2 + 3*1]*vector.y + r_matrix->matrix[2 + 3*2]*vector.z;
	
	return (result);
}


rotation_matrix *
multiply_matrix_matrix(rotation_matrix *matrix_1, rotation_matrix *matrix_2)
{
	rotation_matrix *result = malloc(sizeof(rotation_matrix));

	int i, j, k;

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result->matrix[i + 3*j] = 0.0;

			for(k=0; k<3; k++)
			{
				result->matrix[i + 3*j] += matrix_1->matrix[k + 3*j] * matrix_2->matrix[i + 3*k];
			}
		}
	}

	return (result);
}


// The roll, pitch and yaw angles returned by this function are in the convetion
// that first rotates by yaw, then by pitch and then by roll, with the frame of reference rotating with the object
carmen_orientation_3D_t
get_angles_from_rotation_matrix(rotation_matrix *r_matrix)
{
	carmen_orientation_3D_t angles;

	angles.roll = 	atan2(r_matrix->matrix[2 + 3*1], r_matrix->matrix[2 + 3*2]);
	angles.pitch = 	-asin(r_matrix->matrix[2 + 3*0]);
	angles.yaw = 	atan2(r_matrix->matrix[1 + 3*0], r_matrix->matrix[0 + 3*0]);

	return (angles);
}


carmen_vector_3D_t
add_vectors(carmen_vector_3D_t v1, carmen_vector_3D_t v2)
{
	carmen_vector_3D_t result;

	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;
	result.z = v1.z + v2.z;

	return (result);
}


carmen_vector_3D_t
sub_vectors(carmen_vector_3D_t v1, carmen_vector_3D_t v2)
{
	carmen_vector_3D_t result;

	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;

	return (result);
}


carmen_vector_3D_t
carmen_change_sensor_reference(carmen_vector_3D_t position, carmen_vector_3D_t reference, rotation_matrix* transformation_matrix)
{
	carmen_vector_3D_t new_reference = multiply_matrix_vector(transformation_matrix, reference);
	carmen_vector_3D_t point = add_vectors(new_reference, position);

	return (point);
}


// @@@ Esta funcao nao considera a velocidade angular
carmen_vector_3D_t
carmen_get_interpolated_robot_position_at_time(carmen_pose_3D_t robot_pose, carmen_vector_3D_t robot_velocity, double robot_time, double interpolated_time, rotation_matrix* r_matrix_robot_to_global)
{
	double dt = interpolated_time - robot_time;

	carmen_vector_3D_t displacement_robot_reference;
	displacement_robot_reference.x = robot_velocity.x * dt; // @@@ a variacao angular nao ee considerada...
	displacement_robot_reference.y = robot_velocity.y * dt;
	displacement_robot_reference.z = robot_velocity.z * dt;

	return (carmen_change_sensor_reference(robot_pose.position, displacement_robot_reference, r_matrix_robot_to_global));
}


carmen_pose_3D_t
carmen_ackerman_interpolated_robot_position_at_time(carmen_pose_3D_t robot_pose, double dt, double v, double phi, double distance_between_front_and_rear_axles)
{
	carmen_pose_3D_t pose = robot_pose;
	int i;
	int steps = 1;
	double ds;

	ds = v * (dt / (double) steps);

	for (i = 0; i < steps; i++)
	{
		pose.position.x = pose.position.x + ds * cos(pose.orientation.yaw);
		pose.position.y = pose.position.y + ds * sin(pose.orientation.yaw);
		pose.orientation.yaw = carmen_normalize_theta(pose.orientation.yaw + ds * (tan(phi) / distance_between_front_and_rear_axles));
	}
	return (pose);
}


carmen_vector_3D_t
carmen_get_sensor_sphere_point_in_robot_cartesian_reference(carmen_sphere_coord_t sphere_point, carmen_pose_3D_t sensor_pose, carmen_pose_3D_t sensor_board_pose, rotation_matrix* sensor_to_board_matrix, rotation_matrix* board_to_car_matrix)
{
	carmen_vector_3D_t sensor_reference, board_reference, car_reference;

	// ****************************************
	// TODO: Colocar TF nessas transformadas!!!
	// ****************************************

	sensor_reference = carmen_covert_sphere_to_cartesian_coord(sphere_point);
	board_reference = carmen_change_sensor_reference(sensor_pose.position, sensor_reference, sensor_to_board_matrix);
	car_reference = carmen_change_sensor_reference(sensor_board_pose.position, board_reference, board_to_car_matrix);

	return (car_reference);
}
