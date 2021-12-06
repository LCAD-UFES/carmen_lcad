#ifndef GEOMETRY_H
#define GEOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

struct _rotation_matrix
{
	double matrix[9];
};

typedef struct _rotation_matrix rotation_matrix;

double *get_pointer_from_rotation_matrix(rotation_matrix *matrix);

//	Let roll = r, pitch = p, yaw = y
//
//	This matrix is the result of first rotating the z axis by yaw, then the y axis by pitch and then the x axis by roll.
//
//	| cos(p)*cos(y)		-cos(r)*sin(y)+sin(r)*sin(p)*cos(y)		 sin(r)*sin(y)+cos(r)*sin(p)*cos(y)	|
//	| cos(p)*sin(y)		 cos(r)*cos(y)+sin(r)*sin(p)*sin(y)		-sin(r)*cos(y)+cos(r)*sin(p)*sin(y)	|
//	|    -sin(p)					sin(r)*cos(p)							cos(r)*cos(p)			|
rotation_matrix* create_rotation_matrix(carmen_orientation_3D_t orientation);
rotation_matrix* compute_rotation_matrix(rotation_matrix *r_matrix, carmen_orientation_3D_t orientation);
rotation_matrix* create_rotation_matrix_inverse(carmen_orientation_3D_t orientation);
rotation_matrix* create_rotation_matrix_from_matrix(double matrix[3][3]);
rotation_matrix* create_rotation_matrix_from_matrix_inverse(double matrix[3][3]);
rotation_matrix* create_rotation_matrix_from_vector(double* matrix);
rotation_matrix* create_rotation_matrix_from_vector_inverse(double* matrix);
rotation_matrix* create_rotation_matrix_from_quaternions(carmen_quaternion_t quat);
rotation_matrix* transpose_matrix(rotation_matrix* matrix_original);

void destroy_rotation_matrix(rotation_matrix* r_matrix);

carmen_vector_3D_t multiply_matrix_vector(rotation_matrix* r_matrix, carmen_vector_3D_t vector);
rotation_matrix* multiply_matrix_matrix(rotation_matrix* matrix_1, rotation_matrix* matrix_2);

// The roll, pitch and yaw angles returned by this function are in the convetion
// that first rotates by yaw, then by pitch and then by roll, with the frame of reference rotating with the object
carmen_orientation_3D_t get_angles_from_rotation_matrix(rotation_matrix* r_matrix);

carmen_vector_3D_t add_vectors(carmen_vector_3D_t v1, carmen_vector_3D_t v2);
carmen_vector_3D_t sub_vectors(carmen_vector_3D_t v1, carmen_vector_3D_t v2);
carmen_vector_3D_t carmen_change_sensor_reference(carmen_vector_3D_t position, carmen_vector_3D_t reference, rotation_matrix* transformation_matrix);

carmen_vector_3D_t carmen_get_interpolated_robot_position_at_time(carmen_pose_3D_t robot_pose, carmen_vector_3D_t robot_velocity, double robot_time, double interpolated_time, rotation_matrix* r_matrix_robot_to_global)
__attribute__ ((deprecated("Alberto say -> use carmen_libcarmodel_recalc_pos_ackerman() instead")));

carmen_pose_3D_t carmen_ackerman_interpolated_robot_position_at_time(carmen_pose_3D_t robot_pose, double dt, double v, double phi, double distance_between_front_and_rear_axles)
__attribute__ ((deprecated("Alberto say -> use carmen_libcarmodel_recalc_pos_ackerman() instead")));

carmen_vector_3D_t carmen_get_sensor_sphere_point_in_robot_cartesian_reference(carmen_sphere_coord_t sphere_point, carmen_pose_3D_t sensor_pose, carmen_pose_3D_t sensor_board_pose, rotation_matrix* sensor_to_board_matrix, rotation_matrix* board_to_car_matrix);

carmen_pose_3D_t get_world_pose(carmen_pose_3D_t* local_pose);

#ifdef __cplusplus
}
#endif

#endif
