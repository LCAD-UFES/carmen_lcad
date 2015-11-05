#include <carmen/carmen.h>
#include <carmen/xsens_mtig_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/gps_xyz_interface.h>


double xsens_phi = 0.0;


static void xsens_mtig_message_handler(carmen_xsens_mtig_message *xsens_mtig_message) 
{ 	
	printf("Quaternions: % lf % lf % lf % lf\n", xsens_mtig_message->quat.q0, xsens_mtig_message->quat.q1, xsens_mtig_message->quat.q2, xsens_mtig_message->quat.q3);
	rotation_matrix* r_mat = create_rotation_matrix_from_quaternions(xsens_mtig_message->quat);
	carmen_orientation_3D_t euler_angles = get_angles_from_rotation_matrix(r_mat);
	destroy_rotation_matrix(r_mat);

	printf("Euler: yaw:% lf pitch:% lf roll:% lf\n", euler_angles.yaw, euler_angles.pitch, euler_angles.roll);
	printf("Acc: % lf % lf % lf\n", xsens_mtig_message->acc.x, xsens_mtig_message->acc.y, xsens_mtig_message->acc.z);
	printf("Gyr: % lf % lf % lf\n", xsens_mtig_message->gyr.x, xsens_mtig_message->gyr.y, xsens_mtig_message->gyr.z);
	printf("Mag: % lf % lf % lf\n", xsens_mtig_message->mag.x, xsens_mtig_message->mag.y, xsens_mtig_message->mag.z);
	printf("PositionLLA: % lf % lf % lf\n", xsens_mtig_message->latitude, xsens_mtig_message->longitude, xsens_mtig_message->height);
	printf("Velocity: % lf % lf % lf\n", xsens_mtig_message->velocity.x, xsens_mtig_message->velocity.y, xsens_mtig_message->velocity.z);
	printf("GPS_fix: % d \t XKF_valid: % d \n", xsens_mtig_message->gps_fix, xsens_mtig_message->xkf_valid);
	printf("Sensor ID: %x \t Timestamp:% lf\n", xsens_mtig_message->sensor_ID, xsens_mtig_message->timestamp);
	printf("\n\n");

	xsens_phi = euler_angles.yaw;
}

static void xsens_xyz_message_handler(carmen_xsens_xyz_message *xsens_xyz_message) 
{ 	
	printf("Quaternions: % lf % lf % lf % lf\n", xsens_xyz_message->quat.q0, xsens_xyz_message->quat.q1, xsens_xyz_message->quat.q2, xsens_xyz_message->quat.q3);
	rotation_matrix* r_mat = create_rotation_matrix_from_quaternions(xsens_xyz_message->quat);
	carmen_orientation_3D_t euler_angles = get_angles_from_rotation_matrix(r_mat);
	destroy_rotation_matrix(r_mat);
	printf("Euler: yaw:% lf pitch:% lf roll:% lf\n", euler_angles.yaw, euler_angles.pitch, euler_angles.roll);
	printf("Acc: % lf % lf % lf\n", xsens_xyz_message->acc.x, xsens_xyz_message->acc.y, xsens_xyz_message->acc.z);
	printf("Gyr: % lf % lf % lf\n", xsens_xyz_message->gyr.x, xsens_xyz_message->gyr.y, xsens_xyz_message->gyr.z);
	printf("Mag: % lf % lf % lf\n", xsens_xyz_message->mag.x, xsens_xyz_message->mag.y, xsens_xyz_message->mag.z);
	printf("xyz: % lf % lf % lf\n", xsens_xyz_message->position.x, xsens_xyz_message->position.y, xsens_xyz_message->position.z);
	printf("Velocity: % lf % lf % lf\n", xsens_xyz_message->velocity.x, xsens_xyz_message->velocity.y, xsens_xyz_message->velocity.z);
	printf("GPS_fix: % d \t XKF_valid: % d \n", xsens_xyz_message->gps_fix, xsens_xyz_message->xkf_valid);
	printf("Sensor ID: %x \t Timestamp:% lf\n", xsens_xyz_message->sensor_ID, xsens_xyz_message->timestamp);
	printf("\n\n");
}


void 
car_odometry_message_handler(carmen_base_ackerman_odometry_message *car_odometry_message)
{
	static FILE *caco = NULL;
	
	if (caco == NULL)
		caco = fopen("caco.txt", "w");
	printf("%lf %lf\n", xsens_phi, car_odometry_message->phi);
	fprintf(caco, "%lf %lf\n", xsens_phi, car_odometry_message->phi);
	fflush(caco);
}


void initialize(int argc, char** argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	printf("Use parameter 1 for xsens_mtig_message or 0 for xsens_xyz_message\n");
	
	if(argc > 1 && argv[1][0]=='1')
	{
		carmen_xsens_mtig_subscribe_message(	NULL,
		                                    (carmen_handler_t)xsens_mtig_message_handler,
		                                    CARMEN_SUBSCRIBE_ALL);
	}	
	else
	{
		carmen_xsens_xyz_subscribe_message(	NULL,
											 (carmen_handler_t)xsens_xyz_message_handler,
											 CARMEN_SUBSCRIBE_ALL);
	}
/*	carmen_base_ackerman_subscribe_odometry_message(NULL,
							(carmen_handler_t)car_odometry_message_handler,
							CARMEN_SUBSCRIBE_LATEST);
*/
}

int main(int argc, char** argv)
{
	initialize(argc, argv);
            
	carmen_ipc_dispatch();	                                                             
	
	carmen_ipc_disconnect();  

	return 0;
}

