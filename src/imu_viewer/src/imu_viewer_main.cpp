/* OpenGL Frame
   Designed by Philippe Lucidarme
   LISA - University of Angers
   Supported by the Cart-O-matic project (ANR CAROTTE)

   Contact : philippe.lucidarme@univ-angers.fr
   Website : http://5lair.free.fr

   sudo apt-get install freeglut3 freeglut3-dev

*/

//#include <QtGui/QApplication>
#include <QtWidgets/QApplication>
#include "mainwindow.h"
#include <carmen/carmen.h>
#include <carmen/xsens_mtig_interface.h>
#include <carmen/xsens_interface.h>
#include <carmen/xsens_messages.h>
#include <carmen/rotation_geometry.h>
#include <carmen/gps_xyz_interface.h>
carmen_xsens_global_quat_message *xsens_quat_message;
carmen_xsens_global_message* data;

rotation_matrix *
create_rotation_matrix_from_quaternions_new(carmen_xsens_quat quat)
{
	double q0 = quat.m_data[0];
	double q1 = quat.m_data[1];
	double q2 = quat.m_data[2];
	double q3 = quat.m_data[3];

	rotation_matrix *r_matrix =(rotation_matrix *) malloc(sizeof(rotation_matrix));

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

void
xsens_message_handler(carmen_xsens_global_quat_message *xsens_quat_message)
{
	data->m_acc.x = xsens_quat_message->m_acc.x;
	data->m_acc.y = xsens_quat_message->m_acc.y;
	data->m_acc.z = xsens_quat_message->m_acc.z;

	data->m_gyr.x = xsens_quat_message->m_gyr.x;
	data->m_gyr.y = xsens_quat_message->m_gyr.y;
	data->m_gyr.z = xsens_quat_message->m_gyr.z;

	rotation_matrix* r_mat = create_rotation_matrix_from_quaternions_new(xsens_quat_message->quat_data);
	carmen_orientation_3D_t euler_angles = get_angles_from_rotation_matrix(r_mat);
	destroy_rotation_matrix(r_mat);

	data->m_roll = euler_angles.roll;
	data->m_pitch = euler_angles.pitch;
	data->m_yaw = euler_angles.yaw;
/*
	data->m_.x = xsens_quat_message->m_mag.x;
	data->mag.y = xsens_quat_message->m_mag.y;
	data->mag.z = xsens_quat_message->m_mag.z;*/
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();

        printf("Imu Interface: disconnected.\n");
        exit(0);
    }
}


int 
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

	data = (carmen_xsens_global_message*) calloc (1, sizeof(carmen_xsens_global_message));

	carmen_xsens_subscribe_xsens_global_quat_message(NULL,
			(carmen_handler_t) xsens_message_handler, CARMEN_SUBSCRIBE_LATEST);

    QApplication a(argc, argv);
    MainWindow w(0,800,600, data);
	// Connect to the Arduino

    if (!w.connect()) 
		return 0;

    w.show();
    return a.exec();
}


