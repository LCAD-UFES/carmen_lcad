#include <carmen/carmen.h>
#include "xsens_messages.h"
#include "xsens_interface.h"

/* global variables */
carmen_xsens_global_matrix_message xsens_matrix_global;
carmen_xsens_global_euler_message xsens_euler_global;
carmen_xsens_global_quat_message xsens_quat_global;


void
xsens_euler_message_handler()
{
//    fprintf(stderr, "Message #%d:\n", xsens_euler_global.m_count);
    printf("\tAcceleration\nX: %f \t Y: %f \t Z: %f\n", xsens_euler_global.m_acc.x, xsens_euler_global.m_acc.y, xsens_euler_global.m_acc.z);
    printf("\tGyro\nX: %f \t Y: %f \t Z: %f\n", xsens_euler_global.m_gyr.x, xsens_euler_global.m_gyr.y, xsens_euler_global.m_gyr.z);
    printf("\tMagnetism\nX: %f \t Y: %f \t Z: %f\n", xsens_euler_global.m_mag.x, xsens_euler_global.m_mag.y, xsens_euler_global.m_mag.z);
    printf("Temperature: %f \nTimestamp: %f \nHost: %s\n", xsens_euler_global.m_temp, xsens_euler_global.timestamp, xsens_euler_global.host);

    printf(" == Euler Data == \n");
    printf("Pitch: %f \t Roll: %f \t Yaw: %f \n", xsens_euler_global.euler_data.m_pitch, xsens_euler_global.euler_data.m_roll, xsens_euler_global.euler_data.m_yaw);
    
    printf("\n");

    fflush(stdout);
}


void
xsens_quat_message_handler()
{
//    fprintf(stderr, "Message #%d:\n", xsens_quat_global.m_count);
    printf("\tAcceleration\nX: %f \t Y: %f \t Z: %f\n", xsens_quat_global.m_acc.x, xsens_quat_global.m_acc.y, xsens_quat_global.m_acc.z);
    printf("\tGyro\nX: %f \t Y: %f \t Z: %f\n", xsens_quat_global.m_gyr.x, xsens_quat_global.m_gyr.y, xsens_quat_global.m_gyr.z);
    printf("\tMagnetism\nX: %f \t Y: %f \t Z: %f\n", xsens_quat_global.m_mag.x, xsens_quat_global.m_mag.y, xsens_quat_global.m_mag.z);
    printf("Temperature: %f \nTimestamp: %f \nHost: %s\n", xsens_quat_global.m_temp, xsens_quat_global.timestamp, xsens_quat_global.host);

    printf(" == Quaterion Data == \n");
    printf("Q0: %f \t Q1: %f \t Q2: %f \t Q3: %f \n", xsens_quat_global.quat_data.m_data[0], xsens_quat_global.quat_data.m_data[1],                                                         xsens_quat_global.quat_data.m_data[2], xsens_quat_global.quat_data.m_data[3]);

    fflush(stdout);
}


void
xsens_matrix_message_handler()
{
//    fprintf(stderr, "Message #%d:\n", xsens_matrix_global.m_count);
    printf("\tAcceleration\nX: %f \t Y: %f \t Z: %f\n", xsens_matrix_global.m_acc.x, xsens_matrix_global.m_acc.y, xsens_matrix_global.m_acc.z);
    printf("\tGyro\nX: %f \t Y: %f \t Z: %f\n", xsens_matrix_global.m_gyr.x, xsens_matrix_global.m_gyr.y, xsens_matrix_global.m_gyr.z);
    printf("\tMagnetism\nX: %f \t Y: %f \t Z: %f\n", xsens_matrix_global.m_mag.x, xsens_matrix_global.m_mag.y, xsens_matrix_global.m_mag.z);
    printf("Temperature: %f \nTimestamp: %f \nHost: %s\n", xsens_matrix_global.m_temp, xsens_matrix_global.timestamp, xsens_matrix_global.host);
    
    printf(" == Matrix Data == \n");
    printf("[%f]\t[%f]\t[%f]\n", xsens_matrix_global.matrix_data.m_data[0][0], xsens_matrix_global.matrix_data.m_data[0][1],
                                    xsens_matrix_global.matrix_data.m_data[0][2]);
    printf("[%f]\t[%f]\t[%f]\n", xsens_matrix_global.matrix_data.m_data[1][0], xsens_matrix_global.matrix_data.m_data[1][1],
                                    xsens_matrix_global.matrix_data.m_data[1][2]);
    printf("[%f]\t[%f]\t[%f]\n", xsens_matrix_global.matrix_data.m_data[2][0], xsens_matrix_global.matrix_data.m_data[2][1],
                                    xsens_matrix_global.matrix_data.m_data[2][2]);

    fflush(stdout);
}


void
shutdown_catcher(int x)
{
    if (x == SIGINT)
    {
        carmen_verbose("Disconnecting from IPC network.\n");
        exit(1);
    }
}


static void
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}


int
main(int argc, char **argv)
{ 
    /* initialize carmen */
    carmen_randomize(&argc, &argv);
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);
    register_ipc_messages();

    /* Setup exit handler */
    signal(SIGINT, shutdown_catcher);

    /* Subscribe to xsens messages */
    /*carmen_xsens_subscribe_xsens_global_message(&xsens_global, 
						     (carmen_handler_t)xsens_message_handler, 
						     CARMEN_SUBSCRIBE_LATEST);*/

    carmen_xsens_subscribe_xsens_global_matrix_message(&xsens_matrix_global,
					    (carmen_handler_t) xsens_matrix_message_handler,
					    CARMEN_SUBSCRIBE_LATEST);
					    
    carmen_xsens_subscribe_xsens_global_euler_message(&xsens_euler_global,
					    (carmen_handler_t) xsens_euler_message_handler,
					    CARMEN_SUBSCRIBE_LATEST);
					    
    carmen_xsens_subscribe_xsens_global_quat_message(&xsens_quat_global,
					    (carmen_handler_t) xsens_quat_message_handler,
					    CARMEN_SUBSCRIBE_LATEST);

    /* Loop forever */
    carmen_ipc_dispatch();

    return (0);
}
