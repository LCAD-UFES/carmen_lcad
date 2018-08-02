/*
 * pi_imu_interface.cpp
 *
 *  Created on: Aug 1, 2018
 *      Author: Marcelo
 */
#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>
#include <carmen/xsenscore.h>

carmen_xsens_global_quat_message *xsens_quat_message;
xsens_global data;

void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();

        printf("Imu Interface: disconnected.\n");
        exit(0);
    }
}


void
xsens_message_handler(carmen_xsens_global_quat_message *xsens_quat_message)
{
	data.acc.x = xsens_quat_message->m_acc.x;
	data.acc.y = xsens_quat_message->m_acc.y;
	data.acc.z = xsens_quat_message->m_acc.z;

	data.gyr.x = xsens_quat_message->m_gyr.x;
	data.gyr.y = xsens_quat_message->m_gyr.y;
	data.gyr.z = xsens_quat_message->m_gyr.z;

	data.mag.x = xsens_quat_message->m_mag.x;
	data.mag.y = xsens_quat_message->m_mag.y;
	data.mag.z = xsens_quat_message->m_mag.z;

	printf("ACELEROMETRO = X:%f m/s^2 Y:%f m/s^2 Z:%f m/s^2\n", data.acc.x, data.acc.y, data.acc.z);
	printf("GIROSCÓPIO = X:%f radps Y:%f radps Z:%f radps\n", data.gyr.x, data.gyr.y, data.gyr.z);
	printf("MAGNETOMETRO = X:%f mgauss Y:%f mgauss Z:%f mgauss\n", data.mag.x, data.mag.y, data.mag.z);

}


void
carmen_xsens_subscribe_xsens_global_quat_message(carmen_xsens_global_quat_message
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message((char *) CARMEN_XSENS_GLOBAL_QUAT_NAME,
    						 (char *) CARMEN_XSENS_GLOBAL_QUAT_FMT,
                             xsens_global, sizeof(carmen_xsens_global_quat_message),
            			     handler, subscribe_how);

}

void
carmen_xsens_unsubscribe_xsens_global_quat_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message((char *) CARMEN_XSENS_GLOBAL_QUAT_NAME, handler);
}


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	carmen_xsens_subscribe_xsens_global_quat_message(xsens_quat_message, (carmen_handler_t) xsens_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return (0);
}
