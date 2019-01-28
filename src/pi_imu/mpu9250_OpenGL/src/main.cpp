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
#include <carmen/xsens_interface.h>
#include <carmen/xsenscore.h>

carmen_xsens_global_quat_message *xsens_quat_message;
xsens_global* data;

void
xsens_message_handler(carmen_xsens_global_quat_message *xsens_quat_message)
{
	data->acc.x = xsens_quat_message->m_acc.x;
	data->acc.y = xsens_quat_message->m_acc.y;
	data->acc.z = xsens_quat_message->m_acc.z;

	data->gyr.x = xsens_quat_message->m_gyr.x;
	data->gyr.y = xsens_quat_message->m_gyr.y;
	data->gyr.z = xsens_quat_message->m_gyr.z;

	data->mag.x = xsens_quat_message->m_mag.x;
	data->mag.y = xsens_quat_message->m_mag.y;
	data->mag.z = xsens_quat_message->m_mag.z;
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

	data = (xsens_global*) calloc (1, sizeof(xsens_global));

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


