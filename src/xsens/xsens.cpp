#include <carmen/carmen.h>
#include "xsenscore.h"
#include "xsens_messages.h"

//Xsens Stuff
#include <stdlib.h>
#include <cmtdef.h>
#include <xsens_time.h>
#include <xsens_list.h>
#include <cmtscan.h>
#include <cmt3.h>

/* global variables */
xsens_global g_data;
xsens::Cmt3 cmt3;
xsens::Packet* packet;
CmtOutputMode mode;
CmtOutputSettings settings;
char *dev;

/* publish a global position message */

void publish_xsens_global(xsens_global data, /*CmtOutputMode& mode,*/ CmtOutputSettings& settings)
{

    IPC_RETURN_TYPE err;

    switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {

		case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:

            carmen_xsens_global_quat_message xsens_quat_message;

            //Acceleration
            xsens_quat_message.m_acc.x = data.acc.x;
            xsens_quat_message.m_acc.y = data.acc.y;
            xsens_quat_message.m_acc.z = data.acc.z;

            //Gyro
            xsens_quat_message.m_gyr.x = data.gyr.x;
            xsens_quat_message.m_gyr.y = data.gyr.y;
            xsens_quat_message.m_gyr.z = data.gyr.z;

            //Magnetism
            xsens_quat_message.m_mag.x = data.mag.x;
            xsens_quat_message.m_mag.y = data.mag.y;
            xsens_quat_message.m_mag.z = data.mag.z;

            xsens_quat_message.quat_data.m_data[0] = data.quatData.data[0];
            xsens_quat_message.quat_data.m_data[1] = data.quatData.data[1];
            xsens_quat_message.quat_data.m_data[2] = data.quatData.data[2];
            xsens_quat_message.quat_data.m_data[3] = data.quatData.data[3];

            //Temperature and Message Count
            xsens_quat_message.m_temp = data.temp;
            xsens_quat_message.m_count = data.count;

            //Timestamp
            xsens_quat_message.timestamp = carmen_get_time();

            //Host
            xsens_quat_message.host = carmen_get_host();

            err = IPC_publishData(CARMEN_XSENS_GLOBAL_QUAT_NAME, &xsens_quat_message);
            carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_QUAT_NAME);

			break;

		case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:

            carmen_xsens_global_euler_message xsens_euler_message;

            //Acceleration
            xsens_euler_message.m_acc.x = data.acc.x;
            xsens_euler_message.m_acc.y = data.acc.y;
            xsens_euler_message.m_acc.z = data.acc.z;

            //Gyro
            xsens_euler_message.m_gyr.x = data.gyr.x;
            xsens_euler_message.m_gyr.y = data.gyr.y;
            xsens_euler_message.m_gyr.z = data.gyr.z;

            //Magnetism
            xsens_euler_message.m_mag.x = data.mag.x;
            xsens_euler_message.m_mag.y = data.mag.y;
            xsens_euler_message.m_mag.z = data.mag.z;

            xsens_euler_message.euler_data.m_pitch = data.eulerData.pitch;
            xsens_euler_message.euler_data.m_roll = data.eulerData.roll;
            xsens_euler_message.euler_data.m_yaw = data.eulerData.yaw;


            //Temperature and Message Count
            xsens_euler_message.m_temp = data.temp;
            xsens_euler_message.m_count = data.count;

            //Timestamp
            xsens_euler_message.timestamp = carmen_get_time();

            //Host
            xsens_euler_message.host = carmen_get_host();

            err = IPC_publishData(CARMEN_XSENS_GLOBAL_EULER_NAME, &xsens_euler_message);
            carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_EULER_NAME);

            break;

		case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:

			carmen_xsens_global_matrix_message xsens_matrix_message;

			//Acceleration
            xsens_matrix_message.m_acc.x = data.acc.x;
            xsens_matrix_message.m_acc.y = data.acc.y;
            xsens_matrix_message.m_acc.z = data.acc.z;

            //Gyro
            xsens_matrix_message.m_gyr.x = data.gyr.x;
            xsens_matrix_message.m_gyr.y = data.gyr.y;
            xsens_matrix_message.m_gyr.z = data.gyr.z;

            //Magnetism
            xsens_matrix_message.m_mag.x = data.mag.x;
            xsens_matrix_message.m_mag.y = data.mag.y;
            xsens_matrix_message.m_mag.z = data.mag.z;


            xsens_matrix_message.matrix_data.m_data[0][0] = data.matrixData.data[0][0];
            xsens_matrix_message.matrix_data.m_data[0][1] = data.matrixData.data[0][1];
            xsens_matrix_message.matrix_data.m_data[0][2] = data.matrixData.data[0][2];

            xsens_matrix_message.matrix_data.m_data[1][0] = data.matrixData.data[1][0];
            xsens_matrix_message.matrix_data.m_data[1][1] = data.matrixData.data[1][1];
            xsens_matrix_message.matrix_data.m_data[1][2] = data.matrixData.data[1][2];

            xsens_matrix_message.matrix_data.m_data[2][0] = data.matrixData.data[2][0];
            xsens_matrix_message.matrix_data.m_data[2][1] = data.matrixData.data[2][1];
            xsens_matrix_message.matrix_data.m_data[2][2] = data.matrixData.data[2][2];

            //Temperature and Message Count
            xsens_matrix_message.m_temp = data.temp;
            xsens_matrix_message.m_count = data.count;

            //Timestamp
            xsens_matrix_message.timestamp = carmen_get_time();
	    xsens_matrix_message.timestamp2 = data.timestamp;

            //Host
            xsens_matrix_message.host = carmen_get_host();

            err = IPC_publishData(CARMEN_XSENS_GLOBAL_MATRIX_NAME, &xsens_matrix_message);
            carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_MATRIX_NAME);

			break;
      
		default:
            carmen_die("Unknown settings in publish_xsens_global()\n");
      break;
	}
}


/*void robot_frontlaser_handler(carmen_robot_laser_message *flaser)
{
    //fprintf(stderr, "#X#");
    getDataFromXsens(cmt3, packet, mode, settings, g_data);
    publish_xsens_global(g_data);
}*/


/* declare all IPC messages */
//int register_ipc_messages(void)
//{
//    IPC_RETURN_TYPE err;
//
//    /* register xsens's global message */
//    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH,
//	          CARMEN_XSENS_GLOBAL_QUAT_FMT);
//    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
//
//
//    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_EULER_NAME, IPC_VARIABLE_LENGTH,
//	          CARMEN_XSENS_GLOBAL_EULER_FMT);
//    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_EULER_NAME);
//
//
//    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_MATRIX_NAME, IPC_VARIABLE_LENGTH,
//	          CARMEN_XSENS_GLOBAL_MATRIX_FMT);
//    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_MATRIX_NAME);
//
//    return 0;
//
//}


void shutdown_xsens(int x)
{
    if(x == SIGINT) {
        carmen_verbose("Disconnecting from IPC network.\n");
        shutDownXsens(cmt3);
        delete packet;
        exit(1);
    }
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
		{(char*)"xsens_mti", (char*)"mode", CARMEN_PARAM_INT, &mode, 0, NULL},
		{(char*)"xsens_mti", (char*)"settings", CARMEN_PARAM_INT, &settings, 0, NULL},
		{(char*)"xsens_mti", (char*)"dev", CARMEN_PARAM_STRING, &dev, 0, NULL}
		};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


int main(int argc, char **argv)
{
    unsigned long mtCount;

    /* initialize carmen */
    //carmen_randomize(&argc, &argv);
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    mode = 6;
    settings = 2;
    const char* pName = NULL;

    read_parameters(argc, argv);

    pName = dev;

    packet = new xsens::Packet((unsigned short) mtCount, cmt3.isXm());
    /* Initializing Xsens */
    initializeXsens(cmt3, mode, settings, mtCount, (char*)pName);

    /* Setup exit handler */
    signal(SIGINT, shutdown_xsens);

    /* register localize related messages */
    // register_ipc_messages();
    carmen_xsens_define_messages();

    while (1)
    {
		getDataFromXsens(cmt3, packet, mode, settings, g_data);
		publish_xsens_global(g_data, /*mode,*/ settings);
		usleep(10000);
    }

    return (0);
}

