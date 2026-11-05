#include <carmen/carmen.h>
#include <carmen/base_ackerman_messages.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    IPC_RETURN_TYPE result = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME,
        IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
    carmen_test_ipc_exit(result, "define odometry", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
    for (int i = 0; i < 20; ++i)
    {
        carmen_base_ackerman_odometry_message msg{};
        msg.v = -2.5 + i * 0.25;
        msg.phi = -0.3 + i * 0.01;
        msg.timestamp = 1793710000.125 + i * 0.125;
        msg.host = carmen_get_host();
        result = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &msg);
        carmen_test_ipc_exit(result, "publish odometry", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
        usleep(100000);
    }
    IPC_disconnect();
}
