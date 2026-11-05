#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>

#include <iostream>

// Input lines select a unique sample, receiver and quality for the ROS2 test.
int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    IPC_RETURN_TYPE result = IPC_defineMsg(
        CARMEN_GPS_XYZ_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_GPS_XYZ_MESSAGE_FMT);
    carmen_test_ipc_exit(result, "define", CARMEN_GPS_XYZ_MESSAGE_NAME);
    std::cout << "READY" << std::endl;

    int sequence, receiver, quality;
    while (std::cin >> sequence >> receiver >> quality)
    {
        carmen_gps_xyz_message message{};
        message.host = carmen_get_host();
        message.nr = receiver;
        message.gps_quality = quality;
        message.timestamp = 1793710000.0 + sequence * 0.125;
        message.x = sequence;
        message.y = -sequence;
        message.z = 12.345;
        result = IPC_publishData(CARMEN_GPS_XYZ_MESSAGE_NAME, &message);
        carmen_test_ipc_exit(result, "publish", CARMEN_GPS_XYZ_MESSAGE_NAME);
    }
    IPC_disconnect();
    return 0;
}
