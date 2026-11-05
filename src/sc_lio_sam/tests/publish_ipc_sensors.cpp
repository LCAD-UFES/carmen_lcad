#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/imu_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/gps_nmea_interface.h>
#include <unistd.h>
#include <cstring>
#include <set>
#include <string>

static void send_message(const char *name, const char *format, void *message)
{
    static std::set<std::string> defined;
    if (defined.insert(name).second)
    {
        IPC_RETURN_TYPE result = IPC_defineMsg(name, IPC_VARIABLE_LENGTH, format);
        carmen_test_ipc_exit(result, "define", name);
    }
    IPC_RETURN_TYPE result = IPC_publishData(name, message);
    carmen_test_ipc_exit(result, "publish", name);
}

int main(int argc, char **argv)
{
    bool warmup = argc > 1 && std::strcmp(argv[1], "warmup") == 0;
    carmen_ipc_initialize(argc, argv);
    const int receivers[] = {1, 2, 7};
    const int qualities[] = {0, 1, 2, 4, 5, 9};
    for (int i = 0; i < 24; ++i)
    {
        double timestamp = (warmup ? 1000.0 : 1793710000.1234567) + i * .125;
        carmen_imu_message lidar{};
        lidar.host = carmen_get_host(); lidar.timestamp = timestamp;
        lidar.accX=1.25; lidar.accY=-2.5; lidar.accZ=9.8;
        lidar.gyroX=.1; lidar.gyroY=-.2; lidar.gyroZ=.3;
        lidar.q0=.5; lidar.q1=.5; lidar.q2=-.5; lidar.q3=.5;
        carmen_imu_publish_imu_lidar_message(&lidar, 5);
        carmen_xsens_global_quat_message xsens{};
        xsens.host=carmen_get_host(); xsens.timestamp=timestamp;
        xsens.m_acc={-1.0, 2.0, 9.5}; xsens.m_gyr={.01, -.02, .03};
        if (i==10) {xsens.m_acc={0,0,0};xsens.m_gyr={0,0,0};}
        xsens.quat_data.m_data[0]=1.0;
        send_message(CARMEN_XSENS_GLOBAL_QUAT_NAME,CARMEN_XSENS_GLOBAL_QUAT_FMT,&xsens);
        carmen_gps_xyz_message gps{};
        gps.host=carmen_get_host();gps.timestamp=timestamp;gps.nr=receivers[i%3];
        gps.gps_quality=warmup ? 4 : qualities[i%6];
        gps.x=7750000.123+i;gps.y=-350000.456-i;gps.z=12.345;
        send_message(CARMEN_GPS_XYZ_MESSAGE_NAME,CARMEN_GPS_XYZ_MESSAGE_FMT,&gps);
        carmen_gps_gphdt_message heading{};
        heading.host=carmen_get_host();heading.timestamp=timestamp;
        heading.valid=warmup ? 1 : i%2;heading.heading=-3.0+i*.25;
        send_message(CARMEN_GPS_GPHDT_MESSAGE_NAME,CARMEN_GPS_GPHDT_MESSAGE_FMT,&heading);
        usleep(100000);
    }
    IPC_disconnect();
}
