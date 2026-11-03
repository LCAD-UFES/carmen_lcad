#include <carmen/carmen.h>
#include <carmen/velodyne_interface.h>
#include <vector>
#include <type_traits>
#include <unistd.h>
int main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_velodyne_variable_scan_message msg{};
    using Shot = std::remove_pointer_t<decltype(msg.partial_scan)>;
    using Distance = std::remove_pointer_t<decltype(Shot{}.distance)>;
    using Intensity = std::remove_pointer_t<decltype(Shot{}.intensity)>;
    std::vector<Shot> shots(100);
    std::vector<Distance> distance(128);
    std::vector<Intensity> intensity(128);
    for (int i=0;i<128;++i) { distance[i]=(i%4==0?20:i%4==1?250:2500);intensity[i]=i; }
    for (int i=0;i<100;++i)
    {
        shots[i].angle=i*3.6;shots[i].shot_size=128;
        shots[i].distance=distance.data();shots[i].intensity=intensity.data();
    }
    msg.number_of_shots=100;msg.partial_scan=shots.data();msg.host=carmen_get_host();
    for (int i=0;i<20;++i)
    {
        msg.timestamp=1000+i*.1;
        carmen_velodyne_publish_variable_scan_message(&msg,0);
        usleep(150000);
    }
    IPC_disconnect();
}
