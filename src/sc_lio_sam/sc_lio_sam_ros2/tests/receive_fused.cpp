#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <cstdio>
static void handler(carmen_fused_odometry_message *m)
{
    printf("FUSED %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n",m->timestamp,m->pose.position.x,
           m->pose.position.y,m->pose.position.z,m->velocity.x,m->phi,m->angular_velocity.yaw);
    fflush(stdout);
}
int main(int argc,char **argv)
{
    carmen_ipc_initialize(argc,argv);
    carmen_fused_odometry_subscribe_fused_odometry_message(nullptr,
        reinterpret_cast<carmen_handler_t>(handler),CARMEN_SUBSCRIBE_ALL);
    carmen_ipc_dispatch();
}
