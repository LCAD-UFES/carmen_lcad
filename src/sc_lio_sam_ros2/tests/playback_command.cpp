#include <carmen/carmen.h>
#include <carmen/playback_interface.h>
int main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    if (argc < 2) return 1;
    if (!strcmp(argv[1], "stop"))
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_STOP, NULL, 0, 1.0);
    else
    {
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_MESSAGE, argv[1], 0, 1.0);
        carmen_ipc_sleep(0.2);
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_PLAY, NULL, 0, 1.0);
    }
    carmen_ipc_sleep(0.2);
    carmen_ipc_disconnect();
}
