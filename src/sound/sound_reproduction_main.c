#include <carmen/carmen.h>
#include <carmen/sound_interface.h>
#include "sound.h"

snd_pcm_t *handle = NULL;

void
carmen_microphone_message_handler(carmen_microphone_message *message)
{
	int frames = message->buffer_size / 4;
	carmen_sound_reproduce_sound(handle, message->buffer, frames);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (handle != NULL)
			carmen_sound_stop_device(handle);

		carmen_ipc_disconnect();
		printf("microphone test module: disconnected.\n");
		exit(0);
	}
}


int main(int argc, char *argv[])
{
	int sampling_rate = 44100, num_frames = 32;

	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_sound_open_device(&handle, CARMEN_SOUND_OPEN_FOR_REPRODUCE);
	carmen_sound_set_device_parameters(handle, sampling_rate, num_frames);

	carmen_microphone_subscribe_message(NULL, (carmen_handler_t) carmen_microphone_message_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_ipc_dispatch();

	return 0;
}
