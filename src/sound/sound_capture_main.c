
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <carmen/carmen.h>
#include <carmen/sound_interface.h>
#include "sound.h"

//#define __DEBUG_PRINT_PUBLISHING_FREQUENCY_

int num_messages = 0;
double last_timestamp = 0;

snd_pcm_t *handle = NULL;
IplImage *sound_image = NULL;

void
publish_microphone_message(char *buffer, int buffer_size)
{
	IPC_RETURN_TYPE err;

	carmen_microphone_message message;

	message.buffer = buffer;
	message.buffer_size = buffer_size;
	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_MICROPHONE_MESSAGE_NAME, &message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MICROPHONE_MESSAGE_FMT);

#ifdef __DEBUG_PRINT_PUBLISHING_FREQUENCY_
	num_messages++;

	if ((message.timestamp - last_timestamp) > 1.0)
	{
		printf("num messages: %d\n", num_messages);

		num_messages = 0;
		last_timestamp = message.timestamp;
	}
#endif

	printf("%lf -> publish %d bytes\n", message.timestamp, message.buffer_size);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (handle != NULL)
			carmen_sound_stop_device(handle);

		carmen_ipc_disconnect();
		printf("microphone module: disconnected.\n");
		exit(0);
	}
}


void shift_input_to_right (IplImage *img, int num_cols)
{
	int i, j, w, h, org_col, dst_col;

	w = img->width;
	h = img->height;

	// perform the shift
	for(j = 0; j < (w - num_cols); j++)
	{
		dst_col = w - j - 1; // width - column - 1
		org_col = dst_col - num_cols;

		for(i = 0; i < h; i++)
			img->imageData[dst_col + i * w] = img->imageData[org_col + i * w];
	}

	// clean the first num_cols columns (set to zero)
	for(i = 0; i < h; i++)
		for(j = 0; j < num_cols; j++)
			img->imageData[i * w + j] = 0;
}

void
update_sound_image(double *magnitudes)
{
	int i, max_is_init, min_is_init, weighted_sound_value, pixel_pos;
	double max_freq, min_freq;

	max_freq = min_freq = 0;
	max_is_init = min_is_init = 0;

	shift_input_to_right(sound_image, 1);

	for(i = 0; i < sound_image->height; i++)
	{
		if (magnitudes[i] > max_freq || !max_is_init)
		{
			max_freq = magnitudes[i];
			max_is_init = 1;
		}

		if (magnitudes[i] < min_freq || !min_is_init)
		{
			min_freq = magnitudes[i];
			min_is_init = 1;
		}
	}

	for(i = 0; i < sound_image->height; i++)
	{
		weighted_sound_value = (int) 255 * (magnitudes[i] - min_freq) / (max_freq - min_freq);

		pixel_pos = sound_image->height - i; // a imagem da opencv tem zero em cima e estamos mais acostumados com zero em baixo
		sound_image->imageData[pixel_pos * sound_image->width] = weighted_sound_value;
	}
}


void
initialize_sound_image(int num_frames)
{
	int i, j;
	int fourrier_output_size = num_frames / 2.0;

	sound_image = cvCreateImage(cvSize(fourrier_output_size * 2, fourrier_output_size), IPL_DEPTH_8U, 1);

	for(i = 0; i < sound_image->height; i++)
		for(j = 0; j < sound_image->width; j++)
			sound_image->imageData[i * sound_image->width + j] = 0;
}


int
main(int argc, char *argv[])
{
	int sampling_rate = 3200 * 2, num_frames = 4 * 64;
	int buffer_size = num_frames * 4;

	char *buffer = (char *) calloc (buffer_size, sizeof(char));
	double *magnitudes = (double *) calloc ((buffer_size / 2 + 1), sizeof(double));

	carmen_test_alloc(buffer);
	carmen_test_alloc(magnitudes);

	initialize_sound_image(num_frames);

	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_microphone_define_messages();

	carmen_sound_open_device(&handle, CARMEN_SOUND_OPEN_FOR_CAPTURE);
	carmen_sound_set_device_parameters(handle, sampling_rate, num_frames);

	cvNamedWindow("sound_image_viewer", CV_WINDOW_AUTOSIZE);

	while (1)
	{
		carmen_sound_capture_sound(handle, buffer, num_frames);
		carmen_sound_get_magnitudes_from_sound_buffer(buffer, magnitudes, buffer_size);
		update_sound_image(magnitudes);

		cvShowImage("sound_image_viewer", sound_image);
    if ((cvWaitKey(10) & 255) == 27) break;

		//publish_microphone_message(buffer, buffer_size);
	}

	return 0;
}
