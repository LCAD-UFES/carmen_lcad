/*
 * sound.h
 *
 *  Created on: Oct 3, 2012
 *      Author: _filipe
 */

#ifndef SOUND_H_
#define SOUND_H_

#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>

typedef enum
{
	CARMEN_SOUND_OPEN_FOR_CAPTURE = 0,
	CARMEN_SOUND_OPEN_FOR_REPRODUCE
}CARMEN_SOUND_OPEN_FORMAT;

#ifdef __cplusplus
extern "C" {
#endif

	void carmen_sound_open_device(snd_pcm_t **handle, CARMEN_SOUND_OPEN_FORMAT format);
	void carmen_sound_stop_device(snd_pcm_t *handle);
	void carmen_sound_set_device_parameters(snd_pcm_t *handle, int sampling_rate, int num_frames);
	void carmen_sound_check_read(snd_pcm_t *handle, int rc, int num_frames);
	void carmen_sound_check_write(snd_pcm_t *handle, int rc);
	void compute_fft_magnitudes (double *real, double *output, int size);
	void carmen_apply_fourrier_transform(double *input, int input_size, double *output_real, double *output_imaginary);
	void carmen_apply_inverse_fourrier_transform(double *input_real, double *input_imaginary, int input_size, double *output);
	void carmen_sound_get_magnitudes_from_sound_buffer(char *buffer, double *magnitudes, int buffer_size);
	void carmen_sound_reproduce_sound(snd_pcm_t *handle, char *buffer, int num_frames);
	void carmen_sound_capture_sound(snd_pcm_t *handle, char *buffer, int num_frames);

#ifdef __cplusplus
}
#endif

#endif /* SOUND_H_ */
