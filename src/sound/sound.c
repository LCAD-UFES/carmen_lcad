/*
 * sound.c
 *
 *  Created on: Oct 3, 2012
 *      Author: _filipe
 */

#include "sound.h"
#include <alsa/asoundlib.h>
#include <fftw3.h>
#include <math.h>

void
carmen_sound_open_device(snd_pcm_t **handle, CARMEN_SOUND_OPEN_FORMAT format)
{
	int rc;

	if (format == CARMEN_SOUND_OPEN_FOR_CAPTURE)
		rc = snd_pcm_open(handle, "default", SND_PCM_STREAM_CAPTURE, 0);
	else
		rc = snd_pcm_open(handle, "default", SND_PCM_STREAM_PLAYBACK, 0);

	if (rc < 0)
		exit(printf("unable to open pcm device: %s\n", snd_strerror(rc)));
}


void
carmen_sound_stop_device(snd_pcm_t *handle)
{
	snd_pcm_drain(handle);
	snd_pcm_close(handle);
}


void
carmen_sound_set_device_parameters(snd_pcm_t *handle, int sampling_rate, int num_frames)
{
	int rc, dir;
	unsigned int val;
	snd_pcm_hw_params_t *params;
	snd_pcm_uframes_t frames;

	val = sampling_rate;
	frames = (snd_pcm_uframes_t) num_frames;

	/* Allocate a hardware parameters object. */
	snd_pcm_hw_params_alloca(&params);

	/* Fill it in with default values. */
	snd_pcm_hw_params_any(handle, params);

	/* Interleaved mode */
	snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

	/* Signed 16-bit little-endian format */
	snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);

	/* Two channels (stereo) */
	snd_pcm_hw_params_set_channels(handle, params, 2);

	/* set bits/second sampling rate */
	snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);

	/* Set period size to num of frames. */
	snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

	/* Write the parameters to the driver */
	rc = snd_pcm_hw_params(handle, params);

	if (rc < 0)
		exit(printf("unable to set hw parameters: %s\n", snd_strerror(rc)));

	/* Use a buffer large enough to hold one period */
	snd_pcm_hw_params_get_period_size(params, &frames, &dir);

	/* We want to loop for 5 seconds */
	snd_pcm_hw_params_get_period_time(params, &val, &dir);

	// @TODO: pesquisar por que tem que ler um pouco que vem vazio nao sei por que...
	char buffer[4 * 32];
	rc = snd_pcm_readi(handle, buffer, 32);
}


void
carmen_sound_check_read(snd_pcm_t *handle, int rc, int num_frames)
{
	if (rc == -EPIPE)
	{
		printf("overrun occurred\n");
		snd_pcm_prepare(handle);
	}
	else if (rc < 0)
		printf("error from read: %s\n", snd_strerror(rc));
	else if (rc != num_frames)
		printf("short read, read %d frames\n", rc);
}


void
carmen_sound_check_write(snd_pcm_t *handle, int rc)
{
	if (rc == -EPIPE)
	{
		printf("underrun occurred\n");
		snd_pcm_prepare(handle);
	}
	else if (rc < 0)
		printf("error from writei: %s\n", snd_strerror(rc));
}


void
carmen_apply_fourrier_transform(double *input, int input_size, double *output_real, double *output_imaginary)
{
	int i;
	fftw_plan p;
	fftw_complex *output;

	output = (fftw_complex *) fftw_malloc (sizeof(fftw_complex) * (input_size / 2 + 1));

	p = fftw_plan_dft_r2c_1d(input_size, input, output, FFTW_ESTIMATE);

	fftw_execute(p);

	for(i = 0; i < (input_size / 2 + 1); i++)
	{
		output_real[i] = output[i][0];
		output_imaginary[i] = output[i][1];
	}

	fftw_free(output);
	fftw_destroy_plan(p);
}


void
compute_fft_magnitudes (double *real, double *output, int size)
{
	int i;

	for (i = 0; i < size; i++)
		output[i] = sqrt(real[i] * real[i]) / 2.0;
}


void
carmen_apply_inverse_fourrier_transform(double *input_real, double *input_imaginary, int input_size, double *output)
{
	int i;
	fftw_plan p;
	fftw_complex *input;

	input = (fftw_complex *) fftw_malloc (sizeof(fftw_complex) * (input_size));

	for(i = 0; i < input_size; i++)
	{
		input[i][0] = input_real[i];
		input[i][1] = input_imaginary[i];
	}

	p = fftw_plan_dft_c2r_1d(input_size, input, output, FFTW_ESTIMATE);
	fftw_execute(p);

	fftw_destroy_plan(p);
}


void
carmen_sound_reproduce_sound(snd_pcm_t *handle, char *buffer, int num_frames)
{
	int rc = snd_pcm_writei(handle, buffer, num_frames);
	carmen_sound_check_write(handle, rc);
}


void
carmen_sound_copy_raw_data_to_sound_buffer(char *raw_buffer, double *sound_buffer, int buffer_size)
{
	int i, j, sound_val;

	for (i = j = 0; i < (buffer_size - 2); i += 4, j++)
	{
		sound_val = raw_buffer[i] + (raw_buffer[i + 1] << 8);
		sound_buffer[j] = (double) sound_val / 32000.0;
	}
}


void
carmen_sound_get_magnitudes_from_sound_buffer(char *buffer, double *magnitudes, int buffer_size)
{
	double output_real[(buffer_size / 2 + 1)], output_imaginary[(buffer_size / 2 + 1)];
	double sound[buffer_size];

	carmen_sound_copy_raw_data_to_sound_buffer(buffer, sound, buffer_size);
	carmen_apply_fourrier_transform(sound, buffer_size / 4, output_real, output_imaginary);
	compute_fft_magnitudes(output_real, magnitudes, ((buffer_size / 4) / 2 + 1));
}


void
carmen_sound_capture_sound(snd_pcm_t *handle, char *buffer, int num_frames)
{
	int rc = snd_pcm_readi(handle, buffer, num_frames);
	carmen_sound_check_read(handle, rc, num_frames);
}

