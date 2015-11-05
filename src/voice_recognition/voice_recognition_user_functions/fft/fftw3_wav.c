/*
 * fftw_interface.c
 *
 *  Created on: 29/05/2012
 *      Author: filipe
 */
#include "wav_decoder.h"
#include "fftw3_util.h"
#include <stdlib.h>
#include <math.h>

// input
fftw_complex *fftw_complex_left;
fftw_complex *fftw_complex_right;

// output
fftw_complex *fftw_complex_left_frequency;
fftw_complex *fftw_complex_right_frequency;

unsigned int fftw_num_samples;

void fftw_alloc_stereo_input_arrays (fftw_complex **left, fftw_complex **right, int n)
{
	*left = (fftw_complex *) fftw_malloc (sizeof(fftw_complex) * n);
	*right = (fftw_complex *) fftw_malloc (sizeof(fftw_complex) * n);

	if (*left == NULL)
		exit(printf("error: could not alloc stereo input arrays\n"));
	if (*right == NULL)
			exit(printf("error: could not alloc stereo input arrays\n"));
}

void fftw_alloc_stereo_output_arrays (fftw_complex **output_left, fftw_complex **output_right, int n)
{
	fftw_alloc_stereo_input_arrays(output_left, output_right, n);
}

void fftw_convert_raw_data_to_complex ()
{
	unsigned int i;
	unsigned int num_samples = 131072;
	fftw_num_samples = num_samples;

	fftw_alloc_stereo_input_arrays(&fftw_complex_left, &fftw_complex_right, num_samples);
	fftw_alloc_stereo_output_arrays(&fftw_complex_left_frequency, &fftw_complex_right_frequency, num_samples);

	for(i = 0; i < fftw_num_samples; i++)
	{
		// real part
		fftw_complex_left[i][0] = (double) wav_get_sample(i, 0);
		fftw_complex_right[i][0] = (double) wav_get_sample(i, 1);

		// complex part
		fftw_complex_left[i][1] = 0;
		fftw_complex_right[i][1] = 0;
	}
}

void fftw_apply_fourier_transform ()
{
	fftw_fourier_transform(fftw_complex_left, fftw_complex_left_frequency, fftw_num_samples);
	fftw_fourier_transform(fftw_complex_right, fftw_complex_right_frequency, fftw_num_samples);
}

void fftw_write_frequency_to_file (fftw_complex *frequency, unsigned int n, char *output_filename)
{
	unsigned int i;
	double norm;

	FILE *fileptr = fopen(output_filename, "w");

	for(i = 0; i < n; i++)
	{
		// get the magnitude of the signal
		norm = pow (frequency[i][0], 2) + pow (frequency[i][1], 2);
		norm = sqrt (norm);

		fprintf (fileptr, "%d %f\n", i, norm);
	}

	fclose (fileptr);
}

void fftw_release_data ()
{
	fftw_free (fftw_complex_left);
	fftw_free (fftw_complex_right);

	fftw_free (fftw_complex_left_frequency);
	fftw_free (fftw_complex_right_frequency);
}
