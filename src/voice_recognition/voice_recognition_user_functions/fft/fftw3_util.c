/*
 * fft3.c
 *
 *  Created on: 28/05/2012
 *      Author: filipe
 */
#include <fftw3.h>
#include <math.h>

void fftw_fourier_transform (fftw_complex *input, fftw_complex *output, unsigned int n)
{
	fftw_plan fftw3_plan =
		fftw_plan_dft_1d (
			n,
			input,
			output,
			FFTW_FORWARD, // default option
			FFTW_ESTIMATE // default option
		);

	fftw_execute (fftw3_plan);
	fftw_destroy_plan(fftw3_plan);
}

void fftw_short2complex (short *input, fftw_complex *output, unsigned int n)
{
	unsigned int i;

	for(i = 0; i < n; i++)
	{
		output[i][0] = input [i];
		output[i][1] = 0;
	}
}

void fftw_int2complex (int *input, fftw_complex *output, unsigned int n)
{
	unsigned int i;

	for(i = 0; i < n; i++)
	{
		output[i][0] = input [i];
		output[i][1] = 0;
	}
}

void fftw_float2complex (float *input, fftw_complex *output, unsigned int n)
{
	unsigned int i;

	for(i = 0; i < n; i++)
	{
		output[i][0] = input [i];
		output[i][1] = 0;
	}
}

void fftw_double2complex (double *input, fftw_complex *output, unsigned int n)
{
	unsigned int i;

	for(i = 0; i < n; i++)
	{
		output[i][0] = input [i];
		output[i][1] = 0;
	}
}
