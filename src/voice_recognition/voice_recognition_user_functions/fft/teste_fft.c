/*
 * teste_fft.c
 *c
 *  Created on: 29/05/2012
 *      Author: filipe
 */

#include "wav_decoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fftw3.h>

fftw_complex *sin_in;
fftw_complex *sin_out;

unsigned int N = 100;
double B = 0;
double E = 31.41526;

void alloc_data (void)
{
	sin_in = (fftw_complex *) fftw_malloc (sizeof(fftw_complex) * N);
	sin_out = (fftw_complex *) fftw_malloc (sizeof(fftw_complex) * N);
}

void generate_sin_serie (void)
{
	double JMP = (double) (E - B) / (double) N;

	unsigned int i;

	for(i = 0; i < N; i++)
	{
		// real part
		sin_in[i][0] = sin(B + i * JMP);
		sin_in[i][1] = 0;
	}
}

void release_data (void)
{
	fftw_free (sin_in);
	fftw_free (sin_out);
}

void apply_fft (void)
{
	fftw_plan fftw3_plan = fftw_plan_dft_1d (N, sin_in, sin_out, FFTW_FORWARD, FFTW_ESTIMATE);

	fftw_execute (fftw3_plan);
	fftw_destroy_plan(fftw3_plan);
}

double double_abs (double x)
{
	return ((x < 0) ? (-x) : (x));
}

void write_frequency_to_file (fftw_complex *frequency, char *filename)
{
	double JMP = (double) (E - B) / (double) N;
	unsigned int i;
	double norm;

	FILE *fileptr = fopen(filename, "w");

	for(i = 0; i < N; i++)
	{
		norm = frequency[i][0];

		fprintf (fileptr, "%f %f\n", B + i * JMP, norm);
	}

	fclose (fileptr);
}

void write_fft_to_file (fftw_complex *frequency, char *filename)
{
	double JMP = (double) (E - B) / (double) N;
	unsigned int i;

	FILE *fileptr = fopen(filename, "w");

	double min_freq = 0;
	double max_freq = 1 / (2 * JMP);
	double var_freq = (max_freq - min_freq) / (N / 2);

	for(i = 0; i < N/2; i++)
	{
		fprintf (fileptr, "%f %f\n", (double) (i) * var_freq, double_abs(frequency[i][0]));
	}

	fclose (fileptr);
}


int main (void)
{
	alloc_data();
	generate_sin_serie();
	write_frequency_to_file(sin_in, "sin.in.dat");
	apply_fft();
	write_fft_to_file(sin_out, "sin.out.dat");
	release_data();

	printf("Terminou!\n");
	return 0;
}


