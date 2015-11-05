/*
 * fft3.h
 *
 *  Created on: 28/05/2012
 *      Author: filipe
 */

#ifndef __FFT3_H_
#define __FFT3_H_

#include <fftw3.h>

/**
 * Function to perform fast fourier transform.
 * The n/2 first elements of the output vector correspond to the output frequency
 *
 * @param input vector of complex numbers
 * @param output vector of complex numbers
 * @param n size of input and output vector (n MUST be in base 2)
 */
void fftw_fourier_transform (fftw_complex *input, fftw_complex *output, unsigned int n);

/**
 * Functions to convert input vectors to fftw_complex data.
 * The output vector should be allocated with fftw_malloc
 *
 * @param input input data
 * @param output fftw_complex converted data
 * @param n size of input and output arrays
 */
void fftw_short2complex (short *input, fftw_complex *output, unsigned int n);
void fftw_int2complex (int *input, fftw_complex *output, unsigned int n);
void fftw_float2complex (float *input, fftw_complex *output, unsigned int n);
void fftw_double2complex (double *input, fftw_complex *output, unsigned int n);

#endif /* __FFT3_H_ */
