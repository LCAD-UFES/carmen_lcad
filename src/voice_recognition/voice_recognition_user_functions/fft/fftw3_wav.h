/*
 * fftw3_interface.h
 *
 *  Created on: 29/05/2012
 *      Author: filipe
 */

#ifndef __FFTW3_INTERFACE_H_
#define __FFTW3_INTERFACE_H_

#include "fftw3_util.h"

void fftw_convert_raw_data_to_complex ();
void fftw_apply_fourier_transform ();
void fftw_write_frequency_to_file (fftw_complex *frequency, char *output_filename);
void fftw3_release_data ();

#endif /* FFTW3_INTERFACE_H_ */
