/*
 * spectogram.h
 *
 *  Created on: 29/05/2012
 *      Author: filipe
 */

#ifndef __SPECTOGRAM_H_
#define __SPECTOGRAM_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

void spectogram_alloc (void);
void spectogram_create_fft_input (unsigned int);
void spectogram_apply_fft (void);
void spectogram_update_with_fft (void);
void spectogram_create (unsigned int);
void spectogram_release (void);
void spectogram_write_to_file (char *filename);
void spectogram_show ();

#endif /* __SPECTOGRAM_H_ */
