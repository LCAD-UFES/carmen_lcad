
/**
 * @File: wav_io.h
 * @Author: _Filipe
 * @Date: 23/05/2012 19:02
 * 
 * This file describes functions to print information about a 
 * WAVE internal structures.
 * 
 */

#ifndef __WAV_IO_H_
#define __WAV_IO_H_

#include "wav_decoder.h"

void wav_show_header_and_format_information (void);
void wav_show_list_information (void);
void wav_show_data_information (void);
void write_raw_data_to_file (char *output_filename);

#endif // __WAV_IO_H_
