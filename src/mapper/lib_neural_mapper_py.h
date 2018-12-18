/*
 * libfcnpy.h
 *
 *  Created on: Nov 29, 2018
 *      Author: vinicius
 */

#ifndef MAPPER_lib_neural_mapper_py_H_
#define MAPPER_lib_neural_mapper_py_H_


void
initialize_inference_context_mapper();

unsigned char*
process_image_nm(int width, int height, unsigned char *image1, unsigned char *image2, unsigned char *image3, unsigned char *image4, unsigned char *image5);

#endif /*MAPPER_lib_neural_mapper_py_H_ */
