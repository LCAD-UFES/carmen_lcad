/*
 * libfcnpy.h
 *
 *  Created on: Nov 29, 2018
 *      Author: vinicius
 */

#ifndef MAPPER_lib_neural_mapper_py_H_
#define MAPPER_lib_neural_mapper_py_H_

#include <carmen/carmen.h>
#include <carmen/map.h>

void
initialize_inference_context_mapper_();

double*
process_map_neural_mapper(int size, carmen_map_t *map_max, carmen_map_t *map_mean, carmen_map_t *map_min, carmen_map_t *map_numb, carmen_map_t *map_std);
#endif /*MAPPER_lib_neural_mapper_py_H_ */
