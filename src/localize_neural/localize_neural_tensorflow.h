/*
 * localize_neural_inference.h
 *
 *  Created on: Jul 3, 2017
 *      Author: avelino
 */

#include <carmen/carmen.h>

#ifndef SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_INFERENCE_H_
#define SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_INFERENCE_H_

void initialize_tensorflow(const char * saved_network);

void finalize_tensorflow();

carmen_point_t run_cnn_inference(const carmen_localize_neural_imagepos_message &keyframe, const carmen_localize_neural_imagepos_message &curframe);

#endif /* SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_INFERENCE_H_ */
