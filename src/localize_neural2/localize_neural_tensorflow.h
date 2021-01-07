/*
 * localize_neural_tensorflow.h
 *
 *  Created on: Jul 3, 2017
 *      Author: avelino
 *
 IFLAGS += -std=c++11 -isystem /usr/local/include/tensorflow \
		  -isystem /usr/local/include/tensorflow/bazel-genfiles \
		  -isystem /usr/local/include/tensorflow/tensorflow/core/lib/core \
		  -isystem /usr/local/include/tensorflow/tensorflow/contrib/makefile/downloads \
		  -isystem /usr/local/include/tensorflow/tensorflow/contrib/makefile/downloads/eigen \
		  -isystem /usr/local/include/tensorflow/tensorflow/contrib/makefile/downloads/gemmlowp \
		  -isystem /usr/local/include/tensorflow/tensorflow/contrib/makefile/gen/protobuf-host/include \
		  -isystem /usr/local/include/tensorflow/third_party/eigen3
LFLAGS += -L/usr/local/lib/tensorflow_cc/ -ltensorflow_cc
 *
 */

#include <carmen/carmen.h>

#ifndef SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_TENSORFLOW_H_
#define SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_TENSORFLOW_H_

void initialize_tensorflow(const char * saved_network);

void finalize_tensorflow();

carmen_point_t run_cnn_inference(const carmen_localize_neural_imagepos_message &keyframe, const carmen_localize_neural_imagepos_message &curframe);

#endif /* SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_TENSORFLOW_H_ */
