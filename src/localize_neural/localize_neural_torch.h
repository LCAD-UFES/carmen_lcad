/*
 * localize_neural_torch.h
 *
 *  Created on: Sep 22, 2017
 *      Author: avelino
 */

#ifndef SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_TORCH_H_
#define SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_TORCH_H_

#include <carmen/carmen.h>

void initialize_network(const char * saved_network);

void finalize_network();

carmen_pose_3D_t forward_network(
		const carmen_localize_neural_imagepos_message &curframe,
		const carmen_localize_neural_imagepos_message &keyframe);

#endif /* SRC_LOCALIZE_NEURAL_LOCALIZE_NEURAL_TORCH_H_ */
