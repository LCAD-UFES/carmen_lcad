#ifndef CAR_NEURAL_MODEL_H
#define CAR_NEURAL_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_VELOCITY_ANN_INPUTS	360
#define NUM_STEERING_ANN_INPUTS	80

void
carmen_libcarneuralmodel_init_steering_ann_input(fann_type *input);


void
carmen_libcarneuralmodel_build_steering_ann_input(fann_type *input, double s, double cc);


void
carmen_libcarneuralmodel_init_steering_ann (struct fann *steering_ann, fann_type *steering_ann_input);


#ifdef __cplusplus
}
#endif

#endif /* CAR_NEURAL_MODEL_H */
