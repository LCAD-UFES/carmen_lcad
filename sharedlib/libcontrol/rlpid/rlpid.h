#ifndef RLPID_H
#define RLPID_H

#ifdef __cplusplus
extern "C" {
#endif


#include <car_model.h>


#define neural_network_size (6)
#define PI (3.141592653589793)

typedef struct {
	double alfa_weight_coef;       //alfa
	double beta_weight_coef;       //beta
	double learning_rate_center;   //Eta mi
	double learning_rate_width;    //Eta gama
	double discount_factor;        //Gama
	double error_band;             //epslon
	double actor_learning_rate;    //alfa_a
	double critic_learning_rate;   //alfa_c
} intelligent_control_params;

typedef struct {
	double past_td_error;
	double past_sigma_critical_deviation;
	double past_critic_value; //V(t)
	double past_future_critic_value; //V(t+1)
	double past_reinforcement_signal; // r(t)
	double past_U[3]; //The size of the vector is 3, because "u" uses U(t-2) in equation, this means that you need store the past value of U
	double past_error[3]; //You need the last 3 values of e =>t(t), e(t-1) and e(t-2)
	double past_error_order[3]; //e(t), Delta(e) and Delta^2(e)
	double past_pid_params[3]; //K ->The parameters Kp,Ki and Kd respectvely
	double past_recomended_pid_params[3]; //K' ->The new recomended params of Kp,Ki and Kd respectvely
} past_variables;


typedef struct {
	double center_vector[3];
	double width_scalar_sigma;
	double phi_value;
	double phi_future;
	double w_neuron_weight[3];
	double v_neuron_weight;
} rbf_neuron;


double
carmen_librlpid_compute_effort_signal(double current_phi, double desired_phi, double next_desired_phi, fann_type *steering_ann_input,
	struct fann *steering_ann, double v, double understeer_coeficient, double distance_between_front_and_rear_axles,
	double max_phi);


double carmen_librlpid_compute_effort(double current_curvature, double desired_curvature);


#ifdef __cplusplus
}
#endif

#endif // RLPID_H
