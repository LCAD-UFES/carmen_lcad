#ifndef RLPIDVEL_H
#define RLPIDVEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include <car_model.h>
#include "../control.h"


#define PI (3.141592653589793)
#define neural_network_size (1000)
#define N_HIDDEN 1000


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
	double center_vector[3];
	double width_scalar_sigma;
	double phi_value;
	double phi_future;
	double w_neuron_weight[3];
	double v_neuron_weight;
} rbf_neuron;


typedef struct {
	double sigma_critical_deviation;
	double critic_value; //V(t)
	double U[3]; //The size of the vector is 3, because "u" uses U(t-2) in equation, this means that you need store the past value of U
	double error[3]; //You need the last 3 values of e => t(t), e(t-1) and e(t-2)
	double error_order[3]; //e(t), Delta(e) and Delta^2(e)
	double pid_params[3]; //K ->The parameters Kp,Ki and Kd respectvely
	double recomended_pid_params[3]; //K' ->The new recomended params of Kp,Ki and Kd respectvely
} rl_variables;


typedef struct {
	intelligent_control_params params;
	rbf_neuron network[neural_network_size]; //The size is defined at .h file
	rl_variables variables;
	rl_variables pv; // Past variables
	double td_error;
	double future_critic_value; //V(t+1)
	double reinforcement_signal; // r(t)
	double best_pid[3];
	double total_error_quadratico;
} rl_data;


typedef struct {
	double sigma_critical_deviation;
	double previous_error;
	double proportional_error;
	double integral_error;
	double derivative_error;
	double error_0;
	double error_1;
	double error_2;
	double error_order_0;
	double error_order_1;
	double error_order_2;
	double recomended_kp;
	double recomended_ki;
	double recomended_kd;
	double actual_kp;
	double actual_ki;
	double actual_kd;
	double critic_value;
	double future_critic_value;
	double previous_critic_value;
	double reinforcement_signal;
	double td_error;

	intelligent_control_params params;
	rbf_neuron neuron[N_HIDDEN];   // Number of Neurons in the Hidden Unit
} data;


void
carmen_librlpid_exporta_pesos();


/*double
carmen_librlpid_compute_effort_signal(double current_phi, double desired_phi, double next_desired_phi, fann_type *steering_ann_input,
	struct fann *steering_ann, double v, double understeer_coeficient, double distance_between_front_and_rear_axles, double max_phi);*/


double
carmen_librlpid_compute_effort(double current_curvature, double desired_curvature, double delta_t);


double
carmen_librlpid_compute_effort_new(double current_curvature, double desired_curvature, double delta_t);


#ifdef __cplusplus
}
#endif

#endif // RLPIDVEL_H
