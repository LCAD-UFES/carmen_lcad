#include <carmen/carmen.h>
#include "intelligent_control.h"


double reward;
double td_error;
double standard_deviation;
double critic_value_actual;
double critic_value_new;

double U_t[2];
double system_error[3];
double k_pid_params[3];
double recomended_pid_params[3];
rbf_neuron network[neural_network_size];


intelligent_control_params
read_parameters (char* argv)
{
	FILE* file = fopen(argv, "r");
	intelligent_control_params params; 
	char trash[50];
	
	fscanf(file, "%lf", &params.alfa_weight_coef);
	//printf("%f\n", params.alfa_weight_coef);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);
	
	fscanf(file, "%lf", &params.beta_weight_coef);
	//printf("%f\n", params.beta_weight_coef);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);
	
	fscanf(file, "%lf", &params.learning_rate_center);
	//printf("%f\n", params.learning_rate_center);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);
	
	fscanf(file, "%lf", &params.learning_rate_width);
	//printf("%f\n", params.learning_rate_width);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);
	
	fscanf(file, "%lf", &params.discount_factor);
	//printf("%f\n", params.discount_factor);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);
	
	fscanf(file, "%lf", &params.error_band);
	//printf("%f\n", params.error_band);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);
	
	fscanf(file, "%lf", &params.actor_learning_rate);
	//printf("%f\n", params.alfa_a);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);
	
	fscanf(file, "%lf", &params.critic_learning_rate);
	//printf("%f\n", params.alfa_c);
	
	return (params);
}


void
intelligent_control_initialize ()
{
	int i;
	
	reward = 0;
	td_error = 0;
	standard_deviation = 0;
	critic_value_actual = 0;
	critic_value_new = 0;
	U_t[0] = 0;
	U_t[1] = 0;
	
	for (i = 0; i < 3; i++)
	{
		system_error[i] = 0;
		k_pid_params[i] = 0;
		recomended_pid_params[i] = 0;
	}
	
	for (i = 0; i < neural_network_size; i++)
	{
		network[i].width_scalar_sigma = 0;
		network[i].phi_value = 0;
		network[i].v_neuron_wight = 0;
		
		for (int j = 0; j < 3; j++)
		{
			network[i].center_vector_mi[j] = 0;
			network[i].w_neuron_wight[j] = 0;
		}
	}
}


void
calculate_system_error (double Y_disired, double Y_t)
{
	double e_t = Y_disired - Y_t;
	
	system_error[2] =  e_t - 2 * system_error[1] + system_error[2];
	system_error[1] =  e_t - system_error[0];
	system_error[0] =  e_t;
}


void
receive_imediate_reward (double alfa_weight_coef, double beta_weight_coef, double error_band)
{	
	double Re, Rec;
	if (abs(system_error[0]) <= error_band)
	{
		Re = 0;
	}
	else
	{
		Re = (-0.5);
	}
	
	if (abs(system_error[0]) <= abs(system_error[1]))
	{
		Rec = 0;
	}
	else
	{
		Rec = (-0.5);
	}
	reward = (alfa_weight_coef * Re + beta_weight_coef * Rec);
}


double
update_neuron_phi_value (double center_vector[2], double width_scalar)
{
	return exp ( - (pow ((system_error[0] - center_vector[0]), 2) + pow ((system_error[1] - center_vector[1]), 2) + pow ((system_error[2] - center_vector[2]), 2))
	                                      / (2 * pow (width_scalar, 2)));
}


void
update_network_phi_value ()
{
	for (int i = 0; i < neural_network_size; i++)
	{
		network[i].phi_value = update_neuron_phi_value (network[i].center_vector_mi, network[i].width_scalar_sigma);
	}
}


void
calculate_actor_output_and_critic_value()
{
	recomended_pid_params[0] = 0;
	recomended_pid_params[1] = 0;
	recomended_pid_params[2] = 0;
	critic_value_actual = 0;
	
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < neural_network_size; j++)
		{
			 recomended_pid_params[i] = recomended_pid_params[i] + network[j].w_neuron_wight[i] * network[j].phi_value;
		}
	}
	
	for (int k = 0; k < neural_network_size; k++)
	{
		 critic_value_actual = critic_value_actual + network[k].v_neuron_wight * network[k].phi_value;
	}
}


void
calculate_actual_PID_param_K ()
{
	standard_deviation = 1 / (1 + exp(2 * critic_value_actual));
	
	for (int i = 0; i < 3; i++)
	{
		k_pid_params[i] = recomended_pid_params[i] + carmen_gaussian_random(0, standard_deviation);
	}
}


void
apply_u_to_controlled_plant ()
{
	
}


void
calculate_TD_error (double discount_factor, double critic_value_actual)
{
	td_error = reward + (discount_factor * critic_value_new) - critic_value_actual;
}


void
update_actor_critic_wights (double actor_learning_rate, double critic_learning_rate)
{
	for (int j = 0; j < neural_network_size; j++)
	{
		for (int m = 0; m < 3; m++)
		{
			network[j].w_neuron_wight[m] = network[j].w_neuron_wight[m] + (actor_learning_rate * td_error *
				((k_pid_params[m] - recomended_pid_params[m]) / standard_deviation) * network[j].phi_value);
		}
		network[j].v_neuron_wight = network[j].v_neuron_wight + (critic_learning_rate * td_error * network[j].phi_value);
	}
}


void
update_centers_and_widths (double learning_rate_center, double learning_rate_width)
{
	double aux;
	
	for (int j = 0; j < neural_network_size; j++)
	{
		aux = (learning_rate_center * td_error * network[j].v_neuron_wight * network[j].phi_value) / pow(network[j].width_scalar_sigma, 2);
		network[j].center_vector_mi[1] = network[j].center_vector_mi[1] + aux * (system_error[0] - network[j].center_vector_mi[0]);
		network[j].center_vector_mi[1] = network[j].center_vector_mi[1] + aux * (system_error[1] - network[j].center_vector_mi[1]);
		network[j].center_vector_mi[1] = network[j].center_vector_mi[1] + aux * (system_error[2] - network[j].center_vector_mi[2]);
		
		aux = (learning_rate_width * td_error * network[j].v_neuron_wight * network[j].phi_value) / pow(network[j].width_scalar_sigma, 3);
		
		network[j].width_scalar_sigma = network[j].width_scalar_sigma + aux * (pow((system_error[0] - network[j].center_vector_mi[0]), 2) 
			+ pow((system_error[1] - network[j].center_vector_mi[1]), 2) + pow((system_error[2] - network[j].center_vector_mi[2]), 2));
	}
}


void 
calculate_control_output_pid ()
{
	double aux;
	
	aux = U_t[0];
	U_t[0] = U_t[1] + (k_pid_params[0] * system_error[0]) + (k_pid_params[1] * system_error[1]) + (k_pid_params[2] * system_error[2]);
	U_t[1] = aux;
}


double
calculate_disired_test_signal (int t)
{
	double angle, frequence;
	
	frequence = 0.025;            // = (1/40)
	angle = (2 * PI * frequence * t);
	
	return sin(angle);;
}


double
detect_actual_system_output (double Y_t_1, double t)
{
	double a;
	
	a = 1.2 * (1 - 0.8 * exp(-0.01 * (t-1500)));
	
	return (((a * Y_t_1 * U_t[0]) / (1 + pow(Y_t_1, 2))) + U_t[1]);
}


int
main (int argc, char** argv)
{
	double Y_disired=0, Y_t=0, sine;
	argc = 1;
	intelligent_control_params params = read_parameters(argv[argc]);
	intelligent_control_initialize ();
	
	for (int t = 0; t < 1000; t++)
	{
		Y_t = detect_actual_system_output (Y_t, t);
		//printf ("%lf\n", Y_t); 
		sine = calculate_disired_test_signal (t);
		printf ("%lf\n", sine); 
		calculate_system_error (Y_disired, Y_t);
		receive_imediate_reward (params.alfa_weight_coef, params.beta_weight_coef, params.error_band);
		calculate_actor_output_and_critic_value();
		calculate_actual_PID_param_K ();
		calculate_control_output_pid ();
		//aply ut to controled plant TODO
		Y_t = detect_actual_system_output (Y_t, (t+1));
		receive_imediate_reward (params.alfa_weight_coef, params.beta_weight_coef, params.error_band);
		calculate_actor_output_and_critic_value();
		calculate_TD_error (params.discount_factor, critic_value_actual);
		update_actor_critic_wights (params.actor_learning_rate, params.critic_learning_rate);
		update_centers_and_widths (params.learning_rate_center, params.learning_rate_width);
		
		//printf ("%lf %lf\n", Y_t, Y_disired);
	}
	return 0;
}