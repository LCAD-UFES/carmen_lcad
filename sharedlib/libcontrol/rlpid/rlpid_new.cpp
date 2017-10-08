#include "rlpid.h"

//   INITIALIZATION
void
initializate_variables(data* data)
{
	int i = 0;

	data->sigma_critical_deviation = 0.0;
	data->previous_error = 0.0;
	data->proportional_error = 0.0;
	data->integral_error = 0.0;
	data->derivative_error = 0.0;
	data->recomended_kp = 0.0;
	data->recomended_ki = 0.0;
	data->recomended_kd = 0.0;
	data->actual_kp = 0.0;
	data->actual_ki = 0.0;
	data->actual_kd = 0.0;
	data->critic_value = 0.0;
	data->previous_critic_value = 0.0;
	data->reinforcement_signal = 0.0;
	data->td_error = 0.0;
	data->u_t = 0.0;

    // INITIALIZE NETWORK
	for (i = 0; i < HIDDEN_UNIT_NEURONS; i++)
	{
		data->neuron[i].w_neuron_weight[0] = /*2.55*/5.1 + random_double() * FF_MULTIPLIER;//W_kp
		data->neuron[i].w_neuron_weight[1] = /*1.2*/2.5 + random_double() * FF_MULTIPLIER;
		data->neuron[i].w_neuron_weight[2] = -0.025 + random_double() * FF_MULTIPLIER;
		data->neuron[i].v_neuron_weight    = random_double() * FF_MULTIPLIER;
		data->neuron[i].center_vector[0]   = random_double() * RBF_MULTIPLIER;
		data->neuron[i].center_vector[1]   = random_double() * RBF_MULTIPLIER;
		data->neuron[i].center_vector[2]   = random_double() * RBF_MULTIPLIER;
		data->neuron[i].width_scalar_sigma = random_double() * RBF_MULTIPLIER;
		data->neuron[i].phi_value = 1; //random_double() * RBF_MULTIPLIER;
	}
}


#define RBF_MULTIPLIER 1.0//5.0//10.0 //1.0
#define FF_MULTIPLIER 0.2 //1.0//5.0 //0.05
#define GAUSS_MULTIPLIER 0.002 //1.0//1.0 //0.05

//   INITIALIZATION
rl_variables
initializate_variables(data* data)
{
	//variables data->pv;
	int i = 0;
	data->td_error = 0;
	data->variables.critic_value = 0;
	data->reinforcement_signal = 0;
	data->variables.sigma_critical_deviation = 0;
	data->total_error_quadratico = 0;

	for (i = 0; i < 3;i++)
	{
		data->variables.U[i] = 0;
		data->variables.error[i] = 0;
		data->variables.error_order[i] = 0;
	}
	data->variables.recomended_pid_params[0] = 1250;
	data->variables.recomended_pid_params[1] = 600;
	data->variables.recomended_pid_params[2] = 25;
	data->variables.pid_params[0] = 1250;//0.12;//1250
	data->variables.pid_params[1] = 600;//0.32; //600
	data->variables.pid_params[2] = 25;//0.08; //25

	// INITIALIZE PAST VARIABLES
	data->pv.critic_value = 0;
	data->pv.sigma_critical_deviation = 0;

	for (i = 0; i < 3;i++)
	{
		data->pv.U[i] = 0;
		data->pv.error[i] = 0;
		data->pv.error_order[i] = 0;
	}
	data->pv.recomended_pid_params[0] = 1250;
	data->pv.recomended_pid_params[1] = 600;
	data->pv.recomended_pid_params[2] = 25;
	data->pv.pid_params[0] = 1250;//0.12; //1250
	data->pv.pid_params[1] = 600;//0.32; //600; //
	data->pv.pid_params[2] = 25;//0.08; //25; //

    // INITIALIZE NETWORK
	for (i = 0; i < neural_network_size; i++)
	{
		data->neuron[i].w_neuron_weight[0] = /*2.55*/5.1 + random_double() * FF_MULTIPLIER;//W_kp
		data->neuron[i].w_neuron_weight[1] = /*1.2*/2.5 + random_double() * FF_MULTIPLIER;
		data->neuron[i].w_neuron_weight[2] = -0.025 + random_double() * FF_MULTIPLIER;
		data->neuron[i].v_neuron_weight = random_double() * FF_MULTIPLIER;
		data->neuron[i].center_vector[0] = random_double() * RBF_MULTIPLIER;
		data->neuron[i].center_vector[1] = random_double() * RBF_MULTIPLIER;
		data->neuron[i].center_vector[2] = random_double() * RBF_MULTIPLIER;
		data->neuron[i].width_scalar_sigma = random_double() * RBF_MULTIPLIER;
		data->neuron[i].phi_value = 1;//random_double() * RBF_MULTIPLIER;
	}

	return data->pv;
}


void
calculate_pid_errors(double y_desired, double y,  double delta_t, data* data)
{
	double error_t_1 = data->error_order_0;

	data->error_order_0 = (y_desired - y);
	data->error_order_1 = data->error_order_1 + data->error_order_0 * delta_t;
	data->error_order_2 = (data->error_order_0 - error_t_1) / delta_t;
}


//   EQUATION 1
double
update_plant_input_u(double atan_desired_curvature, double atan_current_curvature, double delta_t, data* data)
{
	//double 	integral_t_1 = data->variables.error_order[1];
	double 	u_t;			// u(t)	-> actuation in time t


	if (delta_t == 0.0)
	{
		return 0.0;
	}

	calculate_pid_errors(atan_desired_curvature, atan_current_curvature, delta_t, data);

	u_t = (data->actual_kp * data->error_order_0) + (data->actual_ki * data->error_order_1) + (data->actual_kd * data->error_order_2);

	data->error_2 = data->error_1;
	data->error_1 = data->error_0;
	data->error_0 = data->error_order_0;

	// Anti windup
	//if ((u_t < -100.0) || (u_t > 100.0))
	//	data->variables.error_order[1] = integral_t_1;

	u_t = carmen_clamp(-100.0, u_t, 100.0);
	return (-u_t);
}


void
calculate_error_old(double y_desired, double y, data* data)
{
	data->error_2 = data->error_1;
	data->error_1 = data->error_0;
	data->error_0 = (y_desired - y);
}


//   EQUATION 2
void
external_reinforcement_signal(data* data) //r(t) = alfa*Re + beta*Rec
{
	double Re = 0;
	double Rec = 0;
	if (fabs(data->variables.error[0]) > data->params.error_band)
	{
		Re = -0.5;
	}
	if (fabs(data->variables.error[0]) > fabs(data->variables.error[1]))
	{
		Rec = -0.5;
	}
	data->reinforcement_signal = (data->params.alfa_weight_coef*Re) + (data->params.beta_weight_coef*Rec); //r(t+1)
}


// EQUATION 3
// UPDATE A SINGLE NEURON
double
update_neuron_hidden_unit_phi(double width_scalar, double* center_vector, data* data)
{
	double neuron_hidden_phi = pow((data->variables.error_order[0] - center_vector[0]),2) + pow((data->variables.error_order[1] - center_vector[1]),2) + pow((data->variables.error_order[2] - center_vector[2]),2);
	neuron_hidden_phi = -neuron_hidden_phi / (2 * width_scalar * width_scalar);
	neuron_hidden_phi = exp(neuron_hidden_phi);
	return neuron_hidden_phi;
}


// UPDATE ALL NEURAL NETWORK
void
update_neetwork_hidden_unit_phi(data* data)
{
	for (int i = 0; i < neural_network_size;i++)
	{
		data->neuron[i].phi_value = update_neuron_hidden_unit_phi(data->neuron[i].width_scalar_sigma, data->neuron[i].center_vector, data);
	}
}


void
update_neetwork_hidden_unit_phi_future(data* data)
{
	int i = 0;
	for (i = 0; i < neural_network_size;i++)
	{
		data->neuron[i].phi_future = update_neuron_hidden_unit_phi(data->neuron[i].width_scalar_sigma, data->neuron[i].center_vector, data);
	}
}


// EQUATION 4
void
update_recomended_pid_output(data* data)
{
	int i = 0;

	data->recomended_kp = 0.0;
	data->recomended_ki = 0.0;
	data->recomended_kd = 0.0;

	for (i = 0; i < neural_network_size; i++)
	{
		data->recomended_kp = data->recomended_kp + (data->neuron[i].w_neuron_weight[0] * data->neuron[i].phi_value);
		data->recomended_ki = data->recomended_ki + (data->neuron[i].w_neuron_weight[1] * data->neuron[i].phi_value);
		data->recomended_kd = data->recomended_kd + (data->neuron[i].w_neuron_weight[2] * data->neuron[i].phi_value);
	}
}


// EQUATION 5
double
update_critic_value_output(data* data)
{
	double c_value = 0;

	for (int i = 0; i < neural_network_size; i++)
	{
		c_value = c_value + (data->neuron[i].v_neuron_weight * data->neuron[i].phi_value);
	}
	return c_value;
}


double
update_critic_value_future(data* data)
{
	double c_value = 0.0;

	for (int i = 0; i < neural_network_size; i++)
	{
		c_value = c_value + (data->neuron[i].v_neuron_weight * data->neuron[i].phi_future);
	}
	return c_value;
}


// EQUATION 6 GAUSSIAN FUNCTION
double
gaussian_noise(int mean, double sigma)
{
	const double norm = 1.0 / (RAND_MAX + 1.0); // RAND_MAX is the max value that the rand() function can give.
	double u = 1.0 - rand() * norm; // can't let u == 0 //  0 < rand_value <=1

	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * PI * v);
	return (mean + sigma * z);
}


void
update_pid_params(data* data)
{
	data->sigma_critical_deviation = 1 / (1 + exp(2 * data->critic_value));
	double g_noise = gaussian_noise(0, data->sigma_critical_deviation) * GAUSS_MULTIPLIER;
	data->actual_kp = data->recomended_kp + g_noise;

	g_noise = gaussian_noise(0, data->sigma_critical_deviation) * GAUSS_MULTIPLIER;
	data->actual_ki = data->recomended_ki + g_noise;

	g_noise = gaussian_noise(0, data->sigma_critical_deviation) * GAUSS_MULTIPLIER;
	data->actual_kd = data->recomended_kd + g_noise;
}


// EQUATION 7
void
calculate_td_error(data* data)
{
	data->td_error = data->reinforcement_signal + (data->params.discount_factor*data->future_critic_value) - data->variables.critic_value; // 0 < discount_factor < 1
}


// EQUATION 8
double
performance_index_of_system_learning(data* data)
{
	return (pow(data->td_error,2)/2);
}


// EQUATION 9 AND 10
void
weights_update(data* data)
{
	double aux = 0;

	for (int i = 0;i < neural_network_size;i++)
	{
		aux = (data->params.actor_learning_rate * data->td_error * data->neuron[i].phi_value) / data->sigma_critical_deviation;
		data->neuron[i].w_neuron_weight[0] = data->neuron[i].w_neuron_weight[0] + (aux * (data->actual_kp - data->recomended_kp));
		data->neuron[i].w_neuron_weight[1] = data->neuron[i].w_neuron_weight[1] + (aux * (data->actual_ki - data->recomended_ki));
		data->neuron[i].w_neuron_weight[2] = data->neuron[i].w_neuron_weight[2] + (aux * (data->actual_kd - data->recomended_kd));

		data->neuron[i].v_neuron_weight = data->neuron[i].v_neuron_weight + data->params.critic_learning_rate * data->td_error * data->neuron[i].phi_value;
	}
}


// EQUATION 11
void
center_vector_update(data* data)
{
	double aux = 0;

	for (int i = 0;i < neural_network_size;i++)
	{
		// @FILIPE USAR O ERROR_ORDER DO TEMPO T
		aux = (data->params.learning_rate_center * data->td_error * data->neuron[i].v_neuron_weight * data->neuron[i].phi_value) / pow(data->neuron[i].width_scalar_sigma,2);
		data->neuron[i].center_vector[0] = data->neuron[i].center_vector[0] + aux * (data->error_0 - data->neuron[i].center_vector[0]);
		data->neuron[i].center_vector[1] = data->neuron[i].center_vector[1] + aux * (data->error_1 - data->neuron[i].center_vector[1]);
		data->neuron[i].center_vector[2] = data->neuron[i].center_vector[2] + aux * (data->error_2 - data->neuron[i].center_vector[2]);
	}
}


// EQUATION 12
void
width_scalar_update(data* data)
{
	double aux = 0.0;
	double aux2 = 0.0;

	for (int i = 0;i < neural_network_size;i++)
	{
		// @FILIPE USAR O PHI_VALUE DO TEMPO T
		aux = (data->params.learning_rate_width * data->td_error * data->neuron[i].v_neuron_weight * data->neuron[i].phi_value) / pow(data->neuron[i].width_scalar_sigma, 3);
		aux2 = pow(data->error_0 - data->neuron[i].center_vector[0], 2) + pow(data->error_1 - data->neuron[i].center_vector[1], 2) + pow(data->error_2 - data->neuron[i].center_vector[2], 2);
		data->neuron[i].width_scalar_sigma = data->neuron[i].width_scalar_sigma + aux * aux2; // Width Scalar update here
	}
}


// SAVE AND LOAD VARIABLES
void
store_variables(data* data)
{
	data->pv.sigma_critical_deviation = data->variables.sigma_critical_deviation;
	data->pv.critic_value = data->variables.critic_value;

	for(int i = 0; i<3; i++)
	{
		data->pv.U[i] = data->variables.U[i];
		data->pv.error[i] = data->variables.error[i];
		data->pv.error_order[i] = data->variables.error_order[i];
		data->pv.pid_params[i] = data->variables.pid_params[i];
		data->pv.recomended_pid_params[i] = data->variables.recomended_pid_params[i];
	}
}


void
load_variables(data* data)
{
	data->variables.sigma_critical_deviation = data->pv.sigma_critical_deviation;
	data->variables.critic_value = data->pv.critic_value;

	for (int i = 0; i < 3; i++)
	{
		data->variables.U[i] = data->pv.U[i];
		data->variables.error[i] = data->pv.error[i];
		data->variables.error_order[i] = data->pv.error_order[i];
		data->variables.pid_params[i] = data->pv.pid_params[i];
		data->variables.recomended_pid_params[i] = data->pv.recomended_pid_params[i];
	}
}


double
carmen_librlpid_compute_effort(double current_curvature, double desired_curvature, double delta_t)
{
	static bool first_time = true;
	static data data;
	double effort = 0.0;  // U(t) Control command

	if(first_time)
	{
		data.params = read_parameters("rlpid_params.txt");
		data = initializate_variables(&data); // Step 1
		first_time = false;
	}
	else
	{
		calculate_error_old(desired_curvature, current_curvature, &data);       // Step 6 ==> CALCULA ERRO
		update_neetwork_hidden_unit_phi_future(&data);                          //        ==> UPDATE PHI
		data.future_critic_value = update_critic_value_future(&data);           // Step 7 ==> UPDATE V
		calculate_td_error(&data);                                              // Step 8 ==> CALCULA ERRO TD
		load_variables(&data);
		weights_update(&data);                                                  // Step 9 ==> UPDATE PESOS
		center_vector_update(&data);                                            // Step 10 ==> UPDATE CENTRO
		width_scalar_update(&data);                                             // Step 10 ==> UPDATE WIDTH SCALAR
	}

	load_variables(&data);
	calculate_error_old(desired_curvature, current_curvature, &data);           // Step 2 ==> CALCULA ERRO
	external_reinforcement_signal(&data);                                       // Step 3 ==> RECOMPENSA
	update_neetwork_hidden_unit_phi(&data);                                     //        ==> UPDATE PHI
	update_recomended_pid_output(&data);                                        // Step 4 ==> UPDATE K'
	data.critic_value = update_critic_value_output(&data);	                    // Step 4 ==> UPDATE V
	update_pid_params(&data);                                                   // Step 5 ==> UPDATE K
	effort = update_plant_input_u(current_curvature, desired_curvature, delta_t, &data); // Step 5 ==> UPDATE U
	store_variables(&data);

	printf("u%lf el%f kp%lf ki%lf kd%lf\n", effort, data.error_0, data.actual_kp, data.actual_ki, data.actual_kd);

	return effort;
}
