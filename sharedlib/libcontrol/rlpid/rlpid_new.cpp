#include "rlpid.h"


intelligent_control_params
read_parameters(const char* parameters)        //   READ PARAMETERS From file
{
	FILE* file = fopen(parameters, "r");

	if(file == NULL)
	{
		printf("\nError: Could not open Reinforcement Learning PID parameters\n\n");
		exit(1);
	}

	intelligent_control_params params;
	char trash[50];

	fscanf(file, "%lf", &params.alfa_weight_coef);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);

	fscanf(file, "%lf", &params.beta_weight_coef);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);

	fscanf(file, "%lf", &params.learning_rate_center);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);

	fscanf(file, "%lf", &params.learning_rate_width);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);

	fscanf(file, "%lf", &params.discount_factor);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);

	fscanf(file, "%lf", &params.error_band);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);

	fscanf(file, "%lf", &params.actor_learning_rate);
	fscanf(file, "%s", trash);
	fscanf(file, "%s", trash);

	fscanf(file, "%lf", &params.critic_learning_rate);

	return (params);
}


double
random_double()
{
	return ((double) rand() / (double) RAND_MAX);
}


#define RBF_MULTIPLIER 1.0//5.0//10.0 //1.0
#define FF_MULTIPLIER 0.2 //1.0//5.0 //0.05
#define GAUSS_MULTIPLIER 0.002 //1.0//1.0 //0.05


//   INITIALIZATION
rl_variables
initializate_variables(rl_data* data)
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
		data->network[i].w_neuron_weight[0] = /*2.55*/5.1 + random_double() * FF_MULTIPLIER;//W_kp
		data->network[i].w_neuron_weight[1] = /*1.2*/2.5 + random_double() * FF_MULTIPLIER;
		data->network[i].w_neuron_weight[2] = -0.025 + random_double() * FF_MULTIPLIER;
		data->network[i].v_neuron_weight = random_double() * FF_MULTIPLIER;
		data->network[i].center_vector[0] = random_double() * RBF_MULTIPLIER;
		data->network[i].center_vector[1] = random_double() * RBF_MULTIPLIER;
		data->network[i].center_vector[2] = random_double() * RBF_MULTIPLIER;
		data->network[i].width_scalar_sigma = random_double() * RBF_MULTIPLIER;
		data->network[i].phi_value = 1;//random_double() * RBF_MULTIPLIER;
	}

	return data->pv;
}


void
compute_errors(double y_actual, double y_desired, double delta_t, rl_data* data)
{
	double error = (y_desired - y_actual);

	data->variables.previous_error = data->variables.proportional_error;
	data->variables.proportional_error = error;
	data->variables.integral_error = data->variables.integral_error + (error * delta_t);
	data->variables.derivative_error = (data->variables.derivative_error - error) / delta_t;
}


//   EQUATION 1 eq1
void
compute_control_command_u(double delta_t, rl_data* data)
{
	double u_t = (data->variables.kp * data->variables.proportional_error) + (data->variables.ki * data->variables.integral_error) +
			(data->variables.kd * data->variables.derivative_error);

	u_t = carmen_clamp(-100.0, u_t, 100.0);
	data->variables.control_command_u = -u_t;
}


//   EQUATION 2 eq2 Reward
void
compute_external_reinforcement_signal(rl_data* data)     //r(t) = alfa*Re + beta*Rec
{
	double Re = 0.0;
	double Rec = 0.0;

	if (fabs(data->variables.proportional_error) > data->params.error_band)
	{
		Re = -0.5;
	}

	if (fabs(data->variables.proportional_error) > fabs(data->variables.previous_error))
	{
		Rec = -0.5;
	}

	data->variables.previous_reinforcement_signal = data->variables.reinforcement_signal;
	data->variables.reinforcement_signal = ((data->params.alfa_weight_coef * Re) + (data->params.beta_weight_coef * Rec));
}


// EQUATION 3 eq3 UPDATE A SINGLE NEURON
double
update_neuron_hidden_unit_phi(double width_scalar, double* center_vector, rl_data* data)
{
	double neuron_hidden_phi = pow((data->variables.error_order[0] - center_vector[0]),2) + pow((data->variables.error_order[1]
								- center_vector[1]),2) + pow((data->variables.error_order[2] - center_vector[2]),2);

	neuron_hidden_phi = -neuron_hidden_phi / (2 * width_scalar * width_scalar);
	neuron_hidden_phi = exp(neuron_hidden_phi);
	return neuron_hidden_phi;
}


// UPDATE ALL NEURAL NETWORK
void
update_neetwork_hidden_unit_phi(rl_data* data)
{
	int i = 0;
	for (i = 0; i < neural_network_size;i++)
	{
		data->network[i].phi_value = update_neuron_hidden_unit_phi(data->network[i].width_scalar_sigma, data->network[i].center_vector, data);
	}
}


void
update_neetwork_hidden_unit_phi_future(rl_data* data)
{
	int i = 0;
	for (i = 0; i < neural_network_size;i++)
	{
		data->network[i].phi_future = update_neuron_hidden_unit_phi(data->network[i].width_scalar_sigma, data->network[i].center_vector, data);
	}
}


// EQUATION 4 eq4
void
compute_recomended_pid_output(rl_data* data)
{
	int i = 0;

	data->variables.kp = 0.0;
	data->variables.ki = 0.0;
	data->variables.kd = 0.0;

	for (i = 0; i < neural_network_size; i++)    // FINDING THE RECOMMENDED KP
	{
		data->variables.kp += data->network[i].w_neuron_weight[0] * data->network[i].phi_value;
	}

	for (i = 0; i < neural_network_size; i++)    // FINDING THE RECOMMENDED KI
	{
		data->variables.ki += data->network[i].w_neuron_weight[1] * data->network[i].phi_value;
	}

	for (i = 0; i < neural_network_size; i++)    // FINDING THE RECOMMENDED KD
	{
		data->variables.kd += data->network[i].w_neuron_weight[2] * data->network[i].phi_value;
	}
}


// EQUATION 5 eq5 FINDING THE CRITIC VALUE OUTPUT
void
compute_critic_value(rl_data* data)
{
	double c_value = 0;

	for (int i = 0; i < neural_network_size; i++)
	{
		c_value = c_value + (data->network[i].v_neuron_weight * data->network[i].phi_value);
	}

	data->variables.previous_critic_value = data->variables.critic_value;
	data->variables.critic_value = c_value;
}


// EQUATION 6 eq6 GAUSSIAN FUNCTION
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
update_pid_params(rl_data* data)
{
	data->variables.sigma_critical_deviation = 1 / (1 + exp(2 * data->variables.critic_value));
	double g_noise = gaussian_noise(0, data->variables.sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	data->variables.pid_params[0] = data->variables.recomended_pid_params[0] + g_noise;
	g_noise = gaussian_noise(0, data->variables.sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	data->variables.pid_params[1] = data->variables.recomended_pid_params[1] + g_noise;
	g_noise = gaussian_noise(0, data->variables.sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	data->variables.pid_params[2] = data->variables.recomended_pid_params[2] + g_noise;
}


// EQUATION 7 eq7
void
compute_td_error(rl_data* data)    // 0 < discount_factor < 1
{
	data->td_error = data->variables.previous_reinforcement_signal + (data->params.discount_factor * data->variables.critic_value) - data->variables.previous_critic_value; // 0 < discount_factor < 1

}


// EQUATION 9 AND 10 eq9 eq10
void
weights_update(rl_data* data)
{
	int i = 0;
	double aux = 0;
	for (i = 0;i < neural_network_size;i++)
	{
		// W WEIGHTS UPDATE-EQUATION 9
		aux = /*fabs(*/(data->params.actor_learning_rate * data->td_error * data->network[i].phi_value) / data->variables.sigma_critical_deviation/*network[i].width_scalar_sigma*/;//);
		data->network[i].w_neuron_weight[0] = data->network[i].w_neuron_weight[0] + (aux*(data->variables.pid_params[0] - data->variables.recomended_pid_params[0])); //Kp
		data->network[i].w_neuron_weight[1] = data->network[i].w_neuron_weight[1] + (aux*(data->variables.pid_params[1] - data->variables.recomended_pid_params[1])); //Ki
		data->network[i].w_neuron_weight[2] = data->network[i].w_neuron_weight[2] + (aux*(data->variables.pid_params[2] - data->variables.recomended_pid_params[2])); //Kd

		// V WEIGHT UPDATE-EQUATION 10
		aux = data->params.critic_learning_rate * data->td_error * data->network[i].phi_value;
		data->network[i].v_neuron_weight = data->network[i].v_neuron_weight + aux;
	}
}


// EQUATION 11 eq11
void
center_vector_update(rl_data* data)
{
	int i = 0;
	double aux = 0;
	for (i = 0;i < neural_network_size;i++)
	{
		// @FILIPE USAR O ERROR_ORDER DO TEMPO T
		aux = (data->params.learning_rate_center * data->td_error * data->network[i].v_neuron_weight * data->network[i].phi_value) / pow(data->network[i].width_scalar_sigma,2);
		data->network[i].center_vector[0] = data->network[i].center_vector[0] + aux * (data->variables.error_order[0] - data->network[i].center_vector[0]);//Ki
		data->network[i].center_vector[1] = data->network[i].center_vector[1] + aux * (data->variables.error_order[1] - data->network[i].center_vector[1]);//Kp
		data->network[i].center_vector[2] = data->network[i].center_vector[2] + aux * (data->variables.error_order[2] - data->network[i].center_vector[2]);//Kd
	}
}


// EQUATION 12 eq12
void
width_scalar_update(rl_data* data)
{
	int i = 0;
	double aux = 0;
	double aux2 = 0;
	for (i = 0;i < neural_network_size;i++)
	{
		// @FILIPE USAR O PHI_VALUE DO TEMPO T
		aux = (data->params.learning_rate_width * data->td_error * data->network[i].v_neuron_weight * data->network[i].phi_value) / pow(data->network[i].width_scalar_sigma, 3);
		aux2 = pow(data->variables.error_order[0] - data->network[i].center_vector[0], 2) + pow(data->variables.error_order[1] - data->network[i].center_vector[1], 2) + pow(data->variables.error_order[2] - data->network[i].center_vector[2], 2);
		data->network[i].width_scalar_sigma = data->network[i].width_scalar_sigma + aux * aux2; // Width Scalar update here
	}
}


double
carmen_librlpid_compute_effort(double current_curvature, double desired_curvature, double delta_t)
{
	static bool first_time = true;
	static rl_data data;

	if(first_time)
	{
		first_time = false;
		data.params = read_parameters("rlpid_params.txt");
		data.variables = initializate_variables(&data);

		compute_errors(current_curvature, desired_curvature, delta_t, &data);
		compute_external_reinforcement_signal(&data);
		compute_critic_value(&data);
		compute_recomended_pid_output(&data);
		compute_control_command_u(delta_t, &data);
	}
	else
	{
		compute_errors(current_curvature, desired_curvature, delta_t, &data);
		compute_external_reinforcement_signal(&data);
		compute_critic_value(&data);
		compute_recomended_pid_output(&data);
		compute_control_command_u(delta_t, &data);

		compute_td_error(&data);
	}
	printf("u%lf e %f  kp %f ki %f  kd %f\n", data.variables.U[0], data.variables.error[0], data.variables.pid_params[0], data.variables.pid_params[1], data.variables.pid_params[2]);

	return data.variables.U[0];//carmen_clamp(-100.0, (U[0]), 100.0);
}

