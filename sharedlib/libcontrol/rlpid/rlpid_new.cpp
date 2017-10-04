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
	for (i = 0; i < NEURONS_NUMBER_HIDDEN_UNIT; i++)
	{
		data->hidden_neuron[i].w_neuron_weight[0] = /*2.55*/5.1 + random_double() * FF_MULTIPLIER;//W_kp
		data->hidden_neuron[i].w_neuron_weight[1] = /*1.2*/2.5 + random_double() * FF_MULTIPLIER;
		data->hidden_neuron[i].w_neuron_weight[2] = -0.025 + random_double() * FF_MULTIPLIER;
		data->hidden_neuron[i].v_neuron_weight    = random_double() * FF_MULTIPLIER;
		data->hidden_neuron[i].center_vector[0]   = random_double() * RBF_MULTIPLIER;
		data->hidden_neuron[i].center_vector[1]   = random_double() * RBF_MULTIPLIER;
		data->hidden_neuron[i].center_vector[2]   = random_double() * RBF_MULTIPLIER;
		data->hidden_neuron[i].width_scalar_sigma = random_double() * RBF_MULTIPLIER;
		data->hidden_neuron[i].phi_value = 1; //random_double() * RBF_MULTIPLIER;
	}
}


void
compute_errors(double y_actual, double y_desired, double delta_t, data* data)
{
	double error = (y_desired - y_actual);

	data->previous_error = data->proportional_error;
	data->proportional_error = error;
	data->integral_error = data->integral_error + (error * delta_t);
	data->derivative_error = (error - data->previous_error) / delta_t;
}


//   EQUATION 1 eq1
void
compute_control_command_u(data* data)
{
	double u_t = (data->actual_kp * data->proportional_error) + (data->actual_ki * data->integral_error) +
			(data->actual_kd * data->derivative_error);

	u_t = carmen_clamp(-100.0, u_t, 100.0);

	data->u_t = -u_t;
}


//   EQUATION 2 eq2 Reward
void
compute_external_reinforcement_signal(data* data)     //r(t) = alfa*Re + beta*Rec
{
	double Re = 0.0;
	double Rec = 0.0;

	if (fabs(data->proportional_error) > data->params.error_band)
	{
		Re = -0.5;
	}

	if (fabs(data->proportional_error) > fabs(data->previous_error))
	{
		Rec = -0.5;
	}

 //data->previous_reinforcement_signal = data->reinforcement_signal;
	data->reinforcement_signal = ((data->params.alfa_weight_coef * Re) + (data->params.beta_weight_coef * Rec));
}


// EQUATION 3 eq3 UPDATE A SINGLE NEURON
double
update_neuron_hidden_unit_phi(double width_scalar, double* center_vector, data* data)
{
	double neuron_hidden_phi = pow((data->proportional_error - center_vector[0]),2) + pow((data->integral_error - center_vector[1]),2) + pow((data->derivative_error - center_vector[2]),2);

	neuron_hidden_phi = -neuron_hidden_phi / (2 * width_scalar * width_scalar);
	neuron_hidden_phi = exp(neuron_hidden_phi);
	return neuron_hidden_phi;
}


void
update_neetwork_hidden_unit_phi(data* data)
{
	int i = 0;
	for (i = 0; i < NEURONS_NUMBER_HIDDEN_UNIT;i++)
	{
		data->hidden_neuron[i].phi_value = update_neuron_hidden_unit_phi(data->hidden_neuron[i].width_scalar_sigma, data->hidden_neuron[i].center_vector, data);
	}
}


// EQUATION 4 eq4
void
compute_recomended_pid_output(data* data)
{
	int i = 0;

	data->recomended_kp = 0.0;
	data->recomended_ki = 0.0;
	data->recomended_kd = 0.0;

	for (i = 0; i < NEURONS_NUMBER_HIDDEN_UNIT; i++)     // TODO juntar esses 3 loops
	{
		data->recomended_kp += data->hidden_neuron[i].w_neuron_weight[0] * data->hidden_neuron[i].phi_value;
		//printf("w %lf p %lf\n", data->hidden_neuron[i].w_neuron_weight[0], data->hidden_neuron[i].phi_value);

		data->recomended_ki += data->hidden_neuron[i].w_neuron_weight[1] * data->hidden_neuron[i].phi_value;

		data->recomended_kd += data->hidden_neuron[i].w_neuron_weight[2] * data->hidden_neuron[i].phi_value;
	}

	//printf("kp %lf ki %lf  kd %lf\n", data->recomended_kp, data->recomended_ki, data->recomended_kd);
}


// EQUATION 5 eq5 FINDING THE CRITIC VALUE OUTPUT
void
compute_critic_value(data* data)
{
	double c_value = 0.0;

	for (int i = 0; i < NEURONS_NUMBER_HIDDEN_UNIT; i++)
	{
		c_value += (data->hidden_neuron[i].v_neuron_weight * data->hidden_neuron[i].phi_value);
	}

	data->previous_critic_value = data->critic_value;
	data->critic_value = c_value;
}


double
gaussian_noise(int mean, double sigma)
{
	const double norm = 1.0 / (RAND_MAX + 1.0); // RAND_MAX is the max value that the rand() function can give.
	double u = 1.0 - rand() * norm; // can't let u == 0 //  0 < rand_value <=1

	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * PI * v);
	return (mean + sigma * z);
}


// EQUATION 6 eq6
void
compute_actual_pid_params(data* data)
{
	double g_noise = 0.0;

	data->sigma_critical_deviation = 1 / (1 + exp(2 * data->critic_value));

	g_noise = gaussian_noise(0, data->sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	data->actual_kp = data->recomended_kp + g_noise;
	//printf("%lf ", g_noise);

	g_noise = gaussian_noise(0, data->sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	data->actual_ki = data->recomended_ki + g_noise;
	//printf("%lf ", g_noise);

	g_noise = gaussian_noise(0, data->sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	data->actual_kd = data->recomended_kd + g_noise;
	//printf("%lf\n", g_noise);

	//printf("akp %lf aki %lf akd %lf\n", data->actual_kp, data->actual_ki, data->actual_kd);
}


// EQUATION 7 eq7
void
compute_td_error(data* data)    // 0 < discount_factor < 1
{
	data->td_error = data->reinforcement_signal + (data->params.discount_factor * data->critic_value) - data->previous_critic_value; // 0 < discount_factor < 1
	//printf("%lf %lf\n", data->critic_value, data->previous_critic_value);
}


// EQUATION 9 AND 10 eq9 eq10
void
weights_update(data* data)
{
	int i = 0;
	double aux = 0;

	for (i = 0; i < NEURONS_NUMBER_HIDDEN_UNIT; i++)
	{
		aux = (data->params.actor_learning_rate * data->td_error * data->hidden_neuron[i].phi_value) / data->sigma_critical_deviation;

		data->hidden_neuron[i].w_neuron_weight[0] = data->hidden_neuron[i].w_neuron_weight[0] + (aux * (data->actual_kp - data->recomended_kp));
		data->hidden_neuron[i].w_neuron_weight[1] = data->hidden_neuron[i].w_neuron_weight[1] + (aux * (data->actual_ki - data->recomended_ki));
		data->hidden_neuron[i].w_neuron_weight[2] = data->hidden_neuron[i].w_neuron_weight[2] + (aux * (data->actual_kd - data->recomended_kd));

		data->hidden_neuron[i].v_neuron_weight = data->hidden_neuron[i].v_neuron_weight + data->params.critic_learning_rate * data->td_error * data->hidden_neuron[i].phi_value;
	}
}


// EQUATION 11 eq11
void
center_vector_update(data* data)
{
	double aux = 0.0;

	for (int i = 0; i < NEURONS_NUMBER_HIDDEN_UNIT; i++)
	{
		aux = (data->params.learning_rate_center * data->td_error * data->hidden_neuron[i].v_neuron_weight * data->hidden_neuron[i].phi_value) / pow(data->hidden_neuron[i].width_scalar_sigma, 2);

		data->hidden_neuron[i].center_vector[0] = data->hidden_neuron[i].center_vector[0] + aux * (data->proportional_error - data->hidden_neuron[i].center_vector[0]);
		data->hidden_neuron[i].center_vector[1] = data->hidden_neuron[i].center_vector[1] + aux * (data->integral_error     - data->hidden_neuron[i].center_vector[1]);
		data->hidden_neuron[i].center_vector[2] = data->hidden_neuron[i].center_vector[2] + aux * (data->derivative_error   - data->hidden_neuron[i].center_vector[2]);
	}
}


// EQUATION 12 eq12
void
width_scalar_update(data* data)
{
	double aux = 0.0, aux2 = 0.0;

	for (int i = 0; i < NEURONS_NUMBER_HIDDEN_UNIT; i++)
	{
		aux  = (data->params.learning_rate_width * data->td_error * data->hidden_neuron[i].v_neuron_weight * data->hidden_neuron[i].phi_value) / pow(data->hidden_neuron[i].width_scalar_sigma, 3);
		aux2 =  pow(data->proportional_error - data->hidden_neuron[i].center_vector[0], 2) +
				pow(data->integral_error - data->hidden_neuron[i].center_vector[1], 2) +
				pow(data->derivative_error - data->hidden_neuron[i].center_vector[2], 2);

		data->hidden_neuron[i].width_scalar_sigma = data->hidden_neuron[i].width_scalar_sigma + aux * aux2; // Width Scalar update here
	}
}


double
carmen_librlpid_compute_effort_new(double current_curvature, double desired_curvature, double delta_t)
{
	static bool first_time = true;
	static data data;

	if(first_time)
	{
		first_time = false;
		data.params = read_parameters("rlpid_params.txt");
		initializate_variables(&data);

		compute_errors(current_curvature, desired_curvature, delta_t, &data);
		compute_external_reinforcement_signal(&data);
		update_neetwork_hidden_unit_phi(&data);
		compute_recomended_pid_output(&data);
		compute_critic_value(&data);
		compute_actual_pid_params(&data);
		compute_control_command_u(&data);
	}
	else
	{
		compute_critic_value(&data);
		compute_td_error(&data);

		weights_update(&data);
		center_vector_update(&data);
		width_scalar_update(&data);

		compute_errors(current_curvature, desired_curvature, delta_t, &data);
		compute_external_reinforcement_signal(&data);
		update_neetwork_hidden_unit_phi(&data);
		compute_recomended_pid_output(&data);
		compute_actual_pid_params(&data);
		compute_control_command_u(&data);
	}

	printf("u %lf e %lf kp %lf ki %lf kd %lf\n", data.u_t, data.proportional_error, data.actual_kp, data.actual_ki, data.actual_kd);

	return data.u_t;
}
