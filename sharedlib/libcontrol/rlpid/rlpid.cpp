#include "rlpid.h"

//   READ PARAMETERS
intelligent_control_params
read_parameters(const char* parameters) { //Read the initial values of the parameters from a file
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


//void
//carmen_librlpid_exporta_pesos()
//{
//	FILE *output;
//	output = fopen("weigths.txt", "w");
//	int i = 0;
//
//	for (i = 0; i < neural_network_size; i++)
//	{
//		fprintf(output, "%lf ->w_weight[0]\n%lf ->w_weight[1]\n%lf ->w_weight[2]\n", network[i].w_neuron_weight[0], network[i].w_neuron_weight[1], network[i].w_neuron_weight[2]);
//		fprintf(output, "%lf ->v_weight\n", network[i].v_neuron_weight);
//		fprintf(output, "%lf ->center_vector[0]\n%lf ->center_vector[1]\n%lf ->center_vector[2]\n", network[i].center_vector[0], network[i].center_vector[1], network[i].center_vector[2]);
//		fprintf(output, "%lf ->width_scalar_sigma\n", network[i].width_scalar_sigma);
//		fprintf(output, "%lf ->phi_value\n", network[i].phi_value);
//	}
//	fflush(output);
//	fclose(output);
//}
//
//void
//carmen_librlpid_importa_pesos(rl_data* data)
//{
//	FILE* file = fopen("weigths.txt", "r");
//	char trash[50];
//	int i = 0;
//
//	if(file != NULL){
//		for (i = 0; i < neural_network_size; i++)
//		{
//			fscanf(file, "%lf", &data->network[i].w_neuron_weight[0]);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].w_neuron_weight[1]);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].w_neuron_weight[2]);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].v_neuron_weight);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].center_vector[0]);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].center_vector[1]);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].center_vector[2]);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].width_scalar_sigma);
//			fscanf(file, "%s", trash);
//			fscanf(file, "%lf", &data->network[i].phi_value);
//			fscanf(file, "%s", trash);
//		}
//	}
//}


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
calculate_pid_errors(double y_desired, double y,  double delta_t, rl_data* data)
{
// Update error values

	double 	error_t_1 = data->variables.error_order[0];

	data->variables.error_order[0] = (y_desired - y);
	data->variables.error_order[1] = data->variables.error_order[1] + data->variables.error_order[0]* delta_t;
	data->variables.error_order[2] = (data->variables.error_order[0] - error_t_1) / delta_t;
}


double
calculate_total_error(rl_data* data, int media)
{
	data->total_error_quadratico = data->total_error_quadratico + (data->variables.error[0]*data->variables.error[0]);
	return (data->total_error_quadratico/media);
}

//   EQUATION 1
void
update_plant_input_u(double atan_desired_curvature, double atan_current_curvature, double delta_t, rl_data* data)
{
	data->variables.U[2] = data->variables.U[1]; //Update the past u values
	data->variables.U[1] = data->variables.U[0];

	double 	integral_t_1 = data->variables.error_order[1];
	double 		u_t;			// u(t)	-> actuation in time t


	if (delta_t == 0.0)
	{
		data->variables.U[0] = 0;
		return;
	}

	calculate_pid_errors(atan_desired_curvature, atan_current_curvature, delta_t, data);

	u_t = (data->variables.pid_params[0] * data->variables.error_order[0]) + (data->variables.pid_params[1] * data->variables.error_order[1]) + (data->variables.pid_params[2] * data->variables.error_order[2]);

	data->variables.error[2] = data->variables.error[1];
	data->variables.error[1] = data->variables.error[0];
	data->variables.error[0] = data->variables.error_order[0];

	// Anti windup
	if ((u_t < -100.0) || (u_t > 100.0))
		data->variables.error_order[1] = integral_t_1;

	u_t = carmen_clamp(-100.0, u_t, 100.0);
	data->variables.U[0] = -u_t;//carmen_libpid_steering_PID_controler(atan_desired_curvature, atan_current_curvature, delta_t);
}

void
update_plant_input_u_old(rl_data* data)
{ //Where:  U[0] = u(t), U[1] = u(t-1), U[2] = u(t-2)
	data->variables.U[2] = data->variables.U[1]; //Update the past u values
	data->variables.U[1] = data->variables.U[0];
	data->variables.pid_params[1] = 600;
	data->variables.pid_params[0] = 1250;
	data->variables.pid_params[2] = 25;
	data->variables.U[0] = data->variables.U[1] + data->variables.pid_params[1] * data->variables.error_order[0] + data->variables.pid_params[0] * data->variables.error_order[1] + data->variables.pid_params[2] * data->variables.error_order[2]; //Calculate actual output value.
}
// where
void
calculate_error_old(double y_desired, double y, rl_data* data)
{
// Update error values //
	data->variables.error[2] = data->variables.error[1];
	data->variables.error[1] = data->variables.error[0];
	data->variables.error[0] = (y_desired - y);
// Update error order //
//	error_order[2] = error[0] - (2 * error[1]) + error[2]; //Delta^2(e(t)) = e(t) - 2*e(t-1) + e(t-2)
//	error_order[1] = error[0] - error[1]; //Delta(e(t)) = e(t) - e(t-1)
//	error_order[0] = error[0];
}


//   EQUATION 2
void
external_reinforcement_signal(rl_data* data) //r(t) = alfa*Re + beta*Rec
{
	double Re = 0;
	double Rec = 0;
// Calculate Re
	if (fabs(data->variables.error[0]) > data->params.error_band)
	{
		Re = -0.5;
	}
// Calculate Rec
	if (fabs(data->variables.error[0]) > fabs(data->variables.error[1]))
	{
		Rec = -0.5;
	}
	data->reinforcement_signal = (data->params.alfa_weight_coef*Re) + (data->params.beta_weight_coef*Rec); //r(t+1)
}


// EQUATION 3
// UPDATE A SINGLE NEURON
double
update_neuron_hidden_unit_phi(double width_scalar, double* center_vector, rl_data* data)
{
	double neuron_hidden_phi = pow((data->variables.error_order[0] - center_vector[0]),2) + pow((data->variables.error_order[1] - center_vector[1]),2) + pow((data->variables.error_order[2] - center_vector[2]),2);
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


// EQUATION 4
void
update_recomended_pid_output(rl_data* data)
{
	int i = 0;
// RESET THE VALUE OF THE RECOMENDED PID
	for (i = 0;i < 3;i++)
	{
		data->variables.recomended_pid_params[i] = 0;
	}
// FINDING THE RECOMMENDED KP
	for (i = 0; i < neural_network_size; i++)
	{
		data->variables.recomended_pid_params[0] = data->variables.recomended_pid_params[0] + (data->network[i].w_neuron_weight[0] * data->network[i].phi_value);
		//printf("\nKp w wheight = %lf\n", network[i].w_neuron_weight[1]);
	}
// FINDING THE RECOMMENDED KI
	for (i = 0; i < neural_network_size; i++)
	{
		data->variables.recomended_pid_params[1] = data->variables.recomended_pid_params[1] + (data->network[i].w_neuron_weight[1] * data->network[i].phi_value);
	}
// FINDING THE RECOMMENDED KD
	for (i = 0; i < neural_network_size; i++)
	{
		data->variables.recomended_pid_params[2] = data->variables.recomended_pid_params[2] + (data->network[i].w_neuron_weight[2] * data->network[i].phi_value);
	}
}


// EQUATION 5
double
update_critic_value_output(rl_data* data)
{
	int i = 0;
	double c_value = 0;
// FINDING THE CRITIC VALUE OUTPUT
	for (i = 0; i < neural_network_size; i++)
	{
		c_value = c_value + (data->network[i].v_neuron_weight * data->network[i].phi_value);
	}
	return c_value;
}


double
update_critic_value_future(rl_data* data)
{
	int i = 0;
	double c_value = 0;
// FINDING THE CRITIC VALUE OUTPUT
	for (i = 0; i < neural_network_size; i++)
	{
		c_value = c_value + (data->network[i].v_neuron_weight * data->network[i].phi_future);
	}
	return c_value;
}


// EQUATION 6
// GAUSSIAN FUNCTION
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
// EQUATION 7
void
calculate_td_error(rl_data* data)
{
	data->td_error = data->reinforcement_signal/*[1]*/ + (data->params.discount_factor*data->future_critic_value) - data->variables.critic_value; // 0 < discount_factor < 1
}
// EQUATION 8
double
performance_index_of_system_learning(rl_data* data)
{
	return (pow(data->td_error,2)/2);
}
// EQUATION 9 AND 10
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
// EQUATION 11
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
// EQUATION 12
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
// SAVE AND LOAD VARIABLES
void
store_variables(rl_data* data)
{
	int i =0;

	data->pv.sigma_critical_deviation = data->variables.sigma_critical_deviation;
	data->pv.critic_value = data->variables.critic_value;
	for(i = 0; i<3; i++)
	{
		data->pv.U[i] = data->variables.U[i];
		data->pv.error[i] = data->variables.error[i];
		data->pv.error_order[i] = data->variables.error_order[i];
		data->pv.pid_params[i] = data->variables.pid_params[i];
		data->pv.recomended_pid_params[i] = data->variables.recomended_pid_params[i];
	}
}

void
load_variables(rl_data* data)
{
	int i = 0;

	data->variables.sigma_critical_deviation = data->pv.sigma_critical_deviation;
	data->variables.critic_value = data->pv.critic_value;

	for (i = 0; i < 3; i++)
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
	static rl_data data;

	if(first_time)
	{
		data.params = read_parameters("rlpid_params.txt");
		data.pv = initializate_variables(&data); // Step 1
		first_time = false;
	}else // So começa a rodar a partir da segunda iteração
	{

	calculate_error_old(desired_curvature, current_curvature, &data); // Step 6 ==> CALCULA ERRO
	update_neetwork_hidden_unit_phi_future(&data);// ==> UPDATE PHI
	data.future_critic_value = update_critic_value_future(&data); //Step 7 ==> UPDATE V
	calculate_td_error(&data); //Step 8 ==> CALCULA ERRO TD
	load_variables(&data);
	weights_update(&data); //Step 9 ==> UPDATE PESOS
	center_vector_update(&data); //Step 10 ==> UPDATE CENTRO
	width_scalar_update(&data); //Step 10 ==> UPDATE WIDTH SCALAR
	}

	//imprime_os_pesos_agora++;
	load_variables(&data);
	calculate_error_old(desired_curvature, current_curvature, &data); // Step 2 ==> CALCULA ERRO
	external_reinforcement_signal(&data); //Step 3 ==> RECOMPENSA
	update_neetwork_hidden_unit_phi(&data);// ==> UPDATE PHI
	update_recomended_pid_output(&data); //Step 4 ==> UPDATE K`
	data.variables.critic_value = update_critic_value_output(&data);	//Step 4 ==> UPDATE V

	update_pid_params(&data); //Step 5 ==> UPDATE K
	update_plant_input_u(current_curvature, desired_curvature, delta_t, &data); //Step 5 ==> UPDATE U
	store_variables(&data);

	printf("u%lf e %f  kp %f ki %f  kd %f\n", data.variables.U[0], data.variables.error[0], data.variables.pid_params[0], data.variables.pid_params[1], data.variables.pid_params[2]);

	return data.variables.U[0];//carmen_clamp(-100.0, (U[0]), 100.0);
}
