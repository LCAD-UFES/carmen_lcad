#include <carmen/carmen.h>
#include <fann.h>
#include <fann_train.h>
#include <fann_data.h>
#include <floatfann.h>
#include <pthread.h>
#include <car_neural_model.h>
#include "rlpid.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////   GLOBAL VARIABLES   /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double td_error;
double sigma_critical_deviation;
double critic_value; //V(t)
double future_critic_value; //V(t+1)
double reinforcement_signal;//[2]; // r(t)
double U[3]; //The size of the vector is 3, because "u" uses U(t-2) in equation, this means that you need store the past value of U
double error[3]; //You need the last 3 values of e =>t(t), e(t-1) and e(t-2)
double error_order[3]; //e(t), Delta(e) and Delta^2(e)
double pid_params[3]; //K ->The parameters Kp,Ki and Kd respectvely
double recomended_pid_params[3]; //K' ->The new recomended params of Kp,Ki and Kd respectvely
double best_pid[3];
rbf_neuron network[neural_network_size]; //The size is defined at .h file


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////   READ PARAMETERS   //////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
intelligent_control_params read_parameters(const char* parameters) { //Read the initial values of the parameters from a file
	FILE* file = fopen(parameters, "r");
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

#define RBF_MULTIPLIER 1.0
#define FF_MULTIPLIER 0.05
#define GAUSS_MULTIPLIER 0.05

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////   INITIALIZATION   //////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
past_variables
initializate_variables(past_variables pv)
{
	//past_variables pv;
	int i = 0;
////////////////////////////// INITIALIZE GLOBAL VARIABLES //////////////////////////////
	td_error = 0;
	critic_value = 0;
	reinforcement_signal = 0;
	sigma_critical_deviation = 0;

	for (i = 0; i < 3;i++)
	{
		U[i] = 0;
		error[i] = 0;
		error_order[i] = 0;
		recomended_pid_params[i] = 0;
	}
	pid_params[0] = 1250; //0.12;
	pid_params[1] = 600; //0.32;
	pid_params[2] = 25; //0.08;
/////////////////////////// INITIALIZE ESTRUCT VARIABLES //////////////////////////////
	pv.past_td_error = 0;
	pv.past_critic_value = 0;
	pv.past_reinforcement_signal = 0; //r(t+1)
	pv.past_sigma_critical_deviation = 0;

	for (i = 0; i < 3;i++)
	{
		pv.past_U[i] = 0;
		pv.past_error[i] = 0;
		pv.past_error_order[i] = 0;
		pv.past_recomended_pid_params[i] = 0;
	}
	pv.past_pid_params[0] = 0.12;
	pv.past_pid_params[1] = 0.32;
	pv.past_pid_params[2] = 0.08;
///////////////////////////////// INITIALIZE NETWORK //////////////////////////////////
	for (i = 0; i < neural_network_size; i++)
	{
		network[i].w_neuron_weight[0] = random_double() * FF_MULTIPLIER;
		network[i].w_neuron_weight[1] = random_double() * FF_MULTIPLIER;
		network[i].w_neuron_weight[2] = random_double() * FF_MULTIPLIER;
		network[i].v_neuron_weight = random_double() * FF_MULTIPLIER;
		network[i].center_vector[0] = random_double() * RBF_MULTIPLIER;
		network[i].center_vector[1] = random_double() * RBF_MULTIPLIER;
		network[i].center_vector[2] = random_double() * RBF_MULTIPLIER;
		network[i].width_scalar_sigma = random_double() * RBF_MULTIPLIER;
		network[i].phi_value = random_double() * RBF_MULTIPLIER;
	}

	return pv;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 1   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
update_plant_input_u()
{ //Where:  U[0] = u(t), U[1] = u(t-1), U[2] = u(t-2)
	U[2] = U[1]; //Update the past u values
	U[1] = U[0];
	U[0] = U[1] + pid_params[1] * error_order[0] + pid_params[0] * error_order[1] + pid_params[2] * error_order[2]; //Calculate actual output value.
	//printf("\nkp = %lf, ki = %lf, kd = %lf, error_o1 = %lf, error_o2 = %lf, error_o3 = %lf\n",pid_params[1], pid_params[0], pid_params[2], error_order[0], error_order[1], error_order[2]);
	//U(t) = U(t-1) + Ki*e(t) + Kp*Delta(e(t)) + Kd*Delta^2(e(t))
}
//////////////////////////////////////////////////////////// where ////////////////////////////////////////////////////////////////////////////
void
calculate_error(double y_desired, double y)
{
/////////// Update error values ///////////
	error[2] = error[1];
	error[1] = error[0];
	error[0] = y_desired - y;
/////////// Update error order ///////////
	error_order[2] = error[0] - (2 * error[1]) + error[2]; //Delta^2(e(t)) = e(t) - 2*e(t-1) + e(t-2)
	error_order[1] = error[0] - error[1]; //Delta(e(t)) = e(t) - e(t-1)
	error_order[0] = error[0];
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 2   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
external_reinforcement_signal(double weighted_alfa, double weighted_beta, double error_band) //r(t) = alfa*Re + beta*Rec
{
	double Re = 0;
	double Rec = 0;
/////////// Calculate Re ////////////
	if (fabs(error[0]) > error_band)
	{
		Re = -0.5;
	}
/////////// Calculate Rec ///////////
	if (fabs(error[0]) > fabs(error[1]))
	{
		Rec = -0.5;
	}
	//reinforcement_signal[1] = reinforcement_signal[0]; //r(t)
	//reinforcement_signal[0] = (weighted_alfa*Re) + (weighted_beta*Rec); //r(t+1)
	reinforcement_signal = (weighted_alfa*Re) + (weighted_beta*Rec); //r(t+1)
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 3   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////  UPDATE A SINGLE NEURON  ///////////////////////////////////////////////////
double
update_neuron_hidden_unit_phi(double width_scalar, double* center_vector)
{
	double neuron_hidden_phi = pow((error_order[0] - center_vector[0]),2) + pow((error_order[1] - center_vector[1]),2) + pow((error_order[2] - center_vector[2]),2);
	neuron_hidden_phi = -neuron_hidden_phi / (2 * width_scalar * width_scalar);
	neuron_hidden_phi = exp(neuron_hidden_phi);
	return neuron_hidden_phi;
}
/////////////////////////////////////////  UPDATE ALL NEURAL NETWORK  ///////////////////////////////////////////////////
void
update_neetwork_hidden_unit_phi()
{
	int i = 0;
	for (i = 0; i < neural_network_size;i++)
	{
		network[i].phi_value = update_neuron_hidden_unit_phi(network[i].width_scalar_sigma, network[i].center_vector);
		//printf("\nneuron %d, phi_v = %lf\n", i, network[i].phi_value);
	}
}
void
update_neetwork_hidden_unit_phi_future()
{
	int i = 0;
	for (i = 0; i < neural_network_size;i++)
	{
		network[i].phi_future = update_neuron_hidden_unit_phi(network[i].width_scalar_sigma, network[i].center_vector);
		//printf("\nneuron %d, phi_v = %lf\n", i, network[i].phi_value);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 4   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
update_recomended_pid_output()
{
	int i = 0;
///////////   RESET THE VALUE OF THE RECOMENDED PID   /////////////
	for (i = 0;i < 3;i++)
	{
		recomended_pid_params[i] = 0;
	}
/////////////////   FINDING THE RECOMMENDED KP   /////////////////
	for (i = 0; i < neural_network_size; i++)
	{
		recomended_pid_params[0] = recomended_pid_params[0] + (network[i].w_neuron_weight[1] * network[i].phi_value);
		//printf("\nKp w wheight = %lf\n", network[i].w_neuron_weight[1]);
	}
/////////////////   FINDING THE RECOMMENDED KI   /////////////////
	for (i = 0; i < neural_network_size; i++)
	{
		recomended_pid_params[1] = recomended_pid_params[1] + (network[i].w_neuron_weight[0] * network[i].phi_value);
	}
/////////////////   FINDING THE RECOMMENDED KD   /////////////////
	for (i = 0; i < neural_network_size; i++)
	{
		recomended_pid_params[2] = recomended_pid_params[2] + (network[i].w_neuron_weight[2] * network[i].phi_value);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 5   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double
update_critic_value_output()
{
	int i = 0;
	double c_value = 0;
//////////////   FINDING THE CRITIC VALUE OUTPUT   ///////////////
	for (i = 0; i < neural_network_size; i++)
	{
		c_value = c_value + (network[i].v_neuron_weight * network[i].phi_value);
	}
	return c_value;
}
double
update_critic_value_future()
{
	int i = 0;
	double c_value = 0;
//////////////   FINDING THE CRITIC VALUE OUTPUT   ///////////////
	for (i = 0; i < neural_network_size; i++)
	{
		c_value = c_value + (network[i].v_neuron_weight * network[i].phi_future);
	}
	return c_value;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 6   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                          //////
//////////////////////////////////////////////////////// GAUSSIAN FUNCTION ///////////////////////////////////////////////////////
double
gaussian_noise(int mean, double sigma)
{
	const double norm = 1.0 / (RAND_MAX + 1.0); // RAND_MAX is the max value that the rand() function can give.
	double u = 1.0 - rand() * norm; // can't let u == 0 //  0 < rand_value <=1

	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * PI * v);
	return (mean + sigma * z);
}
//////////////////////////////////////////////////////////// EQUATION ////////////////////////////////////////////////////////////
void
update_pid_params()
{
	sigma_critical_deviation = 1 / (1 + exp(2 * critic_value));
	//printf("\nsigma_c_d = %lf\n",sigma_critical_deviation);
	double g_noise = gaussian_noise(0, sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	pid_params[0] = recomended_pid_params[0] + g_noise;
	g_noise = gaussian_noise(0, sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	pid_params[1] = recomended_pid_params[1] + g_noise;
	g_noise = gaussian_noise(0, sigma_critical_deviation) * GAUSS_MULTIPLIER; //mean = 0// zero-mean and magntude = sigma_critical_deviation
	pid_params[2] = recomended_pid_params[2] + g_noise;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 7   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
calculate_td_error(double discount_factor)
{
	td_error = reinforcement_signal/*[1]*/ + (discount_factor*future_critic_value) - critic_value; // 0 < discount_factor < 1
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 8   ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double
performance_index_of_system_learning()
{
	return (pow(td_error,2)/2);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////   EQUATION 9 AND 10   /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
weights_update(double l_rate_a, double l_rate_c)
{
	int i = 0;
	double aux = 0;
	for (i = 0;i < neural_network_size;i++)
	{
		// @FILIPE USAR O PHI_VALUE DO TEMPO T
//////////////////////   W WEIGHTS UPDATE   //////////////////////   EQUATION 9   //////////////////////////////////////////
		aux = /*fabs(*/(l_rate_a * td_error * network[i].phi_value) / sigma_critical_deviation/*network[i].width_scalar_sigma*/;//);
		//printf("\nk - k' = %lf\nEtd = %lf", (pid_params[0] - recomended_pid_params[0]), td_error);
		network[i].w_neuron_weight[0] = network[i].w_neuron_weight[0] + (aux*(pid_params[1] - recomended_pid_params[1])); //Ki
		//printf("\nw update before = %lf\n", network[i].w_neuron_weight[1]);
		network[i].w_neuron_weight[1] = network[i].w_neuron_weight[1] + (aux*(pid_params[0] - recomended_pid_params[0])); //Kp
		//printf("\nw update after = %lf\n", network[i].w_neuron_weight[1]);
		network[i].w_neuron_weight[2] = network[i].w_neuron_weight[2] + (aux*(pid_params[2] - recomended_pid_params[2])); //Kd
//////////////////////   V WEIGHT UPDATE   ///////////////////////   EQUATION 10  //////////////////////////////////////////
		aux = l_rate_c * td_error * network[i].phi_value;
		network[i].v_neuron_weight = network[i].v_neuron_weight + aux;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 11   ///////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
center_vector_update(double learning_rate_center)
{
	int i = 0;
	double aux = 0;
	for (i = 0;i < neural_network_size;i++)
	{
		// @FILIPE USAR O ERROR_ORDER DO TEMPO T
		aux = (learning_rate_center * td_error * network[i].v_neuron_weight * network[i].phi_value) / pow(network[i].width_scalar_sigma,2);
		network[i].center_vector[0] = network[i].center_vector[0] + aux * (error_order[0] - network[i].center_vector[0]);//Ki
		network[i].center_vector[1] = network[i].center_vector[1] + aux * (error_order[1] - network[i].center_vector[1]);//Kp
		network[i].center_vector[2] = network[i].center_vector[2] + aux * (error_order[2] - network[i].center_vector[2]);//Kd
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////   EQUATION 12   ///////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
width_scalar_update(double learning_rate_width)
{
	int i = 0;
	double aux = 0;
	double aux2 = 0;
	for (i = 0;i < neural_network_size;i++)
	{
		// @FILIPE USAR O PHI_VALUE DO TEMPO T
		aux = (learning_rate_width * td_error * network[i].v_neuron_weight * network[i].phi_value) / pow(network[i].width_scalar_sigma, 3);
		aux2 = pow(error_order[0] - network[i].center_vector[0], 2) + pow(error_order[1] - network[i].center_vector[1], 2) + pow(error_order[2] - network[i].center_vector[2], 2);
		network[i].width_scalar_sigma = network[i].width_scalar_sigma + aux * aux2; // Width Scalar update here
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////   EQUATION 13   //////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double
update_output_y(double last_y, int t)
{
	double y = 0;
	double a = t;
	//double aux = -0.01*(t - 1500);
	//double a = 1.2*(1 - (0.8*exp(aux)));
	//double a = 1 + (t % 1500) * 0.0005;
	a = 1;
	//printf("eq 13 U[0] = %lf, U[1] = %lf\n", U[0], U[1]);
	y = ((a * last_y * U[0]) / (1 + pow(last_y, 2))) + U[1];
	return y;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////   DESIRED SIGNAL   /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////// SIN SIGNAL ////////////////////////////
double
sin_signal(int t, double frequence)
{
	double angle;
	angle = (2 * PI * frequence * t);
	return sin(angle);
}
/////////////////////////// SQUARE SIGNAL ///////////////////////////
double
square_signal(int t,int interval)//interval será o intervalo no qual o degrau se dará
{
	int mod = (t%(interval*2));
	if (mod >= interval)
	{
		return 1;
	}else
	{
		return 0;
	}
}
////////////////////////// TRIANGLE SIGNAL //////////////////////////
double
triangle_signal(int t,int interval)//interval é o intervalo no qual o triangulo se dara
{
	double result = 0;
	int mod = (t%(interval*2));
	double inter = interval;

	if (mod <= interval)
	{//subindo
		result = (mod/inter);
		//printf("mod = %d, val = %lf ",mod,result);
	}else
	{//descendo
		result = ((500 - mod)/inter) + 1;
	}
	return result;
}
////////////////////////// DESIRED SIGNAL //////////////////////////
double
calculate_disired_test_signal(int t)
{
	double result = 0;


	if(t<3000){
		result = sin_signal(t, 0.05);            //frequence = (1/40)update_control_output_pid
	}else{
		if(t<6500){
			result = square_signal(t,500); //sinal quadrangular
		}else{
			if(t>8000){
				result = triangle_signal(t,500);//sinal triangular
			}else{
				result = sin_signal(t, 0.001);  //sinal senoidal
			}
		}
	}
	return result;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////   SAVE AND LOAD VARIABLES //////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
past_variables
save_variables(past_variables pv)
{
	int i =0;

	pv.past_sigma_critical_deviation = sigma_critical_deviation;
	pv.past_critic_value = critic_value;
	for(i = 0; i<3; i++)
	{
		pv.past_U[i] = U[i];
		pv.past_error[i] = error[i];
		pv.past_error_order[i] = error_order[i];
		pv.past_pid_params[i] = pid_params[i];
		pv.past_recomended_pid_params[i] = recomended_pid_params[i];
	}
	return pv;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////   PRINT VARIABLES   ////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
print_variables()
{
	printf("\n\ntd_error = %lf, sigma_critical_deviation = %lf\ncritic_value (V) = %lf, reinforcement_signal = %lf\nU[0] = %lf, U[1] = %lf\nerror[0] = %lf, error_order[1] = %lf, error_order[2] = %lf\npid_params[0] = %lf, pid_params[1] = %lf, pid_params[2] = %lf\nrecomended_pid_params[0] = %lf, recomended_pid_params[1] = %lf, recomended_pid_params[2] = %lf\n\n",td_error, sigma_critical_deviation, critic_value, reinforcement_signal, U[0], U[1], error[0], error_order[1], error_order[2], pid_params[0], pid_params[1], pid_params[2], recomended_pid_params[0], recomended_pid_params[1], recomended_pid_params[2]);
}

void
plota_graficos(int opcao, int largura_janela, int altura_janela, int intervalox, int intervaloy)// o intervalo y vai de -y até +y
{
	FILE *gnuplot;

	gnuplot= popen("gnuplot -persistent", "w");
	fprintf(gnuplot, "set terminal wxt size %d,%d\n",largura_janela, altura_janela);
	fprintf(gnuplot, "set yrange [%d:%d]\n", -intervaloy, intervaloy);
	fprintf(gnuplot, "set xrange [0:%d]\n", intervalox);

	switch(opcao)
	{
	case 1:
		fprintf(gnuplot, "plot './b.txt' using  6 title \"y_desejado\"with lines,  \
	     '' using 3 title \"y_obtido\" with lines\n");
		break;
	case 2:
		fprintf(gnuplot, "set multiplot layout 1,2 rowsfirst\n");
		fprintf(gnuplot, "plot './b.txt' using 9 title \"erro\" with lines\n");
		fprintf(gnuplot, "plot './b.txt' using  6 title \"y_desejado\"with lines,  \
	     '' using 3 title \"y_obtido\" with lines\n");
		fprintf(gnuplot, "unset multiplot\n");
		break;
	case 3:
		fprintf(gnuplot, "plot './b.txt' using  6 title \"y_desejado\"with lines,  \
	     '' using 3 title \"y_obtido\" with lines,  \
	     '' using 9 title \"erro\" with lines\n");
		break;
	case 4:
		fprintf(gnuplot, "set multiplot layout 1,2 rowsfirst\n");
		fprintf(gnuplot, "plot './b.txt' using 12 title \"u_entrada_da_planta\" with lines\n");
		fprintf(gnuplot, "plot './b.txt' using  6 title \"y_desejado\"with lines,  \
	     '' using 3 title \"y_obtido\" with lines\n");
		fprintf(gnuplot, "unset multiplot\n");
		break;
	default:
		fprintf(gnuplot, "plot './b.txt' using  6 title \"y_desejado\"with lines,  \
	     '' using 3 title \"y_obtido\" with lines\n");
		break;
	}
	//fprintf(gnuplot, "e\n");
	fflush(gnuplot);//*/**.c
}


void
load_variables(past_variables pv)
{
	int i = 0;

	sigma_critical_deviation = pv.past_sigma_critical_deviation;
	critic_value = pv.past_critic_value;

	for(i = 0; i < 3; i++)
	{
		U[i] = pv.past_U[i];
		error[i] = pv.past_error[i];
		error_order[i] = pv.past_error_order[i];
		pid_params[i] = pv.past_pid_params[i];
		recomended_pid_params[i] = pv.past_recomended_pid_params[i];
	}
}


double
carmen_librlpid_compute_effort_signal (double current_phi, double desired_phi, double next_desired_phi, fann_type *steering_ann_input,
							struct fann *steering_ann, double v, double understeer_coeficient, double distance_between_front_and_rear_axles)
{
	bool first_time = true;
	past_variables pv;
	intelligent_control_params params;

	if(first_time)
	{
		params = read_parameters("params.txt");
//		if(params == NULL)
//		{
//			printf("\nError: Could not open Reinforcement Learning PID parameters\n\n");
//			exit(1);
//		}
		pv = initializate_variables(pv); // Step 1
		load_variables(pv);
		first_time = false;
	}

	calculate_error(desired_phi, desired_phi); // Step 2 ==> CALCULA ERRO

	external_reinforcement_signal(params.alfa_weight_coef, params.beta_weight_coef, params.error_band); //Step 3 ==> RECOMPENSA

	update_neetwork_hidden_unit_phi();// ==> UPDATE PHI
	update_recomended_pid_output(); //Step 4 ==> UPDATE K`
	critic_value = update_critic_value_output();	//Step 4 ==> UPDATE V

	update_pid_params(); //Step 5 ==> UPDATE K

	update_plant_input_u(); //Step 5 ==> UPDATE U

	pv = save_variables(pv);

	//Estimate FUTURE reward
	double atan_current_curvature = carmen_get_curvature_from_phi(current_phi, v, understeer_coeficient, distance_between_front_and_rear_axles);

	double future_phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(U[0], atan_current_curvature, steering_ann_input, steering_ann, v,
																			understeer_coeficient, distance_between_front_and_rear_axles);		//Step 6 ==> PREVE Y(t+1)

	calculate_error(next_desired_phi, future_phi); // Step 6 ==> CALCULA ERRO

	update_neetwork_hidden_unit_phi_future();// ==> UPDATE PHI

	future_critic_value = update_critic_value_future(); //Step 7 ==> UPDATE V

	calculate_td_error(params.discount_factor); //Step 8 ==> CALCULA ERRO TD

	load_variables(pv);

	weights_update(params.actor_learning_rate, params.critic_learning_rate); //Step 9 ==> UPDATE PESOS
	center_vector_update(params.learning_rate_center); //Setp 10 ==> UPDATE CENTRO
	width_scalar_update(params.learning_rate_width); //Step 10 ==> UPDATE WIDTH SCALAR

	return U[0];
}


/*int
main()
{
	//FILE *erro;
	FILE *output;
	char* name = "param.txt";
	double y_desired = 0;
	double y = 0;
	int t = 0;
	past_variables pv;

	output = fopen("b.txt", "w");

	intelligent_control_params params = read_parameters("params.txt");
	srand(time(NULL));

//________________________________________________________________________
	pv = initializate_variables(pv); // Step 1
//________________________________________________________________________

	while (t < 11500)
	{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////   PRESENT   /////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//__________________________________ LOAD ________________________________
		load_variables(pv);
//________________________________________________________________________
		//printf("\nYcoletado = %lf, Ydesejado = %lf, t = %d\n", y, y_desired, t);
		//print_variables();

		y = update_output_y(y, t);// Step 2 ==> PEGA OUTPUT Y
		y_desired = calculate_disired_test_signal(t);// ==> PEGA Y DESEJADO
		calculate_error(y_desired, y); // Step 2 ==> CALCULA ERRO

		if (fabs(error[0]) <= params.error_band){
			//flag = 1; //Encontrou o controlador certo.
			fprintf(output, "Yconvergiu = %lf ,Yd = %lf ,erro = %lf ,u = %lf %d\n", y, y_desired, error_order[0], U[0], t);
		}else{
			fprintf(output, "Ydivergiu = %lf ,Yd = %lf ,erro = %lf ,u = %lf %d\n", y, y_desired, error_order[0], U[0], t);
		}

//________________________________________________________________________
		external_reinforcement_signal(params.alfa_weight_coef, params.beta_weight_coef, params.error_band); //Step 3 ==> RECOMPENSA
//________________________________________________________________________
		update_neetwork_hidden_unit_phi();// ==> UPDATE PHI
		update_recomended_pid_output(); //Step 4 ==> UPDATE K`
		critic_value = update_critic_value_output();	//Step 4 ==> UPDATE V

		//fprintf("Error: %lf PID: %lf %lf %lf DELTAS: %lf %lf %lf\n", error_order[0], pid_params[0], pid_params[1], pid_params[2],
		//		recomended_pid_params[0], recomended_pid_params[1], recomended_pid_params[2]);

		//fprintf(erro, "Error = %lf\n", error_order[0]);

		update_pid_params(); //Step 5 ==> UPDATE K
//________________________________________________________________________
		update_plant_input_u(); //Step 5 ==> UPDATE U
//__________________________________ SAVE _________________________________
		pv = save_variables(pv);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////     FUTURE      ///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//printf("\nY = %lf ,Yd = %lf %d\n", y, y_desired, t);
		double future_y = update_output_y(y, t + 1); //Step 6 ==> PREVE Y(t+1)
		y_desired = calculate_disired_test_signal(t + 1); // ==> PREVE Y DESEJADO (t+1)
		calculate_error(y_desired, future_y); // Step 6 ==> CALCULA ERRO
//________________________________________________________________________
		//external_reinforcement_signal(params.alfa_weight_coef, params.beta_weight_coef, params.error_band); //Step 6 ==> RECOMPENSA
//________________________________________________________________________
		update_neetwork_hidden_unit_phi_future();// ==> UPDATE PHI
		//update_recomended_pid_output(); //Step 7 ==> UPDATE K`
		future_critic_value = update_critic_value_future(); //Step 7 ==> UPDATE V
//________________________________________________________________________
		calculate_td_error(params.discount_factor); //Step 8 ==> CALCULA ERRO TD

		load_variables(pv);

		weights_update(params.actor_learning_rate, params.critic_learning_rate); //Step 9 ==> UPDATE PESOS
		center_vector_update(params.learning_rate_center); //Setp 10 ==> UPDATE CENTRO
		width_scalar_update(params.learning_rate_width); //Step 10 ==> UPDATE WIDTH SCALAR
//________________________________________________________________________
		//print_variables();
		t++;
	}
	plota_graficos(2, 1100, 600, 10000, 2);
	return 0;
}
*/
