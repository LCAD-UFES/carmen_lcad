#ifndef INTELLIGENT_CONTROL
#define INTELLIGENT_CONTROL


#define neural_network_size (6)
#define PI (3.141592653589793)


typedef struct 
{
	//double* W;                     //Weigt between actor units
	//double* V;                     //Weigt between critic units
	//double* M;                     //
	double alfa_weight_coef;       //alfa
	double beta_weight_coef;       //beta
	double learning_rate_center;   //Eta mi
	double learning_rate_width;    //Eta gama
	double discount_factor;        //Gama
	double error_band;             //epslon
	double actor_learning_rate;    //alfa_a
	double critic_learning_rate;   //alfa_c
} intelligent_control_params;


typedef struct 
{
	double center_vector_mi[3];
	double width_scalar_sigma;
	double phi_value;
	double w_neuron_wight[3];
	double v_neuron_wight;
} rbf_neuron;


#endif