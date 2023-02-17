#include <carmen/carmen.h>
#include <carmen/motion_planner.h>
#include "ford_escape_hybrid_tune_pid_automatic.h"
#include <carmen/car_model.h>
#include <vector>

#include "ford_escape_hybrid_interface.h"
#include "ford_escape_hybrid_messages.h"
#include <carmen/behavior_selector_interface.h>
#include <carmen/simulator_ackerman_interface.h>

using namespace std;
bool plan_in_progress = false;
double error_sum = 0.0;
bool first_time = true;

double kp_final = 551.52;
double ki_final = 1606.96;
double kd_final = 30.8;

typedef double (*MotionControlFunction)(double v, double w, double t);

static carmen_robot_ackerman_config_t motion_planner_config;

static carmen_robot_and_trailers_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_PER_VECTOR];

static double max_v = 0.0;
static double max_phi = 0.0;
static double frequency = 0.0;
static double timer_period = 1.0;

static double t1, t2, t3;

/*static MotionControlFunction v_function;
static MotionControlFunction phi_function;*/

static int wave_form = 0;


carmen_simulator_ackerman_truepos_message *simulator_velocity;
velocity_pid_data_message *velocity_msg =NULL;
//vector<double> current_steer_angle_vector, desired_steer_angle_vector, error_vector, timestamp_vector;

double velocity_kp = 35.0;
double velocity_ki = 9.0;
double velocity_kd = -0.3;

double velocity_errror_sum = 0.0;

static void
signal_handler(int signo __attribute__ ((unused)) )
{
	carmen_ipc_disconnect();
	
	exit(0);
}


static void
send_trajectory_to_robot()
{
	carmen_base_ackerman_publish_motion_command(motion_commands_vector, NUM_MOTION_COMMANDS_PER_VECTOR, carmen_get_time());
}


double
get_effort_in_time_from_spline(EFFORT_SPLINE_DESCRIPTOR *descriptors, double time)
{
	double x[4] = { 0.0, PREDICTION_HORIZON / 3.0, 2.0 * PREDICTION_HORIZON / 3.0, PREDICTION_HORIZON };
	double y[4] = { descriptors->k1, descriptors->k2, descriptors->k3, descriptors->k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	double effort = gsl_spline_eval(phi_effort_spline, time, acc);

	gsl_spline_free(phi_effort_spline);
	gsl_interp_accel_free(acc);

	return (effort);
}


vector<double>
get_effort_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, double prediction_horizon)
{
	double x[4] = { 0.0, prediction_horizon / 3.0, 2.0 * prediction_horizon / 3.0, prediction_horizon };
	double y[4] = { descriptors->k1, descriptors->k2, descriptors->k3, descriptors->k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	vector<double> effort_vector;
	for (double t = 0.0; t < prediction_horizon; t += DELTA_T)
		effort_vector.push_back(gsl_spline_eval(phi_effort_spline, t, acc));
		//effort_vector.push_back(carmen_clamp(-100.0, gsl_spline_eval(phi_effort_spline, t, acc), 100.0));

	gsl_spline_free(phi_effort_spline);
	gsl_interp_accel_free(acc);

	return (effort_vector);
}


unsigned int
get_motion_timed_index_to_motion_command(PARAMS* params) // TODO nao devia retornar j--   ????
{
	double motion_commands_vector_time = params->motion_commands_vector[0].time;
	unsigned int j = 0;
	while ((motion_commands_vector_time	< params->time_elapsed_since_last_motion_command) && (j < params->motion_commands_vector_size))
	{
		j++;
		motion_commands_vector_time += params->motion_commands_vector[j].time;
	}

	return j;
}


vector<double>
get_velocity_supersampling_motion_commands_vector(PARAMS *params, unsigned int size)
{
	vector<double> velocity_vector;
	double phi_vector_time = 0.0;

	unsigned int timed_index_to_motion_command = get_motion_timed_index_to_motion_command(params);
	double motion_commands_vector_time = params->motion_commands_vector[timed_index_to_motion_command].time;

	for (unsigned int i = 0; i < size; i++)
	{
		velocity_vector.push_back(params->motion_commands_vector[timed_index_to_motion_command].v);

		phi_vector_time += DELTA_T;
		if (phi_vector_time > motion_commands_vector_time)
		{
			timed_index_to_motion_command++;
			if (timed_index_to_motion_command >= params->motion_commands_vector_size)
				break;

			motion_commands_vector_time += params->motion_commands_vector[timed_index_to_motion_command].time;
		}
	}

	return (velocity_vector);
}


double
car_steering_model(double steering_effort, double atan_current_curvature, double v, fann_type *steering_ann_input, PARAMS *params)
{
	//steering_effort *= (1.0 / (1.0 + (params->current_velocity * params->current_velocity) / CAR_MODEL_GAIN)); // boa
	steering_effort = carmen_clamp(-100.0, steering_effort, 100.0);

	// TODO fazer funcao aterar a proxima curvatura tambem
	double phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(steering_effort, atan_current_curvature, steering_ann_input,
			params->steering_ann, v, params->understeer_coeficient, params->distance_rear_axles, 2.0 * params->max_phi);
//	phi = 1.0 * phi;// - 0.01;
//	phi *= (1.0 / (1.0 + v / 10.0));
	
	return (phi);
}


vector<double>
get_phi_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, PARAMS *params)
{
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	memcpy(steering_ann_input, params->steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));

	vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors, PREDICTION_HORIZON);
	vector<double> velocity_vector = get_velocity_supersampling_motion_commands_vector(params, effort_vector.size());
	vector<double> phi_vector;
	double phi, atan_current_curvature = params->atan_current_curvature;

	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
		phi = car_steering_model(effort_vector[i], atan_current_curvature, velocity_vector[i], steering_ann_input, params);

		phi_vector.push_back(phi + params->dk);

		atan_current_curvature = carmen_get_curvature_from_phi(phi, params->current_velocity, params->understeer_coeficient, params->distance_rear_axles);
	}
	return (phi_vector);
}




void
build_trajectory_trapezoidal_v_phi()
{
	double delta_t, t;
	int i;

	double t0 = 2.0;
	double t4 = 2.0;
	delta_t = (t0 + t1 + t2 + t3 + t4) / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);

	static FILE *gnuplot_pipe = NULL;

	if(gnuplot_pipe != NULL)
		fclose(gnuplot_pipe);

	//printf("max_v %lf\n", max_v);
	for (i = 0, t = 0.0; t < t0; t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
		//printf("av %lf\n", motion_commands_vector[i].v);
	}

	for (t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = t * (max_v / t1);
		motion_commands_vector[i].phi = t * (max_phi / t1);
		motion_commands_vector[i].time = delta_t;
		//printf("bv %lf\n", motion_commands_vector[i].phi);
		//printf("const %lf\n", (motion_commands_vector[i].phi - motion_commands_vector[i-1].phi));
	}

	for (t = 0.0; t < t2; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = max_phi;
		motion_commands_vector[i].time = delta_t;
		//printf("cv %lf\n", motion_commands_vector[i].phi);
		//printf("const %lf\n", (motion_commands_vector[i].phi - motion_commands_vector[i-1].phi));
	}

	for (t = 0.0; t <= t3; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v - t * (max_v / t3);
		motion_commands_vector[i].phi = max_phi - t * (max_phi / t3);;
		motion_commands_vector[i].time = delta_t;
		//printf("dv %lf\n", motion_commands_vector[i].phi);
		//printf("const %lf\n", (motion_commands_vector[i].phi - motion_commands_vector[i-1].phi));
	}

	for (t = 0.0; t <= (t4 + delta_t); t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
		//printf("ev %lf\n", motion_commands_vector[i].v);
	}
	/*FILE *mcv_file;
	mcv_file = fopen("plot.txt", "w");
	for(i = 0; i < NUM_MOTION_COMMANDS_PER_VECTOR; i++)
	{
		//printf("i = %d\n", i);
		fprintf(mcv_file, "%lf %lf\n", motion_commands_vector[i].phi,(double) i);
	}
	fclose(mcv_file);

	gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes

	fprintf(gnuplot_pipe, "plot "
			"'./plot.txt' using ($2) : ($1) with lines title 'phi'\n");

	fflush(gnuplot_pipe);*/
	send_trajectory_to_robot();
}


void
build_trajectory_trapezoidal_phi()
{
	double delta_t, t;
	int i;
	
	double t0 = 2.0;
	double t4 = 2.0;
	delta_t = (t0 + t1 + t2 + t3 + t4) / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);

	for (i = 0, t = 0.0; t < t0; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}

	for (t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = t * (max_phi / t1);
		motion_commands_vector[i].time = delta_t;
	}
	
	for (t = 0.0; t < t2; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = max_phi;
		motion_commands_vector[i].time = delta_t;
	}	
	
	for (t = 0.0; t <= t3; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = max_phi - t * (max_phi / t3);
		motion_commands_vector[i].time = delta_t;
	}	
	
	for (t = 0.0; t <= (t4 + delta_t); t += delta_t, i++)
	{
		//3 * exp(-((10 - x) * (10 - x)) / (4 * 4))
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}
//	printf("i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", i, NUM_MOTION_COMMANDS_PER_VECTOR);
	send_trajectory_to_robot();
}

void
build_trajectory_sinusoidal_phi()
{
	double delta_t, t;
	int i;
	double t0 = 2.0;

	delta_t = (t0 + t1) / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);

	for (i = 0, t = 0.0; t < t0; t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
		printf("av %lf\n", motion_commands_vector[i].v);
	}

	for (t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;

		motion_commands_vector[i].phi = max_phi * sin(2.0 * M_PI * frequency * t);
		motion_commands_vector[i].time = delta_t;

		printf("%lf   %lf\n", frequency * t, motion_commands_vector[i].phi);
	}

	printf("i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", i, NUM_MOTION_COMMANDS_PER_VECTOR);
	send_trajectory_to_robot();
}


tune_pid_gain_velocity_parameters_message* 
fill_pid_gain_velocity_parameters_message(double kp, double ki, double kd)
{
	tune_pid_gain_velocity_parameters_message *pid_msg = (tune_pid_gain_velocity_parameters_message*) malloc (sizeof (tune_pid_gain_velocity_parameters_message));
	pid_msg->kd = kd;
	pid_msg->ki = ki;
	pid_msg->kp = kp;
	pid_msg->host = carmen_get_host();
	pid_msg->timestamp = carmen_get_time();
	return pid_msg;
}

tune_pid_gain_steering_parameters_message* 
fill_pid_gain_steering_parameters_message(double kp, double ki, double kd)
{
	tune_pid_gain_steering_parameters_message *pid_msg = (tune_pid_gain_steering_parameters_message*) malloc (sizeof (tune_pid_gain_steering_parameters_message));
	pid_msg->kd = kd; //30.8;
	pid_msg->ki = ki; //2008.7;
	pid_msg->kp = kp; //689.4;
	pid_msg->host = carmen_get_host();
	pid_msg->timestamp = carmen_get_time();
	return pid_msg;
}

void
publish_current_state(carmen_behavior_selector_state_message *msg)
{
	IPC_RETURN_TYPE err;

	msg->timestamp = carmen_get_time();
	msg->host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}


static void
timer_handler()
{
	static int first_time = 1;

	// Publicar ganhos de steering 
	// Publicar ganhos de velocity
	
	if (first_time)
	{
		/*if (frequency != 0.0)
			build_trajectory_sinusoidal_phi();
		else*/
		build_trajectory_trapezoidal_v_phi();
		first_time = 0;

		carmen_behavior_selector_state_message msg_behavior;
		msg_behavior.low_level_state = Free_Running;
		msg_behavior.low_level_state_flags = 0;
		msg_behavior.host = carmen_get_host();
		msg_behavior.timestamp = carmen_get_time();
		publish_current_state(&msg_behavior);

	}
}

void
get_velocity_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	simulator_velocity->trailer_theta[0] = msg->trailer_theta[0];
	simulator_velocity->num_trailers = msg->num_trailers;
	simulator_velocity->v = msg->v;
	simulator_velocity->odometrypose = msg->odometrypose;
	simulator_velocity->phi = msg->phi;
	simulator_velocity->truepose = msg->truepose;
	simulator_velocity->host = msg->host;
	simulator_velocity->timestamp = msg->timestamp;
}


double
my_f(const gsl_vector *v,__attribute__((unused)) void *params_ptr)
{
	double kp = gsl_vector_get(v, 0);
	double ki = gsl_vector_get(v, 1);
	double kd = gsl_vector_get(v, 2);

	printf("VALOR Kp %lf\n", kp);
	printf("VALOR Ki %lf\n", ki);
	printf("VALOR Kd %lf\n", kd);

	kp_final = kp;
	ki_final = ki;
	kd_final = kd;
	//return (execute_motion_command(kp, ki, kd));  // TODO ??? send the motion command to the car and wait for the return

	//chamar o publisher e publicar a mensagem com os parametros kp, ki, kd
	tune_pid_gain_steering_parameters_message *pid_steer_msg = fill_pid_gain_steering_parameters_message(kp, ki, kd);
	carmen_ford_escape_publish_tune_pid_gain_steering_parameters_message(pid_steer_msg , carmen_get_time());

	if(!first_time)
		error_sum = 0.0;

	first_time = false;
	//send_trajectory_to_robot();
	build_trajectory_trapezoidal_v_phi();
	plan_in_progress = true;



	while(plan_in_progress)
	{
		usleep(500000);
		carmen_ipc_dispatch_nonblocking();
	}


	printf("ERROR SUM %lf\n\n", error_sum);

	return (error_sum);
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double h = 105.0;                  // This value influences the result, think how to use it as a function of the size of the error (the biggest the error the biggest h)
	double f_x = my_f(v, params);
	gsl_vector *x_h;

	x_h = gsl_vector_alloc(3);

	double kp = gsl_vector_get(v, 0);
	double ki = gsl_vector_get(v, 1);
	double kd = gsl_vector_get(v, 2);

	gsl_vector_set(x_h, 0, kp + h);
	gsl_vector_set(x_h, 1, ki);
	gsl_vector_set(x_h, 2, kd);
	double f_kp_h = my_f(x_h, params);
	double df_kp_h = (f_kp_h - f_x) / h;

	gsl_vector_set(x_h, 0, kp);
	gsl_vector_set(x_h, 1, ki + h);
	gsl_vector_set(x_h, 2, kd);
	double f_ki_h = my_f(x_h, params);
	double df_ki_h = (f_ki_h - f_x) / h;

//	gsl_vector_set(x_h, 0, kp);         // Do not optimize the derivative initialy
//	gsl_vector_set(x_h, 1, ki);
//	gsl_vector_set(x_h, 2, kd + h);
//	double f_kd_h = my_f(x_h, params);
//	double df_kd_h = (f_kd_h - f_x) / h;

	gsl_vector_set(df, 0, df_kp_h);
	gsl_vector_set(df, 1, df_ki_h);
//	gsl_vector_set(df, 2, df_kd_h);

	gsl_vector_free(x_h);
}


void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f (x, params);
	my_df (x, params, df);
}

void
optimize_PID_gains()
{
	gsl_vector *v;
	gsl_multimin_function_fdf my_func;
	size_t iter = 0;
	int status;

	my_func.n = 3;            // Num of parameters to optimize
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	// my_func.params = motion_command_message; ???? usar algum parametro?

	v = gsl_vector_alloc (my_func.n);
	gsl_vector_set(v, 0, kp_final); //gsl_vector_set(v, 0, 35.0);
	gsl_vector_set(v, 1, ki_final); // gsl_vector_set(v, 1, 9.0);
	gsl_vector_set(v, 2, kd_final); //gsl_vector_set(v, 2, -0.3);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, my_func.n);

	gsl_multimin_fdfminimizer_set(s, &my_func, v, 2.5, 0.001);     //(function_fdf, gsl_vector, step_size, tol)

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate(s);
		if (status)
		{
			break;
		}

		status = gsl_multimin_test_gradient(s->gradient, 0.2);

	} while ((status == GSL_CONTINUE) && (iter < 15));

//	double kp = gsl_vector_get(s->x, 0); //carmen_clamp(-100.0, gsl_vector_get(s->x, 0), 100.0);
//	double ki = gsl_vector_get(s->x, 1); //carmen_clamp(-100.0, gsl_vector_get(s->x, 1), 100.0);
//	double kd = gsl_vector_get(s->x, 2); //carmen_clamp(-100.0, gsl_vector_get(s->x, 2), 100.0);


	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(v);

	optimize_PID_gains();

	return;
}

/*void
make_plot()
{
	static FILE *gnuplot_pipe;
	double y_range = 0.55;
	static int iter = 0;
	if (gnuplot_pipe != NULL) {
	    fclose(gnuplot_pipe);
	}
	if(error_vector.empty())
	{
		return;
	}
	FILE *arquivo_teste = fopen("steer_pid.txt", "w");
	for(long unsigned int i = 0 ; i < current_steer_angle_vector.size(); i++)
		fprintf(arquivo_teste, "%lf %lf %lf %ld %lf %lf %lf\n",
			current_steer_angle_vector[i], desired_steer_angle_vector[i], error_vector[i], i,
			velocity_kp, velocity_ki, velocity_kd);
	fflush(arquivo_teste);
	fclose(arquivo_teste);
	gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
	fprintf(gnuplot_pipe, "set terminal png size 1000,1000\n");
	fprintf(gnuplot_pipe, "set output '%d.png'\n", iter);
	fprintf(gnuplot_pipe, "set title 'KP %lf KI %lf KD %lf Err %lf'\n", kp_final, ki_final , kd_final, error_sum);
	fprintf(gnuplot_pipe, "set yrange [-%lf:%lf]\n", y_range, y_range);
	fprintf(gnuplot_pipe, "plot "
			"'./steer_pid.txt' using 4:1 with lines title 'current steer angle',"
			"'./steer_pid.txt' using 4:2 with lines title 'desired steer angle', "
			"'./steer_pid.txt' using 4:3 with lines title 'error'\n");
	fflush(gnuplot_pipe);
	iter++;

}*/

void
subscribe_pid_steer_feedback_handler(steering_pid_data_message *msg)
{

	static int count = 0;
	count++;
	if(count > 10 && msg->atan_current_curvature < 0.01 )
	{
		plan_in_progress = false;
		count = 0;
		//TODO criar uma função pra plotar o gráfico
		//make_plot();
		//TODO achar comando pra não plotar janelas infinitas
		/*current_steer_angle_vector.clear();
		desired_steer_angle_vector.clear();
		timestamp_vector.clear();
		error_vector.clear();*/
		//error_sum = 0;
	}
	//current_steer_angle_vector.push_back(msg->atan_current_curvature);
	//desired_steer_angle_vector.push_back(msg->atan_desired_curvature);
	if(msg->atan_desired_curvature == 0.0)
		msg->error_t = 0.0;
	//error_vector.push_back(msg->error_t);
	//timestamp_vector.push_back(msg->timestamp);

}

void
subscribe_pid_steer_error_feedback_handler(steering_pid_error_message * msg)
{
	error_sum = msg->errror_sum;
}

static int
subscribe_to_relevant_messages()
{
	static int first_time = 1;
	if(first_time)
	{
		simulator_velocity = (carmen_simulator_ackerman_truepos_message *) malloc (1 * sizeof(carmen_simulator_ackerman_truepos_message));
		first_time = 0;
	}

	carmen_simulator_ackerman_subscribe_external_truepos_message(NULL, (carmen_handler_t) get_velocity_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ford_escape_subscribe_steering_pid_data_message(NULL, (carmen_handler_t) subscribe_pid_steer_feedback_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ford_escape_subscribe_steering_pid_error_message(NULL, (carmen_handler_t) subscribe_pid_steer_error_feedback_handler, CARMEN_SUBSCRIBE_LATEST);
	return (0);
}

static void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *) "robot",(char *) "max_velocity", CARMEN_PARAM_DOUBLE, &motion_planner_config.max_v, 1, NULL},
			{(char *) "robot",(char *) "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &motion_planner_config.maximum_acceleration_forward, 1, NULL},
			{(char *) "robot",(char *) "understeer_coeficient", CARMEN_PARAM_DOUBLE, &motion_planner_config.understeer_coeficient, 1, NULL},
			{(char *) "robot",(char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &motion_planner_config.distance_between_front_and_rear_axles, 1, NULL},
			{(char *) "commandline",(char *) "max_v", CARMEN_PARAM_DOUBLE, &max_v, 0, NULL},
			{(char *) "commandline",(char *) "max_phi", CARMEN_PARAM_DOUBLE, &max_phi, 0, NULL},
			{(char *) "commandline",(char *) "frequency", CARMEN_PARAM_DOUBLE, &frequency, 0, NULL},
			{(char *) "commandline",(char *) "timer_period", CARMEN_PARAM_DOUBLE, &timer_period, 0, NULL},
			{(char *) "commandline",(char *) "wave_form", CARMEN_PARAM_INT, &wave_form, 0, NULL},

			{(char *) "commandline",(char *) "t1", CARMEN_PARAM_DOUBLE, &t1, 0, NULL},
			{(char *) "commandline",(char *) "t2", CARMEN_PARAM_DOUBLE, &t2, 0, NULL},
			{(char *) "commandline",(char *) "t3", CARMEN_PARAM_DOUBLE, &t3, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);

	carmen_param_allow_unfound_variables(1);

	carmen_param_install_params(argc, argv, param_list, num_items);

	max_phi = carmen_degrees_to_radians(max_phi);
}


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);

	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
}


int
main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	define_messages();

	read_parameters(argc, argv);

	

	/*
	tune_pid_gain_velocity_parameters_message *pid_vel_msg = fill_pid_gain_velocity_parameters_message();
	carmen_ford_escape_publish_tune_pid_gain_velocity_parameters_message(pid_vel_msg , carmen_get_time());
	tune_pid_gain_steering_parameters_message *pid_steer_msg = fill_pid_gain_steering_parameters_message();
	carmen_ford_escape_publish_tune_pid_gain_steering_parameters_message(pid_steer_msg , carmen_get_time());
	*/



	carmen_ipc_addPeriodicTimer(timer_period, (TIMER_HANDLER_TYPE) timer_handler, NULL);
	subscribe_to_relevant_messages();

	optimize_PID_gains();
	carmen_ipc_dispatch();

	return 0;
}
