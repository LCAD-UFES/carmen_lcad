#include <carmen/carmen.h>
#include "mpc.h"


typedef struct
{
	double k1, k2, k3, k4;
} EFFORT_SPLINE_DESCRITOR;


//TrajectoryLookupTable::TrajectoryControlParameters
//get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
//		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams &params,
//		bool has_previous_good_tcp)
//{
//	get_optimization_params(target_v, tcp_seed, target_td, params);
//
//	gsl_multimin_function_fdf my_func;
//
//	my_func.n = 3;
//	my_func.f = my_f;
//	my_func.df = my_df;
//	my_func.fdf = my_fdf;
//	my_func.params = &params;
//
//	/* Starting point, x */
//	gsl_vector *x = gsl_vector_alloc(3);
//	gsl_vector_set(x, 0, tcp_seed.k2);
//	gsl_vector_set(x, 1, tcp_seed.k3);
//	if (params.optimize_time)
//		gsl_vector_set(x, 2, tcp_seed.tt);
//	else
//		gsl_vector_set(x, 2, tcp_seed.a);
//
//	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_vector_bfgs2;
//	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 3);
//
//	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.0001, 0.01);
//
//	size_t iter = 0;
//	int status;
//	do
//	{
//		iter++;
//
//		status = gsl_multimin_fdfminimizer_iterate(s);
//		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
//			break;
//
//		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero
////		params.suitable_acceleration = compute_suitable_acceleration(gsl_vector_get(x, 2), target_td, target_v);
//
//	} while ((params.plan_cost > 0.005) && (status == GSL_CONTINUE) && (iter < 30));
//
//	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);
//
//	if ((tcp.tt < 0.2) || (params.plan_cost > 0.05)) // too short plan or bad minimum (s->f should be close to zero) mudei de 0.05 para outro
//		tcp.valid = false;
//
//	if (target_td.dist < 3.0 && tcp.valid == false) // para debugar
//		tcp.valid = false;
//
//	gsl_multimin_fdfminimizer_free(s);
//	gsl_vector_free(x);
//
//	return (tcp);
//}


double
generate_optimized_steering_using_mpc(carmen_ackerman_motion_command_p current_motion_command_vector,
			double nun_motion_commands,
			double time_of_last_command,
			EFFORT_SPLINE_DESCRITOR &effort_spline_descriptor_seed,
			past_steering)
{
	// (i) Usar o effort_spline_descriptor_seed para criar uma seed do vetor de esforcos.
	// (ii) Usar o simulador para, com o vetor de esforcos, gerar um vetor de motion_commands
	// O passo (ii) ocorrer como parte do conjugate gradient de forma continua, ate chagar a um
	// vetor de motion_commands otimo.
	// Retornar o primeiro (proximo) effort associado a este vetor de motion_commands otimo.
}


double
model_predictive_control(double atan_desired_curvature, double atan_current_curvature,
		carmen_simulator_ackerman_config_t *simulator_config)
{
	static double simulated_atan_current_curvature = 0.0;
	double u_t;

	update_past_steering(atan_current_curvature);
	update_past_simulated_steering(simulated_atan_current_curvature);

	u_t = generate_optimized_steering_using_mpc(simulator_config->current_motion_command_vector,
			simulator_config->nun_motion_commands,
			simulator_config->time_of_last_command,
			effort_spline_descriptor_seed,
			past_steering);

	update_simulator_bias(past_steering, past_simulated_steering);

	return (u_t);
}


void
carmen_libmpc_steering_MPC_controler(double *steering_command, double atan_desired_curvature,
		double atan_current_curvature, carmen_simulator_ackerman_config_t *simulator_config)
{
	double u_t;

	u_t = model_predictive_control(atan_desired_curvature, atan_current_curvature, simulator_config);
	*steering_command = carmen_clamp(-100.0, u_t, 100.0);

	return *steering_command;
}
