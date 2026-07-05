// Codigo baseado em https://github.com/niklaskugler/PathPlanning-Control_TruckTrailerSystem

#include <array>
#include <sys/stat.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>
#include "g2o/types/slam2d/se2.h"
#include "nlp_mat_planner_path_planner_astar.h"
#include "trailer_analytical_expansion.h"


#ifdef USE_CASADI
#include <carmen/trailer_nlp.h>
#endif

using namespace std;
using namespace g2o;

#define MAX_TRAILER_IMPLEMENTED MAX_NUM_TRAILERS

// #define PRINT_DEBUG

extern int number_of_trailers_for_path_planning;
//int number_of_trailers_for_path_planning = 1;
int limited_number_of_trailers;

kuglerParameters_t Parameters;


static void
add_points_to_goal_list_interval(carmen_robot_and_trailers_traj_point_t p1, carmen_robot_and_trailers_traj_point_t p2,
		std::vector<carmen_robot_and_trailers_traj_point_t> &detailed_lane)
{
	double distance = DIST2D(p1, p2);

	double distance_between_goals = 0.07;
	int num_points = ceil(distance / distance_between_goals);
	if (num_points == 0)
		return;

	if (num_points > 10) // Safeguard
		num_points = 10;

	double delta_x = (p2.x - p1.x) / (double) num_points;
	double delta_y = (p2.y - p1.y) / (double) num_points;
	double delta_theta = carmen_normalize_theta(p2.theta - p1.theta) / (double) num_points;
	double delta_beta = carmen_normalize_theta(p2.trailer_theta[0] - p1.trailer_theta[0]) / (double) num_points;

	carmen_robot_and_trailers_traj_point_t new_point = {p1.x, p1.y, p1.theta, p1.num_trailers, {0.0}, p1.v, p1.phi}; // necessario para capturar v e phi
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		new_point.trailer_theta[z] = p1.trailer_theta[z];

	for (int i = 0; i < num_points; i++)
	{
		new_point.x = p1.x + (double) i * delta_x;
		new_point.y = p1.y + (double) i * delta_y;
		new_point.theta = carmen_normalize_theta(p1.theta + (double) i * delta_theta);
		new_point.trailer_theta[0] = carmen_normalize_theta(p1.trailer_theta[0] + (double) i * delta_beta);
		for (size_t z = 1; z < MAX_NUM_TRAILERS; z++)
		{
			double current_delta_trailer_theta = carmen_normalize_theta(p2.trailer_theta[z] - p1.trailer_theta[z]) / (double) num_points;
			new_point.trailer_theta[z] = carmen_normalize_theta(p1.trailer_theta[z] + (double) i * current_delta_trailer_theta);
		}

		detailed_lane.push_back(new_point);
		new_point.v = p2.v; // A velocidade de pontos adicionados é igual a do próximo ponto, p2. O primeiro ponto adicionado é o próprio p1
	}
}


std::vector<carmen_robot_and_trailers_traj_point_t>
increase_path_resolution(std::vector<carmen_robot_and_trailers_traj_point_t> path)
{
	std::vector<carmen_robot_and_trailers_traj_point_t> new_path;

	for (unsigned int i = 0; i < (path.size() - 1); i++)
		add_points_to_goal_list_interval(path.at(i), path.at(i+1), new_path);
	new_path.push_back(path.back());

	std::vector<carmen_robot_and_trailers_traj_point_t> new_path_resolution_fixed;

        if (new_path.size() > 0)
        {
            new_path_resolution_fixed.push_back(new_path[0]);
        }

        double current_acum_distance = 0.0;
        double offroad_min_resolution_path = get_offroad_min_resolution_path_param();

        if (offroad_min_resolution_path < 0.0)
        {
            return (new_path);
        }

        new_path_resolution_fixed.push_back(new_path[0]);
	for (unsigned int i = 1; i < (new_path.size() - 1); i++)
        {
            current_acum_distance += DIST2D(new_path[i - 1], new_path[i]);

            if (current_acum_distance > offroad_min_resolution_path)
            {
                new_path_resolution_fixed.push_back(new_path[i]);
                current_acum_distance = 0.0;
            }
        }

        // new_path_resolution_fixed.push_back(new_path[new_path.size() - 1]);


	return (new_path_resolution_fixed);
}


kugler_coef_t
linsolve_coef(double *A, double *b)
{
	gsl_matrix_view gsl_A = gsl_matrix_view_array(A, 8, 8);
	gsl_vector_view gsl_b = gsl_vector_view_array(b, 8);
	gsl_vector *gsl_x = gsl_vector_alloc(8);

	gsl_permutation *p = gsl_permutation_alloc(8);
	int s;
	gsl_linalg_LU_decomp(&gsl_A.matrix, p, &s);
	gsl_linalg_LU_solve(&gsl_A.matrix, p, &gsl_b.vector, gsl_x);

	kugler_coef_t coef = {};
	coef.a = gsl_vector_get(gsl_x, 0);
	coef.b = gsl_vector_get(gsl_x, 1);
	coef.c = gsl_vector_get(gsl_x, 2);
	coef.d = gsl_vector_get(gsl_x, 3);
	coef.e = gsl_vector_get(gsl_x, 4);
	coef.f = gsl_vector_get(gsl_x, 5);
	coef.g = gsl_vector_get(gsl_x, 6);
	coef.h = gsl_vector_get(gsl_x, 7);

	gsl_permutation_free(p);
	gsl_vector_free(gsl_x);

	return (coef);
}


array<double, 2>
linsolve_w(double *A, double *b)
{
	gsl_matrix_view gsl_A = gsl_matrix_view_array(A, 2, 2);
	gsl_vector_view gsl_b = gsl_vector_view_array(b, 2);
	gsl_vector *gsl_x = gsl_vector_alloc(2);

	gsl_permutation *p = gsl_permutation_alloc(2);
	int s;
	gsl_linalg_LU_decomp(&gsl_A.matrix, p, &s);
	gsl_linalg_LU_solve(&gsl_A.matrix, p, &gsl_b.vector, gsl_x);

	array<double, 2> w = {};
	w[0] = gsl_vector_get(gsl_x, 0);
	w[1] = gsl_vector_get(gsl_x, 1);

	gsl_permutation_free(p);
	gsl_vector_free(gsl_x);

	return (w);
}


kugler_xRef_t
PathPlannerHelper(kugler_state_t state, double d0, double d1, double M)
{
	kugler_xRef_t yRef;

	double eta = 1.0 / cos(state.theta1);
//	double x0_x = eta / cos(state.theta0 - state.theta1) * cos(state.theta0);
//	double y0_x = eta / cos(state.theta0 - state.theta1) * sin(state.theta0);
	double theta0_x = 1.0 / d0 * eta / cos(state.theta0 - state.theta1) * tan(state.phi);
//	double theta0_x = 1.0 / d0 * eta / cos(state.theta0 - state.theta1) * tan(state.theta1); // No matlab, o phi recebe o valor de state_x0(4), que é o theta1
//	double theta1_x = 1.0 / d1 * eta * tan(state.theta0 - state.theta1);
	double theta1_x = 1.0 / d1 * eta * tan(state.theta0 - state.theta1) - (eta/cos(state.theta0 - state.theta1)) * (M * cos(state.theta0 - state.theta1) * tan(state.phi) / (d0 * d1));

	// Express yRef and its derivatives only dependent on the state elements
	yRef.xRef = state.y - d1 * sin(state.theta1); // With regard to rear axle of Trailer
	yRef.dxRef = tan(state.theta1);
	yRef.d2xRef = pow(eta, 3.0) * (1.0 / d1) * tan(state.theta0 - state.theta1);

	double eta_x = yRef.dxRef * yRef.d2xRef / eta; // additional helper variable

	yRef.d3xRef = 3.0 * pow(eta, 2.0) * eta_x * (1.0 / d1) * tan(state.theta0 - state.theta1) + pow(eta, 3.0) * (1.0 / d1) * (1.0 / (pow(cos(state.theta0 - state.theta1), 2.0))) * (theta0_x - theta1_x);

	return (yRef);
}


kugler_coef_t
KuglerPathPlanner(kugler_state_t state_x0, kugler_state_t state_x1, double d0, double d1, double M)
{
	// Inititalize intital state vector - start pose
	double x1_s = state_x0.x;

	// Inititalize final state vector - end pose
	double x1_e = state_x1.x;

	// Calculate reference values

	// yRef and its first, second and third derivatives regarding start pose
	kugler_xRef_t yRef_val_s = PathPlannerHelper(state_x0, d0, d1, M);

	// yRef and its first, second and third derivatives regarding end pose
	kugler_xRef_t yRef_val_e = PathPlannerHelper(state_x1, d0, d1, M);

	////
	// Determine the Coefficents of the polynomial by solving the LGS (Ax = b)

	// Concatenate to vector b (refers to both start and end pose)
	double b[8] = {yRef_val_s.xRef, 	yRef_val_s.dxRef, 		yRef_val_s.d2xRef, 	yRef_val_s.d3xRef,
				   yRef_val_e.xRef, 	yRef_val_e.dxRef, 		yRef_val_e.d2xRef, 	yRef_val_e.d3xRef};

	double x1_s_7 = pow(x1_s, 7.0);
	double x1_s_6 = pow(x1_s, 6.0);
	double x1_s_5 = pow(x1_s, 5.0);
	double x1_s_4 = pow(x1_s, 4.0);
	double x1_s_3 = pow(x1_s, 3.0);
	double x1_s_2 = pow(x1_s, 2.0);
	double x1_s_1 = pow(x1_s, 1.0);
	double x1_e_7 = pow(x1_e, 7.0);
	double x1_e_6 = pow(x1_e, 6.0);
	double x1_e_5 = pow(x1_e, 5.0);
	double x1_e_4 = pow(x1_e, 4.0);
	double x1_e_3 = pow(x1_e, 3.0);
	double x1_e_2 = pow(x1_e, 2.0);
	double x1_e_1 = pow(x1_e, 1.0);

	// Matrix (containing 8 conditions)
	double A[8*8] = {
		x1_s_7,    				x1_s_6,     		x1_s_5,    			x1_s_4,    			x1_s_3,   			x1_s_2,  		x1_s,  		1.0,
		7.0 * x1_s_6,			6.0 * x1_s_5,   	5.0 * x1_s_4,  		4.0 * x1_s_3,  		3.0 * x1_s_2, 		2.0 * x1_s,   	1.0 ,   	0.0,
		42.0 * x1_s_5,  		30.0 * x1_s_4,  	20.0 * x1_s_3, 		12.0 * x1_s_2, 		6.0 * x1_s_1, 		2.0,        	0.0,     	0.0,
		210.0 * x1_s_4, 		120.0 * x1_s_3, 	60.0 * x1_s_2, 		24.0 * x1_s_1, 		6.0,        		0.0,        	0.0,     	0.0,
		x1_e_7,     			x1_e_6,     		x1_e_5,    			x1_e_4,    			x1_e_3,   			x1_e_2,   		x1_e,  		1.0,
		7.0 * x1_e_6,   		6.0 * x1_e_5,   	5.0 * x1_e_4,  		4.0 * x1_e_3,  		3.0 * x1_e_2, 		2.0 * x1_e,   	1.0 ,   	0.0,
		42.0 * x1_e_5,  		30.0 * x1_e_4,  	20.0 * x1_e_3, 		12.0 * x1_e_2, 		6.0 * x1_e_1, 		2.0,        	0.0,     	0.0,
		210.0 * x1_e_4, 		120.0 * x1_e_3, 	60.0 * x1_e_2, 		24.0 * x1_e_1, 		6.0,        		0.0,       		0.0,     	0.0};

	// Output (solving the LGS via linsolve)
	kugler_coef_t coef = linsolve_coef(A, b);

	return (coef);
}


double
get_theta1(carmen_robot_and_trailers_traj_point_t pose)
{
//	return (carmen_normalize_theta(pose.theta - pose.trailer_theta[0]));	// @@@ Anderson vai checar
	return (pose.trailer_theta[0]);
}


double
get_beta(double theta0, double theta1)
{
	return (carmen_normalize_theta(theta0 - theta1));
}


kugler_state_t
get_kugler_state_from_carmen_robot_and_trailer_traj_point(carmen_robot_and_trailers_traj_point_t traj_point)
{
	kugler_state_t kugler_state = {traj_point.x, traj_point.y, traj_point.theta, get_theta1(traj_point), traj_point.phi};

	return (kugler_state);
}


kugler_state_t
get_trailer_pose_from_truck_pose(carmen_robot_and_trailers_traj_point_t truck_pose, double d1)
{
	kugler_state_t trailer_pose = {truck_pose.x - d1 * cos(get_theta1(truck_pose)), truck_pose.y - d1 * sin(get_theta1(truck_pose)), truck_pose.theta, get_theta1(truck_pose), truck_pose.phi};

	return (trailer_pose);
}


//carmen_robot_and_trailer_traj_point_t
//get_truck_pose_from_trailer_pose(kugler_state_t trailer_pose, double d1)
//{
//	carmen_robot_and_trailer_traj_point_t truck_pose = {trailer_pose.x + d1 * cos(get_theta1(trailer_pose)), trailer_pose.y + d1 * sin(get_theta1(trailer_pose)), trailer_pose.theta, get_theta1(trailer_pose), trailer_pose.phi};
//
//	return (truck_pose);
//}


carmen_robot_and_trailers_traj_point_t
get_robot_and_trailer_traj_point_from_solver_state(const double x[], kuglerParameters_t Parameters)
{
	// double beta = get_beta(x[2], x[3]);
	carmen_robot_and_trailers_traj_point_t robot_and_trailer_traj_point =
	{
		x[0],	// x
		x[1],	// y
		x[2],	// theta
		1,
		{x[3], 0.0, 0.0, 0.0, 0.0},
		1.0 * (double) Parameters.direction,	// v
		x[4]	// phi
	};

	return (robot_and_trailer_traj_point);
}


kugler_state_t
KuglerCalcRefValues(double t, kuglerParameters_t Parameters)
{
	double tau = 0.0;
	if (Parameters.direction == 1)
		tau = t / Parameters.T;
	else
		tau = (Parameters.T - t) / Parameters.T;

	double s_tau = 3.0 * pow(tau, 2.0) - 2.0 * pow(tau, 3.0);
//	double ds_tau = 6.0 * tau - 6.0 * pow(tau, 2.0);

	double xRef = Parameters.x0 + (Parameters.x1 - Parameters.x0) * s_tau;
//	double xRef_dot = 1.0 / Parameters.T * (Parameters.x1 - Parameters.x0) * ds_tau;

	double a = Parameters.coef.a;
	double b = Parameters.coef.b;
	double c = Parameters.coef.c;
	double d = Parameters.coef.d;
	double e = Parameters.coef.e;
	double f = Parameters.coef.f;
	double g = Parameters.coef.g;
	double h = Parameters.coef.h;

	// Polynomial with time parametrization
	double yRef = a * pow(xRef, 7.0) + b * pow(xRef, 6.0) + c * pow(xRef, 5.0) + d * pow(xRef, 4.0) + e * pow(xRef, 3.0) + f * pow(xRef, 2.0) + g * xRef + h;

	kugler_state_t traj_point = {xRef, yRef, 0.0, 0.0, 0.0};

	return (traj_point);
}


array<double, 2>
Fourth_Derivatives_of_References(double dyx, double dyxx, double dyxxx, double dyxxxx, double eta)
{
	array<double, 2> ret;

	// Calculates the fourth derivative of the trajectory subject to the arclength (x'''' and y'''')

	// Inputs: 1.,2.,3. and 4. derivative of the reference polynomial subject to
	// x, ( dyx, dyxx, dyxxx, dyxxxx )

	// Outputs: x'''' and y''''
	// d4xref_dsigma4
	ret[0] = -((2.0 * dyxx * dyxxx + dyxx * dyxxx + dyx * dyxxxx) / pow(eta, 5.0) - 5.0 / pow(eta, 6.0) * dyx * dyxx / eta * (pow(dyxx, 2.0) + dyx * dyxxx)) / eta
			 + ((8.0 * dyx * pow(dyxx, 3.0) + 8.0 * pow(dyx, 2.0) * dyxx * dyxxx) / pow(eta, 7.0) - 7.0 / pow(eta, 8.0) * dyx * dyxx / eta * 4.0 * pow(dyx, 2.0) * pow(dyxx, 2.0)) / eta;
	//d4yref_dsigma4
	ret[1] = 1.0 / eta * (dyxxxx / pow(eta, 3.0) - 3.0 * dyxxx / pow(eta, 4.0) * dyx * dyxx / eta - (8.0 * dyxx * dyx * dyxxx + 4.0 * pow(dyxx, 3.0)) / pow(eta, 5.0) + 20.0 / pow(eta, 6.0) * pow(dyxx, 3.0) * pow(dyx, 2.0) / eta
							 - (2.0 * dyx * dyxx * dyxxx + pow(dyx, 2.0) * dyxxxx) / pow(eta, 5.0) + 5.0 * pow(dyx, 3.0) * dyxx * dyxxx / pow(eta, 7.0)
							 + (12.0 * pow(dyx, 2.0) * pow(dyxx, 3.0) + 8.0 * pow(dyx, 3.0) * dyxx * dyxxx) / pow(eta, 7.0) - 28.0 * pow(dyx, 4.0) * pow(dyxx, 3.0) / pow(eta, 9.0));

	return (ret);
}


kugler_Ref_t
KuglerCalcRefValues_ODE(double &eta, double &xRef_dot, double t, kuglerParameters_t Parameters)
{
	double tau = 0.0;
	if (Parameters.direction == 1)
		tau = t / Parameters.T;
	else
		tau = (Parameters.T - t) / Parameters.T;

	double s_tau = 3.0 * pow(tau, 2.0) - 2.0 * pow(tau, 3.0);
	double ds_tau = 6.0 * tau - 6.0 * pow(tau, 2.0);

	double xRef = Parameters.x0 + (Parameters.x1 - Parameters.x0) * s_tau;
	if (Parameters.direction == 1)
		xRef_dot = 1.0 / Parameters.T * (Parameters.x1 - Parameters.x0) * ds_tau; // @@@ Alberto: precidencia divisao multiplicacao?
	else
		xRef_dot = -1.0 / Parameters.T * (Parameters.x1 - Parameters.x0) * ds_tau; // @@@ Alberto: precidencia divisao multiplicacao?

	double a = Parameters.coef.a;
	double b = Parameters.coef.b;
	double c = Parameters.coef.c;
	double d = Parameters.coef.d;
	double e = Parameters.coef.e;
	double f = Parameters.coef.f;
	double g = Parameters.coef.g;
	double h = Parameters.coef.h;

	// Polynomial with time parametrization
	double yRef = a * pow(xRef, 7.0) + b * pow(xRef, 6.0) + c * pow(xRef, 5.0) + d * pow(xRef, 4.0) + e * pow(xRef, 3.0) + f * pow(xRef, 2.0) + g * xRef + h;

	// 1st derivative without time parameter (parametrized by x)
	double dyx = 7.0 * a * pow(xRef, 6.0) + 6.0 * b * pow(xRef, 5.0) + 5.0 * c * pow(xRef, 4.0) + 4.0 * d * pow(xRef, 3.0) + 3.0 * e * pow(xRef, 2.0) + 2.0 * f * pow(xRef, 1.0) + g;

	// 2nd derivative without time parameter (parametrized by x)
	double dyxx = 42.0 * a * pow(xRef, 5.0) + 30.0 * b * pow(xRef, 4.0) + 20.0 * c * pow(xRef, 3.0) + 12.0 * d * pow(xRef, 2.0) + 6.0 * e * pow(xRef, 1.0) + 2.0 * f;

	// 3rd derivative without time parameter (parametrized by x)
	double dyxxx = 210.0 * a * pow(xRef, 4.0) + 120.0 * b * pow(xRef, 3.0) + 60.0 * c * pow(xRef, 2.0) + 24.0 * d * pow(xRef, 1.0) + 6.0 * e;

	// 4th derivative without time parameter (parametrized by x)
	double dyxxxx = 840.0 * a * pow(xRef, 3.0) + 360.0 * b * pow(xRef, 2.0) + 120.0 * c * pow(xRef, 1.0) + 24.0 * d;

	// System model parametized by arc length to remove time dependency
	// Determine eta
	eta = sqrt(1.0 + pow(dyx, 2.0));

	// Determine the ref values derived according to sigma (arc length)
	double dxref_dsigma = 1.0 / eta;
	double d2xref_dsigma2 = -(dyx * dyxx) / (pow(eta, 4.0));
	double d3xref_dsigma3 = -((pow(dyxx, 2.0)) + dyx * dyxxx) / (pow(eta, 5.0)) + (4.0 * (pow(dyx, 2.0)) * (pow(dyxx, 2.0))) / (pow(eta, 7.0));

	double dyref_dsigma = dyx / eta;
	double d2yref_dsigma2 = dyxx / (pow(eta, 2.0)) - ((pow(dyx, 2.0)) * dyxx) / (pow(eta, 4.0));
	double d3yref_dsigma3 = dyxxx / (pow(eta, 3.0)) - (4.0 * (pow(dyxx, 2.0)) * dyx) / (pow(eta, 5.0)) - ((pow(dyx, 2.0)) * dyxxx) / (pow(eta, 5.0)) + (4.0 * (pow(dyx, 3.0)) * (pow(dyxx, 2.0))) / (pow(eta, 7.0));

	array<double, 2> dsigma4;
	dsigma4 = Fourth_Derivatives_of_References(dyx, dyxx, dyxxx, dyxxxx, eta);

	kugler_Ref_t Ref;

	// Define struct for reference values
	Ref.xRef = xRef;
	Ref.dxref_dsigma = dxref_dsigma;
	Ref.d2xref_dsigma2 = d2xref_dsigma2;
	Ref.d3xref_dsigma3 = d3xref_dsigma3;
	Ref.d4xref_dsigma4 = dsigma4[0];

	Ref.yRef = yRef;
	Ref.dyref_dsigma = dyref_dsigma;
	Ref.d2yref_dsigma2 = d2yref_dsigma2;
	Ref.d3yref_dsigma3 = d3yref_dsigma3;
	Ref.d4yref_dsigma4 = dsigma4[1];

	Ref.dyxx = dyxx;

	return (Ref);
}


array<double, 2>
NonlinearOutputFunction(const double State[], kuglerParameters_t Parameters)
{
	double d_1 = Parameters.d1;
	double x_0 = State[0];
	double y_0 = State[1];
	double theta_0 = State[2];
	double theta_1 = State[3];

	// Define elements of the nonlinear output vector
	array<double, 2> y_T;
	y_T[0] = x_0 - d_1 * cos(theta_1) - Parameters.M * cos(theta_0);
	y_T[1] = y_0 - d_1 * sin(theta_1) - Parameters.M * sin(theta_0);

	return (y_T);
}


lie_derivative_t
Truck_1T_LieDeriv(array<double, 5> x_truck, array<double, 3> x_ctrl, double d0, double d1)
{
	// Abbreviations
	double theta0 = x_truck[2];
	double theta1 = x_truck[3];
	double phi = x_truck[4];
		
	double z1 = x_ctrl[0]; // xi_1
	double z2 = x_ctrl[1]; // xi_2
	double z3 = x_ctrl[2]; // xi_3

	double z1sqr = z1 * z1;
	double z1cub = z1sqr * z1;
	double z1qua = z1cub * z1;

	double z2sqr = z2 * z2;
//	double c0 = cos(theta0);
	double c1 = cos(theta1);
//	double s0 = sin(theta0);
	double s1 = sin(theta1);

	double t_phi = tan(phi);

	double t01 = tan(theta0 - theta1);
	double t01_sqr = t01 * t01;
	double t01_cub = t01_sqr * t01;

	// Secants
	double Sec_phi_2 = 1.0 / pow(cos(phi), 2.0);
	double Sec01     = 1.0 / cos(theta0 - theta1);
	double Sec01_sqr = Sec01 * Sec01;
	double Sec01_cub = Sec01_sqr * Sec01;
	double Sec01_qua = Sec01_cub * Sec01;
	double Sec01_qui = Sec01_qua * Sec01;

	lie_derivative_t LD = {};

	LD.L_g1_Lf3_h1 = c1;
	LD.L_g2_Lf3_h1 = -z1cub * Sec_phi_2 * Sec01_cub * s1 / (d0 * d1);

	LD.L_g1_Lf3_h2 = s1;
	LD.L_g2_Lf3_h2 = z1cub * Sec_phi_2 * Sec01_cub * c1 / (d0 * d1);

	LD.Lf_h1 = z1 * c1;
	
	LD.Lf_h2 = z1 * s1;

	LD.Lf2_h1 = z2 * c1 - (z1sqr * s1 * t01 ) / d1;

	LD.Lf2_h2 = z2 * s1 + (z1sqr * c1 * t01) / d1;

	LD.Lf3_h1 = z3 * c1 - (z1cub * s1 * Sec01_cub * t_phi) / (d0 * d1) - (3.0 * z1 * z2 * s1 * t01) / d1 + (z1cub * s1 * Sec01_sqr * t01) / pow(d1, 2.0)
				- (z1cub * c1 * t01_sqr) / pow(d1, 2.0);

	LD.Lf3_h2 = z3 * s1 + (z1cub * c1 * Sec01_cub * t_phi) / (d0 * d1) + (3.0 * z1 * z2 * c1 * t01) / d1 - (z1cub * c1 * Sec01_sqr * t01) / pow(d1, 2.0)
				- (z1cub * s1 * t01_sqr) / pow(d1, 2.0);

	LD.Lf4_h1 = -(6.0 * z1sqr * z2 * Sec01_cub * s1 *t_phi ) / (d0 * d1) + (z1qua * Sec01_qui * s1 * t_phi) / (d0 * pow(d1, 2.0)) - (3.0 * z2sqr * s1 * t01) / d1
				- (4.0 * z1 * z3 * s1 * t01) / d1 + (6.0 * z1sqr * z2 * Sec01_sqr * s1 * t01) / pow(d1, 2.0) - (z1qua * Sec01_qua * s1 * t01) / pow(d1, 3.0)
				- (3.0 * z1qua * c1 * Sec01_cub * t_phi * t01) / (d0 * pow(d1, 2.0))
				- (3.0 * z1qua * Sec01_qua * s1 * pow(t_phi, 2.0) * t01) / (pow(d0, 2.0) * d1) - (6.0 * z1sqr * z2 * c1 * t01_sqr) / pow(d1, 2.0)
				+ (3.0 * z1qua * c1 * Sec01_sqr * t01_sqr) / pow(d1, 3.0) + (5.0 * z1qua * Sec01_cub * s1 * t_phi * t01_sqr) / (d0 * pow(d1, 2.0))
				+ (z1qua * s1 * t01_cub) / pow(d1, 3.0) - (2.0 * z1qua * Sec01_sqr * s1 * t01_cub) / pow(d1, 3.0);
		
		
	LD.Lf4_h2 = (6.0 * z1sqr * z2 * Sec01_cub * c1 * t_phi) / (d0 * d1) - (z1qua * c1 * Sec01_qui * t_phi) / (d0 * pow(d1, 2.0))
				+ (3.0 * z2sqr * c1 * t01) / d1 + (4 * z1 * z3 * c1 * t01) / d1 - (6.0 * z1sqr * z2 * c1 * Sec01_sqr * t01) / pow(d1, 2.0)
				+ (z1qua * c1 * Sec01_qua * t01) / pow(d1, 3.0) - (3.0 * z1qua * Sec01_cub * s1 * t_phi * t01) / (d0 * pow(d1, 2.0))
				+ (3.0 * z1qua * c1 * Sec01_qua * pow(t_phi, 2.0) * t01) / (pow(d0, 2.0) * d1) - (6.0 * z1sqr * z2 * s1 * t01_sqr) / pow(d1, 2.0)
				+ (3.0 * z1qua * Sec01_sqr * s1 * t01_sqr) / pow(d1, 3.0) - (5 * z1qua * c1 * Sec01_cub * t_phi * t01_sqr) / (d0 * pow(d1, 2.0))
				- (z1qua * c1 * t01_cub) / pow(d1, 3.0) + (2.0 * z1qua * c1 * Sec01_sqr * t01_cub) / pow(d1, 3.0);

	return (LD);
}


array<double, 2>
Steering_StabControl(kugler_Ref_t Ref, const double State[], array<double, 2> y_T, kuglerParameters_t Parameters)
{
	// Read weighting factors from struct
	double k0 = Parameters.k0;
	double k1 = Parameters.k1;
	double k2 = Parameters.k2;
	double k3 = Parameters.k3;

	// Read distances between axles from struct
	double d0 = Parameters.d0;
	double d1 = Parameters.d1;

	// Split state vector in controller and vehicle state
	array<double, 5> x_truck = {State[0], State[1], State[2], State[3], State[4]};
	array<double, 3> x_ctrl = {State[5], State[6], State[7]};

	// Set references from CalcRefValues.m
	double xRef = Ref.xRef;
	double dxRef = Ref.dxref_dsigma;
	double d2xRef = Ref.d2xref_dsigma2;
	double d3xRef = Ref.d3xref_dsigma3;

	double yRef = Ref.yRef;
	double dyRef = Ref.dyref_dsigma;
	double d2yRef = Ref.d2yref_dsigma2;
	double d3yRef = Ref.d3yref_dsigma3;

	// Extract actual values from Truck_1T_LieDeriv.m
	lie_derivative_t LD = Truck_1T_LieDeriv(x_truck, x_ctrl, d0, d1);

	double dx_t = LD.Lf_h1;
	double d2x_t = LD.Lf2_h1;
	double d3x_t = LD.Lf3_h1;

	double dy_t = LD.Lf_h2;
	double d2y_t = LD.Lf2_h2;
	double d3y_t = LD.Lf3_h2;


	// y(T) equals current Trailer position
	double x1 = y_T[0];   // x coordinate of the Trailer's rear axle
	double y1 = y_T[1];   // y coordinate of the Trailer's rear axle

	// Tracking error dynamics
	// Difference between reference and actual values
	double ex_t = xRef - x1;
	double dex_t = dxRef - dx_t;
	double d2ex_t = d2xRef - d2x_t;
	double d3ex_t = d3xRef - d3x_t;

	double ey_t = yRef - y1;
	double dey_t = dyRef - dy_t;
	double d2ey_t = d2yRef - d2y_t;
	double d3ey_t = d3yRef - d3y_t;

	// Equations of the controller for feedback linearization (input)
	array<double, 2> ny;
	ny[0] = Ref.d4xref_dsigma4 + k3 * d3ex_t + k2 * d2ex_t + k1 * dex_t + k0 * ex_t;
	ny[1] = Ref.d4yref_dsigma4 + k3 * d3ey_t + k2 * d2ey_t + k1 * dey_t + k0 * ey_t;

	return (ny);
}


array<double, 2>
LinearizingFeedback(array<double, 2> ny, const double State[], kuglerParameters_t Parameters)
{
	// Extract first five elements of state vector
	array<double, 5> x_truck = {State[0], State[1], State[2], State[3], State[4]};

	// Extract first four elements of state vector
	array<double, 3> x_ctrl = {State[5], State[6], State[7]};

	// Extract axle distances from parameters
	double d0 = Parameters.d0;
	double d1 = Parameters.d1;

	// External function to get lie derivatives
	lie_derivative_t LD = Truck_1T_LieDeriv(x_truck, x_ctrl, d0, d1);

	// Given equation (lecture 6, slide 7)
	// ny = A*w + c

	// Define matrix A
	double A[4] = {LD.L_g1_Lf3_h1, LD.L_g2_Lf3_h1,
				   LD.L_g1_Lf3_h2, LD.L_g2_Lf3_h2};

	// %eig(A)

	// Define vector c
	double c[2] = {LD.Lf4_h1,
				   LD.Lf4_h2};

	// Transform the equation to linsolve structure A*w = b

	// Define vector b = ny - c
	double b[2] = {ny[0] - c[0],
				   ny[1] - c[1]};

	// Solve the LGS A*w = b to get w vector
	array<double, 2> w = linsolve_w(A, b);

	return (w);
}


array<double, 3>
ControllerState(array<double, 2> w, double eta, double xRef_dot, array<double, 3> x_C)
{
	// Initialize necessary values for formula implementation
	double xi_2 = x_C[1];
	double xi_3 = x_C[2];
	double w_1 = w[0];

	// Compute controller state
	double dxi_1 = xi_2 * eta * xRef_dot;
	double dxi_2 = xi_3 * eta * xRef_dot;
	double dxi_3 = w_1 * eta * xRef_dot;

	// Derivate of controller state vector
	array<double, 3> dx_C = {dxi_1,
							 dxi_2,
							 dxi_3};

	return (dx_C);
}


array<double, 2>
TimeReparametrization(array<double, 2> w, double eta, double xRef_dot, array<double, 5> x_T, array<double, 3> x_C)
{
	// Initialize necessary values for formula implementation
	double xi_1 = x_C[0]; 
	double theta_0 = x_T[2];
	double theta_1 = x_T[3];
	double w_2 = w[1]; 

	// Define input vector elements u1 & u2 for Truck/Trailer model
	double u_1 = xi_1 / cos(theta_0 - theta_1) * eta * xRef_dot; 
	double u_2 = w_2 * eta * xRef_dot;

	// Define input vector u for Truck/Trailer model
	array<double, 2> u = {u_1,
						  u_2};

	return (u);
}


array<double, 5>
TruckTrailerModel(array<double, 5> x_T, array<double, 2> u, kuglerParameters_t Parameters)
{
	// Initialize necessary values for formula implementation
	double d_0 = Parameters.d0;
	double d_1 = Parameters.d1;
	double theta_0 = x_T[2];
	double theta_1 = x_T[3];
	double phi = x_T[4];
	double u_1 = u[0]; // u_1 = v_0
	double u_2 = u[1];

	// Define elements of the Truck/Trailer vector (vehicle model)
	double dx_0 = u_1 * cos(theta_0);
	double dy_0 = u_1 * sin(theta_0);
	double dtheta_0 = u_1 / d_0 * tan(phi);
	double dtheta_1 = u_1 / d_1 * (sin(theta_0 - theta_1) - Parameters.M / d_0 * sin(theta_0 - theta_1) * tan(phi));
	double dphi = u_2;

	// Define Truck/Trailer vector (vehicle model)
	array<double, 5> dx_T = {dx_0,
							 dy_0,
							 dtheta_0,
							 dtheta_1,
							 dphi};

	return (dx_T);
}


int
KuglerODEFunc(double t, const double State[], double dState[], void *params)
{
	kuglerParameters_t Parameters = *((kuglerParameters_t *) params);

	// Calculate reference values of x and y
	double eta, xRef_dot;
	kugler_Ref_t Ref = KuglerCalcRefValues_ODE(eta, xRef_dot, t, Parameters);

	// Nonlinear output function y_T = h(x_T)
	array<double, 2> y_T = NonlinearOutputFunction(State, Parameters);

	// Steering law and linearizing control considering error dynamics
	// Stabilization in case of position uncertainty
	array<double, 2> ny = Steering_StabControl(Ref, State, y_T, Parameters);

	array<double, 2> w = LinearizingFeedback(ny, State, Parameters);

    // Controller State/Decoupling Controller in new parametrization (sigma)
	array<double, 3> x_C = {State[5], State[6], State[7]}; // x_C equals xi vector [xi_1, xi_2, xi_3]
	array<double, 3> dx_C = ControllerState(w, eta, xRef_dot, x_C);

	// Reparametrization of the subject (change in time replacing the arc length)
	array<double, 5> x_T = {State[0], State[1], State[2], State[3], State[4]}; // x_T equals Truck vector [x0, y0, theta0, theta1, phi]
    array<double, 2> u = TimeReparametrization(w, eta, xRef_dot, x_T, x_C);

	// Simple vehicle model (time parametrized calculated state)
    array<double, 5> dx_T = TruckTrailerModel(x_T, u, Parameters);

    dState[0] = dx_T[0];
    dState[1] = dx_T[1];
    dState[2] = dx_T[2];
    dState[3] = dx_T[3];
    dState[4] = dx_T[4];

//    dState[5] = dx_C[0];
//    dState[6] = dx_C[1];
//    dState[7] = dx_C[2];

    dState[5] = 0.0;
    dState[6] = dx_C[0];
    dState[7] = dx_C[1];

//    dState[5] = x_T[5];	// @@@ Alberto: este x_T[5] não está errado?
//    dState[6] = dx_C[0];
//    dState[7] = dx_C[1];
//    dState[8] = dx_C[2];

	return (GSL_SUCCESS);
}


vector<kugler_state_t>
compute_trailer_reference_trajectory(kuglerParameters_t Parameters, double dt)
{
	vector<kugler_state_t> trajectory;

	for (double t = 0; t < Parameters.T; t += dt)
	{
		kugler_state_t traj_point = KuglerCalcRefValues(t, Parameters);
		trajectory.push_back(traj_point);
//		printf("%lf, %lf\n", traj_point.x, traj_point.y);
//		fflush(stdout);
	}

	return (trajectory);
}


carmen_robot_and_trailers_traj_point_t
change_pose_to_relative_coordinates(carmen_robot_and_trailers_traj_point_t reference_pose, carmen_robot_and_trailers_traj_point_t pose)
{
//	double beta = convert_theta1_to_beta(pose.theta, pose.trailer_theta[0]);

	SE2 reference_pose_se2(reference_pose.x, reference_pose.y, reference_pose.theta);
	SE2 pose_se2(pose.x, pose.y, pose.theta);
	SE2 pose_in_relative_coordinates_se2 = reference_pose_se2.inverse() * pose_se2;

	carmen_robot_and_trailers_traj_point_t pose_in_relative_coordinates = pose;
	pose_in_relative_coordinates.x = pose_in_relative_coordinates_se2[0];
	pose_in_relative_coordinates.y = pose_in_relative_coordinates_se2[1];
	pose_in_relative_coordinates.theta = pose_in_relative_coordinates_se2[2];
//	pose_in_relative_coordinates.trailer_theta[0] = convert_beta_to_theta1(pose_in_relative_coordinates_se2[2], beta);
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
	{
		double current_beta = convert_theta1_to_beta(pose.theta, pose.trailer_theta[z]); // Necessário para trocar a referência do trailer_theta
		pose_in_relative_coordinates.trailer_theta[z] = convert_beta_to_theta1(pose_in_relative_coordinates.theta, current_beta);
	}
	return (pose_in_relative_coordinates);
}


//carmen_robot_and_trailer_traj_point_t
//change_pose_to_absolute_coordinates(carmen_robot_and_trailer_traj_point_t reference_pose, carmen_robot_and_trailer_traj_point_t pose)
//{
//	carmen_robot_and_trailer_traj_point_t pose_in_absolute_coordinates = pose;
//
//	pose_in_absolute_coordinates.x = reference_pose.x + pose.x * cos(reference_pose.theta) - pose.y * sin(reference_pose.theta);
//	pose_in_absolute_coordinates.y = reference_pose.y + pose.x * sin(reference_pose.theta) + pose.y * cos(reference_pose.theta);
//	pose_in_absolute_coordinates.theta = carmen_normalize_theta(pose.theta + reference_pose.theta);
//
//	return (pose_in_absolute_coordinates);
//}


void
change_path_to_absolute_coordinates(vector<carmen_robot_and_trailers_traj_point_t> &path, carmen_robot_and_trailers_traj_point_t localizer_pose)
{
//	int i = 0;
	for (std::vector<carmen_robot_and_trailers_traj_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
//		double beta = convert_theta1_to_beta(it->theta, it->trailer_theta[0]);
		double old_theta = it->theta;
		double x = localizer_pose.x + it->x * cos(localizer_pose.theta) - it->y * sin(localizer_pose.theta);
		double y = localizer_pose.y + it->x * sin(localizer_pose.theta) + it->y * cos(localizer_pose.theta);
		it->x = x;
		it->y = y;
		it->theta = carmen_normalize_theta(it->theta + localizer_pose.theta);

//		it->trailer_theta[0] = convert_beta_to_theta1(it->theta, beta);

		for (size_t j = 0; j < MAX_NUM_TRAILERS; j++)
		{
			double beta = convert_theta1_to_beta(old_theta, it->trailer_theta[j]);
			it->trailer_theta[j] = convert_beta_to_theta1(it->theta, beta);
		}

//		printf("%d %lf %lf %lf %lf %lf\n", i++, x, y, it->theta, it->trailer_theta[0], beta);
	}
}


void
get_start_and_final_states(kugler_state_t &start_x0, kugler_state_t &start_x1, kugler_state_t &final_x0, kugler_state_t &final_x1,
		int direction,
		carmen_robot_and_trailers_traj_point_t current_pose_in_relative_coordinates, double d1,
		carmen_robot_and_trailers_traj_point_t goal_pose_in_relative_coordinates)
{
	if (direction == 1)
	{
		start_x0 = get_kugler_state_from_carmen_robot_and_trailer_traj_point(current_pose_in_relative_coordinates);
		start_x1 = get_trailer_pose_from_truck_pose(current_pose_in_relative_coordinates, d1);
		final_x0 = get_kugler_state_from_carmen_robot_and_trailer_traj_point(goal_pose_in_relative_coordinates);
		final_x1 = get_trailer_pose_from_truck_pose(goal_pose_in_relative_coordinates, d1);
	}
	else
	{
		final_x0 = get_kugler_state_from_carmen_robot_and_trailer_traj_point(current_pose_in_relative_coordinates);
		final_x1 = get_trailer_pose_from_truck_pose(current_pose_in_relative_coordinates, d1);
		start_x0 = get_kugler_state_from_carmen_robot_and_trailer_traj_point(goal_pose_in_relative_coordinates);
		start_x1 = get_trailer_pose_from_truck_pose(goal_pose_in_relative_coordinates, d1);
	}
//	final_x1.y = final_x0.y;	// @@@ Alberto: Isso esta certo???
}


void
get_ode_start_state(double odeStartState[], int direction, const kugler_state_t &start_x0, const kugler_state_t &final_x0, double xi_1, double xi_2, double xi_3)
{
	if (direction == 1)
	{
		odeStartState[0] = start_x0.x;
		odeStartState[1] = start_x0.y;
		odeStartState[2] = start_x0.theta0;
		odeStartState[3] = start_x0.theta1;
		odeStartState[4] = start_x0.phi;
	}
	else
	{
		odeStartState[0] = final_x0.x;
		odeStartState[1] = final_x0.y;
		odeStartState[2] = final_x0.theta0;
		odeStartState[3] = final_x0.theta1;
		odeStartState[4] = final_x0.phi;
	}
	odeStartState[5] = xi_1;
	odeStartState[6] = xi_2;
	odeStartState[7] = xi_3;
}


//static void
//print_poses(std::vector<carmen_robot_and_trailer_traj_point_t> poses, char *filename)
//{
//	FILE *arq = fopen(filename, "w");
//	for (unsigned int i = 0; i < poses.size(); i++)
//		fprintf(arq, "x %lf, y %lf, theta %lf, phi %lf, beta %lf, v %lf\n",
//				poses[i].x, poses[i].y, poses[i].theta, poses[i].phi, poses[i].trailer_theta[0], poses[i].v);
//	fclose(arq);
//}


vector<double>
get_vector_of_doubles_from_file(char *working_directory, const char *filename)
{
	char wk_and_filename[1000];
	sprintf(wk_and_filename, "%s/%s", working_directory, filename);
	FILE *file_with_vector = fopen(wk_and_filename, "r");
	if (file_with_vector == NULL)
		exit(printf("Houve um erro ao abrir o arquivo %s em get_vector_of_doubles_from_file()\n", wk_and_filename));

	char file_contents[100000];
    fgets(file_contents, 100000-1, file_with_vector);

    vector<double> vector_of_doubles;
    char *double_from_file = strtok(file_contents, "; ");
    if (double_from_file)
    {
		do
		{
			char *eptr;
			vector_of_doubles.push_back(strtod(double_from_file, &eptr));
		} while ((double_from_file = strtok(NULL, "; ")) != NULL);
    }

	if (fclose(file_with_vector) != 0)
		printf("Erro no fechamento do aquivo %s em get_vector_of_doubles_from_file()\n", wk_and_filename);

    return (vector_of_doubles);
}


void
set_robot_configuration(const carmen_robot_ackerman_config_t &robot_config, const carmen_semi_trailers_config_t &semi_trailer_config,
		char *working_directory)
{
	char filename[1000];

	// Necessário criar um arquivo separado que informa a quantidade de trailers usado (O sistema utiliza o numero de trailers + 1)
	sprintf(filename, "%s/%s", working_directory, "Num_tractors_plus_trailers_f");
	FILE *file_pointer_num_trailers = fopen(filename, "w");
	if (file_pointer_num_trailers == NULL)
		exit(printf("Houve um erro ao criar o arquivo %s em set_robot_configuration()\n", filename));

	fprintf(file_pointer_num_trailers, "%d\n", (limited_number_of_trailers + 1));

	if (fclose(file_pointer_num_trailers) != 0)
		printf("Erro no fechamento do aquivo %s em set_robot_configuration()\n", filename);
	//

	sprintf(filename, "%s/%s", working_directory, "Robot_config");
	FILE *file_pointer = fopen(filename, "w");
	if (file_pointer == NULL)
		exit(printf("Houve um erro ao criar o arquivo %s em set_robot_configuration()\n", filename));

	double amax = 0.68;
	double vmax = 1.0;
	double wmax = 1.0;
	double phimax = robot_config.max_phi;

	int index = 1;
	fprintf(file_pointer, "%d  %f \n", index++, robot_config.distance_between_front_car_and_front_wheels);
	fprintf(file_pointer, "%d  %f \n", index++, robot_config.distance_between_front_and_rear_axles);
	fprintf(file_pointer, "%d  %f \n", index++, robot_config.distance_between_rear_car_and_rear_wheels);
	fprintf(file_pointer, "%d  %f \n", index++, robot_config.distance_between_rear_wheels / 2.0);
	fprintf(file_pointer, "%d  %f \n", index++, amax);
	fprintf(file_pointer, "%d  %f \n", index++, vmax);
	fprintf(file_pointer, "%d  %f \n", index++, wmax);
	fprintf(file_pointer, "%d  %f \n", index++, phimax);

	// Coloquei a sessão de trailers para baixo, para permitir um valor variável de trailers
	fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[0].d);
	fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[0].distance_between_axle_and_front);
	fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[0].distance_between_axle_and_back);
	fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[0].M);
	for (int ii = 1; ii < limited_number_of_trailers; ii++)
	{
		fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[ii].d);
		fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[ii].distance_between_axle_and_front);
		fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[ii].distance_between_axle_and_back);
		fprintf(file_pointer, "%d  %f \n", index++, semi_trailer_config.semi_trailers[ii].M);
	}

	if (fclose(file_pointer) != 0)
		printf("Erro no fechamento do aquivo %s em set_robot_configuration()\n", filename);
}


void
set_initial_state(const carmen_robot_and_trailers_traj_point_t current_pose, char *working_directory)
{
	char filename[1000];
	sprintf(filename, "%s/%s", working_directory, "Initial_state");
	FILE *file_pointer = fopen(filename, "w");
	if (file_pointer == NULL)
		exit(printf("Houve um erro ao criar o arquivo %s em set_initial_state()\n", filename));

	double v_0 = 0.0;
	double a_0 = 0.0;
	double w_0 = 0.0;
	int index = 1;
	fprintf(file_pointer, "%d  %f \n", index++, current_pose.x);
	fprintf(file_pointer, "%d  %f \n", index++, current_pose.y);
	fprintf(file_pointer, "%d  %f \n", index++, current_pose.theta);
	fprintf(file_pointer, "%d  %f \n", index++, current_pose.phi);
	fprintf(file_pointer, "%d  %f \n", index++, v_0);
	fprintf(file_pointer, "%d  %f \n", index++, a_0);
	fprintf(file_pointer, "%d  %f \n", index++, w_0);
	fprintf(file_pointer, "%d  %f \n", index++, current_pose.trailer_theta[0]);
	for (int ii = 1; ii < limited_number_of_trailers; ii++)
		fprintf(file_pointer, "%d  %f \n", index++, current_pose.trailer_theta[ii]);

	if (fclose(file_pointer) != 0)
		printf("Erro no fechamento do aquivo %s in set_initial_state()\n", filename);
}


void
set_final_state(const carmen_robot_and_trailers_traj_point_t goal_pose, char *working_directory)
{
	char filename[1000];
	sprintf(filename, "%s/%s", working_directory, "Final_state");
	FILE *file_pointer = fopen(filename, "w");
	if (file_pointer == NULL)
		exit(printf("Houve um erro ao criar o arquivo %s em set_final_state()\n", filename));

	double v_tf = 0.0;
	double a_tf = 0.0;
	double w_tf = 0.0;
	int index = 1;
	fprintf(file_pointer, "%d  %f \n", index++, goal_pose.x);
	fprintf(file_pointer, "%d  %f \n", index++, goal_pose.y);
	fprintf(file_pointer, "%d  %f \n", index++, goal_pose.theta);
	fprintf(file_pointer, "%d  %f \n", index++, v_tf);
	fprintf(file_pointer, "%d  %f \n", index++, a_tf);
	fprintf(file_pointer, "%d  %f \n", index++, w_tf);
	fprintf(file_pointer, "%d  %f \n", index++, goal_pose.trailer_theta[0]);
	for (int ii = 1; ii < limited_number_of_trailers; ii++)
		fprintf(file_pointer, "%d  %f \n", index++, goal_pose.trailer_theta[ii]);

	if (fclose(file_pointer) != 0)
		printf("Erro no fechamento do aquivo %s set_final_state()\n", filename);
}


void
set_initial_guess(const carmen_robot_and_trailers_traj_point_t current_pose, const carmen_robot_and_trailers_traj_point_t goal_pose,
		carmen_semi_trailers_config_t semi_trailer_config, char *working_directory)
{
	char filename[1000];
	sprintf(filename, "%s/%s", working_directory, "Initial_guess");
	FILE *file_pointer = fopen(filename, "w");
	if (file_pointer == NULL)
		exit(printf("Houve um erro ao criar o arquivo %s em set_initial_guess()\n", filename));

	int num_time_steps = 80;	// Tem que ser igual ao valor em case2.mod
	double dx = (goal_pose.x - current_pose.x) / (double) num_time_steps;
	double dy = (goal_pose.y - current_pose.y) / (double) num_time_steps;
	double dtheta1 = carmen_normalize_theta(goal_pose.theta - current_pose.theta) / (double) num_time_steps;
	//double current_theta2 = current_pose.theta - current_pose.trailer_theta[0];
	double current_theta2 = current_pose.trailer_theta[0];
	//double goal_theta2 = goal_pose.theta - goal_pose.trailer_theta[0];
	double goal_theta2 = goal_pose.trailer_theta[0];
	double dtheta2 = carmen_normalize_theta(goal_theta2 - current_theta2) / (double) num_time_steps;
	for (int i = 0; i < num_time_steps; i++)
	{
		fprintf(file_pointer, "let x[%d,%d]:=%5.4f;\n", i + 1, 1, current_pose.x + dx * (double) i);
		fprintf(file_pointer, "let y[%d,%d]:=%5.4f;\n", i + 1, 1, current_pose.y + dy * (double) i);
		fprintf(file_pointer, "let theta[%d,%d]:=%5.4f;\n", i + 1, 1, carmen_normalize_theta(current_pose.theta + dtheta1 * (double) i));
		fprintf(file_pointer, "let theta[%d,%d]:=%5.4f;\n", i + 1, 2, carmen_normalize_theta(current_theta2 + dtheta2 * (double) i));
		for (int ii = 1; ii < limited_number_of_trailers; ii++)
		{
			double current_thetax = current_pose.trailer_theta[ii];
			double goal_thetax = goal_pose.trailer_theta[ii];
			double dthetax = carmen_normalize_theta(goal_thetax - current_thetax) / (double) num_time_steps;
			fprintf(file_pointer, "let theta[%d,%d]:=%5.4f;\n", i + 1, ii + 2, carmen_normalize_theta(current_thetax + dthetax * (double) i));
		}
	}
	carmen_robot_and_trailers_traj_point_t semi_trailers_poses[MAX_NUM_TRAILERS];

	for (int ii = 0; ii < limited_number_of_trailers; ii++)
	{
		double d = semi_trailer_config.semi_trailers[ii].d;
		double M = semi_trailer_config.semi_trailers[ii].M;

		for (int i = 0; i < num_time_steps - 1; i++)
		{
			double x1 = current_pose.x + dx * (double) i;
			double y1 = current_pose.y + dy * (double) i;
			double theta1 = carmen_normalize_theta(current_pose.theta + dtheta1 * (double) i);
			double theta2 = carmen_normalize_theta(current_theta2 + dtheta2 * (double) i);
			if (ii == 0)
			{
				semi_trailers_poses[ii].x = x1 - d * cos(theta2) - M * cos(theta1);
				semi_trailers_poses[ii].y = y1 - d * sin(theta2) - M * sin(theta1);
				semi_trailers_poses[ii].theta = current_theta2;
			}
			else
			{
				semi_trailers_poses[ii].x = semi_trailers_poses[ii-1].x - M * cos(current_pose.trailer_theta[ii - 1]) - d * cos(current_pose.trailer_theta[ii]);
				semi_trailers_poses[ii].y = semi_trailers_poses[ii-1].y - M * sin(current_pose.trailer_theta[ii - 1]) - d * sin(current_pose.trailer_theta[ii]);
				semi_trailers_poses[ii].theta = current_pose.trailer_theta[ii]; //- semi_trailers_poses_localizer[j].theta;
			}

			fprintf(file_pointer, "let x[%d,%d]:=%5.4f;\n", i + 1, ii + 2, semi_trailers_poses[ii].x);
			fprintf(file_pointer, "let y[%d,%d]:=%5.4f;\n", i + 1, ii + 2, semi_trailers_poses[ii].y);
		}

	}
	if (fclose(file_pointer) != 0)
		printf("Erro no fechamento do aquivo %s in set_initial_guess()\n", filename);
}


bool
run_nlp_planner(char *working_directory)
{
	static bool first_time = true;
	if (first_time)
	{
		char new_path_env_var[10000];
		sprintf(new_path_env_var, "%s/AMPL/ampl.linux-intel64:%s", getenv("HOME"), getenv("PATH"));
		setenv("PATH", new_path_env_var, 1);
		first_time = false;
	}

	char command[1000];
//	if (limited_number_of_trailers == 1)
		sprintf(command, "cd %s; ampl.bin r2_linux.run > r2_linux.out", working_directory);
//	else if (limited_number_of_trailers == 2)
//		sprintf(command, "cd %s; ampl.bin r2_linux_2_trailer.run > r2_linux.out", working_directory);

	system(command);

	char filename[1000];
	sprintf(filename, "%s/%s", working_directory, "flag_for_nlp.txt");
	FILE *file_pointer = fopen(filename, "r");
	if (file_pointer == NULL)
		exit(printf("Houve um erro ao abrir o arquivo %s em run_nlp_planner()\n", filename));

	char file_contents[100];
    fgets(file_contents, 100-1, file_pointer);

	if (fclose(file_pointer) != 0)
		printf("Erro no fechamento do aquivo %s em run_nlp_planner()\n", filename);

    if (file_contents[0] == '1')
    	return (true);
    else
    	return (false);
}


void
set_working_directory(char *working_directory, carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal)
{
	char base_directory[512];
	strcpy(base_directory, getenv("HOME"));
	strcat(base_directory, (char *) ("/carmen/src/offroad_planner/Tractor_trailer_trajectory_planning_ICRA19"));
//	sprintf(working_directory, "%s/tmp/%lf/", base_directory, carmen_get_time());
	sprintf(working_directory, "%s/tmp/%f_%f_%f_%f_%f_%f_%f_%f", base_directory, current.x, current.y, current.theta, current.trailer_theta[0], goal.x, goal.y, goal.theta, goal.trailer_theta[0]);
	char command[2048];
//	if (limited_number_of_trailers == 1)
		sprintf(command, "mkdir -p %s; cp %s/r2_linux.run %s/ipopt.opt %s/case2.mod %s", working_directory, base_directory, base_directory, base_directory, working_directory);
//	else if (limited_number_of_trailers == 2)
//		sprintf(command, "mkdir -p %s; cp %s/r2_linux_2_trailer.run  %s/ipopt.opt %s/case2_2_trailer.mod %s", working_directory, base_directory, base_directory, base_directory, working_directory);

	system(command);
}


void
remove_working_directory(char *working_directory)
{
//	return;
	char command[1000];
	sprintf(command, "rm %s/*; rmdir %s", working_directory, working_directory);
	system(command);
}


vector<carmen_robot_and_trailers_traj_point_t>
trailer_polynomial_analytical_expansion(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose,
		int direction, carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map,
		double max_phi_multiplier, bool test_limits)
{
	double T = 10.0;
	double dt = 0.1;

	double k0 = 0.0625;
	double k1 = 0.5;
	double k2 = 0.75;
	double k3 = 2.0;

	double d0 = robot_config.distance_between_front_and_rear_axles;
	double d1 = semi_trailer_config.semi_trailers[0].d;

	carmen_robot_and_trailers_traj_point_t current_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, current_pose);
	carmen_robot_and_trailers_traj_point_t goal_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, goal_pose);

	kugler_state_t start_x0, start_x1, final_x0, final_x1;
	get_start_and_final_states(start_x0, start_x1, final_x0, final_x1, direction, current_pose_in_relative_coordinates, d1, goal_pose_in_relative_coordinates);

	kugler_coef_t coef = KuglerPathPlanner(start_x1, final_x1, d0, d1, semi_trailer_config.semi_trailers[0].M);
//	start_x1 = {-19.859, 5.0, 0.0, 0.0, 0.0};
//	start_x1 = {-30.0, 5.0, 0.0, 0.0, 0.0};
//	final_x1.phi = 0.0;
//	final_x0.phi = 0.0;

	if (direction == -1)
	{
		k1 = k1 * -1.0;
		k3 = k3 * -1.0;
	}

	kuglerParameters_t Parameters = {coef, d0, d1, T, start_x1.x, final_x1.x, k0, k1, k2, k3, direction, semi_trailer_config.semi_trailers[0].M};

	// Steady controller states
	double xi_1 = 1.0;
	double xi_2 = 0.0;
	double xi_3 = 0.0;

#define SYSTEM_SIZE	8
	const gsl_odeiv2_step_type *stepper = gsl_odeiv2_step_rkf45;
	gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(stepper, SYSTEM_SIZE);
	gsl_odeiv2_system sys = { KuglerODEFunc, NULL, SYSTEM_SIZE, &Parameters };

	double State[SYSTEM_SIZE];
	get_ode_start_state(State, direction, start_x0, final_x0, xi_1, xi_2, xi_3);

	double xerr[SYSTEM_SIZE];

	vector<carmen_robot_and_trailers_traj_point_t> trajectory;
	for (double t = 0.0; t <= T; t += dt)
	{
		trajectory.push_back(get_robot_and_trailer_traj_point_from_solver_state(State, Parameters));
		if (isnan(trajectory[trajectory.size() - 1].x))
			return {}; // Identifica se as coordenadas são NaN, e já descarta o path para economizar custo computacional
		gsl_odeiv2_step_apply(s, t, dt, State, xerr, NULL, NULL, &sys);
	}

	gsl_odeiv2_step_free(s);

	change_path_to_absolute_coordinates(trajectory, current_pose);

	if (test_limits)
	{
		if ((DIST2D(trajectory[trajectory.size() - 1], goal_pose) > 0.5) ||
			(fabs(THETA_DIFF(trajectory[trajectory.size() - 1], goal_pose)) > MAX_POLY_THETA_DIFF) ||
			(fabs(THETA0_DIFF(trajectory[trajectory.size() - 1], goal_pose)) > MAX_POLY_BETA_DIFF))
			return {};

		for (size_t i = 0; i < trajectory.size(); i++)
		{
			if ((obstacle_distance_grid_map != NULL && trajectory_pose_hit_obstacle(trajectory[i], robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config)) || // Checar colisao
				(fabs(trajectory[i].phi) > (robot_config.max_phi / max_phi_multiplier)) || // Checar phi
				(fabs(convert_theta1_to_beta(trajectory[i].theta, trajectory[i].trailer_theta[0])) > semi_trailer_config.semi_trailers[0].max_beta) || // Checar beta do trailer
				(i > 0 && (DIST2D(trajectory[i], trajectory[i - 1]) > 1.0))) // Checar se os pontos do path sao aproximados
				return {};
		}
	}

	return (trajectory);
}


bool near_enough_multi_trailer(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal, goal_constraints_t goal_constraints)
{
    double distance = (DIST2D(current, goal));
    double theta_diff = (THETA_DIFF(current, goal));

    if (distance < goal_constraints.goal_achieved_distance && fabs(theta_diff) < goal_constraints.goal_achieved_theta)
    {
            for (int i = 0; i < number_of_trailers_for_path_planning; i++)
            {
                if (fabs(carmen_normalize_theta(carmen_normalize_theta(current.trailer_theta[i]) - carmen_normalize_theta(goal.trailer_theta[i]))) > goal_constraints.goal_achieved_trailer_theta)
                    return false;
            }
            return true;
    }
    else
        return false;
}


vector<carmen_robot_and_trailers_traj_point_t>
trailer_nlp_analytical_expansion_ampl(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config)
{
	printf("Entrou no ampl:\n");
	limited_number_of_trailers = std::min(number_of_trailers_for_path_planning, MAX_TRAILER_IMPLEMENTED);

	char working_directory[1000];
	set_working_directory(working_directory, current_pose, goal_pose);
	usleep(10000);
	struct stat sb;
	if (stat(working_directory, &sb) != 0 || !S_ISDIR(sb.st_mode))
		exit(printf("O diretorio %s nao foi criado...\n", working_directory));

	set_robot_configuration(robot_config, semi_trailer_config, working_directory);

//	printf("Initial: %lf %lf %lf %lf\n", current_pose.x, current_pose.y, current_pose.theta, current_pose.trailer_theta[0]);
//	printf("Final: %lf %lf %lf %lf\n", goal_pose.x, goal_pose.y, goal_pose.theta, goal_pose.trailer_theta[0]);

	carmen_robot_and_trailers_traj_point_t current_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, current_pose);
	carmen_robot_and_trailers_traj_point_t goal_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, goal_pose);

	set_initial_state(current_pose_in_relative_coordinates, working_directory);
	set_final_state(goal_pose_in_relative_coordinates, working_directory);
	set_initial_guess(current_pose_in_relative_coordinates, goal_pose_in_relative_coordinates, semi_trailer_config, working_directory);

//	printf("RInitial: %lf %lf %lf %lf\n", current_pose_in_relative_coordinates.x, current_pose_in_relative_coordinates.y, current_pose_in_relative_coordinates.theta, current_pose_in_relative_coordinates.trailer_theta[0]);
//	printf("RFinal: %lf %lf %lf %lf\n", goal_pose_in_relative_coordinates.x, goal_pose_in_relative_coordinates.y, goal_pose_in_relative_coordinates.theta, goal_pose_in_relative_coordinates.trailer_theta[0]);

	vector<carmen_robot_and_trailers_traj_point_t> trajectory;
	bool plan_ok = run_nlp_planner(working_directory);
	if (!plan_ok)
	{
		remove_working_directory(working_directory);
		return (trajectory);
	}

	vector<double> x = get_vector_of_doubles_from_file(working_directory, "x.txt");
	vector<double> y = get_vector_of_doubles_from_file(working_directory, "y.txt");
	vector<double> theta = get_vector_of_doubles_from_file(working_directory, "theta.txt");
	vector<double> v = get_vector_of_doubles_from_file(working_directory, "v.txt");
	vector<double> phi = get_vector_of_doubles_from_file(working_directory, "phi.txt");

	remove_working_directory(working_directory);

	for (unsigned int i = 0; i < v.size(); i++)
	{
		carmen_robot_and_trailers_traj_point_t traj_point = {x[i], y[i], theta[i], 1, {0.0}, v[i], phi[i]};
		for (int ii = 0; ii < limited_number_of_trailers; ii++)
		{
			traj_point.trailer_theta[ii] = theta[i + (v.size() * (ii + 1))];
//			printf("traj_point: %d %d %lf %d\n", i, ii, traj_point.trailer_theta[ii], i + (v.size() * (ii + 1)));
		}
		trajectory.push_back(traj_point);
	}

	change_path_to_absolute_coordinates(trajectory, current_pose);

	return (trajectory);
}


pair<bool, vector<carmen_robot_and_trailers_traj_point_t>>
trailer_nlp_analytical_expansion(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map,
		double max_phi_multiplier, bool test_limits, bool make_table, goal_constraints_t goal_constraints,
		int semi_trailer_solver_type
    __attribute__ ((unused))
    )
{
	vector<carmen_robot_and_trailers_traj_point_t> trajectory;

	limited_number_of_trailers = std::min(number_of_trailers_for_path_planning, MAX_TRAILER_IMPLEMENTED);

#ifdef USE_CASADI
//	if (semi_trailer_solver_type == 1) //0=AMPL, 1=CASADI
		printf("Entrou no CASADI \n");
		trajectory = trailer_nlp_analytical_expansion_casadi(current_pose, goal_pose, robot_config, semi_trailer_config, limited_number_of_trailers);
#else
		trajectory = trailer_nlp_analytical_expansion_ampl(current_pose, goal_pose, robot_config, semi_trailer_config);
		printf("Entrou no AMPL\n");
#endif
		
	if (trajectory.size() > 0)
	{
		if (make_table == false)
			trajectory = increase_path_resolution(trajectory);

		if (test_limits)
		{
			if (!near_enough_multi_trailer(trajectory[trajectory.size() - 1], goal_pose, goal_constraints))
			{
#ifdef PRINT_DEBUG
				double distance = (DIST2D(trajectory[trajectory.size() - 1], goal_pose));
    			double theta_diff = (THETA_DIFF(trajectory[trajectory.size() - 1], goal_pose));

				if (distance >= goal_constraints.goal_achieved_distance)
					printf("Goal not near enough multi trailer | distance = %lf\n", distance);
				if (fabs(theta_diff) >= goal_constraints.goal_achieved_theta)
					printf("Goal not near enough multi trailer | theta diff = %lf\n", theta_diff);
				for (int i = 0; i < number_of_trailers_for_path_planning; i++)
				{
					theta_diff = fabs(carmen_normalize_theta(carmen_normalize_theta(trajectory[trajectory.size() - 1].trailer_theta[i]) - carmen_normalize_theta(goal_pose.trailer_theta[i])));
					if (theta_diff > goal_constraints.goal_achieved_trailer_theta)
						printf("Goal not near enough multi trailer | trailer %d theta diff = %lf\n", i, theta_diff);
				}
#endif
				return {false, trajectory};
			}

			for (size_t i = 0; i < trajectory.size(); i++)
			{
				if ((obstacle_distance_grid_map != NULL && trajectory_pose_hit_obstacle(trajectory[i], robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config)) || // Checar colisao
					(fabs(trajectory[i].phi) > (robot_config.max_phi / max_phi_multiplier)) || // Checar phi
					(i > 0 && (DIST2D(trajectory[i], trajectory[i - 1]) > 1.0))) // Checar se os pontos do path sao aproximados
				{
#ifdef PRINT_DEBUG
					if (obstacle_distance_grid_map != NULL && trajectory_pose_hit_obstacle(trajectory[i], robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config))
						printf("Trajectory pose hit obstacle\n");
					if (fabs(trajectory[i].phi) > (robot_config.max_phi / max_phi_multiplier))
						printf("Trajectory phi bigger then max allowed | phi = %lf\n", trajectory[i].phi);
					if (i > 0 && (DIST2D(trajectory[i], trajectory[i - 1]) > 1.0))
						printf("Distance between two points greater than allowed | distance = %lf\n", DIST2D(trajectory[i], trajectory[i - 1]));
#endif

					return {false, trajectory};
				}

				for (int ii = 0; ii < limited_number_of_trailers; ii++)
				{
					if (ii == 0)
					{
						if (fabs(convert_theta1_to_beta(trajectory[i].theta, trajectory[i].trailer_theta[0])) > semi_trailer_config.semi_trailers[0].max_beta) // Checar beta do trailer
						{
#ifdef PRINT_DEBUG
							printf("Trailer %d Jack-knifing | phi = %lf\n", ii, fabs(convert_theta1_to_beta(trajectory[i].theta, trajectory[i].trailer_theta[0])));
#endif

							return {false, trajectory};
						}
					}
					else
					{
						if (fabs(convert_theta1_to_beta(trajectory[i].trailer_theta[ii - 1], trajectory[i].trailer_theta[ii])) > (semi_trailer_config.semi_trailers[ii].max_beta)) // Checar beta do trailer
						{
#ifdef PRINT_DEBUG
							printf("Trailer %d Jack-knifing | phi = %lf\n", ii, fabs(convert_theta1_to_beta(trajectory[i].theta, trajectory[i].trailer_theta[0])));
#endif

							return {false, trajectory};
						}
					}
				}
//				printf("[] %lf %lf %lf %lf %lf %lf %lf\n", trajectory[i].x, trajectory[i].y, trajectory[i].theta, trajectory[i].trailer_theta[0], trajectory[i].trailer_theta[1], trajectory[i].trailer_theta[2], abs(convert_theta1_to_beta(trajectory[i].trailer_theta[1], trajectory[i].trailer_theta[2])));

			}
		}
	}
	else
	{
#ifdef PRINT_DEBUG
		printf("Empty trajectory\n");
#endif

		return {false, {}};
	}

	return {true, trajectory};
}
