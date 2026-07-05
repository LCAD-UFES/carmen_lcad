#include "trailer_nlp.h"
#include <carmen/carmen.h>
#include <casadi/casadi.hpp>
#include <g2o/types/slam2d/se2.h>
#include <sstream>    // Para garantir a conversão de int para string com segurança
#include <unistd.h>   // Para a função access()


#define THETA_DIFF(x1, x2) (carmen_normalize_theta((carmen_normalize_theta((x1).theta) - carmen_normalize_theta((x2).theta))))
#define BETA_DIFF(x1, x2) (carmen_normalize_theta((carmen_normalize_theta((x1).beta) - carmen_normalize_theta((x2).beta))))

#define MAX_POLY_BETA_DIFF carmen_degrees_to_radians(5.0)
#define MAX_POLY_THETA_DIFF carmen_degrees_to_radians(5.0)

carmen_trailer_nlp_config_all_t
get_robot_configuration(const carmen_robot_ackerman_config_t &robot_config, const carmen_semi_trailers_config_t &semi_trailer_config, int limited_number_of_trailers)
{
	carmen_trailer_nlp_config_all_t trailer_nlp_config_all;
	trailer_nlp_config_all.distance_between_front_car_and_front_wheels = robot_config.distance_between_front_car_and_front_wheels;
	trailer_nlp_config_all.distance_between_front_and_rear_axles = robot_config.distance_between_front_and_rear_axles;
	trailer_nlp_config_all.distance_between_rear_car_and_rear_wheels = robot_config.distance_between_rear_car_and_rear_wheels;
	trailer_nlp_config_all.distance_between_rear_wheels_half = robot_config.distance_between_rear_wheels / 2.0;
	trailer_nlp_config_all.amax = 0.4;	 // aceleracao maxima
	trailer_nlp_config_all.vmax = 0.656; // velocidade maxima
	trailer_nlp_config_all.wmax = 0.2;	 // 11,46 graus por segundo de velocidade maxima de phi
	trailer_nlp_config_all.phimax = robot_config.max_phi * 1.0;
	for (int ii = 0; ii < limited_number_of_trailers; ii++)
	{
		trailer_nlp_config_all.semi_trailers[ii].d = semi_trailer_config.semi_trailers[ii].d;
		trailer_nlp_config_all.semi_trailers[ii].distance_between_axle_and_front = semi_trailer_config.semi_trailers[ii].distance_between_axle_and_front;
		trailer_nlp_config_all.semi_trailers[ii].distance_between_axle_and_back = semi_trailer_config.semi_trailers[ii].distance_between_axle_and_back;
		trailer_nlp_config_all.semi_trailers[ii].M = semi_trailer_config.semi_trailers[ii].M;
	}
	return trailer_nlp_config_all;
}

carmen_trailer_nlp_initial_state_t
get_initial_state(const carmen_robot_and_trailers_traj_point_t &current_pose, int limited_number_of_trailers)
{
	carmen_trailer_nlp_initial_state_t initial_state;
	initial_state.x = current_pose.x;
	initial_state.y = current_pose.y;
	initial_state.theta[0] = current_pose.theta;
	initial_state.phi = current_pose.phi;
	initial_state.v = 0.0;
	initial_state.a = 0.0;
	initial_state.w = 0.0;
	for (int ii = 0; ii < limited_number_of_trailers; ii++)
		initial_state.theta[ii + 1] = current_pose.trailer_theta[ii];
	return initial_state;
}

carmen_trailer_nlp_final_state_t
get_final_state(const carmen_robot_and_trailers_traj_point_t &goal_pose, int limited_number_of_trailers)
{
	carmen_trailer_nlp_final_state_t final_state;
	final_state.x = goal_pose.x;
	final_state.y = goal_pose.y;
	final_state.theta[0] = goal_pose.theta;
	final_state.v = 0.0;
	final_state.a = 0.0;
	final_state.w = 0.0;
	for (int ii = 0; ii < limited_number_of_trailers; ii++)
		final_state.theta[ii + 1] = goal_pose.trailer_theta[ii];
	return final_state;
}

carmen_trailer_nlp_initial_guess_t
get_initial_guess(const carmen_robot_and_trailers_traj_point_t &current_pose, const carmen_robot_and_trailers_traj_point_t &goal_pose,
				  const carmen_semi_trailers_config_t &semi_trailer_config, int limited_number_of_trailers)
{
	carmen_trailer_nlp_initial_guess_t initial_guess;

	size_t linhas = sizeof(initial_guess.poses) / sizeof(initial_guess.poses[0]);
	size_t colunas = sizeof(initial_guess.poses[0]) / sizeof(initial_guess.poses[0][0]);
	printf("Linhas: %zu \n", linhas);
	printf("colunas: %zu \n", colunas);

	double dx = (goal_pose.x - current_pose.x) / (double)NUM_TIME_STEPS;
	double dy = (goal_pose.y - current_pose.y) / (double)NUM_TIME_STEPS;
	double dtheta1 = carmen_normalize_theta(goal_pose.theta - current_pose.theta) / (double)NUM_TIME_STEPS;

	for (int i = 0; i < NUM_TIME_STEPS; i++)
	{
		double x1 = current_pose.x + dx * (double)i;
		double y1 = current_pose.y + dy * (double)i;
		double theta1 = carmen_normalize_theta(current_pose.theta + dtheta1 * (double)i);
		initial_guess.poses[i][0].x = x1;
		initial_guess.poses[i][0].y = y1;
		initial_guess.poses[i][0].theta = theta1;
		for (int ii = 0; ii < limited_number_of_trailers; ii++)
		{
			double current_theta2 = current_pose.trailer_theta[ii];
			double goal_theta2 = goal_pose.trailer_theta[ii];
			double dtheta2 = carmen_normalize_theta(goal_theta2 - current_theta2) / (double)NUM_TIME_STEPS;
			double theta2 = carmen_normalize_theta(current_theta2 + dtheta2 * (double)i);
			initial_guess.poses[i][ii + 1].theta = theta2;
		}
	}

	for (int ii = 0; ii < limited_number_of_trailers; ii++)
	{
		double d = semi_trailer_config.semi_trailers[ii].d;
		double M = semi_trailer_config.semi_trailers[ii].M;

		for (int i = 0; i < NUM_TIME_STEPS; i++)
		{
			if (ii == 0)
			{
				double current_theta2 = current_pose.trailer_theta[ii];
				double goal_theta2 = goal_pose.trailer_theta[ii];
				double dtheta2 = carmen_normalize_theta(goal_theta2 - current_theta2) / (double)NUM_TIME_STEPS;
				double x1 = current_pose.x + dx * (double)i;
				double y1 = current_pose.y + dy * (double)i;
				double theta1 = carmen_normalize_theta(current_pose.theta + dtheta1 * (double)i);
				double theta2 = carmen_normalize_theta(current_theta2 + dtheta2 * (double)i);
				initial_guess.poses[i][ii + 1].x = x1 - M * cos(theta1) - d * cos(theta2);
				initial_guess.poses[i][ii + 1].y = y1 - M * sin(theta1) - d * sin(theta2);
			}
			else
			{
				initial_guess.poses[i][ii + 1].x = initial_guess.poses[i][ii].x - M * cos(current_pose.trailer_theta[ii - 1]) - d * cos(current_pose.trailer_theta[ii]);
				initial_guess.poses[i][ii + 1].y = initial_guess.poses[i][ii].y - M * sin(current_pose.trailer_theta[ii - 1]) - d * sin(current_pose.trailer_theta[ii]);
			}
		}
	}
	return initial_guess;
}







inline bool file_exists(const std::string& name) {
    return (access(name.c_str(), F_OK) != -1);
}

static bool
save_and_plot_solution(const casadi::DM& theta_dm,
                       const std::vector<double>& v,
                       const std::vector<double>& phi,
                       const std::vector<double>& a,
                       const std::vector<double>& w,
                       double dt,
                       const std::string& csv_path,
                       bool plot_with_gnuplot)
{
    // --- Lógica Corrigida para Nome Único ---
    std::string final_csv_path = csv_path;

    if (file_exists(final_csv_path)) {
        std::string base_name = csv_path;
        std::string extension = "";

        size_t last_dot = csv_path.find_last_of(".");
        size_t last_slash = csv_path.find_last_of("/\\");

        if (last_dot != std::string::npos && 
           (last_slash == std::string::npos || last_dot > last_slash)) {
            base_name = csv_path.substr(0, last_dot);
            extension = csv_path.substr(last_dot);
        }

        int counter = 1;
        // Tenta novos nomes até encontrar um que não exista
        while (true) {
            std::stringstream ss;
            ss << base_name << "_" << counter << extension;
            std::string candidate = ss.str();
            
            if (!file_exists(candidate)) {
                final_csv_path = candidate;
                break;
            }
            counter++;
            
            // Segurança para evitar loop infinito
            if(counter > 1000) break; 
        }
    }
    // --------------------------------------------

    int n_theta = (int)theta_dm.size1();
    int m_theta = (int)theta_dm.size2();

    std::size_t n = (std::size_t)n_theta;
    if(v.size() < n) n = v.size();
    if(phi.size() < n) n = phi.size();
    if(a.size() < n) n = a.size();
    if(w.size() < n) n = w.size();

    if(n == 0 || n_theta <= 0 || m_theta <= 0 || dt <= 0.0) return false;

    // IMPORTANTE: f agora abre o final_csv_path
    std::ofstream f(final_csv_path.c_str());
    if(!f.is_open()) return false;

    // (Opcional) Debug para ver no console qual arquivo está sendo criado
    // std::cout << "Salvando em: " << final_csv_path << std::endl;

    f << "t";
    for(int j = 0; j < m_theta; j++) f << ",theta_" << j;
    f << ",v,phi,a,w\n";

    for(std::size_t i = 0; i < n; i++) {
        f << (double)i * dt;
        for(int j = 0; j < m_theta; j++) {
            f << "," << (double)theta_dm((int)i, j);
        }
        f << "," << v[i] << "," << phi[i] << "," << a[i] << "," << w[i] << "\n";
    }
    f.close();

    if(plot_with_gnuplot) {
        // Criamos o script .gp baseado no nome final do CSV
        std::string gp_path = final_csv_path + ".gp";
        std::ofstream gp(gp_path.c_str());
        if(!gp.is_open()) return false;

        gp << "set datafile separator ','\nset key left top\nset grid\n";
        
        // Plot theta
        gp << "set terminal qt 0\nset title 'theta'\nplot ";
        for(int j = 0; j < m_theta; j++) {
            gp << "'" << final_csv_path << "' using 1:" << (2 + j) 
               << " with lines title 'theta_" << j << (j == m_theta - 1 ? "" : ", ");
        }
        gp << "\npause -1\n";

        // Plots das outras variáveis
        int base_col = 2 + m_theta;
        const char* names[] = {"v", "phi", "a", "w"};
        for(int i = 0; i < 4; i++) {
            gp << "set terminal qt " << (i + 1) << "\nset title '" << names[i] << "'\n";
            gp << "plot '" << final_csv_path << "' using 1:" << (base_col + i) << " with lines title '" << names[i] << "'\n";
            gp << "pause -1\n";
        }
        gp.close();

        std::string cmd = "gnuplot '" + gp_path + "'";
        return (std::system(cmd.c_str()) == 0);
    }

    return true;
}



bool run_nlp_planner(const carmen_trailer_nlp_initial_guess_t &initial_guess, const carmen_trailer_nlp_initial_state_t &initial_state, const carmen_trailer_nlp_final_state_t &final_state,
					 const carmen_trailer_nlp_config_all_t &robot_params, vector<carmen_robot_and_trailers_traj_point_t> &trajectory, int limited_number_of_trailers)
{
	auto num_tractors_plus_trailers = limited_number_of_trailers + 1;

	auto opti = casadi::Opti();

	auto max_tf = sqrt(pow(initial_state.x - final_state.x, 2.0) + pow(initial_state.y - final_state.y, 2.0)) * 5.0;

	// ########### Declaration of the variables ###########
	auto x = opti.variable(NUM_TIME_STEPS, num_tractors_plus_trailers);
	auto y = opti.variable(NUM_TIME_STEPS, num_tractors_plus_trailers);
	auto theta = opti.variable(NUM_TIME_STEPS, num_tractors_plus_trailers);
	auto v = opti.variable(NUM_TIME_STEPS);
	auto a = opti.variable(NUM_TIME_STEPS);
	auto phi = opti.variable(NUM_TIME_STEPS); 
	auto w = opti.variable(NUM_TIME_STEPS); // derivada de phi
	auto tf = opti.variable();
	auto dt = opti.variable();

	// ########### Initial guess ###########
	opti.set_initial(tf, max_tf / 8.0);
	for (int pp = 0; pp < num_tractors_plus_trailers; pp++)
	{
		for (int i = 0; i < NUM_TIME_STEPS; i++)
		{
			// printf("%d %d %d\n", pp, i, NUM_TIME_STEPS);
			// printf("%lf %lf %lf\n", initial_guess.poses[i][pp].x, initial_guess.poses[i][pp].y, initial_guess.poses[i][pp].theta);
			opti.set_initial(x(i, pp), initial_guess.poses[i][pp].x);
			opti.set_initial(y(i, pp), initial_guess.poses[i][pp].y);
			opti.set_initial(theta(i, pp), initial_guess.poses[i][pp].theta);
		}
	}
	// ########### Total time constraint ###########
	// s.t. total_time:
	opti.subject_to(opti.bounded(max_tf / 8.0, tf, max_tf));

	// ########### Upper bound constraints ###########
	for (int i = 0; i < NUM_TIME_STEPS; i++)
	{
		// s.t. Bonds_v {i in TIME_STEPS}:
		opti.subject_to(pow(v(i), 2.0) <= pow(robot_params.vmax, 2.0));
		// s.t. Bonds_a {i in TIME_STEPS}:
		opti.subject_to(pow(a(i), 2.0) <= pow(robot_params.amax, 2.0));
		// s.t. Bonds_w {i in TIME_STEPS}:
		opti.subject_to(pow(w(i), 2.0) <= pow(robot_params.wmax * (1.0 - 0.97 * (i / (double)NUM_TIME_STEPS)), 2.0));
		// s.t. Bonds_phi {i in TIME_STEPS}:
		opti.subject_to(pow(phi(i), 2.0) <= pow(robot_params.phimax, 2.0));
	}

	// s.t. Jerk_avoidance {pp in {1..(num_tractors_plus_trailers-1)},i in TIME_STEPS}:
	for (int pp = 0; pp < num_tractors_plus_trailers - 1; pp++)
	{
		for (int i = 0; i < NUM_TIME_STEPS; i++)
		{
			opti.subject_to(pow(theta(i, pp) - theta(i, pp + 1), 2.0) <= 2.4649); // TODO ler o max_beta do carmen_semi_trailer_config_t (maximo 90 graus)
		}
	}

	// ########### Boundary constraints ###########
	opti.subject_to(dt == tf / NUM_TIME_STEPS);
	// s.t. EQ_starting_x :
	opti.subject_to(x(0, 0) == initial_state.x);
	// s.t. EQ_starting_y :
	opti.subject_to(y(0, 0) == initial_state.y);
	// s.t. EQ_starting_theta_N :
	for (int pp = 0; pp < num_tractors_plus_trailers; pp++)
		opti.subject_to(theta(0, pp) == initial_state.theta[pp]);
	// s.t. EQ_starting_phi :
	opti.subject_to(phi(0) == initial_state.phi);
	// s.t. EQ_starting_v :
	opti.subject_to(v(0) == initial_state.v);
	// s.t. EQ_starting_a :
	opti.subject_to(a(0) == initial_state.a);
	// s.t. EQ_starting_w :
	opti.subject_to(w(0) == initial_state.w);

	// s.t. Eq_end_x :
	opti.subject_to(pow(x(NUM_TIME_STEPS - 1, 0) - final_state.x, 2.0) <= 0.001);
	// s.t. Eq_end_y :
	opti.subject_to(pow(y(NUM_TIME_STEPS - 1, 0) - final_state.y, 2.0) <= 0.001);
	// s.t. Eq_end_theta_N :
	for (int pp = 0; pp < num_tractors_plus_trailers; pp++)
		opti.subject_to(pow(theta(NUM_TIME_STEPS - 1, pp) - final_state.theta[pp], 2.0) <= 0.00001);
	// s.t. EQ_ending_v :
	opti.subject_to(pow(v(NUM_TIME_STEPS - 1) - final_state.v, 2.0) <= 0.005);
	// s.t. EQ_ending_a :
	opti.subject_to(pow(a(NUM_TIME_STEPS - 1) - final_state.a, 2.0) <= 0.001);
	// s.t. EQ_ending_w :
	opti.subject_to(pow(w(NUM_TIME_STEPS - 1) - final_state.w, 2.0) <= 0.0001);

	// ########### ODEs for tractor ###########
	for (int i = 1; i < NUM_TIME_STEPS; i++)
	{
		// s.t. DIFF_dx1dt {i in {2..num_time_steps}}:
		opti.subject_to(x(i, 0) - x(i - 1, 0) == dt * v(i - 1) * cos(theta(i - 1, 0)));
		// s.t. DIFF_dy1dt {i in {2..num_time_steps}}:
		opti.subject_to(y(i, 0) - y(i - 1, 0) == dt * v(i - 1) * sin(theta(i - 1, 0)));
		// s.t. DIFF_dvdt {i in {2..num_time_steps}}:
		opti.subject_to(v(i) - v(i - 1) == dt * a(i - 1));
		// s.t. DIFF_dthetadt {i in {2..num_time_steps}}:
		opti.subject_to(theta(i, 0) - theta(i - 1, 0) == dt * tan(phi(i - 1)) * v(i - 1) / robot_params.distance_between_front_and_rear_axles);
		// s.t. DIFF_dphidt {i in {2..num_time_steps}}:
		opti.subject_to(phi(i) - phi(i - 1) == dt * w(i - 1));
	}

	// ########### ODEs for the first trailer ###########
	for (int i = 1; i < NUM_TIME_STEPS; i++)
	{
		// s.t. DIFF_dtheta2dt {i in {2..num_time_steps}}:
		opti.subject_to(theta(i, 1) - theta(i - 1, 1) == dt * v(i - 1) * ((sin(theta(i - 1, 0) - theta(i - 1, 1)) / robot_params.semi_trailers[0].d) - (robot_params.semi_trailers[0].M / (robot_params.distance_between_front_and_rear_axles * robot_params.semi_trailers[0].d)) * cos(theta(i - 1, 0) - theta(i - 1, 1)) * tan(phi(i - 1))));
	}

	//	//########### ODEs for the remaining trailers ###########
	//	for (int pp=2; pp < num_tractors_plus_trailers; pp++)
	//	{
	//		for (int i=1; i < NUM_TIME_STEPS; i++)
	//		{
	//			//s.t. DIFF_dthetaNdt {i in {2..num_time_steps}}:
	//			opti.subject_to( theta(i,pp) - theta(i-1,pp) == (dt * v(i-1) / robot_params.semi_trailers[pp-1].d) * cos(theta(i-1,pp-1) - theta(i-1,pp)) * sin(theta(i-1,pp-1) - theta(i-1,pp)) );
	//		}
	//	}
	// ########### ODEs for the remaining trailers ###########
	for (int pp = 2; pp < num_tractors_plus_trailers; pp++)
	{
		casadi::MX betas[pp];
		int real_trailer_index = pp - 1;
		for (int i = 1; i < NUM_TIME_STEPS; i++)
		{
			betas[0] = phi(i - 1);
			for (int z = 1; z < pp; z++)
				betas[z] = theta(i - 1, z - 1) - theta(i - 1, z);

			casadi::MX product = 1.0;
			for (int k = 0; k < (pp); k++)
				product = product * (cos(betas[k]));

			casadi::MX product2 = 1.0;
			for (int k = 0; k < (pp - 1); k++)
				product2 = product2 * (1 + (robot_params.semi_trailers[k + 1].M / robot_params.semi_trailers[k].d) * tan(betas[k]) * tan(betas[k + 1])); // Tem que ser + 1, pois o item 0 do beta é o phi, e queremos o beta anterior e atual no tan

			casadi::MX restante = (sin(theta(i - 1, pp - 1) - theta(i - 1, pp)) / robot_params.semi_trailers[real_trailer_index].d) -
								  (robot_params.semi_trailers[real_trailer_index].M / (robot_params.semi_trailers[real_trailer_index - 1].d * robot_params.semi_trailers[real_trailer_index].d)) *
									  (tan(betas[real_trailer_index])) * cos(theta(i - 1, pp - 1) - theta(i - 1, pp));

			// s.t. DIFF_dthetaNdt {i in {2..num_time_steps}}:
			opti.subject_to(theta(i, pp) - theta(i - 1, pp) == (dt * v(i - 1) * product * product2 * restante));
		}
	}

	// ########### ODEs for the second trailer ###########
	//	for (int i=1; i < NUM_TIME_STEPS; i++) {
	//	    opti.subject_to(theta(i,2) - theta(i-1,2) == dt * v(i-1) * ((sin(theta(i-1,1) - theta(i-1,2)) / robot_params.semi_trailers[1].d) -
	//	        (robot_params.semi_trailers[1].M / (robot_params.distance_between_front_and_rear_axles * robot_params.semi_trailers[1].d)) *
	//	        cos(theta(i-1,1) -
	//	        		theta(i-1,2)) *
	//					tan(phi(i-1))));
	//	}

	//	//########### ODEs for the remaining trailers ###########
	//	for (int pp=3; pp < num_tractors_plus_trailers; pp++) {
	//	    for (int i=1; i < NUM_TIME_STEPS; i++) {
	//	    	casadi::MX aux = 1 + (robot_params.semi_trailers[2].M  / robot_params.semi_trailers[1].d) * tan(theta(i-1,1) - theta(i-1,(1+1))) * tan(phi(i-1));
	//	    	casadi::MX product2 = 1.0;
	//	        for (int jj=1; jj <= pp-2; jj++) {
	//	        	casadi::MX factor = 1 + (robot_params.semi_trailers[jj + 1].M / robot_params.semi_trailers[jj].d) * tan(theta(i-1,jj) - theta(i-1,(jj+1))) * tan(theta(i-1,jj-1) - theta(i-1,(jj)));
	//	            product2 *= factor;
	//	        }
	//	        casadi::MX factor1 = sin(theta(i-1,pp-1) - theta(i-1,pp)) / robot_params.semi_trailers[pp - 2].d - (robot_params.semi_trailers[pp - 2].M / (robot_params.semi_trailers[pp - 1].M) * robot_params.semi_trailers[pp - 2].d);
	//	        casadi::MX factor2 = cos(theta(i-1,pp-1) - theta(i-1,pp)) * tan(theta(i-1,pp-2) - theta(i-1,pp-1));
	//	        opti.subject_to(theta(i,pp) - theta(i-1,pp) == dt * v(i-1) * cos(phi(i-1)) * product2 * aux * factor1 * factor2);
	//	    }
	//	}

	// ########### AEs for the whole systems ###########
	for (int pp = 0; pp < num_tractors_plus_trailers - 1; pp++)
	{
		for (int i = 0; i < NUM_TIME_STEPS; i++)
		{
			// s.t. Geometric_x_general {i in TIME_STEPS}:
			opti.subject_to(x(i, pp + 1) == x(i, pp) - robot_params.semi_trailers[pp].d * cos(theta(i, pp + 1)) - robot_params.semi_trailers[pp].M * cos(theta(i, pp)));
			// s.t. Geometric_y_general {i in TIME_STEPS}:
			opti.subject_to(y(i, pp + 1) == y(i, pp) - robot_params.semi_trailers[pp].d * sin(theta(i, pp + 1)) - robot_params.semi_trailers[pp].M * sin(theta(i, pp)));
		}
	}

	opti.minimize(tf);

	casadi::Dict casadi_options;
	casadi_options["print_time"] = false;
	casadi::Dict solver_options;
	solver_options["sb"] = "yes";
	solver_options["max_cpu_time"] = 15;
	solver_options["bound_push"] = 0.0001;
	solver_options["linear_solver"] = "mumps";
	solver_options["print_level"] = 0;
	opti.solver("ipopt", casadi_options, solver_options);

	try
	{
		auto sol = opti.solve();

		casadi::DM theta_dm = sol.value(theta);

		vector<double> sol_x(sol.value(x));
		vector<double> sol_y(sol.value(y));
		vector<double> sol_theta(sol.value(theta));
		vector<double> sol_v(sol.value(v));
		vector<double> sol_phi(sol.value(phi));
		vector<double> sol_a(sol.value(a));
		vector<double> sol_w(sol.value(w));

		casadi::DM dt_dm = sol.value(dt);
		double dt_val = dt_dm.scalar();

		save_and_plot_solution(theta_dm, sol_v, sol_phi, sol_a, sol_w, dt_val, "/dados/ype_audits/solucao_nlp.csv", false);
		
		
		for (unsigned int i = 0; i < sol_v.size(); i++)
		{
			carmen_robot_and_trailers_traj_point_t traj_point = {sol_x[i], sol_y[i], sol_theta[i], 1, {0.0}, sol_v[i], sol_phi[i]};
			for (int ii = 0; ii < limited_number_of_trailers; ii++)
			{
				traj_point.trailer_theta[ii] = sol_theta[i + (sol_v.size() * (ii + 1))];
			}
			trajectory.push_back(traj_point);
		}

		return true;
	}
	catch (casadi::CasadiException const &)
	{
		return false;
	}
}

carmen_robot_and_trailers_traj_point_t
change_pose_to_relative_coordinates(const carmen_robot_and_trailers_traj_point_t &reference_pose, const carmen_robot_and_trailers_traj_point_t &pose, int limited_number_of_trailers)
{
	g2o::SE2 reference_pose_se2(reference_pose.x, reference_pose.y, reference_pose.theta);
	g2o::SE2 pose_se2(pose.x, pose.y, pose.theta);
	g2o::SE2 pose_in_relative_coordinates_se2 = reference_pose_se2.inverse() * pose_se2;

	carmen_robot_and_trailers_traj_point_t pose_in_relative_coordinates = pose;
	pose_in_relative_coordinates.x = pose_in_relative_coordinates_se2[0];
	pose_in_relative_coordinates.y = pose_in_relative_coordinates_se2[1];
	pose_in_relative_coordinates.theta = pose_in_relative_coordinates_se2[2];

	for (size_t z = 0; z < limited_number_of_trailers; z++)
	{
		double current_beta = convert_theta1_to_beta(pose.theta, pose.trailer_theta[z]); // Necessário para trocar a referência do trailer_theta
		pose_in_relative_coordinates.trailer_theta[z] = convert_beta_to_theta1(pose_in_relative_coordinates.theta, current_beta);
	}

	return (pose_in_relative_coordinates);
}

void change_path_to_absolute_coordinates(vector<carmen_robot_and_trailers_traj_point_t> &path, const carmen_robot_and_trailers_traj_point_t &pose, int limited_number_of_trailers)
{
	for (std::vector<carmen_robot_and_trailers_traj_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		double old_theta = it->theta;
		double x = pose.x + it->x * cos(pose.theta) - it->y * sin(pose.theta);
		double y = pose.y + it->x * sin(pose.theta) + it->y * cos(pose.theta);
		it->x = x;
		it->y = y;
		it->theta = carmen_normalize_theta(it->theta + pose.theta);

		for (size_t j = 0; j < limited_number_of_trailers; j++)
		{
			double beta = convert_theta1_to_beta(old_theta, it->trailer_theta[j]);
			it->trailer_theta[j] = convert_beta_to_theta1(it->theta, beta);
		}
	}
}

vector<carmen_robot_and_trailers_traj_point_t>
trailer_nlp_analytical_expansion_casadi(const carmen_robot_and_trailers_traj_point_t &current_pose, const carmen_robot_and_trailers_traj_point_t &goal_pose,
										const carmen_robot_ackerman_config_t &robot_config, const carmen_semi_trailers_config_t &semi_trailer_config, int limited_number_of_trailers)
{

	vector<carmen_robot_and_trailers_traj_point_t> trajectory;

	carmen_trailer_nlp_config_all_t trailer_nlp_config_all = get_robot_configuration(robot_config, semi_trailer_config, limited_number_of_trailers);

	carmen_robot_and_trailers_traj_point_t current_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, current_pose, limited_number_of_trailers);
	carmen_robot_and_trailers_traj_point_t goal_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, goal_pose, limited_number_of_trailers);

	carmen_trailer_nlp_initial_state_t initial_state = get_initial_state(current_pose_in_relative_coordinates, limited_number_of_trailers);
	carmen_trailer_nlp_final_state_t final_state = get_final_state(goal_pose_in_relative_coordinates, limited_number_of_trailers);
	carmen_trailer_nlp_initial_guess_t initial_guess = get_initial_guess(current_pose_in_relative_coordinates, goal_pose_in_relative_coordinates, semi_trailer_config, limited_number_of_trailers);

	bool plan_ok = run_nlp_planner(initial_guess, initial_state, final_state, trailer_nlp_config_all, trajectory, limited_number_of_trailers);
	if (!plan_ok)
		return trajectory;

	change_path_to_absolute_coordinates(trajectory, current_pose, limited_number_of_trailers);

	return trajectory;
}
