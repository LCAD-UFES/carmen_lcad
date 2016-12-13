#include "mpc.h"


void
plot_velocity(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_velocity, PARAMS *params, double prediction_horizon)
{
	#define PAST_SIZE 300
	static list<double> current_velocity_vector;
	static list<double> desired_velocity_vector;
	static list<double> effort_vector;
	static bool first_time = true;
	static FILE *gnuplot_pipe;
	FILE *gnuplot_data_file = fopen("gnuplot_velocity.txt", "w");
	double time = 0.0;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [0:PAST_SIZE/20]\n");
		fprintf(gnuplot_pipe, "set yrange [-110.0:110.0]\n");
		fprintf(gnuplot_pipe, "set y2range [0:16]\n");
		fprintf(gnuplot_pipe, "set xlabel 'senconds'\n");
		fprintf(gnuplot_pipe, "set ylabel 'effort'\n");
		fprintf(gnuplot_pipe, "set y2label 'Velocity (ms)'\n");
		fprintf(gnuplot_pipe, "set ytics nomirror\n");
		fprintf(gnuplot_pipe, "set y2tics\n");
		fprintf(gnuplot_pipe, "set tics out\n");
	}

	while (current_velocity_vector.size() > PAST_SIZE)
	{
		current_velocity_vector.pop_back();
		desired_velocity_vector.pop_back();
		effort_vector.pop_back();
	}

	// PAST DATA
	list<double>::reverse_iterator it_cphi, it_dphi, it_effort;
	for (it_cphi = current_velocity_vector.rbegin(), it_dphi = desired_velocity_vector.rbegin(), it_effort = effort_vector.rbegin();
		 it_cphi != current_velocity_vector.rend();
		 it_cphi++, it_dphi++, it_effort++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n", time, *it_cphi, *it_dphi, *it_effort, 1, 2); //1-red 2-green 3-blue 4-magenta 5-lightblue 6-yellow 7-black 8-orange 9-grey
		time += DELTA_T;
	}

	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, graph 0 to %lf, graph 1 nohead\n", time, time);

	// FUTURE DATA
	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(descriptors, prediction_horizon);

	for (unsigned int i = 0; i < params->optimized_path.v.size(); i++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n", time, params->optimized_path.v[i], params->path.v[i], future_effort_vector[i], 8, 2);
		time += DELTA_T;
	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_velocity.txt' using 1:2:5 with lines linecolor variable title 'Cvel' axes x1y2,"
			"'./gnuplot_velocity.txt' using 1:3:6 with lines linecolor variable title 'Dvel' axes x1y2,"
			"'./gnuplot_velocity.txt' using 1:4 with lines title 'effort' axes x1y1\n");

	fflush(gnuplot_pipe);

	desired_velocity_vector.push_front(params->path.v[0]);
	current_velocity_vector.push_front(current_velocity);
	effort_vector.push_front(descriptors->k1);

}


void
plot_phi(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_phi, PARAMS *params, double prediction_horizon)
{
	#define PAST_SIZE 300
	static list<double> cphi_vector;
	static list<double> dphi_vector;
	static list<double> effort_vector;
	static bool first_time = true;
	static FILE *gnuplot_pipe;
	FILE *gnuplot_data_file = fopen("gnuplot_phi.txt", "w");
	double time = 0.0;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [0:PAST_SIZE/20]\n");
		fprintf(gnuplot_pipe, "set yrange [-110.0:110.0]\n");
		fprintf(gnuplot_pipe, "set y2range [-0.55:0.55]\n");
		fprintf(gnuplot_pipe, "set xlabel 'senconds'\n");
		fprintf(gnuplot_pipe, "set ylabel 'effort'\n");
		fprintf(gnuplot_pipe, "set y2label 'phi (radians)'\n");
		fprintf(gnuplot_pipe, "set ytics nomirror\n");
		fprintf(gnuplot_pipe, "set y2tics\n");
		fprintf(gnuplot_pipe, "set tics out\n");
	}

	while (cphi_vector.size() > PAST_SIZE)
	{
		cphi_vector.pop_back();
		dphi_vector.pop_back();
		effort_vector.pop_back();
	}

	// PAST DATA
	list<double>::reverse_iterator it_cphi, it_dphi, it_effort;
	for (it_cphi = cphi_vector.rbegin(), it_dphi = dphi_vector.rbegin(), it_effort = effort_vector.rbegin();
		 it_cphi != cphi_vector.rend();
		 it_cphi++, it_dphi++, it_effort++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n", time, *it_cphi, *it_dphi, *it_effort, 1, 2); //1-red 2-green 3-blue 4-magenta 5-lightblue 6-yellow 7-black 8-orange 9-grey
		time += DELTA_T;
	}

	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, graph 0 to %lf, graph 1 nohead\n", time, time);

	// FUTURE DATA
	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(descriptors, prediction_horizon);

	//printf("--------%ld %ld %ld\n", params->optimized_path.phi.size(), params->path.phi.size(), future_effort_vector.size());
	for (unsigned int i = 0; i < params->optimized_path.phi.size(); i++)
	{
		//printf("%lf %lf %lf %lf\n", time, params->optimized_path.phi[i], params->path.phi[i], future_effort_vector[i]);
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n", time, params->optimized_path.phi[i], params->path.phi[i], future_effort_vector[i], 8, 2);
		time += DELTA_T;
	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_phi.txt' using 1:2:5 with lines linecolor variable title 'cphi' axes x1y2,"
			"'./gnuplot_phi.txt' using 1:3:6 with lines linecolor variable title 'dphi' axes x1y2,"
			"'./gnuplot_phi.txt' using 1:4 with lines title 'effort' axes x1y1\n");

	fflush(gnuplot_pipe);

	dphi_vector.push_front(params->optimized_path.phi[0]);
	cphi_vector.push_front(current_phi);
	effort_vector.push_front(descriptors->k1);

}


void
plot_position(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_x, double current_y, PARAMS *params)
{
	#define POSITION_PAST_SIZE 100
	static list<double> cx_vector;
	static list<double> cy_vector;
	static list<double> dx_vector;
	static list<double> dy_vector;
	static list<double> effort_vector;
	static bool first_time = true;
	static FILE *gnuplot_pipe;
	FILE *gnuplot_data_file = fopen("gnuplot_position.txt", "w");

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		//fprintf(gnuplot_pipe, "set xrange [0:POSITION_PAST_SIZE/20]\n");
		//fprintf(gnuplot_pipe, "set yrange [-110.0:110.0]\n");
		//fprintf(gnuplot_pipe, "set y2range [-0.55:0.55]\n");
		fprintf(gnuplot_pipe, "set xlabel 'm'\n");
		fprintf(gnuplot_pipe, "set ylabel 'm'\n");
		fprintf(gnuplot_pipe, "set y2label 'phi (radians)'\n");
		fprintf(gnuplot_pipe, "set ytics nomirror\n");
		fprintf(gnuplot_pipe, "set y2tics\n");
		fprintf(gnuplot_pipe, "set tics out\n");
	}

	while (cx_vector.size() > POSITION_PAST_SIZE)
	{
		cx_vector.pop_back();
		cy_vector.pop_back();
		dx_vector.pop_back();
		dy_vector.pop_back();
		effort_vector.pop_back();
	}

	// PAST DATA
	list<double>::reverse_iterator it_cx, it_cy, it_dx, it_dy, it_effort;
	for (it_cx = cx_vector.rbegin(), it_cy = cy_vector.rbegin(), it_dx = dx_vector.rbegin(), it_dy = dy_vector.rbegin(), it_effort = effort_vector.rbegin();
		 it_cx != cx_vector.rend();
		 it_cx++, it_cy++, it_dx++, it_dy++, it_effort++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %d %d\n", *it_cx, *it_cy, *it_dx, *it_dy, *it_effort, 1, 2); //1-red 2-green 3-blue 4-magenta 5-lightblue 6-yellow 7-black 8-orange 9-grey
	}

	//fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, graph 0 to %lf, graph 1 nohead\n", *it_cx, *it_cx);

	// FUTURE DATA
//	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(descriptors, POSITION_PREDICTION_HORIZON);
//
//	for (unsigned int i = 0; i < params->optimized_path.phi.size(); i++)
//	{
//		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %d %d\n", params->optimized_path.x[i], params->optimized_path.y[i], params->path.x[i], params->path.y[i], future_effort_vector[i], 8, 2);
//	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_position.txt' using 1:2:5 with lines linecolor variable title 'cphi' axes x1y2,"
			"'./gnuplot_position.txt' using 3:4:6 with lines linecolor variable title 'dphi' axes x1y2\n");
//			"'./gnuplot_position.txt' using 5:4 with lines title 'effort' axes x1y1\n");

	fflush(gnuplot_pipe);

	dx_vector.push_front(params->optimized_path.x[0]);
	dy_vector.push_front(params->optimized_path.y[0]);
	cx_vector.push_front(current_x);
	cy_vector.push_front(current_y);
	effort_vector.push_front(descriptors->k1);

}
