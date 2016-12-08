/*
#include "mpc.h"


void
plot_velocity(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_velocity, PARAMS *params)
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
	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(descriptors);

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
plot_phi(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_phi, PARAMS *params)
{
	#define PAST_SIZE 300
	static list<double> cphi_vector;
	static list<double> dphi_vector;
	static list<double> effort_vector;
	static bool first_time = true;
	static FILE *gnuplot_pipe;
	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");
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
	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(descriptors);

	for (unsigned int i = 0; i < params->optimized_path.phi.size(); i++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n", time, params->optimized_path.phi[i], params->path.phi[i], future_effort_vector[i], 8, 2);
		time += DELTA_T;
	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' using 1:2:5 with lines linecolor variable title 'cphi' axes x1y2,"
			"'./gnuplot_data.txt' using 1:3:6 with lines linecolor variable title 'dphi' axes x1y2,"
			"'./gnuplot_data.txt' using 1:4 with lines title 'effort' axes x1y1\n");

	fflush(gnuplot_pipe);

	dphi_vector.push_front(params->optimized_path.phi[0]);
	cphi_vector.push_front(current_phi);
	effort_vector.push_front(descriptors->k1);

}
*/
