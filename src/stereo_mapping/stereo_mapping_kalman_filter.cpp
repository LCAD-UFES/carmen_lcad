#include "stereo_mapping_kalman_filter.h"

//auxiliar function to print the variables in a graph
#include <list>
void
data_plot_curvature(double measured_value, double correctedmeasured_value)
{
#define PAST_SIZE 1000
	static std::list<double> measured;
	static std::list<double> corrected;
	static std::list<double> timestamp;
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;
	std::list<double>::iterator itc;
	std::list<double>::iterator itd;
	std::list<double>::iterator itt;

	double t = carmen_get_time();
	if (first_time)
	{
		first_timestamp = t;
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); //("gnuplot -persist", "w") to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [0:30]\n");
		fprintf(gnuplot_pipe, "set yrange [-1.0:1.0]\n");
	}

	measured.push_front(measured_value);
	corrected.push_front(correctedmeasured_value);
	timestamp.push_front(t - first_timestamp);

	while(measured.size() > PAST_SIZE)
	{
		measured.pop_back();
		corrected.pop_back();
		timestamp.pop_back();
	}


	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");

	for (itc = measured.begin(), itd = corrected.begin(), itt = timestamp.begin(); itc != measured.end(); itc++, itd++, itt++)
		fprintf(gnuplot_data_file, "%lf %lf %lf\n", *itt - timestamp.back(), *itc, *itd);


	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' using 1:2 with lines title 'measured',"
			"'./gnuplot_data.txt' using 1:3 with lines title 'corrected'\n");

	fflush(gnuplot_pipe);
}


void
kalman_update_state(cv::KalmanFilter *filter, kalman_filter_params *state[], double measurements[])
{
	// First predict, to update the internal statePre variable
	cv::Mat prediction = filter->predict();

	// Get measurements
	cv::Mat_<double> measurement(2,1);
	measurement(0) = measurements[0];
	measurement(1) = measurements[1];

	// The update phase
	cv::Mat corrected = filter->correct(measurement);
	state[0]->value = corrected.at<double>(0);
	state[1]->value = corrected.at<double>(1);

	//end of kalman filter update, printing to check
	fprintf(stdout, "Measurement = [%f°, %f m]\n", carmen_radians_to_degrees(measurements[0]), measurements[1]);
	fprintf(stdout, "Predicted   = [%f°, %f m]\n", carmen_radians_to_degrees(prediction.at<double>(0)), prediction.at<double>(1));
	fprintf(stdout, "Corrected   = [%f°, %f m]\n", carmen_radians_to_degrees(corrected.at<double>(0)), corrected.at<double>(1));

	printf("State: %f, %f\n",state[0]->value,state[1]->value );
	// it's working! yay!
	data_plot_curvature(measurements[0],state[0]->value);
}


void init_kalman_filter_params(kalman_filter_params *state, double value_variance, double value_variance_factor, double observation_variance)
{
	state->value_variance = value_variance;
	state->value_variance_factor = value_variance_factor;
	state->observation_variance = observation_variance;
	state->filter_outliers = true;
}
