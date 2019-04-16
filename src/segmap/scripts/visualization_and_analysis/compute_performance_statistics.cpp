
#include <cstdio>
#include <string>
#include <vector>
#include <carmen/command_line.h>
#include <carmen/util_io.h>
#include <carmen/util_math.h>
#include <g2o/types/slam2d/se2.h>
#include <carmen/ackerman_motion_model.h>

using namespace std;
using namespace g2o;


class ExperimentStatistics
{
public:
	vector<SE2> estimates_in_gt_ref;
	vector<SE2> abs_diff_values;
	vector<double> squared_dist_by_sample;

	double dist_mean, dist_rmse, dist_std;
	double abs_x_mean, x_rmse, abs_x_std;
	double abs_y_mean, y_rmse, abs_y_std;
	double th_mean, th_rmse, th_std;
	double dx_mean, dy_mean, dth_mean;

	double mean_dth_x, mean_dth_y;
	double mean_abs_th_x, mean_abs_th_y;
	double std_abs_th_x, std_abs_th_y;

	double percentage_samples_below_1m;
	double percentage_samples_below_2m;
	double percentage_samples_below_0_5m;
	double percentage_samples_below_0_2m;

	ExperimentStatistics()
	{
		reset();
	}

	void reset()
	{
		x_rmse = abs_x_std = abs_x_mean = 0.0;
		y_rmse = abs_y_std = abs_y_mean = 0.0;
		th_rmse = th_std = th_mean = 0.0;
		dist_rmse = dist_std = dist_mean = 0.0;
		dx_mean = dy_mean = dth_mean = 0.0;
		mean_abs_th_x = mean_abs_th_y = 0.0;
		std_abs_th_x = std_abs_th_y = 0.0;
		mean_dth_x = mean_dth_y = 0.0;

		percentage_samples_below_2m = 0.0;
		percentage_samples_below_1m = 0.0;
		percentage_samples_below_0_5m = 0.0;
		percentage_samples_below_0_2m = 0.0;
	}
};


int
find_nearest(vector<SE2> &gt, SE2 &pose)
{
	int nearest_id = 0;
	double d, min_dist = DBL_MAX;

	for (int i = 0; i < gt.size(); i++)
	{
		d = dist2d(gt[i][0], gt[i][1], pose[0], pose[1]);

		if (d < min_dist)
		{
			min_dist = d;
			nearest_id = i;
		}
	}

	return nearest_id;
}


int
find_most_sync(vector<double> &gt_times, double ref_time)
{
	int nearest_id = 0;
	double d, min_dist = DBL_MAX;

	for (int i = 0; i < gt_times.size(); i++)
	{
		d = fabs(gt_times[i] - ref_time);

		if (d < min_dist)
		{
			min_dist = d;
			nearest_id = i;
		}
	}

	return nearest_id;
}


void
read_groundtruth(const string &path,
                 vector<SE2> *gt,
                 vector<double> *gt_times)
{
	int id, n;
	double x, y, th, time, gps_x, gps_y;

	FILE *f = safe_fopen(path.c_str(), "r");

	while (!feof(f))
	{
		n = fscanf(f, "%d %lf %lf %lf %lf %lf %lf",
		           &id, &x, &y, &th, &time, &gps_x, &gps_y);

		if (n == 7)
		{
			gt->push_back(SE2(x, y, th));
			gt_times->push_back(time);
		}
	}

	fclose(f);
}


void
read_odometry(const string &path,
							vector<SE2> *odometry,
							vector<double> *odometry_times)
{
	int n;
	double v, phi, timestamp;

	FILE *f = safe_fopen(path.c_str(), "r");

	while (!feof(f))
	{
		n = fscanf(f, "%lf %lf %lf", &v, &phi, &timestamp);

		if (n == 3)
		{
			odometry->push_back(SE2(v, phi, 0));
			odometry_times->push_back(timestamp);
		}
	}

	fclose(f);
}


void
read_result(const string &path,
            vector<SE2> *mean,
            vector<SE2> *std,
            vector<SE2> *mode,
            vector<double> *times)
{
	int id, total, n;
	double mean_x, mean_y, mean_th;
	double std_x, std_y, std_th;
	double mode_x, mode_y, mode_th;
	double gt_x, gt_y, gt_th;
	double duration, sample_time;

	FILE *f = safe_fopen(path.c_str(), "r");

	while (!feof(f))
	{
		// in the file produced by carmen, only the mean and std are filled.
		n = fscanf(f, "%d of %d Mean: %lf %lf %lf Std: %lf %lf %lf GT: %lf %lf %lf Mode: %lf %lf %lf Duration: %lf Timestamp: %lf",
		           &id, &total, &mean_x, &mean_y, &mean_th, &std_x, &std_y, &std_th, &gt_x, &gt_y, &gt_th, &mode_x, &mode_y, &mode_th, &duration, &sample_time);

		if (n == 16)
		{
			mode->push_back(SE2(mode_x, mode_y, mode_th));
			mean->push_back(SE2(mean_x, mean_y, mean_th));
			std->push_back(SE2(std_x, std_y, std_th));
			times->push_back(sample_time);
		}
	}

	fclose(f);
}

/*
void
read_result2(const string &path,
            vector<SE2> *mean,
            vector<SE2> *std,
            vector<SE2> *mode,
            vector<double> *times)
{
	int id, total, n;
	double mean_x, mean_y, mean_th;
	double std_x, std_y, std_th;
	double mode_x, mode_y, mode_th;
	double gt_x, gt_y, gt_th;
	double duration, sample_time;

	FILE *f = safe_fopen(path.c_str(), "r");

	while (!feof(f))
	{
		// in the file produced by carmen, only the mean and std are filled.
		n = fscanf(f, "%d of %d Mean: %lf %lf %lf Std: %lf %lf %lf GT: %lf %lf %lf Mode: %lf %lf %lf Duration: %lf",
		           &id, &total, &mean_x, &mean_y, &mean_th, &std_x, &std_y, &std_th, &gt_x, &gt_y, &gt_th, &mode_x, &mode_y, &mode_th, &duration);

		if (n == 15)
		{
			mode->push_back(SE2(mode_x, mode_y, mode_th));
			mean->push_back(SE2(mean_x, mean_y, mean_th));
			std->push_back(SE2(std_x, std_y, std_th));
		}
	}

	fclose(f);
}
*/

SE2
adjust_estimate_pose(SE2 pose, double v, double phi, double dt)
{
	SE2 adjusted_pose;
	double x, y, th;

	x = pose[0];
	y = pose[1];
	th = pose[2];

	ackerman_motion_model(x, y, th, v, phi, dt);
	adjusted_pose = SE2(x, y, th);

	return (adjusted_pose);
}


void
compute_means_and_rmses(ExperimentStatistics &e,
												vector<SE2> &gt,
												vector<double> &gt_times,
												vector<SE2> &means,
												vector<double> &means_times,
												vector<SE2> &odometry)
{
	printf("Computing means and rmses\n");
	int n = means.size();

	// for each pose in the groundtruth, compute the estimated pose
	// in the coordinate system given by the groundtruth pose.
	for (int i = 0; i < means.size(); i++)
	{
		//int gt_id = find_nearest(gt, means[i]);
		int gt_id = find_most_sync(gt_times, means_times[i]);

		double dt = means_times[i] - gt_times[i];
		SE2 adjusted_pose = adjust_estimate_pose(means[i], odometry[i][0], odometry[i][1], dt);
		SE2 est_in_gt_ref = gt[gt_id].inverse() * adjusted_pose;

		double squared_dist = pow(est_in_gt_ref[0], 2) + pow(est_in_gt_ref[1], 2);
		double dist = sqrt(squared_dist);
		double angle_diff = g2o::normalize_theta(gt[gt_id][2] - adjusted_pose[2]);

		e.estimates_in_gt_ref.push_back(est_in_gt_ref);

		e.abs_diff_values.push_back(SE2(fabs(est_in_gt_ref[0]),
		                                fabs(est_in_gt_ref[1]),
		                                fabs(angle_diff)));

		e.squared_dist_by_sample.push_back(squared_dist);

		e.dist_rmse += e.squared_dist_by_sample[i];
		e.dist_mean += sqrt(e.squared_dist_by_sample[i]);

		e.x_rmse += pow(e.estimates_in_gt_ref[i][0], 2);
		e.y_rmse += pow(e.estimates_in_gt_ref[i][1], 2);

		e.abs_x_mean += fabs(e.estimates_in_gt_ref[i][0]);
		e.abs_y_mean += fabs(e.estimates_in_gt_ref[i][1]);

		e.dx_mean += e.estimates_in_gt_ref[i][0];
		e.dy_mean += e.estimates_in_gt_ref[i][1];

		e.mean_abs_th_x += cos(fabs(angle_diff));
		e.mean_abs_th_y += sin(fabs(angle_diff));

		e.mean_dth_x += cos(angle_diff);
		e.mean_dth_y += sin(angle_diff);

		if (dist < 2)
			e.percentage_samples_below_2m++;

		if (dist < 1.0)
			e.percentage_samples_below_1m++;

		if (dist < 0.5)
			e.percentage_samples_below_0_5m++;

		if (dist < 0.2)
			e.percentage_samples_below_0_2m++;
	}

	e.dist_rmse = sqrt(e.dist_rmse / ((double) n));
	e.dist_mean /= ((double) n);

	e.abs_x_mean /= ((double) n);
	e.abs_y_mean /= ((double) n);

	e.dx_mean /= ((double) n);
	e.dy_mean /= ((double) n);

	e.x_rmse = sqrt(e.x_rmse / ((double) n));
	e.y_rmse = sqrt(e.y_rmse / ((double) n));

	e.mean_abs_th_x /= ((double) n);
	e.mean_abs_th_y /= ((double) n);

	e.mean_dth_x /= ((double) n);
	e.mean_dth_y /= ((double) n);

	e.th_mean = g2o::normalize_theta(atan2(e.mean_abs_th_y, e.mean_abs_th_x));
	e.dth_mean = g2o::normalize_theta(atan2(e.mean_dth_y, e.mean_dth_x));

	e.percentage_samples_below_1m /= ((double) means.size());
	e.percentage_samples_below_2m /= ((double) means.size());
	e.percentage_samples_below_0_5m /= ((double) means.size());
	e.percentage_samples_below_0_2m /= ((double) means.size());

	e.percentage_samples_below_1m *= 100.;
	e.percentage_samples_below_2m *= 100.;
	e.percentage_samples_below_0_5m *= 100.;
	e.percentage_samples_below_0_2m *= 100.;
}


void
compute_stds(ExperimentStatistics &e)
{
	printf("Computing stds\n");

	int n = e.estimates_in_gt_ref.size();

	for (int i = 0; i < n; i++)
	{
		e.dist_std += pow(sqrt(e.squared_dist_by_sample[i]) - e.dist_mean, 2);
		e.abs_x_std += pow(fabs(e.estimates_in_gt_ref[i][0]) - e.abs_x_mean, 2);
		e.abs_y_std += pow(fabs(e.estimates_in_gt_ref[i][1]) - e.abs_y_mean, 2);

		e.std_abs_th_x += pow(cos(e.abs_diff_values[i][2] - e.mean_abs_th_y), 2);
		e.std_abs_th_y += pow(sin(e.abs_diff_values[i][2] - e.mean_abs_th_x), 2);
	}

	e.dist_std /= ((double) n);
	e.abs_x_std /= ((double) n);
	e.abs_y_std /= ((double) n);
	e.std_abs_th_x /= ((double) n);
	e.std_abs_th_y /= ((double) n);
	e.th_std = g2o::normalize_theta(atan2(e.std_abs_th_y, e.std_abs_th_x));
}


ExperimentStatistics
compute_statistics(vector<SE2> &gt, vector<double> &gt_times,
									 vector<SE2> &means, vector<double> &means_times,
									 vector<SE2> &stds,
									 vector<SE2> odometry)
{
	ExperimentStatistics e;

	// in the future, the std can be used to discard outliers.
	(void) stds;
	//assert(gt.size() == means.size());

	e.reset();

	compute_means_and_rmses(e, gt, gt_times, means, means_times, odometry);
	compute_stds(e);

	return e;
}


void
save_statistics(ExperimentStatistics &e,
                string path_summary,
                string path_report)
{
	FILE *summary_f = safe_fopen(path_summary.c_str(), "w");
	FILE *report_f = safe_fopen(path_report.c_str(), "w");

	for (int i = 0; i < e.estimates_in_gt_ref.size(); i++)
	{
		fprintf(report_f, "estimate_in_gt_ref: %lf %lf %lf abs_difference: %lf %lf %lf distance: %lf\n",
		        e.estimates_in_gt_ref[i][0], e.estimates_in_gt_ref[i][1], e.estimates_in_gt_ref[i][2],
		        e.abs_diff_values[i][0], e.abs_diff_values[i][1], e.abs_diff_values[i][2],
		        sqrt(e.squared_dist_by_sample[i]));
	}

	fprintf(summary_f, "dist_rmse\tdist_mean\tdist_std\t%%<2m\t%%<1m\t%%<0.5m\t%%<0.2m\tx_rmse\tabs_x_mean\tabs_x_std\tdx_mean\ty_rmse\tabs_y_mean\tabs_y_std\tdy_mean\tabs_th_mean\tabs_th_std\tdth_mean\n");
	fprintf(summary_f, "%.3lf\t%.3lf\t%.3lf\t", e.dist_rmse, e.dist_mean, e.dist_std);
	fprintf(summary_f, "%.3lf\t%.3lf\t%.3lf\t%.3lf\t", e.percentage_samples_below_2m, e.percentage_samples_below_1m, e.percentage_samples_below_0_5m, e.percentage_samples_below_0_2m);
	fprintf(summary_f, "%.3lf\t%.3lf\t%.3lf\t%.3lf\t", e.x_rmse, e.abs_x_mean, e.abs_x_std, e.dx_mean);
	fprintf(summary_f, "%.3lf\t%.3lf\t%.3lf\t%.3lf\t", e.y_rmse, e.abs_y_mean, e.abs_y_std, e.dy_mean);
	fprintf(summary_f, "%.3lf\t%.3lf\t%.3lf\n", e.th_mean, e.th_std, e.dth_mean);

	fclose(summary_f);
	fclose(report_f);
}


int
main(int argc, char **argv)
{
	vector<double> means_times, gt_times, odometry_times;
	vector<SE2> gt, means, modes, stds, odometry;
	ExperimentStatistics statistics;
	CommandLineArguments args;

	args.add_positional<string>("gt", "Path to the groundtruth file", 1);
	args.add_positional<string>("result", "Path to the output file produced by the localization system", 1);
	args.add_positional<string>("odometry", "Path to the odometry file", 1);
	args.add_positional<string>("summary", "Output summary of statistics", 1);
	args.add_positional<string>("report", "Output long report of statistics data", 1);
	args.parse(argc, argv);

	read_groundtruth(args.get<string>("gt"), &gt, &gt_times);
	read_result(args.get<string>("result"), &means, &stds, &modes, &means_times);
	read_odometry(args.get<string>("odometry"), &odometry, &odometry_times);

	statistics = compute_statistics(gt, gt_times, means, means_times, stds, odometry);
	save_statistics(statistics, args.get<string>("summary"), args.get<string>("report"));

	printf("Done.\n");
	return 0;
}


