
#include <string>
#include <vector>
#include <carmen/command_line.h>
#include <g2o/types/slam2d/se2.h>
#include <carmen/util_io.h>
#include <carmen/segmap_dataset.h>

using namespace std;
using namespace g2o;


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


void
find_synchronized_samples(vector<double> &means_times,
													NewCarmenDataset &dataset,
													vector<int> *dataset_indices)
{
	for (int i = 0; i < means_times.size(); i++)
	{
		double diff_t = DBL_MAX;
		int most_sync_id = -1;

		for (int j = 0; j < dataset.size(); j++)
		{
			double dt = fabs(dataset[j]->time - means_times[i]);

			if (dt < diff_t)
			{
				diff_t = dt;
				most_sync_id = j;
			}
		}

		dataset_indices->push_back(most_sync_id);
	}
}


void
save_odometries(const string &path, NewCarmenDataset &dataset, vector<int> &dataset_indices)
{
	FILE *f = safe_fopen(path.c_str(), "w");

	for (int i = 0; i < dataset_indices.size(); i++)
	{
		DataSample *sample = dataset[dataset_indices[i]];

		fprintf(f, "%d %lf %lf %lf\n",
						dataset_indices[i],
						sample->v, sample->phi,
						sample->time);
	}

	fclose(f);
}


int
main(int argc, char **argv)
{
	vector<int> dataset_indices;
	vector<double> means_times;
	vector<SE2> means, odometry, stds, modes;
	CommandLineArguments args;

	args.add_positional<string>("log", "Path to a log", 1);
	args.add_positional<string>("result", "Path to the output file produced by the localization system", 1);
	args.add_positional<string>("output", "Output file", 1);
	args.parse(argc, argv);

	NewCarmenDataset dataset(args.get<string>("log"));

	read_result(args.get<string>("result"), &means, &stds, &modes, &means_times);
	find_synchronized_samples(means_times, dataset, &dataset_indices);
	save_odometries(args.get<string>("output"), dataset, dataset_indices);

	printf("Done.\n");
	return 0;
}


