
#include <boost/filesystem.hpp>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_preproc.h>
#include <carmen/util_math.h>
#include <carmen/command_line.h>
#include "libsegmap/initializations/segmap_args.h"
#include <carmen/segmap_constructors.h>

#include <carmen/util_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>
#include <map>

using namespace std;
using namespace pcl;
using namespace Eigen;

typedef unsigned char uchar;


class Reading
{
public:
	uchar laser_id;
	uchar intensity;

	Reading() { laser_id = intensity = 0; }
	Reading(uchar laser_id_val, uchar intensity_val)
	{
		laser_id = laser_id_val;
		intensity = intensity_val;
	}
};

typedef vector<Reading> ReadingVec;
typedef map<int, ReadingVec> Row;
typedef map<int, Row> Grid;


class Calibrator
{
public:
	static const int N_RAYS = 32;

	Calibrator(double resolution)
	{
		_resolution = resolution;
	}

	void add(SensorPreproc::CompletePointData &p)
	{
		int cy = cell_coord(p.world.y, _resolution);
		int cx = cell_coord(p.world.x, _resolution);

		_row_it = _cells.find(cy);

		if (_row_it == _cells.end())
			_row_it = _cells.insert(pair<int, Row>(cy, Row())).first;

		_col_it = _row_it->second.find(cx);

		if (_col_it == _row_it->second.end())
			_col_it = _row_it->second.insert(pair<int, ReadingVec>(cx, ReadingVec())).first;

		_col_it->second.push_back(Reading(p.laser_id, p.raw_intensity));
	}

	static int cell_coord(double x, double resolution)
	{
		return (int) (x / resolution);
	}

	void create_calibration_table()
	{
		// Initialize calibration table.
		_support_table.clear();
		_support_table = vector<vector<pair<double, long>>>(N_RAYS); // read this value.

		for (int i = 0; i < N_RAYS; i++)
			for (uchar v = 0; v < 256; v++)
				_support_table[i].push_back(pair<double, long>(0, 0));

		// Integrate points to support table
		for (_row_it = _cells.begin(); _row_it != _cells.end(); _row_it++)
			for (_col_it = _row_it->second.begin(); _col_it != _row_it->second.end(); _col_it++)
				integrate_cell_points_to_table(_col_it->second, &_support_table);
	}

	int
	ids_and_intens_already_accounted(Reading r, vector<pair<int, uchar>> &ids_and_intens)
	{
		for (int i = 0; i < ids_and_intens.size(); i++)
			if (ids_and_intens[i].first == r.laser_id && ids_and_intens[i].second == r.intensity)
				return 1;

		return 0;
	}


	void
	integrate_cell_points_to_table(ReadingVec &readings, vector<vector<pair<double, long>>> *support_table)
	{
		vector<pair<int, uchar>> ids_and_intens;

		for (int i = 0; i < readings.size(); i++)
		{
			double sum = 0;
			int count = 0;

			// to prevent adding cell values more than once.
			if (ids_and_intens_already_accounted(readings[i], ids_and_intens))
				continue;

			for (int j = 0; j < readings.size(); j++)
			{
				if (readings[j].laser_id != readings[i].laser_id)
				{
					sum += (double) readings[j].intensity / (double) 255;
					count++;
				}
			}

			ids_and_intens.push_back(pair<int, uchar>(readings[i].laser_id, readings[i].intensity));
		}
	}

	void save_calibration_table(const char *path)
	{
		FILE *f = safe_fopen("calib.txt", "w");

		for (int i = 0; i < _support_table.size(); i++)
		{
			for (int j = 0; j < _support_table[i].size(); j++)
			{
				double calibrated_value = 0.0;

				if (_support_table[i][j].second > 0)
					calibrated_value = _support_table[i][j].first / (double) _support_table[i][j].second;

				fprintf(f, "%d %d %lf %lf %d\n",
								i, j, _support_table[i][j].first,
								_support_table[i][j].second,
								(int) 255 * calibrated_value);
			}
		}

		fclose(f);
	}

protected:
	Grid _cells;
	vector<vector<pair<double, long>>> _support_table;

	double _resolution;

	Grid::iterator _row_it;
	Row::iterator _col_it;
};


string
poses_path_from_mode(string mode, string log_path)
{
	string path;

	if (mode.compare("fused") == 0)
		path = default_fused_odom_path(log_path.c_str());
	else if (mode.compare("graphslam") == 0)
		path = default_graphslam_path(log_path.c_str());
	else if (mode.compare("graphslam_to_map") == 0)
		path = default_graphslam_to_map_path(log_path.c_str());
	else
		exit(printf("Error: Invalid mode '%s'\n.", mode.c_str()));

	return path;
}

void
add_all_points(NewCarmenDataset *dataset, SensorPreproc &preproc, Calibrator *calib)
{
	DataSample *sample;

	for (int i = 0; i < dataset->size(); i ++)
	{
		sample = dataset->at(i);

		if (fabs(sample->v) < 1.0)
			continue;

		preproc.reinitialize(sample);

		for (int i = 0; i < preproc.size(); i++)
		{
			vector<SensorPreproc::CompletePointData> points = preproc.next_points();

			for (int j = 0; j < points.size(); j++)
				calib->add(points[j]);
		}
	}
}


Calibrator*
run_calibration(CommandLineArguments &args)
{
	string log_path = args.get<string>("log_path");
	string odom_calib_path = default_odom_calib_path(log_path.c_str());
	string poses_path = poses_path_from_mode(args.get<string>("mode"), log_path);

	NewCarmenDataset *dataset = new NewCarmenDataset(log_path, odom_calib_path, poses_path);
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);
	Calibrator *calib = new Calibrator(args.get<double>("resolution"));

	add_all_points(dataset, preproc, calib);
	calib->create_calibration_table();

	return calib;
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	args.add_positional<string>("log_path", "Path to a log.", 1);
	args.add_positional<string>("output", "Path to an output file.", 1);
	args.add_positional<string>("pose_mode", "Type of pose to be used [fused | graphslam | graphslam_to_map]", 0.2);
	args.add_positional<double>("resolution", "Grid map resolution", 0.2);
	add_default_sensor_preproc_args(args);

	args.parse(argc, argv);
	run_calibration(args);

	return 0;
}

