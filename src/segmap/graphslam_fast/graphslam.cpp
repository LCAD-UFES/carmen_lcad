
#include <carmen/carmen.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/core/sparse_optimizer.h>

#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/factory.h>
#include "edge_gps_2D.h"

#include <carmen/gicp.h>
#include <carmen/command_line.h>

#include <carmen/segmap_dataset.h>
#include <carmen/util_io.h>
#include <carmen/util_math.h>
#include <carmen/ackerman_motion_model.h>
#include <carmen/segmap_loop_closures.h>
#include <carmen/segmap_args.h>
#include <carmen/segmap_conversions.h>
#include <carmen/segmap_constructors.h>

using namespace std;
using namespace g2o;

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> MyBlockSolver;
typedef LinearSolverCholmod<MyBlockSolver::PoseMatrixType> MyCholmodSolver;


class GraphSlamData
{
public:
	NewCarmenDataset *dataset;
	vector<LoopRestriction> gicp_loop_data, gicp_odom_data, gicp_based_gps;
	vector<LoopRestriction> pf_loop_data, pf_based_gps;

	~GraphSlamData() { delete(dataset);	}

	string log_file;
	string gicp_loop_closure_file;
	string pf_loop_closure_file;
	string gicp_odom_file;
	string gicp_map_file;
	string pf_to_map_file;
	string output_file;
	string odom_calib_file;

	int gps_id;
	double gps_xy_std, gps_angle_std;
	double odom_xy_std, odom_angle_std;
	double gicp_loop_xy_std, gicp_loop_angle_std;
	double gicp_to_map_xy_std, gicp_to_map_angle_std;
	double pf_loop_xy_std, pf_loop_angle_std;
	double pf_to_map_xy_std, pf_to_map_angle_std;
	double gicp_odom_xy_std, gicp_odom_angle_std;

	int n_iterations;

	double min_velocity_for_estimating_heading;
	double min_velocity_for_considering_gps;
	string global_angle_mode;

	vector<int> gps_is_valid;

protected:
	void _load_parameters(char *config);
};


const double distance_between_front_and_rear_axles = 2.625;


void
read_loop_restrictions(string &filename, vector<LoopRestriction> *loop_data)
{
	int n;
	FILE *f;
	double x, y, theta;

	if (filename.size() <= 0)
		return;

	f = fopen(filename.c_str(), "r");

	if (f != NULL)
	{
		while(!feof(f))
		{
			LoopRestriction l;

			n = fscanf(f, "%d %d %d %lf %lf %lf\n",
								 &l.from, &l.to, &l.converged, &x, &y, &theta
			);

			if (n == 6)
			{
				l.transform = SE2(x, y, theta);
				loop_data->push_back(l);
			}
		}

		fclose(f);
	}
	else
		fprintf(stderr, "Warning: ignoring loop closure file '%s'\n", filename.c_str());
}


void
add_vertices(vector<SE2> &dead_reckoning, SparseOptimizer *optimizer)
{
	uint i;

	for (i = 0; i < dead_reckoning.size(); i++)
	{
		//SE2 estimate(
		//	input_data[i].x - input_data[0].x,
		//	input_data[i].y - input_data[0].y,
		//	input_data[i].angle
		//);

		SE2 estimate = dead_reckoning[i];
		//SE2 estimate = SE2(i, 0, 0);

		VertexSE2* vertex = new VertexSE2;
		vertex->setId(i);
		vertex->setEstimate(estimate);
		optimizer->addVertex(vertex);
	}
}


Matrix3d
create_information_matrix(double x_std, double y_std, double z_std)
{
	Matrix3d cov;

	cov(0, 0) = pow(x_std, 2);
	cov(0, 1) = 0;
	cov(0, 2) = 0;
	cov(1, 0) = 0;
	cov(1, 1) = pow(y_std, 2);
	cov(1, 2) = 0;
	cov(2, 0) = 0;
	cov(2, 1) = 0;
	cov(2, 2) = pow(z_std, 2);

	return cov.inverse();
}


void
add_odometry_edges(SparseOptimizer *optimizer, vector<SE2> &dead_reckoning, double xy_std, double th_std)
{
	Matrix3d information = create_information_matrix(xy_std, xy_std, th_std);

	for (size_t i = 1; i < dead_reckoning.size(); i++)
	{
		SE2 prev_inv = dead_reckoning[i - 1].inverse();
		SE2 measure = prev_inv * dead_reckoning[i];

//		printf("i: %lf %lf %lf prev_inv: %lf %lf %lf measure: %lf %lf %lf\n",
//					 dead_reckoning[i][0],dead_reckoning[i][1], dead_reckoning[i][2],
//					 prev_inv[0], prev_inv[1],prev_inv[2],
//					 measure[0], measure[1], measure[2]);

		EdgeSE2* edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(i - 1);
		edge->vertices()[1] = optimizer->vertex(i);
		edge->setMeasurement(measure);
		edge->setInformation(information);
		optimizer->addEdge(edge);
	}
}


void
compute_angle_from_gps(GraphSlamData &data, int i,
                       double th_std,
                       double *angle,
                       double *filtered_th_std)
{

	DataSample *sample = data.dataset->at(i);

	if (data.global_angle_mode.compare("xsens") == 0)
	{
		Matrix<double, 3, 1> ypr = sample->xsens.toRotationMatrix().eulerAngles(2, 1, 0);
		*angle = ypr(0, 0);
		*filtered_th_std = th_std;
	}
	else if (data.global_angle_mode.compare("gps") == 0)
	{
		*angle = sample->gps.th;
		*filtered_th_std = th_std;
	}
	else if (data.global_angle_mode.compare("consecutive_gps") == 0)
	{
		// estimate the angle using consecutive gps positions
		// when the car velocity is high enough.
		if (i > 0 && fabs(sample->v) >= data.min_velocity_for_estimating_heading)
		{
			DataSample *prev = data.dataset->at(i - 1);
			*angle = atan2(sample->gps.y - prev->gps.y, sample->gps.x - prev->gps.x);
			*filtered_th_std = th_std;
		}
		else
		{
			// if the car is slow, ignore the heading and use a high angular variance.
			*angle = 0.;
			*filtered_th_std = 1e8;
		}
	}
	else
		exit(printf("Error: Invalid global angle estimation mode '%s'\n", data.global_angle_mode.c_str()));
}


void
add_gps_edges(GraphSlamData &data, SparseOptimizer *optimizer,
              double xy_std, double th_std)
{
	double angle, filtered_th_std;
	DataSample *sample;
	Matrix3d information;

	for (int i = 0; i < data.dataset->size(); i++)
	{
		sample = data.dataset->at(i);

		// ignore the messages when the car is almost stopped.
		if (fabs(sample->v) < data.min_velocity_for_considering_gps || !data.gps_is_valid[i])
			continue;

		compute_angle_from_gps(data, i, th_std,
		                       &angle,
		                       &filtered_th_std);

		sample->gps.th = angle;

		information = create_information_matrix(xy_std, xy_std, th_std);

		SE2 measure(sample->gps.x,
		            sample->gps.y,
		            angle);

//		printf("GPS: measure: %lf %lf %lf x_std: %lf th_std: %lf\n",
//					 measure[0], measure[1], angle,
//					 xy_std, filtered_th_std);

		EdgeGPS *edge_gps = new EdgeGPS;
		edge_gps->vertices()[0] = optimizer->vertex(i);
		edge_gps->setMeasurement(measure);
		edge_gps->setInformation(information);
		optimizer->addEdge(edge_gps);
	}
}


void
add_gps_from_map_registration_edges(GraphSlamData &data, vector<LoopRestriction> &gicp_gps,
																		SparseOptimizer *optimizer, double xy_std_mult, double th_std,
                                    int check_for_outliers = 0)
{

	//int n_discarded = 0;

	for (size_t i = 0; i < gicp_gps.size(); i += 1)
	{
		if (gicp_gps[i].converged)
		{
			SE2 measure(gicp_gps[i].transform[0],
			            gicp_gps[i].transform[1],
			            gicp_gps[i].transform[2]);

			/*
			DataSample *sample = data.dataset->at(gicp_gps[i].to);

			double dist_to_graphslam = dist2d(gicp_gps[i].transform[0],
			                			            gicp_gps[i].transform[1],
			                			            sample->gps.x,
			                			            sample->gps.y);

			if (dist_to_graphslam > 10.0)
			{
				n_discarded++;
				continue;
			}

			if (check_for_outliers)
			{
				int is_outlier;
				is_outlier = 0;

				if (i > 0)
				{
					// try and discard outliers
					double dt = data.dataset->at(gicp_gps[i].to)->time - data.dataset->at(gicp_gps[i-1].to)->time;
					double dx = gicp_gps[i].transform[0] - gicp_gps[i-1].transform[0];
					double dy = gicp_gps[i].transform[1] - gicp_gps[i-1].transform[1];
					double registr_th = atan2(dy, dx);

					double gdx = data.dataset->at(gicp_gps[i].to)->gps.x - data.dataset->at(gicp_gps[i-1].to)->gps.x;
					double gdy = data.dataset->at(gicp_gps[i].to)->gps.y - data.dataset->at(gicp_gps[i-1].to)->gps.y;
					double gps_th = atan2(gdy, gdx);
					double dth = fabs(g2o::normalize_theta(registr_th - gps_th));

					double vx = fabs(dx / dt);
					double vy = fabs(dy / dt);

					if (fabs(vx - data.dataset->at(gicp_gps[i].to)->v) > 2.0
							|| vy > 1.0
							|| (data.dataset->at(gicp_gps[i].to)->v > 1.0 && dth > degrees_to_radians(20.)))
						is_outlier = 1;
				}

				if (is_outlier)
				{
					continue;
				}

			}
			 */

			//printf("ADDING %lf %lf\n", gicp_gps[i].transform[0], gicp_gps[i].transform[1]);
			Matrix3d information = create_information_matrix(xy_std_mult, xy_std_mult, th_std);

			EdgeGPS *edge_gps = new EdgeGPS;
			edge_gps->vertices()[0] = optimizer->vertex(gicp_gps[i].to);
			edge_gps->setMeasurement(measure);
			edge_gps->setInformation(information);
			optimizer->addEdge(edge_gps);
		}
	}

	//if (gicp_gps.size())
		//printf("Discard %.2lf%% of the registration edges\n", 100 * (double) n_discarded / (double) gicp_gps.size());
}


void
add_loop_closure_edges(GraphSlamData &data, vector<LoopRestriction> &loop_data, SparseOptimizer *optimizer, double xy_std, double th_std)
{
	int n_discarded = 0;
	Matrix3d information = create_information_matrix(xy_std, xy_std, th_std);

	for (size_t i = 0; i < loop_data.size(); i++)
	{
		if (loop_data[i].converged)
		{
			DataSample *sample = data.dataset->at(loop_data[i].to);
			DataSample *target_sample = data.dataset->at(loop_data[i].from);

			g2o::SE2 target_pose(target_sample->pose.x, target_sample->pose.y, target_sample->pose.th);
			g2o::SE2 source_pose(sample->pose.x, sample->pose.y, sample->pose.th);
			g2o::SE2 source_in_target = target_pose.inverse() * source_pose;

			if (fabs(source_in_target[0] - loop_data[i].transform[0]) > 5.0
					|| fabs(source_in_target[1] - loop_data[i].transform[1]) > 5.0
					)
			{
				n_discarded++;
				continue;
			}

			EdgeSE2* edge = new EdgeSE2;
			edge->vertices()[0] = optimizer->vertex(loop_data[i].from);
			edge->vertices()[1] = optimizer->vertex(loop_data[i].to);
			edge->setMeasurement(loop_data[i].transform);
			edge->setInformation(information);
			optimizer->addEdge(edge);
		}
	}

	if (loop_data.size())
		printf("Discard %.2lf%% loop closure edges\n", 100 * (double) n_discarded / (double) loop_data.size());
}


void
create_dead_reckoning(GraphSlamData &data, vector<SE2> &dead_reckoning)
{
	double dt;
	DataSample *sample;

	Pose2d pose(0, 0, data.dataset->initial_angle());
	dead_reckoning.push_back(SE2(pose.x, pose.y, pose.th));

	for (int i = 1; i < data.dataset->size(); i++)
	{
		sample = data.dataset->at(i);

		dt = sample->time - data.dataset->at(i - 1)->time;
		ackerman_motion_model(pose, sample->v, sample->phi, dt);
		//printf("Dead reckoning: %d dt: %lf v: %lf phi: %lf x: %lf y: %lf\n", i, dt, sample->v, sample->phi, pose.x, pose.y);
		dead_reckoning.push_back(SE2(pose.x, pose.y, pose.th));

	}

	printf("N odometry edges: %ld\n", dead_reckoning.size() - 1);
}


void
load_data_to_optimizer(GraphSlamData &data, SparseOptimizer* optimizer)
{
	vector<SE2> dead_reckoning;

	create_dead_reckoning(data, dead_reckoning);
	add_vertices(dead_reckoning, optimizer);
	add_odometry_edges(optimizer, dead_reckoning, data.odom_xy_std, deg2rad(data.odom_angle_std));

	if (data.gicp_based_gps.size() > 0 || data.pf_based_gps.size() > 0)
	{
		add_gps_from_map_registration_edges(data, data.gicp_based_gps, optimizer,
																				data.gicp_to_map_xy_std,
																				deg2rad(data.gicp_to_map_angle_std));
		add_gps_from_map_registration_edges(data, data.pf_based_gps, optimizer,
																				data.pf_to_map_xy_std,
																				deg2rad(data.pf_to_map_angle_std));
	}
	//else
		add_gps_edges(data, optimizer, data.gps_xy_std, deg2rad(data.gps_angle_std));

	add_loop_closure_edges(data, data.gicp_loop_data, optimizer, data.gicp_loop_xy_std, deg2rad(data.gicp_loop_angle_std));
	add_loop_closure_edges(data, data.pf_loop_data, optimizer, data.pf_loop_xy_std, deg2rad(data.pf_loop_angle_std));
	add_loop_closure_edges(data, data.gicp_odom_data, optimizer, data.gicp_odom_xy_std, deg2rad(data.gicp_odom_angle_std));
}


void
save_corrected_vertices(GraphSlamData &data, SparseOptimizer *optimizer)
{
	double x, y, th;
	DataSample *sample;

	FILE *f = safe_fopen(data.output_file.c_str(), "w");

	for (size_t i = 0; i < optimizer->vertices().size(); i++)
	{
		sample = data.dataset->at(i);

		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(i));
		SE2 pose = v->estimate();

		x = pose.toVector().data()[0];
		y = pose.toVector().data()[1];
		th = pose.toVector().data()[2];

		fprintf(f, "%ld %lf %lf %lf %lf %lf %lf\n", i, x, y, th, sample->time, sample->gps.x, sample->gps.y);
	}

	fclose(f);
}


void
detect_and_stamp_invalid_gps_measurements(GraphSlamData *data,
																					double gps_discontinuity_threshold,
																					int gps_min_cluster_size)
{
	double d;
	vector<int> gps_group_ids;
	DataSample *current, *previous;
	int number_invalid_measurements;

	// assume initially that all gps measurements are valid.
	data->gps_is_valid = vector<int>(data->dataset->size(), 1);

	previous = data->dataset->at(0);
	gps_group_ids.push_back(0);

	number_invalid_measurements = 0;

	FILE *f = fopen("/tmp/invalid_gps.txt", "w");

	double dist_to_add_gps = 1.0;

	for (int i = 1; i < data->dataset->size(); i++)
	{
		current = data->dataset->at(i);

		d = dist2d(current->gps.x, current->gps.y, previous->gps.x, previous->gps.y);
		Pose2d estimated_motion(0,0,0);
		ackerman_motion_model(estimated_motion, current->v, current->phi, fabs(current->time - previous->time));
		double d_estimated = sqrt(pow(estimated_motion.x, 2) + pow(estimated_motion.y, 2));

		// discountinuity detected
		if (fabs(d - d_estimated) > gps_discontinuity_threshold)
		{
			// current is set to invalid
			data->gps_is_valid[i] = 0;
			number_invalid_measurements += 1;

			fprintf(f, "%lf %lf\n",
							current->gps.x,
							current->gps.y);

			// if the previous cluster is small, set all measurements from the group as invalid.
			if (gps_group_ids.size() < gps_min_cluster_size)
			{
				for (int j = 0; j < gps_group_ids.size(); j++)
				{
					data->gps_is_valid[gps_group_ids[j]] = 0;

					fprintf(f, "%lf %lf\n",
									data->dataset->at(gps_group_ids[j])->gps.x,
									data->dataset->at(gps_group_ids[j])->gps.y);
				}

				number_invalid_measurements += gps_group_ids.size();
			}

			gps_group_ids.clear();
		}
		else
			gps_group_ids.push_back(i);

		previous = current;
	}

	/*
	double accum_dist = 0;
	int last_valid = 0;
	DataSample *last_valid_sample;

	for (int i = 1; i < data->dataset->size(); i++)
	{
		if (!data->gps_is_valid[i])
			continue;

		current = data->dataset->at(i);
		last_valid_sample = data->dataset->at(last_valid);

		d = dist2d(current->gps.x, current->gps.y, last_valid_sample->gps.x, last_valid_sample->gps.y);

		if (d < dist_to_add_gps)
		{
			data->gps_is_valid[i] = 0;
			number_invalid_measurements += 1;
			fprintf(f, "%lf %lf\n",
							current->gps.x,
							current->gps.y);
		}

		if (data->gps_is_valid[i])
			last_valid = i;
	}
	*/

	fprintf(stderr, "Percentage of invalid gps measurements: %lf%%\n",
					100 * ((double) number_invalid_measurements / (double) data->dataset->size()));

	fclose(f);
}


void
initialize_g2o_stuff(g2o::Factory *factory, SparseOptimizer *optimizer)
{
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);

	MyCholmodSolver *cholmod_solver = new MyCholmodSolver();
	cholmod_solver->setBlockOrdering(false);

	g2o::Solver *solver = new MyBlockSolver(cholmod_solver);

	g2o::OptimizationAlgorithm *optimization_algorithm =
			new g2o::OptimizationAlgorithmGaussNewton(solver);

	optimizer->setAlgorithm(optimization_algorithm);
	optimizer->setVerbose(true);
}


void
prepare_optimization(SparseOptimizer *optimizer)
{
	for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it)
	{
		OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
		v->setMarginalized(false);
	}

	optimizer->initializeOptimization();

	for (SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it)
	{
		OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
		e->setRobustKernel(0);
	}

	optimizer->computeActiveErrors();
}


void
free_g2o_stuff(g2o::Factory *factory,
							SparseOptimizer* optimizer)
{
  if (factory != nullptr)
      g2o::Factory::destroy();

  if (optimizer != nullptr)
  {
      optimizer->clear();
      delete optimizer;
  }
}


void
parse_command_line(int argc, char **argv, CommandLineArguments &args, GraphSlamData *data)
{
	args.parse(argc, argv);

	data->log_file = args.get<string>("log");
	data->output_file = args.get<string>("output");
	data->gps_id = args.get<int>("gps_id");
	data->n_iterations = args.get<int>("n_iterations");

	data->odom_calib_file = args.get<string>("odom_calib");
	data->gicp_loop_closure_file = args.get<string>("gicp_loops");
	data->pf_loop_closure_file = args.get<string>("pf_loops");
	data->gicp_odom_file = args.get<string>("gicp_odom");
	data->gicp_map_file = args.get<string>("gicp_to_map");
	data->pf_to_map_file = args.get<string>("pf_to_map");

	data->odom_xy_std = args.get<double>("odom_xy_std");
	data->odom_angle_std = args.get<double>("odom_angle_std");

	data->gps_xy_std = args.get<double>("gps_xy_std");
	data->gps_angle_std = args.get<double>("gps_angle_std");

	data->gicp_loop_xy_std = args.get<double>("gicp_loops_xy_std");
	data->gicp_loop_angle_std = args.get<double>("gicp_loops_angle_std");

	data->pf_loop_xy_std = args.get<double>("pf_loops_xy_std");
	data->pf_loop_angle_std = args.get<double>("pf_loops_angle_std");

	data->gicp_odom_xy_std = args.get<double>("gicp_odom_xy_std");
	data->gicp_odom_angle_std = args.get<double>("gicp_odom_angle_std");

	data->gicp_to_map_xy_std = args.get<double>("gicp_to_map_xy_std");
	data->gicp_to_map_angle_std = args.get<double>("gicp_to_map_angle_std");

	data->pf_to_map_xy_std = args.get<double>("pf_to_map_xy_std");
	data->pf_to_map_angle_std = args.get<double>("pf_to_map_angle_std");

	data->min_velocity_for_estimating_heading = args.get<double>("v_for_gps_heading");
	data->min_velocity_for_considering_gps = args.get<double>("v_for_gps");
	data->global_angle_mode = args.get<string>("global_angle_mode");
}


void
add_graphslam_parameters(CommandLineArguments &args)
{
	args.add<int>("n_iterations,n", "Number of optimization terations", 50);

	args.add<string>("odom_calib,o", "Path to the odometry calibration file", "none");
	args.add<string>("gicp_loops,l", "Path to a file with loop closure relations", "none");
	args.add<string>("pf_loops", "Path to a file with loop closures estimated with particle filters", "none");
	args.add<string>("gicp_odom", "Path to a file with odometry relations generated with GICP", "none");
	args.add<string>("pf_to_map", "Path to a file with poses given by registration to an a priori map with particle filter", "none");
	args.add<string>("gicp_to_map", "Path to a file with poses given by registration to an a priori map with gicp", "none");

	args.add<double>("odom_xy_std", "Std in xy of odometry-based movement estimations (m)", 0.005);
	args.add<double>("odom_angle_std", "Std of heading of odometry-based movement estimations (degrees)", 0.1);

	args.add<double>("gps_xy_std", "Std in xy of gps measurements (m)", 10.0);
	args.add<double>("gps_angle_std", "Std of heading estimated using consecutive gps measurements when the car is moving (degrees)", 10);

	args.add<double>("gicp_loops_xy_std", "Std in xy of loop closure measurements (m)", 0.3);
	args.add<double>("gicp_loops_angle_std", "Std of heading in loop closure measurements (degrees)", 3);

	args.add<double>("pf_loops_xy_std", "Std in xy of loop closure measurements (m)", 0.3);
	args.add<double>("pf_loops_angle_std", "Std of heading in loop closure measurements (degrees)", 3);

	args.add<double>("gicp_odom_xy_std", "Std in xy of odometry estimated using GICP (m)", 0.3);
	args.add<double>("gicp_odom_angle_std", "Std of heading when estimating odometry using GICP (degrees)", 3);

	args.add<double>("gicp_to_map_xy_std", "Std in xy of loop closure measurements in relation to a map (m)", 0.3);
	args.add<double>("gicp_to_map_angle_std", "Std of heading in loop closure in relation to a map (degrees)", 3);

	args.add<double>("pf_to_map_xy_std", "Std in xy of loop closure measurements in relation to a map (m)", 0.3);
	args.add<double>("pf_to_map_angle_std", "Std of heading in loop closure in relation to a map (degrees)", 3);

	args.add<double>("v_for_gps_heading", "Minimum velocity for estimating heading with consecutive GPS poses", 1.0);
	args.add<double>("v_for_gps", "Minimum velocity for adding GPS edges (without this trick, positions in which v=0 can be overweighted)", 1.0);
	args.add<string>("global_angle_mode", "Method for estimating global angle: [xsens, gps, consecutive_gps]", "consecutive_gps");
}


class Bla
{
public:

	g2o::Factory *factory;
	SparseOptimizer* optimizer;

	Bla()
	{
		factory = g2o::Factory::instance();
		optimizer = new SparseOptimizer;
		initialize_g2o_stuff(factory, optimizer);
	}

	~Bla()
	{
		free_g2o_stuff(factory, optimizer);
	}
};


int main(int argc, char **argv)
{
	srand(time(NULL));

	CommandLineArguments args;
	GraphSlamData data;

	args.add_positional<string>("log", "Path to a log", 1);
	args.add_positional<string>("output", "Path to the output file", 1);
	args.add<int>("gps_id", "Index of the gps to be used", 1);
	add_graphslam_parameters(args);
	add_default_sensor_preproc_args(args);
	args.add<double>("gps_discontinuity_threshold", "Threshold for consireding detecting a jump in consecutive gps messages", 0.5);
	args.add<int>("gps_min_cluster_size", "Minimum number of messages for keeping a group when a discountinuity is detected", 50);
	args.save_config_file(default_data_dir() + "/graphslam_config.txt");

	parse_command_line(argc, argv, args, &data);

	//g2o::Factory *factory = g2o::Factory::instance();
	//SparseOptimizer* optimizer = new SparseOptimizer;
	//initialize_g2o_stuff(factory, optimizer);
	Bla bla;

	data.dataset = create_dataset(args.get<string>("log"), args.get<double>("camera_latency"), "fused_odometry");
	if (data.dataset->size() <= 0)
		exit(printf("Error: Empty dataset.\n"));

	read_loop_restrictions(data.gicp_loop_closure_file, &data.gicp_loop_data);
	read_loop_restrictions(data.pf_loop_closure_file, &data.pf_loop_data);
	read_loop_restrictions(data.gicp_odom_file, &data.gicp_odom_data);
	read_loop_restrictions(data.gicp_map_file, &data.gicp_based_gps);
	read_loop_restrictions(data.pf_to_map_file, &data.pf_based_gps);

	detect_and_stamp_invalid_gps_measurements(&data,
																						args.get<double>("gps_discontinuity_threshold"),
																						args.get<int>("gps_min_cluster_size"));
	load_data_to_optimizer(data, bla.optimizer);

	bla.optimizer->setVerbose(true);
	prepare_optimization(bla.optimizer);
	bla.optimizer->optimize(data.n_iterations);
	cerr << "OptimizationDone!" << endl;

	// output
	save_corrected_vertices(data, bla.optimizer);
	cerr << "OutputSaved!" << endl;

	//free_g2o_stuff(factory, optimizer);

	return 0;
}

