#include <carmen/carmen.h>
#include <iostream>
#include <vector>
#include <cmath>

#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/types_slam2d.h"

#include "g2o/apps/g2o_cli/dl_wrapper.h"
#include "g2o/apps/g2o_cli/g2o_common.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/stuff/macros.h"
#include "g2o/core/optimizable_graph.h"

#include "edge_gps_2D.h"
#include "edge_gps_2D_new.h"
#include <carmen/carmen_param_file.h>
#include <Eigen/Core>

using namespace std;
using namespace g2o;

class Line
{
	public:
		SE2 gps;
		SE2 odom;
		SE2 icp;
		double time;
		double gps_std;
		double gps_yaw;
		double gps_orientation_valid;
};

class LoopRestriction
{
	public:
		SE2 transform;
		int from;
		int to;
};

vector<Line> input_data;
vector<LoopRestriction> loop_data;

vector<pair<int, int> > gps_to_odometry;
vector<pair<SE2, double> > odometry;
vector<pair<SE2, double> > gps;

#define BEGIN_ID 100000
#define END_ID 100000
//#define BEGIN_ID 6120
//#define END_ID 6140
//#define BEGIN_ID 5120
//#define END_ID 5140

char *input_file;
char *loops_file;
char *out_file;


Matrix<double, 4, 4>
pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw)
{
	Matrix<double, 3, 3> Rx, Ry, Rz, R;
	Matrix<double, 4, 4> T;

	double rx, ry, rz;

	rx = roll;
	ry = pitch;
	rz = yaw;

    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
    R  = Rz * Ry * Rx;

    T << R(0, 0), R(0, 1), R(0, 2), x,
    	R(1, 0), R(1, 1), R(1, 2), y,
		R(2, 0), R(2, 1), R(2, 2), z,
		0, 0, 0, 1;

    return T;
}


void
read_data(char *filename)
{
	int n;
	FILE *f;
	double ox, oy, otheta;
	double ix, iy, itheta;
	double gx, gy, gtheta;
	double time;
	double gps_std;
	double gps_yaw;
	int gps_orientation_valid;

	if ((f = fopen(filename, "r")) == NULL)
		exit(printf("Error: Unable to open file '%s'!\n", filename));

	SE2 last_icp(0, 0, 0);

	while(!feof(f))
	{
		n = fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",
			&ox, &oy, &otheta,
			&gx, &gy, &gtheta,
			&ix, &iy, &itheta,
			&time, &gps_std,
			&gps_yaw, &gps_orientation_valid
		);

		if (n == 13) // se o num de campos lidos do scanf foi correto
		{
			Line l;

//			otheta = carmen_degrees_to_radians(otheta);
//			gtheta = carmen_degrees_to_radians(gtheta);
//			itheta = carmen_degrees_to_radians(itheta);

			l.odom = SE2(ox, oy, otheta);
			l.gps = SE2(gx, gy, gtheta);
//			l.icp = last_icp.inverse() * SE2(ix, iy, itheta);
			l.icp = SE2(ix, iy, itheta);
			l.time = time;
			l.gps_std = gps_std;
			l.gps_yaw = gps_yaw;
			l.gps_orientation_valid = gps_orientation_valid;

			input_data.push_back(l);
			last_icp = SE2(ix, iy, itheta);
		}
	}

	fclose(f);
}


void
read_loop_restrictions(char *filename)
{
	int n;
	FILE *f;
	double x, y, theta;

	if ((f = fopen(filename, "r")) == NULL)
		exit(printf("Error: Unable to open file '%s'!\n", filename));

	while(!feof(f))
	{
		LoopRestriction l;

		n = fscanf(f, "%d %d %lf %lf %lf\n",
			&l.from, &l.to, &x, &y, &theta
		);

		if (n == 5)
		{
			l.transform = SE2(x, y, theta);
			loop_data.push_back(l);
		}
	}

	fclose(f);
}


double
r2d(double angle)
{
	return (angle * 180.0) / M_PI;
}


void
add_gps_edge(SparseOptimizer *optimizer, VertexSE2 *v, SE2 measure,
						 double gps_std_from_quality_flag, double gps_xy_std_multiplier,
						 Matrix<double, 4, 4> &car2gps)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(gps_std_from_quality_flag * gps_xy_std_multiplier, 2); // Alberto
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(gps_std_from_quality_flag * gps_xy_std_multiplier, 2);
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(3.14 * 1000000, 2);
	//cov.data()[8] = pow(yaw_std, 2);

	information = cov.inverse();

	EdgeGPSNew *edge_gps = new EdgeGPSNew;
	edge_gps->set_car2gps(car2gps);
	edge_gps->vertices()[0] = optimizer->vertex(v->id());
	edge_gps->setMeasurement(measure);
	edge_gps->setInformation(information);
	optimizer->addEdge(edge_gps);
}


void
add_vertices(SparseOptimizer *optimizer)
{
	uint i;

	for (i = 0; i < input_data.size(); i++)
	{
		SE2 estimate(
			input_data[i].gps.toVector()[0] - input_data[0].gps.toVector()[0],
			input_data[i].gps.toVector()[1] - input_data[0].gps.toVector()[1],
			input_data[i].gps.toVector()[2]
		);

		VertexSE2* vertex = new VertexSE2;
		vertex->setId(i);
		vertex->setEstimate(estimate);
		optimizer->addVertex(vertex);
	}
}


void
add_odometry_edges(SparseOptimizer *optimizer, double odom_xy_std, double odom_orient_std)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(odom_xy_std, 2);
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(odom_xy_std, 2);
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(odom_orient_std, 2);

	information = cov.inverse();

	double total_dist = 0.0;
	for (size_t i = 0; i < (input_data.size() - 1); i++)
	{
		SE2 measure = input_data[i].odom.inverse() * input_data[i + 1].odom;

		double dist = sqrt(pow(measure[0], 2) + pow(measure[1], 2));
		total_dist += dist;

//		dist = dist * (1.045); // Alberto 1.0293
		measure.setTranslation(Vector2d(dist * cos(measure[2]), dist * sin(measure[2])));

		if (abs(input_data[i + 1].time - input_data[i].time) > 10)
			continue;

		if (i >= BEGIN_ID && i <= END_ID)
		{
			// SE2 bla_teste = input_data[BEGIN_ID].odom.inverse() * input_data[i + 1].odom;
			SE2 bla_teste = input_data[i + 1].odom;

			bla_teste.setTranslation(Vector2d(
				input_data[i + 1].odom[0] - input_data[BEGIN_ID].odom[0],
				input_data[i + 1].odom[1] - input_data[BEGIN_ID].odom[1]
			));

			SE2 rot_gps = input_data[0].gps;
			rot_gps.setTranslation(Vector2d(0, 0));

			bla_teste = rot_gps * bla_teste;

			printf("odom: %d %lf %lf %lf\n", (int) i + 1, bla_teste[0], bla_teste[1], carmen_radians_to_degrees(bla_teste[2]));
		 }

//		if ((fabs(measure[2]) < 0.35)) // && (sqrt(pow(measure[0], 2) + pow(measure[1], 2)) < 1.0))
		{
			EdgeSE2* edge = new EdgeSE2;
			edge->vertices()[0] = optimizer->vertex(i);
			edge->vertices()[1] = optimizer->vertex(i + 1);
			edge->setMeasurement(measure);
			edge->setInformation(information);
			optimizer->addEdge(edge);
		}
//		else
//			printf("Attention to odometry %d: %lf %lf %lf\n", (int) i, measure[0], measure[1], measure[2]);
	}

	printf("total_dist odom: %lf\n", total_dist);
}


int
find_nearest_gps(double time)
{
	uint i;
	int id;
	double time_diff;
	double min_time_diff = DBL_MAX;

	id = -1;

	for (i = 0; i < gps.size(); i++)
	{
		time_diff = fabs(time - gps[i].second);

		if (time_diff < min_time_diff)
		{
			min_time_diff = time_diff;
			id = i;
		}
	}

	return id;
}


int
find_loop_with_gps(SE2 pose, double time)
{
	uint i;
	int id;
	double dist;
	double time_diff;
	double min_dist = DBL_MAX;

	id = -1;

	for (i = 0; i < gps_to_odometry.size(); i++)
	{
		int pgps = gps_to_odometry[i].first;

		time_diff = fabs(time - gps[pgps].second);
		dist = sqrt(pow(gps[pgps].first[0] - pose[0], 2) + pow(gps[pgps].first[1] - pose[1], 2));

		if ((dist < min_dist) && (dist < 3.0) && (time_diff > 120.0))
		{
			min_dist = dist;
			id = i;
		}
	}

	return id;
}


void
add_gps_edges(SparseOptimizer *optimizer, double gps_xy_std_multiplier, Matrix<double, 4, 4> &car2gps)
{
//	int is_first = 1;
	SE2 diff(0, 0, 0);
	SE2 last_measure(0, 0, 0);

	double total_dist = 0.0;

	for (size_t i = 0; i < input_data.size(); i++)
	{
		double gps_yaw = input_data[i].gps_yaw;
		double gps_std_from_quality_flag = input_data[i].gps_std;
		SE2 gmeasure = input_data[i].gps;
		SE2 measure(gmeasure[0] - input_data[0].gps[0], gmeasure[1] - input_data[0].gps[1], gps_yaw /*gmeasure[2]*/); // subtract the first gps

		if (i > 0)
		{
			SE2 step = last_measure.inverse() * measure;
			double dist = sqrt(pow(step[0], 2) + pow(step[1], 2));
			total_dist += dist;
		}

		if (i >= BEGIN_ID && i <= END_ID)
		{
			// SE2 bla_teste = input_data[BEGIN_ID].gps.inverse() * gmeasure;
			SE2 bla_teste = gmeasure;

			bla_teste.setTranslation(Vector2d(
				gmeasure[0] - input_data[BEGIN_ID].gps[0],
				gmeasure[1] - input_data[BEGIN_ID].gps[1]
			));

			printf("gps: %d %lf %lf %lf\n", (int) i, bla_teste[0], bla_teste[1], carmen_radians_to_degrees(bla_teste[2]));
		}

//		diff = last_measure.inverse() * measure;

//		if ( (is_first) ||
//			 ((fabs(diff[2]) < 0.35))) // && (sqrt(pow(diff[0], 2) + pow(diff[1], 2)) < 1.0)))
		{
			VertexSE2 *v = dynamic_cast<VertexSE2*>(optimizer->vertices()[i]);
			add_gps_edge(optimizer, v, measure, gps_std_from_quality_flag, gps_xy_std_multiplier, car2gps);
		}
//		else
//			printf("Attention to GPS %d: %lf %lf %lf!!\n", (int) i, diff[0], diff[1], diff[2]);

//		is_first = 0;
		last_measure = measure;
	}

	printf("Total dist gps: %lf\n", total_dist);
}


void
add_loop_closure_edges(SparseOptimizer *optimizer)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(3.0, 2);
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(3.0, 2);
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(0.60, 2);

	information = cov.inverse();

	for (size_t i = 0; i < loop_data.size(); i++)
	{
		EdgeSE2* edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(loop_data[i].from);
		edge->vertices()[1] = optimizer->vertex(loop_data[i].to);
		edge->setMeasurement(loop_data[i].transform);
		edge->setInformation(information);
		optimizer->addEdge(edge);
	}
}


void
add_icp_edges(SparseOptimizer *optimizer)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(1.0, 2);
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(1.0, 2);
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(0.09, 2);

	information = cov.inverse();

	// debug:
	SE2 accum(0, 0, 0);
	SE2 accum_base;
	double total_dist = 0;

	for (size_t i = 0; i < (input_data.size() - 1); i++)
	{
		SE2 measure = input_data[i + 1].icp;

		if (i == BEGIN_ID)
			accum_base = accum;

		if (i >= BEGIN_ID && i <= END_ID)
		{
			// SE2 bla_teste = accum_base.inverse() * accum;

			SE2 bla_teste = accum;

			bla_teste.setTranslation(Vector2d(
				accum[0] - accum_base[0],
				accum[1] - accum_base[1]
			));

			SE2 rot_gps = input_data[0].gps;
			rot_gps.setTranslation(Vector2d(0, 0));
			bla_teste = rot_gps * bla_teste;

			printf("icp: %d %lf %lf %lf\n", (int) i + 1, bla_teste[0], bla_teste[1], carmen_radians_to_degrees(bla_teste[2]));
		}

		double dist = sqrt(pow(measure[0], 2) + pow(measure[1], 2));
		total_dist += dist;

		dist = dist * 1.65; // (3715.809434 / 2047.590339);
		//measure.setTranslation(Vector2d(dist * cos(measure[2]), dist * sin(measure[2])));

//		if ((fabs(measure[2]) < 0.35)) // && (sqrt(pow(measure[0], 2) + pow(measure[1], 2)) < 1.0))
//		{
			EdgeSE2* edge = new EdgeSE2;
			edge->vertices()[0] = optimizer->vertex(i);
			edge->vertices()[1] = optimizer->vertex(i + 1);
			edge->setMeasurement(measure);
			edge->setInformation(information);
			optimizer->addEdge(edge);
//		}
//		else
//			printf("Attention to icp %d: %lf %lf %lf\n", (int) i, measure[0], measure[1], measure[2]);

		// debug:
		accum = accum * measure;
		//printf("%lf %lf\n", accum[0], accum[1]);
	}

	printf("total dist icp: %lf\n", total_dist);
}


void
load_data_to_optimizer(
		SparseOptimizer *optimizer, double gps_xy_std_multiplier,
		double odom_xy_std, double odom_orient_std,
		Matrix<double, 4, 4> &car2gps)
{
	// read_data("data-log_voltadaufes-20130916-with-icp-processed.txt");
	// read_data("data-log_voltadaufes-20130916-odom-processed-without-icp.txt");
	// read_data("data-log_voltadaufes-20130916-odomevel-processed-with-icp.txt");
	// read_data("data-log_voltadaufes-20130916-ramdisk-processed-with-icp.txt");
	// read_data("data-log_voltadaufes-20130916-ramdisk-processed-with-icp-pred-odom.txt");
	read_data(input_file);
	read_loop_restrictions(loops_file);

	add_vertices(optimizer);
	add_odometry_edges(optimizer, odom_xy_std, odom_orient_std);
	add_gps_edges(optimizer, gps_xy_std_multiplier, car2gps);
	add_loop_closure_edges(optimizer);
	// add_icp_edges(optimizer);
	// exit(0);

	optimizer->save("poses_before.g2o");
	cout << "load complete!" << endl;
}


void
save_corrected_vertices(SparseOptimizer *optimizer)
{
	// FILE *f = fopen("poses_after.txt", "w");
	FILE *f = fopen(out_file, "w");

	if (f == NULL)
		exit(printf("File '%s' couldn't be opened!\n", out_file));

	// VertexSE2* vtest = dynamic_cast<VertexSE2*>(optimizer->vertex(5190));

	for (size_t i = 0; i < optimizer->vertices().size(); i++)
	{
		VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer->vertex(i));
		SE2 pose = v->estimate();

		pose.setTranslation(Vector2d(v->estimate()[0] + input_data[0].gps[0], v->estimate()[1] + input_data[0].gps[1]));
		pose.setRotation(Rotation2Dd(v->estimate()[2]));

		if (i >= BEGIN_ID && i <= END_ID)
		{
			VertexSE2* vbla = dynamic_cast<VertexSE2*>(optimizer->vertex(BEGIN_ID));
			SE2 bla_teste = vbla->estimate().inverse() * v->estimate();
			printf("graph: %d %lf %lf %lf\n", (int) i, bla_teste[0], bla_teste[1], carmen_radians_to_degrees(bla_teste[2]));
		}

		fprintf(f, "%lf %lf %lf %lf\n", pose.toVector().data()[0], pose.toVector().data()[1], pose.toVector().data()[2], input_data[i].time);
		// fprintf(f, "%lf %lf %lf %lf %d\n", vertex_data[0], vertex_data[1], vertex_data[2], input_data[i].time, (int) i);
		// fprintf(f, "%lf %lf %lf %lf\n", input_data[i].odom[0], input_data[i].odom[1], input_data[i].odom[2], input_data[i].time);
	}

	fclose(f);
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
}


SparseOptimizer*
initialize_optimizer()
{
	SparseOptimizer *optimizer = new SparseOptimizer;

	OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
	g2o::OptimizationAlgorithmProperty _currentOptimizationAlgorithmProperty;
	// _currentOptimizationAlgorithmProperty.name = "lm_pcg";
	_currentOptimizationAlgorithmProperty.name = "gn_var_cholmod";
	_currentOptimizationAlgorithmProperty.requiresMarginalize = false;
	// _currentOptimizationAlgorithmProperty.type = "PCG";
	_currentOptimizationAlgorithmProperty.type = "CHOLMOD";
	_currentOptimizationAlgorithmProperty.landmarkDim = -1;
	_currentOptimizationAlgorithmProperty.poseDim = -1;
	_currentOptimizationAlgorithmProperty.desc = ""; // "Gauss-Newton: Cholesky solver using CHOLMOD (variable blocksize)";

	// OptimizationAlgorithm *solver = solverFactory->construct("lm_pcg", _currentOptimizationAlgorithmProperty);
	OptimizationAlgorithm *solver = solverFactory->construct("gn_var_cholmod", _currentOptimizationAlgorithmProperty);

	optimizer->setAlgorithm(solver);

	return optimizer;
}


Eigen::Matrix<double, 4, 4>
get_transform_from_car_to_gps(CarmenParamFile &params, int gps_id)
{
	Eigen::Matrix<double, 4, 4> board2car, gps2board, car2gps;

	char gps_name[128];
	sprintf(gps_name, "gps_nmea_%d", gps_id);
	string gps_name_str = string(gps_name);

	gps2board = pose6d_to_matrix(
		params.get<double>(gps_name_str + "_x"),
		params.get<double>(gps_name_str + "_y"),
		params.get<double>(gps_name_str + "_z"),
		params.get<double>(gps_name_str + "_roll"),
		params.get<double>(gps_name_str + "_pitch"),
		params.get<double>(gps_name_str + "_yaw")
	);

	board2car = pose6d_to_matrix(
		params.get<double>("sensor_board_1_x"),
		params.get<double>("sensor_board_1_y"),
		params.get<double>("sensor_board_1_z"),
		params.get<double>("sensor_board_1_roll"),
		params.get<double>("sensor_board_1_pitch"),
		params.get<double>("sensor_board_1_yaw")
	);

	car2gps = (board2car * gps2board).inverse();

	return (car2gps);
}


int
main(int argc, char **argv)
{
	if (argc < 6)
		exit(printf("Use %s <input-file> <carmen_ini> <gps_id> <loops-file> <saida.txt> <gps_xy_std_multiplier (meters)> <odom_xy_std (meters)> <odom_orient_std (degrees)>\n", argv[0]));

	input_file = argv[1];
	CarmenParamFile params = CarmenParamFile(argv[2]);
	int gps_id = atoi(argv[3]);

	loops_file = argv[4];
	out_file = argv[5];

	double gps_xy_std_multiplier = 5.0;
	double odom_xy_std = 0.1;
	double odom_orient_std = 0.009;

	if (argc >= 7)
		gps_xy_std_multiplier = atof(argv[6]);

	if (argc >= 8)
		odom_xy_std = atof(argv[7]);

	if (argc >= 9)
		odom_orient_std = carmen_degrees_to_radians(atof(argv[8]));

	SparseOptimizer* optimizer;

	srand(time(NULL));

	DlWrapper dlSolverWrapper;
	loadStandardSolver(dlSolverWrapper, argc, argv);
	Factory* factory = Factory::instance();
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);

	optimizer = initialize_optimizer();

	Matrix<double, 4, 4> car2gps = get_transform_from_car_to_gps(params, gps_id);
	load_data_to_optimizer(optimizer, gps_xy_std_multiplier, odom_xy_std, odom_orient_std, car2gps);

	optimizer->setVerbose(true);
	cerr << "Optimizing" << endl;
	prepare_optimization(optimizer);
	optimizer->optimize(100);
	cerr << "OptimizationDone!" << endl;
	save_corrected_vertices(optimizer);
	cerr << "OutputSaved!" << endl;

	printf("Programa terminado normalmente. Tecle Ctrl+C para terminar\n");
	fflush(stdout);
	getchar();
	fprintf(stderr, "\n\n **** IGNORE O SEG FAULT ABAIXO. ELE EH CAUSADO PELO DESTRUTOR DO DLWRAPPER DO G2O!\n");
	
	return 0;
}
