#include <carmen/carmen.h>
#include <carmen/util_io.h>

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

#include <carmen/carmen_param_file.h>
#include <carmen/command_line.h>
#include <carmen/tf.h>

#include "edge_gps_2D.h"
#include "edge_gps_2D_new.h"


using namespace tf;
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

char input_file[1024];
char loops_file[1024];
char carmen_ini_file[1024];
char out_file[1024];


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
		// x, y, theta da odometria (pos base_ackerman)
		// x, y, theta do gps_xyz
		// 0.0, 0.0, 0.0 - não usado mais
		// timestamp do Velodyne
		// gps_std (ver gps_xyz_message_handler() em grab_data.cpp)
		// gps_yaw - mesmo que o theta de gps_xyz, acima
		// gps_orientation_valid - flag que diz se o yaw do gps é valido
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

		// índice de uma núvem do Velodyne, índice de outra núvem do Velodyne, deslocamento x, y, theta da primeira para a segunda núvem
		n = fscanf(f, "%d %d %lf %lf %lf\n", &l.from, &l.to, &x, &y, &theta);

		if (n == 5)
		{
			l.transform = SE2(x, y, theta);
			loop_data.push_back(l);
		}
	}

	fclose(f);
}


void
add_and_initialize_vertices(SparseOptimizer *optimizer, tf::Transformer *transformer)
{
	uint i;

	tf::StampedTransform gps_to_car;
	transformer->lookupTransform("/gps", "/car", tf::Time(0), gps_to_car);
//	printf("gps with respect to car: x: %lf, y: %lf, z: %lf\n", gps_to_car.getOrigin().x(), gps_to_car.getOrigin().y(), gps_to_car.getOrigin().z());

	double x_ = gps_to_car.getOrigin().x();
	double y_ = gps_to_car.getOrigin().y();

//	FILE *caco = fopen("caco.txt", "w");
	for (i = 0; i < input_data.size(); i++)
	{
		double gps_yaw = input_data[i].gps[2];
		double gps_x = input_data[i].gps[0] - input_data[0].gps[0];
		double gps_y = input_data[i].gps[1] - input_data[0].gps[1];
		double car_pose_in_the_world_x = x_ * cos(gps_yaw) - y_ * sin(gps_yaw) + gps_x;
		double car_pose_in_the_world_y = x_ * sin(gps_yaw) + y_ * cos(gps_yaw) + gps_y;

//		fprintf(caco, "%lf %lf %lf %lf\n", gps_x + input_data[0].gps[0], gps_y + input_data[0].gps[1],
//				car_pose_in_the_world_x + input_data[0].gps[0], car_pose_in_the_world_y + input_data[0].gps[1]);

		// Cada estimate é o valor inicial de um vértice que representa (é) uma pose do carro quando a núvem de pontos do Velodyne i foi capturada.
		SE2 estimate(car_pose_in_the_world_x, car_pose_in_the_world_y, gps_yaw);

		VertexSE2 *vertex = new VertexSE2;
		vertex->setId(i);
		vertex->setEstimate(estimate);
		optimizer->addVertex(vertex);
	}
//	fclose(caco);
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
		// Na linha abaixo, o "*" é um overload do * que calcula a delta_odometria (delta_x, delta_y, delta_theta) entre a odometria i + 1 e i.
		SE2 measure = input_data[i].odom.inverse() * input_data[i + 1].odom;

		EdgeSE2 *edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(i);
		edge->vertices()[1] = optimizer->vertex(i + 1);
		edge->setMeasurement(measure);
		edge->setInformation(information);
		optimizer->addEdge(edge);

		double dist = sqrt(pow(measure[0], 2) + pow(measure[1], 2));
		total_dist += dist;
	}

	printf("total_dist odom: %lf\n", total_dist);
}


void
add_gps_edge(SparseOptimizer *optimizer, VertexSE2 *v, SE2 measure,
						 double gps_std_from_quality_flag, double gps_xy_std_multiplier, double gps_yaw_std,
						 tf::Transformer *transformer)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(gps_std_from_quality_flag * gps_xy_std_multiplier, 2.0);
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(gps_std_from_quality_flag * gps_xy_std_multiplier, 2.0);
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(carmen_degrees_to_radians(gps_yaw_std), 2.0);

	information = cov.inverse();

	EdgeGPSNew *edge_gps = new EdgeGPSNew;
	edge_gps->set_car2gps(transformer);
	edge_gps->vertices()[0] = optimizer->vertex(v->id());
	edge_gps->setMeasurement(measure);
	edge_gps->setInformation(information);
	optimizer->addEdge(edge_gps);
}


void
add_gps_edges(SparseOptimizer *optimizer, double gps_xy_std_multiplier, double gps_yaw_std, tf::Transformer *transformer)
{
	SE2 diff(0, 0, 0);
	SE2 last_measure(0, 0, 0);

	double total_dist = 0.0;

	for (size_t i = 0; i < input_data.size(); i++)
	{
		double gps_yaw = input_data[i].gps_yaw;
		double gps_std_from_quality_flag = input_data[i].gps_std;
		SE2 gmeasure = input_data[i].gps;
		SE2 measure(gmeasure[0] - input_data[0].gps[0], gmeasure[1] - input_data[0].gps[1], gps_yaw /*gmeasure[2]*/); // subtract the first gps

		VertexSE2 *v = dynamic_cast<VertexSE2*>(optimizer->vertices()[i]);
		add_gps_edge(optimizer, v, measure, gps_std_from_quality_flag, gps_xy_std_multiplier, gps_yaw_std, transformer);

		last_measure = measure;

		if (i > 0)
		{
			SE2 step = last_measure.inverse() * measure;
			double dist = sqrt(pow(step[0], 2) + pow(step[1], 2));
			total_dist += dist;
		}
	}

	printf("Total dist gps: %lf\n", total_dist);
}


void
add_loop_closure_edges(SparseOptimizer *optimizer, double loop_xy_std, double loop_orient_std)
{
	Matrix3d cov;
	Matrix3d information;

	cov.data()[0] = pow(loop_xy_std, 2);	// xy std
	cov.data()[1] = 0;
	cov.data()[2] = 0;
	cov.data()[3] = 0;
	cov.data()[4] = pow(loop_xy_std, 2);	// xy std
	cov.data()[5] = 0;
	cov.data()[6] = 0;
	cov.data()[7] = 0;
	cov.data()[8] = pow(loop_orient_std, 2);	// yaw std (radians)

	information = cov.inverse();

	for (size_t i = 0; i < loop_data.size(); i++)
	{
		EdgeSE2 *edge = new EdgeSE2;
		edge->vertices()[0] = optimizer->vertex(loop_data[i].from);
		edge->vertices()[1] = optimizer->vertex(loop_data[i].to);
		edge->setMeasurement(loop_data[i].transform);
		edge->setInformation(information);
		optimizer->addEdge(edge);
	}
}


//void
//add_icp_edges(SparseOptimizer *optimizer)
//{
//	Matrix3d cov;
//	Matrix3d information;
//
//	cov.data()[0] = pow(1.0, 2);
//	cov.data()[1] = 0;
//	cov.data()[2] = 0;
//	cov.data()[3] = 0;
//	cov.data()[4] = pow(1.0, 2);
//	cov.data()[5] = 0;
//	cov.data()[6] = 0;
//	cov.data()[7] = 0;
//	cov.data()[8] = pow(0.09, 2);
//
//	information = cov.inverse();
//
//	for (size_t i = 0; i < (input_data.size() - 1); i++)
//	{
//		SE2 measure = input_data[i + 1].icp;
//
//		EdgeSE2 *edge = new EdgeSE2;
//		edge->vertices()[0] = optimizer->vertex(i);
//		edge->vertices()[1] = optimizer->vertex(i + 1);
//		edge->setMeasurement(measure);
//		edge->setInformation(information);
//		optimizer->addEdge(edge);
//	}
//}


void
build_optimization_graph(SparseOptimizer *optimizer,
		double gps_xy_std_multiplier, double gps_yaw_std,
		double odom_xy_std, double odom_orient_std,
		double loop_xy_std, double loop_orient_std,
		tf::Transformer *transformer)
{
	// A fução read_data(), abaixo, lê a saída do grab_data, que é uma arquivo txt (sync.txt)) onde cada linha contém um vetor
	// com vários dados sincronizados (projetando ackerman) com cada núvem de pontos do Velodyne, sendo eles:
	// x, y, theta da odometria (pós base_ackerman)
	// x, y, theta do gps_xyz
	// 0.0, 0.0, 0.0 - não usado mais
	// timestamp do Velodyne
	// gps_std (ver gps_xyz_message_handler() em grab_data.cpp)
	// gps_yaw - mesmo que o theta de gps_xyz, acima
	// gps_orientation_valid - flag que diz se o yaw do gps é valido
	read_data(input_file);

	// A fução read_loop_restrictions(), abaixo, lê a saída do run_icp_for_loop_closure, que é uma arquivo txt onde cada linha
	// contém o vetor:
	// índice de uma núvem do Velodyne, índice de outra núvem do Velodyne, deslocamento x, y, theta da primeira para a
	// segunda núvem (as outras coordenadas são jogadas fora - não há uma projeção explicita para o plano x, y)
	read_loop_restrictions(loops_file);

	// Cria e inicializa vértices de um grafo, onde cada vertice é a pose (x, y, theta) do carro
	add_and_initialize_vertices(optimizer, transformer);

	// Para cada par de vértices, adiciona uma aresta de odometria, que é um delta_x, delta_y, delta_theta.
	add_odometry_edges(optimizer, odom_xy_std, odom_orient_std);

	// Para cada par de vértices, adiciona uma aresta de gps, que é um x, y, theta.
	add_gps_edges(optimizer, gps_xy_std_multiplier, gps_yaw_std, transformer);

	add_loop_closure_edges(optimizer, loop_xy_std, loop_orient_std);
	// add_icp_edges(optimizer);

	optimizer->save("poses_before.g2o");
	cout << "load complete!" << endl;
}


void
save_corrected_vertices(SparseOptimizer *optimizer, tf::Transformer *transformer)
{
	FILE *f = fopen(out_file, "w");

	if (f == NULL)
		exit(printf("File '%s' couldn't be opened!\n", out_file));

	VertexSE2 *v = dynamic_cast<VertexSE2*>(optimizer->vertex(0));
	SE2 car_pose_0 = v->estimate();

	tf::StampedTransform gps_to_car;
	transformer->lookupTransform("/gps", "/car", tf::Time(0), gps_to_car);
//	printf("gps with respect to car: x: %lf, y: %lf, z: %lf\n", gps_to_car.getOrigin().x(), gps_to_car.getOrigin().y(), gps_to_car.getOrigin().z());

	double x_ = gps_to_car.getOrigin().x();
	double y_ = gps_to_car.getOrigin().y();
	double yaw = car_pose_0[2];
	double car_pose_0_in_the_world_x = x_ * cos(yaw) - y_ * sin(yaw) + input_data[0].gps[0];
	double car_pose_0_in_the_world_y = x_ * sin(yaw) + y_ * cos(yaw) + input_data[0].gps[1];
//	printf("car_pose_0_in_the_world (%lf, %lf)\n", car_pose_0_in_the_world_x, car_pose_0_in_the_world_y);

	for (size_t i = 0; i < optimizer->vertices().size(); i++)
	{
		v = dynamic_cast<VertexSE2*>(optimizer->vertex(i));
		SE2 pose = v->estimate();

		pose.setTranslation(Vector2d(v->estimate()[0] + car_pose_0_in_the_world_x, v->estimate()[1] + car_pose_0_in_the_world_y));
		pose.setRotation(Rotation2Dd(v->estimate()[2]));

		fprintf(f, "%lf %lf %lf %lf\n", pose.toVector().data()[0], pose.toVector().data()[1], pose.toVector().data()[2], input_data[i].time);
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

	OptimizationAlgorithmFactory *solverFactory = OptimizationAlgorithmFactory::instance();
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

	return (optimizer);
}


tf::Transform
read_object_pose_by_name_from_carmen_ini(CarmenParamFile *params, string object_name)
{
	double x, y, z, roll, pitch, yaw;

	x = params->get<double>(object_name + "_x");
	y = params->get<double>(object_name + "_y");
	z = params->get<double>(object_name + "_z");
	roll = params->get<double>(object_name + "_roll");
	pitch = params->get<double>(object_name + "_pitch");
	yaw = params->get<double>(object_name + "_yaw");

	return tf::Transform(tf::Quaternion(yaw, pitch, roll), tf::Vector3(x, y, z));
}


void
initialize_tf_transfoms(CarmenParamFile *params, Transformer *transformer, int gps_id)
{
	char gps_name[128];

	sprintf(gps_name, "gps_nmea_%d", gps_id);

	transformer->setTransform(tf::StampedTransform(read_object_pose_by_name_from_carmen_ini(params, "sensor_board_1"), tf::Time(0), "/car", "/board"));
	transformer->setTransform(tf::StampedTransform(read_object_pose_by_name_from_carmen_ini(params, gps_name), tf::Time(0), "/board", "/gps"));

//	tf::StampedTransform a_to_b;
//	transformer->lookupTransform("/car", "/board", tf::Time(0), a_to_b);
//	printf("board with respect to car: x: %lf, y: %lf, z: %lf\n", a_to_b.getOrigin().x(), a_to_b.getOrigin().y(), a_to_b.getOrigin().z());
//	transformer->lookupTransform("/board", "/gps", tf::Time(0), a_to_b);
//	printf("gps with respect to board: x: %lf, y: %lf, z: %lf\n", a_to_b.getOrigin().x(), a_to_b.getOrigin().y(), a_to_b.getOrigin().z());
//	transformer->lookupTransform("/car", "/gps", tf::Time(0), a_to_b);
//	printf("gps with respect to car: x: %lf, y: %lf, z: %lf\n", a_to_b.getOrigin().x(), a_to_b.getOrigin().y(), a_to_b.getOrigin().z());
}


void
graphslam(int gps_id, double gps_xy_std_multiplier, double gps_yaw_std,
		double odom_xy_std, double odom_orient_std,
		double loop_xy_std, double loop_orient_std,
		int argc, char **argv)
{
	srand(time(NULL));

	CarmenParamFile *params = new CarmenParamFile(carmen_ini_file);
	tf::Transformer *transformer = new tf::Transformer(false);
	initialize_tf_transfoms(params, transformer, gps_id);

	DlWrapper dlSolverWrapper;
	loadStandardSolver(dlSolverWrapper, argc, argv);

	Factory *factory = Factory::instance();
	factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);
	factory->registerType("EDGE_GPS_NEW", new HyperGraphElementCreator<EdgeGPSNew>);

	SparseOptimizer *optimizer = initialize_optimizer();
	build_optimization_graph(optimizer, gps_xy_std_multiplier, gps_yaw_std, odom_xy_std, odom_orient_std, loop_xy_std, loop_orient_std, transformer);
	optimizer->setVerbose(true);

	cerr << "Optimizing" << endl;
	prepare_optimization(optimizer);
	optimizer->optimize(50);
	cerr << "OptimizationDone!" << endl;

	delete (transformer);
	transformer = new tf::Transformer(false);
	delete (params);
	params = new CarmenParamFile(carmen_ini_file);
	initialize_tf_transfoms(params, transformer, gps_id);
	save_corrected_vertices(optimizer, transformer);

	cerr << "OutputSaved!" << endl;

	printf("Programa concluído normalmente. Tecle Ctrl+C para terminar\n");
	fflush(stdout);
	getchar();
	fprintf(stderr, "\n\n **** IGNORE O SEG FAULT ABAIXO. ELE EH CAUSADO PELO DESTRUTOR DO DLWRAPPER DO G2O!\n");
}


void
declare_and_parse_args(int argc, char **argv, CommandLineArguments *args)
{
	args->add_positional<string>("sync.txt", "Synchronized sensors data");
	args->add_positional<string>("loops.txt", "Points cloud pairs closing loops");
	args->add_positional<string>("carmen.ini", "Path to a file containing system parameters");
	args->add_positional<string>("calibrated_odometry.txt", "Path to a file containing the odometry calibration data");
	args->add_positional<string>("poses_opt.txt", "Path to a file in which the poses will be saved in graphslam format");
	args->add<double>("gps_xy_std_multiplier", "Multiplier of the standard deviation of the gps position (times meters)", 5.0);
	args->add<double>("gps_yaw_std", "GPS yaw standard deviation (degrees)", 1000.0);
	args->add<double>("odom_xy_std", "Odometry position (x, y) standard deviation (meters)", 0.1);
	args->add<double>("odom_orient_std", "Odometry orientation (yaw) standard deviation (degrees)", 1.0);
	args->add<double>("loop_xy_std", "Loop closure delta position (x, y) standard deviation (meters)", 3.0);
	args->add<double>("loop_orient_std", "Loop closure orientation (yaw) standard deviation (degrees)", 34.0);

	args->parse(argc, argv);
	args->save_config_file("graphslam_config_default.txt");
}


int
main(int argc, char **argv)
{
	CommandLineArguments args;

	declare_and_parse_args(argc, argv, &args);

	strcpy(input_file, (char *) args.get<string>("sync.txt").c_str());
	strcpy(loops_file, (char *) args.get<string>("loops.txt").c_str());
	strcpy(carmen_ini_file, (char *) args.get<string>("carmen.ini").c_str());
	strcpy(out_file, (char *) args.get<string>("poses_opt.txt").c_str());

	FILE *odometry_calibration_file = safe_fopen(args.get<string>("calibrated_odometry.txt").c_str(), "r");
	double v_multiplier, v_bias, phi_multiplier, phi_bias, initial_angle, gps_latency, L;
	int gps_to_use;
	fscanf(odometry_calibration_file, "v (multiplier bias): (%lf %lf),  phi (multiplier bias): (%lf %lf),  Initial Angle: %lf, GPS to use: %d, GPS Latency: %lf, L: %lf",
			&v_multiplier, &v_bias, &phi_multiplier, &phi_bias, &initial_angle, &gps_to_use, &gps_latency, &L);
	int gps_id = gps_to_use;

	double gps_xy_std_multiplier = 	args.get<double>("gps_xy_std_multiplier");
	double gps_yaw_std = args.get<double>("gps_yaw_std");
	double odom_xy_std = args.get<double>("odom_xy_std");
	double odom_orient_std = carmen_degrees_to_radians(args.get<double>("odom_orient_std"));
	double loop_xy_std = args.get<double>("loop_xy_std");
	double loop_orient_std = carmen_degrees_to_radians(args.get<double>("loop_orient_std"));

	graphslam(gps_id, gps_xy_std_multiplier, gps_yaw_std, odom_xy_std, odom_orient_std, loop_xy_std, loop_orient_std, argc, argv);
	
	return (0);
}
