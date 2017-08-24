#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <g2o/apps/g2o_cli/dl_wrapper.h>
#include <g2o/apps/g2o_cli/g2o_common.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/core/hyper_dijkstra.h"
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/stuff/macros.h>
#include <g2o/core/optimizable_graph.h>

// custom edges
#include <EdgeGPS.hpp>
#include <EdgeSickCalibration.hpp>
#include <EdgeVelodyneCalibration.hpp>

#include <Helpers/StringHelper.hpp>

#include <fstream>
#include <sstream>
#include <exception>
#include <cmath>
#include <map>

#define ODOMETRY_XX_VAR 1.5
#define ODOMETRY_YY_VAR 1.5
#define ODOMETRY_HH_VAR 0.04

#define SICK_ICP_XX_VAR 0.5
#define SICK_ICP_YY_VAR 0.5
#define SICK_ICP_HH_VAR M_PI_4

#define SICK_LOOP_ICP_XX_VAR 0.8
#define SICK_LOOP_ICP_YY_VAR 0.8
#define SICK_LOOP_ICP_HH_VAR M_PI_4

#define VELODYNE_ICP_XX_VAR 1.5
#define VELODYNE_ICP_YY_VAR 1.5
#define VELODYNE_ICP_HH_VAR 0.04

#define VELODYNE_LOOP_ICP_XX_VAR 1.5
#define VELODYNE_LOOP_ICP_YY_VAR 1.5
#define VELODYNE_LOOP_ICP_HH_VAR 0.3

#define CURVATURE_CONSTRAINT_XX_VAR 20.0
#define CURVATURE_CONSTRAINT_YY_VAR 20.0
#define CURVATURE_CONSTRAINT_HH_VAR 20.0

#define GPS_POSE_STD_MULTIPLIER 20.0
#define GPS_POSE_HH_STD M_PI * 200.0

#define SICK_VERTEX_OFFSET_ID 0
#define VELODYNE_VERTEX_OFFSET_ID 1

#define OPTIMIZER_ITERATIONS 300

g2o::SparseOptimizer*
initialize_optimizer() {

    // creates a new sparse optimizer in memmory
    g2o::SparseOptimizer *optimizer = new g2o::SparseOptimizer;

    // it creates a new solver based on the properties, g2o easy way
    g2o::OptimizationAlgorithmFactory* solverFactory = g2o::OptimizationAlgorithmFactory::instance();

    // the property object define the solver
    g2o::OptimizationAlgorithmProperty _currentOptimizationAlgorithmProperty;

    _currentOptimizationAlgorithmProperty.name = "gn_var_cholmod";
    // _currentOptimizationAlgorithmProperty.type = "PCG";
    _currentOptimizationAlgorithmProperty.type = "CHOLMOD";
    _currentOptimizationAlgorithmProperty.landmarkDim = -1;
    _currentOptimizationAlgorithmProperty.poseDim = -1;
    _currentOptimizationAlgorithmProperty.desc = ""; // "Gauss-Newton: Cholesky solver using CHOLMOD (variable blocksize)";

    // OptimizationAlgorithm *solver = solverFactory->construct("lm_pcg", _currentOptimizationAlgorithmProperty);
    g2o::OptimizationAlgorithm *solver = solverFactory->construct("gn_var_cholmod", _currentOptimizationAlgorithmProperty);

    // set the cholmod solver
    optimizer->setAlgorithm(solver);

    // delete the solver factory
    g2o::OptimizationAlgorithmFactory::destroy();

    return optimizer;

}

void
prepare_optimization(g2o::SparseOptimizer *optimizer) {

    for (g2o::SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it) {

        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);

        // true => this node should be marginalized out during the optimization
        v->setMarginalized(false);

    }

    // Initializes the structures for optimizing the whole graph.
    optimizer->initializeOptimization();

    for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {

        g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);

        // specify the robust kernel to be used in this edge
        e->setRobustKernel(0);

    }

    // pre-compute the active errors
    optimizer->computeActiveErrors();

}

// load the data into the optimizer
void
load_hypergraph_to_optimizer(
    const std::string &input_filename,
    g2o::SparseOptimizer *optimizer,
    Eigen::Vector2d &gps_origin,
    std::map<unsigned, double> &id_times) {

    // try to open the input filename
    std::ifstream is(input_filename, std::ifstream::in);

    if (!is.is_open()) {

        throw std::runtime_error("Could not read the input file!");

    }

    // the current line
    std::stringstream ss;

    // the vertex id
    unsigned vertex_id, from, to;

    // the position and heading
    double x, y, theta, timestamp;

    // set the odometry covariance matrix
    Eigen::Matrix3d cov(Eigen::Matrix3d::Zero());

    // update the diagonals
    cov.data()[0] = std::pow(ODOMETRY_XX_VAR, 2);
    cov.data()[4] = std::pow(ODOMETRY_YY_VAR, 2);
    cov.data()[8] = std::pow(ODOMETRY_HH_VAR, 2);

    // get the inverse of the covariance matrix,
    // it's the information matrix
    // in this case the odometry information matrix
    Eigen::Matrix3d odom_information(cov.inverse());

    // the special odom matrix
    Eigen::Matrix3d special_odom_information(Eigen::Matrix3d::Identity() * 1e09);

    // reset the matrix to the icp covariance matrix
    cov.data()[0] = std::pow(SICK_ICP_XX_VAR, 2);
    cov.data()[4] = std::pow(SICK_ICP_YY_VAR, 2);
    cov.data()[8] = std::pow(SICK_ICP_HH_VAR, 2);

    // get the sick icp information matrix
    Eigen::Matrix3d sick_icp_information(cov.inverse());

    // reset the matrix to the velodyne icp covariance matrix
    cov.data()[0] = std::pow(VELODYNE_ICP_XX_VAR, 2);
    cov.data()[4] = std::pow(VELODYNE_ICP_YY_VAR, 2);
    cov.data()[8] = std::pow(VELODYNE_ICP_HH_VAR, 2);

    // get the velodyne icp information matrix
    Eigen::Matrix3d velodyne_icp_information(cov.inverse());

    // reset the covariance to the gps initial values
    // the xx and yy variances will be placed inside the while loop below
    cov.data()[0] = 0.0;
    cov.data()[4] = 0.0;
    cov.data()[8] = std::pow(GPS_POSE_HH_STD, 2);

    // creates the sick offset vertex with a given initial estimate
    g2o::VertexSE2 *sick_offset = new g2o::VertexSE2;

    // set the initial estimate
    // front_bullbar_x 3.52
    // front_bullbar_y 0
    // front_bullbar_z 0.395 # 0.2
    // front_bullbar_yaw   0.0
    // front_bullbar_pitch 0.0
    // front_bullbar_roll  0.0
    sick_offset->setEstimate(g2o::SE2(3.52, 0.0, 0.0));

    // set the sick offset id 0
    sick_offset->setId(SICK_VERTEX_OFFSET_ID);

    // id_times[SICK_VERTEX_OFFSET_ID] = 0.0;
    if (!optimizer->addVertex(sick_offset)) {

        // error
        throw std::runtime_error("Could not add the sick offset vertex to the optimizer!");

    }

    // creates the velodyne offset vertex with a given initial estimate
    g2o::VertexSE2 *velodyne_offset = new g2o::VertexSE2;

    // set the velodyne initial estimate
    // sensor_board_1_x    0.572
    // sensor_board_1_y    0.0
    // sensor_board_1_z    1.394
    // sensor_board_1_yaw  0.0
    velodyne_offset->setEstimate(g2o::SE2(0.572, 0.0, 0.0));

    // set the velodyne offset id
    velodyne_offset->setId(VELODYNE_VERTEX_OFFSET_ID);

    // save the offset vertex
    if (!optimizer->addVertex(velodyne_offset)) {

        // error
        throw std::runtime_error("Could not add the velodyne offset vertex to the optimizer!");

    }

    unsigned vertex_counter = 0;
    unsigned odom_counter = 0;
    unsigned gps_counter = 0;

    // the main loop
    while(-1 != hyper::StringHelper::ReadLine(is, ss)) {

        // the tag
        std::string tag;

        // read the tag
        ss >> tag;

        // the VERTEX tags should become first in the sync file
        if ("VERTEX" == tag) {

            // creates the new vertex
            g2o::VertexSE2 *v = new g2o::VertexSE2;

            // read the vertex id and the pose
            ss >> vertex_id >> x >> y >> theta >> timestamp;

            // set the id timestamps
            id_times[vertex_id] = timestamp;

            // set the id
            v->setId(vertex_id);

            // set the tranlation
            v->setEstimate(g2o::SE2(x, y, theta));

            // add to the optimizer
            if(!optimizer->addVertex(v)) {

                // error
                throw std::runtime_error("Could not add a new vertex to the optimizer!");

            }

            ++vertex_counter;

        } else if ("ODOM_EDGE" == tag) {

            // creates a new edge
            g2o::EdgeSE2 *edge = new g2o::EdgeSE2;

            // read the vertices ids and the measure
            ss >> from >> to >> x >> y >> theta;

            // set the vertices
            edge->vertices()[0] = optimizer->vertex(from);
            edge->vertices()[1] = optimizer->vertex(to);

            // set the measure
            edge->setMeasurement(g2o::SE2(x, y, theta));

            // take care of gps messages with same timestamps
            if (0.0 == x && 0.0 == y && 0.0 == theta) {

                // set the special info
                edge->setInformation(special_odom_information);

            } else {
            
                // set the info
                edge->setInformation(odom_information);
            
            }

            if (!optimizer->addEdge(edge)) {

                // error
                throw std::runtime_error("Could not add the odometry edge to the optimizer!");

            }

            ++odom_counter;

        } else if ("GPS_EDGE" == tag){

            // creates a new gps edge
            g2o::EdgeGPS *gps_edge = new g2o::EdgeGPS;

            // the standard deviation
            double gps_std;

            // read the vertex id, gps measure and std deviation
            ss >> from >> x >> y >> theta >> gps_std;

            // set the vertices
            gps_edge->vertices()[0] = optimizer->vertex(from);

            // set the measurement
            gps_edge->setMeasurement(g2o::SE2(x, y, theta));

            // update the diagonals
            cov.data()[0] = std::pow(gps_std * GPS_POSE_STD_MULTIPLIER, 2);
            cov.data()[4] = std::pow(gps_std * GPS_POSE_STD_MULTIPLIER, 2);

            // set the info matrix
            gps_edge->setInformation(Eigen::Matrix3d(cov.inverse()));

            if (!optimizer->addEdge(gps_edge)) {

                // error
                throw std::runtime_error("Could not add the gps edge to the optimizer!");

            }

            ++gps_counter;

        } else if ("SICK_SEQ_" == tag) {

            // read the ids and icp measure
            ss >> from >> to >> x >> y >> theta;

            // create the new calibration edge
            g2o::EdgeSickCalibration *sick_seq_edge = new g2o::EdgeSickCalibration;

            // set the measurement
            sick_seq_edge->setMeasurement(g2o::SE2(x, y, theta));

            // set the position estimates vertices
            sick_seq_edge->vertices()[0] = optimizer->vertex(from);
            sick_seq_edge->vertices()[1] = optimizer->vertex(to);

            // set the sick offset vertex
            sick_seq_edge->vertices()[2] = optimizer->vertex(SICK_VERTEX_OFFSET_ID);

            // set the sick icp information matrix
            sick_seq_edge->setInformation(sick_icp_information);

            // try to append the the sick seq edge
            if (!optimizer->addEdge(sick_seq_edge)) {

                // error
                throw std::runtime_error("Could not add the sick seq icp edge to the optimizer!");

            }

        } else if ("SICK_LOOP" == tag) {

        } else if ("VELODYNE_SEQ" == tag) {

            // read the ids and icp measure
            ss >> from >> to >> x >> y >> theta;

            // create the new calibration edge
            g2o::EdgeVelodyneCalibration *velodyne_seq_edge = new g2o::EdgeVelodyneCalibration;

            // set the measurement
            velodyne_seq_edge->setMeasurement(g2o::SE2(x, y, theta));

            // set the position estimates vertices
            velodyne_seq_edge->vertices()[0] = optimizer->vertex(from);
            velodyne_seq_edge->vertices()[1] = optimizer->vertex(to);

            // set the velodyne offset vertex
            velodyne_seq_edge->vertices()[2] = optimizer->vertex(VELODYNE_VERTEX_OFFSET_ID);

            // set the velodyne icp information matrix
            velodyne_seq_edge->setInformation(velodyne_icp_information);

            // try to append the the velodyne seq edge
            if (!optimizer->addEdge(velodyne_seq_edge)) {

                // error
                throw std::runtime_error("Could not add the velodyne seq icp edge to the optimizer!");

            }

        } else if ("VELODYNE_LOOP" == tag) {

        } else if ("BUMBLEBEE_SEQ" == tag) {

        } else if ("BUMBLEBE_LOOP" == tag) {

        } else if ("GPS_ORIGIN" == tag) {

            // read the origin
            ss >> gps_origin[0] >> gps_origin[1];

            std::cout << "GPS origin: " << std::fixed << gps_origin.transpose() << std::endl;

        }

    }

    std::cout << "Vertex counter: " << vertex_counter << std::endl;
    std::cout << "Odom counter: " << odom_counter << std::endl;
    std::cout << "GPS counter: " << gps_counter << std::endl;

}

void
save_corrected_vertices(const std::string &output_filename, const std::map<unsigned, double> &id_time_map, const Eigen::Vector2d &gps_origin, g2o::SparseOptimizer *optimizer) {

    // open the output file
    std::ofstream ofs(output_filename, std::ofstream::out);

    if (!ofs.is_open()) {

        throw std::runtime_error("Could not open the output file!");

    }

    // how many vertices
    unsigned size = optimizer->vertices().size();

    // report
    std::cout << "How many vertices: " << size << std::endl;

    // the first vertex is the sick displacement
    // downcast to the base vertex
    g2o::VertexSE2* sick_offset = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(SICK_VERTEX_OFFSET_ID));

    // show the resulting offset
    std::cout << std::endl << "The SICK offset: " << std::fixed << sick_offset->estimate().toVector().transpose() << std::endl;

    // the second vertex is the velodyne displacement
    // downcast to the base vertex
    g2o::VertexSE2* velodyne_offset = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(VELODYNE_VERTEX_OFFSET_ID));

    // show the resulting offset
    std::cout << std::endl << "The Velodyne offset: " << std::fixed << std::setprecision(10) << velodyne_offset->estimate().toVector().transpose() << std::endl;

    // iterators
    g2o::SparseOptimizer::VertexIDMap::iterator it(optimizer->vertices().begin());
    g2o::SparseOptimizer::VertexIDMap::iterator end(optimizer->vertices().end());

    while (end != it) {

        // downcast to the base vertex
        g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(it->second);

        if (nullptr != v && 2 < v->id()) {

            // get the estimate
            Eigen::Vector3d pose(v->estimate().toVector());

            // build the base
            pose[0] += gps_origin[0];
            pose[1] += gps_origin[1];

            if (0 > v->id()) {

                // error
                throw std::runtime_error("Invalid verteix id");

            }

            // write to the output file
            ofs << std::fixed << pose[0] << " " << pose[1] << " " << pose[2] << " " << id_time_map.at((unsigned) v->id()) << "\n";

        }

        // go to the next vertex
        ++it;

    }

    // close the output file
    ofs.close();

    (void) gps_origin;

}

int main(int argc, char **argv) {

    if (argc != 3) {

        // error
        std::cerr << "Usage: hypergraphslam <sync_file> <output_file>\n";

        return -1;

    }

    // get the input filename
    std::string input_filename(argv[1]);
    std::string output_filename(argv[2]);

    // the main sparse optimizer
    g2o::SparseOptimizer *optimizer;

    // rand?

    // the library loader
    g2o::DlWrapper dlSolverWrapper;

    // load the libraries
    g2o::loadStandardSolver(dlSolverWrapper, argc, argv);

    // registering the custom edges
    g2o::Factory *factory = g2o::Factory::instance();

    // register the custom gps edge
    factory->registerType("EDGE_GPS", new g2o::HyperGraphElementCreator<g2o::EdgeGPS>);

    // register the custom sick edge
    factory->registerType("EDGE_SICK_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeSickCalibration>);

    // register the custom velodyne edge
    factory->registerType("EDGE_VELODYNE_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeVelodyneCalibration>);

    // load the new optimizer to memmory
    optimizer = initialize_optimizer();

    // the timestamp and ids vectors
    // I'm using the bad design ugly pointer in order to avoid a mess with the g2o libraries
    // it's easier for now than find the bug inside the old g2o library
    // TODO  move to the new g2o implementation from Kummerle repo
    std::map<unsigned, double> *id_times = new std::map<unsigned, double>();

    // the gps origin
    Eigen::Vector2d gps_origin(0.0, 0.0);

    // load the data
    load_hypergraph_to_optimizer(input_filename, optimizer, gps_origin, *id_times);

    // set the verbose mode
    optimizer->setVerbose(true);

    // optimzization status report
    std::cout << "Optimizing " << optimizer->vertices().size() << " vertices" << std::endl;

    // set the internal flags
    prepare_optimization(optimizer);

    // the main optimization process
    // the input value is the maximum number of iterations
    optimizer->optimize(OPTIMIZER_ITERATIONS);

    // optimzization status report
    std::cout << "Optimization Done!" << std::endl;

    // save the optimized graph to the output file
    save_corrected_vertices(output_filename, *id_times, gps_origin, optimizer);

    // destroy the factory
    g2o::Factory::destroy();

    (void) optimizer;

    // clear the id timestamps map
    id_times->clear();

    delete id_times;

    std::cout << "HyperGraphSCLAM end!" << std::endl;

    return 0;

}
