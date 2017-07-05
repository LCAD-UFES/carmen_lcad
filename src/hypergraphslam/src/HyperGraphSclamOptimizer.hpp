#ifndef HYPERGRAPHSCLAM_OPTIMIZER_HPP
#define HYPERGRAPHSCLAM_OPTIMIZER_HPP

#include <fstream>
#include <sstream>
#include <exception>
#include <cmath>
#include <map>

#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/core/factory.h>
#include <g2o/core/hyper_dijkstra.h>

// custom edges
#include <EdgeGPS.hpp>
#include <EdgeXSENS.hpp>
#include <EdgeVelodyneCalibration.hpp>
#include <EdgeSickCalibration.hpp>
#include <EdgeSE2OdomAckermanCalibration.hpp>
#include <EdgeCurvatureConstraint.hpp>

#include <StringHelper.hpp>

namespace hyper {

#define ODOMETRY_XX_VAR 1.5
#define ODOMETRY_YY_VAR 1.5
#define ODOMETRY_HH_VAR 0.04

#define SPECIAL_ODOM_INFORMATION 1e09

#define SICK_ICP_XX_VAR 1.0
#define SICK_ICP_YY_VAR 1.0
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

#define XSENS_CONSTRAINT_VAR M_PI_4

#define CURVATURE_XX_VAR 20.0
#define CURVATURE_YY_VAR 20.0
#define CURVATURE_HH_VAR 20.0

#define GPS_POSE_STD_MULTIPLIER 20.0
#define GPS_POSE_HH_STD M_PI

#define SICK_VERTEX_OFFSET_ID 0
#define VELODYNE_VERTEX_OFFSET_ID 1

#define ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID 2
#define ODOM_ACKERMAN_PARAMS_VERTICES 20

#define OPTIMIZER_OUTER_ITERATIONS 10
#define OPTIMIZER_INNER_POSE_ITERATIONS 20
#define OPTIMIZER_INNER_ODOM_CALIB_ITERATIONS 5

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> HyperBlockSolver;
typedef g2o::LinearSolverCholmod<HyperBlockSolver::PoseMatrixType> HyperCholmodSolver;

class HyperGraphSclamOptimizer {

    private:

        // the gps origin
        Eigen::Vector2d gps_origin;

        // input filename, containing the graph that comes out from the parser
        std::string input_filename;

        // output filename, the optimization poses are going to be saved in that file
        std::string output_filename;

        // the main sparse optimizer
        g2o::SparseOptimizer *optimizer;

        // the g2o factory
        g2o::Factory *factory;

        // the timestamp and ids vectors
        std::map<unsigned, double> id_time_map;

        // hack
        std::list<g2o::EdgeGPS*> gps_buffer;

        // return a 3x3 information matrix given the diagonal covariances
        Eigen::Matrix3d GetInformationMatrix(double xx_var, double yy_var, double hh_var);

        // registering the custom edges and vertices
        void RegisterCustomTypes();

        // add the sick, velodyne and odometry calibration vertices
        void AddParametersVertices();

        // read the current vertex from the input stream and save it to the optimizer
        void AddVertex(std::stringstream &ss);

        // read the sick edge from the input stream and save it to the optimizer
        void AddSickEdge(std::stringstream &ss, Eigen::Matrix3d &sick_icp_information);

        // read the velodyne edge from the input stream and save it to the optimizer
        void AddVelodyneEdge(std::stringstream &ss, Eigen::Matrix3d &velodyne_icp_information);

        // add the odometry calibration edge to the optimizer
        void AddOdomCalibrationEdge(
            g2o::VertexSE2 *l_vertex,
            g2o::VertexSE2 *r_vertex,
            g2o::EdgeSE2 *odom_edge,
            unsigned odom_param_id,
            double vel,
            double phi,
            double time,
            const Eigen::Matrix3d &special,
            const Eigen::Matrix3d &info);

        // read the odometry edge and the odometry calibration edge and save them to the optimizer
        void AddOdometryAndCalibEdges(std::stringstream &ss, unsigned odom_param_id, const Eigen::Matrix3d &special, const Eigen::Matrix3d &odom_info);

        // gps edges filtering
        void AddFilteredGPSEdge(g2o::EdgeGPS *next_gps);

        // read the gps edge and save it to the optimizer
        void AddGPSEdge(std::stringstream &ss, Eigen::Matrix3d &cov);

        // read the xsens edge and save it to the optimizer
        void AddXSENSEdge(std::stringstream &ss, Eigen::Matrix<double, 1, 1> &information);

        // read the curvature edge and save it to the optimizer
        void AddCurvatureConstraintEdge(std::stringstream &ss, Eigen::Matrix3d &information);

        // initialize the sparse optimizer
        void InitializeOptimizer();

        // manage the hypergraph region
        void ManageHypergraphRegion(std::vector<g2o::VertexSE2*> &group, bool status);

        // reset the graph to the pose estimation
        void PreparePrevOptimization();

        // reset the graph to the odometry calibration estimation
        void PreparePostOptimization();

        // reset the graph to the next optimization round
        void PrepareRoundOptimization();

        // load the data into the optimizer
        void LoadHyperGraphToOptimizer();

        // save the optimized estimates to the output file
        void SaveCorrectedVertices();

        // the main optimization loop
        void OptimizationLoop();

    public:

        // the basic constructor
        HyperGraphSclamOptimizer(int argc, char **argv);

        // basic destructor
        ~HyperGraphSclamOptimizer();

        // verify if the optimizer is ready
        bool Good();

        // the main run method
        void Run();

        // clear the entire hypergraph
        void Clear();

};


}

#endif
