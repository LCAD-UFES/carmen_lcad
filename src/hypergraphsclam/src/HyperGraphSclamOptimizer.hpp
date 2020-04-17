#ifndef HYPERGRAPHSCLAM_OPTIMIZER_HPP
#define HYPERGRAPHSCLAM_OPTIMIZER_HPP

#include <fstream>
#include <sstream>
#include <exception>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <list>

#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_dijkstra.h>

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

// custom edges
#include <EdgeGPS.hpp>

namespace hyper {

#define DEFAULT_ODOMETRY_XX_VAR 0.1
#define DEFAULT_ODOMETRY_YY_VAR 0.1
#define DEFAULT_ODOMETRY_HH_VAR 0.009

#define DEFAULT_SPECIAL_ODOMETRY_INFORMATION 1e09

#define DEFAULT_SICK_ICP_XX_VAR 1.8
#define DEFAULT_SICK_ICP_YY_VAR 1.8
#define DEFAULT_SICK_ICP_HH_VAR 0.8

#define DEFAULT_SICK_LOOP_ICP_XX_VAR 0.4
#define DEFAULT_SICK_LOOP_ICP_YY_VAR 0.4
#define DEFAULT_SICK_LOOP_ICP_HH_VAR 0.2

#define DEFAULT_VELODYNE_ICP_XX_VAR 0.05
#define DEFAULT_VELODYNE_ICP_YY_VAR 0.05
#define DEFAULT_VELODYNE_ICP_HH_VAR 0.05

#define DEFAULT_VELODYNE_LOOP_ICP_XX_VAR 0.25
#define DEFAULT_VELODYNE_LOOP_ICP_YY_VAR 0.25
#define DEFAULT_VELODYNE_LOOP_ICP_HH_VAR 0.05

#define DEFAULT_VELODYNE_EXTERNAL_LOOP_ICP_XX_VAR 1.0
#define DEFAULT_VELODYNE_EXTERNAL_LOOP_ICP_YY_VAR 1.0
#define DEFAULT_VELODYNE_EXTERNAL_LOOP_ICP_HH_VAR 0.5

#define DEFAULT_XSENS_CONSTRAINT_VAR M_PI_4

#define DEFAULT_CURVATURE_XX_VAR 0.1
#define DEFAULT_CURVATURE_YY_VAR 0.1
#define DEFAULT_CURVATURE_HH_VAR M_PI_4

#define DEFAULT_VISUAL_XX_VAR 0.45
#define DEFAULT_VISUAL_YY_VAR 0.45
#define DEFAULT_VISUAL_HH_VAR 0.1

#define DEFAULT_GPS_POSE_STD_MULTIPLIER 5.0
#define DEFAULT_GPS_POSE_HH_STD M_PI * 2.0

#define SICK_VERTEX_OFFSET_ID 0
#define VELODYNE_VERTEX_OFFSET_ID 1
#define BUMBLEBEE_VERTEX_OFFSET_ID 2

#define ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID 3
#define DEFAULT_ODOM_ACKERMAN_PARAMS_VERTICES 4

#define DEFAULT_OPTIMIZER_OUTER_ITERATIONS 1
#define DEFAULT_OPTIMIZER_INNER_POSE_ITERATIONS 30
#define DEFAULT_OPTIMIZER_INNER_ODOM_CALIB_ITERATIONS 5

#define DEFAULT_FAKE_GPS_CLUSTERING_DISTANCE 0.0
#define DEFAULT_GPS_SPARSITY_THRESHOLD 0.0

    using HyperBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > ;
    using HyperCholmodSolver = g2o::LinearSolverCholmod<HyperBlockSolver::PoseMatrixType>;

    class HyperGraphSclamOptimizer {

        private:

            // the odometry variances
            double odometry_xx_var, odometry_yy_var, odometry_hh_var;

            // the special odometry information
            double special_odometry_information;

            // the sick icp seq variances
            double sick_icp_xx_var, sick_icp_yy_var, sick_icp_hh_var;

            // the sick icp loop variances
            double sick_loop_xx_var, sick_loop_yy_var, sick_loop_hh_var;

            // velodyne icp variances
            double velodyne_icp_xx_var, velodyne_icp_yy_var, velodyne_icp_hh_var;

            // velodyne loop variances
            double velodyne_loop_xx_var, velodyne_loop_yy_var, velodyne_loop_hh_var;

            // velodyne external loop variances
            double velodyne_external_loop_icp_xx_var, velodyne_external_loop_icp_yy_var, velodyne_external_loop_icp_hh_var;

            // xsens default constraint variances
            double xsens_constraint_var;

            // visual odometry variances
            double visual_xx_var, visual_yy_var, visual_hh_var;

            // gps variances
            double gps_pose_std_multiplier, gps_pose_hh_std;

            // how many odometry calibration vertices
            unsigned odom_ackerman_params_vertices;

            // how many external iterations
            unsigned external_loop;

            // how many internal iterations
            unsigned internal_loop;

            // odom calibration iterations
            unsigned optimizer_inner_odom_calib_iterations;

            // default fake gps clustering
            double fake_gps_clustering_distance;

            // sparsity threshold u
            double gps_sparsity_threshold;

            // use the gps
            bool use_gps;

            // use velodyne icp
            bool use_velodyne_seq;

            // use  velodyne loop
            bool use_velodyne_loop;

            // use sick_seq
            bool use_sick_seq;

            // use sick_loop
            bool use_sick_loop;

            // use bumblebee_seq
            bool use_bumblebee_seq;

            // use bumblebee_loop
            bool use_bumblebee_loop;

            // use bumblebee_loop
            bool use_odometry;

            // the gps origin
            Eigen::Vector2d gps_origin;

            // input filename, containing the graph that comes out from the parser
            std::string input_filename;

            // output filename, the optimization poses are going to be saved in this file
            std::string output_filename;

            // the main sparse optimizer
            g2o::SparseOptimizer *optimizer;

            // the g2o factory
            g2o::Factory *factory;

            // the timestamp and ids vectors
            std::map<unsigned, std::pair<double, unsigned>> id_time_type_map;

            // hack
            std::list<g2o::EdgeGPS*> gps_buffer;

            std::map<std::string, std::vector<g2o::VertexSE2*> > logs;

            std::set<unsigned> els;

            // the ids offsets given the initial calibration vertices
            unsigned vertex_id_offset;

            // parse the input args
            void ArgsParser(int argc, char **argv);

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
            void AddVelodyneEdge(unsigned from, unsigned to, double x, double y, double theta, Eigen::Matrix3d &velodyne_icp_information);

            // read the velodyne edge from the input stream and save it to the optimizer
            void AddVelodyneEdge(std::stringstream &ss, Eigen::Matrix3d &velodyne_icp_information);

            // read the velodyne external loop edge and append it to the optimizer while saving the from/to indexes
            void AddExternalVelodyneEdge(std::stringstream &ss, Eigen::Matrix3d velodyne_external_loop_information);

            // add the odometry calibration edge to the optimizer
            void AddOdomCalibrationEdge(
                g2o::VertexSE2 *l_vertex,
                g2o::VertexSE2 *r_vertex,
                g2o::EdgeSE2 *odom_edge,
                unsigned odom_param_id,
                double raw_v,
                double raw_phi,
                double time,
                const Eigen::Matrix3d &special,
                const Eigen::Matrix3d &info);

            // read the odometry edge and the odometry calibration edge and save them to the optimizer
            void AddOdometryAndCalibEdges(
                std::stringstream &ss,
                unsigned odom_param_id,
                const Eigen::Matrix3d &special,
                const Eigen::Matrix3d &odom_info);

            // read the visual odometry edge
            void AddVisualOdometryEdge(std::stringstream &ss, Eigen::Matrix3d &visual_odom_information);

            // compute fake orientations
            void ComputeGPSFakeOrientations();

            // remove the undesired chunks
            void RemoveUndesiredDisplacements(double threshold);

            // remove gps messages
            void MakeGPSSparse();

            // gps edges filtering
            void GPSFiltering();

            // save the valid gps edges to the optimizer
            void AddAllValidGPS();

            // gps edges filtering
            void AddFilteredGPSEdges();

            // read the gps edge and save it to the optimizer
            void AddGPSEdge(std::stringstream &ss, Eigen::Matrix3d &cov);

            // add the raw gps edge
            void AddRawGPSEdge(std::stringstream &ss, Eigen::Matrix3d &cov);

            // read the xsens edge and save it to the optimizer
            void AddXSENSEdge(std::stringstream &ss, Eigen::Matrix<double, 1, 1> &information);

            // initialize the sparse optimizer
            void InitializeOptimizer();

            // manage the hypergraph region
            void ManageHypergraphRegion(std::vector<g2o::VertexSE2*> &group, bool status);

            // disable all vertices
            void PrepareIndividualOptimization();

            // reset the graph to the pose estimation
            void PreparePrevOptimization();

            // reset the graph to the odometry calibration estimation
            void PreparePostOptimization();

            // reset the graph to the next optimization round
            void PrepareRoundOptimization();

            // load the data into the optimizer
            void LoadHyperGraphToOptimizer();

            // save the external loops for ploting
            void SaveExternalLoops();

            // show the sensor pose otimization
            void ShowAllParametersVertices();

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
