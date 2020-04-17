#include <HyperGraphSclamOptimizer.hpp>

// custom edges
#include <EdgeVelodyneCalibration.hpp>
#include <EdgeBumblebeeCalibration.hpp>

#include <EdgeXSENS.hpp>
#include <EdgeSickCalibration.hpp>
#include <EdgeSE2OdomAckermanCalibration.hpp>

#include <StringHelper.hpp>
#include <StampedMessageType.hpp>

#include <algorithm>
#include <unistd.h>

using namespace hyper;

// the basic constructor
HyperGraphSclamOptimizer::HyperGraphSclamOptimizer(int argc, char **argv) :
    odometry_xx_var(DEFAULT_ODOMETRY_XX_VAR),
    odometry_yy_var(DEFAULT_ODOMETRY_YY_VAR),
    odometry_hh_var(DEFAULT_ODOMETRY_HH_VAR),
    special_odometry_information(DEFAULT_SPECIAL_ODOMETRY_INFORMATION),
    sick_icp_xx_var(DEFAULT_SICK_ICP_XX_VAR),
    sick_icp_yy_var(DEFAULT_SICK_ICP_YY_VAR),
    sick_icp_hh_var(DEFAULT_SICK_ICP_HH_VAR),
    sick_loop_xx_var(DEFAULT_SICK_LOOP_ICP_XX_VAR),
    sick_loop_yy_var(DEFAULT_SICK_LOOP_ICP_YY_VAR),
    sick_loop_hh_var(DEFAULT_SICK_LOOP_ICP_HH_VAR),
    velodyne_icp_xx_var(DEFAULT_VELODYNE_ICP_XX_VAR),
    velodyne_icp_yy_var(DEFAULT_VELODYNE_ICP_YY_VAR),
    velodyne_icp_hh_var(DEFAULT_VELODYNE_ICP_HH_VAR),
    velodyne_loop_xx_var(DEFAULT_VELODYNE_LOOP_ICP_XX_VAR),
    velodyne_loop_yy_var(DEFAULT_VELODYNE_LOOP_ICP_YY_VAR),
    velodyne_loop_hh_var(DEFAULT_VELODYNE_LOOP_ICP_HH_VAR),
    velodyne_external_loop_icp_xx_var(DEFAULT_VELODYNE_EXTERNAL_LOOP_ICP_XX_VAR),
    velodyne_external_loop_icp_yy_var(DEFAULT_VELODYNE_EXTERNAL_LOOP_ICP_YY_VAR),
    velodyne_external_loop_icp_hh_var(DEFAULT_VELODYNE_EXTERNAL_LOOP_ICP_HH_VAR),
    xsens_constraint_var(DEFAULT_XSENS_CONSTRAINT_VAR),
    visual_xx_var(DEFAULT_VISUAL_XX_VAR),
    visual_yy_var(DEFAULT_VISUAL_YY_VAR),
    visual_hh_var(DEFAULT_VISUAL_HH_VAR),
    gps_pose_std_multiplier(DEFAULT_GPS_POSE_STD_MULTIPLIER),
    gps_pose_hh_std(DEFAULT_GPS_POSE_HH_STD),
    odom_ackerman_params_vertices(DEFAULT_ODOM_ACKERMAN_PARAMS_VERTICES),
    external_loop(DEFAULT_OPTIMIZER_OUTER_ITERATIONS),
    internal_loop(DEFAULT_OPTIMIZER_INNER_POSE_ITERATIONS),
    optimizer_inner_odom_calib_iterations(DEFAULT_OPTIMIZER_INNER_ODOM_CALIB_ITERATIONS),
    fake_gps_clustering_distance(DEFAULT_FAKE_GPS_CLUSTERING_DISTANCE),
    gps_sparsity_threshold(DEFAULT_GPS_SPARSITY_THRESHOLD),
    use_gps(false),
    use_velodyne_seq(false),
    use_velodyne_loop(false),
    use_sick_seq(false),
    use_sick_loop(false),
    use_bumblebee_seq(false),
    use_bumblebee_loop(false),
    use_odometry(false),
    gps_origin(0.0, 0.0),
    optimizer(nullptr),
    factory(nullptr),
    id_time_type_map(),
    gps_buffer(0),
    logs(),
    els()
{
    // get the input filename
    ArgsParser(argc, argv);

    // update the vertex ids offset
    vertex_id_offset = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + odom_ackerman_params_vertices + 1;

    // registering the custom edges
    factory = g2o::Factory::instance();

    // registe all types
    RegisterCustomTypes();

    // load the new optimizer to memmory
    InitializeOptimizer();

    std::cout << "Loading the Hypergraph!\n";

    // load the data
    LoadHyperGraphToOptimizer();

    std::cout << "Hypergraph loaded!\n";
}


// basic destructor
HyperGraphSclamOptimizer::~HyperGraphSclamOptimizer()
{
    // call the clear method
    Clear();
}


// parse the input args
void HyperGraphSclamOptimizer::ArgsParser(int argc, char **argv)
{
    if (3 < argc)
    {
        // get the input filename
        input_filename = std::string(argv[1]);
        output_filename = std::string(argv[2]);

        std::string carmen_home(getenv("CARMEN_HOME"));
        std::string config_filename = 3 < argc ? std::string(argv[3]) : carmen_home + "/src/hypergraphsclam/config/optimization_config.txt";

        std::ifstream is(config_filename);
        if (is.good())
        {
            // helpers
            std::stringstream ss;

            while (-1 != StringHelper::ReadLine(is, ss))
            {
                std::string str;

                ss >> str;

                if ("ODOMETRY_XX_VAR" == str)
                {
                     ss >> odometry_xx_var;
                }
                else if ("ODOMETRY_YY_VAR" == str)
                {
                    ss >> odometry_yy_var;
                }
                else if ("ODOMETRY_HH_VAR" == str)
                {
                    ss >> odometry_hh_var;
                }
                else if ("SPECIAL_ODOMETRY_INFORMATION" == str)
                {
                    ss >> special_odometry_information;
                }
                else if ("SICK_ICP_XX_VAR" == str)
                {
                    ss >> sick_icp_xx_var;
                }
                else if ("SICK_ICP_YY_VAR" == str)
                {
                    ss >> sick_icp_yy_var;
                }
                else if ("SICK_ICP_HH_VAR" == str)
                {
                    ss >> sick_icp_hh_var;
                }
                else if ("SICK_LOOP_ICP_XX_VAR" == str)
                {
                    ss >> sick_loop_xx_var;
                }
                else if ("SICK_LOOP_ICP_YY_VAR" == str)
                {
                    ss >> sick_loop_yy_var;
                }
                else if ("SICK_LOOP_ICP_HH_VAR" == str)
                {
                    ss >> sick_loop_hh_var;
                }
                else if ("VELODYNE_ICP_XX_VAR" == str)
                {
                    ss >> velodyne_icp_xx_var;
                }
                else if ("VELODYNE_ICP_YY_VAR" == str)
                {
                    ss >> velodyne_icp_yy_var;
                }
                else if ("VELODYNE_ICP_HH_VAR" == str)
                {
                    ss >> velodyne_icp_hh_var;
                }
                else if ("VELODYNE_LOOP_ICP_XX_VAR" == str)
                {
                    ss >> velodyne_loop_xx_var;
                }
                else if ("VELODYNE_LOOP_ICP_YY_VAR" == str)
                {
                    ss >> velodyne_loop_yy_var;
                }
                else if ("VELODYNE_LOOP_ICP_HH_VAR" == str)
                {
                    ss >> velodyne_loop_hh_var;
                }
                else if ("VELODYNE_EXTERNAL_LOOP_ICP_XX_VAR" == str)
                {
                    ss >> velodyne_external_loop_icp_xx_var;
                }
                else if ("VELODYNE_EXTERNAL_LOOP_ICP_YY_VAR" == str)
                {
                    ss >> velodyne_external_loop_icp_yy_var;
                }
                else if ("VELODYNE_EXTERNAL_LOOP_ICP_HH_VAR" == str)
                {
                    ss >> velodyne_external_loop_icp_hh_var;
                }
                else if ("XSENS_CONSTRAINT_VAR" == str)
                {
                    ss >> xsens_constraint_var;
                }
                else if ("VISUAL_XX_VAR" == str)
                {
                    ss >> visual_xx_var;
                }
                else if ("VISUAL_YY_VAR" == str)
                {
                    ss >> visual_yy_var;
                }
                else if ("VISUAL_HH_VAR" == str)
                {
                    ss >> visual_hh_var;
                }
                else if ("GPS_POSE_STD_MULTIPLIER" == str)
                {
                    ss >> gps_pose_std_multiplier;
                }
                else if ("GPS_POSE_HH_STD" == str)
                {
                    ss >> gps_pose_hh_std;
                }
                else if ("ODOM_ACKERMAN_PARAMS_VERTICES" == str)
                {
                    ss >> odom_ackerman_params_vertices;
                }
                else if ("OPTIMIZER_OUTER_ITERATIONS" == str)
                {
                    ss >> external_loop;
                }
                else if ("OPTIMIZER_INNER_POSE_ITERATIONS" == str)
                {
                    ss >> internal_loop;
                }
                else if ("OPTIMIZER_INNER_ODOM_CALIB_ITERATIONS" == str)
                {
                    ss >> optimizer_inner_odom_calib_iterations;
                }
                else if ("FAKE_GPS_CLUSTERING_DISTANCE" == str)
                {
                    ss >> fake_gps_clustering_distance;
                }
                else if ("GPS_SPARSITY_THRESHOLD" == str)
                {
                    ss >> gps_sparsity_threshold;
                }
                else if ("USE_GPS" == str)
                {
                    use_gps = true;
                }
                else if ("USE_VELODYNE_SEQ" == str)
                {
                    use_velodyne_seq = true;
                }
                else if ("USE_VELODYNE_LOOP" == str)
                {
                    use_velodyne_loop = true;
                }
                else if ("USE_SICK_SEQ" == str)
                {
                    use_sick_seq = true;
                }
                else if ("USE_SICK_LOOP" == str)
                {
                    use_sick_loop = true;
                }
                else if ("USE_BUMBLEBEE_SEQ" == str)
                {
                    use_bumblebee_seq = true;
                }
                else if ("USE_BUMBLEBEE_LOOP" == str)
                {
                    use_bumblebee_loop = true;
                }
                else if ("USE_ODOMETRY" == str)
                {
                    use_odometry = true;
                }
            }

            is.close();
        }
        else
        {
            std::cout << "Could not open the config file, using default parameters!" << std::endl;
        }
    }
}


// return a 3x3 information matrix given the diagonal covariances
Eigen::Matrix3d HyperGraphSclamOptimizer::GetInformationMatrix(double xx_var, double yy_var, double hh_var)
{
    // create the covariance matrix
    Eigen::Matrix3d cov(Eigen::Matrix3d::Zero());

    // set the identity values
    cov.data()[0] = xx_var * xx_var;
    cov.data()[4] = yy_var * yy_var;
    cov.data()[8] = hh_var * hh_var;

    // get the information
    return cov.inverse();
}


// registering the custom edges and vertices
void HyperGraphSclamOptimizer::RegisterCustomTypes()
{
    // register the custom gps edge
    factory->registerType("EDGE_GPS", new g2o::HyperGraphElementCreator<g2o::EdgeGPS>);

    // register the custom sick edge
    factory->registerType("EDGE_SICK_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeSickCalibration>);

    // register the custom velodyne edge
    factory->registerType("EDGE_VELODYNE_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeVelodyneCalibration>);

    // register the custom bumblebee edge
    factory->registerType("EDGE_BUMBLEBEE_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeBumblebeeCalibration>);

    // register the custom odometry calibration edge
    factory->registerType("EDGE_SE2_ODOM_ACKERMAN_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeSE2OdomAckermanCalibration>);

    // register the custom vertex
    factory->registerType("VERTEX_ODOM_ACKERMAN_PARAM_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::VertexOdomAckermanParams>);
}


// initialize the sparse optimizer
void HyperGraphSclamOptimizer::InitializeOptimizer()
{
    // creates a new sparse optimizer in memmory
    optimizer = new g2o::SparseOptimizer();

    // allocate a new cholmod solver
    std::unique_ptr<HyperCholmodSolver> cholmod_solver(new HyperCholmodSolver());

    // the block ordering
    cholmod_solver->setBlockOrdering(false);

    // the base solver

    std::unique_ptr<g2o::Solver> solver(new HyperBlockSolver(std::move(cholmod_solver)));

    // the base solver

    g2o::OptimizationAlgorithm *optimization_algorithm = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver));

    // set the cholmod solver
    optimizer->setAlgorithm(optimization_algorithm);

    // set the verbose mode
    optimizer->setVerbose(true);
}


// add the sick, velodyne and odometry calibration vertices
void HyperGraphSclamOptimizer::AddParametersVertices()
{
    // creates the sick offset vertex with a given initial estimate
    g2o::VertexSE2 *sick_offset = new g2o::VertexSE2;

    // the initial bullbar measure
    sick_offset->setEstimate(g2o::SE2(3.52, 0.0, 0.0));

    // marginalize it
    sick_offset->setMarginalized(true);

    // set fixed
    sick_offset->setFixed(true);

    // set the sick offset id 0
    sick_offset->setId(SICK_VERTEX_OFFSET_ID);

    if (!optimizer->addVertex(sick_offset))
    {
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

    // marginalize it
    velodyne_offset->setMarginalized(true);

    // set fixed
    velodyne_offset->setFixed(true);

    // save the offset vertex
    if (!optimizer->addVertex(velodyne_offset))
    {
        throw std::runtime_error("Could not add the velodyne offset vertex to the optimizer!");
    }

    // creates the bumblebee offset vertex with a given initial estimate
    g2o::VertexSE2 *bumblebee_offset = new g2o::VertexSE2;

    // set the bumblebee initial estimate
    // camera3_x       0.245
    // camera3_y       0.350
    // camera3_z       0.210
    // camera3_roll    0.0
    // camera3_pitch   0.04
    // camera3_yaw     0.0
    bumblebee_offset->setEstimate(g2o::SE2(0.572 + 0.245, 0.0, 0.0));

    // set the bumblebee offset id
    bumblebee_offset->setId(BUMBLEBEE_VERTEX_OFFSET_ID);

    // marginalize it
    bumblebee_offset->setMarginalized(true);

    // set fixed
    bumblebee_offset->setFixed(true);

    // save the offset vertex
    if (!optimizer->addVertex(bumblebee_offset))
    {
        throw std::runtime_error("Could not add the bumblebee offset vertex to the optimizer!");
    }

    // include all ackerman params
    for (unsigned i = 0; i < odom_ackerman_params_vertices; ++i)
    {
        // get the current param
        g2o::VertexOdomAckermanParams *odom_param = new g2o::VertexOdomAckermanParams();

        // set the id
        odom_param->setId(ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + i);

        // marginalized it
        odom_param->setMarginalized(true);

        // set fixed?
        odom_param->setFixed(true);

        // set the initial estimate
        odom_param->setToOriginImpl();

        // try to save the current vertex to the optimizer
        if (!optimizer->addVertex(odom_param))
        {
            throw std::runtime_error("Could not add the odom ackerman params calibration vertex to the optimizer!");
        }
    }
}


// read the current vertex from the input stream and save it to the optimizer
void HyperGraphSclamOptimizer::AddVertex(std::stringstream &ss)
{
    // helpers
    unsigned vid, vertex_id, msg_type;
    double x, y, theta, timestamp;
    std::string gid;

    // read the current line
    ss >> gid >> vid >> x >> y >> theta >> timestamp >> msg_type;

    vertex_id = vid + vertex_id_offset;

    // creates the new vertex
    g2o::VertexSE2 *v = new g2o::VertexSE2();

    // set the id
    v->setId(vertex_id);

    // set the initial estimate
    v->setEstimate(g2o::SE2(x, y, theta));

    // disable the vertex
    // v->setFixed(true);

    // marginalize it
    // v->setMarginalized(true);

    // add to the optimizer
    if(!optimizer->addVertex(v))
    {
        throw std::runtime_error("Could not add a new vertex to the optimizer!");
    }

    // save the timestamp and type
    id_time_type_map[vertex_id] = std::pair<double, unsigned>(timestamp, msg_type);
    if (logs.find(gid) == logs.end())
    {
        logs[gid] = std::vector<g2o::VertexSE2*>();
    }
    logs[gid].push_back(v);
}


// read the sick edge from the input stream and save it to the optimizer
void HyperGraphSclamOptimizer::AddSickEdge(std::stringstream &ss, Eigen::Matrix3d &sick_icp_information)
{
    // helpers
    unsigned from, to;
    double x, y, theta;

    // read the ids and icp measure
    ss >> from >> to >> x >> y >> theta;

    // create the new calibration edge
    g2o::EdgeSickCalibration *sick_seq_edge = new g2o::EdgeSickCalibration;

    // set the measurement
    sick_seq_edge->setMeasurement(g2o::SE2(x, y, theta));

    // set the position estimates vertices
    sick_seq_edge->vertices()[0] = optimizer->vertex(from + vertex_id_offset);
    sick_seq_edge->vertices()[1] = optimizer->vertex(to + vertex_id_offset);

    // set the sick offset vertex
    sick_seq_edge->vertices()[2] = optimizer->vertex(SICK_VERTEX_OFFSET_ID);

    // set the sick icp information matrix
    sick_seq_edge->setInformation(sick_icp_information);

    // try to append the the sick seq edge
    if (!optimizer->addEdge(sick_seq_edge))
    {
        throw std::runtime_error("Could not add the sick seq icp edge to the optimizer!");
    }
}

void HyperGraphSclamOptimizer::AddVelodyneEdge(unsigned from, unsigned to, double x, double y, double theta, Eigen::Matrix3d &velodyne_icp_information)
{
    // create the new calibration edge
    g2o::EdgeVelodyneCalibration *velodyne_seq_edge = new g2o::EdgeVelodyneCalibration();

    // set the measurement
    velodyne_seq_edge->setMeasurement(g2o::SE2(x, y, theta));

    // set the position estimates vertices
    velodyne_seq_edge->vertices()[0] = optimizer->vertex(from + vertex_id_offset);
    velodyne_seq_edge->vertices()[1] = optimizer->vertex(to + vertex_id_offset);

    // set the velodyne offset vertex
    velodyne_seq_edge->vertices()[2] = optimizer->vertex(VELODYNE_VERTEX_OFFSET_ID);

    // set the velodyne icp information matrix
    velodyne_seq_edge->setInformation(velodyne_icp_information);

    // try to append the the velodyne seq edge
    if (!optimizer->addEdge(velodyne_seq_edge))
    {
        throw std::runtime_error("Could not add the velodyne seq icp edge to the optimizer!");
    }
}


// read the velodyne edge from the input stream and save it to the optimizer
void HyperGraphSclamOptimizer::AddVelodyneEdge(std::stringstream &ss, Eigen::Matrix3d &velodyne_icp_information)
{
    // helpers
    unsigned from, to;
    double x, y, theta;

    // read the ids and icp measure
    ss >> from >> to >> x >> y >> theta;

    AddVelodyneEdge(from, to, x, y, theta, velodyne_icp_information);
}

void HyperGraphSclamOptimizer::AddExternalVelodyneEdge(std::stringstream &ss, Eigen::Matrix3d velodyne_external_loop_information)
{
    // helpers
    unsigned from, to;
    double x, y, theta;

    // read the ids and icp measure
    ss >> from >> to >> x >> y >> theta;

    AddVelodyneEdge(from, to, x, y, theta, velodyne_external_loop_information);

    els.insert(from + vertex_id_offset);
    els.insert(to + vertex_id_offset);
}

// add the odometry calibration edge to the optimizer
void HyperGraphSclamOptimizer::AddOdomCalibrationEdge(
    g2o::VertexSE2 *l_vertex,
    g2o::VertexSE2 *r_vertex,
    g2o::EdgeSE2 *odom_edge,
    unsigned odom_param_id,
    double vel,
    double phi,
    double time,
    const Eigen::Matrix3d &special,
    const Eigen::Matrix3d &info)
{
    // create the new edges
    g2o::EdgeSE2OdomAckermanCalibration *odom_calib_edge = new g2o::EdgeSE2OdomAckermanCalibration();

    // get the odometry ackerman param vertex
    g2o::VertexOdomAckermanParams *params = static_cast<g2o::VertexOdomAckermanParams*>(optimizer->vertex(odom_param_id));

    // set the vertices
    odom_calib_edge->setVertices(l_vertex, r_vertex, params);

    // set the odometry measure
    odom_calib_edge->setOdometryEdge(odom_edge);

    // set the raw values
    odom_calib_edge->setRawValues(vel, phi, time);

    // set the info matrix
    if (0.0 == time)
    {
        // set the special info
        odom_calib_edge->setInformation(special);
    }
    else
    {
        // set the general info
        odom_calib_edge->setInformation(info);
    }

    if (!optimizer->addEdge(odom_calib_edge))
    {
        throw std::runtime_error("Could not add the odometry calibration edge to the optimizer!");
    }
}


// read the odometry edge and the odometry calibration edge and save them to the optimizer
void HyperGraphSclamOptimizer::AddOdometryAndCalibEdges(
    std::stringstream &ss, unsigned odom_param_id,
    const Eigen::Matrix3d &special,
    const Eigen::Matrix3d &odom_info)
{
    // helpers
    unsigned from, to;
    double x, y, theta, _v, _p, _t;

    ss >> from >> to >> x >> y >> theta >> _v >> _p >> _t;

    // get both vertices
    g2o::VertexSE2 *l_vertex = static_cast<g2o::VertexSE2*>(optimizer->vertex(from + vertex_id_offset));
    g2o::VertexSE2 *r_vertex = static_cast<g2o::VertexSE2*>(optimizer->vertex(to + vertex_id_offset));

    // creates a new edge
    g2o::EdgeSE2 *edge = new g2o::EdgeSE2();

    // set the vertices
    edge->vertices()[0] = l_vertex;
    edge->vertices()[1] = r_vertex;

    // set the base measurement
    edge->setMeasurement(g2o::SE2(x, y, theta));

    if (0.0 == x && 0.0 == y && 0.0 == theta)
    {
        // set the special info
        edge->setInformation(special);
    }
    else
    {
        // set the info
        edge->setInformation(odom_info);
    }

    if (!optimizer->addEdge(edge))
    {
        throw std::runtime_error("Could not add the odometry edge to the optimizer!");
    }

    // add a new odom calibration edge
    AddOdomCalibrationEdge(l_vertex, r_vertex, edge, odom_param_id, _v, _p, _t, special, odom_info);
}

// read the visual odometry edge
void HyperGraphSclamOptimizer::AddVisualOdometryEdge(std::stringstream &ss, Eigen::Matrix3d &visual_odom_information)
{
    // helpers
    unsigned from, to;
    double x, y, theta;

    // read the ids and icp measure
    ss >> from >> to >> x >> y >> theta;

    // create the new calibration edge
    g2o::EdgeBumblebeeCalibration *bumblebee_seq_edge = new g2o::EdgeBumblebeeCalibration();

    // set the measurement
    bumblebee_seq_edge->setMeasurement(g2o::SE2(x, y, theta));

    // set the position estimates vertices
    bumblebee_seq_edge->vertices()[0] = optimizer->vertex(from + vertex_id_offset);
    bumblebee_seq_edge->vertices()[1] = optimizer->vertex(to + vertex_id_offset);

    // set the bumblebee offset vertex
    bumblebee_seq_edge->vertices()[2] = optimizer->vertex(BUMBLEBEE_VERTEX_OFFSET_ID);

    // set the bumblebee icp information matrix
    bumblebee_seq_edge->setInformation(visual_odom_information);

    // try to append the the bumblebee seq edge
    if (!optimizer->addEdge(bumblebee_seq_edge))
    {
        throw std::runtime_error("Could not add the bumblebee seq edge to the optimizer!");
    }
}


// compute fake orientations
void HyperGraphSclamOptimizer::ComputeGPSFakeOrientations()
{

    if (3 < gps_buffer.size())
    {
        // iterators
        std::list<g2o::EdgeGPS*>::iterator end(gps_buffer.end());
        std::list<g2o::EdgeGPS*>::iterator prev(gps_buffer.begin());
        std::list<g2o::EdgeGPS*>::iterator curr(prev);
        std::list<g2o::EdgeGPS*>::iterator next(prev);

        std::advance(curr, 1);
        std::advance(next, 2);

        // process the first one
        g2o::EdgeGPS *prev_gps = *prev;
        g2o::EdgeGPS *curr_gps = *curr;
        g2o::EdgeGPS *next_gps = *next;

        // Compute the first fake orientation
        prev_gps->computeFakeOrientation(curr_gps->measurement());

        // main loop
        while (end != next)
        {
            // process the current one
            prev_gps = *prev;
            curr_gps = *curr;
            next_gps = *next;

            // compute fake orientation
            curr_gps->computeFakeOrientation(prev_gps->measurement(), next_gps->measurement());

            // go to the next messages
            prev = curr;
            curr = next;
            ++next;
        }

        // compute the last fake orientation
        prev_gps->computeFakeOrientation(curr_gps->measurement());
    }
    else
    {
        throw std::runtime_error("Invalid GPS edges! Make sure you'are loading the gps correctly!");
    }
}


// remove the undesired chunks
void HyperGraphSclamOptimizer::RemoveUndesiredDisplacements(double threshold)
{
    if (5 < gps_buffer.size())
    {
        // iterators
        std::list<g2o::EdgeGPS*>::iterator prev(gps_buffer.begin());
        std::list<g2o::EdgeGPS*>::iterator curr(prev);
        std::list<g2o::EdgeGPS*>::iterator next(prev);

        std::advance(next, 1);

        // let's see
        (*prev)->validate(*curr, *next);

        Eigen::Vector2d last_t((*curr)->measurement().translation());

        unsigned counter = 0;

        while (gps_buffer.end() != next)
        {
            Eigen::Vector2d next_t((*next)->measurement().translation());

            double diff = (next_t - last_t).norm();

            if (threshold < diff)
            {
                if (40 > counter)
                {
                    curr = prev;
                    std::advance(curr, 1);

                    while (prev != next)
                    {
                        g2o::EdgeGPS* gps(*prev);

                        gps_buffer.erase(prev);

                        delete gps;

                        prev = curr;
                        ++curr;
                    }
                }

                curr = prev = next;
                counter = 0;
            }
            else
            {
                ++curr;
                counter += 1;
            }

            last_t = next_t;

            ++next;
        }
    }
}


// remove gps messages
void HyperGraphSclamOptimizer::MakeGPSSparse()
{
    if (5 < gps_buffer.size())
    {
        // iterators
        std::list<g2o::EdgeGPS*>::iterator curr(gps_buffer.begin());
        std::list<g2o::EdgeGPS*>::iterator next(curr);

        std::advance(next, 1);

        Eigen::Vector2d last_t((*curr)->measurement().translation());

        unsigned counter = 0;

        while (gps_buffer.end() != next)
        {
            Eigen::Vector2d next_t((*next)->measurement().translation());

            double diff = (next_t - last_t).norm();

            if (gps_sparsity_threshold > diff)
            {
                g2o::EdgeGPS* gps(*next);
                gps_buffer.erase(next);
                delete gps;

                next = curr;
            }
            else
            {
                ++curr;
                counter += 1;
                last_t = next_t;
            }

            ++next;
        }
    }
}


// gps edges filtering
void HyperGraphSclamOptimizer::GPSFiltering()
{
    // no displacement
    if (0.01 < fake_gps_clustering_distance)
    {
        RemoveUndesiredDisplacements(fake_gps_clustering_distance);
    }

    // sparse!
    if (0.01 < gps_sparsity_threshold)
    {
        MakeGPSSparse();
    }

    // no displacement
    if (0.01 < fake_gps_clustering_distance)
    {
        RemoveUndesiredDisplacements(fake_gps_clustering_distance);
    }
}


// save the valid gps edges to the optimizer
void HyperGraphSclamOptimizer::AddAllValidGPS()
{
    // open the file
    std::ofstream os("fake_gps.txt", std::ofstream::out);

    if (!os.good())
    {

        throw std::runtime_error("Could not open the fake_gps.txt!");
    }

    while (!gps_buffer.empty())
    {
        // get the first one
        g2o::EdgeGPS *gps_edge = gps_buffer.front();

        // remove it from the buffer
        gps_buffer.pop_front();

        // save it
        if (!optimizer->addEdge(gps_edge))
        {

            throw std::runtime_error("Could not add the gps edge to the optimizer!");
        }

        // save it to the output
        Eigen::Vector3d gps_pose(gps_edge->fakeMeasurement().toVector());

        double x = gps_pose[0] + gps_origin[0];
        double y = gps_pose[1] + gps_origin[1];
        double sina = std::sin(gps_pose[2]) * 0.5;
        double cosa = std::cos(gps_pose[2]) * 0.5;

        os << std::fixed << x << " " << y << " " << gps_pose[2] << " " << cosa << " " << sina << "\n";
    }

    // close the fake gps file
    os.close();
}


// gps edges filtering
void HyperGraphSclamOptimizer::AddFilteredGPSEdges()
{

    std::cout << "Computing the gps filtering!" << std::endl;

    // gps filtering
    GPSFiltering();

    std::cout << "Computing the filtered gps fake orientation!" << std::endl;

    // compute all fake orientations
    ComputeGPSFakeOrientations();

    std::cout << "Adding all valid edges to the optimizer!" << std::endl;

    // add all valid gps to the optimizer
    AddAllValidGPS();
}


// add the raw gps edge
void HyperGraphSclamOptimizer::AddRawGPSEdge(std::stringstream &ss, Eigen::Matrix3d &cov)
{
    // helpers
    unsigned from;
    double x, y, theta, gps_std;

    // creates a new gps edge
    g2o::EdgeGPS *gps_edge = new g2o::EdgeGPS;

    // read the vertex id, gps measure and std deviation
    ss >> from >> x >> y >> theta >> gps_std;

    // set the vertices
    gps_edge->vertices()[0] = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(from + vertex_id_offset));

    // update the internal measurement
    gps_edge->setMeasurement(g2o::SE2(x, y, theta));

    // update the diagonals
    cov.data()[0] = cov.data()[4] = std::pow(gps_std * gps_pose_std_multiplier, 2);

    // set the info matrix
    gps_edge->setInformation(Eigen::Matrix3d(cov.inverse()));

    // save it
    if (!optimizer->addEdge(gps_edge))
    {
        throw std::runtime_error("Could not add the gps edge to the optimizer!");
    }
}


// read the gps edge and save it to the optimizer
void HyperGraphSclamOptimizer::AddGPSEdge(std::stringstream &ss, Eigen::Matrix3d &cov)
{
    // helpers
    unsigned from;
    double x, y, theta, gps_std;

    // creates a new gps edge
    g2o::EdgeGPS *gps_edge = new g2o::EdgeGPS;

    // read the vertex id, gps measure and std deviation
    ss >> from >> x >> y >> theta >> gps_std;

    // set the vertices
    gps_edge->vertices()[0] = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(from + vertex_id_offset));

    // update the internal measurement
    gps_edge->setMeasurement(g2o::SE2(x, y, theta));

    // update the diagonals
    cov.data()[0] = cov.data()[4] = std::pow(gps_std * gps_pose_std_multiplier, 2);

    // set the info matrix
    gps_edge->setInformation(Eigen::Matrix3d(cov.inverse()));

    // save it to the gps buffer
    gps_buffer.push_back(gps_edge);
}


// read the xsens edge and save it to the optimizer
void HyperGraphSclamOptimizer::AddXSENSEdge(std::stringstream &ss, Eigen::Matrix<double, 1, 1> &information)
{
    // helpers
    unsigned from;
    double yaw;

    // creates the new xsens edge
    g2o::EdgeXSENS *xsens_edge = new g2o::EdgeXSENS;

    // read the vertex id and the yaw measure
    ss >> from >> yaw;

    // set the vertices
    xsens_edge->vertices()[0] = optimizer->vertex(from + vertex_id_offset);

    // set the measurement
    xsens_edge->setMeasurement(yaw);

    // set the information matrix
    xsens_edge->setInformation(information);

    // try to save it
    if (!optimizer->addEdge(xsens_edge))
    {
        throw std::runtime_error("Could not add the xsens edge to the optimizer!");
    }
}


// manage the hypergraph region
void HyperGraphSclamOptimizer::ManageHypergraphRegion(std::vector<g2o::VertexSE2*> &group, bool status)
{
    for (g2o::VertexSE2 *v : group)
    {
        v->setMarginalized(status);
        v->setFixed(status);
    }

    if (!status)
    {
        optimizer->initializeOptimization();
        optimizer->computeActiveErrors();
        std::cout << "Next Log..." << std::endl;
    }
}

void HyperGraphSclamOptimizer::PrepareIndividualOptimization()
{
    for (g2o::SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it)
    {
        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
        v->setMarginalized(true);
        v->setFixed(true);
    }
}


// reset the graph to the pose estimation
void HyperGraphSclamOptimizer::PreparePrevOptimization()
{
    for (g2o::SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it)
    {

        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);

        // define the status
        bool status = nullptr != dynamic_cast<g2o::VertexOdomAckermanParams*>(v);

        // true => this node should be marginalized out during the optimization
        v->setMarginalized(status);

        // enable it
        v->setFixed(status);
    }

    // Initializes the structures for optimizing the whole graph.
    optimizer->initializeOptimization();

    for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it)
    {

        g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);

        // specify the robust kernel to be used in this edge
        e->setRobustKernel(nullptr);
    }

    // pre-compute the active errors
    optimizer->computeActiveErrors();
}


// reset the graph to the odometry calibration estimation
void HyperGraphSclamOptimizer::PreparePostOptimization()
{
    // preprare
    for (g2o::SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it)
    {
        // downcasting
        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);

        // define the status
        bool status = nullptr == dynamic_cast<g2o::VertexOdomAckermanParams*>(v);

        // true => this node should be marginalized out during the optimization
        v->setMarginalized(status);

        // enable it
        v->setFixed(status);
    }

    // Initializes the structures for optimizing the whole graph.
    optimizer->initializeOptimization();

    for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it)
    {

        g2o::EdgeSE2OdomAckermanCalibration *odom_calib_edge = dynamic_cast<g2o::EdgeSE2OdomAckermanCalibration*>(*it);

        if (nullptr != odom_calib_edge)
        {
            // get the measurements from the current vertices
            odom_calib_edge->getMeasurementFromVertices();
        }
    }

    // pre-compute the active errors
    optimizer->computeActiveErrors();
}


// reset the graph to the next optimization round
void HyperGraphSclamOptimizer::PrepareRoundOptimization()
{
    // update the odometry measure
    for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it)
    {
        g2o::EdgeSE2OdomAckermanCalibration *odom_calib_edge = dynamic_cast<g2o::EdgeSE2OdomAckermanCalibration*>(*it);
        if (nullptr != odom_calib_edge)
        {
            // get the measurements from the current vertices
            odom_calib_edge->updateOdometryMeasure();
        }
    }
}


// load the data into the optimizer
void HyperGraphSclamOptimizer::LoadHyperGraphToOptimizer()
{
    // try to open the input filename
    std::ifstream is(input_filename);

    if (!is.is_open())
    {
        throw std::runtime_error("Could not read the input file!");
    }

    // helpers
    std::stringstream ss;
    unsigned subdivision = std::numeric_limits<unsigned>::max();
    unsigned odom_counter = 0;
    unsigned odom_param_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID;
    unsigned last_odom_param_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + odom_ackerman_params_vertices - 1;

    // get the inverse of the covariance matrix,
    Eigen::Matrix3d odom_information(GetInformationMatrix(odometry_xx_var, odometry_yy_var, odometry_hh_var));
    Eigen::Matrix3d special_odom_information(Eigen::Matrix3d::Identity() * special_odometry_information);
    Eigen::Matrix3d sick_icp_information(GetInformationMatrix(sick_icp_xx_var, sick_icp_yy_var, sick_icp_hh_var));
    Eigen::Matrix3d sick_loop_information(GetInformationMatrix(sick_loop_xx_var, sick_loop_yy_var, sick_loop_hh_var));
    Eigen::Matrix3d velodyne_icp_information(GetInformationMatrix(velodyne_icp_xx_var, velodyne_icp_yy_var, velodyne_icp_hh_var));
    Eigen::Matrix3d velodyne_loop_information(GetInformationMatrix(velodyne_loop_xx_var, velodyne_loop_yy_var, velodyne_loop_hh_var));
    Eigen::Matrix3d velodyne_external_loop_information(GetInformationMatrix(velodyne_external_loop_icp_xx_var, velodyne_external_loop_icp_yy_var, velodyne_external_loop_icp_hh_var));
    Eigen::Matrix3d visual_odom_information(GetInformationMatrix(visual_xx_var, visual_yy_var, visual_hh_var));
    Eigen::Matrix<double, 1, 1> xsens_information(Eigen::Matrix<double, 1, 1>::Identity() * 1.0 /  std::pow(xsens_constraint_var, 2));

    // set the odometry covariance matrix
    Eigen::Matrix3d cov(Eigen::Matrix3d::Identity());
    cov.data()[8] = std::pow(gps_pose_hh_std, 2);

    // add the parameters vertexes
    AddParametersVertices();

    // the main loop
    while(-1 != hyper::StringHelper::ReadLine(is, ss))
    {
        // the tag
        std::string tag;

        // read the tag
        ss >> tag;

        // the VERTEX tags should become first in the sync file
        if ("VERTEX" == tag)
        {
            // push the new vertex to the optimizer
            AddVertex(ss);
        }
        else if ("ODOM_EDGE" == tag && use_odometry)
        {
            if (0 != odom_counter)
            {
                AddOdometryAndCalibEdges(ss, odom_param_id, special_odom_information, odom_information);
            }
            else
            {
                AddOdometryAndCalibEdges(ss, odom_param_id, special_odom_information, special_odom_information);
            }

            // increment the odometry counter
            ++odom_counter;

            if (0 == odom_counter % subdivision && last_odom_param_id > odom_param_id)
            {
                // increment the odometry param index
                ++odom_param_id;
            }
        }
        else if ("GPS_EDGE" == tag && use_gps)
        {
            // add the gps edge to the optimizer
            AddGPSEdge(ss, cov);
        }
        else if ("XSENS_EDGE_" == tag)
        {
            // add the xsens edge to the optimizer
            AddXSENSEdge(ss, xsens_information);
        }
        else if ("SICK_SEQ" == tag && use_sick_seq)
        {
            // push the sick edge to the optimizer
            AddSickEdge(ss, sick_icp_information);
        }
        else if ("SICK_LOOP" == tag && use_sick_loop)
        {
            // push the sick edge to the optimizer
            AddSickEdge(ss, sick_loop_information);
        }
        else if ("VELODYNE_SEQ" == tag && use_velodyne_seq)
        {
            // push the velodyne icp edge to the optimizer
            AddVelodyneEdge(ss, velodyne_icp_information);
        }
        if (use_velodyne_loop && "VELODYNE_LOOP" == tag)
        {
            // push the velodyne icp edge to the optimizer
            AddVelodyneEdge(ss, velodyne_loop_information);
        }
        else if (use_velodyne_loop && "VELODYNE_EXTERNAL_LOOP" == tag)
        {
            AddExternalVelodyneEdge(ss, velodyne_external_loop_information);
        }
        else if ("BUMBLEBEE_SEQ" == tag && use_bumblebee_seq)
        {
            // push the visual odometry edge to the optimizer
            AddVisualOdometryEdge(ss, visual_odom_information);
        }
        else if ("BUMBLEBE_LOOP" == tag && use_bumblebee_loop)
        {
            // push the visual odometry edge to the optimizer
            AddVisualOdometryEdge(ss, visual_odom_information);
        }
        else if ("GPS_ORIGIN" == tag)
        {
            // read the origin
            ss >> gps_origin[0] >> gps_origin[1];

            std::cout << "GPS origin: " << std::fixed << gps_origin.transpose() << std::endl;
        }
        else if ("VERTICES_QUANTITY" == tag)
        {
            // read the qnt
            ss >> subdivision;

            // divide it by the number of odometry param vertices
            subdivision /= odom_ackerman_params_vertices;
        }
    }

    if (use_gps) { AddFilteredGPSEdges(); }

    // close the input file stream
    is.close();
}

void HyperGraphSclamOptimizer::SaveExternalLoops()
{
    std::ofstream external_loops(output_filename + "_external_loops.txt");
    if (not external_loops.is_open())
    {
        throw std::runtime_error("Could not open the external loops file!");
    }

    for (unsigned s : els)
    {
        auto _v { optimizer->vertex(s)};

        if (nullptr == _v)
        {
            std::cout << "Get a nullptr!" << std::endl;
            continue;
        }

        g2o::VertexSE2 *v = dynamic_cast<g2o::VertexSE2*>(_v);


        if (nullptr != v) 
        {
            const g2o::SE2 &pose(v->estimate());
            Eigen::Vector2d p(pose.translation() + gps_origin);

            double cosa = std::cos(pose[2]);
            double sina = std::sin(pose[2]);

            std::pair<double, unsigned> time_type { id_time_type_map.at(unsigned(v->id())) };

            double t = time_type.first;
            
            external_loops << std::fixed << p[0] << " " << p[1] << " " << pose[2] << " " << cosa << " " << sina << " " << t << std::endl;
        }
    }

    external_loops.close();
}

void HyperGraphSclamOptimizer::ShowAllParametersVertices()
{
    // the first vertex is the sick displacement
    // downcast to the base vertex
    g2o::VertexSE2* sick_offset = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(SICK_VERTEX_OFFSET_ID));

    // show the resulting offset
    std::cout << std::endl << "The SICK offset: " << std::fixed << sick_offset->estimate().toVector().transpose() << std::endl;

    // the second vertex is the velodyne displacement
    // downcast to the base vertex
    g2o::VertexSE2* velodyne_offset = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(VELODYNE_VERTEX_OFFSET_ID));

    // the velodyne offset
    g2o::SE2 velodyne_offset_pose = velodyne_offset->estimate();

    // show the resulting offset
    std::cout << std::endl << "The Velodyne offset: " << std::fixed << std::setprecision(10) << velodyne_offset_pose.toVector().transpose() << std::endl;

    // the third vertex is the bumblebee displacement
    // downcast to the base vertex
    g2o::VertexSE2* bumblebee_offset = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(BUMBLEBEE_VERTEX_OFFSET_ID));

    // show the resulting offset
    std::cout << std::endl << "The Bumblebee offset: " << std::fixed << std::setprecision(10) << bumblebee_offset->estimate().toVector().transpose() << std::endl << std::endl;

    // get all ackerman params
    for (unsigned i = 0; i < odom_ackerman_params_vertices; ++i)
    {
        // the current id
        unsigned curr_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + i;

        // get the current param
        g2o::VertexOdomAckermanParams *odom_param = dynamic_cast<g2o::VertexOdomAckermanParams*>(optimizer->vertex(curr_id));

        if (nullptr != odom_param)
        {
            // show the resulting calibration
            std::cout << "The " << curr_id - 2 << "º odom ackerman bias calibration vertex: ";
            std::cout << std::fixed << odom_param->estimate().transpose() << std::endl << std::endl;
        }
    }
}

// save the optimized estimates to the output file
void HyperGraphSclamOptimizer::SaveCorrectedVertices()
{

    std::cout << "Filename: " << output_filename << std::endl;
    // open the output files
    std::ofstream car_poses(output_filename + ".txt");
    std::ofstream velodyne_poses(output_filename + "_velodyne.txt");
    std::ofstream sick_poses(output_filename + "_sick.txt");
    std::ofstream bumblebee_poses(output_filename + "_bumblebee.txt");
    std::ofstream gps_plus(output_filename + "_gps_plus_error.txt");
    std::ofstream gps_minus(output_filename + "_gps_minus_error.txt");

    if (!car_poses.is_open() || !velodyne_poses.is_open() || !sick_poses.is_open() || !bumblebee_poses.is_open() || !gps_plus.is_open() || !gps_minus.is_open())
    {
        throw std::runtime_error("Could not open the global output files!");
    }

    // how many vertices
    unsigned size = optimizer->vertices().size();

    // report
    std::cout << "How many vertices: " << size << std::endl;

    // get the first valid id
    unsigned start_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + odom_ackerman_params_vertices - 1;

    for (std::pair<const std::string, std::vector<g2o::VertexSE2*> > &entry: logs)
    {
        std::vector<g2o::VertexSE2*> &vs(entry.second);
        std::map<double, g2o::VertexSE2*> sorted_vertices;

        for (g2o::VertexSE2 *v : vs)
        {
            if (start_id < unsigned(v->id()))
            {
                double t = id_time_type_map.at(v->id()).first;
                sorted_vertices[t] = v;
            }
        }

        std::ofstream sep_car_poses(output_filename + "_" + entry.first + ".txt");

        if (!sep_car_poses.is_open())
        {
            throw std::runtime_error("Could not open the separated output file!");
        }

        for (std::pair<const double, g2o::VertexSE2*> &entry: sorted_vertices)
        {
            g2o::VertexSE2 *v(entry.second);
            const g2o::SE2 &pose(v->estimate());

            g2o::SE2 plus { pose * g2o::SE2(0, 2, M_PI_2) };
            g2o::SE2 minus { pose * g2o::SE2(0, -2, -M_PI_2) };

            gps_plus  << std::fixed << plus[0]  << " " << plus[1]  << "\n";
            gps_minus << std::fixed << minus[0] << " " << minus[1] << "\n";

            Eigen::Vector2d p(pose.translation() + gps_origin);

            double cosa = std::cos(pose[2]);
            double sina = std::sin(pose[2]);

            std::pair<double, unsigned> time_type { id_time_type_map.at(unsigned(v->id())) };

            double t = time_type.first;

            StampedMessageType msg_type = StampedMessageType(time_type.second);

            sep_car_poses << std::fixed << p[0] << " " << p[1] << " " << pose[2] << " " << t << " " << cosa << " " << sina << "\n";
            car_poses << std::fixed << p[0] << " " << p[1] << " " << pose[2] << " " << t << " " << cosa << " " << sina << "\n";

            switch (msg_type)
            {
                case StampedMessageType::StampedVelodyneMessage:
                    velodyne_poses << std::fixed << p[0] << " " << p[1] << " " << pose[2] << " " << t << " " << cosa << " " << sina << "\n";
                    break;
                case StampedMessageType::StampedBumblebeeMessage:
                    bumblebee_poses << std::fixed << p[0] << " " << p[1] << " " << pose[2] << " " << t << " " << cosa << " " << sina << "\n";
                    break;
                case StampedMessageType::StampedSICKMessage:
                    sick_poses << std::fixed << p[0] << " " << p[1] << " " << pose[2] << " " << t << " " << cosa << " " << sina << "\n";
                default:
                    break;
            }
        }
        std::cout << "Closing another separated file!" << std::endl;
        sep_car_poses.close();
    }

    std::cout << "Closing all files!" << std::endl;

    // close the output files
    car_poses.close();
    velodyne_poses.close();
    sick_poses.close();
    bumblebee_poses.close();
    gps_plus.close();
    gps_minus.close();

    // save the external loops
    SaveExternalLoops();

    // save the parameters vertices
    ShowAllParametersVertices();

}


// the main optimization loop
void HyperGraphSclamOptimizer::OptimizationLoop()
{
    // the outer loop
    for (unsigned i = 0; i < external_loop; ++i)
    {
        // optimzization status report
        std::cout << "First Stage Optimization with " << optimizer->vertices().size() << " vertices" << std::endl;

        // set the internal flags
        PreparePrevOptimization();

        // optimize
        optimizer->optimize(internal_loop);

        // disable all vertices
        PrepareIndividualOptimization();

        std::cout << "Second Stage Optimization: Individual Logs!" << std::endl;

        for (std::pair<const std::string, std::vector<g2o::VertexSE2*>> &entry : logs)
        {
            std::vector<g2o::VertexSE2*> &vs(entry.second);
            std::cout << "Current gid: " << entry.first << " and size: " << vs.size() << std::endl;

            ManageHypergraphRegion(vs, false);

            optimizer->optimize(internal_loop);

            ManageHypergraphRegion(vs, true);
        }

        // set the internal flags
        PreparePostOptimization();

        // optimzization status report
        std::cout << "Third stage: odometry calibration optimization" << std::endl;

        // the input value is the maximum number of iterations
        optimizer->optimize(optimizer_inner_odom_calib_iterations);

        // restart the odometry measures
        PrepareRoundOptimization();
    }

    // optimzization status report
    std::cout << "Optimization Done!" << std::endl;
}


// verify if the optimizer is ready
bool HyperGraphSclamOptimizer::Good()
{
    return nullptr != optimizer;
}


// the main run method
void HyperGraphSclamOptimizer::Run()
{
    // start
    std::cout << "Start running!\n";

    if (nullptr != optimizer)
    {
        // the main optimization process
        OptimizationLoop();

        // save the optimized graph to the output file
        SaveCorrectedVertices();
    }
}


// clear the entire hypergraph
void HyperGraphSclamOptimizer::Clear()
{
    // clear the id timestamps map
    id_time_type_map.clear();
    logs.clear();

    if (nullptr != optimizer)
    {
        // remove all edges and vertices
        optimizer->clear();

        // remove it from the stack
        delete optimizer;

        // reset the value
        optimizer = nullptr;
    }
}
