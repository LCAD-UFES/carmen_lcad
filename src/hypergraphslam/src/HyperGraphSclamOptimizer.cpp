#include <HyperGraphSclamOptimizer.hpp>

#include <algorithm>

using namespace hyper;

// the basic constructor
HyperGraphSclamOptimizer::HyperGraphSclamOptimizer(int argc, char **argv) :
        gps_origin(0.0, 0.0),
        optimizer(nullptr),
        factory(nullptr),
        id_time_map(),
        gps_buffer(0)
{

    // get the input filename
    input_filename = std::string(argv[1]);
    output_filename = std::string(argv[2]);

    (void) argc;

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
HyperGraphSclamOptimizer::~HyperGraphSclamOptimizer() {

    // call the clear method
    Clear();

}

// return a 3x3 information matrix given the diagonal covariances
Eigen::Matrix3d HyperGraphSclamOptimizer::GetInformationMatrix(double xx_var, double yy_var, double hh_var) {

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
void HyperGraphSclamOptimizer::RegisterCustomTypes() {

    // register the custom gps edge
    factory->registerType("EDGE_GPS", new g2o::HyperGraphElementCreator<g2o::EdgeGPS>);

    // register the custom sick edge
    factory->registerType("EDGE_SICK_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeSickCalibration>);

    // register the custom velodyne edge
    factory->registerType("EDGE_VELODYNE_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeVelodyneCalibration>);

    // register the custom odometry calibration edge
    factory->registerType("EDGE_SE2_ODOM_ACKERMAN_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::EdgeSE2OdomAckermanCalibration>);

    // register the custom curvature constraint edge
    factory->registerType("EDGE_CURVATURE_CONSTRAINT", new g2o::HyperGraphElementCreator<g2o::EdgeCurvatureConstraint>);

    // register the custom vertex
    factory->registerType("VERTEX_ODOM_ACKERMAN_PARAM_CALIBRATION", new g2o::HyperGraphElementCreator<g2o::VertexOdomAckermanParams>);

}

// initialize the sparse optimizer
void HyperGraphSclamOptimizer::InitializeOptimizer() {

    // creates a new sparse optimizer in memmory
    optimizer = new g2o::SparseOptimizer;

    // allocate a new cholmod solver
    HyperCholmodSolver *cholmod_solver = new HyperCholmodSolver();

    // the block ordering
    cholmod_solver->setBlockOrdering(false);

    // the base solver
    g2o::Solver *solver = new HyperBlockSolver(cholmod_solver);

    // the base solver
    g2o::OptimizationAlgorithm *optimization_algorithm = new g2o::OptimizationAlgorithmGaussNewton(solver);

    // set the cholmod solver
    optimizer->setAlgorithm(optimization_algorithm);

    // set the verbose mode
    optimizer->setVerbose(true);

}

// add the sick, velodyne and odometry calibration vertices
void HyperGraphSclamOptimizer::AddParametersVertices() {

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

    // marginalize it
    velodyne_offset->setMarginalized(true);

    // set fixed
    velodyne_offset->setFixed(true);

    // save the offset vertex
    if (!optimizer->addVertex(velodyne_offset)) {

        // error
        throw std::runtime_error("Could not add the velodyne offset vertex to the optimizer!");

    }

    // include all ackerman params
    for (unsigned i = 0; i < ODOM_ACKERMAN_PARAMS_VERTICES; ++i) {

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
        if (!optimizer->addVertex(odom_param)) {

            // error
            throw std::runtime_error("Could not add the odom ackerman params calibration vertex to the optimizer!");

        }

    }

}

// read the current vertex from the input stream and save it to the optimizer
void HyperGraphSclamOptimizer::AddVertex(std::stringstream &ss) {

    // helpers
    unsigned vertex_id;
    double x, y, theta, timestamp;

    // read the current line
    ss >> vertex_id >> x >> y >> theta >> timestamp;

    // creates the new vertex
    g2o::VertexSE2 *v = new g2o::VertexSE2;

    // set the id
    v->setId(vertex_id);

    // set the tranlation
    v->setEstimate(g2o::SE2(x, y, theta));

    // disable the vertex
    // v->setFixed(true);

    // marginalize it
    // v->setMarginalized(true);

    // add to the optimizer
    if(!optimizer->addVertex(v)) {

        // error
        throw std::runtime_error("Could not add a new vertex to the optimizer!");

    }

    // save the timestamp
    id_time_map[vertex_id] = timestamp;

}

// read the sick edge from the input stream and save it to the optimizer
void HyperGraphSclamOptimizer::AddSickEdge(std::stringstream &ss, Eigen::Matrix3d &sick_icp_information) {

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

}

// read the velodyne edge from the input stream and save it to the optimizer
void HyperGraphSclamOptimizer::AddVelodyneEdge(std::stringstream &ss, Eigen::Matrix3d &velodyne_icp_information) {

    // helpers
    unsigned from, to;
    double x, y, theta;

    // read the ids and icp measure
    ss >> from >> to >> x >> y >> theta;

    // create the new calibration edge
    g2o::EdgeVelodyneCalibration *velodyne_seq_edge = new g2o::EdgeVelodyneCalibration();

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
    const Eigen::Matrix3d &info) {

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
    if (0.0 == time) {

        // set the special info
        odom_calib_edge->setInformation(special);

    } else {

        // set the general info
        odom_calib_edge->setInformation(info);

    }

    if (!optimizer->addEdge(odom_calib_edge)) {

        // error
        throw std::runtime_error("Could not add the odometry calibration edge to the optimizer!");

    }

}

// read the odometry edge and the odometry calibration edge and save them to the optimizer
void HyperGraphSclamOptimizer::AddOdometryAndCalibEdges(
    std::stringstream &ss, unsigned odom_param_id, const Eigen::Matrix3d &special, const Eigen::Matrix3d &odom_info) {

    // helpers
    unsigned from, to;
    double x, y, theta, _v, _p, _t;

    // read the vertices ids, measures, etc
    ss >> from >> to >> x >> y >> theta >> _v >> _p >> _t;

    // get both vertices
    g2o::VertexSE2 *l_vertex = static_cast<g2o::VertexSE2*>(optimizer->vertex(from));
    g2o::VertexSE2 *r_vertex = static_cast<g2o::VertexSE2*>(optimizer->vertex(to));

    // creates a new edge
    g2o::EdgeSE2 *edge = new g2o::EdgeSE2();

    // set the vertices
    edge->vertices()[0] = l_vertex;
    edge->vertices()[1] = r_vertex;

    // set the base measurement
    edge->setMeasurement(g2o::SE2(x, y, theta));

    if (0.0 == x && 0.0 == y && 0.0 == theta) {

        // set the special info
        edge->setInformation(special);

    } else {

        // set the info
        edge->setInformation(odom_info);

    }

    if (!optimizer->addEdge(edge)) {

        // error
        throw std::runtime_error("Could not add the odometry edge to the optimizer!");

    }

    (void) odom_param_id;

    // add a new odom calibration edge
    AddOdomCalibrationEdge(l_vertex, r_vertex, edge, odom_param_id, _v, _p, _t, special, odom_info);

}

// gps edges filtering
void HyperGraphSclamOptimizer::AddFilteredGPSEdge(g2o::EdgeGPS *next_gps) {

    if (2 == gps_buffer.size()) {

        // get the neighbours
        g2o::EdgeGPS *prev_gps(gps_buffer.front());
        g2o::EdgeGPS *current_gps(gps_buffer.back());

        // udpate the neighbors pointers inside the current one
        if (current_gps->updateNeighbors(prev_gps, next_gps)) {

            if (!optimizer->addEdge(current_gps)) {

                // error
                throw std::runtime_error("Could not add the gps edge to the optimizer!");

            }

        } else {

            // remove it from the gps_buffer
            gps_buffer.pop_back();

            // delete it
            delete current_gps;

        }

        // remove the latest from the gps_buffer
        gps_buffer.pop_front();

        if (!prev_gps->validGPS()) {

            delete prev_gps;

        }

    }

    // save the current gps edge
    gps_buffer.push_back(next_gps);

}

// read the gps edge and save it to the optimizer
void HyperGraphSclamOptimizer::AddGPSEdge(std::stringstream &ss, Eigen::Matrix3d &cov) {

    // helpers
    unsigned from;
    double x, y, theta, gps_std;

    // creates a new gps edge
    g2o::EdgeGPS *next_gps = new g2o::EdgeGPS;

    // read the vertex id, gps measure and std deviation
    ss >> from >> x >> y >> theta >> gps_std;

    // set the vertices
    next_gps->vertices()[0] = dynamic_cast<g2o::VertexSE2*>(optimizer->vertex(from));

    // update the internal measurement
    next_gps->setMeasurement(g2o::SE2(x, y, theta));

    // update the diagonals
    cov.data()[0] = cov.data()[4] = std::pow(gps_std * GPS_POSE_STD_MULTIPLIER, 2);

    // set the info matrix
    next_gps->setInformation(Eigen::Matrix3d(cov.inverse()));

    // save the gps edge to the optimizer if it's ok
    AddFilteredGPSEdge(next_gps);

}

// read the xsens edge and save it to the optimizer
void HyperGraphSclamOptimizer::AddXSENSEdge(std::stringstream &ss, Eigen::Matrix<double, 1, 1> &information) {

    // helpers
    unsigned from;
    double yaw;

    // creates the new xsens edge
    g2o::EdgeXSENS *xsens_edge = new g2o::EdgeXSENS;

    // read the vertex id and the yaw measure
    ss >> from >> yaw;

    // set the vertices
    xsens_edge->vertices()[0] = optimizer->vertex(from);

    // set the measurement
    xsens_edge->setMeasurement(yaw);

    // set the information matrix
    xsens_edge->setInformation(information);

    // try to save it
    if (!optimizer->addEdge(xsens_edge)) {

        // error
        throw std::runtime_error("Could not add the xsens edge to the optimizer!");

    }

}

// read the curvature edge and save it to the optimizer
void HyperGraphSclamOptimizer::AddCurvatureConstraintEdge(std::stringstream &ss, Eigen::Matrix3d &information) {

    // helpers
    unsigned l, c, r;

    // read the actual ids
    ss >> l >> c >> r;

    // the timestamps
    double t1 = id_time_map[l];
    double t2 = id_time_map[c];
    double t3 = id_time_map[r];

    double dt1 = std::fabs(t2 - t1);
    double dt2 = std::fabs(t3 - t2);

    if (0.0001 < dt1 && 0.0001 < dt2) {

        // build a new curvature edge
        g2o::EdgeCurvatureConstraint *edge = new g2o::EdgeCurvatureConstraint;

        // set the current vertex
        edge->vertices()[0] = optimizer->vertex(l);
        edge->vertices()[1] = optimizer->vertex(c);
        edge->vertices()[2] = optimizer->vertex(r);

        // set the information matrix
        edge->setInformation(information);

        // set the delta time
        edge->setTimeDifference(dt1 / std::fabs(t3 - t1));

        // try to save it
        if (!optimizer->addEdge(edge)) {

            // error
            throw std::runtime_error("Could not add the curvature constraint edge to the optimizer!");

        }

    }

}

// manage the hypergraph region
void HyperGraphSclamOptimizer::ManageHypergraphRegion(std::vector<g2o::VertexSE2*> &group, bool status) {

    // helpers
    std::vector<g2o::VertexSE2*>::iterator it(group.begin());
    std::vector<g2o::VertexSE2*>::iterator end(group.end());

    while (end != it) {

        // direct access
        g2o::VertexSE2* v = *it;

        // set the marginalization flag
        v->setMarginalized(status);

        // set the fixed flag
        v->setFixed(status);

        // go to the next vertex
        ++it;

    }

}

// reset the graph to the pose estimation
void HyperGraphSclamOptimizer::PreparePrevOptimization() {

    for (g2o::SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it) {

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

    for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {

        g2o::OptimizableGraph::Edge* e = static_cast<g2o::OptimizableGraph::Edge*>(*it);

        // specify the robust kernel to be used in this edge
        e->setRobustKernel(nullptr);

    }

    // pre-compute the active errors
    optimizer->computeActiveErrors();

}

// reset the graph to the odometry calibration estimation
void HyperGraphSclamOptimizer::PreparePostOptimization() {

    // preprare
    for (g2o::SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it) {

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

    for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {

        g2o::EdgeSE2OdomAckermanCalibration *odom_calib_edge = dynamic_cast<g2o::EdgeSE2OdomAckermanCalibration*>(*it);

        if (nullptr != odom_calib_edge) {

            // get the measurements from the current vertices
            odom_calib_edge->getMeasurementFromVertices();

        }

    }

    // pre-compute the active errors
    optimizer->computeActiveErrors();

}

// reset the graph to the next optimization round
void HyperGraphSclamOptimizer::PrepareRoundOptimization() {

    // update the odometry measure
    for (g2o::SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {

        g2o::EdgeSE2OdomAckermanCalibration *odom_calib_edge = dynamic_cast<g2o::EdgeSE2OdomAckermanCalibration*>(*it);

        if (nullptr != odom_calib_edge) {

            // get the measurements from the current vertices
            odom_calib_edge->updateOdometryMeasure();

        }

    }

}

// load the data into the optimizer
void HyperGraphSclamOptimizer::LoadHyperGraphToOptimizer() {

    // try to open the input filename
    std::ifstream is(input_filename, std::ifstream::in);

    if (!is.is_open()) {

        throw std::runtime_error("Could not read the input file!");

    }

    // helpers
    std::stringstream ss;
    unsigned subdivision = std::numeric_limits<unsigned>::max();
    unsigned odom_counter = 0;
    unsigned odom_param_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID;
    unsigned last_odom_param_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + ODOM_ACKERMAN_PARAMS_VERTICES - 1;

    // get the inverse of the covariance matrix,
    Eigen::Matrix3d odom_information(GetInformationMatrix(ODOMETRY_XX_VAR, ODOMETRY_YY_VAR, ODOMETRY_HH_VAR));
    Eigen::Matrix3d sick_icp_information(GetInformationMatrix(SICK_ICP_XX_VAR, SICK_ICP_YY_VAR, SICK_ICP_HH_VAR));
    Eigen::Matrix3d velodyne_icp_information(GetInformationMatrix(VELODYNE_ICP_XX_VAR, VELODYNE_ICP_YY_VAR, VELODYNE_ICP_HH_VAR));
    Eigen::Matrix3d curvature_constraint_information(GetInformationMatrix(CURVATURE_XX_VAR, CURVATURE_YY_VAR, CURVATURE_HH_VAR));
    Eigen::Matrix3d special_odom_information(Eigen::Matrix3d::Identity() * SPECIAL_ODOM_INFORMATION);
    Eigen::Matrix<double, 1, 1> xsens_information(Eigen::Matrix<double, 1, 1>::Identity() * 1.0 /  std::pow(XSENS_CONSTRAINT_VAR, 2));

    // set the odometry covariance matrix
    Eigen::Matrix3d cov(Eigen::Matrix3d::Identity());
    cov.data()[8] = std::pow(GPS_POSE_HH_STD, 2);

    // add the parameters vertexes
    AddParametersVertices();

    // the main loop
    while(-1 != hyper::StringHelper::ReadLine(is, ss)) {

        // the tag
        std::string tag;

        // read the tag
        ss >> tag;

        // the VERTEX tags should become first in the sync file
        if ("VERTEX" == tag) {

            // push the new vertex to the optimizer
            AddVertex(ss);

        } else if ("ODOM_EDGE" == tag) {

            // add odom edges
            AddOdometryAndCalibEdges(ss, odom_param_id, special_odom_information, odom_information);

            // increment the odometry counter
            ++odom_counter;

            if (0 == odom_counter % subdivision && last_odom_param_id > odom_param_id) {

                // increment the param index
                ++odom_param_id;

            }

        } else if ("GPS_EDGE" == tag){

            // add the gps edge to the optimizer
            AddGPSEdge(ss, cov);

        } else if ("XSENS_EDGE_" == tag) {

            // add the xsens edge to the optimizer
            AddXSENSEdge(ss, xsens_information);

        } else if ("SICK_SEQ_" == tag) {

            // push the sick edge to the optimizer
            AddSickEdge(ss, sick_icp_information);

        } else if ("SICK_LOOP" == tag) {

        } else if ("VELODYNE_SEQ" == tag) {

            // push the velodyne icp edge to the optimizer
            AddVelodyneEdge(ss, velodyne_icp_information);

        } else if ("VELODYNE_LOOP" == tag) {

        } else if ("BUMBLEBEE_SEQ" == tag) {

        } else if ("BUMBLEBE_LOOP" == tag) {

        } else if ("CURVATURE_CONSTRAINT_" == tag) {

            // create the curvature constraint edge
            AddCurvatureConstraintEdge(ss, curvature_constraint_information);

        } else if ("GPS_ORIGIN" == tag) {

            // read the origin
            ss >> gps_origin[0] >> gps_origin[1];

            std::cout << "GPS origin: " << std::fixed << gps_origin.transpose() << std::endl;

        } else if ("VERTICES_QUANTITY" == tag) {

            // read the qnt
            ss >> subdivision;

            // divide it by 4
            subdivision /= ODOM_ACKERMAN_PARAMS_VERTICES;

        }

    }

    // close the input file stream
    is.close();

}

// save the optimized estimates to the output file
void HyperGraphSclamOptimizer::SaveCorrectedVertices() {

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

    // get all ackerman params
    for (unsigned i = 0; i < ODOM_ACKERMAN_PARAMS_VERTICES; ++i) {

        // the current id
        unsigned curr_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + i;

        // get the current param
        g2o::VertexOdomAckermanParams *odom_param = dynamic_cast<g2o::VertexOdomAckermanParams*>(optimizer->vertex(curr_id));

        if (nullptr != odom_param) {

            // show the resulting calibration
            std::cout << "The current " << curr_id << " odom ackerman calib: " << std::fixed << odom_param->estimate().transpose() << std::endl;

        }

    }

    // get the first valid id
    unsigned start_id = ODOM_ACKERMAN_PARAMS_VERTEX_INITIAL_ID + ODOM_ACKERMAN_PARAMS_VERTICES - 1;

    // sort by timestamp
    g2o::SparseOptimizer::VertexIDMap &vs(optimizer->vertices());

    (void) vs;

    // iterators
    g2o::SparseOptimizer::VertexIDMap::iterator it(optimizer->vertices().begin());
    g2o::SparseOptimizer::VertexIDMap::iterator end(optimizer->vertices().end());

    while (end != it) {

        // downcast to the base vertex
        g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(it->second);

        if (nullptr != v && start_id < unsigned(v->id())) {

            // get the estimate
            Eigen::Vector3d pose(v->estimate().toVector());

            // build the base
            pose[0] += gps_origin[0];
            pose[1] += gps_origin[1];

            double a = pose[2];

            double sina = std::sin(a);
            double cosa = std::cos(a);

            double timestamp = id_time_map.at((unsigned) v->id());

            if (0 > v->id()) {

                // error
                throw std::runtime_error("Invalid verteix id");

            }

            // write to the output file
            ofs << std::fixed << pose[0] << " " << pose[1] << " " << pose[2] << " " << timestamp << " " << a << " " << sina << " " << cosa << "\n";

        }

        // go to the next vertex
        ++it;

    }


    // close the output file
    ofs.close();

}

// the main optimization loop
void HyperGraphSclamOptimizer::OptimizationLoop() {

    // the outer loop
    for (unsigned i = 0; i < OPTIMIZER_OUTER_ITERATIONS; ++i) {

        // optimzization status report
        std::cout << "First Stage Optimization with " << optimizer->vertices().size() << " vertices" << std::endl;

        // set the internal flags
        PreparePrevOptimization();

        // optimize
        optimizer->optimize(OPTIMIZER_INNER_POSE_ITERATIONS);

        // optimzization status report
        std::cout << "Second stage optimization with " << optimizer->vertices().size() << " vertices" << std::endl;

        // set the internal flags
        PreparePostOptimization();

        // the input value is the maximum number of iterations
        optimizer->optimize(OPTIMIZER_INNER_ODOM_CALIB_ITERATIONS);

        // restart the odometry measures
        PrepareRoundOptimization();

    }

    // optimzization status report
    std::cout << "Optimization Done!" << std::endl;

}

// verify if the optimizer is ready
bool HyperGraphSclamOptimizer::Good() {

    return nullptr != optimizer;

}

// the main run method
void HyperGraphSclamOptimizer::Run() {

    // start
    std::cout << "Start running!\n";

    if (nullptr != optimizer) {

        // the main optimization process
        OptimizationLoop();

        // save the optimized graph to the output file
        SaveCorrectedVertices();

    }

}

// clear the entire hypergraph
void HyperGraphSclamOptimizer::Clear() {

    // clear the id timestamps map
    id_time_map.clear();

    if (nullptr != factory) {

        // destroy the factory
        g2o::Factory::destroy();

        // reset the pointer
        factory = nullptr;

    }

    if (nullptr != optimizer) {

        // remove all edges and vertices
        optimizer->clear();

        // remove it from the stack
        delete optimizer;

        // reset the value
        optimizer = nullptr;

    }

}
