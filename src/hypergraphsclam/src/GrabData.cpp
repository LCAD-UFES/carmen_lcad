#include <GrabData.hpp>

#include <cmath>
#include <thread>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <limits>
#include <unistd.h>
#include <list>

using namespace hyper;

// the main constructor
GrabData::GrabData() :
    raw_messages(0),
    messages(0),
    gps_messages(0),
    velodyne_messages(0),
    sick_messages(0),
    point_cloud_lidar_messages(nullptr),
    odometry_messages(0),
    gps_origin(0.0, 0.0),
    icp_start_index(0),
    icp_end_index(0),
    icp_mutex(),
    dmax(std::numeric_limits<double>::max()) {}

// the main destructor
GrabData::~GrabData() {

    // clear all the messages
    Clear();

}

// PRIVATE METHODS

// clear the messages

// separate the gps and velodyne
void GrabData::SeparateMessages() {

    // clear the gps, sick and velodyne list
    messages.clear();
    gps_messages.clear();
    sick_messages.clear();
    velodyne_messages.clear();
        // bumblebee_messages.clear();

    // helpers
    StampedMessagePtrVector::iterator it(raw_messages.begin());
    StampedMessagePtrVector::iterator end(raw_messages.end());

    // status report
    std::cout << "Separating all " << raw_messages.size() << " raw messages by their types" << std::endl;

    // the speed flag
    bool valid_velocity = false;

    // how many lidar odometry measures with velocity equals to zero
    int max_zeros = 2;

    // the odometry messages with invalid speed counter
    int zeros = max_zeros;

    // try to downcast
    // find the first valid odometry value
    while (end != it) {

        // try to downcast
        StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(*it);

        if (nullptr != odom && 0.001 < std::fabs(odom->v)) {

            // break the loop
            break;

        }

        // go to the next messsage
        ++it;

    }

    while (end != it) {

        // try the downcasting
        StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(*it);

        if (nullptr != odom) {

            // does it have a valid velocity?
            valid_velocity = 0.001 < std::fabs(odom->v);

            zeros = valid_velocity ? max_zeros : zeros - 1;

        }

        if (valid_velocity || 0 < zeros) {

            // direct access
            StampedMessagePtr msg = *it;

            // try to downcast
            StampedGPSPosePtr gps_pose = dynamic_cast<StampedGPSPosePtr>(msg);
            StampedSICKPtr sick = dynamic_cast<StampedSICKPtr>(msg);
            StampedVelodynePtr velodyne = dynamic_cast<StampedVelodynePtr>(msg);
            // StampedVelodynePtr bumblebee = dynamic_cast<StampedBumblebeePtr>(*it);

            if (nullptr != odom) {

                odometry_messages.push_back(odom);

                zeros = valid_velocity ? max_zeros : zeros - 1;

            } else if (nullptr != gps_pose) {

                gps_messages.push_back(gps_pose);

            } else if (nullptr != sick) {

                sick_messages.push_back(static_cast<StampedLidarPtr>(sick));

            } else if (nullptr != velodyne) {

                velodyne_messages.push_back(static_cast<StampedLidarPtr>(velodyne));

            }

            // save to the the general messages
            messages.push_back(msg);

        }

        // go to the next message
        ++it;

    }

    // status report
    std::cout << "The messages were separated" << std::endl;

    std::cout << "GPS pose messages: " << gps_messages.size() << std::endl;
    std::cout << "SICK messages: " << sick_messages.size() << std::endl;
    std::cout << "Velodyne messages: " << velodyne_messages.size() << std::endl;
    std::cout << "Odometry messages: " << odometry_messages.size() << std::endl;
    std::cout << "All valid messages: " << messages.size() << std::endl;

}

// get the gps estimation
void GrabData::GetNearestOrientation(
            StampedMessagePtrVector::iterator it,
            StampedMessagePtrVector::iterator end,
            int adv,
            double timestamp,
            double &h,
            double &dt)
{

    // move the main iterator
    std::advance(it, adv);

    // iterate to the left messages
    while (end != it) {

        // downcasting to gps orientation
        StampedGPSOrientationPtr gps_orientation = dynamic_cast<StampedGPSOrientationPtr>(*it);

        if (nullptr != gps_orientation) {

            // get the heading info
            h = mrpt::math::wrapToPi<double>(gps_orientation->yaw);

            // get the timestamp
            dt = std::fabs(timestamp - gps_orientation->timestamp);

            break;

        }

        // go to the left/right
        std::advance(it, adv);

    }

}
// get the gps estimation
Eigen::Rotation2Dd GrabData::GetGPSRotation(
            StampedMessagePtrVector::iterator begin,
            StampedMessagePtrVector::iterator gps,
            StampedMessagePtrVector::iterator end,
            double timestamp)
{

    // the left gps orientation and dt
    double lh = 0.0, ldt = 0.0;

    // the right gps orientation and dt
    double rh = 0.0, rdt = 0.0;

    // iterate to the left
    GetNearestOrientation(gps, begin, -1, timestamp, lh, ldt);

    // iterate to the right
    GetNearestOrientation(gps, end, 1, timestamp, rh, rdt);

    // set the current heading
    double yaw = ldt < rdt ? lh : rh;

    // create the axis
    return Eigen::Rotation2Dd(mrpt::math::wrapToPi<double>(yaw));

}

// get the first gps position
Eigen::Vector2d GrabData::GetFirstGPSPosition() {

    if (!gps_messages.empty()) {

        // update the gps position
        return gps_messages.front()->gps_measure.translation();

    }

    return Eigen::Vector2d(0.0, 0.0);

}

// filter the entire gps positions
void GrabData::GPSFiltering() {

    if (3 < gps_messages.size()) {

        // status report
        std::cout << "Starting the GPS filtering process" << std::endl;

        // the iterators
        StampedGPSPosePtrVector::iterator end = gps_messages.end();
        StampedGPSPosePtrVector::iterator prev = gps_messages.begin();
        StampedGPSPosePtrVector::iterator it = prev;
        StampedGPSPosePtrVector::iterator next = prev;

        // advance
        std::advance(it, 1);
        std::advance(next, 2);

        while (end != next) {

            // direct access
            StampedGPSPosePtr ln = *prev;
            StampedGPSPosePtr curr = *it;
            StampedGPSPosePtr rn = *next;

            // get the gps positions
            const Eigen::Vector2d &gps_left_neighbour(ln->gps_measure.translation());
            const Eigen::Vector2d &gps_current(curr->gps_measure.translation());
            const Eigen::Vector2d &gps_right_neighbour(rn->gps_measure.translation());

            // get the displacements
            Eigen::Vector2d nd(gps_right_neighbour - gps_left_neighbour);
            Eigen::Vector2d lnd(gps_current - gps_left_neighbour);
            Eigen::Vector2d rnd(gps_right_neighbour - gps_current);

            // verify the distance between the neighbours
            double nd_norm = nd.squaredNorm();
            double lnd_norm = lnd.squaredNorm();
            double rnd_norm = rnd.squaredNorm();

            // get the delta time
            double ldt = curr->timestamp - ln->timestamp;
            double rdt = rn->timestamp - curr->timestamp;
            double tdt = ldt + rdt;

            // the max allowed distance from the left neighbour
            double mld = ldt * GPS_FILTER_THRESHOLD;

            // the max allowed distance from the right neighbour
            double mrd = rdt * GPS_FILTER_THRESHOLD;

            // verify the noise case
            if ((lnd_norm > mld && rnd_norm > mld) && (nd_norm < mld || nd_norm < mrd) && 0.0 != tdt ) {

                // update the message measure
                curr->gps_measure.setTranslation(gps_left_neighbour + (ldt / tdt) * nd);

            }

            // advance
            prev = it;
            it = next;
            ++next;

        }

        // status report
        std::cout << "The GPS messages were filtered" << std::endl;

    }

}

// build the gps measures
// including the orientation from xsens sensor or the gps itself
void GrabData::BuildGPSMeasures() {

    if (5 < messages.size()) {

        // status reporting
        std::cout << "Start building the GPS measures and estimates\n";

        // helpers
        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator begin(messages.begin());
        StampedMessagePtrVector::iterator it(begin);

        // find the first gps position
        gps_origin = GetFirstGPSPosition();

        while (end != it) {

            // downcasting
            StampedGPSPosePtr gps_pose = dynamic_cast<StampedGPSPosePtr>(*it);

            if (nullptr != gps_pose) {

                // update the reference frame
                gps_pose->gps_measure.setTranslation(gps_pose->gps_measure.translation() - gps_origin);

                // update the orientation
                gps_pose->gps_measure.setRotation(GetGPSRotation(begin, it, end, gps_pose->timestamp));

                // let's take an advantage here
                // gps_pose->est = gps_pose->gps_measure;

            }

            // go to the next message
            ++it;

        }

        // status reporting
        std::cout << "GPS measures and estimates were built successfully\n";

    }

}

// build the odometry measures
void GrabData::BuildOdometryMeasures() {

    if (5 < messages.size() && !odometry_messages.empty()) {

        // status reporting
        std::cout << "Start building the odometry measures\n";

        // the current command
        double v = 0.0, phi = 0.0;

        // iterators
        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator begin(messages.begin());
        StampedMessagePtrVector::iterator it(begin);

        // find the first odometry message position
        while (end != it) {

            // try to downcast the current message
            StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(*it);

            if (nullptr != odom) {

                // get the initial odometry values
                v = odom->v;
                phi = odom->phi;

                // quit the loop
                break;

            }

            // go to the next message
            ++it;

        }

        if (end != it) {

            if (begin != it) {

                // iterators
                StampedMessagePtrVector::reverse_iterator rend(messages.rend());
                StampedMessagePtrVector::reverse_iterator rit(it + 1);
                StampedMessagePtrVector::reverse_iterator prev(it);

                while (rend != prev) {

                    // ge the internal message pointers
                    StampedMessagePtr current_msg = *rit;
                    StampedMessagePtr prev_msg = *prev;

                    // update the measure
                    prev_msg->odom_measure = VehicleModel::GetOdometryMeasure(v, phi, current_msg->timestamp - prev_msg->timestamp);

                    // go to the previous message
                    rit = prev;
                    ++prev;

                }

            }

            // get the next iterator
            StampedMessagePtrVector::iterator next(it + 1);

            while (end != next) {

                // get the internal pointers
                StampedMessagePtr current_msg = *it;
                StampedMessagePtr next_msg = *next;

                // try to downcast the current message to the odometry one
                StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(current_msg);

                if (nullptr != odom) {

                    // update the odometry values
                    v = odom->v;
                    phi = odom->phi;

                } else {

                    // save the v value inside the lidar message
                    StampedLidarPtr lidar = dynamic_cast<StampedLidarPtr>(current_msg);

                    if (nullptr != lidar) {

                        lidar->speed = v;

                    }

                }

                double dt = next_msg->timestamp - current_msg->timestamp;

                // get the current odometry measure
                current_msg->odom_measure = VehicleModel::GetOdometryMeasure(v, phi, dt);

                // go to the next messages
                it = next;
                ++next;

            }

        }

        // status reporting
        std::cout << "Odometry measures built\n";

    }

}

// build the initial estimates
void GrabData::BuildOdometryEstimates() {

    if (5 < messages.size()) {

        // status reporting
        std::cout << "Start building the general estimates\n";

        // helpers
        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator it(messages.begin());

        StampedGPSPosePtr first_gps_pose = nullptr;

        // forward to the first gps pose
        while (end != it) {

            // try to downcast
            first_gps_pose = dynamic_cast<StampedGPSPosePtr>(*it);

            if (nullptr != first_gps_pose) {

                // show the first gps measure
                std::cout << "First GPS measure: " << first_gps_pose->gps_measure.toVector().transpose() << std::endl;

                // set the first gps pose estimate
                first_gps_pose->est = first_gps_pose->gps_measure;

                // exit the while loop
                break;

            }

            // go to the next message
            ++it;

        }

        if (end != it && messages.begin() != it) {

            // iterate to the left
            StampedMessagePtrVector::reverse_iterator rend(messages.rend());
            StampedMessagePtrVector::reverse_iterator rb(it + 1);
            StampedMessagePtrVector::reverse_iterator ra(it);

            // update the previous estimates
            // to the left
            while (rend != ra) {

                // direct access
                StampedMessagePtr current = *rb;
                StampedMessagePtr previous = *ra;

                // set the estimate
                previous->est = current->est * (previous->odom_measure.inverse());

                // go the previous message
                rb = ra;
                ++ra;

            }

        } else {

            // reset the iterator
            it = messages.begin();

        }

        // forward iterators
        StampedMessagePtrVector::iterator a(it);
        StampedMessagePtrVector::iterator b(it + 1);

        // to the right
        while (end != b) {

            // direct access
            StampedMessagePtr current = *a;
            StampedMessagePtr next = *b;

            // set the initial estimate
            next->est = current->est * (current->odom_measure);

            // go to the next messages
            a = b;
            ++b;

        }

        // status report
        std::cout << "General estimates done!\n";

    } else {

        // error
        throw std::runtime_error("Odometry messages are missing!");

    }

}

// build an icp measure
bool GrabData::BuildLidarOdometryMeasure(
        GeneralizedICP &gicp,
        VoxelGridFilter &grid_filtering,
        double cf,
        const g2o::SE2 &odom,
        PointCloudHSV::Ptr source_cloud,
        PointCloudHSV::Ptr target_cloud,
        g2o::SE2 &icp_measure )
{

    // the resulting aligned point cloud
    PointCloudHSV result;

    // unset the dense flags
    source_cloud->is_dense = false;
    target_cloud->is_dense = false;

    // set the new correspondence factor
    gicp.setMaxCorrespondenceDistance(std::fabs(cf));

    // inverting the target and source
    gicp.setInputSource(target_cloud);
    gicp.setInputTarget(source_cloud);

    // initial guess
    Eigen::Matrix4f initial_guess(BuildMatrixFromSE2(odom));

    // perform the icp method
    //gicp.align(result, initial_guess);
    gicp.align(result, initial_guess);

    if (gicp.hasConverged()) {

        // get the final transformation
        Eigen::Matrix4f icp_guess(gicp.getFinalTransformation());

        // get the desired transformation
        icp_measure = BuildSE2FromMatrix(icp_guess);

        // the distance
        double translation_difference = (odom.translation() - icp_measure.translation()).norm();

        // the movement and the cf value should have the same sign
        if (0 < cf * icp_guess(0, 3) && ICP_TRANSLATION_CONFIDENCE_FACTOR > translation_difference) {

            // get the inverse transformation
            Eigen::Matrix4f icp_inverse(icp_guess.inverse());

            // the transformed point cloud
            pcl::PointCloud<pcl::PointXYZHSV>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZHSV>());

            // transform the cloud
            pcl::transformPointCloud(*source_cloud, *transformed_cloud, icp_inverse);

            // clear the entire source cloud
            source_cloud->clear();

            // accumulate the clouds
            *transformed_cloud += *target_cloud;

            // filtering process
            grid_filtering.setInputCloud(transformed_cloud);

            // filtering process!
            grid_filtering.filter(*source_cloud);

            // success
            return true;

        } else {

            // report
            std::cout << "Error: " << translation_difference << " is greater than " << ICP_TRANSLATION_CONFIDENCE_FACTOR << " or " << cf
                << " is lesser than zero and icp is greater, let's see icp matrix: \n" << icp_guess << std::endl;

        }

    } else {

        std::cout << "Error: It haven't converged!" << std::endl;

    }

    // invalid
    return false;

}

// build an icp measure
bool GrabData::BuildLidarLoopMeasure(
        GeneralizedICP &gicp,
        double cf,
        PointCloudHSV::Ptr source_cloud,
        PointCloudHSV::Ptr target_cloud,
        g2o::SE2 &loop_measure)
{

    // the resulting aligned point cloud
    PointCloudHSV result;

    // unset the dense flags
    source_cloud->is_dense = false;
    target_cloud->is_dense = false;

    // set the new correspondence factor
    gicp.setMaxCorrespondenceDistance(std::fabs(cf));

    // inverting the target and source
    gicp.setInputSource(target_cloud);
    gicp.setInputTarget(source_cloud);

    // perform the icp method
    gicp.align(result);

    if (gicp.hasConverged()) {

        // get the desired transformation
        loop_measure = BuildSE2FromMatrix(gicp.getFinalTransformation());

        return true;

    }

    // invalid
    return false;

}

// get the next lidar block
bool GrabData::GetNextLidarBlock(unsigned &first_index, unsigned &last_index) {

    // the status
    bool status = false;

    // lock the mutex
    icp_mutex.lock();

    // get the point cloud vector size
    const unsigned upper_limit = point_cloud_lidar_messages->size() - 1;

    // verify the start index
    if (upper_limit > icp_start_index) {

        // set the first index
        first_index = icp_start_index;

        // get the end index
        icp_end_index = icp_start_index + ICP_THREAD_BLOCK_SIZE;

        // verify the end index
        if (upper_limit < icp_end_index) {

            // set the new end index
            icp_end_index = upper_limit;

        }

        // set the end index
        last_index = icp_end_index;

        // set the start index to the next thread
        icp_start_index = icp_end_index;

        // set the status flag
        status = true;

    }

    // unlock the mutex
    icp_mutex.unlock();

    return status;

}

// get the next safe region to a given thread
bool GrabData::GetNextICPIterators(StampedLidarPtrVector::iterator &begin, StampedLidarPtrVector::iterator &end) {

    // the status
    bool status = true;

    // lock the mutex
    icp_mutex.lock();

    // get the point cloud vector size
    const unsigned upper_limit = point_cloud_lidar_messages->size() - 1;

    // verify the start index
    if (upper_limit > icp_start_index) {

        // set the begin iterator
        begin = point_cloud_lidar_messages->begin() + icp_start_index;

        // get the end index
        icp_end_index = icp_start_index + ICP_THREAD_BLOCK_SIZE;

        // verify the end index
        if (upper_limit < icp_end_index) {

            // set the new end index
            icp_end_index = upper_limit;

        }

        // set the end iterator
        end = point_cloud_lidar_messages->begin() + icp_end_index;

        // set the start index to the next thread
        icp_start_index = icp_end_index;

    } else {

        status = false;

    }

    // unlock the mutex
    icp_mutex.unlock();

    return status;

}

// the main icp measure method, multithreading version
void GrabData::BuildLidarMeasuresMT() {

    // this thread iterators
    StampedLidarPtrVector::iterator begin = point_cloud_lidar_messages->begin();
    StampedLidarPtrVector::iterator end = point_cloud_lidar_messages->end();

    // the pcl point cloud ICP solver
    GeneralizedICP gicp;

    // set the default gicp configuration
    gicp.setEuclideanFitnessEpsilon(1e-06);
    gicp.setTransformationEpsilon(1e-06);
    gicp.setMaximumIterations(2000);

    // the voxel grid filtering
    VoxelGridFilter grid_filtering;

    // set the voxel value
    grid_filtering.setLeafSize(StampedLidar::vg_leaf, StampedLidar::vg_leaf, StampedLidar::vg_leaf);

    // the local grid map
    // LocalGridMap3D<float> lgm(4.0, 40.0, 30.0f, 5.0f);

    // the lidar frequency times the base distance
    float lfd = LIDAR_ODOMETRY_MIN_DISTANCE;

    // the base path
    std::string path("/dados/tmp/lgm/");

    bool is_sick = (point_cloud_lidar_messages == &sick_messages);

    if (is_sick) {

        // set the sick base path
        path += "sick/";

        // set the origin
        // lgm.SetSickOrigin();

        // set the sick frequency
        lfd *= 25.0;

    } else {

        // set the velodyne base path
        path += "velodyne/";

        // set the velodyne frequency
        lfd *= 20.0;

    }

    // the first and last index
    unsigned current_index, last_index;

    // counter for status report
    unsigned counter = 0;

    // how many lidar messages corresponds to one percent
    unsigned percent = unsigned(0.01 * float(point_cloud_lidar_messages->size()));

    // count how many iCP errors
    unsigned errors = 0;

    // get the the range iterators
    while (GetNextLidarBlock(current_index, last_index)) {

        // get the current message pointer
        StampedLidarPtr current = *(begin + current_index);

        // the current cloud
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZHSV>());

        // try to open the current cloud
        if (-1 == pcl::io::loadPCDFile(current->path, *current_cloud)) {

            // error
            throw std::runtime_error("Could not open the source cloud");

        }

        // the main iterator
        while (current_index < last_index) {

            // compute the desired StampedMessagePtr
            unsigned next_index = current_index + unsigned(std::max(1.0, std::min(10.0, lfd / current->speed)));

            if (next_index <= last_index) {

                // get the next
                StampedLidarPtr next = *(begin + next_index);

                // the next cloud
                pcl::PointCloud<pcl::PointXYZHSV>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZHSV>());

                // try to open the next cloud
                if (-1 == pcl::io::loadPCDFile(next->path, *next_cloud)) {

                    // error
                    throw std::runtime_error("Could not open the target cloud");

                }

                // get the factor
                double cf = double(int(current->speed * (next->timestamp - current->timestamp) * 100.0)) * 0.02;

                // get the current odometry estimate
                g2o::SE2 odom(current->est.inverse() * next->est);

                if (0.0 != cf) {

                    if (BuildLidarOdometryMeasure(gicp, grid_filtering, cf, odom, current_cloud, next_cloud, current->seq_measure)) {

                        // set the base id
                        current->seq_id = next->id;

                        // save the acculated cloud
                        // StampedLidar::SavePointCloud(path, current_index, *current_cloud);

                    } else {

                        std::cout << "clouds: " << current->path << " and " << next->path << std::endl;

                        ++errors;

                    }

                }

                // update the current pointer
                current = next;

                // update the counter
                ++counter;

                if (0 == counter % percent) {

                    std::cout << "=" << std::flush;

                }

            }

            // go to the next index
            current_index = next_index;

        }

    }

    // error counter report
    std::cout << "Lidar odometry errors: " << errors << std::endl;

}

// build sequential and loop restriction ICP measures
void GrabData::BuildLidarOdometryMeasuresWithThreads(StampedLidarPtrVector &lidar_messages) {

    if (1 < lidar_messages.size()) {

        // status reporting
        std::cout << "Starting the Lidar measures with " << ICP_THREADS_POOL_SIZE << " threads" << std::endl;

        // set the point cloud lidar_messages to be used inside the lidar
        point_cloud_lidar_messages = &lidar_messages;

        // reset the index
        icp_start_index = 0;

        // the thread pool
        std::vector<std::thread> pool(0);

        // status report
        std::cout << "Start building the Lidar measures of " << lidar_messages.size() << " point clouds\n";

        // create the threads using the defined pool size
        for (unsigned i = 0; i < ICP_THREADS_POOL_SIZE; ++i) {

            // create a new thread
            pool.push_back(std::thread(&GrabData::BuildLidarMeasuresMT, this));

        }

        // wait all threads
        for (unsigned i = 0; i < ICP_THREADS_POOL_SIZE; ++i) {

            // join the thread
            pool[i].join();

        }

        // status reporting
        std::cout << "\nLidar measures done" << std::endl;

    }

}


// build the lidar odometry estimates, we should call this method after the BuildOdometryEstimates
void GrabData::BuildLidarOdometryEstimates(StampedLidarPtrVector &lidar_messages) {

    if (2 < lidar_messages.size()) {

        // the limit index
        unsigned limit_index = messages.size() + 1;

        // iterators
        StampedLidarPtrVector::iterator end(velodyne_messages.end());
        StampedLidarPtrVector::iterator current(velodyne_messages.begin());
        StampedLidarPtrVector::iterator next(current + 1);

        // the first lidar estimate is the general odometry estimate
        (*current)->lidar_estimate = (*current)->est;

        while (end != next) {

            // get the current lidar
            StampedLidarPtr current_lidar = *current;
            StampedLidarPtr next_lidar = *next;

            // set the next
            if (limit_index > current_lidar->seq_id) {

                // set the next estimate
                next_lidar->lidar_estimate = current_lidar->lidar_estimate * current_lidar->seq_measure;

            } else {

                // hack
                next_lidar->lidar_estimate = current_lidar->lidar_estimate * (current_lidar->est.inverse() * next_lidar->est);

            }

            // go to the next messages
            current = next;
            ++next;

        }

    }

}

// compute the loop closure measure
void GrabData::BuildLidarLoopClosureMeasures(StampedLidarPtrVector &lidar_messages) {

    if (!lidar_messages.empty()) {

        // report
        std::cout << "Start Build the loop closure measures from " << lidar_messages.size() << " messages\n";

        // the pcl point cloud ICP solver
        GeneralizedICP gicp;

        // set the default gicp configuration
        gicp.setEuclideanFitnessEpsilon(1e-06);
        gicp.setTransformationEpsilon(1e-06);
        gicp.setMaximumIterations(2000);

        // iterators
        StampedLidarPtrVector::iterator end(lidar_messages.end());
        StampedLidarPtrVector::iterator it(lidar_messages.begin());

        while (end != it) {

            // get the current lidar message pointer
            StampedLidarPtr current = *it;

            // the loop iterator
            StampedLidarPtrVector::iterator loop(end);

            // the mininum distance found
            double min_dist = dmax;

            // the next message
            StampedLidarPtrVector::iterator next(it + 1);

            while (end != next) {

                // get the current loop message
                StampedLidarPtr lidar_loop = *next;

                // compute the current distance
                double distance = (current->est.translation() - lidar_loop->est.translation()).squaredNorm();

                // the time difference
                double dt = std::fabs(lidar_loop->timestamp - current->timestamp);

                if (min_dist > distance && LOOP_REQUIRED_TIME < dt && LOOP_REQUIRED_SQR_DISTANCE > distance) {

                    // update the min distance
                    min_dist = distance;

                    // set the loop index
                    loop = next;

                }

                // go the next loop
                ++next;

            }

            if (end != loop) {

                // the current cloud
                pcl::PointCloud<pcl::PointXYZHSV>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZHSV>());

                // try to open the current cloud
                if (-1 == pcl::io::loadPCDFile(current->path, *current_cloud)) {

                    // error
                    throw std::runtime_error("Could not open the source cloud");

                }

                // found it
                StampedLidarPtr lidar_loop = *loop;

                // load the loop cloud
                pcl::PointCloud<pcl::PointXYZHSV>::Ptr loop_cloud(new pcl::PointCloud<pcl::PointXYZHSV>());

                // try to open the next cloud
                if (-1 == pcl::io::loadPCDFile(lidar_loop->path, *loop_cloud)) {

                    // error
                    throw std::runtime_error("Could not open the target cloud");

                }

                // try the icp method
                if (BuildLidarLoopMeasure(gicp, min_dist * 2.0, current_cloud, loop_cloud, current->loop_measure)) {

                    // set the base id
                    current->loop_closure_id = lidar_loop->id;

                }

            }

            // go to the next message
            ++it;

        }

        // report
        std::cout << "Lidar loop closure measures done!\n";

    }

}

// save all vertices to the external file
void GrabData::SaveAllVertices(std::ofstream &os) {

    // helpers
    StampedMessagePtrVector::iterator it = messages.begin();
    StampedMessagePtrVector::iterator end = messages.end();

    // savet the gps origin
    os << "GPS_ORIGIN " << std::fixed << gps_origin[0] << " " << gps_origin[1] << "\n";

    while (end != it) {

        // direct acccess
        StampedMessagePtr msg = *it;

        // write the estimate id and initial value
        os << "VERTEX " << msg->id << " " << std::fixed << std::setprecision(6) << msg->est[0] << " " << msg->est[1] << " " << msg->est[2] << " " << std::fixed << std::setprecision(25) << msg->timestamp << "\n";

        // go to the next message
        ++it;

    }

}

// save the odometry edges
void GrabData::SaveOdometryEdges(std::ofstream &os) {

    if (!messages.empty()) {

        // helpers
        StampedMessagePtrVector::iterator end = messages.end();
        StampedMessagePtrVector::iterator current = messages.begin();
        StampedMessagePtrVector::iterator next = messages.begin();
        ++next;

        while (end != next) {

            // direct acccess
            StampedMessagePtr from = *current;
            StampedMessagePtr to = *next;

            // direct acces
            const g2o::SE2 &measure(from->odom_measure);

            // write the odometry measures
            os << "ODOM_EDGE " << from->id << " " << to->id << " " << std::fixed <<  measure[0] << " " << measure[1] << " " << measure[2] << "\n";

            // go to the next messages
            current = next;
            ++next;

        }

    }

}

// save the odometry estimates
void GrabData::SaveOdometryEstimates() {

    if (!messages.empty()) {

        // open the odometry output
        std::ofstream ofs("odom.txt", std::ofstream::out);

        if (!ofs.good()) {

            throw std::runtime_error("Could not open the odometry edge output file!");

        }

        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator it(messages.begin());

        while (end != it) {

            // direct access
            Eigen::Vector3d pose((*it)->est.toVector());

            // add the gps origin
            pose[0] += gps_origin[0];
            pose[1] += gps_origin[1];

            double yaw = pose[2];

            // save the odometry estimated pose
            ofs << std::fixed << pose[0] << " " << pose[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << "\n";

            // go to the next message
            ++it;

        }

        // close it
        ofs.close();

    }

}

// save the gps edges
void GrabData::SaveGPSEdges(std::ofstream &os) {

    // helpers
    StampedGPSPosePtrVector::iterator end = gps_messages.end();
    StampedGPSPosePtrVector::iterator current = gps_messages.begin();

    while (end != current) {

        // direct access
        StampedGPSPosePtr gps = *current;

        // measure direct access
        const g2o::SE2 &measure(gps->gps_measure);

        // write the gps mesaure
        os << "GPS_EDGE " << gps->id << " " << std::fixed << measure[0] << " " << measure[1] << " " << measure[2] << " " << gps->gps_std << "\n";

        // go to the next messages
        ++current;

    }

}

// save the gps edges
void GrabData::SaveGPSEstimates() {

    if (1 < gps_messages.size()) {

        // open the odometry output
        std::ofstream ofs("gps.txt", std::ofstream::out);

        if (!ofs.good()) {

            throw std::runtime_error("Could not open the gps estimate output file!");

        }

        // helpers
        StampedGPSPosePtrVector::iterator end = gps_messages.end();
        StampedGPSPosePtrVector::iterator current = gps_messages.begin();

        while (end != current) {

            // direct access
            const g2o::SE2 &gps_measure((*current)->gps_measure);

            // get the orientation
            double yaw = gps_measure[2];

            // the translation
            Eigen::Vector2d position(gps_measure.translation() + gps_origin);

            // write the gps mesaure
            ofs << std::fixed << position[0] << " " << position[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << "\n";

            // go to the next messages
            ++current;

        }

    }

}

// save lidar message
void GrabData::SaveLidarEdges(const std::string &msg_name, std::ofstream &os, const StampedLidarPtrVector &lidar_messages) {

    if (!lidar_messages.empty()) {

        // iterate over the lidar messages
        StampedLidarPtrVector::const_iterator end = lidar_messages.end();
        StampedLidarPtrVector::const_iterator current = lidar_messages.begin();

        // limit index
        unsigned limit_index = messages.size();

        while (end != current) {

            // direct access
            StampedLidarPtr from = *current;

            if (limit_index > from->seq_id) {

                // get the SE2 reference
                g2o::SE2 &measure(from->seq_measure);

                // save the icp measure
                os << msg_name << "_SEQ " << from->id << " " << from->seq_id << " " << std::fixed << measure[0] << " " << measure[1] << " " << measure[2] << "\n";

            }

            if (limit_index > from->loop_closure_id) {

                // get the SE2 reference
                g2o::SE2 &measure(from->loop_measure);

                // save the icp measure
                os << msg_name << "_LOOP " << from->id << " " << from->loop_closure_id << " " << std::fixed << measure[0] << " " << measure[1] << " " << measure[2] << "\n";

            }

            // go to the next message
            ++current;

        }

    }

}

// save all the edges containgin icp measures, from SICK and Velodyne sensors
void GrabData::SaveICPEdges(std::ofstream &os) {

    std::cout << "\tSaving all SICK edges..." << std::endl;

    // the sick icp edges
    SaveLidarEdges(std::string("SICK"), os, sick_messages);

    std::cout << "\tSaving all Velodyne edges..." << std::endl;

    // the velodyne icp edges
    SaveLidarEdges(std::string("VELODYNE"), os, velodyne_messages);

}

// save the lidar estimates
void GrabData::SaveLidarEstimates(const std::string &filename, const StampedLidarPtrVector &lidar_messages) {

    if (5 < lidar_messages.size()) {

        // open the lidar output file
        std::ofstream ofs(filename, std::ofstream::out);

        if (!ofs.good()) {

            throw std::runtime_error("Could not open the lidar estimate output file!");

        }

        // iterators
        StampedLidarPtrVector::iterator end(velodyne_messages.end());
        StampedLidarPtrVector::iterator it(velodyne_messages.begin());

        while (end != it) {

            // get the current lidar message
            StampedLidarPtr current_lidar = *it;

            // direct access
            Eigen::Vector3d pose(current_lidar->lidar_estimate.toVector());

            // get the yaw
            double yaw = pose[2];

            // update the position
            pose[0] += gps_origin[0];
            pose[1] += gps_origin[1];

            // save the estimate
            ofs << std::fixed << pose[0] << " " << pose[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << "\n";

            // go to the next message
            ++it;

        }

        // close the output file
        ofs.close();

    }

}

// save the curvature constraint edges
void GrabData::SaveCurvatureEdges(std::ofstream &os) {

    if (2 < messages.size()) {

        // iterators
        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator prev(messages.begin());
        StampedMessagePtrVector::iterator curr(prev + 1);
        StampedMessagePtrVector::iterator next(curr + 1);

        while (next != end) {

            // get the messages direct access
            StampedMessagePtr a(*prev);
            StampedMessagePtr b(*curr);
            StampedMessagePtr c(*next);

            // only sequencial ids
            if (c->id == b->id + 1 && b->id == a->id + 1) {

                // save the current hyper edge
                os << "CURVATURE_CONSTRAINT " << a->id << " " << b->id << " " << c->id << " " << "\n";

            }

            // go to the next messages
            prev = curr;
            curr = next;
            ++next;

        }

    }

}

// build Eigen homogeneous matrix from g2o SE2
Eigen::Matrix4f GrabData::BuildMatrixFromSE2(const g2o::SE2 &transform) {

    // the rotation around the z axis
    Eigen::Transform<float, 3, Eigen::Affine> yaw(Eigen::AngleAxis<float>(transform.rotation().angle(), Eigen::Matrix<float, 3, 1>::UnitZ()));

    // the translation
    Eigen::Translation<float, 3> translation(Eigen::Matrix<float, 3, 1>(transform.toVector()[0], transform.toVector()[1], 0.0f));

    // return the homogeneous matrix
    return (translation * yaw).matrix();

}

// build g2o SE2 from Eigen matrix
g2o::SE2 GrabData::BuildSE2FromMatrix(const Eigen::Matrix4f &matrix) {

    return g2o::SE2(matrix(0, 3), matrix(1, 3), std::atan2(matrix(1, 0), matrix(0, 0)));

}

// the main sync method
// PUBLIC METHODS

// the main process
// it reads the entire log file and builds the hypergraph
bool GrabData::ParseLogFile(const std::string &input_filename) {

    // the input file stream
    std::ifstream logfile(input_filename);

    if (!logfile.is_open()) {

        std::cerr << "Unable to open the input file: " << input_filename << "\n";

        return false;

    }

    // status report
    std::cout << "Start reading the input logfile...\n";

    // the input string stream
    std::stringstream current_line;

    // a general counter
    unsigned counter = 0;

    // how many velodyne messages
    unsigned vldn_msgs = 0;

    // the msg id
    // we must start with 2
    unsigned msg_id = 2;

	unsigned vel_scans = 0 == MINIMUM_VEL_SCANS ? std::numeric_limits<unsigned>::max() : MINIMUM_VEL_SCANS;

    // how many messages
    unsigned percent = vel_scans * 0.1;

    // parse the carmen log file and extract all the desired infos
    while(-1 != StringHelper::ReadLine(logfile, current_line) && vel_scans > vldn_msgs) {

        // the input tag
        std::string tag;

        // the first word is the message name
        current_line >> tag;

        // the new message
        StampedMessagePtr msg = nullptr;

        // take care of each message
        if ("XSENS_QUAT__" == tag) {

            // build a new XSENS orientation message
            msg = new StampedXSENS(msg_id);

        } else if ("ROBOTVELOCITY_ACK" == tag) {

            // build a new odometry message
            msg = new StampedOdometry(msg_id);

        } else if ("NMEAGGA" == tag) {

            // build a new GPS pose message
            msg = new StampedGPSPose(msg_id);

        } else if ("NMEAHDT" == tag) {

            // build a new GPS orientation message
            msg = new StampedGPSOrientation(msg_id);

        } else if (false) { //"LASER_LDMRS_NEW" == tag) {
	
            // build a new sick message
            msg = new StampedSICK(msg_id);

        } else if ("BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3" == tag) {

            // parse the Bumblebee stereo image message
            continue;

        } else if ("VELODYNE_PARTIAL_SCAN_IN_FILE" == tag || "VELODYNE_PARTIAL_SCAN" == tag) {

            // build a new velodyne message
            msg = new StampedVelodyne(msg_id);

            // increment the velodyne messages
            ++vldn_msgs;

        } else {

            // go to the next iteration
            continue;

        }

        // parse the stringstream based on the derived class implementation
        if(msg->FromCarmenLog(current_line)) {

            // update the msg id
            ++msg_id;

            // save it to the msg vector
            raw_messages.push_back(msg);

            // increment the counter
            ++counter;

            if (percent < counter) {

            	// reset the counter value
            	counter = 0;

            	// status report
            	std::cout << "#" << std::flush;

            }

        } else {

            // remove it
            delete msg;

        }

    }

    // close the streams
    logfile.close();

    std::cout << std::endl;

    // success!
    return true;

}

// parser
void GrabData::BuildHyperGraph() {

    if (!raw_messages.empty()) {

        // sort all the messages by the timestamp
        std::sort(raw_messages.begin(), raw_messages.end(), StampedMessage::compare);

        // separate all messages after the sorting process
        SeparateMessages();

        // gps filtering process
        GPSFiltering();

        // build the gps measures
        // also moves the global position to local considering the first position as origin
        BuildGPSMeasures();

        // build odometry measures
        BuildOdometryMeasures();

        // build the initial estimates
        BuildOdometryEstimates();

        // build the velodyne odometry measures
        BuildLidarOdometryMeasuresWithThreads(velodyne_messages);

        // built the velodyne odometry estimates
        BuildLidarOdometryEstimates(velodyne_messages);

        // build the velodyne loop closure measures
        BuildLidarLoopClosureMeasures(velodyne_messages);

        // build the sick odometry measures
        // BuildLidarOdometryMeasuresWithThreads(sick_messages);

        // build the lidar estimates
        // BuildVelodyneLidarOdometryEstimates();

        // build the sick loop closure measMatrix4ures
        // BuildLidarLoopClosureMeasures(sick_messages);

        // build the camera icp measures
        // BuildBumblebeeMeasuresWithThreads(bumblebee_messages);

    }

}

// save the hyper graph to the output file
void GrabData::SaveHyperGraph(const std::string &output_filename) {

    if (5 < messages.size()) {

        // open the file
        std::ofstream out(output_filename, std::ofstream::out);

        if (out.is_open()) {

            // save all vertex
            // IT MUST BECOME FIRST
            // so the edges list can use the vertices ids
            SaveAllVertices(out);

            // save the odometry edges
            SaveOdometryEdges(out);

            // save the odometry estimate to external file
            SaveOdometryEstimates();

            // save the GPS edges
            SaveGPSEdges(out);

            // save the gps estimate to gps.txt file
            SaveGPSEstimates();

            // save the icp edges
            SaveICPEdges(out);

            // save the velodyne estimates
            SaveLidarEstimates("velodyne.txt", velodyne_messages);

            // save the curvature constraint edges
            // SaveCurvatureEdges(out);

        } else {

            throw std::runtime_error("Could not open the output file\n");

        }

        // close the output file
        out.close();

    }

}

// clear the entire object
void GrabData::Clear() {

    // discards the lidar messages pointers
    sick_messages.clear();
    velodyne_messages.clear();

    // discard the gps messages pointers
    gps_messages.clear();

    // discards the odometry messages pointers
    odometry_messages.clear();

    // clear the messages list
    messages.clear();

    // the messages vector needs to be deleted from memmory
    while (!raw_messages.empty()) {

        // get the last element
        StampedMessagePtr tmp = raw_messages.back();

        // remove the last element from the vector
        raw_messages.pop_back();

        // delete the message from memmory
        delete tmp;

    }

}
