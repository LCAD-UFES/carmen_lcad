#include <GrabData.hpp>
#include <StampedMessageType.hpp>
#include <SimpleLidarSegmentation.hpp>

#include <list>
#include <cmath>
#include <thread>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <limits>
#include <unistd.h>

#include <viso_stereo.h>
#include <png++/png.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <sstream>

using namespace hyper;

// the main constructor
GrabData::GrabData() :
    raw_messages(0),
    messages(0),
    gps_messages(0),
    xsens_messages(0),
    velodyne_messages(0),
    used_velodyne(0),
    sick_messages(0),
    point_cloud_lidar_messages(nullptr),
    odometry_messages(0),
    bumblebee_messages(0),
    used_frames(0),
    gps_origin(0.0, 0.0),
    icp_start_index(0),
    icp_end_index(0),
    icp_mutex(),
    first_last_mutex(),
    error_increment_mutex(),
    icp_errors(0),
    dmax(std::numeric_limits<double>::max()),
    maximum_vel_scans(MAXIMUM_VEL_SCANS),
    loop_required_time(LOOP_REQUIRED_TIME),
    loop_required_distance(LOOP_REQUIRED_DISTANCE),
    icp_threads_pool_size(ICP_THREADS_POOL_SIZE),
    icp_thread_block_size(ICP_THREAD_BLOCK_SIZE),
    lidar_odometry_min_distance(LIDAR_ODOMETRY_MIN_DISTANCE),
    visual_odometry_min_distance(VISUAL_ODOMETRY_MIN_DISTANCE),
    icp_translation_confidence_factor(ICP_TRANSLATION_CONFIDENCE_FACTOR),
    save_accumulated_point_clouds(false),
    save_loop_closure_point_clouds(false),
    use_velodyne_odometry(true),
    use_sick_odometry(true),
    use_bumblebee_odometry(true),
    use_velodyne_loop(true),
    use_external_velodyne_loop(false),
    use_sick_loop(true),
    use_external_sick_loop(false),
    use_bumblebee_loop(true),
    use_fake_gps(false),
    use_gps_orientation(false),
    use_restricted_loops(false),
    grab_data_id(0),
    min_speed(MIN_SPEED),
    initial_guess_pose(),
    gps_error(0.0, 4.0, M_PI_2)
{
}


// the main constructor
GrabData::GrabData(unsigned _gid) :
    raw_messages(0),
    messages(0),
    gps_messages(0),
    xsens_messages(0),
    velodyne_messages(0),
    used_velodyne(0),
    sick_messages(0),
    point_cloud_lidar_messages(nullptr),
    odometry_messages(0),
    bumblebee_messages(0),
    used_frames(0),
    gps_origin(0.0, 0.0),
    icp_start_index(0),
    icp_end_index(0),
    icp_mutex(),
    first_last_mutex(),
    error_increment_mutex(),
    icp_errors(0),
    dmax(std::numeric_limits<double>::max()),
    maximum_vel_scans(MAXIMUM_VEL_SCANS),
    loop_required_time(LOOP_REQUIRED_TIME),
    loop_required_distance(LOOP_REQUIRED_DISTANCE),
    icp_threads_pool_size(ICP_THREADS_POOL_SIZE),
    icp_thread_block_size(ICP_THREAD_BLOCK_SIZE),
    lidar_odometry_min_distance(LIDAR_ODOMETRY_MIN_DISTANCE),
    visual_odometry_min_distance(VISUAL_ODOMETRY_MIN_DISTANCE),
    icp_translation_confidence_factor(ICP_TRANSLATION_CONFIDENCE_FACTOR),
    save_accumulated_point_clouds(false),
    save_loop_closure_point_clouds(false),
    use_velodyne_odometry(true),
    use_sick_odometry(true),
    use_bumblebee_odometry(true),
    use_velodyne_loop(true),
    use_external_velodyne_loop(false),
    use_sick_loop(true),
    use_external_sick_loop(false),
    use_bumblebee_loop(true),
    use_fake_gps(false),
    use_gps_orientation(false),
    use_restricted_loops(false),
    grab_data_id(_gid),
    min_speed(MIN_SPEED),
    initial_guess_pose(),
    gps_error(0.0, 5.0, M_PI_2)
{
}


// the main destructor
GrabData::~GrabData()
{
    // clear all the messages
    Clear();
}

// the move constructor
GrabData::GrabData(GrabData &&gd) :
    raw_messages(std::move(gd.raw_messages)),
    messages(std::move(gd.messages)),
    gps_messages(std::move(gd.gps_messages)),
    xsens_messages(std::move(gd.xsens_messages)),
    velodyne_messages(std::move(gd.velodyne_messages)),
    used_velodyne(std::move(gd.used_velodyne)),
    sick_messages(std::move(gd.sick_messages)),
    point_cloud_lidar_messages(nullptr),
    odometry_messages(std::move(gd.odometry_messages)),
    bumblebee_messages(std::move(gd.bumblebee_messages)),
    used_frames(std::move(gd.used_frames)),
    gps_origin(gd.gps_origin),
    icp_start_index(gd.icp_start_index),
    icp_end_index(gd.icp_end_index),
    icp_mutex(),
    first_last_mutex(),
    error_increment_mutex(),
    icp_errors(gd.icp_errors),
    dmax(gd.dmax),
    maximum_vel_scans(gd.maximum_vel_scans),
    loop_required_time(gd.loop_required_time),
    loop_required_distance(gd.loop_required_distance),
    icp_threads_pool_size(gd.icp_threads_pool_size),
    icp_thread_block_size(gd.icp_thread_block_size),
    lidar_odometry_min_distance(gd.lidar_odometry_min_distance),
    visual_odometry_min_distance(gd.visual_odometry_min_distance),
    icp_translation_confidence_factor(gd.icp_translation_confidence_factor),
    save_accumulated_point_clouds(gd.save_accumulated_point_clouds),
    save_loop_closure_point_clouds(gd.save_loop_closure_point_clouds),
    use_velodyne_odometry(gd.use_velodyne_odometry),
    use_sick_odometry(gd.use_sick_odometry),
    use_bumblebee_odometry(gd.use_bumblebee_odometry),
    use_velodyne_loop(gd.use_velodyne_loop),
    use_external_velodyne_loop(gd.use_velodyne_loop),
    use_sick_loop(gd.use_sick_loop),
    use_external_sick_loop(gd.use_sick_loop),
    use_bumblebee_loop(gd.use_bumblebee_loop),
    use_fake_gps(gd.use_fake_gps),
    use_gps_orientation(gd.use_gps_orientation),
    use_restricted_loops(gd.use_restricted_loops),
    grab_data_id(gd.grab_data_id),
    min_speed(gd.min_speed),
    initial_guess_pose(gd.initial_guess_pose),
    gps_error(gd.gps_error)
{
}


// PRIVATE METHODS

// separate the gps and velodyne
void GrabData::SeparateMessages()
{
    // clear the gps, sick and velodyne list
    messages.clear();
    gps_messages.clear();
    sick_messages.clear();
    velodyne_messages.clear();
    bumblebee_messages.clear();

    StampedMessagePtrVector::iterator it(raw_messages.begin());
    StampedMessagePtrVector::iterator end(raw_messages.end());

    // status report
    std::cout << "Separating all " << raw_messages.size() << " raw messages by their types" << std::endl;

    // the speed flag
    bool valid_velocity = false;

    // how many lidar odometry measurements with velocity equals to zero
    int max_zeros = 2;

    // the odometry messages with invalid speed counter
    int zeros = max_zeros;

    // find the first valid odometry value
    while (end != it)
    {
        StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(*it);

        if (nullptr != odom && min_speed < std::fabs(odom->v))
        {
            break;
        }

        ++it;
    }

    while (end != it)
    {
        StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(*it);

        if (nullptr != odom)
        {
            // does it have a valid velocity?
            valid_velocity = min_speed < std::fabs(odom->v);
            zeros = valid_velocity ? max_zeros : zeros - 1;
        }

        if (valid_velocity || 0 < zeros)
        {
            StampedMessagePtr msg = *it;

            StampedGPSPosePtr gps_pose = dynamic_cast<StampedGPSPosePtr>(msg);
            StampedXSENSPtr xsens = dynamic_cast<StampedXSENSPtr>(msg);
            StampedSICKPtr sick = dynamic_cast<StampedSICKPtr>(msg);
            StampedVelodynePtr velodyne = dynamic_cast<StampedVelodynePtr>(msg);
            StampedBumblebeePtr bumblebee = dynamic_cast<StampedBumblebeePtr>(msg);

            if (nullptr != odom)
            {
                odometry_messages.push_back(odom);
                zeros = valid_velocity ? max_zeros : zeros - 1;
            }
            else if (nullptr != gps_pose)
            {
                gps_messages.push_back(gps_pose);
            }
            else if (nullptr != xsens)
            {
                xsens_messages.push_back(xsens);
            }
            else if (nullptr != sick)
            {
                sick_messages.push_back(static_cast<StampedLidarPtr>(sick));
            }
            else if (nullptr != velodyne)
            {
                velodyne_messages.push_back(static_cast<StampedLidarPtr>(velodyne));
            }
            else if (nullptr != bumblebee)
            {
                bumblebee_messages.push_back(bumblebee);
            }

            // save to the the general messages
            messages.push_back(msg);
        }
        ++it;
    }

    // status report
    std::cout << "The messages were separated" << std::endl;

    std::cout << "GPS pose messages: " << gps_messages.size() << std::endl;
    std::cout << "XSENS messages: " << xsens_messages.size() << std::endl;
    std::cout << "SICK messages: " << sick_messages.size() << std::endl;
    std::cout << "Velodyne messages: " << velodyne_messages.size() << std::endl;
    std::cout << "Odometry messages: " << odometry_messages.size() << std::endl;
    std::cout << "Bumblebee messages: " << bumblebee_messages.size() << std::endl;
    std::cout << "All valid messages: " << messages.size() << std::endl;
}


// get the gps estimation
g2o::SE2 GrabData::GetNearestGPSMeasure(
            StampedMessagePtrVector::iterator it,
            StampedMessagePtrVector::iterator end,
            int adv,
            double timestamp,
            double &dt)
{
    g2o::SE2 gps_measurement(0.0, 0.0, 0.0);
    dt = dmax;

    // move the main iterator
    std::advance(it, adv);

    // iterate to the left messages
    while (end != it)
    {
        StampedGPSPosePtr msg = dynamic_cast<StampedGPSPosePtr>(*it);

        if (nullptr != msg)
        {
            dt = std::fabs(timestamp - msg->timestamp);
            gps_measurement = msg->gps_measurement;
            break;
        }

        std::advance(it, adv);
    }

    return gps_measurement;
}


// get the gps estimation
g2o::SE2 GrabData::GetGPSMeasure(
            StampedMessagePtrVector::iterator begin,
            StampedMessagePtrVector::iterator gps,
            StampedMessagePtrVector::iterator end,
            double timestamp)
{

    double ldt = 0.0, rdt = 0.0;

    // iterate to the left
    g2o::SE2 left(GetNearestGPSMeasure(gps, begin, -1, timestamp, ldt));

    // iterate to the right
    g2o::SE2 right(GetNearestGPSMeasure(gps, end, 1, timestamp, rdt));

    // return the desired gps measurement
    return ldt < rdt ? left : right;
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
    while (end != it)
    {
        StampedGPSOrientationPtr gps_orientation = dynamic_cast<StampedGPSOrientationPtr>(*it);

        if (nullptr != gps_orientation)
        {
            // get the heading info
            h = mrpt::math::wrapToPi<double>(gps_orientation->yaw);

            dt = std::fabs(timestamp - gps_orientation->timestamp);

            break;
        }

        std::advance(it, adv);
    }
}


// get the gps estimation
Eigen::Rotation2Dd GrabData::GetGPSOrientation(
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
Eigen::Vector2d GrabData::GetFirstGPSPosition()
{

    if (!gps_messages.empty())
    {
        // update the gps position
        return gps_messages.front()->gps_measurement.translation();
    }

    return Eigen::Vector2d(0.0, 0.0);
}


// build the gps measurements
// including the orientation from xsens sensor or the gps itself
void GrabData::BuildGPSMeasures()
{
    if (5 < messages.size())
    {
        std::cout << "Start building the GPS measurements and estimates\n";

        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator begin(messages.begin());
        StampedMessagePtrVector::iterator prev(messages.end());
        StampedMessagePtrVector::iterator curr(messages.begin());
        StampedMessagePtrVector::iterator next(messages.begin());
        ++next;

        gps_origin = GetFirstGPSPosition();
        std::cout << "GPS ORIGIN: " << gps_origin.transpose() << std::endl;

        while (end != curr)
        {
            StampedGPSPosePtr gps_pose = dynamic_cast<StampedGPSPosePtr>(*curr);

            if (nullptr != gps_pose)
            {
                gps_pose->gps_measurement.setTranslation(gps_pose->gps_measurement.translation() - gps_origin);
                gps_pose->gps_measurement.setRotation(Eigen::Rotation2Dd(GetGPSOrientation(begin, curr, end, gps_pose->timestamp)));
            }
            ++curr;
        }
        std::cout << "GPS measurements and estimates were built successfully\n";
    }
}

// including the orientation from computed angle
void GrabData::BuildFakeGPSMeasures()
{
    if (5 < messages.size())
    {
        std::cout << "Start building the fake GPS measurements and estimates\n";

        StampedGPSPosePtrVector::iterator end(gps_messages.end());
        StampedGPSPosePtrVector::iterator prev(gps_messages.end());
        StampedGPSPosePtrVector::iterator curr(gps_messages.begin());
        StampedGPSPosePtrVector::iterator next(gps_messages.begin() + 1);

        StampedGPSPosePtr prev_gps, curr_gps, next_gps;
        Eigen::Vector2d diff;

        while (end != next)
        {
            curr_gps = *curr;
            next_gps = *next;
            prev_gps = prev != end ? *prev : *curr;

            diff = next_gps->gps_measurement.translation() - prev_gps->gps_measurement.translation();
            curr_gps->gps_measurement.setRotation(Eigen::Rotation2Dd(std::atan2(diff[1], diff[0])));

            prev = curr;
            curr = next;
            ++next;
        }

        curr_gps->gps_measurement.setRotation(prev_gps->gps_measurement.rotation());

        curr = gps_messages.begin();

        gps_origin = GetFirstGPSPosition();

        while(end != curr)
        {
            curr_gps = *curr;
            curr_gps->gps_measurement.setTranslation(curr_gps->gps_measurement.translation() - gps_origin);
            ++curr;
        }

        std::cout << "GPS measurements and estimates were built successfully\n";
    }
}


// build the odometry measurements
void GrabData::BuildOdometryMeasures()
{

    if (5 < messages.size() && !odometry_messages.empty())
    {
        // status reporting
        std::cout << "Start building the odometry measurements\n";

        // the current commands
        double v = 0.0, phi = 0.0;
        double raw_v = 0.0, raw_phi = 0.0;

        // iterators
        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator begin(messages.begin());
        StampedMessagePtrVector::iterator it(begin);

        // find the first odometry message position
        while (end != it)
        {
            StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(*it);

            if (nullptr != odom)
            {
                // get the initial odometry values
                v = odom->v;
                phi = odom->phi;
                raw_v = odom->raw_v;
                phi = odom->raw_phi;

                // quit the loop
                break;
            }

            ++it;
        }

        if (end != it)
        {

            if (begin != it)
            {
                // iterators
                StampedMessagePtrVector::reverse_iterator rend(messages.rend());
                StampedMessagePtrVector::reverse_iterator rit(it + 1);
                StampedMessagePtrVector::reverse_iterator prev(it);

                while (rend != prev)
                {
                   // get the internal message pointers
                    StampedMessagePtr current_msg = *rit;
                    StampedMessagePtr prev_msg = *prev;

                    double dt = current_msg->timestamp - prev_msg->timestamp;
                    if (dt >= 0 && dt < 600)
                    {
                        // update the measurement
                        prev_msg->odom_measurement = VehicleModel::GetOdometryMeasure(v, phi, dt);
                        prev_msg->raw_measurement = VehicleModel::GetOdometryMeasure(raw_v, raw_phi, dt);
                    }

                    rit = prev;
                    ++prev;
                }
            }

            // get the next iterator
            StampedMessagePtrVector::iterator next(it + 1);

            while (end != next)
            {
                // get the internal pointers
                StampedMessagePtr current_msg = *it;
                StampedMessagePtr next_msg = *next;

                StampedOdometryPtr odom = dynamic_cast<StampedOdometryPtr>(current_msg);
                if (nullptr != odom)
                {
                    // update the odometry values
                    v = odom->v;
                    phi = odom->phi;
                    raw_v = odom->raw_v;
                    raw_phi = odom->raw_phi;
                }
                else
                {
                    // save the v value inside the lidar and bumblebee messages
                    StampedLidarPtr lidar = dynamic_cast<StampedLidarPtr>(current_msg);
                    StampedBumblebeePtr bumblebee = dynamic_cast<StampedBumblebeePtr>(current_msg);

                    if (nullptr != lidar)
                    {
                        lidar->speed = v;
                    }
                    else if (nullptr != bumblebee)
                    {
                        bumblebee->speed = v;
                    }
                }

                double dt = next_msg->timestamp - current_msg->timestamp;
                if (dt >= 0 && dt < 1200)
                {
                    // get the current odometry measurement
                    current_msg->odom_measurement = VehicleModel::GetOdometryMeasure(v, phi, dt);
                    current_msg->raw_measurement = VehicleModel::GetOdometryMeasure(raw_v, raw_phi, dt);
                }

                it = next;
                ++next;
            }
        }

        // status reporting
        std::cout << "Odometry measurements built\n";
    }
}


// build the initial estimates
void GrabData::
BuildOdometryEstimates(bool gps_based, bool gps_orientation)
{
    if (5 < messages.size())
    {
        // status reporting
        std::cout << "Start building the general estimates\n";

        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator it(messages.begin());

        double distance = 0.0f;

        StampedGPSPosePtr gps = nullptr;

        // forward to the first gps pose
        if (gps_orientation) {
            while (end != it)
            {
                gps = dynamic_cast<StampedGPSPosePtr>(*it);

                if (nullptr != gps)
                {
                    // set the first gps pose estimate
                    gps->est = gps->gps_measurement;
                    gps->raw_est = gps->gps_measurement;
                    break;
                }

                ++it;
            }
            
            if (end != it && messages.begin() != it)
            {
                // iterate to the left
                StampedMessagePtrVector::reverse_iterator rend(messages.rend());
                StampedMessagePtrVector::reverse_iterator rb(it + 1);
                StampedMessagePtrVector::reverse_iterator ra(it);

                // update the previous estimates
                // to the left
                while (rend != ra)
                {
                    StampedMessagePtr current = *rb;
                    StampedMessagePtr previous = *ra;

                    previous->est = current->est * (previous->odom_measurement.inverse());
                    previous->raw_est = current->raw_est * (previous->raw_measurement.inverse());

                    rb = ra;
                    ++ra;
                }
            }
            else
            {
                it = messages.begin();
            }
        }

        StampedMessagePtrVector::iterator a(it);
        StampedMessagePtrVector::iterator b(it + 1);

        while (end != b)
        {
            StampedMessagePtr current = *a;
            StampedMessagePtr next = *b;

            next->est = current->est * (current->odom_measurement);
            next->raw_est = current->raw_est * (current->raw_measurement);

            distance += current->odom_measurement.translation().norm();

            if (gps_based)
            {
                StampedGPSPosePtr m_gps = dynamic_cast<StampedGPSPosePtr>(next);
                if (nullptr != m_gps && 3.0f < distance)
                {
                    next->est = m_gps->gps_measurement;
                    next->raw_est = m_gps->gps_measurement;
                    distance = 0.0f;
                }
            }

            a = b;
            ++b;
        }

        // status report
        std::cout << "General estimates done!\n";
    }
    else
    {
        throw std::runtime_error("Odometry messages are missing!");
    }
}


void GrabData::BuildHackedOdometryEstimates()
{
    if (5 < messages.size())
    {
        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator curr(messages.begin());
        StampedMessagePtrVector::iterator next(messages.begin() + 1);

        std::cout << "The initial guess: " << initial_guess_pose.toVector().transpose() << std::endl;

        // error: no match for call to ‘(g2o::SE2) (hyper::StampedMessage*&)’
        // (*curr)->est = initial_guess_pose
        (*curr)->raw_est = initial_guess_pose;
        (*curr)->est = initial_guess_pose;

        while (end != next)
        {
            StampedMessagePtr c = *curr;
            StampedMessagePtr n = *next;

            n->est = c->est * (c->odom_measurement);
            n->raw_est = c->raw_est * (c->raw_measurement);

            curr = next++;
        }
    }
}


// build an icp measurement
bool GrabData::BuildLidarOdometryMeasure(
        GeneralizedICP &gicp,
        VoxelGridFilter &grid_filtering,
        double cf,
        const g2o::SE2 &odom,
        PointCloudHSV::Ptr source_cloud,
        PointCloudHSV::Ptr target_cloud,
        g2o::SE2 &icp_measurement )
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
    // gicp.align(result);
    gicp.align(result, BuildEigenMatrixFromSE2(odom));

    if (gicp.hasConverged())
    {
        // get the final transformation
        Eigen::Matrix4f icp_guess(gicp.getFinalTransformation());

        // get the desired transformation
        icp_measurement = GetSE2FromEigenMatrix(icp_guess);

        // the distance
        double translation_difference = (odom.translation() - icp_measurement.translation()).norm();

        // try to validate the current transformation
        bool valid_transformation = (0 != cf * icp_guess(0, 3)) && icp_translation_confidence_factor > translation_difference;

        // the movement and the cf value should have the same sign
        if (valid_transformation)
        {
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
        }
        else
        {
            std::cerr << "Error: " << translation_difference << " is greater than " << icp_translation_confidence_factor;
            std::cerr << " or " << cf << " is lesser than zero and icp is greater, let's see icp matrix: \n" << icp_guess << std::endl;
        }
    }
    else
    {
        std::cerr << "Error: It hasn't converged!" << std::endl;
    }

    // invalid
    return false;
}


// build an icp measurement
bool GrabData::BuildLidarLoopMeasure(
        GeneralizedICP &gicp,
        double cf,
        double orientation,
        PointCloudHSV::Ptr source_cloud,
        PointCloudHSV::Ptr target_cloud,
        g2o::SE2 &loop_measurement)
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
    gicp.align(result, BuildEigenMatrixFromSE2(g2o::SE2(0.0, 0.0, orientation)));

    if (gicp.hasConverged())
    {
        // get the desired transformation
        loop_measurement = GetSE2FromEigenMatrix(gicp.getFinalTransformation());
        Eigen::Vector3d v = loop_measurement.toVector();
        if (min_speed < std::fabs(v[0]) || min_speed < std::fabs(v[1]) || min_speed < std::fabs(v[2]))
        {
            return true;
        }
    }

    // invalid
    return false;
}


// get the next lidar block
bool GrabData::GetNextLidarBlock(unsigned &first_index, unsigned &last_index)
{
    // the status
    bool status = false;

    // lock the mutex
    icp_mutex.lock();

    // get the point cloud vector size
    const unsigned upper_limit = point_cloud_lidar_messages->size() - 1;

    // verify the start index
    if (upper_limit > icp_start_index)
    {
        // set the first index
        first_index = icp_start_index;

        // get the end index
        icp_end_index = icp_start_index + icp_thread_block_size;

        // verify the end index
        if (upper_limit < icp_end_index)
        {
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
bool GrabData::GetNextICPIterators(StampedLidarPtrVector::iterator &begin, StampedLidarPtrVector::iterator &end)
{
    // the status
    bool status = true;

    // lock the mutex
    icp_mutex.lock();

    // get the point cloud vector size
    const unsigned upper_limit = point_cloud_lidar_messages->size() - 1;

    // verify the start index
    if (upper_limit > icp_start_index)
    {
        // set the begin iterator
        begin = point_cloud_lidar_messages->begin() + icp_start_index;

        // get the end index
        icp_end_index = icp_start_index + icp_thread_block_size;

        // verify the end index
        if (upper_limit < icp_end_index)
        {
            // set the new end index
            icp_end_index = upper_limit;
        }

        // set the end iterator
        end = point_cloud_lidar_messages->begin() + icp_end_index;

        // set the start index to the next thread
        icp_start_index = icp_end_index;
    }
    else
    {
        status = false;
    }

    // unlock the mutex
    icp_mutex.unlock();

    return status;
}
PointCloudHSV::Ptr GrabData::GetFilteredPointCloud(SimpleLidarSegmentation &segm, VoxelGridFilter &grid_filtering, StampedLidarPtr lidar)
{
    PointCloudHSV::Ptr filtered_cloud(new PointCloudHSV());
    PointCloudHSV::Ptr raw_cloud(new PointCloudHSV());
    
    lidar->LoadPointCloud(*raw_cloud);

    lidar->RemoveUndesiredPoints(segm, *raw_cloud);

    grid_filtering.setInputCloud(raw_cloud);
    grid_filtering.filter(*filtered_cloud);

    raw_cloud->clear();

    return filtered_cloud;
}


// the main icp measurement method, multithreaded version
void GrabData::BuildLidarMeasuresMT()
{
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

    SimpleLidarSegmentation segm;

    // the lidar frequency times the base distance
    float lfd = lidar_odometry_min_distance;

    // the base path
    std::string path("/dados/tmp/lgm/");

    bool is_sick = (point_cloud_lidar_messages == &sick_messages);

    if (is_sick)
    {
        // set the sick base path
        path += "sick/";

        // set the origin
        // lgm.SetSickOrigin();

        // set the sick frequency
        lfd *= 25.0;
    }
    else
    {
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

    // get the the range iterators
    while (GetNextLidarBlock(current_index, last_index))
    {
        // save the first index to mutex usage
       unsigned first_index = current_index;

        // get the current message pointer
        StampedLidarPtr current = *(begin + current_index);

        // the current cloud
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr current_cloud(GetFilteredPointCloud(segm, grid_filtering, current));

        // the main iterator
        while (current_index < last_index)
        {
            // compute the desired StampedMessagePtr
            unsigned next_index = current_index + unsigned(std::max(1.0, std::min(10.0, lfd / std::fabs(current->speed))));

            if (next_index <= last_index)
            {
                // get the next
                StampedLidarPtr next = *(begin + next_index);

                // the next cloud
                pcl::PointCloud<pcl::PointXYZHSV>::Ptr next_cloud(GetFilteredPointCloud(segm, grid_filtering, next));

                double dt = next->timestamp - current->timestamp;

                if (dt > 0 && dt < 600)
                {
                    // get the factor
                    double cf = double(int(current->speed * (next->timestamp - current->timestamp) * 100.0)) * 0.02;

                    if (0.0 != cf)
                    {
                        // the odometry guess
                        g2o::SE2 odom(current->est.inverse() * next->est);

                        if (BuildLidarOdometryMeasure(gicp, grid_filtering, cf, odom, current_cloud, next_cloud, current->seq_measurement))
                        {
                            // set the base id
                            current->seq_id = next->id;

                            if (save_accumulated_point_clouds)
                            {
                                if (first_index == current_index || last_index == current_index)
                                {
                                    std::lock_guard<std::mutex> lock(first_last_mutex);
                                    StampedLidar::SavePointCloud(path, current_index, *current_cloud);

                                }
                                else
                                {
                                    StampedLidar::SavePointCloud(path, current_index, *current_cloud);
                                }
                            }
                        }
                        else
                        {
                            error_increment_mutex.lock();
                            ++icp_errors;
                            error_increment_mutex.unlock();
                        }
                    }
                }

                // update the current pointer
                current = next;

                // update the counter
                ++counter;

                if (0 == counter % percent)
                {
                    std::cout << counter / percent << "%\n" << std::flush;
                }
            }

           current_index = next_index;
        }
    }
}


// build sequential and loop restriction ICP measurements
void GrabData::BuildLidarOdometryMeasuresWithThreads(StampedLidarPtrVector &lidar_messages)
{

    if (1 < lidar_messages.size())
    {
        // set the point cloud lidar_messages to be used inside the lidar
        point_cloud_lidar_messages = &lidar_messages;

        // reset the index
        icp_start_index = 0;

        // the thread pool
        std::vector<std::thread> pool(0);

        // status report
        std::cout << "Start building the Lidar measurements of " << lidar_messages.size() << " point clouds\n";

        // create the threads using the defined pool size
        for (unsigned i = 0; i < icp_threads_pool_size; ++i)
        {
            pool.push_back(std::thread(&GrabData::BuildLidarMeasuresMT, this));
        }

        // wait all threads
        for (unsigned i = 0; i < icp_threads_pool_size; ++i)
        {
            pool[i].join();
        }

        std::cout << "\nLidar odometry errors: " << icp_errors << std::endl;

        // status reporting
        std::cout << "Lidar measurements done!" << std::endl;
    }
}


// remove the unused lidar messages
void GrabData::LidarMessagesFiltering(StampedLidarPtrVector &lidar_messages, StampedLidarPtrVector &used_lidar)
{
    // iterators
    StampedLidarPtrVector::iterator end(lidar_messages.end());
    StampedLidarPtrVector::iterator it(lidar_messages.begin());

    StampedMessagePtr last { raw_messages.back() };
    const unsigned last_index = last->id + 1;

    while (end != it)
    {
        StampedLidarPtr lidar = *it;

        if (last_index > lidar->seq_id)
        {
            used_lidar.push_back(lidar);
        }

        ++it;
    }
}


// build the gps sync lidar estimates
void GrabData::BuildLidarOdometryGPSEstimates()
{
    if (2 < messages.size())
    {
        StampedMessagePtrVector::iterator first(messages.begin());
        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator it(first);

        while (end != it)
        {
            StampedMessagePtr current = *it;
            StampedLidarPtr lidar = dynamic_cast<StampedLidarPtr>(current);

            if (nullptr != lidar)
            {
                lidar->gps_sync_estimate = GetGPSMeasure(first, it, end, current->timestamp);
            }

            ++it;
        }
    }
}


// build the lidar odometry estimates, we should call this method after the BuildOdometryEstimates
void GrabData::BuildRawLidarOdometryEstimates(StampedLidarPtrVector &lidar_messages, StampedLidarPtrVector &used_lidar)
{

    if (2 < lidar_messages.size())
    {
        // the last valid index
        StampedMessagePtr last { raw_messages.back() };
        const unsigned last_index = last->id + 1;

        // iterators
        StampedLidarPtrVector::iterator end(lidar_messages.end());
        StampedLidarPtrVector::iterator current(lidar_messages.begin());
        StampedLidarPtrVector::iterator next(current + 1);

        // the first lidar estimate is the general odometry estimate
        (*current)->lidar_estimate = (*current)->est;

        used_lidar.push_back((*current));

        while (end != next)
        {
            // get the current lidar
            StampedLidarPtr current_lidar = *current;
            StampedLidarPtr next_lidar = *next;

            if (last_index > current_lidar->seq_id)
            {
                StampedLidarPtr previous_lidar = current_lidar;

                // find the used one
                while(next_lidar->id != current_lidar->seq_id)
                {
                    // hack
                    next_lidar->lidar_estimate = previous_lidar->lidar_estimate * (previous_lidar->est.inverse() * next_lidar->est);
                    previous_lidar = next_lidar;
                    ++next;
                    next_lidar = *next;
                }
                // set the next estimate
                next_lidar->lidar_estimate = current_lidar->lidar_estimate * current_lidar->seq_measurement;

                // push to the used lidar vector
                used_lidar.push_back(next_lidar);
            }
            else
            {
                // hack
                next_lidar->lidar_estimate = current_lidar->lidar_estimate * (current_lidar->est.inverse() * next_lidar->est);
            }

            current = next;
           ++next;
        }
    }
    else
    {
        std::cout << "Empty lidar list!" << std::endl;
    }
}


// build the visual odometry estimates, we should call this method after the BuildOdometryEstimates
void GrabData::BuildVisualOdometryEstimates()
{

    if (2 < bumblebee_messages.size())
    {
        // the last valid index
        StampedMessagePtr last { raw_messages.back() };
        const unsigned last_index = last->id + 1;

        // iterators
        StampedBumblebeePtrVector::iterator end(bumblebee_messages.end());
        StampedBumblebeePtrVector::iterator current(bumblebee_messages.begin());
        StampedBumblebeePtrVector::iterator next(current + 1);

        // the first lidar estimate is the general odometry estimate
        (*current)->visual_estimate = (*current)->est;

        while (end != next)
        {
            // get the current lidar
            StampedBumblebeePtr current_msg = *current;
            StampedBumblebeePtr next_msg = *next;

            if (last_index > current_msg->seq_id)
            {
                StampedBumblebeePtr previous_msg = current_msg;

                // find the used one
                while(next_msg->id != current_msg->seq_id)
                {
                    // hack
                    next_msg->visual_estimate = previous_msg->visual_estimate * (previous_msg->est.inverse() * next_msg->est);
                   previous_msg = next_msg;
                    ++next;
                    next_msg = *next;
                }
                // set the next estimate
                next_msg->visual_estimate = current_msg->visual_estimate * current_msg->seq_measurement;

                // push to the used frames vector
                used_frames.push_back(next_msg);
            }
            else
            {
                // hack
                next_msg->visual_estimate = current_msg->visual_estimate * (current_msg->est.inverse() * next_msg->est);
            }

            current = next;
           ++next;
        }
    }
}


// compute the loop closure measurement
void GrabData::BuildLidarLoopClosureMeasures(StampedLidarPtrVector &lidar_messages)
{
    if (!lidar_messages.empty())
    {
        // report
        std::cout << "Start building the loop closure measurements from " << lidar_messages.size() << " messages\n";

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

        SimpleLidarSegmentation segm;

        // iterators
        StampedLidarPtrVector::iterator end(lidar_messages.end());
        StampedLidarPtrVector::iterator it(lidar_messages.begin());

        unsigned lmsize = lidar_messages.size();
        unsigned counter = 0, match_counter = 0;
        unsigned percent = lmsize / 40;

        std::string path("/dados/tmp/loops/");

        bool is_sick = (point_cloud_lidar_messages == &sick_messages);

        if (is_sick)
        {
            path += "sick/";
        }
        else
        {
            path += "velodyne/";
        }

        while (end != it)
        {
            counter += 1;

            // std::cout << counter << " of " << lmsize << std::endl;
            if (0 == percent % counter) { std::cout << "-" << std::flush; }

            // get the current lidar message pointer
            StampedLidarPtr current = *it;

            // the loop iterator
            StampedLidarPtrVector::iterator loop(end);

            // the mininum distance found
            double min_dist = dmax;

            // the next message
            StampedLidarPtrVector::iterator next(it + 1);

            while (end != next)
            {
                // get the current loop message
                StampedLidarPtr lidar_loop = *next;

                // compute the current distance
                double distance = (current->gps_sync_estimate.translation() - lidar_loop->gps_sync_estimate.translation()).norm();

                // the time difference
                double dt = std::fabs(lidar_loop->timestamp - current->timestamp);

                if ((min_dist > distance) && (loop_required_time < dt) && (loop_required_distance > distance))
                {
                    min_dist = distance;
                    loop = next;
                }

                // go the next loop
                ++next;
            }

            if (end != loop)
            {
                double a(current->est.rotation().angle());
                double b((*loop)->est.rotation().angle());
                double ad = mrpt::math::angDistance<double>(a, b);

                if (!use_restricted_loops || M_PI_2 > ad)
                {
                    // found it
                    StampedLidarPtr lidar_loop = *loop;

                    // load the current de loop  cloud
                    pcl::PointCloud<pcl::PointXYZHSV>::Ptr current_cloud(GetFilteredPointCloud(segm, grid_filtering, current));
                    pcl::PointCloud<pcl::PointXYZHSV>::Ptr loop_cloud(GetFilteredPointCloud(segm, grid_filtering, lidar_loop));

                    if (BuildLidarLoopMeasure(gicp, min_dist * 2.0, ad, current_cloud, loop_cloud, current->loop_measurement))
                    {
                        current->loop_closure_id = lidar_loop->id;
                        if (save_loop_closure_point_clouds)
                        {
                            std::stringstream ss;

                            ss << path << match_counter << "_" << current->id << "_to_" << lidar_loop->id;
                            std::string curr_cloud_path(ss.str());
                            match_counter += 1;
                            ss = std::stringstream();
                            ss << path << match_counter << "_" << current->id << "_to_" << lidar_loop->id;
                            std::string next_cloud_path(ss.str());
                            match_counter += 1;

                            if (-1 == pcl::io::savePCDFile(curr_cloud_path, *current_cloud, true)) {
                                throw std::runtime_error("Could not save the current cloud");
                            }

                            if (-1 == pcl::io::savePCDFile(next_cloud_path, *loop_cloud, true)) {
                                throw std::runtime_error("Could not save the next cloud");
                            }
                        }
                    }
                }
            }

            ++it;
        }

        // report
        std::cout << "Lidar loop closure measurements done!\n";
    }
}

bool GrabData::BuildExternalLidarLoopClosureMeasures(
    StampedLidarPtrVector &internal_messages,
    StampedLidarPtrVector &external_messages,
    Eigen::Vector2d external_gps_origin,
    double external_loop_min_distance,
    double external_loop_required_distance)
{
    if (!internal_messages.empty() and !external_messages.empty())
    {
        // report
        std::cout << "Start building the loop closure measurements from internal messages with size: " << internal_messages.size() << " to external messages with size: " << external_messages.size() << std::endl;

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

        SimpleLidarSegmentation segm;

        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>());
        std::unordered_map<int, StampedLidarPtr> refs;

        cloud->clear();
        for (StampedLidarPtr lidar : external_messages)
        {
            const Eigen::Vector2d position(lidar->gps_sync_estimate.translation() + external_gps_origin);
            cloud->push_back(pcl::PointXY { (float) position[0], (float) position[1] });
            refs[cloud->size() - 1] = lidar;
        }

        std::vector<int> nearestId(1);
        std::vector<float> nearestDistance(1);

        kdtree.setInputCloud(cloud);

        Eigen::Vector2d last_loop_position(-1e16, -1e16);
        
        for (StampedLidarPtr internal : internal_messages)
        {
            const Eigen::Vector2d position(internal->gps_sync_estimate.translation() + gps_origin);

            if ((position - last_loop_position).norm() < external_loop_min_distance) 
            {
                continue;
            }

            pcl::PointXY xy { (float) position[0], (float) position[1] };

            if (0 < kdtree.nearestKSearch(xy, 1, nearestId, nearestDistance))
            {
                float nd = nearestDistance.front();

                if (external_loop_required_distance > nd)
                {
                    StampedLidarPtr external(refs[nearestId.front()]);

                    double a(internal->gps_sync_estimate.rotation().angle());
                    double b(external->gps_sync_estimate.rotation().angle());

                    double ad = mrpt::math::angDistance<double>(a, b);

                    if (!use_restricted_loops || M_PI_2 > ad)
                    {
                        pcl::PointCloud<pcl::PointXYZHSV>::Ptr current_cloud(GetFilteredPointCloud(segm, grid_filtering, internal));
                        pcl::PointCloud<pcl::PointXYZHSV>::Ptr loop_cloud(GetFilteredPointCloud(segm, grid_filtering, external));

                        // try the icp method
                        if (BuildLidarLoopMeasure(gicp, nd * 2.0, ad, current_cloud, loop_cloud, internal->external_loop_measurement))
                        {
                            last_loop_position = position;
                            internal->external_loop_closure_id = external->id;
                        }
                    }
                }
            }
        }

        return true;
    }
    return false;
}


// compute the bumblebee measurement
void GrabData::BuildVisualOdometryMeasures()
{
    std::cout << "Building the visual odometry measurements from " << bumblebee_messages.size() << " messages!" << std::endl;

    // the camera freq
    float bfd = visual_odometry_min_distance * 16.0;

    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    VisualOdometryStereo::parameters param;

    // focal length in pixels
    param.calib.f  = 0.764749 * 640;

    // principal point (u-coordinate) in pixels
    param.calib.cu = 0.505423 * 640;

    // principal point (v-coordinate) in pixels
    param.calib.cv = 0.493814 * 480;

    // baseline in meters
    param.base     = 0.24004;

    // init visual odometry
    VisualOdometryStereo viso(param);

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    Matrix pose = Matrix::eye(4);

    StampedBumblebeePtr last { bumblebee_messages.back() };
    const unsigned last_index = last->id + 1;
    
    unsigned curr_index = 0;

    while (last_index > curr_index)
    {
        // update the seq measurement
        StampedBumblebeePtr current_msg(bumblebee_messages[curr_index]);

        // compute the next image to be used
        unsigned next_index = curr_index + unsigned(std::max(1.0, std::min(10.0, bfd / std::fabs(current_msg->speed))));

        if (last_index > next_index)
        {
            // get the next message
            StampedBumblebeePtr next_msg(bumblebee_messages[next_index]);

            try
            {
                // the buffers
                std::vector<uint8_t> limg, rimg;

                // fill the buffers
                if (next_msg->ParseBumblebeeImage(limg, rimg))
                {
                    int32_t dims[3] = {int32_t(next_msg->GetWidth()), int32_t(next_msg->GetHeight()), int32_t(next_msg->GetWidth())};
                    if (viso.process(&limg[0], &rimg[0], dims))
                    {
                        double dt = next_msg->timestamp - current_msg->timestamp;
                        if (dt > 0 && dt < 600)
                        {
                            // get the current transformation matrix
                            Matrix transf = Matrix::inv(viso.getMotion());
                            // update the sequential id
                            current_msg->seq_id = next_msg->id;

                            // save the measur
                            current_msg->seq_measurement = GetSE2FromVisoMatrix(transf);
                        }
                    }
                    else
                    {
                        std::cout << "Error! Can't build the visual odometry measurement! Speed: " << current_msg->speed << std::endl;
                    }
                }
            }
            catch (...)
            {
                throw std::runtime_error("Can't read the bumblebee images!");
            }
        }

        // update the inde
        curr_index = next_index;
    }

    std::cout << "Visual odometry done!" << std::endl;
}


// save all vertices to the external file
void GrabData::SaveAllVertices(std::ofstream &os, bool global = false)
{
    StampedMessagePtrVector::iterator it = messages.begin();
    StampedMessagePtrVector::iterator end = messages.end();

    // save the gps origin
    if (global)
        os << "GPS_ORIGIN 0.0 0.0" << std::endl;
    else
        os << "GPS_ORIGIN " << std::fixed << gps_origin[0] << " " << gps_origin[1] << std::endl;

    // save the vertices ammount
    os << "VERTICES_QUANTITY " << messages.size() << std::endl;

    while (end != it)
    {
        // direct acccess
        StampedMessagePtr m = *it;

        Eigen::Vector2d position(m->est.translation());

        if (global)
            position += gps_origin;

        // write the estimate id and initial value
        os << "VERTEX " << grab_data_id << " " << m->id << " " << std::fixed << position[0] << " " << position[1] << " " << m->est[2] << " " << m->timestamp << " " << int(m->GetType()) << std::endl;

        ++it;
    }
}


// save the odometry edges
void GrabData::SaveOdometryEdges(std::ofstream &os)
{
    if (!messages.empty())
    {
        StampedMessagePtrVector::iterator end = messages.end();
        StampedMessagePtrVector::iterator current = messages.begin();
        StampedMessagePtrVector::iterator next = messages.begin();
        ++next;

        // get the first speed
        StampedOdometryPtr odom = odometry_messages[0];
        double rv = odom->raw_v;
        double rphi = odom->raw_phi;

        std::string _s(" ");

        while (end != next)
        {
            // direct acccess
            StampedMessagePtr a = *current;
            StampedMessagePtr b = *next;

            // direct acces
            const g2o::SE2 &measurement(a->odom_measurement);

            double dx = measurement[0];
            double dy = measurement[1];
            double yaw = measurement[2];
            double dt = b->timestamp - a->timestamp;

            if (dt < 0 || dt > 600)
                continue;

            // write the odometry measurements
            os << "ODOM_EDGE " << a->id << _s << b->id << _s << std::fixed << dx << _s << dy << _s << yaw << _s << rv << _s << rphi << _s << dt << std::endl;

            odom = dynamic_cast<StampedOdometryPtr>(a);

            if (nullptr != odom)
            {
                // update the speed and steering angle
                rv = odom->raw_v;
                rphi = odom->raw_phi;
            }

           current = next;
            ++next;
        }
    }
}


// save the odometry estimates
void GrabData::SaveOdometryEstimates(const std::string &output_filename, bool raw_version)
{
    if (!messages.empty())
    {
        // open the odometry output
        std::ofstream ofs(output_filename, std::ofstream::out);

        if (!ofs.good())
        {
            throw std::runtime_error("Could not open the odometry estimate output file!");
        }

        StampedMessagePtrVector::iterator end(messages.end());
        StampedMessagePtrVector::iterator it(messages.begin());

        while (end != it)
        {
            StampedMessagePtr msg { *it };

            Eigen::Vector3d pose = raw_version ? msg->raw_est.toVector() : msg->est.toVector();

            // add the gps origin
            // pose[0] += gps_origin[0];
            // pose[1] += gps_origin[1];

            double yaw = pose[2];

            // save the odometry estimated pose
            ofs << std::fixed << pose[0] << " " << pose[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << " " << msg->timestamp << std::endl;

            ++it;
        }

        // close it
        ofs.close();
    }
}


// save the gps edges
void GrabData::SaveGPSEdges(std::ofstream &os, bool global = false)
{
    std::cout << "Saving all GPS edges..." << std::endl;

    StampedGPSPosePtrVector::iterator end = gps_messages.end();
    StampedGPSPosePtrVector::iterator current = gps_messages.begin();

    while (end != current)
    {
        StampedGPSPosePtr gps = *current;

        // measurement direct access
        Eigen::Vector2d position(gps->gps_measurement.translation());

        if (global)
            position += gps_origin;

        // write the gps mesaure
        os << "GPS_EDGE " << gps->id << " " << std::fixed << position[0] << " " << position[1] << " " << gps->gps_measurement[2] << " " << gps->gps_std << std::endl;

        ++current;
    }

    std::cout << "GPS edges saved!" << std::endl;
}


// save the gps edges
void GrabData::SaveXSENSEdges(std::ofstream &os)
{
    StampedXSENSPtrVector::iterator end(xsens_messages.end());
    StampedXSENSPtrVector::iterator current(xsens_messages.begin());

    while (end != current)
    {
        StampedXSENSPtr xsens = *current;

        // write the gps mesaure
        os << "XSENS_EDGE " << xsens->id << " " << std::fixed << xsens->yaw << std::endl;

        ++current;
    }
}


void GrabData::SaveAllGPSEstimates(std::string filename, bool original = false)
{
    if (1 < raw_messages.size())
    {
        std::ofstream ofs(filename, std::ofstream::out);
        if (!ofs.good())
        {
            throw std::runtime_error("Could not open the raw gps estimates output file!");
        }

        for (StampedMessagePtr r : raw_messages)
        {
            StampedGPSPosePtr gps = dynamic_cast<StampedGPSPosePtr>(r);
            if (nullptr != gps)
            {
                g2o::SE2 &gps_pose = StampedGPSPose::gps_pose_delays[gps->gps_id].first;

                g2o::SE2 gps_measurement = original ? gps->gps_measurement * gps_pose : gps->gps_measurement;
                
                double yaw = gps_measurement[2];
                
                Eigen::Vector2d position(gps_measurement.translation());
            
                ofs << std::fixed << position[0] << " " << position[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << " " << gps->timestamp << std::endl;
            }
        }

        ofs.close();
    }
}


// save the gps edges
void GrabData::SaveGPSEstimates(std::string filename, bool original = false)
{
    if (1 < gps_messages.size())
    {
        // open the odometry output
        std::ofstream ofs(filename, std::ofstream::out);

        if (!ofs.good())
        {
            throw std::runtime_error("Could not open the gps estimate output file!");
        }

        StampedGPSPosePtrVector::iterator end = gps_messages.end();
        StampedGPSPosePtrVector::iterator current = gps_messages.begin();

        while (end != current)
        {

            StampedGPSPosePtr gps { *current };

            g2o::SE2 &gps_pose = StampedGPSPose::gps_pose_delays[gps->gps_id].first;

            g2o::SE2 gps_measurement = original ? gps->gps_measurement * gps_pose : gps->gps_measurement;
            double yaw = gps_measurement[2];
            
            // + gps_origin);
            Eigen::Vector2d position(gps_measurement.translation());
            
            ofs << std::fixed << position[0] << " " << position[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << " " << gps->timestamp << std::endl;
            ++current;
        }

        ofs.close();
    }
}


void GrabData::SaveGPSError(std::string filename, g2o::SE2 gps_error)
{
    if (1 < gps_messages.size())
    {
        // open the odometry output
        std::ofstream ofs(filename, std::ofstream::out);

        if (!ofs.good())
        {
            throw std::runtime_error("Could not open the gps estimate output file!");
        }

        StampedGPSPosePtrVector::iterator end = gps_messages.end();
        StampedGPSPosePtrVector::iterator current = gps_messages.begin();

        bool original = false;

        while (end != current)
        {

            StampedGPSPosePtr gps { *current };
            g2o::SE2 &gps_pose = StampedGPSPose::gps_pose_delays[gps->gps_id].first;

            g2o::SE2 gps_measurement = original ? gps->gps_measurement * gps_pose : gps->gps_measurement;
            gps_measurement = gps_measurement * gps_error;
            double yaw = gps_measurement[2];
            // + gps_origin);
            Eigen::Vector2d position(gps_measurement.translation());
            ofs << std::fixed << position[0] << " " << position[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << " " << gps->timestamp << std::endl;
            ++current;
        }

        ofs.close();
    }
}


// save lidar message
void GrabData::SaveLidarEdges(const std::string &msg_name, std::ofstream &os, const StampedLidarPtrVector &lidar_messages, bool use_lidar_odometry, bool use_lidar_loop)
{
    if (!lidar_messages.empty())
    {
        StampedMessagePtr last { raw_messages.back() };
        
        const unsigned last_index = last->id + 1;
        
        for (StampedLidarPtr from : lidar_messages)
        {
            if (last_index > from->seq_id && use_lidar_odometry)
            {
                g2o::SE2 &measurement(from->seq_measurement);
                os << msg_name << "_SEQ " << from->id << " " << from->seq_id << " " << std::fixed << measurement[0] << " " << measurement[1] << " " << measurement[2] << std::endl;
            }
            if (last_index > from->loop_closure_id && use_lidar_loop)
            {
                g2o::SE2 &measurement(from->loop_measurement);
                os << msg_name << "_LOOP " << from->id << " " << from->loop_closure_id << " " << std::fixed << measurement[0] << " " << measurement[1] << " " << measurement[2] << std::endl;
            }
        }
    }
}


// save lidar message
void GrabData::SaveExternalLidarEdges(const std::string &msg_name, std::ofstream &os, const StampedLidarPtrVector &lidar_messages)
{
    if (!lidar_messages.empty())
    {
        // limit index
        for (StampedLidarPtr from : lidar_messages)
        {
            if (std::numeric_limits<unsigned>::max() > from->external_loop_closure_id)
            {
                g2o::SE2 &measurement(from->external_loop_measurement);
                os << msg_name << "_EXTERNAL_LOOP " << from->id << " " << from->external_loop_closure_id << " " << std::fixed << measurement[0] << " " << measurement[1] << " " << measurement[2] << std::endl;
            }
        }
    }
}


// save all the edges containgin icp measurements, from SICK and Velodyne sensors
void GrabData::SaveVisualOdometryEdges(std::ofstream &os)
{
    std::cout << "\tSaving all visual odometry edges..." << std::endl;

    if (!bumblebee_messages.empty() && use_bumblebee_odometry)
    {
        // iterate over the lidar messages
        StampedBumblebeePtrVector::const_iterator end = bumblebee_messages.end();
        StampedBumblebeePtrVector::const_iterator current = bumblebee_messages.begin();

        // limit index
        StampedMessagePtr last { raw_messages.back() };
        const unsigned last_index = last->id + 1;

        while (end != current)
        {
            StampedBumblebeePtr from = *current;

            if (last_index > from->seq_id)
            {
                // get the SE2 reference
                g2o::SE2 &measurement(from->seq_measurement);

                // save the icp measurement
                os << "BUMBLEBEE_SEQ " << from->id << " " << from->seq_id << " ";
                os << std::fixed << measurement[0] << " " << measurement[1] << " " << measurement[2] << std::endl;
            }

            ++current;
        }
    }

    std::cout << "\tVisual odometry saved!" << std::endl;
}


// save all the edges containing icp measurements, from SICK and Velodyne sensors
void GrabData::SaveICPEdges(std::ofstream &os)
{
    std::cout << "\tSaving all " << sick_messages.size() << " SICK edges..." << std::endl;

    // the sick icp edges
    SaveLidarEdges(std::string("SICK"), os, sick_messages, use_sick_odometry, use_sick_loop);

    std::cout << "\tSick edges saved!" << std::endl;

    std::cout << "\tSaving all " << velodyne_messages.size() << " Velodyne edges..." << std::endl;

    // the velodyne icp edges
    SaveLidarEdges(std::string("VELODYNE"), os, velodyne_messages, use_velodyne_odometry, use_velodyne_loop);

    std::cout << "\tVelodyne edges saved!" << std::endl;
}


// save the lidar estimates
void GrabData::SaveRawLidarEstimates(const std::string &filename, const StampedLidarPtrVector &lidar_messages)
{

    if (5 < lidar_messages.size())
    {
        // open the lidar output file
        std::ofstream ofs(filename, std::ofstream::out);

        if (!ofs.good())
        {
            throw std::runtime_error("Could not open the lidar estimate output file!");
        }

        // iterators
        StampedLidarPtrVector::const_iterator end(lidar_messages.end());
        StampedLidarPtrVector::const_iterator it(lidar_messages.begin());

        while (end != it)
        {
            // get the current lidar message
            StampedLidarPtr current_lidar = *it;

            Eigen::Vector3d pose(current_lidar->lidar_estimate.toVector());

            // get the yaw
            double yaw = pose[2];

            // update the position
            // pose[0] += gps_origin[0];
            // pose[1] += gps_origin[1];

            // save the estimate
            ofs << std::fixed << pose[0] << " " << pose[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << " " << current_lidar->timestamp << std::endl;

            ++it;
        }

        // close the output file
        ofs.close();
    }
    else 
    {
        std::cout << "Empty lidar messages of " << filename << std::endl;
    }
}


// save the visual odometry estimates
void GrabData::SaveVisualOdometryEstimates(const std::string &filename)
{

    if (5 < used_frames.size())
    {
        // open the visual odometry output file
        std::ofstream ofs(filename, std::ofstream::out);

        if (!ofs.good())
        {

            throw std::runtime_error("Could not open the visual odometry estimate output file!");
        }

        // iterators
        StampedBumblebeePtrVector::const_iterator end(used_frames.end());
        StampedBumblebeePtrVector::const_iterator it(used_frames.begin());

        while (end != it)
        {
            // get the current lidar message
            StampedBumblebeePtr current_msg = *it;

            Eigen::Vector3d pose(current_msg->visual_estimate.toVector());

            // get the yaw
            double yaw = pose[2];

            // update the position
            // pose[0] += gps_origin[0];
            // pose[1] += gps_origin[1];

            // save the estimate
            ofs << std::fixed << pose[0] << " " << pose[1] << " " << yaw << " " << std::cos(yaw) << " " << std::sin(yaw) << " " << current_msg->timestamp << std::endl;

            ++it;
        }

        // close the output file
        ofs.close();
    }
}


// build Eigen homogeneous matrix from g2o SE2
Eigen::Matrix4f GrabData::BuildEigenMatrixFromSE2(const g2o::SE2 &transform)
{
    // the rotation around the z axis
    Eigen::Transform<float, 3, Eigen::Affine> yaw(Eigen::AngleAxis<float>(transform.rotation().angle(), Eigen::Matrix<float, 3, 1>::UnitZ()));

    // the translation
    Eigen::Translation<float, 3> translation(Eigen::Matrix<float, 3, 1>(transform.toVector()[0], transform.toVector()[1], 0.0f));

    // return the homogeneous matrix
    return (translation * yaw).matrix();
}


// build g2o SE2 from Eigen matrix
g2o::SE2 GrabData::GetSE2FromEigenMatrix(const Eigen::Matrix4f &matrix)
{
    return g2o::SE2(matrix(0, 3), matrix(1, 3), std::atan2(matrix(1, 0), matrix(0, 0)));
}


// get SE2 transform from libviso homogeneous coordinate matrix
g2o::SE2 GrabData::GetSE2FromVisoMatrix(const Matrix &matrix)
{
    // compute the square root
    double root = std::sqrt(std::pow(matrix.val[2][1], 2) + std::pow(matrix.val[2][2], 2 ));

    return g2o::SE2(matrix.val[2][3], -matrix.val[0][3], std::atan2(matrix.val[2][0], root));
    // return g2o::SE2(matrix.val[0][3], matrix.val[1][3], std::atan2(matrix.val[1][0], matrix.val[0][0]));
}


// the main sync method
// PUBLIC METHODS


// configuration
void GrabData::Configure(std::string config_filename, std::string carmen_ini)
{
    std::cout << "Reading cofigure file '" << config_filename << "'" << std::endl;

    std::ifstream is(config_filename, std::ifstream::in);
    if (is.good())
    {
        std::stringstream ss;

        while (-1 != StringHelper::ReadLine(is, ss))
        {
            std::string str;
            ss >> str;

            if ("MAXIMUM_VEL_SCANS" == str)
            {
                ss >> maximum_vel_scans;
            }
            else  if ("LOOP_REQUIRED_TIME" == str)
            {
                ss >> loop_required_time;
            }
            else  if ("LOOP_REQUIRED_DISTANCE" == str)
            {
                ss >> loop_required_distance;
            }
            else  if ("ICP_THREADS_POOL_SIZE" == str)
            {
                ss >> icp_threads_pool_size;
            }
            else  if ("ICP_THREAD_BLOCK_SIZE" == str)
            {
                ss >> icp_thread_block_size;
            }
            else  if ("LIDAR_ODOMETRY_MIN_DISTANCE" == str)
            {
                ss >> lidar_odometry_min_distance;
            }
            else  if ("VISUAL_ODOMETRY_MIN_DISTANCE" == str)
            {
                ss >> visual_odometry_min_distance;
            }
            else  if ("ICP_TRANSLATION_CONFIDENCE_FACTOR" == str)
            {
                ss >> icp_translation_confidence_factor;
            }
            else if ("DISTANCE_BETWEEN_AXLES" == str)
            {
                ss >> VehicleModel::axle_distance;
            }
            else if ("MAX_STEERING_ANGLE" == str)
            {
                ss >> VehicleModel::max_steering_angle;
            }
            else if ("UNDERSTEER" == str)
            {
                ss >> VehicleModel::understeer;
            }
            else if ("KMAX" == str)
            {
                ss >> VehicleModel::kmax;
            }
            else if ("ODOMETRY_BIAS" == str)
            {
                ss >> StampedOdometry::vmb >> StampedOdometry::phimb >> StampedOdometry::phiab;
            }
            else if ("SAVE_ACCUMULATED_POINT_CLOUDS" == str)
            {
                save_accumulated_point_clouds = true;
            }
            else if ("SAVE_LOOP_CLOSURE_POINT_CLOUDS" == str)
            {
                save_loop_closure_point_clouds = true;
            }
            else if ("DISABLE_VELODYNE_ODOMETRY" == str)
            {
                std::cout << "Disabling lidar odometry" << std::endl;
                use_velodyne_odometry = false;
            }
            else if ("DISABLE_VELODYNE_LOOP" == str)
            {
                std::cout << "Disabling lidar loop closures" << std::endl;
                use_velodyne_loop = false;
            }
            else if ("DISABLE_SICK_ODOMETRY" == str)
            {
                std::cout << "Disabling sick odometry" << std::endl;
                use_sick_odometry = false;
            }
            else if ("DISABLE_SICK_LOOP" == str)
            {
                std::cout << "Disabling sick loop closures" << std::endl;
                use_sick_loop = false;
            }
            else if ("DISABLE_BUMBLEBEE_ODOMETRY" == str)
            {
                std::cout << "Disabling visual odometry" << std::endl;
                use_bumblebee_odometry = false;
            }
            else if ("DISABLE_BUMBLEBEE_LOOP" == str)
            {
                std::cout << "Disabling visual loop closures" << std::endl;
                use_bumblebee_loop = false;
            }
            else if ("USE_FAKE_GPS" == str)
            {
                use_fake_gps = true;
            }
            else if ("USE_GPS_ORIENTATION" == str)
            {
                use_gps_orientation = true;
            }
            else if ("USE_RESTRICTED_LOOPS" == str)
            {
                use_restricted_loops = true;
            }
            else if ("GPS_IDENTIFIER" == str)
            {
                std::string gps_id;
                double delay { 0.0 };
                ss >> gps_id;
                ss >> delay;
                if (!gps_id.empty())
                {
                    StampedGPSPose::gps_pose_delays.emplace(gps_id, std::pair<g2o::SE2, double>(g2o::SE2(0.0,0.0,0.0), delay));
                }
            }
            else if ("MIN_SPEED" == str)
            {
                ss >> min_speed;
            }
            else if ("INITIAL_GUESS_POSE" == str)
            {
                double x, y, theta;

                ss >> x >> y >> theta;

                initial_guess_pose = g2o::SE2(x, y, theta);
            }
        }
    }
    else
    {
        std::cout << "Can't find the parser_config.txt file, using default parameters!" << std::endl;
    }

    is.close();

    SetGPSPose(carmen_ini);
}


void GrabData::SetGPSPose(std::string carmen_ini)
{
    std::ifstream is(carmen_ini);

    if (is.good())
    {
        std::stringstream ss;
        double sbx = -1.0f, sby = -1.0f, sbyaw = -1.0f;
        g2o::SE2 sensor_board_pose { 0.0, 0.0, 0.0};
        std::unordered_map<std::string, std::pair<std::string, unsigned>> keys;
        std::vector<std::pair<std::string, unsigned>> sufix { {"_x", 0}, {"_y", 1}, {"_yaw", 2} };

        std::unordered_map<std::string, Eigen::Vector3d> transforms;

        for (auto &entry : StampedGPSPose::gps_pose_delays)
        {
            transforms.emplace(entry.first, Eigen::Vector3d {0.0, 0.0, 0.0});

            for (auto &s : sufix)
            {
                std::string k("gps_nmea" + ("0" != entry.first ? "_" + entry.first + s.first : s.first));
                keys.emplace(k, std::pair<std::string, unsigned>(entry.first, s.second));
            }
        }

        while (-1 != StringHelper::ReadLine(is, ss))
        {
            std::string str;
            ss >> str;

            if (keys.end() != keys.find(str))
            {
                std::pair<std::string, unsigned> &strp { keys[str] };
                Eigen::Vector3d &transform { transforms[strp.first] };
                double value;
                ss >> value;
                transform[strp.second] = value;
            }
            else if ("sensor_board_1_x" == str)
            {
                ss >> sbx;
            }
            else if ("sensor_board_1_y" == str)
            {
                ss >> sby;
            }
            else if ("sensor_board_1_yaw" == str)
            {
                ss >> sbyaw;
            }

            if (0.0 < sbx and 0.0 < sby and 0.0 < sbyaw)
            {
                break;
            }
        }

        sensor_board_pose.setTranslation(Eigen::Vector2d {sbx, sby});
        sensor_board_pose.setRotation(Eigen::Rotation2Dd(mrpt::math::wrapToPi<double>(sbyaw)));

        for (auto &entry : StampedGPSPose::gps_pose_delays)
        {
            Eigen::Vector3d &transform { transforms[entry.first]};

            entry.second.first.setTranslation(Eigen::Vector2d(transform[0], transform[1]));
            entry.second.first.setRotation(Eigen::Rotation2Dd(mrpt::math::wrapToPi<double>(transform[2])));
            entry.second.first = (sensor_board_pose * entry.second.first).inverse();
        }
    }

    is.close();
}


// the main process
// it reads the entire log file and builds the hypergraph
unsigned GrabData::ParseLogFile(const std::string &input_filename, unsigned msg_id)
{
    // the input file stream
    std::ifstream logfile(input_filename);

    if (!logfile.is_open())
    {
        std::cerr << "Unable to open the input file: " << input_filename << std::endl;
        return 0;
    }

    // status report
    std::cout << "Start reading the input logfile (this may take a while)\n";

    StampedLidar::filepath_prefix = input_filename;

    // the input string stream
    std::stringstream current_line;

    // a general counter
    unsigned counter = 0;

    // how many velodyne messages
    unsigned vldn_msgs = 0;

    // how many messages
    unsigned vel_scans = 0 == maximum_vel_scans ? std::numeric_limits<unsigned>::max() : maximum_vel_scans;

    // how many messages
    unsigned percent = vel_scans * 0.1;

    // parse the carmen log file and extract all the desired infos
    while(-1 != StringHelper::ReadLine(logfile, current_line) && vel_scans > vldn_msgs)
    {
        // the input tag
        std::string tag;

        // the first word is the message name
        current_line >> tag;

        // the new message
        StampedMessagePtr msg = nullptr;

        // take care of each message
        if ("XSENS_QUAT_" == tag)
        {
            // build a new XSENS orientation message
            msg = new StampedXSENS(msg_id);
        }
        else if ("ROBOTVELOCITY_ACK" == tag)
        {
            // build a new odometry message
            msg = new StampedOdometry(msg_id);
        }
        else if ("NMEAGGA" == tag)
        {
            // build a new GPS pose message
            msg = new StampedGPSPose(msg_id);
        }
        else if (!use_fake_gps && "NMEAHDT" == tag)
        {
            // build a new GPS orientation message
            msg = new StampedGPSOrientation(msg_id);
        }
        else if ((use_sick_odometry or use_sick_loop) and "LASER_LDMRS_NEW" == tag)
        {
            // build a new sick message
            msg = new StampedSICK(msg_id);
        }
        else if ((use_bumblebee_odometry or use_bumblebee_loop) and "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3____" == tag)   // ZED eh FILE4. Os "_____" sao para nao considerar esta mensagem
        {
            // parse the Bumblebee stereo image message
            msg = new StampedBumblebee(msg_id);
        }
        else if ((use_velodyne_odometry or use_velodyne_loop) and ("VELODYNE_PARTIAL_SCAN_IN_FILE" == tag or "VELODYNE_PARTIAL_SCAN" == tag))
        {
            // build a new velodyne message
            msg = new StampedVelodyne(msg_id);

            // increment the velodyne messages
            ++vldn_msgs;
        }
        else
        {
            continue;
        }

        // parse the stringstream based on the derived class implementation
        if(msg->FromCarmenLog(current_line))
        {
            // update the msg id
            ++msg_id;

            // save it to the msg vector
            raw_messages.push_back(msg);

            // increment the counter
            ++counter;

            if (percent < counter)
            {
                // reset the counter value
                counter = 0;

                // status report
                std::cout << "#" << std::flush;
            }
        }
        else
        {
            delete msg;
        }
    }

    // close the streams
    logfile.close();

    std::cout << std::endl;

    // success!
    return msg_id;
}


// parser
void GrabData::BuildHyperGraph()
{
    if (!raw_messages.empty())
    {
        std::sort(raw_messages.begin(), raw_messages.end(), StampedMessage::compare);

        SeparateMessages();

        // build the gps measurements
        // also moves the global position to local considering the first position as origin
        if (use_fake_gps)
            BuildFakeGPSMeasures();
        else
            BuildGPSMeasures();

        BuildOdometryMeasures();

        // rebuild the initial estimates
        BuildOdometryEstimates(false, use_gps_orientation);

        if (use_bumblebee_odometry)
        {
            BuildVisualOdometryMeasures();
            BuildVisualOdometryEstimates();
        }

        // the loop measurement is done after the gps synchronization
        // For each velodyne message, the method searches for the GPS
        // poses with closest timestamp, and computes the velodyne pose
        // if the car is in the GPS pose. It is a hack for plotting and
        // it does not interfere with the hypergraph optimization.
        BuildLidarOdometryGPSEstimates();

        if (use_velodyne_loop)
            BuildLidarLoopClosureMeasures(velodyne_messages);

        if (use_velodyne_odometry)
        {
            BuildLidarOdometryMeasuresWithThreads(velodyne_messages);
            BuildRawLidarOdometryEstimates(velodyne_messages, used_velodyne);
        }

        if (use_sick_loop)
            BuildLidarLoopClosureMeasures(sick_messages);

        if (use_sick_odometry)
        {
            BuildLidarOdometryMeasuresWithThreads(sick_messages);
            BuildRawLidarOdometryEstimates(sick_messages, used_sick);
        }

        // rebuild the initial estimates
        if (use_fake_gps)
            BuildOdometryEstimates(use_fake_gps, use_gps_orientation);
    }
}


// build loop closures - different log version
void GrabData::BuildExternalLoopClosures(GrabData &gd, double elmind = 1.0f, double elreqd = 5.0f)
{
    use_external_velodyne_loop = BuildExternalLidarLoopClosureMeasures(velodyne_messages, gd.velodyne_messages, gd.gps_origin, elmind, elreqd);
    use_external_sick_loop = BuildExternalLidarLoopClosureMeasures(sick_messages, gd.sick_messages, gd.gps_origin, elmind, elreqd);
}


// save the hyper graph to the output file
void GrabData::SaveHyperGraph(std::ofstream &os, bool global = false)
{
    if (5 < messages.size())
    {
        // save all vertex
        // IT MUST BECOME FIRST
        // so the edges list can use the vertices ids
        SaveAllVertices(os, global);

        // save the odometry edges
        SaveOdometryEdges(os);

        // save the GPS edges
        SaveGPSEdges(os, global);

        // save the xsens edges
        SaveXSENSEdges(os);

        // save the visual odometry edges
        SaveVisualOdometryEdges(os);

        // save the icp edges
        SaveICPEdges(os);
    }
}


// save the external lidar loops
void GrabData::SaveExternalLidarLoopEdges(std::ofstream &os)
{
    if (5 < messages.size())
    {
        if (use_external_velodyne_loop)
            SaveExternalLidarEdges(std::string("VELODYNE"), os, velodyne_messages);

        if (use_external_sick_loop)
            SaveExternalLidarEdges(std::string("SICK"), os, sick_messages);
    }
}


// save the extra estimates
void GrabData::SaveEstimates(std::string &base)
{

    if (not use_gps_orientation)
    {
        BuildHackedOdometryEstimates();
    }

    SaveOdometryEstimates(base + "raw_odometry.txt", true);
    SaveOdometryEstimates(base + "odometry.txt", false);

    SaveGPSEstimates(base + "gps.txt", false);
    SaveGPSEstimates(base + "gps_original.txt", true);
    SaveAllGPSEstimates(base + "raw_gps.txt", false);

    SaveGPSError(base + "gps_plus_error.txt", g2o::SE2(0, 4, M_PI_2));
    SaveGPSError(base + "gps_minus_error.txt", g2o::SE2(0, -4, -M_PI_2));

    SaveVisualOdometryEstimates(base + "visual_odometry.txt");

    SaveRawLidarEstimates(base + "velodyne.txt", used_velodyne);
    SaveRawLidarEstimates(base + "sick.txt", used_sick);

    MeanAbsoluteErrorOdometryGPS();
}


// compute the odometry x gps mean absolute error
void GrabData::MeanAbsoluteErrorOdometryGPS()
{
    if (gps_messages.empty())
    {
        std::cout << "Empty vector!" << std::endl;
        return;
    }

    BuildHackedOdometryEstimates();
    
    double sum = 0.0;
    for (StampedGPSPosePtr gps : gps_messages)
    {
        sum += (gps->gps_measurement.translation() + gps_origin - gps->est.translation()).norm();
    }

    std::cout << "Mean absolute error (GPS x Odometry): " << sum / gps_messages.size() << std::endl;
}

// clear the entire object
void GrabData::Clear()
{
    // discards the lidar messages pointers
    sick_messages.clear();
    velodyne_messages.clear();
    used_velodyne.clear();
    bumblebee_messages.clear();

    // discard the gps messages pointers
    gps_messages.clear();

    // discard the xsens messages pointers
    xsens_messages.clear();

    // discards the odometry messages pointers
    odometry_messages.clear();

    // clear the messages list
    messages.clear();

    // the messages vector needs to be deleted from memmory
    while (!raw_messages.empty())
    {
        // get the last element
        StampedMessagePtr tmp = raw_messages.back();

        // remove the last element from the vector
        raw_messages.pop_back();

        // delete the message from memmory
        delete tmp;
    }
}
