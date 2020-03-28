#include <StampedLidar.hpp>
#include <limits>

using namespace hyper;

// set the multiplier
double StampedLidar::to_degree = M_PI / 180.0f;

// define the default voxel grid leaf filter value
double StampedLidar::vg_leaf = 0.2;

// the voxel grid
pcl::VoxelGrid<pcl::PointXYZHSV> StampedLidar::grid_filtering;

// the segmentation class
SimpleLidarSegmentation StampedLidar::segm;

std::string StampedLidar::filepath_prefix = "";

// the basic constructor
StampedLidar::StampedLidar(unsigned msg_id, const std::string &base_path) :
    StampedMessage::StampedMessage(msg_id),
    minx(std::numeric_limits<double>::max()),
    maxx(-std::numeric_limits<double>::max()),
    miny(std::numeric_limits<double>::max()),
    maxy(-std::numeric_limits<double>::max()),
    minz(std::numeric_limits<double>::max()),
    maxz(-std::numeric_limits<double>::max()),
    speed(0.0),
    path(base_path),
    seq_measurement(0.0, 0.0, 0.0),
    seq_id(std::numeric_limits<unsigned>::max()),
    lidar_estimate(0.0, 0.0, 0.0),
    gps_sync_estimate(0.0, 0.0, 0.0),
    loop_measurement(0.0, 0.0, 0.0),
    loop_closure_id(std::numeric_limits<unsigned>::max()),
	external_loop_measurement(0.0, 0.0, 0.0),
	external_loop_closure_id(std::numeric_limits<unsigned>::max())
{
    // set the default leaf size
    StampedLidar::grid_filtering.setLeafSize(vg_leaf, vg_leaf, vg_leaf);

}

// the basic destructor
StampedLidar::~StampedLidar()
{}
// PRIVATE METHODS
pcl::PointXYZHSV StampedLidar::FromSpherical(double phi, double theta, double radius)
{
    // build a new point
    pcl::PointXYZHSV point;

    // precalculate a commom value
    float rsin = float(radius * std::sin(theta));

    // get the rectangular coordinates
    point.x = float(rsin * std::cos(phi));
    point.y = float(rsin * std::sin(phi));
    point.z = float(radius * std::cos(theta));

    // set the intensity
    point.h = float(phi);
    point.s = float(theta);
    point.v = float(radius);

    return point;
}

// save the point cloud
void StampedLidar::SavePointCloud(const std::string &base_path, unsigned cloud_id, const PointCloudHSV &cloud)
{
    if (0 < cloud.size())
    {
        // the string stream
        std::stringstream ss;

        // the number
        ss << base_path << "cloud" << cloud_id << ".pcd";

        // save the cloud
        if (-1 == pcl::io::savePCDFile(ss.str(), cloud, true))
        {
            throw std::runtime_error("Could not save the cloud");

        }
    }
}

// custom output file
void StampedLidar::SavePointCloud(const std::string &cloud_path, const PointCloudHSV &cloud)
{
    // open the ouptut file
    std::ofstream output(cloud_path, std::ofstream::out);

    if (!output.is_open())
    {
       throw std::runtime_error("Could not open the output file\n");
    }

    // points vector access
    std::vector<pcl::PointXYZHSV, Eigen::aligned_allocator<pcl::PointXYZHSV>>::const_iterator point = cloud.points.begin();
    std::vector<pcl::PointXYZHSV, Eigen::aligned_allocator<pcl::PointXYZHSV>>::const_iterator end = cloud.points.end();

    while (end != point)
    {
        // get the current point
        const pcl::PointXYZHSV &p(*point);

        // write to the output file
        output << p.x << " " << p.y << " " << p.z << " " << p.h << " " << p.s << " " << p.v << "\n";

        // go to the next point
        ++point;

    }

    // close the output file
    output.close();

}

// custom point cloud loading process
void StampedLidar::LoadPointCloud(const std::string &cloud_path, PointCloudHSV &cloud)
{
    // open the input file
    std::ifstream input(cloud_path, std::ofstream::in);

    if (!input.is_open())
    {
        throw std::runtime_error("Could not open the input file\n");
    }

    // clear the point cloud
    cloud.clear();

    // the next point
    pcl::PointXYZHSV point;

    // the input string stream
    std::stringstream ss;

    // read the first line
    while (-1 != StringHelper::ReadLine(input, ss))
    {
        // read the coordinates
        ss >> point.x;
        ss >> point.y;
        ss >> point.z;

        // read the intensity
        ss >> point.h;
        ss >> point.s;
        ss >> point.v;

        // save it to the cloud
        cloud.push_back(point);

    }

    // close the file
    input.close();

}

// remove undesired points
void StampedLidar::RemoveUndesiredPoints(PointCloudHSV &cloud)
{
    segm.PointTypeSegmentation(cloud, absx, absy, minz, maxz);

}

void StampedLidar::RemoveUndesiredPoints(SimpleLidarSegmentation &_segm, PointCloudHSV &cloud)
{
    _segm.PointTypeSegmentation(cloud, absx, absy, minz, maxz);
}
