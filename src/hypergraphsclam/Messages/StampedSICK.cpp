#include <StampedSICK.hpp>
#include <iostream>
#include <cmath>
#include <bitset>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace hyper;

const std::string StampedSICK::base_sick_path = "/dados/tmp/sick/sick";

// set the mirror mask
const uint16_t StampedSICK::MIRROR_MASK = 0x1 << 3;

// the basic constructor
StampedSICK::StampedSICK(unsigned msg_id) : StampedMessage(msg_id), StampedLidar(msg_id, base_sick_path) {}

// the basic destructor
StampedSICK::~StampedSICK() {}

// PUBLIC METHODS

void StampedSICK::LoadPointCloud(PointCloudHSV &cloud)
{
    StampedLidar::LoadPointCloud(StampedLidar::path, cloud);
}

// parse the pose from string stream
bool StampedSICK::FromCarmenLog(std::stringstream &ss)
{
    // helpers
    uint16_t scanner_status;
    unsigned points;
    double h_angle, v_angle, distance;

    // the PCL point cloud
    PointCloudHSV cloud;

    // discards the first value
    SkipValues(ss, 1);

    // get the scanner status flag
    ss >> scanner_status;

    // apply the mask and verify
    if (scanner_status & MIRROR_MASK)
    {
        // discards the next six values
        SkipValues(ss, 6);

        // get the number of points
        ss >> points;

        // discards the next value
        SkipValues(ss, 1);

        // read all points
        for (unsigned i = 0; i < points; ++i)
        {
            // get the horizontal angle
            ss >> h_angle;

            // get the vertical angle
            ss >> v_angle;

            // get the distance
            ss >> distance;

            // discards the next 3 values
            SkipValues(ss, 3);

            // and save it to the point cloud
            cloud.push_back(StampedLidar::FromSpherical(h_angle, M_PI_2 - v_angle, distance));

        }

        // get the timestamp
        ss >> StampedMessage::timestamp;

        // save the cloud
        std::stringstream ss_parser;
        
        ss_parser << StampedMessage::id;

        // save the name
        StampedLidar::path += (ss_parser.str() + ".pcd");

        // save the pcd file
        if (-1 != pcl::io::savePCDFile(StampedLidar::path, cloud, true))
        {
            return true;
        }
    }

    return false;
}

// get the message type
StampedMessageType StampedSICK::GetType()
{
    return StampedSICKMessage;
}
