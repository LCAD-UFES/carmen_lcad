#include <StampedVelodyne.hpp>
#include <fstream>
#include <cctype>
#include <clocale>

#include <pcl/io/pcd_io.h>

using namespace hyper;

// the base path
const std::string StampedVelodyne::base_velodyne_path = "/dados/tmp/velodyne/velodyne";

// the double size
const unsigned StampedVelodyne::double_size = sizeof(double);

// the char size
const unsigned StampedVelodyne::char_size = sizeof(char);

// the short size
const unsigned StampedVelodyne::short_size = sizeof(short);

// the char short size
const unsigned StampedVelodyne::short_char_size = sizeof(short) + sizeof(char);

// the velodyne struct size
const unsigned StampedVelodyne::velodyne_struct_size = double_size + 32 * short_char_size;


const double StampedVelodyne::vertical_correction[32] =
{
    M_PI_2 + 0.535292481586660873205119060003198683261871337890625,
    M_PI_2 + 0.16283921746574170352772625847137533128261566162109375,
    M_PI_2 + 0.51190506960993686913496958368341438472270965576171875,
    M_PI_2 + 0.1396263401595463637949734447829541750252246856689453125,
    M_PI_2 + 0.4886921905584122871601948645547963678836822509765625,
    M_PI_2 + 0.11641346285335103794000843890898977406322956085205078125,
    M_PI_2 + 0.46547931150688770518542014542617835104465484619140625,
    M_PI_2 + 0.0930260473859685077524517282654414884746074676513671875,
    M_PI_2 + 0.442091899530163645604119437848567031323909759521484375,
    M_PI_2 + 0.06981317007977318189748672239147708751261234283447265625,
    M_PI_2 + 0.41887902047863911914049594997777603566646575927734375,
    M_PI_2 + 0.046600292773577856042521716517512686550617218017578125,
    M_PI_2 + 0.3956661414271145371657212308491580188274383544921875,
    M_PI_2 + 0.0232128790515245854442216710822322056628763675689697265625,
    M_PI_2 + 0.372278729450390477584420523271546699106693267822265625,
    M_PI_2 - 0.0,
    M_PI_2 + 0.349065850398865895609645804142928682267665863037109375,
    M_PI_2 - 0.0232128790515245854442216710822322056628763675689697265625,
    M_PI_2 + 0.32585297134734136914602231627213768661022186279296875,
    M_PI_2 - 0.046600292773577856042521716517512686550617218017578125,
    M_PI_2 + 0.30246555937061725405357037743669934570789337158203125,
    M_PI_2 - 0.06981317007977318189748672239147708751261234283447265625,
    M_PI_2 + 0.279252680319092727589946889565908350050449371337890625,
    M_PI_2 - 0.0930260473859685077524517282654414884746074676513671875,
    M_PI_2 + 0.256039801267568145615172170437290333211421966552734375,
    M_PI_2 - 0.11641346285335103794000843890898977406322956085205078125,
    M_PI_2 + 0.232652389290844141545022694117506034672260284423828125,
    M_PI_2 - 0.1396263401595463637949734447829541750252246856689453125,
    M_PI_2 + 0.209439510239319559570247974988888017833232879638671875,
    M_PI_2 - 0.16283921746574170352772625847137533128261566162109375,
    M_PI_2 + 0.1862266311877949498398976402313564904034137725830078125,
    M_PI_2 - 0.1862266311877949498398976402313564904034137725830078125
};

// the basic constructor
StampedVelodyne::StampedVelodyne(unsigned msg_id) : StampedMessage(msg_id), StampedLidar(msg_id, base_velodyne_path), vertical_scans(0) {}


// the basic destructor
StampedVelodyne::~StampedVelodyne() {}


// read the point cloud from file
void StampedVelodyne::ReadVelodyneCloudFromFile(PointCloudHSV &cloud)
{
    // open the pointcloud file
    std::ifstream source(StampedLidar::path, std::ifstream::in | std::ifstream::binary);

    if (!source.is_open())
    {
        // error
        std::string error("Could not open the velodyne point cloud: ");
        error += path;

        // return the empty pointer
        throw std::runtime_error(error);
    }

    cloud.clear();
    
    // the point cloud values
    double h_angle, v_angle;
    double distance;

    // the input data size in bytes
    unsigned data_size = velodyne_struct_size * vertical_scans;

    // a memmory helper
    char *binary_buffer = new char[data_size];

    // load the entire point cloud to memmory
    source.read(binary_buffer, data_size);

    // close the file
    source.close();

    // read all the point cloud file
    for (unsigned i = 0; i < vertical_scans; ++i)
    {
        // set the char walking pointer
        char *cp = binary_buffer + i * velodyne_struct_size;

        // get the horizontal angle
        h_angle = -(*((double*) cp)) * StampedLidar::to_degree;

        // set the distance pointer
        short *dp = (short*) (cp + double_size);

        for (unsigned j = 0; j < 32; ++j)
        {
            // get the vertical angle
            v_angle = vertical_correction[j];

            // get the distance value
            distance = ((double) dp[j]) * 0.002;

            // conver to cartesian coords
            pcl::PointXYZHSV point(StampedLidar::FromSpherical(h_angle, v_angle, distance));

            if (4.0 < distance && 100.0 > distance && -2.9 < point.z)
            {
                // update the min max values
                if (minx > point.x) minx = point.x;
                if (maxx < point.x) maxx = point.x;

                if (miny > point.y) miny = point.y;
                if (maxy < point.y) maxy = point.y;

                if (minz > point.z) minz = point.z;
                if (maxz < point.z) maxz = point.z;

                // get the hsv point
                cloud.push_back(point);
            }
        }
    }

    // remove the buffer from memmory
    delete [] binary_buffer;

    // set the abs values
    absx = std::fabs(maxx) > std::fabs(minx) ? std::fabs(maxx) : std::fabs(minx);
    absy = std::fabs(maxy) > std::fabs(miny) ? std::fabs(maxy) : std::fabs(miny);
    absz = std::fabs(maxz) > std::fabs(minz) ? std::fabs(maxz) : std::fabs(minz);
}

// read the point cloud from carmen log
PointCloudHSV::Ptr StampedVelodyne::ReadVelodyneCloudFromLog(std::stringstream &ss)
{
    // the input cloud
    PointCloudHSV::Ptr input_cloud(new PointCloudHSV());

    // beam info
    double h_angle, v_angle, distance;

    // the first, second and fourth nibbles
    unsigned char nibbles[4];

    // helper
    int r;

    // how many vertical scans
    ss >> vertical_scans;

    // read all data
    for (unsigned i = 0; i < vertical_scans; ++i)
    {
        // read the horizontal angle
        ss >> h_angle;

        // apply the conversion
        h_angle *= -0.01 * to_degree;

        // string to read the entire vertical scan
        std::string scan("");

        // read the current vertical scan
        ss >> scan;

        // the current position index
        unsigned current_pos = 0;

        // each individual vertical scan should have the same 32 lasers
        // so the scan string should have 192 bytes
        for (unsigned j = 0; j < 32; ++j)
        {
            // get the vertical angle
            v_angle = M_PI_2 - vertical_correction[j];

            for (unsigned k = 0; k < 4; ++k)
            {
                // get the current nibble, then move forward the current position index
                r = scan[current_pos++];

                // convert the int value to char, bit shifting operation
                nibbles[k] = std::isalpha(r) ? r - 'a' + 10 : r - '0';
            }

            // move the current position two byte ahead, it discards the intensity value
            current_pos += 2;

            // convert the four nibbles to the distance value
            distance = (nibbles[3] << 12 | (nibbles[2] << 8 | (nibbles[1] << 4 | nibbles[0]))) * 0.02;

            // conver to cartesian coords
            pcl::PointXYZHSV point(StampedLidar::FromSpherical(h_angle, v_angle, distance));

            if (4.0 < distance && 100.0 > distance && -2.9 < point.z)
            {
                // update the min max values
                if (minx > point.x) minx = point.x;
                if (maxx < point.x) maxx = point.x;

                if (miny > point.y) miny = point.y;
                if (maxy < point.y) maxy = point.y;

                if (minz > point.z) minz = point.z;
                if (maxz < point.z) maxz = point.z;

                // get the hsv point
                input_cloud->push_back(point);
            }
        }
    }

    // set the abs values
    absx = std::fabs(maxx) > std::fabs(minx) ? std::fabs(maxx) : std::fabs(minx);
    absy = std::fabs(maxy) > std::fabs(miny) ? std::fabs(maxy) : std::fabs(miny);
    absz = std::fabs(maxz) > std::fabs(minz) ? std::fabs(maxz) : std::fabs(minz);

    // return the current cloud
    return input_cloud;
}

void StampedVelodyne::LoadPointCloud(PointCloudHSV &cloud)
{
    if (from_file)
    {
        ReadVelodyneCloudFromFile(cloud);
    }
    else
    {
        StampedLidar::LoadPointCloud(StampedLidar::path, cloud);
    }
}

// parse the pose from string stream
bool StampedVelodyne::FromCarmenLog(std::stringstream &ss)
{
    from_file = 1e04 > StringHelper::GetStringStreamSize(ss);
    
    if (from_file)
    {
        path.clear();
        ss >> StampedLidar::path;
        ss >> vertical_scans;
        ss >> StampedMessage::timestamp;

        if ('/' != StampedLidar::path[0])
        {
        	StampedLidar::path = StampedLidar::filepath_prefix + StampedLidar::path;
        }

        return (bool) std::ifstream(StampedLidar::path);
    }
    else
    {
        PointCloudHSV::Ptr input_cloud { ReadVelodyneCloudFromLog(ss) };

        if (0 < input_cloud->size())
        {
            // creates the filtered version
            PointCloudHSV::Ptr filtered_cloud(new PointCloudHSV());

            // remove undesired points
            StampedLidar::RemoveUndesiredPoints(*input_cloud);

            // filtering the input point cloud
            StampedLidar::grid_filtering.setInputCloud(input_cloud);

            // get the resulting filtered version
            StampedLidar::grid_filtering.filter(*filtered_cloud);

            // set the filename
            std::stringstream pcd_filename;
            pcd_filename << StampedMessage::id << ".pcd";

            // update the path name
            StampedLidar::path += pcd_filename.str();

            // save the input cloud, binary option set to true
            if (-1 == pcl::io::savePCDFile(StampedLidar::path, *filtered_cloud, true))
            {
                // show the error
                std::cerr << "Could not save the input cloud, verify the tmp/velodyne/ directory\n";

                return false;
            }

            // clear the filtered cloud
            filtered_cloud->clear();
        }
        // clear the input cloud
        input_cloud->clear();
        
        return true;
    }
}

// get the message type
StampedMessageType StampedVelodyne::GetType()
{
    return StampedVelodyneMessage;
}
