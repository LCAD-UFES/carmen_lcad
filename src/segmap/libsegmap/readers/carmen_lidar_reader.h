
#ifndef __LIDAR_LOADER_H__
#define __LIDAR_LOADER_H__

#include <string>
#include <carmen/lidar_shot.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class CarmenLidarLoader
{
public:
    CarmenLidarLoader(std::string &cloud_path, int n_rays, unsigned char ***calibration);
    ~CarmenLidarLoader();

    LidarShot* next();
    bool done();
    void reset();

    int _n_readings;

protected:
    int _n_rays;
    unsigned short int *_raw_ranges;
    unsigned char *_raw_intensities;
    FILE *_fptr;
    LidarShot *_shot;
    unsigned char ***_calibration;

    // num vertical lasers in the velodyne used at ufes.
    static const int _n_vert = 32;
    static int _get_distance_index(double distance);
};


void load_as_pointcloud(CarmenLidarLoader *loader, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


#endif
