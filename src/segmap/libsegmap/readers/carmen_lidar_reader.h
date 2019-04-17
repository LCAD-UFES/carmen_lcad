
#ifndef __LIDAR_LOADER_H__
#define __LIDAR_LOADER_H__

#include <string>
#include <carmen/lidar_shot.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class CarmenLidarLoader
{
public:
    CarmenLidarLoader();
    ~CarmenLidarLoader();

    void reinitialize(std::string &cloud_path, int n_rays);

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

    // num vertical lasers in the velodyne used at ufes.
    static const int _n_vert = 32;
};


//void load_as_pointcloud(CarmenLidarLoader *loader, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


#endif
