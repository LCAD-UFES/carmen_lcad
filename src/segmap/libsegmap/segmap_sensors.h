
#ifndef __SEGMAP_SENSORS_H__
#define __SEGMAP_SENSORS_H__

#include <cstdio>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;


class LidarShot
{
public:
    // n rays in the shot
    int n;
    // range measurements
    double *ranges;
    // vertical angles of each ray (in rad)
    double *v_angles;
    // intensity measurements
    unsigned char *intensities;
    // horizontal angle (in rad)
    double h_angle;

    LidarShot(int n_rays);
    ~LidarShot();
};


class CarmenLidarLoader
{
public:
    CarmenLidarLoader(const char *cloud_path, int n_rays, unsigned char ***calibration);
    ~CarmenLidarLoader();

    LidarShot* next();
    bool done();
    void reset();

protected:
    int _n_rays;
    int _n_readings;
    short int *_raw_ranges;
    unsigned char *_raw_intensities;
    FILE *_fptr;
    LidarShot *_shot;
    unsigned char ***_calibration;

    // num vertical lasers in the velodyne used at ufes.
    static const int _n_vert = 32;
    static int _get_distance_index(double distance);
};


void load_as_pointcloud(CarmenLidarLoader *loader, PointCloud<PointXYZRGB>::Ptr cloud);


#endif