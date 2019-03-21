
#ifndef __SEGMAP_SENSORS_H__
#define __SEGMAP_SENSORS_H__

#include <cstdio>
#include <string>
#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_dataset.h>
#include <opencv/cv.hpp>


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


class SemanticSegmentationLoader
{
public:
    SemanticSegmentationLoader(const char *log_path, const char *data_path="/dados/data");
    ~SemanticSegmentationLoader();

    cv::Mat load(DataSample *sample);

protected:

    std::string _log_data_dir;
};


cv::Mat load_image(DataSample *sample);
void load_as_pointcloud(CarmenLidarLoader *loader, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void get_pixel_position(double x, double y, double z, Eigen::Matrix<double, 4, 4> &lidar2cam,
                        Eigen::Matrix<double, 3, 4> &projection, cv::Mat &img, cv::Point *ppixel, int *is_valid);

#endif
