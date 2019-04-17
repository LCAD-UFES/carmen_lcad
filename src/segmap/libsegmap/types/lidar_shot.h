
#ifndef __LIDAR_SHOT_H__
#define __LIDAR_SHOT_H__


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


#endif
