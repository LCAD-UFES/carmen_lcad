
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int 
main (int argc, char** argv)
{
    int32_t num = 1000000;
    float *data = (float*) malloc(num * sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen ("velodyne_0000000000.bin", "rb");
    num = fread(data, sizeof(float), num, stream) / 4;

    double x_origin = 0.;
    double y_origin = -20.;
    double pixel_by_meter_x = 600. / 40.;
    double pixel_by_meter_y = 600. / 40.;

    Mat map(600, 600, CV_8UC3);

    for (int32_t i=0; i < num; i++) 
    {
        px += 4; py += 4; pz += 4; pr += 4;

        if (*px > 0)
        {
            if ((*px) > 20) (*px) = 20;
            if ((*py) > 20) (*py) = 20;
            if ((*py) < -20) (*py) = -20;

            int pixel_x = ((*px) - x_origin) * pixel_by_meter_x;
            int pixel_y = ((*py) - y_origin) * pixel_by_meter_y;
            
            map.data[3 * (pixel_y * map.cols + pixel_x) + 0] = (unsigned char) 255 * (*pr);
            map.data[3 * (pixel_y * map.cols + pixel_x) + 1] = (unsigned char) 255 * (*pr);
            map.data[3 * (pixel_y * map.cols + pixel_x) + 2] = (unsigned char) 255 * (*pr);
        }
    }

    fclose(stream);

    imshow("map", map);
    waitKey(-1);

    return 0;
}


