#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

using namespace cv;
int main()
{

    FILE *pointcloud_file = fopen("/dados/log_volta_da_ufes_art-20210305.txt_lidar/1614950000/1614956100/1614956164.321218.pointcloud", "rb");
    double angle;
    unsigned short distance[32];
    unsigned char intensity[32];
    for (int i = 0; i < 1084; i++)
    {
        fread(&angle, sizeof(double), 1, pointcloud_file);
        fread(&distance, sizeof(short), 32, pointcloud_file);
        fread(&intensity, sizeof(char), 32, pointcloud_file);
        printf("angulo: %f ,",angle);
        for (int j=0; j<32; j++)
        {
            printf("distancia: %d ", (distance[j])/500);
        }
        printf("\n");
    }
    Mat pose_image;
    pose_image = Mat::zeros(Size(1084,32), CV_8UC1);
    fclose(pointcloud_file);
    return 0;
}