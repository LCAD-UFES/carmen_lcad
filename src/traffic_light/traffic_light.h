/* 
 * File:   traffic_light.h
 * Author: tiago
 *
 * Created on February 14, 2012, 5:11 PM
 */

#ifndef TRAFFIC_LIGHT_H
#define	TRAFFIC_LIGHT_H
//OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

//Carmen
#include <carmen/carmen.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>

using namespace std;
using namespace cv;


#ifdef	__cplusplus
extern "C"
{
#endif

    IplImage* convertImageHSVtoRGB(const IplImage *imageHSV);
    IplImage* convertImageRGBtoHSV(const IplImage *imageRGB);
    //void compute_traffic_light(carmen_bumblebee_basic_stereoimage_message *stereo_image, int image_width, int image_height,  IplImage *image);

    void copy_raw_image_to_opencv_image(unsigned char *original, IplImage *copy, int nchannels);
    void copy_opencv_image_to_raw_image(IplImage original, unsigned char *copy, int nchannels);

#ifdef	__cplusplus
}
#endif

#endif	/* TRAFFIC_LIGHT_H */