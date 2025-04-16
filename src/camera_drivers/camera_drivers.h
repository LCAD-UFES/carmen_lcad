#ifndef CAMERA_DRIVERS_H
#define CAMERA_DRIVERS_H

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <list>

#endif

// FFmpeg
extern "C"
{
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

#include "camera_drivers_interface.h"

using namespace std;
using namespace cv;

typedef enum
{
    udp_rtp_h264,
    ip_opencv,
    ip_ffmpeg,
    usb_opencv
} camera_models;

typedef struct tAv_ffmpeg
{
    AVFormatContext *inctx;
    int vstrm_idx;
    AVStream *vstrm;
    AVCodecContext *avctx;
    SwsContext *swsctx;
    AVFrame *frame;
    std::vector<uint8_t> *framebuf;
    AVFrame *decframe;
    unsigned nb_frames;
    bool end_of_stream;
    AVPacket pkt;

    int dst_width;
    int dst_height;

    int use_with_opencv;
    cv::VideoCapture *vcap;

    int ret;
} av_ffmpeg;

void setup_image_struct(int width, int height, int number_of_channels, int data_type, camera_image *image);

unsigned char*
get_image_udp_rtp(cv::VideoCapture& cap, char* ip_address, char* port, char* network_interface, int &width, int &height);

unsigned char*
get_image_ip_camera_with_opencv(char* ip_address, int undistort, char* camera_name, int &width, int &height);

unsigned char*
get_image_usb_camera_with_opencv(char* camera_model, int undistort, char* camera_name, int &width, int &height);

unsigned char*
get_image_ip_camera_with_ffmmpeg(char* ip_address, int undistort, char* camera_name, av_ffmpeg *afp, int &width, int &height);

#endif
