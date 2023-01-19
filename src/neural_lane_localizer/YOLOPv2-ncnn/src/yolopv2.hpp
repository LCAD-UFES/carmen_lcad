
#ifndef YOLOPV2_HPP
#define YOLOPV2_HPP

#if defined(USE_NCNN_SIMPLEOCV)
#include "simpleocv.h"
#else
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include "../include/ncnn/layer.h"
#include "../include/ncnn/net.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <numeric>

using namespace std;
using namespace cv;
#define MAX_STRIDE 64

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

static void slice(const Mat& in, Mat& out, int start, int end, int axis);
static void interp(const Mat& in, const float& scale, const int& out_w, const int& out_h, Mat& out);
static inline float intersection_area(const Object& a, const Object& b);
static void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right);
static void qsort_descent_inplace(std::vector<Object>& faceobjects);
static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold);
static inline float sigmoid(float x);
static void generate_proposals(const Mat& anchors, int stride, const Mat& in_pad, const Mat& feat_blob, float prob_threshold, std::vector<Object>& objects);
static void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects, Mat& da_seg_mask, Mat& ll_seg_mask);
static int detect_yolopv2( cv::Mat& bgr, std::vector<Object>& objects, Mat& da_seg_mask_, Mat&  ll_seg_mask_);

#endif
