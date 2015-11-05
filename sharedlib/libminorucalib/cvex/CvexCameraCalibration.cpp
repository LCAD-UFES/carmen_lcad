#include "CvexCameraCalibration.h"
#include <iostream>

using namespace std;

int CvexCameraCalibration::getImageCount()
{
    return image_count;
}

void CvexCameraCalibration::init(
    CvSize img_size,
    CvSize _pattern_size,
    double _CHESS_LENGTH,
    int _max_chess_count)
{
    image_count = 0;
    image_size = img_size;
    pattern_size=_pattern_size;
    CHESS_LENGTH=_CHESS_LENGTH;

    intrinsic = cvCreateMat (3, 3, CV_64FC1);
    rodrigues = cvCreateMat (1, 3, CV_64FC1);
    rotation = cvCreateMat (3, 3, CV_64FC1);
    translation = cvCreateMat (1, 3, CV_64FC1);
    distortion = cvCreateMat (1, 4, CV_64FC1);

    PATTERN_SIZE = pattern_size.width*pattern_size.height;
    corners =
        (CvPoint2D32f *) cvAlloc (sizeof (CvPoint2D32f) * PATTERN_SIZE);

    MAX_CHESS_COUNT = _max_chess_count;
    subpix_corners  = new CvPoint2D32f*[MAX_CHESS_COUNT];
    for(int i=0;i<MAX_CHESS_COUNT;i++)subpix_corners[i]=NULL;
    p_count = new int[MAX_CHESS_COUNT];
}

CvexCameraCalibration::CvexCameraCalibration()	
{
}

CvexCameraCalibration::CvexCameraCalibration(
    CvSize img_size,
    CvSize _pattern_size,
    double _CHESS_LENGTH,
    int _max_chess_count)
{
    init(img_size, _pattern_size, _CHESS_LENGTH, _max_chess_count);
}

CvexCameraCalibration::~CvexCameraCalibration()
{
    cvReleaseMat(&intrinsic);
    cvReleaseMat(&rodrigues);
    cvReleaseMat(&rotation);
    cvReleaseMat(&translation);
    cvReleaseMat(&distortion);

    cvFree(&corners);

    for (int i=0;i<image_count;i++) {
        if (subpix_corners[i]!=NULL)
            cvFree(&subpix_corners[i]);
    }
    delete[] subpix_corners;
    delete[] p_count;
}

bool CvexCameraCalibration::findChessboardCorners(
    IplImage* src_img,
    bool isStore)
{
    bool ret=false;

    isFound = cvFindChessboardCorners(
        src_img, pattern_size, corners, &corner_count);
    if (isFound) {
        ret = true;
        if (isStore) {
            IplImage *src_gray =
                cvCreateImage (cvGetSize (src_img), IPL_DEPTH_8U, 1);
            cvCvtColor (src_img, src_gray, CV_BGR2GRAY);

            subpix_corners[image_count] =
                (CvPoint2D32f *)cvAlloc(sizeof (CvPoint2D32f) * PATTERN_SIZE);

            cvFindChessboardCorners(
                src_img,
                pattern_size,
                subpix_corners[image_count],
                &corner_count);

            cvFindCornerSubPix(
                src_gray,
                subpix_corners[image_count],
                corner_count,
                cvSize(3, 3),
                cvSize(-1, -1),
                cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

            p_count[image_count] = corner_count;
            image_count++;

            cvReleaseImage(&src_gray);
        }
    }
    return ret;
}

void CvexCameraCalibration::drawChessboardCorners(
    IplImage* src,
    IplImage* dest)
{
    cvCopy(src,dest);
    cvDrawChessboardCorners(
        dest,
        pattern_size,
        corners,
        corner_count,
        isFound);
}

void CvexCameraCalibration::solveExtrinsic(int pattern_number)
{
    CvMat object_points;
    CvMat image_points;

    CvPoint3D32f* objects= new CvPoint3D32f[PATTERN_SIZE];
    for (int j = 0; j < pattern_size.height; j++) {
        for (int k = 0; k < pattern_size.width; k++) {
            objects[j * pattern_size.width + k].x = (float)(j * CHESS_LENGTH);
            objects[j * pattern_size.width + k].y = (float)(k * CHESS_LENGTH);
            objects[j * pattern_size.width + k].z = 0.0f;
        }
    }

    cvInitMatHeader(
        &object_points,
        PATTERN_SIZE, 3,
        CV_32FC1, objects);

    cvInitMatHeader(
        &image_points,
        PATTERN_SIZE, 1,
        CV_32FC2,
        subpix_corners[pattern_number]);

    cvFindExtrinsicCameraParams2 (
        &object_points,
        &image_points,
        intrinsic,
        distortion,
        rodrigues,
        translation);

    cvRodrigues2(rodrigues,rotation);

    delete[]objects;

    cvexShowMatrix(rotation);
    cvexShowMatrix(translation);
}

void CvexCameraCalibration::solveIntrinsic()
{
    if (image_count < 3) {
        cout<<"3 or more input images are required"<<endl;
        return;
    }

    CvMat object_points;
    CvMat image_points;
    CvMat point_counts;

    CvPoint3D32f* objects= new CvPoint3D32f[image_count*PATTERN_SIZE];
    for (int i = 0; i < image_count; i++) {
        for (int j = 0; j < pattern_size.height; j++) {
            for (int k = 0; k < pattern_size.width; k++) {
                objects[i * PATTERN_SIZE + j * pattern_size.width + k].x =
                    (float)(j * CHESS_LENGTH);
                objects[i * PATTERN_SIZE + j * pattern_size.width + k].y =
                    (float)(k * CHESS_LENGTH);
                objects[i * PATTERN_SIZE + j * pattern_size.width + k].z =
                    0.0f;
            }
        }
    }		
    cvInitMatHeader(
        &object_points,
        image_count*PATTERN_SIZE, 3,
        CV_32FC1, objects);

    CvPoint2D32f *_corners = new CvPoint2D32f[image_count*PATTERN_SIZE];

    for (int i = 0; i < image_count; i++) {
        for (int j = 0; j < pattern_size.height; j++) {
            for (int k = 0; k < pattern_size.width; k++) {
                _corners[i * PATTERN_SIZE + j * pattern_size.width + k].x =
                    subpix_corners[i][j * pattern_size.width + k].x;
                _corners[i * PATTERN_SIZE + j * pattern_size.width + k].y =
                    subpix_corners[i][j * pattern_size.width + k].y;
            }
        }
    }

    cvInitMatHeader(
        &image_points, image_count*PATTERN_SIZE, 1, CV_32FC2, _corners);

    cvInitMatHeader(
        &point_counts, image_count, 1, CV_32SC1, p_count);

    cvCalibrateCamera2(
        &object_points, &image_points, &point_counts,
        image_size, intrinsic, distortion);

    cvexShowMatrix(intrinsic);
    cvexShowMatrix(distortion);

    delete[]objects;
    delete[]_corners;
}

