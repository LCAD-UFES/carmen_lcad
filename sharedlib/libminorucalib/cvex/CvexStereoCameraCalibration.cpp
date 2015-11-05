#include "CvexStereoCameraCalibration.h"
#include <iostream>

using namespace std;

int CvexStereoCameraCalibration::getImageCount()
{
    return image_count;
}

void CvexStereoCameraCalibration::getKRK(
    CvMat* srcK,
    CvMat* R,
    CvMat* destK,
    CvMat* dest)
{
    CvMat* temp = cvCloneMat(srcK);
    CvMat* swap = cvCloneMat(srcK);

    cvInvert(srcK,temp);
    cvMatMul(R,temp,swap);
    cvMatMul(destK,swap,dest);

    cvReleaseMat(&temp);
    cvReleaseMat(&swap);	
}

CvexStereoCameraCalibration::CvexStereoCameraCalibration(
    CvSize img_size,
    CvSize _pattern_size,
    double _CHESS_LENGTH,
    int _max_chess_count)
{
    init(img_size,_pattern_size,_CHESS_LENGTH,_max_chess_count);

    leftCamera.init(img_size,_pattern_size,_CHESS_LENGTH,_max_chess_count);
    rightCamera.init(img_size,_pattern_size,_CHESS_LENGTH,_max_chess_count);

    relate_rot= cvCreateMat (3, 1, CV_64FC1);
    relate_trans= cvCreateMat (3, 1, CV_64FC1);

    E = cvCreateMat (3, 3, CV_64FC1);
    F = cvCreateMat (3, 3, CV_64FC1);
    rectification_H_left= cvCreateMat (3, 3, CV_64FC1);
    rectification_H_right= cvCreateMat (3, 3, CV_64FC1);

    remap_left_x=cvCreateMat(img_size.height, img_size.width,CV_32F);
    remap_left_y=cvCreateMat(img_size.height, img_size.width,CV_32F);
    remap_right_x=cvCreateMat(img_size.height, img_size.width,CV_32F);
    remap_right_y=cvCreateMat(img_size.height, img_size.width,CV_32F);

    cvSetIdentity(rectification_H_left);
    cvSetIdentity(rectification_H_right);
    cvSetIdentity(E);
    cvSetIdentity(F);
}

CvexStereoCameraCalibration::~CvexStereoCameraCalibration()
{
    cvReleaseMat(&rectification_H_left);
    cvReleaseMat(&rectification_H_right);

    cvReleaseMat(&relate_rot);
    cvReleaseMat(&relate_trans);

    cvReleaseMat(&remap_left_x);
    cvReleaseMat(&remap_left_y);
    cvReleaseMat(&remap_right_x);
    cvReleaseMat(&remap_right_y);

    cvReleaseMat(&E);
    cvReleaseMat(&F);
}

bool CvexStereoCameraCalibration::findChess(
    IplImage* left_img,
    IplImage* right_img)
{
    bool ret=false;
    ret = leftCamera.findChessboardCorners(left_img,false);
    if (ret)ret = rightCamera.findChessboardCorners(right_img,false);

    if (ret) {	
        leftCamera.findChessboardCorners(left_img,true);
        rightCamera.findChessboardCorners(right_img,true);
        p_count[image_count++] = PATTERN_SIZE;
    }	
    return ret;
}

void CvexStereoCameraCalibration::drawChessboardCorners(
    IplImage* src,
    IplImage* dest,
    int left_or_right)
{
    if (left_or_right==CVEX_STEREO_CALIB_LEFT) {
        leftCamera.drawChessboardCorners(src,dest);			
    }
    else {
        rightCamera.drawChessboardCorners(src,dest);			
    }
}

void CvexStereoCameraCalibration::showIntrinsicParameters()
{
    cout<<"Left Intrinsic Parameters:"<<endl;
    cvexShowMatrix(leftCamera.intrinsic);
    cout<<"Left Distortion Parameters:"<<endl;
    cvexShowMatrix(leftCamera.distortion);

    cout<<"Right Intrinsic Parameters:"<<endl;
    cvexShowMatrix(rightCamera.intrinsic);
    cout<<"Right Distortion Parameters:"<<endl;
    cvexShowMatrix(rightCamera.distortion);
}

void CvexStereoCameraCalibration::saveIntrinsicParameters(
    FILE * file)
{
    fprintf(file,"Left Intrinsic Parameters:\n");
    cvexSaveMatrix(leftCamera.intrinsic, file);
    fprintf(file,"Left Distortion Parameters:\n");
    cvexSaveMatrix(leftCamera.distortion, file);

    fprintf(file,"Right Intrinsic Parameters:\n");
    cvexSaveMatrix(rightCamera.intrinsic, file);
    fprintf(file,"Right Distortion Parameters:\n");
    cvexSaveMatrix(rightCamera.distortion, file);
}


void CvexStereoCameraCalibration::showRectificationHomography()
{
    cout<<"Left Rectification Homography:"<<endl;
    cvexShowMatrix(rectification_H_left);
    cout<<"Right Rectification Homography:"<<endl;
    cvexShowMatrix(rectification_H_right);
}

void CvexStereoCameraCalibration::saveRectificationHomography(
    FILE * file)
{
    fprintf(file,"Left Rectification Homography:\n");
    cvexSaveMatrix(rectification_H_left,file);
    fprintf(file,"Right Rectification Homography:\n");
    cvexSaveMatrix(rectification_H_right, file);
}

void CvexStereoCameraCalibration::showExtrinsicParameters()
{
    cout<<"Left Translation :"<<endl;
    cvexShowMatrix(leftCamera.translation);
    cout<<"Left Rotation:"<<endl;
    cvexShowMatrix(leftCamera.rotation);

    cout<<"Right Translation :"<<endl;
    cvexShowMatrix(rightCamera.translation);
    cout<<"Right Rotation:"<<endl;
    cvexShowMatrix(rightCamera.rotation);

    cout<<"Relative Rotation:"<<endl;
    cvexShowMatrix(relate_rot);
    cout<<"Relative Translation:"<<endl;
    cvexShowMatrix(relate_trans);

    cout<<"Essential Matrix:"<<endl;
    cvexShowMatrix(E);
    cout<<"Fundamental Matrix:"<<endl;
    cvexShowMatrix(F);
}

void CvexStereoCameraCalibration::saveExtrinsicParameters(
    FILE * file)
{
    fprintf(file,"Left Translation :\n");
    cvexSaveMatrix(leftCamera.translation, file);
    fprintf(file,"Left Rotation:\n");
    cvexSaveMatrix(leftCamera.rotation, file);

    fprintf(file,"Right Translation :\n");
    cvexSaveMatrix(rightCamera.translation, file);
    fprintf(file,"Right Rotation:\n");
    cvexSaveMatrix(rightCamera.rotation, file);

    fprintf(file,"Relative Rotation:\n");
    cvexSaveMatrix(relate_rot, file);
    fprintf(file,"Relative Translation:\n");
    cvexSaveMatrix(relate_trans, file);

    fprintf(file,"Essensial Matrix:\n");
    cvexSaveMatrix(E, file);
    fprintf(file,"Fundamental Matrix:\n");
    cvexSaveMatrix(F, file);
}

void CvexStereoCameraCalibration::rectifyImageRemap(
    IplImage* src,
    IplImage* dest,
    int left_right)
{
    IplImage* _dest;
    if (src ==dest) {
        _dest = cvCreateImage(cvGetSize(src),8,src->nChannels);
    }
    else {
        _dest = dest;
    }

    if (left_right==CVEX_STEREO_CALIB_LEFT) {
        cvRemap( src, _dest, remap_left_x, remap_left_y);
    }
    else {
        cvRemap( src, _dest, remap_right_x, remap_right_y);
    }

    if (src ==dest) {
        cvCopy(_dest,dest);
        cvReleaseImage(&_dest);
    }
}

void CvexStereoCameraCalibration::getRectificationMatrix(
    CvMat* h_left,
    CvMat* h_right,
    CvMat* Q)
{
    CvMat* R1 = cvCreateMat(3,3,CV_64F);
    CvMat* R2 = cvCreateMat(3,3,CV_64F);
    CvMat* P1 = cvCreateMat(3,4,CV_64F);
    CvMat* P2 = cvCreateMat(3,4,CV_64F);
    CvMat* k1 = cvCreateMat(3,3,CV_64F);
    CvMat* k2 = cvCreateMat(3,3,CV_64F);

    cvStereoRectify(
        leftCamera.intrinsic,rightCamera.intrinsic,
	leftCamera.distortion,rightCamera.distortion,image_size,
	relate_rot,relate_trans,R1,R2,P1,P2,Q,
        CV_CALIB_ZERO_DISPARITY, 0);

    for(int j=0;j<3;j++) {
        for(int i=0;i<3;i++) {
            cvmSet(k1,j,i,cvmGet(P1,j,i));
            cvmSet(k2,j,i,cvmGet(P2,j,i));
        }
    }

    getKRK(leftCamera.intrinsic,R1,k1,h_left);
    getKRK(rightCamera.intrinsic,R2,k2,h_right);

    //double mx = max(h_left->data.db[2],h_right->data.db[2]);

    cvCopy(h_left,rectification_H_left);
    cvCopy(h_right,rectification_H_right);

    cvInitUndistortRectifyMap(
        leftCamera.intrinsic, leftCamera.distortion,
        R1, P1,remap_left_x,remap_left_y);

    //cvexShowMatrix(remap_left_x);
    cvInitUndistortRectifyMap(
        rightCamera.intrinsic, rightCamera.distortion,
        R2, P2,remap_right_x, remap_right_y);

    cvReleaseMat(&k1);
    cvReleaseMat(&k2);

    cvReleaseMat(&R1);
    cvReleaseMat(&R2);
    cvReleaseMat(&P1);
    cvReleaseMat(&P2);
}

void CvexStereoCameraCalibration::solveStereoParameter()
{
    if (image_count<3) {
        cout<<"3 or more input images are required"<<endl;
        return;
    }

    CvMat object_points;
    CvPoint3D32f* objects= new CvPoint3D32f[image_count*PATTERN_SIZE];

    CvMat image_points1;
    CvPoint2D32f *corners1 = new CvPoint2D32f[image_count*PATTERN_SIZE];

    CvMat image_points2;
    CvPoint2D32f *corners2 = new CvPoint2D32f[image_count*PATTERN_SIZE];

    CvMat point_counts;

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
    cvInitMatHeader (&object_points, image_count*PATTERN_SIZE, 1, CV_32FC3, objects);
    cvInitMatHeader (&point_counts, image_count, 1, CV_32SC1, p_count);

    for (int i = 0; i < image_count; i++) {
        for (int j = 0; j < pattern_size.height; j++) {
            for (int k = 0; k < pattern_size.width; k++) {
                corners1[i * PATTERN_SIZE + j * pattern_size.width + k].x =
                    leftCamera.subpix_corners[i][j * pattern_size.width + k].x;
                corners1[i * PATTERN_SIZE + j * pattern_size.width + k].y =
                    leftCamera.subpix_corners[i][j * pattern_size.width + k].y;
            }
        }
    }

    cvInitMatHeader(
        &image_points1,
        image_count*PATTERN_SIZE, 1,
        CV_32FC2, corners1);

    cvCalibrateCamera2(
        &object_points,
        &image_points1,
        &point_counts,
        image_size,
        leftCamera.intrinsic,
        leftCamera.distortion);

    for (int i = 0; i < image_count; i++) {
        for (int j = 0; j < pattern_size.height; j++) {
            for (int k = 0; k < pattern_size.width; k++) {
                corners2[i * PATTERN_SIZE + j * pattern_size.width + k].x =
                    rightCamera.subpix_corners[i][j *
                    pattern_size.width + k].x;
                corners2[i * PATTERN_SIZE + j * pattern_size.width + k].y =
                    rightCamera.subpix_corners[i][j *
                    pattern_size.width + k].y;
            }
        }
    }

    cvInitMatHeader(
        &image_points2,
        image_count*PATTERN_SIZE, 1,
        CV_32FC2, corners2);

    cvCalibrateCamera2(
        &object_points,
        &image_points2,
        &point_counts,
        image_size,
        rightCamera.intrinsic,
        rightCamera.distortion);

    cvStereoCalibrate(
        &object_points,&image_points1,&image_points2,&point_counts,
        leftCamera.intrinsic,leftCamera.distortion,
        rightCamera.intrinsic,rightCamera.distortion,
        image_size,relate_rot,relate_trans,E,F);

    delete [] objects;
    delete [] corners1;
    delete [] corners2;
}

