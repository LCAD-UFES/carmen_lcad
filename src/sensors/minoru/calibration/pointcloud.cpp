/*
    Point cloud functions
    Copyright (C) 2010 Bob Mottram and Giacomo Spigler
    fuzzgun@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pointcloud.h"
#include <opencv2/calib3d/calib3d_c.h>

CvMat* pointcloud::matMul(const CvMat* A, const CvMat* B) 
{
    assert(A->cols == B->rows);

    CvMat* M = cvCreateMat(A->rows, B->cols, A->type);
    cvMatMul(A, B, M);
    return M;
}

/*
  Format: An 11 float header, followed by three floats and three bytes per point
*/
void pointcloud::save(
    unsigned char * img_left,
    IplImage * points_image, 
    int max_range_mm,
    CvMat * pose,
    float baseline,
    std::string point_cloud_filename)
{
    float pose_x = (float)cvmGet(pose,0,3);
    float pose_y = (float)cvmGet(pose,1,3);
    float pose_z = (float)cvmGet(pose,2,3);

    FILE * fp = fopen(point_cloud_filename.c_str(),"wb");
    if (fp != NULL) {
        float * points_image_data = (float*)points_image->imageData;
        int pixels = points_image->width * points_image->height;
        int n = 0, ctr = 0;
        for (int i = 0; i < pixels; i++, n += 3) {
            float dx = points_image_data[n] - pose_x;
            float dy = points_image_data[n+1] - pose_y;
            float dz = points_image_data[n+2] - pose_z;

            if ((fabs(dx)+fabs(dy)+fabs(dz) > 1) &&
                (fabs(dx) < max_range_mm) &&
                (fabs(dy) < max_range_mm) &&
                (fabs(dz) < max_range_mm)) {
                ctr++;
            }
        }
        if (ctr > 0) {
            float * header = new float[11];
            header[0] = (float)POINT_CLOUD_VERSION;
            header[1] = ctr;
            header[2] = points_image->width;
            header[3] = points_image->height;
            header[4] = cvmGet(pose,0,3);
            header[5] = cvmGet(pose,1,3);
            header[6] = cvmGet(pose,2,3);

            CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);
            CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
            for (int y = 0; y < 3; y++) {
                for (int x = 0; x < 3; x++) {
                    cvmSet(rotation_matrix, y, x, cvmGet(pose, y, x));
                }
            }
            cvRodrigues2(rotation_matrix, rotation_vector);
            header[7] = cvmGet(rotation_vector,0,0);
            header[8] = cvmGet(rotation_vector,1,0);
            header[9] = cvmGet(rotation_vector,2,0);
            header[10] = baseline;

            fwrite(header,sizeof(float),11,fp);
            cvReleaseMat(&rotation_matrix);
            cvReleaseMat(&rotation_vector);

            int elem_bytes = (sizeof(float)*3) + 3;
            unsigned char * data_bytes = new unsigned char[elem_bytes*ctr];
            memset((void*)data_bytes, '\0', elem_bytes * ctr);
            ctr = 0;
            n = 0;
            for (int i = 0; i < pixels; i++, n += 3) {
                float dx = points_image_data[n] - pose_x;
                float dy = points_image_data[n+1] - pose_y;
                float dz = points_image_data[n+2] - pose_z;

                if ((fabs(dx) + fabs(dy) + fabs(dz) > 1) &&
                    (fabs(dx) < max_range_mm) &&
                    (fabs(dy) < max_range_mm) &&
                    (fabs(dz) < max_range_mm)) {
            
                    float * data = (float*)&data_bytes[elem_bytes*ctr];
                    data[0] = dx+pose_x;
                    data[1] = dy+pose_y;
                    data[2] = dz+pose_z;

                    int n2 = (elem_bytes*ctr) + (3*sizeof(float));
                    data_bytes[n2] = img_left[n];
                    data_bytes[n2+1] = img_left[n+1];
                    data_bytes[n2+2] = img_left[n+2];
                    ctr++;
                }
            }
            fwrite(data_bytes,elem_bytes,ctr,fp);
            printf("%d points saved\n", ctr);
            delete [] data_bytes;
            delete [] header;
        }
        fclose(fp);
    }
}

/*
  Format: An 11 float header, followed by three floats and three bytes per point
*/
void pointcloud::save(
    std::vector<float> &point,
    std::vector<unsigned char> &point_colour,
    int max_range_mm,
    CvMat * pose,
    int image_width,
    int image_height,
    float baseline,
    std::string point_cloud_filename)
{
    float pose_x = (float)cvmGet(pose,0,3);
    float pose_y = (float)cvmGet(pose,1,3);
    float pose_z = (float)cvmGet(pose,2,3);

    FILE * fp = fopen(point_cloud_filename.c_str(),"wb");
    if (fp != NULL) {
        int no_of_points = (int)point.size()/3;
        int n = 0, ctr = 0;
        for (int i = 0; i < no_of_points; i++, n += 3) {
            float dx = point[n] - pose_x;
            float dy = point[n+1] - pose_y;
            float dz = point[n+2] - pose_z;

            if ((fabs(dx)+fabs(dy)+fabs(dz) > 1) &&
                (fabs(dx) < max_range_mm) &&
                (fabs(dy) < max_range_mm) &&
                (fabs(dz) < max_range_mm)) {
                ctr++;
            }
        }
        if (ctr > 0) {
            float * header = new float[11];
            header[0] = (float)POINT_CLOUD_VERSION;
            header[1] = ctr;
            header[2] = image_width;
            header[3] = image_height;
            header[4] = cvmGet(pose,0,3);
            header[5] = cvmGet(pose,1,3);
            header[6] = cvmGet(pose,2,3);

            CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);
            CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
            for (int y = 0; y < 3; y++) {
                for (int x = 0; x < 3; x++) {
                    cvmSet(rotation_matrix, y, x, cvmGet(pose, y, x));
                }
            }
            cvRodrigues2(rotation_matrix, rotation_vector);
            header[7] = cvmGet(rotation_vector,0,0);
            header[8] = cvmGet(rotation_vector,1,0);
            header[9] = cvmGet(rotation_vector,2,0);
            header[10] = baseline;

            fwrite(header,sizeof(float),11,fp);
            cvReleaseMat(&rotation_matrix);
            cvReleaseMat(&rotation_vector);

            int elem_bytes = (sizeof(float)*3) + 3;
            unsigned char * data_bytes = new unsigned char[elem_bytes*ctr];
            memset((void*)data_bytes, '\0', elem_bytes * ctr);
            ctr = 0;
            n = 0;
            for (int i = 0; i < no_of_points; i++, n += 3) {
                float dx = point[n] - pose_x;
                float dy = point[n+1] - pose_y;
                float dz = point[n+2] - pose_z;

                if ((fabs(dx) + fabs(dy) + fabs(dz) > 1) &&
                    (fabs(dx) < max_range_mm) &&
                    (fabs(dy) < max_range_mm) &&
                    (fabs(dz) < max_range_mm)) {
            
                    float * data = (float*)&data_bytes[elem_bytes*ctr];
                    data[0] = dx+pose_x;
                    data[1] = dy+pose_y;
                    data[2] = dz+pose_z;

                    int n2 = (elem_bytes*ctr) + (3*sizeof(float));
                    data_bytes[n2] = point_colour[n];
                    data_bytes[n2+1] = point_colour[n+1];
                    data_bytes[n2+2] = point_colour[n+2];
                    ctr++;
                }
            }
            fwrite(data_bytes,elem_bytes,ctr,fp);
            printf("%d points saved\n", ctr);
            delete [] data_bytes;
            delete [] header;
        }
        fclose(fp);
    }
}

bool pointcloud::load(
     std::string point_cloud_filename,
     CvMat * &pose,
     int &image_width,
     int &image_height,
     float &baseline,
     std::vector<float> &point,
     std::vector<unsigned char> &point_colour)
{
    bool success = true;
    FILE * fp = fopen(point_cloud_filename.c_str(),"rb");
    if (fp==NULL) return false;
    float * header = new float[11];
    size_t retval = fread(header, sizeof(float), 11, fp);
    if (retval != 11) return false;

    //float version = header[0];
    if (pose == NULL) pose = cvCreateMat(4, 4, CV_32F);
    CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
    CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);

    int no_of_points = 0, elem_bytes = 0;
    image_width = 0;
    image_height = 0;
    
    elem_bytes = (sizeof(float)*3) + 3;
    no_of_points = (int)header[1];
    image_width = (int)header[2];
    image_height = (int)header[3];
    cvmSet(pose,0,3,header[4]);
    cvmSet(pose,1,3,header[5]);
    cvmSet(pose,2,3,header[6]);
    cvmSet(rotation_vector,0,0,header[7]);
    cvmSet(rotation_vector,1,0,header[8]);
    cvmSet(rotation_vector,2,0,header[9]);
    baseline = header[10];
    cvRodrigues2(rotation_vector,rotation_matrix);
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(rotation_matrix, y, x, cvmGet(pose, y, x));
        }
    }

    if (no_of_points > 0) {
        unsigned char * data_bytes = new unsigned char[elem_bytes * no_of_points];
        retval = fread(data_bytes,elem_bytes,no_of_points,fp);
        for (int i = 0; i < no_of_points; i++) {
            float * data = (float*)&data_bytes[i*elem_bytes];
            point.push_back(data[0]);
            point.push_back(data[1]);
            point.push_back(data[2]);
            int n = (i*elem_bytes) + (3*sizeof(float));
            point_colour.push_back(data_bytes[n]);
            point_colour.push_back(data_bytes[n+1]);
            point_colour.push_back(data_bytes[n+2]);
        }
        delete [] data_bytes;
    }

    cvReleaseMat(&rotation_vector);
    cvReleaseMat(&rotation_matrix);
    delete [] header;
    return success;
}

void pointcloud::disparity_map_to_3d_points(
    float * disparity_map,
    int img_width,
    int img_height,
    CvMat * disparity_to_depth,
    CvMat * pose,
    IplImage * &disparity_image,
    IplImage * &points_image)
{
    if (disparity_to_depth != NULL) {
        // create image objects
        if (points_image == NULL) {
            points_image = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_32F, 3);
        }
        if (disparity_image == NULL) {
            disparity_image = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_32F, 1);
        }

        // insert disparities into the image object
        float * disparity_image_data = (float*)disparity_image->imageData;
        for (int i = 0; i < img_width*img_height; i++) {
            disparity_image_data[i] = disparity_map[i];
        }

        // reproject the points in an egocentric coordinate frame
        cvReprojectImageTo3D(disparity_image, points_image, disparity_to_depth);

        // reposition the points according to the camera pose
        float * points_image_data = (float*)points_image->imageData;
        int n = 0;
        float xx = cvmGet(pose,0,3);
        float yy = cvmGet(pose,1,3);
        float zz = cvmGet(pose,2,3);
        for (int i = 0; i < img_width*img_height; i++, n += 3) {
            if (disparity_map[i] > 0) {
                float px = (cvmGet(pose, 0, 0)*points_image_data[n] + cvmGet(pose, 0, 1)*points_image_data[n+1] + cvmGet(pose, 0, 2)*points_image_data[n+2]);
                float py = (cvmGet(pose, 1, 0)*points_image_data[n] + cvmGet(pose, 1, 1)*points_image_data[n+1] + cvmGet(pose, 1, 2)*points_image_data[n+2]);
                float pz = (cvmGet(pose, 2, 0)*points_image_data[n] + cvmGet(pose, 2, 1)*points_image_data[n+1] + cvmGet(pose, 2, 2)*points_image_data[n+2]);
                points_image_data[n] = xx + px;
                points_image_data[n+1] = yy + py;
                points_image_data[n+2] = zz + pz;
            }
            else {
                points_image_data[n] = xx;
                points_image_data[n+1] = yy;
                points_image_data[n+2] = zz;
            }
        }
    }
}

void pointcloud::show(
    IplImage * points_image,
    float * disparity_map,
    unsigned char * img_left,
    CvMat * pose,
    float max_range_mm,
    float max_height_mm,
    int view_type,
    int output_image_width,
    int output_image_height,
    unsigned char * img_output)
{
    float cx = cvmGet(pose,0,3);
    float cy = cvmGet(pose,1,3);
    float cz = cvmGet(pose,2,3);
    float px,py,pz;
    int ix=0,iy=0,prev_ix,prev_iy,dx,dy,length;
    int n = 0;
    float * points_image_data = (float*)points_image->imageData;

    memset((void*)img_output,'\0',output_image_width*output_image_height*3);

    for (int y = 0; y < points_image->height; y++) {
        prev_ix = prev_iy = 0;
        for (int x = 0; x < points_image->width; x++, n++) {
            if (disparity_map[n] > 1) {
                px = points_image_data[n*3] - cx;
                py = points_image_data[n*3 + 1] - cy;
                pz = points_image_data[n*3 + 2] - cz;
                switch(view_type) {
                    case 0: {
                        ix = (int)((output_image_width>>1) + (px * output_image_width / (max_range_mm*2)));
                        iy = (int)((output_image_height>>1) - (pz * output_image_width / (max_height_mm*2)));
                        break;
                    }
                    case 1: {
                        ix = (int)(py * output_image_width / max_range_mm);
                        iy = (int)((output_image_height>>1) - (pz * output_image_width / max_height_mm));
                        break;
                    }
                    case 2: {
                        ix = (int)((output_image_width>>1) + (px * output_image_width / max_range_mm));
                        iy = (int)(py * output_image_width / max_range_mm);
                        break;
                    }
                }
                if ((!((prev_ix == 0) && (prev_iy == 0))) &&
                    ((ix > 0) && (ix < output_image_width) &&
                     (iy > 0) && (iy < output_image_height))) {
                    dx = ix - prev_ix;
                    dy = iy - prev_iy;
                    length = (int)sqrt(dx*dx + dy*dy);
                    if (length>2) length=2;
                    for (int i = 0; i < length; i++) {
                        int ixx = prev_ix + (i * dx / length);
                        int iyy = prev_iy + (i * dy / length);
                        if ((ixx > 0) && (ixx < output_image_width) &&
                            (iyy > 0) && (iyy < output_image_height)) {
                            int nn = (iyy*output_image_width + ixx)*3;
                            img_output[nn] = img_left[n*3];
                            img_output[nn+1] = img_left[n*3+1];
                            img_output[nn+2] = img_left[n*3+2];
                        }
                    }
                }
                prev_ix = ix;
                prev_iy = iy;
            }
        }
    }
}


void pointcloud::virtual_camera(
    unsigned char * img,
    IplImage * points_image,
    CvMat * pose,
    CvMat * intrinsic_matrix,
    CvMat * distortion_coeffs,
    float max_range_mm,
    bool view_point_cloud,
    unsigned char * img_output)
{
    int pixels = points_image->width*points_image->height;
    float * depth = new float[pixels];
    CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);
    CvMat * translation = cvCreateMat(3, 1, CV_32F);
    CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
    CvMat * points = cvCreateMat(pixels, 3, CV_32F);
    CvMat * image_points = cvCreateMat(pixels, 2, CV_32F);

    virtual_camera(
        img, points_image, pose,
        intrinsic_matrix, distortion_coeffs,
        max_range_mm, depth,
        rotation_matrix, translation, rotation_vector,
        points, image_points,
        view_point_cloud,
        img_output);

    cvReleaseMat(&rotation_matrix);
    cvReleaseMat(&translation);
    cvReleaseMat(&rotation_vector);
    cvReleaseMat(&points);
    cvReleaseMat(&image_points);
    delete [] depth;
}



void pointcloud::virtual_camera(
    unsigned char * img,
    IplImage * points_image,
    CvMat * pose,
    CvMat * intrinsic_matrix,
    CvMat * distortion_coeffs,
    float max_range_mm,
    float * &depth,
    CvMat * &rotation_matrix,
    CvMat * &translation,
    CvMat * &rotation_vector,
    CvMat * &points,
    CvMat * &image_points,
    bool view_point_cloud,
    unsigned char * img_output)
{
    int pixels = points_image->width*points_image->height;
    max_range_mm *= max_range_mm;

    if (depth == NULL) {
        depth = new float[pixels];
        rotation_matrix = cvCreateMat(3, 3, CV_32F);
        translation = cvCreateMat(3, 1, CV_32F);
        rotation_vector = cvCreateMat(3, 1, CV_32F);
        points = cvCreateMat(pixels, 3, CV_32F);
        image_points = cvCreateMat(pixels, 2, CV_32F);
    }

    // view rotation
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(rotation_matrix,y,x,cvmGet(pose,y,x));
        }
    }
    // view translation
    for (int i = 0; i < 3; i++) {
        cvmSet(translation,i,0,cvmGet(pose,i,3));
    }
    // convert from 3x3 to 3x1
    cvRodrigues2(rotation_matrix, rotation_vector);

    float * points_image_data = (float*)points_image->imageData;

    // set 3D points
    for (int i = 0; i < pixels; i++) {
        cvmSet(points,i,0,points_image_data[i*3]);
        cvmSet(points,i,1,points_image_data[i*3+1]);
        cvmSet(points,i,2,points_image_data[i*3+2]);
    }

    // project points
    cvProjectPoints2(
        points,
        rotation_vector, translation,
        intrinsic_matrix,
        distortion_coeffs,
        image_points);

    // draw
    memset((void*)img_output,'\0',pixels*3);
    memset((void*)depth,'\0',pixels*sizeof(float));
    int prev_i=-1,prev_x=-1,prev_y=-1,length;
    float prev_dist=0;
    int w = points_image->width;
    int h = points_image->height;
    int step = 1;
    if (view_point_cloud) step = 2;
    for (int py = 0; py < h; py += step) {
        for (int px = 0; px < w; px += step) {
            int i = py*w + px;
            int x = (int)cvmGet(image_points,i,0);
            if ((x >=0) && (x < w)) {
                int y = (int)cvmGet(image_points,i,1);
                if ((y >=0) && (y < points_image->height-2)) {
                    int n = y*w + x;//(w-1-x);

                    float dx = points_image_data[i*3] - cvmGet(translation,0,0);
                    float dy = points_image_data[i*3+1] - cvmGet(translation,1,0);
                    float dz = points_image_data[i*3+2] - cvmGet(translation,2,0);
                    float dist = dx*dx + dy*dy + dz*dz;
                    if ((dist > 0) && (dist < max_range_mm)) {
                        if (depth[n] == 0) {
                            depth[n] = dist;

                            if (!view_point_cloud) {
                                img_output[n*3] = img[i*3];
                                img_output[n*3+1] = img[i*3+1];
                                img_output[n*3+2] = img[i*3+2];
                                img_output[(n+points_image->width)*3] = img[i*3];
                                img_output[(n+points_image->width)*3+1] = img[i*3+1];
                                img_output[(n+points_image->width)*3+2] = img[i*3+2];
                            }
                            else {
                                img_output[n*3] = 255;
                                img_output[n*3+1] = 255;
                                img_output[n*3+2] = 255;
                            }

                            dist = (float)sqrt(dist);

                            if ((prev_x>-1) && (i-prev_i>0) && (i-prev_i<5) && (fabs(dist-prev_dist)<100)) {
                                int dx2 = x - prev_x;
                                int dy2 = y - prev_y;
                                length = (int)sqrt(dx2*dx2+dy2*dy2);
                                if (length>0) {
                                    for (int l = 0; l <= length; l++) {
                                        int xx = prev_x + (l*dx2/length);
                                        int yy = prev_y + (l*dy2/length);
                                        int n2 = yy*w + xx;//(w-1-xx);
                                        int i2 = prev_i + (l*(i-prev_i)/length);
                                        if (!view_point_cloud) {
                                            img_output[n2*3] = img[i2*3];
                                            img_output[n2*3+1] = img[i2*3+1];
                                            img_output[n2*3+2] = img[i2*3+2];
                                            img_output[(n2+points_image->width)*3] = img[i2*3];
                                            img_output[(n2+points_image->width)*3+1] = img[i2*3+1];
                                            img_output[(n2+points_image->width)*3+2] = img[i2*3+2];
                                        }
                                    }
                                }
                            }
                            prev_i = i;
                            prev_x = x;
                            prev_y = y;
                            prev_dist = dist;
                        }
                        else {
                            if (dist < depth[n]) {
                                depth[n] = dist;

                                if (!view_point_cloud) {
                                    img_output[n*3] = img[i*3];
                                    img_output[n*3+1] = img[i*3+1];
                                    img_output[n*3+2] = img[i*3+2];
                                    img_output[(n+points_image->width)*3] = img[i*3];
                                    img_output[(n+points_image->width)*3+1] = img[i*3+1];
                                    img_output[(n+points_image->width)*3+2] = img[i*3+2];
                                }
                                else {
                                    img_output[n*3] = 255;
                                    img_output[n*3+1] = 255;
                                    img_output[n*3+2] = 255;
                                }

                                dist = (float)sqrt(dist);

                                if ((prev_x>-1) && (i-prev_i>0) && (i-prev_i<5) && (fabs(dist-prev_dist)<100)) {
                                    int dx2 = x - prev_x;
                                    int dy2 = y - prev_y;
                                    length = (int)sqrt(dx2*dx2+dy2*dy2);
                                    for (int l = 0; l < length; l++) {
                                        int xx = prev_x + (l*dx2/length);
                                        int yy = prev_y + (l*dy2/length);
                                        int n2 = yy*w + xx;//(w-1-xx);
                                        int i2 = prev_i + (l*(i-prev_i)/length);
                                        if (!view_point_cloud) {
                                            img_output[n2*3] = img[i2*3];
                                            img_output[n2*3+1] = img[i2*3+1];
                                            img_output[n2*3+2] = img[i2*3+2];
                                            img_output[(n2+points_image->width)*3] = img[i2*3];
                                            img_output[(n2+points_image->width)*3+1] = img[i2*3+1];
                                            img_output[(n2+points_image->width)*3+2] = img[i2*3+2];
                                        }
                                    }
                                }
                                prev_i = i;
                                prev_x = x;
                                prev_y = y;
                                prev_dist = dist;
                            }
                        }
                    }
                }
            }
        }
    }
}

void pointcloud::virtual_camera_show_axes(
    CvMat * intrinsic_matrix,
    CvMat * distortion_coeffs,
    CvMat * &translation,
    CvMat * &rotation_vector,
    int image_width,
    int image_height,
    unsigned char * img_output)
{
    CvMat * axis_points = cvCreateMat(4, 3, CV_32F);
    CvMat * axis_image_points = cvCreateMat(4, 2, CV_32F);

    float length = 2000;

    cvmSet(axis_points,0,0,0.0f);
    cvmSet(axis_points,0,1,0.0f);
    cvmSet(axis_points,0,2,0.0f);

    cvmSet(axis_points,1,0,length);
    cvmSet(axis_points,1,1,0.0f);
    cvmSet(axis_points,1,2,0.0f);

    cvmSet(axis_points,2,0,0.0f);
    cvmSet(axis_points,2,1,length);
    cvmSet(axis_points,2,2,0.0f);

    cvmSet(axis_points,3,0,0.0f);
    cvmSet(axis_points,3,1,0.0f);
    cvmSet(axis_points,3,2,length);

    // project points
    cvProjectPoints2(
        axis_points,
        rotation_vector, translation,
        intrinsic_matrix,
        distortion_coeffs,
        axis_image_points);

    float cx = (float)cvmGet(axis_image_points,0,0);
    float cy = (float)cvmGet(axis_image_points,0,1);
    int r=0,g=0,b=0;
    for (int axis = 0; axis < 3; axis++) {
        float x = (float)cvmGet(axis_image_points,axis+1,0);
        float y = (float)cvmGet(axis_image_points,axis+1,1);
        float dx = x - cx;
        float dy = y - cy;
        float hyp = (float)sqrt(dx*dx + dy*dy);

        switch(axis) {
            case 0: {
                r = 255;
                g = b = 0;
                break;
            }
            case 1: {
                g = 255;
                r = b = 0;
                break;
            }
            case 2: {
                b = 255;
                r = g = 0;
                break;
            }
        }

        for (int i = 0; i < image_width*3; i++) {
            int xx = (int)(cx + (i*dx/hyp));
            if ((xx >= 0) && (xx < image_width)) {
                int yy = (int)(cy + (i*dy/hyp));            
                if ((yy >= 0) && (yy < image_height)) {
                    int n = (yy*image_width + (image_width-1-xx))*3;
                    img_output[n] = b;
                    img_output[n+1] = g;
                    img_output[n+2] = r;
                }
            }
        }
    }

    cvReleaseMat(&axis_points);
    cvReleaseMat(&axis_image_points);
}

void pointcloud::prune(
    std::vector<float> &point,
    std::vector<unsigned char> &point_colour,
    int map_dimension_mm,
    int map_height_mm,
    int cell_size_mm,
    int camera_height_mm,
    int min_threshold)
{
    int dimension = map_dimension_mm / cell_size_mm;
    int height = map_height_mm / cell_size_mm;
    unsigned char * occupancy = new unsigned char[dimension*dimension*height];
    int half_dimension = dimension/2;    
    memset((void*)occupancy,'\0',dimension*dimension*height*sizeof(unsigned char));
    
    for (int i = (int)point.size()-3; i >= 0; i -= 3) {
        int x = half_dimension + (point[i + POINT_CLOUD_X_AXIS] / cell_size_mm);
        if ((x >= 0) && (x < dimension)) {
            int y = half_dimension + (point[i + POINT_CLOUD_Y_AXIS] / cell_size_mm);
            if ((y >= 0) && (y < dimension)) {
                int z = (camera_height_mm + point[i + POINT_CLOUD_Z_AXIS]) / cell_size_mm;
                if ((z >= 0) && (z < height)) {
                    int n = (z*dimension*dimension) + (y*dimension) + x;
                    if (occupancy[n] < 255) {
                        occupancy[n]++;
                    }
                }
            }
        }
    }

    for (int i = (int)point.size()-3; i >= 0; i -= 3) {
        int x = half_dimension + (point[i + POINT_CLOUD_X_AXIS] / cell_size_mm);
        if ((x >= 0) && (x < dimension)) {
            int y = half_dimension + (point[i + POINT_CLOUD_Y_AXIS] / cell_size_mm);
            if ((y >= 0) && (y < dimension)) {
                int z = (camera_height_mm + point[i + POINT_CLOUD_Z_AXIS]) / cell_size_mm;
                if ((z >= 0) && (z < height)) {
                    int n = (z*dimension*dimension) + (y*dimension) + x;
                    if (occupancy[n] < min_threshold) {
                        for (int p = 2; p >= 0; p--) {
                            point.erase(point.begin()+i+p);
                            point_colour.erase(point_colour.begin()+i+p);
                        }
                    }
                }
            }
        }
    }

    delete [] occupancy;
}

void pointcloud::virtual_camera(
    std::vector<float> &point,
    std::vector<unsigned char> &point_colour,
    CvMat * pose,
    CvMat * intrinsic_matrix,
    CvMat * distortion_coeffs,
    float max_range_mm,
    float * &depth,
    CvMat * &rotation_matrix,
    CvMat * &translation,
    CvMat * &rotation_vector,
    CvMat * &points,
    CvMat * &image_points,
    bool view_point_cloud,
    bool show_axes,
    int image_width,
    int image_height,
    unsigned char * img_output)
{
    int no_of_points = (int)point.size()/3;
    max_range_mm *= max_range_mm;

    if (depth == NULL) {
        depth = new float[image_width*image_height];
        rotation_matrix = cvCreateMat(3, 3, CV_32F);
        translation = cvCreateMat(3, 1, CV_32F);
        rotation_vector = cvCreateMat(3, 1, CV_32F);
        points = cvCreateMat(no_of_points, 3, CV_32F);
        image_points = cvCreateMat(no_of_points, 2, CV_32F);
    }

    // view rotation
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(rotation_matrix,y,x,cvmGet(pose,y,x));
        }
    }
    // view translation
    for (int i = 0; i < 3; i++) {
        cvmSet(translation,i,0,cvmGet(pose,i,3));
    }
    // convert from 3x3 to 3x1
    cvRodrigues2(rotation_matrix, rotation_vector);

    // set 3D points
    for (int i = 0; i < no_of_points; i++) {
        cvmSet(points,i,0,point[i*3]);
        cvmSet(points,i,1,point[i*3+1]);
        cvmSet(points,i,2,point[i*3+2]);
    }

    // project points
    cvProjectPoints2(
        points,
        rotation_vector, translation,
        intrinsic_matrix,
        distortion_coeffs,
        image_points);

    // draw
    memset((void*)img_output,'\0',image_width*image_height*3);
    memset((void*)depth,'\0',image_width*image_height*sizeof(float));

    float pose_x = cvmGet(translation,0,0);
    float pose_y = cvmGet(translation,1,0);
    float pose_z = cvmGet(translation,2,0);

    for (int i = 0; i < no_of_points; i++) {
        int x = (int)cvmGet(image_points,i,0);
        if ((x >=0) && (x < image_width)) {
            int y = (int)cvmGet(image_points,i,1);
            if ((y >=0) && (y < image_height-1)) {
                int n = y*image_width + x;//(image_width-1-x);

                float dx = point[i*3] - pose_x;
                float dy = point[i*3+1] - pose_y;
                float dz = point[i*3+2] - pose_z;
                float dist = dx*dx + dy*dy + dz*dz;
                if ((dist > 0) && (dist < max_range_mm)) {
                    bool show = false;
                    if (depth[n] == 0) {
                        depth[n] = dist;
                        show = true;
                    }
                    else {
                        if (dist < depth[n]) {
                            depth[n] = dist;
                            show = true;
                        }
                    }

                    if (show) {
                        if (!view_point_cloud) {
                            img_output[n*3] = point_colour[i*3];
                            img_output[n*3+1] = point_colour[i*3+1];
                            img_output[n*3+2] = point_colour[i*3+2];
                            img_output[(n+image_width)*3] = point_colour[i*3];
                            img_output[(n+image_width)*3+1] = point_colour[i*3+1];
                            img_output[(n+image_width)*3+2] = point_colour[i*3+2];
                        }
                        else {
                            img_output[n*3] = 255;
                            img_output[n*3+1] = 255;
                            img_output[n*3+2] = 255;
                        }
                    }
                }
            }
        }
    }

    if (show_axes) {
        virtual_camera_show_axes(
            intrinsic_matrix,
            distortion_coeffs,
            translation,
            rotation_vector,
            image_width, image_height,
            img_output);
    }
}

void pointcloud::fill(
    int id,
    int * map,
    int map_dimension,
    int x,
    int y,
    int depth,
    int &ctr,
    int threshold)
{
    map[y*map_dimension + x] = id;
    if (depth > 90000) return;
    ctr++;
    if (y > 0) {
        // N
        if (map[(y-1)*map_dimension + x] >= threshold) {
            fill(id,map,map_dimension,x,y-1,depth+1,ctr,threshold);
        }
        // NE
        if (x < map_dimension-1) {
            if (map[(y-1)*map_dimension + x + 1] >= threshold) {
                fill(id,map,map_dimension,x+1,y-1,depth+1,ctr,threshold);
            }
        }
    }
    if (x < map_dimension-1) {
        // E
        if (map[y*map_dimension + x + 1] >= threshold) {
            fill(id,map,map_dimension,x+1,y,depth+1,ctr,threshold);
        }
        // SE
        if (y < map_dimension-1) {
            if (map[(y+1)*map_dimension + x + 1] >= threshold) {
                fill(id,map,map_dimension,x+1,y+1,depth+1,ctr,threshold);
            }
        }
    }
    if (y < map_dimension-1) {
        // S
        if (map[(y+1)*map_dimension + x] >= threshold) {
            fill(id,map,map_dimension,x,y+1,depth+1,ctr,threshold);
        }
        // SW
        if (x > 0) {
            if (map[(y+1)*map_dimension + x - 1] >= threshold) {
                fill(id,map,map_dimension,x-1,y+1,depth+1,ctr,threshold);
            }
        }
    }
    if (x > 0) {
        // W
        if (map[y*map_dimension + x - 1] >= threshold) {
            fill(id,map,map_dimension,x-1,y,depth+1,ctr,threshold);
        }
        // NW
        if (y > 0) {
            if (map[(y-1)*map_dimension + x - 1] >= threshold) {
                fill(id,map,map_dimension,x-1,y-1,depth+1,ctr,threshold);
            }
        }
    }
}

int pointcloud::get_object_id(
    int x,
    int y,
    int width,
    float cos_tilt,
    float sin_tilt,
    int centre,
    float mult,
    float relative_x_mm,
    float relative_y_mm,
    int map_dimension,
    int threshold,
    int * map,
    float * points_image_data,
    float pose_x,
    float pose_y,
    float pose_z,
    float &x2,
    float &y2,
    float &z2)
{
    int id = 0;
    int i = y*width + x;

    if ((points_image_data[i*3+POINT_CLOUD_X_AXIS]==0) &&
        (points_image_data[i*3+POINT_CLOUD_Y_AXIS]==0) &&
        (points_image_data[i*3+POINT_CLOUD_Z_AXIS]==0)) return 0;

    if ((std::isnan(points_image_data[i*3+POINT_CLOUD_X_AXIS])) ||
        (std::isnan(points_image_data[i*3+POINT_CLOUD_Y_AXIS])) ||
        (std::isnan(points_image_data[i*3+POINT_CLOUD_Z_AXIS]))) return 0;

    float dx = points_image_data[i*3+POINT_CLOUD_X_AXIS] - pose_x;
    float dy = points_image_data[i*3+POINT_CLOUD_Y_AXIS] - pose_y;
    float dz = points_image_data[i*3+POINT_CLOUD_Z_AXIS] - pose_z;
            
    x2 = dx;
    y2 = cos_tilt*dy - sin_tilt*dz;
    z2 = sin_tilt*dy + cos_tilt*dz;

    int mx = centre + (int)((x2 + relative_x_mm) * mult);
    if ((mx >= 0) && (mx < map_dimension)) {
        int my = centre + (int)((y2 + relative_y_mm) * mult);
        if ((my >= 0) && (my < map_dimension)) {
            int n = my*map_dimension + mx;
            if ((map[n] > 0) && (map[n] < threshold)) {
                id = map[n];
            }
        }
    }
    return id;
}

void pointcloud::find_objects(
    int format,
    unsigned char * img,
    IplImage * points_image,
    int map_dimension,
    int map_cell_size_mm,
    CvMat * pose,
    float relative_x_mm,
    float relative_y_mm,
    int threshold,
    float tilt_degrees,
    int * map,
    int min_area_mm2,
    int max_area_mm2,
    bool BGR,
    std::vector<std::vector<float> > &objects)
{
    for (int i = 0; i < map_dimension*map_dimension; i++) {
        if (map[i] < threshold) map[i]=0;
    }
    int id=1;
    int min_area = min_area_mm2 / map_cell_size_mm;
    int max_area = max_area_mm2 / map_cell_size_mm;
    int n=0;
    for (int map_y = 0; map_y < map_dimension; map_y++) {
        for (int map_x = 0; map_x < map_dimension; map_x++, n++) {
            if (map[n] >= threshold) {
                int ctr = 0;
                int depth=0;               
                fill(id,map,map_dimension,map_x,map_y,depth,ctr,threshold);
                if ((ctr < min_area) || (ctr > max_area)) {
                    for (int i = 0; i < map_dimension*map_dimension; i++) {
                        if (map[i] == id) map[i] = 0;
                    }  
                }
                else {
                    id++;
                }
            }
        }
    }
    if (id == 1) return;

    for (int i = 0; i < id-1; i++) {
        std::vector<float> points;
        objects.push_back(points);
    }

    float * points_image_data = (float*)points_image->imageData;
    float pose_x = (float)cvmGet(pose,POINT_CLOUD_X_AXIS,3);
    float pose_y = (float)cvmGet(pose,POINT_CLOUD_Y_AXIS,3);
    float pose_z = (float)cvmGet(pose,POINT_CLOUD_Z_AXIS,3);
    float cos_tilt = (float)cos(-tilt_degrees/180.0*3.1415927);
    float sin_tilt = (float)sin(-tilt_degrees/180.0*3.1415927);
    int centre = map_dimension/2;
    int w = points_image->width;
    int max_range_mm = map_dimension * map_cell_size_mm;
    float mult = map_dimension / (float)max_range_mm;
    int prev_x=0,prev_id=0;
    float x2=0,y2=0,z2=0,x3=0,y3=0,z3=0,x4=0,y4=0,z4=0,x5=0,y5=0,z5=0;
    for (int y = 1; y < points_image->height-1; y++) {
        for (int x = 1; x < w-1; x++) {
            int i = y*w + x;

            id = get_object_id(
                x,y,w,
                cos_tilt, sin_tilt,
                centre, mult,
                relative_x_mm, relative_y_mm,
                map_dimension, threshold, map,
                points_image_data,
                pose_x, pose_y, pose_z,
                x2, y2, z2);

            if (id > 0) {
                if (format == POINT_CLOUD_FORMAT_POINTS) {
                    objects[id-1].push_back(x2);
                    objects[id-1].push_back(y2);
                    objects[id-1].push_back(z2);
                    objects[id-1].push_back((float)img[i*3]);
                    objects[id-1].push_back((float)img[i*3+1]);
                    objects[id-1].push_back((float)img[i*3+2]);
                }
                if (format == POINT_CLOUD_FORMAT_STL) {
                    if ((prev_x == x-1) && (prev_id == id)) {
                        if (get_object_id(
                            x,y-1,w,
                            cos_tilt, sin_tilt,
                            centre, mult,
                            relative_x_mm, relative_y_mm,
                            map_dimension, threshold, map,
                            points_image_data,
                            pose_x, pose_y, pose_z,
                            x5, y5, z5) == id) {
                            if (get_object_id(
                                x-1,y-1,w,
                                cos_tilt, sin_tilt,
                                centre, mult,
                                relative_x_mm, relative_y_mm,
                                map_dimension, threshold, map,
                                points_image_data,
                                pose_x, pose_y, pose_z,
                                x4, y4, z4) == id) {

                                objects[id-1].push_back(x2);
                                objects[id-1].push_back(y2);
                                objects[id-1].push_back(z2);

                                objects[id-1].push_back(x3);
                                objects[id-1].push_back(y3);
                                objects[id-1].push_back(z3);

                                objects[id-1].push_back(x4);
                                objects[id-1].push_back(y4);
                                objects[id-1].push_back(z4);

                                objects[id-1].push_back(x5);
                                objects[id-1].push_back(y5);
                                objects[id-1].push_back(z5);

                                if (BGR) {
                                    objects[id-1].push_back((float)rgb15(img[i*3+2],img[i*3+1],img[i*3]));
                                }
                                else {
                                    objects[id-1].push_back((float)rgb15(img[i*3],img[i*3+1],img[i*3+2]));
                                }
                            }
                        }
                    }
                }

                if (format == POINT_CLOUD_FORMAT_X3D) {
                    if ((prev_x == x-1) && (prev_id == id)) {
                        if (get_object_id(
                            x,y-1,w,
                            cos_tilt, sin_tilt,
                            centre, mult,
                            relative_x_mm, relative_y_mm,
                            map_dimension, threshold, map,
                            points_image_data,
                            pose_x, pose_y, pose_z,
                            x5, y5, z5) == id) {
                            if (get_object_id(
                                x-1,y-1,w,
                                cos_tilt, sin_tilt,
                                centre, mult,
                                relative_x_mm, relative_y_mm,
                                map_dimension, threshold, map,
                                points_image_data,
                                pose_x, pose_y, pose_z,
                                x4, y4, z4) == id) {

                                objects[id-1].push_back(x2);
                                objects[id-1].push_back(y2);
                                objects[id-1].push_back(z2);

                                objects[id-1].push_back(x3);
                                objects[id-1].push_back(y3);
                                objects[id-1].push_back(z3);

                                objects[id-1].push_back(x4);
                                objects[id-1].push_back(y4);
                                objects[id-1].push_back(z4);

                                objects[id-1].push_back(x5);
                                objects[id-1].push_back(y5);
                                objects[id-1].push_back(z5);

                                objects[id-1].push_back((float)img[i*3+2]);
                                objects[id-1].push_back((float)img[i*3+1]);
                                objects[id-1].push_back((float)img[i*3]);
                            }
                        }
                    }
                }

                prev_x = x;
                prev_id = id;
                x3 = x2;
                y3 = y2;
                z3 = z2;
                if (format != POINT_CLOUD_FORMAT_STL) {
                    img[i*3+((id-1)%3)]=255;
                }
            }
        }
    }

    //for (int i = 0; i < (int)objects.size(); i++) {
    //    printf("%d - %d\n", i, (int)objects[i].size()/6);
    //}

}

void pointcloud::export_points(
    int format,
    unsigned char * img,
    IplImage * points_image,
    CvMat * pose,
    float tilt_degrees,
    bool BGR,
    std::vector<float> &points,
    int max_range_mm)
{
    float * points_image_data = (float*)points_image->imageData;
    float pose_x = (float)cvmGet(pose,POINT_CLOUD_X_AXIS,3);
    float pose_y = (float)cvmGet(pose,POINT_CLOUD_Y_AXIS,3);
    float pose_z = (float)cvmGet(pose,POINT_CLOUD_Z_AXIS,3);
    float cos_tilt = (float)cos(-tilt_degrees/180.0*3.1415927);
    float sin_tilt = (float)sin(-tilt_degrees/180.0*3.1415927);
    int w = points_image->width;
    int prev_x=0;
    float dx,dy,dz,x2=0,y2=0,z2=0,x3=0,y3=0,z3=0,x4=0,y4=0,z4=0,x5=0,y5=0,z5=0;
    float max_range_mm2 = max_range_mm*max_range_mm;
    for (int y = 1; y < points_image->height-1; y++) {
        for (int x = 1; x < w-1; x++) {
            int i = y*w + x;

            if (points_image_data[i*3+POINT_CLOUD_X_AXIS] +
                points_image_data[i*3+POINT_CLOUD_Y_AXIS] +
                points_image_data[i*3+POINT_CLOUD_Z_AXIS] != 0) {

                dx = points_image_data[i*3+POINT_CLOUD_X_AXIS] - pose_x;
                dy = points_image_data[i*3+POINT_CLOUD_Y_AXIS] - pose_y;
                dz = points_image_data[i*3+POINT_CLOUD_Z_AXIS] - pose_z;

                float range_mm2 = dx*dx+dy*dy+dz*dz;
                if (range_mm2 < max_range_mm2) {
            
                    x2 = dx + pose_x;
                    y2 = (cos_tilt*dy - sin_tilt*dz) + pose_y;
                    z2 = (sin_tilt*dy + cos_tilt*dz) + pose_z;

                    if (format == POINT_CLOUD_FORMAT_POINTS) {
                        points.push_back(x2);
                        points.push_back(y2);
                        points.push_back(z2);
                        points.push_back((float)img[i*3]);
                        points.push_back((float)img[i*3+1]);
                        points.push_back((float)img[i*3+2]);
                    }
                    if (format == POINT_CLOUD_FORMAT_STL) {
                        if (prev_x == x-1) {

                            int i2 = (y-1)*w + x;
                            if (points_image_data[i2*3+POINT_CLOUD_X_AXIS] +
                                points_image_data[i2*3+POINT_CLOUD_Y_AXIS] +
                                points_image_data[i2*3+POINT_CLOUD_Z_AXIS] != 0) {
                                dx = points_image_data[i2*3+POINT_CLOUD_X_AXIS] - pose_x;
                                dy = points_image_data[i2*3+POINT_CLOUD_Y_AXIS] - pose_y;
                                dz = points_image_data[i2*3+POINT_CLOUD_Z_AXIS] - pose_z;
                                float range_mm2 = dx*dx+dy*dy+dz*dz;
                                if (range_mm2 < max_range_mm2) {
                                    x5 = dx + pose_x;
                                    y5 = (cos_tilt*dy - sin_tilt*dz) + pose_y;
                                    z5 = (sin_tilt*dy + cos_tilt*dz) + pose_z;

                                    int i3 = ((y-1)*w) + x - 1;
                                    if (points_image_data[i3*3+POINT_CLOUD_X_AXIS] +
                                        points_image_data[i3*3+POINT_CLOUD_Y_AXIS] +
                                        points_image_data[i3*3+POINT_CLOUD_Z_AXIS] != 0) {
                                        dx = points_image_data[i3*3+POINT_CLOUD_X_AXIS] - pose_x;
                                        dy = points_image_data[i3*3+POINT_CLOUD_Y_AXIS] - pose_y;
                                        dz = points_image_data[i3*3+POINT_CLOUD_Z_AXIS] - pose_z;
                                        float range_mm2 = dx*dx+dy*dy+dz*dz;
                                        if (range_mm2 < max_range_mm2) {
                                            x4 = dx + pose_x;
                                            y4 = (cos_tilt*dy - sin_tilt*dz) + pose_y;
                                            z4 = (sin_tilt*dy + cos_tilt*dz) + pose_z;

                                            points.push_back(x2);
                                            points.push_back(y2);
                                            points.push_back(z2);

                                            points.push_back(x3);
                                            points.push_back(y3);
                                            points.push_back(z3);

                                            points.push_back(x4);
                                            points.push_back(y4);
                                            points.push_back(z4);

                                            points.push_back(x5);
                                            points.push_back(y5);
                                            points.push_back(z5);

                                            if (BGR) {
                                                points.push_back((float)rgb15(img[i*3+2],img[i*3+1],img[i*3]));
                                            }
                                            else {
                                                points.push_back((float)rgb15(img[i*3],img[i*3+1],img[i*3+2]));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        prev_x = x;
                        x3 = x2;
                        y3 = y2;
                        z3 = z2;
                    }

                    if (format == POINT_CLOUD_FORMAT_X3D) {
                        if (prev_x == x-1) {
                            int i2 = (y-1)*w + x;
                            if (points_image_data[i2*3+POINT_CLOUD_X_AXIS] +
                                points_image_data[i2*3+POINT_CLOUD_Y_AXIS] +
                                points_image_data[i2*3+POINT_CLOUD_Z_AXIS] != 0) {
                                dx = points_image_data[i2*3+POINT_CLOUD_X_AXIS] - pose_x;
                                dy = points_image_data[i2*3+POINT_CLOUD_Y_AXIS] - pose_y;
                                dz = points_image_data[i2*3+POINT_CLOUD_Z_AXIS] - pose_z;
                                float range_mm2 = dx*dx+dy*dy+dz*dz;
                                if (range_mm2 < max_range_mm2) {
                                    x5 = dx + pose_x;
                                    y5 = (cos_tilt*dy - sin_tilt*dz) + pose_y;
                                    z5 = (sin_tilt*dy + cos_tilt*dz) + pose_z;

                                    int i3 = ((y-1)*w) + x - 1;
                                    if (points_image_data[i3*3+POINT_CLOUD_X_AXIS] +
                                        points_image_data[i3*3+POINT_CLOUD_Y_AXIS] +
                                        points_image_data[i3*3+POINT_CLOUD_Z_AXIS] != 0) {
                                        dx = points_image_data[i3*3+POINT_CLOUD_X_AXIS] - pose_x;
                                        dy = points_image_data[i3*3+POINT_CLOUD_Y_AXIS] - pose_y;
                                        dz = points_image_data[i3*3+POINT_CLOUD_Z_AXIS] - pose_z;
                                        float range_mm2 = dx*dx+dy*dy+dz*dz;
                                        if (range_mm2 < max_range_mm2) {
                                            x4 = dx + pose_x;
                                            y4 = (cos_tilt*dy - sin_tilt*dz) + pose_y;
                                            z4 = (sin_tilt*dy + cos_tilt*dz) + pose_z;

                                            points.push_back(x2);
                                            points.push_back(y2);
                                            points.push_back(z2);

                                            points.push_back(x3);
                                            points.push_back(y3);
                                            points.push_back(z3);

                                            points.push_back(x4);
                                            points.push_back(y4);
                                            points.push_back(z4);

                                            points.push_back(x5);
                                            points.push_back(y5);
                                            points.push_back(z5);

                                            points.push_back((float)img[i*3+2]);
                                            points.push_back((float)img[i*3+1]);
                                            points.push_back((float)img[i*3]);
                                        }
                                    }
                                }
                            }
                        }

                        prev_x = x;
                        x3 = x2;
                        y3 = y2;
                        z3 = z2;
                    }
                }
            }
        }
    }
}

void pointcloud::obstacle_map(
    IplImage * points_image,
    int map_dimension,
    int map_cell_size_mm,
    CvMat * pose,
    float relative_x_mm,
    float relative_y_mm,
    int threshold,
    float tilt_degrees,
    int * map)
{
    int max_range_mm = map_dimension * map_cell_size_mm;
    int centre = map_dimension/2;
    float * points_image_data = (float*)points_image->imageData;
    memset((void*)map,'\0',map_dimension*map_dimension*sizeof(int));
    float pose_x = (float)cvmGet(pose,POINT_CLOUD_X_AXIS,3);
    float pose_y = (float)cvmGet(pose,POINT_CLOUD_Y_AXIS,3);
    float pose_z = (float)cvmGet(pose,POINT_CLOUD_Z_AXIS,3);
    float mult = map_dimension / (float)max_range_mm;
    float cos_tilt = (float)cos(-tilt_degrees/180.0*3.1415927);
    float sin_tilt = (float)sin(-tilt_degrees/180.0*3.1415927);
    int w = points_image->width;
    #pragma omp parallel for
    for (int y = 1; y < points_image->height-1; y++) {
        for (int x = 1; x < w-1; x++) {            
            int i = y*w + x;

            float dx = points_image_data[i*3+POINT_CLOUD_X_AXIS] - pose_x;
            float dy = points_image_data[i*3+POINT_CLOUD_Y_AXIS] - pose_y;
            float dz = points_image_data[i*3+POINT_CLOUD_Z_AXIS] - pose_z;
            
            float x2 = dx;
            float y2 = cos_tilt*dy - sin_tilt*dz;

            int mx = centre + (int)((x2 + relative_x_mm) * mult);
            if ((mx > 0) && (mx < map_dimension-1)) {
                int my = centre + (int)((y2 + relative_y_mm) * mult);
                if ((my > 0) && (my < map_dimension-1)) {
                    for (int my2 = my-1; my2 <= my+1; my2++) {
                        for (int mx2 = mx-1; mx2 <= mx+1; mx2++) {
                            int n = my2*map_dimension + mx2;
                            if (!((mx2 == mx) && (my2 == my))) {
                                map[n] += 1;
                            }
                            else {
                                map[n] += 4;
                            }
                        }
                    }
                }
            }
        }
    }

    #pragma omp parallel for
    for (int y = 1; y < points_image->height-1; y++) {
        for (int x = 1; x < w-1; x++) {
            int i = y*w + x;

            float dx = points_image_data[i*3+POINT_CLOUD_X_AXIS] - pose_x;
            float dy = points_image_data[i*3+POINT_CLOUD_Y_AXIS] - pose_y;
            float dz = points_image_data[i*3+POINT_CLOUD_Z_AXIS] - pose_z;
            
            float x2 = dx;
            float y2 = cos_tilt*dy - sin_tilt*dz;

            bool found = false;
            int mx = centre + (int)((x2 + relative_x_mm) * mult);
            if ((mx >= 0) && (mx < map_dimension)) {
                int my = centre + (int)((y2 + relative_y_mm) * mult);
                if ((my >= 0) && (my < map_dimension)) {
                    if (map[my*map_dimension + mx] > threshold) {
                        found = true;
                    }
                }
            }
            if (!found) {
                points_image_data[i*3] = 0;
                points_image_data[i*3+1] = 0;
                points_image_data[i*3+2] = 0;
            }
        }
    }
}

void pointcloud::surface_normal(
    float x0, float y0, float z0,
    float x1, float y1, float z1,
    float x2, float y2, float z2,
    float &nx, float &ny, float &nz)
{
    /* Compute edge vectors */
    float x10 = x1 - x0;
    float y10 = y1 - y0;
    float z10 = z1 - z0;
    float x12 = x1 - x2;
    float y12 = y1 - y2;
    float z12 = z1 - z2;

    /* Compute the cross product */
    float cpx = (z10 * y12) - (y10 * z12);
    float cpy = (x10 * z12) - (z10 * x12);
    float cpz = (y10 * x12) - (x10 * y12);

    /* Normalize the result to get the unit-length facet normal */
    float r = (float)sqrt(cpx * cpx + cpy * cpy + cpz * cpz);
    if (r != 0.0f) {
        nx = cpx / r;
        ny = cpy / r;
        nz = cpz / r;
    }
}

void pointcloud::save_stl_binary(
    std::string filename,
    int image_width,
    int image_height,
    CvMat * pose,
    float baseline,
    std::vector<float> &facets)
{
    const int elements = 13;
    FILE * fp = fopen(filename.c_str(),"wb");
    if (fp==NULL) return;
    unsigned int max = (unsigned int)(facets.size()/elements);

    CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);
    CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(rotation_matrix, y, x, cvmGet(pose, y, x));
        }
    }
    cvRodrigues2(rotation_matrix, rotation_vector);

    char buffer[80];
    sprintf((char*)buffer,"%dx%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
        image_width, image_height,
        cvmGet(pose,POINT_CLOUD_X_AXIS,3), cvmGet(pose,POINT_CLOUD_Y_AXIS,3), cvmGet(pose,POINT_CLOUD_Z_AXIS,3),
        cvmGet(rotation_vector,POINT_CLOUD_X_AXIS,0), cvmGet(rotation_vector,POINT_CLOUD_Y_AXIS,0), cvmGet(rotation_vector,POINT_CLOUD_Z_AXIS,0),
        baseline);

    fwrite(buffer,1,80,fp);
    cvReleaseMat(&rotation_matrix);
    cvReleaseMat(&rotation_vector);

    fwrite((const void*)&max,sizeof(unsigned int),1,fp);

    float nx=0,ny=0,nz=0;
    float facet[12];
    for (int f = 0; f < (int)max; f++) {
        float x0 = facets[f*elements];
        float y0 = facets[f*elements+1];
        float z0 = facets[f*elements+2];

        float x1 = facets[f*elements+3];
        float y1 = facets[f*elements+4];
        float z1 = facets[f*elements+5];

        float x2 = facets[f*elements+6];
        float y2 = facets[f*elements+7];
        float z2 = facets[f*elements+8];

        float x3 = facets[f*elements+9];
        float y3 = facets[f*elements+10];
        float z3 = facets[f*elements+11];

        unsigned short attribute = (unsigned short)facets[f*elements+12];

        // first triangle
        surface_normal(
            x0, y0, z0,
            x1, y1, z1,
            x2, y2, z2,
            nx, ny, nz);

        if ((!((nx==0) && (ny==0) && (nz==0))) &&
            (!std::isnan(nx)) && (!std::isnan(ny)) && (!std::isnan(nz))) {

            facet[0] = nx;
            facet[1] = ny;
            facet[2] = nz;
        
            for (int i = 0; i < 9; i++) {
                facet[i+3] = facets[f*elements+i];
            }
            fwrite((const void*)facet,sizeof(float),12,fp);
            fwrite((const void*)&attribute,sizeof(unsigned short),1,fp);
        }

        // second triangle
        surface_normal(
            x2, y2, z2,
            x3, y3, z3,
            x0, y0, z0,
            nx, ny, nz);

        if ((!((nx==0) && (ny==0) && (nz==0))) &&
            (!std::isnan(nx)) && (!std::isnan(ny)) && (!std::isnan(nz))) {

            facet[0] = nx;
            facet[1] = ny;
            facet[2] = nz;
        
            for (int v=0;v<3;v++) {
                int i2 = (v*3) + 6;
                if (i2 >= 12) i2 -= 12;
                for (int j = 0; j < 3; j++) {
                    facet[(v*3)+3+j] = facets[f*elements+i2+j];
                }
            }

            fwrite((const void*)facet,sizeof(float),12,fp);
            fwrite((const void*)&attribute,sizeof(unsigned short),1,fp);
        }
    }
    fclose(fp);
}

void pointcloud::save_mesh_x3d(
    std::string filename,
    int image_width,
    int image_height,
    CvMat * pose,
    float baseline,
    std::vector<float> &facets)
{
    const int elements = 15;
    unsigned int max = (unsigned int)(facets.size()/elements);

    FILE * fp = fopen(filename.c_str(),"w");
    if (fp==NULL) return;

    fprintf(fp,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(fp,"<!DOCTYPE X3D PUBLIC \"ISO//Web3D//DTD X3D 3.1//EN\" \"http://www.web3d.org/specifications/x3d-3.1.dtd\">\n");
    fprintf(fp,"<X3D profile=\"Immersive\" version=\"3.1\" xsd:noNamespaceSchemaLocation=\"http://www.web3d.org/specifications/x3d-3.1.xsd\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema-instance\">\n");
    fprintf(fp, "<head>\n");
    fprintf(fp,"<meta content=\"v4l2stereo\" name=\"generator\"/>\n");    
    fprintf(fp,"<meta content=\"%f\" name=\"baseline\"/>\n", baseline);
    fprintf(fp,"<meta content=\"%dx%d\" name=\"resolution\"/>\n", image_width, image_height);
    fprintf(fp,"<meta content=\"%f %f %f\" name=\"translation\"/>\n",
        cvmGet(pose,POINT_CLOUD_X_AXIS,3), cvmGet(pose,POINT_CLOUD_Y_AXIS,3), cvmGet(pose,POINT_CLOUD_Z_AXIS,3));

    CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);
    CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(rotation_matrix, y, x, cvmGet(pose, y, x));
        }
    }
    cvRodrigues2(rotation_matrix, rotation_vector);
    fprintf(fp,"<meta content=\"%f %f %f\" name=\"rotation\"/>\n",
        cvmGet(rotation_vector,POINT_CLOUD_X_AXIS,0), cvmGet(rotation_vector,POINT_CLOUD_Y_AXIS,0), cvmGet(rotation_vector,POINT_CLOUD_Z_AXIS,0));
    cvReleaseMat(&rotation_matrix);
    cvReleaseMat(&rotation_vector);

    fprintf(fp,"</head>\n");
    fprintf(fp,"<Scene>\n");
    fprintf(fp,"<Shape>\n");
    fprintf(fp,"<IndexedFaceSet coordIndex=\"");

    for (int f = 0; f < (int)max; f++) {
        fprintf(fp,"%d %d %d -1 %d %d %d -1 ", (f*4),(f*4)+1,(f*4)+2,(f*4)+2,(f*4)+3,(f*4));
    }
    fprintf(fp,"\" solid=\"false\" normalPerVertex=\"false\">\n");
    fprintf(fp,"<Coordinate point=\"");
    for (int f = 0; f < (int)max; f++) {
        for (int i = 0; i < 12; i++) {
            fprintf(fp,"%.2f ", facets[f*elements + i]);
        }
    }
    fprintf(fp,"\"/>\n");

    fprintf(fp,"<ColorRGBA color=\"");
    for (int f = 0; f < (int)max; f++) {
        for (int j = 0; j < 4; j++) {
            fprintf(fp,"%.2f %.2f %.2f 1 ", facets[f*elements + 12]/255.0f, facets[f*elements + 13]/255.0f, facets[f*elements + 14]/255.0f);
        }
    }
    fprintf(fp,"\"/>\n");

    fprintf(fp,"<Normal vector=\"");

    float nx=0,ny=0,nz=0;
    for (int f = 0; f < (int)max; f++) {
        float x0 = facets[f*elements];
        float y0 = facets[f*elements+1];
        float z0 = facets[f*elements+2];

        float x1 = facets[f*elements+3];
        float y1 = facets[f*elements+4];
        float z1 = facets[f*elements+5];

        float x2 = facets[f*elements+6];
        float y2 = facets[f*elements+7];
        float z2 = facets[f*elements+8];

        float x3 = facets[f*elements+9];
        float y3 = facets[f*elements+10];
        float z3 = facets[f*elements+11];

        // first triangle
        surface_normal(
            x0, y0, z0,
            x1, y1, z1,
            x2, y2, z2,
            nx, ny, nz);

        fprintf(fp,"%.2f %.2f %.2f ", nx, ny, nz);

        // second triangle
        surface_normal(
            x2, y2, z2,
            x3, y3, z3,
            x0, y0, z0,
            nx, ny, nz);

        fprintf(fp,"%.2f %.2f %.2f ", nx, ny, nz);
    }
    fprintf(fp,"\"/>");
    fprintf(fp,"</IndexedFaceSet>\n");
    fprintf(fp,"</Shape>\n");
    fprintf(fp,"</Scene>\n");
    fprintf(fp,"</X3D>\n");

    fclose(fp);
}

void pointcloud::save_point_cloud_x3d(
    std::string filename,
    int image_width,
    int image_height,
    CvMat * pose,
    float baseline,
    std::vector<float> &point,
    std::vector<unsigned char> &point_colour)
{
    unsigned int max = (unsigned int)(point.size()/3);

    FILE * fp = fopen(filename.c_str(),"w");
    if (fp==NULL) return;

    fprintf(fp,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(fp,"<!DOCTYPE X3D PUBLIC \"ISO//Web3D//DTD X3D 3.1//EN\" \"http://www.web3d.org/specifications/x3d-3.1.dtd\">\n");
    fprintf(fp,"<X3D profile=\"Immersive\" version=\"3.1\" xsd:noNamespaceSchemaLocation=\"http://www.web3d.org/specifications/x3d-3.1.xsd\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema-instance\">\n");
    fprintf(fp, "<head>\n");
    fprintf(fp,"<meta content=\"v4l2stereo\" name=\"generator\"/>\n");    
    fprintf(fp,"<meta content=\"%f\" name=\"baseline\"/>\n", baseline);
    fprintf(fp,"<meta content=\"%dx%d\" name=\"resolution\"/>\n", image_width, image_height);
    fprintf(fp,"<meta content=\"%f %f %f\" name=\"translation\"/>\n",
        cvmGet(pose,POINT_CLOUD_X_AXIS,3), cvmGet(pose,POINT_CLOUD_Y_AXIS,3), cvmGet(pose,POINT_CLOUD_Z_AXIS,3));

    CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);
    CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(rotation_matrix, y, x, cvmGet(pose, y, x));
        }
    }
    cvRodrigues2(rotation_matrix, rotation_vector);
    fprintf(fp,"<meta content=\"%f %f %f\" name=\"rotation\"/>\n",
        cvmGet(rotation_vector,POINT_CLOUD_X_AXIS,0), cvmGet(rotation_vector,POINT_CLOUD_Y_AXIS,0), cvmGet(rotation_vector,POINT_CLOUD_Z_AXIS,0));
    cvReleaseMat(&rotation_matrix);
    cvReleaseMat(&rotation_vector);

    fprintf(fp,"</head>\n");
    fprintf(fp,"<Scene>\n");
    fprintf(fp,"<Shape>\n");
    fprintf(fp,"<PointSet>\n");
    fprintf(fp,"<Coordinate point=\"");

    for (int p = 0; p < (int)max; p++) {
        fprintf(fp,"%.2f %.2f %.2f ", -point[p*3], point[p*3+1], point[p*3+2]);
    }
    fprintf(fp,"\"/>\n");

    fprintf(fp,"<ColorRGBA color=\"");
    for (int p = 0; p < (int)max; p++) {
        fprintf(fp,"%.2f %.2f %.2f 1 ", point_colour[p*3+2]/255.0f, point_colour[p*3+1]/255.0f, point_colour[p*3]/255.0f);
    }
    fprintf(fp,"\"/>\n");

    fprintf(fp,"</PointSet>\n");
    fprintf(fp,"</Shape>\n");
    fprintf(fp,"</Scene>\n");
    fprintf(fp,"</X3D>\n");

    fclose(fp);
}

void pointcloud::save_stl_ascii(
    std::string filename,
    int image_width __attribute__((unused)),
    int image_height __attribute__((unused)),
    CvMat * pose __attribute__((unused)),
    float baseline __attribute__((unused)),
    std::vector<float> &facets)
{
    const int elements = 13;
    FILE * fp = fopen(filename.c_str(),"w");
    if (fp==NULL) return;
    unsigned int max = (unsigned int)(facets.size()/elements);

    std::string name = "";
    const char * name_str = filename.c_str();
    for (int i = 0; i < (int)filename.length(); i++) {
        if ((name_str[i]=='\\') || (name_str[i]=='/')) {
            name="";
        }
        else {
            if (name_str[i]=='.') break;
            name += name_str[i];
        }
    }

    fprintf(fp,"solid %s\n", name.c_str());

    float nx=0,ny=0,nz=0;
    for (int f = 0; f < (int)max; f++) {
        float x0 = facets[f*elements];
        float y0 = facets[f*elements+1];
        float z0 = facets[f*elements+2];

        float x1 = facets[f*elements+3];
        float y1 = facets[f*elements+4];
        float z1 = facets[f*elements+5];

        float x2 = facets[f*elements+6];
        float y2 = facets[f*elements+7];
        float z2 = facets[f*elements+8];

        float x3 = facets[f*elements+9];
        float y3 = facets[f*elements+10];
        float z3 = facets[f*elements+11];

        //unsigned short attribute = (unsigned short)facets[f*elements+12];

        // first triangle
        surface_normal(
            x0, y0, z0,
            x1, y1, z1,
            x2, y2, z2,
            nx, ny, nz);

        fprintf(fp,"facet normal %.3f %.3f %.3f\n", nx, ny, nz);
        fprintf(fp,"outer loop\n");

        for (int v=0;v<3;v++) {
            fprintf(fp,"vertex %.3f %.3f %.3f\n", facets[f*elements+(v*3)], facets[f*elements+(v*3)+1], facets[f*elements+(v*3)+2]);
        }

        fprintf(fp,"end loop\n");
        fprintf(fp,"end facet\n");

        // second triangle
        surface_normal(
            x2, y2, z2,
            x3, y3, z3,
            x0, y0, z0,
            nx, ny, nz);

        fprintf(fp,"facet normal %.3f %.3f %.3f\n", nx, ny, nz);
        fprintf(fp,"outer loop\n");

        for (int v=0;v<3;v++) {
            int i2 = (v*3) + 6;
            if (i2 >= 12) i2 -= 12;
            fprintf(fp,"vertex %.3f %.3f %.3f\n", facets[f*elements+i2], facets[f*elements+i2+1], facets[f*elements+i2+2]);
        }

        fprintf(fp,"end loop\n");
        fprintf(fp,"end facet\n");
    }

    fprintf(fp,"endsolid %s\n", name.c_str());
    fclose(fp);
}

void pointcloud::save_largest_object(
    std::string filename,
    int format,
    bool binary,
    int image_width,
    int image_height,
    CvMat * pose,
    float baseline,
    std::vector<std::vector<float> > &objects)
{
    int max_points = 0;
    int index = -1;
    for (int i = 0; i < (int)objects.size(); i++) {
        if ((int)objects[i].size() > max_points) {
            max_points = (int)objects[i].size();
            index = i;
        }
    }
    if (max_points==0) return;
    if (format == POINT_CLOUD_FORMAT_STL) {
        if (binary) {
            save_stl_binary(filename, image_width, image_height, pose, baseline, objects[index]);
        }
        else {
            save_stl_ascii(filename, image_width, image_height, pose, baseline, objects[index]);
        }
    }
    else {
        save_mesh_x3d(filename, image_width, image_height, pose, baseline, objects[index]);
    }
}

float pointcloud::surface_area(
    std::vector<cv::Point> &surface)
{
    float area = 0;
    int corners = (int)surface.size();
    for (int cornercount=0; cornercount < corners-1; cornercount++) {
        area += 
            -0.5f*(surface[cornercount].x*surface[cornercount+1].y -
            surface[cornercount+1].x*surface[cornercount].y);
    }
    area += -0.5f*(surface[corners-1].x*surface[0].y - surface[0].x*surface[corners-1].y);
    return area;
}

void pointcloud::fill_surface(
    int * height,
    int dimension,
    int x, int y,
    int surface_height_mm,
    int cell_size_mm,
    std::vector<cv::Point> &surface,
    int depth)
{
    if ((depth < 10000) && (x >= 0) && (y >= 0) && (x < dimension) && (y < dimension)) {
        int h = height[y*dimension + x];
        if ((h == surface_height_mm) ||
            (h == surface_height_mm + cell_size_mm) ||
            (h == surface_height_mm - cell_size_mm)) {
//            (h == surface_height_mm - (cell_size_mm*2)) ||
//            (h == surface_height_mm + (cell_size_mm*2))) {
            surface.push_back(cv::Point(x, y));
            height[y*dimension + x]++;

            fill_surface(height, dimension, x, y-1, surface_height_mm, cell_size_mm, surface, depth+1);
            fill_surface(height, dimension, x+1, y-1, surface_height_mm, cell_size_mm, surface, depth+1);
            fill_surface(height, dimension, x+1, y, surface_height_mm, cell_size_mm, surface, depth+1);
            fill_surface(height, dimension, x+1, y+1, surface_height_mm, cell_size_mm, surface, depth+1);
            fill_surface(height, dimension, x, y+1, surface_height_mm, cell_size_mm, surface, depth+1);
            fill_surface(height, dimension, x-1, y+1, surface_height_mm, cell_size_mm, surface, depth+1);
            fill_surface(height, dimension, x-1, y, surface_height_mm, cell_size_mm, surface, depth+1);
            fill_surface(height, dimension, x-1, y-1, surface_height_mm, cell_size_mm, surface, depth+1);
        }
    }
}

void pointcloud::enlarge_surface(
    std::vector<cv::Point> &surface,
    std::vector<cv::Point> &enlarged,
    int enlarge_percent)
{
    enlarged.clear();
    float av_x=0,av_y=0;
    for (int i = 0; i < (int)surface.size(); i++) {
        av_x += surface[i].x;
        av_y += surface[i].y;
    }
    av_x /= (int)surface.size();
    av_y /= (int)surface.size();
    for (int i = 0; i < (int)surface.size(); i++) {
        float dx = (surface[i].x - av_x)*(100+enlarge_percent)/100;
        float dy = (surface[i].y - av_y)*(100+enlarge_percent)/100;
        cv::Point pt;
        pt.x = av_x + dx;
        pt.y = av_y + dy;
        enlarged.push_back(pt);
    }
}

bool pointcloud::connected_surfaces(
    std::vector<cv::Point> &surface1,
    std::vector<cv::Point> &surface2)
{
    bool connected = false;
    std::vector<cv::Point> enlarged1;
    std::vector<cv::Point> enlarged2;

    enlarge_surface(surface1,enlarged1,10);
    enlarge_surface(surface2,enlarged2,10);

    for (int i = 0; i < (int)enlarged1.size(); i++) {
        if (inside_surface(enlarged2, enlarged1[i].x,enlarged1[i].y)) {
            return true;
        }
    }
    for (int i = 0; i < (int)enlarged2.size(); i++) {
        if (inside_surface(enlarged1, enlarged2[i].x,enlarged2[i].y)) {
            return true;
        }
    }
    return connected;
}

bool pointcloud::join_surfaces(
    std::vector<std::vector<cv::Point> > &surfaces,
    std::vector<int> &surface_heights,
    int max_height_difference)
{
    bool joined = false;
    std::vector<std::vector<cv::Point> > new_surfaces;
    std::vector<int> new_surface_heights;
    bool * connection_matrix = new bool[(int)surfaces.size()*(int)surfaces.size()];
    for (int i = ((int)surfaces.size()*(int)surfaces.size())-1; i >= 0; i--) {
        connection_matrix[i] = false;
    }

    for (int i = 0; i < (int)surfaces.size(); i++) {
        for (int j = i+1; j < (int)surfaces.size(); j++) {
            if (fabs(surface_heights[i] - surface_heights[j]) < max_height_difference) {
                if (connected_surfaces(surfaces[i], surfaces[j])) {
                    connection_matrix[i*(int)surfaces.size() + j] = true;
                    connection_matrix[j*(int)surfaces.size() + i] = true;
                }
            }
        }
    }

    for (int i = 0; i < (int)surfaces.size(); i++) {
        std::vector<cv::Point> pts;
        for (int j = 0; j < (int)surfaces[i].size(); j++) {
            pts.push_back(surfaces[i][j]);
        }

        int height = surface_heights[i];
        int height_hits=1;
        for (int j = 0; j < (int)surfaces.size(); j++) {
            if (connection_matrix[i*(int)surfaces.size() + j]) {
                for (int k = 0; k < (int)surfaces[j].size(); k++) {
                    pts.push_back(surfaces[j][k]);
                }
                height += surface_heights[j];
                height_hits++;
            }
        }

        if (height_hits == 1) {
            new_surfaces.push_back(pts);
            new_surface_heights.push_back(surface_heights[i]);
        }
        else {
            joined = true;
            int no_of_points = (int)pts.size();
            CvPoint * points = new CvPoint[no_of_points];
            int * hull = new int[no_of_points];
                        
            for (int k = 0; k < (int)pts.size(); k++) {
                points[k] = pts[k];
            }

            CvMat pointMat = cvMat(1, no_of_points, CV_32SC2, points);
            CvMat hullMat = cvMat(1, no_of_points, CV_32SC1, hull);
            cvConvexHull2( &pointMat, &hullMat, CV_CLOCKWISE, 0 );
            std::vector<cv::Point> hull2;
            for (int k = 0; k < (int)hullMat.cols; k++) {
                hull2.push_back(points[hull[k]]);
            }
            new_surfaces.push_back(hull2);
            new_surface_heights.push_back(height/height_hits);
            delete [] hull;
            delete [] points;
        }
    }

    delete [] connection_matrix;

    surfaces.clear();
    surface_heights.clear();
    for (int i = 0; i < (int)new_surfaces.size(); i++) {
        surfaces.push_back(new_surfaces[i]);
        surface_heights.push_back(new_surface_heights[i]);
    }
    return joined;
}

bool pointcloud::inside_surface(
    std::vector<cv::Point> &vertices,
    float x, float y)
{
    int i, j;
    bool c = false;
    for (i = 0, j = (int)vertices.size()-1; i < (int)vertices.size(); j = i++) {
        if (fabs(vertices[j].y - vertices[i].y) > 0.1f) {
            if ( ((vertices[i].y > y) != (vertices[j].y > y)) &&
	         (x < (vertices[j].x-vertices[i].x) * (y-vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x) ) {
                c = !c;
            }
        }
    }
    return c;
}

void pointcloud::find_horizontal_surfaces(
    int map_dimension_mm,
    int cell_size_mm,
    int min_height_mm,
    int patch_surface_area_mm2,
    int min_surface_area_mm2,
    int * height,
    std::vector<std::vector<cv::Point> > &surfaces,
    std::vector<int> &surface_heights)
{
    int dimension = map_dimension_mm / cell_size_mm;
    int half_dimension = dimension/2;

    for (int y = 1; y < dimension; y++) {
        for (int x = 1; x < dimension; x++) {
            int n = y*dimension + x;
            if (height[n] > min_height_mm) {
                if ((height[n] == height[n-1]) &&
                    (height[n] == height[n-dimension]) &&
                    (height[n] == height[n-dimension-1])) {
                    std::vector<cv::Point> surface_points;
                    fill_surface(height, dimension, x, y, height[n], cell_size_mm, surface_points, 0);
                    int area_mm2 = (int)surface_points.size() * cell_size_mm;
                    if (area_mm2 > patch_surface_area_mm2) {
                        // compute the convex hull
                        CvPoint * points = new CvPoint[(int)surface_points.size()];
                        int * hull = new int[(int)surface_points.size()];

                        float tx=0,ty=0,bx=0,by=0;
                        for (int i = 0; i < (int)surface_points.size(); i++) {
                            points[i] = surface_points[i];
                            if (i>0) {
                                if (points[i].x < tx) tx = points[i].x;
                                if (points[i].y < ty) ty = points[i].y;
                                if (points[i].x > bx) bx = points[i].x;
                                if (points[i].y > by) by = points[i].y;
                            }
                            else {
                                tx = points[i].x;
                                ty = points[i].y;
                                bx = points[i].x;
                                by = points[i].y;
                            }
                        }

                        CvMat pointMat = cvMat(1, (int)surface_points.size(), CV_32SC2, points);
                        CvMat hullMat = cvMat(1, (int)surface_points.size(), CV_32SC1, hull);
                        cvConvexHull2( &pointMat, &hullMat, CV_CLOCKWISE, 0 );
                        std::vector<cv::Point> hull2;
                        for (int i = 0; i < (int)hullMat.cols; i++) {
                            CvPoint pt = points[hull[i]];
                            pt.x = (pt.x - half_dimension) * cell_size_mm;
                            pt.y = (pt.y - half_dimension) * cell_size_mm;
                            hull2.push_back(pt);
                        }
                        surfaces.push_back(hull2);
                        surface_heights.push_back(height[n]);
                        delete [] hull;
                        delete [] points;
                    }
                    // reset heights
                    for (int i = (int)surface_points.size()-1; i >= 0; i--) {
                        int n2 = surface_points[i].y*dimension + surface_points[i].x;
                        height[n2]--;
                    }
                }
            }
        }
    }

    // join nearby surfaces
    int ctr = 0;
    while ((join_surfaces(surfaces, surface_heights,cell_size_mm*2)) && (ctr < 4)) {
        ctr++;
    }

    // remove small surfaces
    for (int i = (int)surfaces.size()-1; i >= 0; i--) {
        if (surface_area(surfaces[i]) < min_surface_area_mm2) {
            surfaces.erase(surfaces.begin() + i);
        }
    }
}

void pointcloud::height_field(
    std::vector<float> &point,
    int camera_height_mm,
    int map_dimension_mm,
    int cell_size_mm,
    int min_height_mm,
    int max_height_mm,
    int * height)
{
    int dimension = map_dimension_mm / cell_size_mm;
    int half_dimension = dimension/2;

    for (int i = 0; i < dimension * dimension; i++) {
        height[i] = min_height_mm;
    }

    for (int i = (int)point.size()-3; i >= 0; i-=3) {
        int z_mm = (int)(point[i+POINT_CLOUD_Z_AXIS] + camera_height_mm);
        if (z_mm >= min_height_mm) {                     
            int x = half_dimension + (int)(point[i+POINT_CLOUD_X_AXIS] / cell_size_mm);
            if ((x >= 0) && (x < dimension)) {
                int y = half_dimension + (int)(point[i+POINT_CLOUD_Y_AXIS] / cell_size_mm);
                if ((y >= 0) && (y < dimension)) {
                    int n = y*dimension + x;
                    int z = (z_mm/cell_size_mm) * cell_size_mm;
                    if (height[n] < z) {
                        height[n] = z;
                    }
                }
            }
        }
    }

    for (int i = 0; i < dimension * dimension; i++) {
        if (height[i] > max_height_mm) {
            height[i] = min_height_mm;
        }
    }
}

void pointcloud::overhead_occupancy(
    std::vector<float> &point,
    int map_dimension_mm,
    int cell_size_mm,
    int * map)
{
    int dimension = map_dimension_mm / cell_size_mm;
    int half_dimension = dimension/2;
    memset((void*)map,'\0',dimension * dimension * sizeof(int));
    for (int i = (int)point.size()-1; i >= 0; i--) {
        int x = half_dimension + (int)(point[i*3+POINT_CLOUD_X_AXIS] / cell_size_mm);
        if ((x >= 0) && (x < dimension)) {
            int y = half_dimension + (int)(point[i*3+POINT_CLOUD_Y_AXIS] / cell_size_mm);
            if ((y >= 0) && (y < dimension)) {
                int n = y*dimension + x;
                map[n]++;
            }
        }
    }
}

void pointcloud::remove_duplicate_surfaces(
    std::vector<std::vector<cv::Point> > &surfaces,
    std::vector<int> &surface_heights)
{
    std::vector<std::vector<cv::Point> > new_surfaces;
    std::vector<int> new_surface_heights;

    for (int i = 0; i < (int)surfaces.size(); i++) {
        std::vector<cv::Point> surface1 = surfaces[i];
        int j = i+1;
        for (j = i+1; j < (int)surfaces.size(); j++) {
            std::vector<cv::Point> surface2 = surfaces[j];
            if (surface1.size()==surface2.size()) {
                int k = 0;
                for (k = 0; k < (int)surface1.size(); k++) {
                    if ((surface1[k].x != surface2[k].x) ||
                        (surface1[k].y != surface2[k].y)) {
                        break;
                    }
                }
                if (k == (int)surface1.size()) {
                    break;
                }
            }
        }
        if (j == (int)surfaces.size()) {
            new_surfaces.push_back(surface1);
            new_surface_heights.push_back(surface_heights[i]);
        }
    }
    surfaces.clear();
    surface_heights.clear();
    for (int i = 0; i < (int)new_surfaces.size(); i++) {
        surfaces.push_back(new_surfaces[i]);
        surface_heights.push_back(new_surface_heights[i]);
    }
}


void pointcloud::detect_horizontal_surfaces(
    std::vector<float> &point,
    int camera_height_mm,
    int map_dimension_mm,
    int cell_size_mm,
    int min_height_mm,
    int max_height_mm,
    int patch_surface_area_mm2,
    int min_surface_area_mm2,
    std::vector<std::vector<cv::Point> > &surfaces,
    std::vector<int> &surface_heights)
{
    int dimension = map_dimension_mm / cell_size_mm;
    int * height = new int[dimension*dimension];

    height_field(
        point, camera_height_mm,
        map_dimension_mm, cell_size_mm,
        min_height_mm, max_height_mm, height);

    find_horizontal_surfaces(
        map_dimension_mm,
        cell_size_mm,
        min_height_mm,
        patch_surface_area_mm2,
        min_surface_area_mm2,
        height,
        surfaces, surface_heights);

    delete [] height;

    remove_duplicate_surfaces(surfaces,surface_heights);
}

void pointcloud::horizontal_surfaces_points(
    std::vector<float> &point,
    std::vector<unsigned char> &point_colour,
    int camera_height_mm,
    int map_dimension_mm,
    int cell_size_mm,
    int min_height_mm,
    int max_height_mm,
    int patch_surface_area_mm2,
    int min_surface_area_mm2,
    std::vector<float> &horizontal_point,
    std::vector<unsigned char> &horizontal_point_colour)
{
    std::vector<std::vector<cv::Point> > surfaces;
    std::vector<int> surface_heights;

    detect_horizontal_surfaces(
        point,
        camera_height_mm,
        map_dimension_mm,
        cell_size_mm,
        min_height_mm, max_height_mm,
        patch_surface_area_mm2,
        min_surface_area_mm2,
        surfaces, surface_heights);

    for (int i = 0; i < (int)surfaces.size(); i++) {
        std::vector<cv::Point> surface = surfaces[i];
        int height = surface_heights[i];

        float tx = 0, ty = 0, bx = 0, by = 0;
        for (int j = 0; j < (int)surface.size(); j++) {
            if (j != 0) {
                if (surface[j].x < tx) tx = surface[j].x;
                if (surface[j].y < ty) ty = surface[j].y;
                if (surface[j].x > bx) bx = surface[j].x;
                if (surface[j].y > by) by = surface[j].y;
            }
            else {
                tx = surface[j].x;
                ty = surface[j].y;
                bx = surface[j].x;
                by = surface[j].y;
            }
        }

        for (int j = 0; j < (int)point.size(); j += 3) {
            if (point[j+POINT_CLOUD_X_AXIS] >= tx) {
                if (point[j+POINT_CLOUD_Y_AXIS] >= ty) {
                    if (point[j+POINT_CLOUD_X_AXIS] <= bx) {
                        if (point[j+POINT_CLOUD_Y_AXIS] <= by) {
                            if (inside_surface(surface, point[j+POINT_CLOUD_X_AXIS], point[j+POINT_CLOUD_Y_AXIS])) {
                                int point_height = (int)point[j+POINT_CLOUD_Z_AXIS]+camera_height_mm;
                                if (fabs(point_height - height) < cell_size_mm*2) {
                                    horizontal_point.push_back(point[j]);
                                    horizontal_point.push_back(point[j+1]);
                                    horizontal_point.push_back(point[j+3]);
                                    horizontal_point_colour.push_back(point_colour[j]);
                                    horizontal_point_colour.push_back(point_colour[j+1]);
                                    horizontal_point_colour.push_back(point_colour[j+2]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void pointcloud::colour_surfaces_points(
    std::vector<float> &point,
    std::vector<unsigned char> &point_colour,
    int camera_height_mm,
    int map_dimension_mm,
    int cell_size_mm,
    int min_height_mm,
    int max_height_mm,
    int patch_surface_area_mm2,
    int min_surface_area_mm2,
    int r, int g, int b)
{
    std::vector<std::vector<cv::Point> > surfaces;
    std::vector<int> surface_heights;

    detect_horizontal_surfaces(
        point,
        camera_height_mm,
        map_dimension_mm,
        cell_size_mm,
        min_height_mm, max_height_mm,
        patch_surface_area_mm2,
        min_surface_area_mm2,
        surfaces, surface_heights);

    for (int i = 0; i < (int)surfaces.size(); i++) {
        std::vector<cv::Point> surface = surfaces[i];
        int height = surface_heights[i];

        float tx = 0, ty = 0, bx = 0, by = 0;
        for (int j = 0; j < (int)surface.size(); j++) {
            if (j != 0) {
                if (surface[j].x < tx) tx = surface[j].x;
                if (surface[j].y < ty) ty = surface[j].y;
                if (surface[j].x > bx) bx = surface[j].x;
                if (surface[j].y > by) by = surface[j].y;
            }
            else {
                tx = surface[j].x;
                ty = surface[j].y;
                bx = surface[j].x;
                by = surface[j].y;
            }
        }

        r=g=b=0;
        switch(i % 6) {
            case 0: {
                r = 255;
                break;
            }
            case 1: {
                g = 255;
                break;
            }
            case 2: {
                b = 255;
                break;
            }
            case 3: {
                r = g = 255;
                break;
            }
            case 4: {
                g = b = 255;
                break;
            }
            case 5: {
                r = b = 255;
                break;
            }
        }

        for (int j = 0; j < (int)point.size(); j += 3) {
            if (point[j+POINT_CLOUD_X_AXIS] >= tx) {
                if (point[j+POINT_CLOUD_Y_AXIS] >= ty) {
                    if (point[j+POINT_CLOUD_X_AXIS] <= bx) {
                        if (point[j+POINT_CLOUD_Y_AXIS] <= by) {
                            if (inside_surface(surface, point[j+POINT_CLOUD_X_AXIS], point[j+POINT_CLOUD_Y_AXIS])) {
                                int point_height = (int)point[j+POINT_CLOUD_Z_AXIS]+camera_height_mm;
                                if (fabs(point_height - height) < cell_size_mm*2) {
                                   point_colour[j] = (unsigned char)b;
                                   point_colour[j+1] = (unsigned char)g;
                                   point_colour[j+2] = (unsigned char)r;
                                }
                            }
                        }
                    }
                }
            }
        }

    }
}

void pointcloud::colour_surface_objects(
    std::vector<float> &point,
    std::vector<unsigned char> &point_colour,
    int camera_height_mm,
    int map_dimension_mm,
    int cell_size_mm,
    int min_height_mm,
    int max_height_mm,
    int patch_surface_area_mm2,
    int min_surface_area_mm2,
    int r, int g, int b)
{
    std::vector<std::vector<cv::Point> > surfaces;
    std::vector<int> surface_heights;

    detect_horizontal_surfaces(
        point,
        camera_height_mm,
        map_dimension_mm,
        cell_size_mm,
        min_height_mm, max_height_mm,
        patch_surface_area_mm2,
        min_surface_area_mm2,
        surfaces, surface_heights);

    for (int i = 0; i < (int)surfaces.size(); i++) {
        std::vector<cv::Point> surface = surfaces[i];
        int height = surface_heights[i];

        float tx = 0, ty = 0, bx = 0, by = 0;
        for (int j = 0; j < (int)surface.size(); j++) {
            if (j != 0) {
                if (surface[j].x < tx) tx = surface[j].x;
                if (surface[j].y < ty) ty = surface[j].y;
                if (surface[j].x > bx) bx = surface[j].x;
                if (surface[j].y > by) by = surface[j].y;
            }
            else {
                tx = surface[j].x;
                ty = surface[j].y;
                bx = surface[j].x;
                by = surface[j].y;
            }
        }

        int min_height = height + (cell_size_mm*3/2);
        for (int j = (int)point.size()-3; j >= 0; j -= 3) {
            if (point[j+POINT_CLOUD_X_AXIS] >= tx) {
                if (point[j+POINT_CLOUD_Y_AXIS] >= ty) {
                    if (point[j+POINT_CLOUD_X_AXIS] <= bx) {
                        if (point[j+POINT_CLOUD_Y_AXIS] <= by) {
                            if (inside_surface(surface, point[j+POINT_CLOUD_X_AXIS], point[j+POINT_CLOUD_Y_AXIS])) {
                                int point_height = (int)point[j+POINT_CLOUD_Z_AXIS]+camera_height_mm;
                                if (point_height > min_height) {
                                   point_colour[j] = (unsigned char)b;
                                   point_colour[j+1] = (unsigned char)g;
                                   point_colour[j+2] = (unsigned char)r;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}



