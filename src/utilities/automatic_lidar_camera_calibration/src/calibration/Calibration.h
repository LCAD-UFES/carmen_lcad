#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_
/**
 * The base class for scan representation.  
 */

//Standard C/C++ Library
#include <iostream>
#include <fstream>
#include <string.h>

//Opencv
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

//GSL (GNU Scientific Library)
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_multifit_nlin.h>

//Glib
#include <glib.h>

//common
#include "config.h"
#include "ssc.h"
#include "so3.h"
#include "small_linalg.h"

#define MAX_BINS 256

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define to_radians(x) ( (x) * (M_PI / 180.0 ))
#define to_degrees(x) ( (x) * (180.0 / M_PI ))

#ifndef RTOD
#define RTOD   (180.0 / M_PI)
#endif

#ifndef DTOR
#define DTOR   (M_PI / 180.0)
#endif

#define CAMERA_Z_THRESH 0
#define CAM_PARAM_CONFIG_PATH "../config/master.cfg"
#define RANGE_THRESH 5.0 //in m

//Structure to hold calibration parameters
typedef struct _CalibParam CalibParam_t;
struct _CalibParam
{
    //Pose of Camera i wrt CH
    ssc_pose_t *X_hi;
    double **K;
    int width, height;
    int numCams;
};

//Structure for each 3D point
typedef struct _Point3d Point3d_t;
struct _Point3d
{
    float x;
    float y;
    float z;
    int refc;
    //ratio of distance of the point from 
    //laser center to max distance
    float range; //[0,1]
};

//Scan data structure. Points in laser reference system
typedef struct _PointCloud PointCloud_t; 
struct _PointCloud
{
    std::vector<Point3d_t> points;
};

//Structure to hold grid parameters. This is used for grid-based search
typedef struct _GridParam GridParam_t;
struct _GridParam
{
    //Level 1 grid parameters
    ssc_pose_t gridCenter;
    //grid size
    double gszTrans;
    double gszRot;
    //grid step
    double gstTrans;
    double gstRot;
};  

//Image data structure
typedef struct _Image Image_t;
struct _Image
{
    std::vector<IplImage*> image;
};

namespace perls
{
    class Probability 
    {
        public:
          Probability (){};
          Probability (int n)
          {
              jointProb = cv::Mat::zeros (n, n, CV_32FC1);
              refcProb = cv::Mat::zeros (1, n, CV_32FC1);
              grayProb = cv::Mat::zeros (1, n, CV_32FC1);
              count = 0;
          };
          ~Probability () {};
          //joint Probability
          cv::Mat jointProb;
          //marginal probability reflectivity
          cv::Mat refcProb;
          //marginal probability grayscale
          cv::Mat grayProb;
          int count;
    };

    class Histogram 
    {
        public:
          Histogram (){};
          Histogram (int n)
          {
              jointHist = cv::Mat::zeros (n, n, CV_32FC1);
              refcHist = cv::Mat::zeros (1, n, CV_32FC1);
              grayHist = cv::Mat::zeros (1, n, CV_32FC1);
              count = 0;
          };
          ~Histogram () {};
          //joint Histogram
          cv::Mat jointHist;
          cv::Mat refcHist;
          cv::Mat grayHist; 
          int count;
          int gray_sum;
          int refc_sum;
    };

    class Calibration
    {
        public:
          Calibration (char *config_file);
          ~Calibration ();

          /**Functions to load the data**/
          ssc_pose_t m_X0; //initial guess
          int m_scanIndex;
          int m_estimatorType;
          double m_corrCoeff;
          int    load_camera_parameters ();
          int    load_point_cloud (char* scanFile);
          int    load_image (int imageIndex);
          int    release_image (Image_t *image);
          void   load_mask ();
          void   release_mask ();
          /*****************************/

          /**Helper functions**/
          void   get_random_numbers (int min, int max, int* index, int num);
          Histogram get_histogram (ssc_pose_t x0);
          Probability get_probability_MLE (Histogram hist);
          Probability get_probability_Bayes (Histogram hist);
          Probability get_probability_JS (Probability probMLE);
          /*****************************/

          /**Cost Functions**/
          float mi_cost (ssc_pose_t x0); 
          float chi_square_cost (ssc_pose_t x0);
          /*****************************/

          /** Covariance Matrix**/
          gsl_matrix* calculate_covariance_matrix (ssc_pose_t x);
          /*****************************/
          
          /**Optimization Functions**/ 
          float gradient_descent_search (ssc_pose_t x0);
          float exhaustive_grid_search (ssc_pose_t x0);
          /****************************/
          
          /**Other optimization functions**/
          /**The definition of these functions can be found in Calibration_deprecated.cpp**/
          //GSL minimization
          static double gsl_f (const gsl_vector *x, void *params);
          static void  gsl_df (const gsl_vector *x0_hl, void *params, gsl_vector *g); 
          static void  gsl_fdf (const gsl_vector *x0_hl, void *params, double *f, gsl_vector *g); 
          float gsl_minimizer (ssc_pose_t x0);
          float gsl_minimizer_nmsimplex (ssc_pose_t x0, double step_trans, double step_rot);
          //Grid search rotation/translation alternatively
          float grid_search_rot_trans (ssc_pose_t x0); 
          float grid_search_rotation (GridParam_t* gridParam); 
          float grid_search_translation (GridParam_t *gridParam); 
          /*****************************/
       private:
          std::vector<PointCloud_t> m_vecPointClouds;
          std::vector<Image_t> m_vecImage;
          Image_t m_Mask;
          CalibParam_t m_Calib;
          Config *m_ConfigHandle;
          char *m_ConfigFile;
          int m_NumScans;
          int m_NumCams;
          cv::Mat m_jointTarget;
          cv::Mat m_grayTarget;
          cv::Mat m_refcTarget;
          int m_numBins;
          int m_binFraction; 
    };
}     
#endif //_CALIBRATION_H_
