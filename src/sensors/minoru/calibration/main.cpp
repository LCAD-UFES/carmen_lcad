/*
    v4l2stereo
    A command line utility for stereoscopic vision
    Copyright (C) 2010 Bob Mottram and Giacomo Spigler
    fuzzgun@gmail.com

    Requires packages:
		libgstreamer-plugins-base0.10-dev
		libgst-dev

    sudo apt-get install libcv2.1 libhighgui2.1 libcvaux2.1 libcv-dev libcvaux-dev libhighgui-dev libgstreamer-plugins-base0.10-dev libgst-dev

    For details of use see:

        http://sluggish.dyndns.org/wiki/Libv4l2cam

    For details of the ELAS dense stereo algorithm see:

        http://rainsoft.de/software/libelas.html

        @INPROCEEDINGS{Geiger10,
        author = {Andreas Geiger and Martin Roser and Raquel Urtasun},
        title = {Efficient Large-Scale Stereo Matching},
        booktitle = {Asian Conference on Computer Vision},
        year = {2010},
        month = {November},
        address = {Queenstown, New Zealand}
        }

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

/* enable or disable gstreamer functionality */
//#define GSTREAMER

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <sstream>
#include <omp.h>

#ifdef GSTREAMER
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappbuffer.h>
#endif

#include "pointcloud.h"
#include "anyoption.h"
#include "drawing.h"
#include "stereo.h"
#include "fast.h"

#include <elas.h>
#include <libcam.h>
#include <camcalib.h>

#define VERSION			1.1

using namespace std;

/*!
 * \brief expands a subregion of the given image with pixel interpolation.  It is assumed that the subregion has the same aspect as the original.
 * \param img colour image data
 * \param img_width width of the image
 * \param img_height height of the image
 * \param tx subregion top left x coordinate
 * \param ty subregion top left y coordinate
 * \param bx subregion bottom right x coordinate
 * \param by subregion bottom right y coordinate
 * \param expanded returned expanded image
 */
void expand(
    unsigned char *img,
    int img_width,
    int img_height,
    int tx,
    int ty,
    int bx,
    int by,
    unsigned char *expanded)
{
    const int mult = 128;
    const int mult2 = mult*2;
    int w = (bx - tx) * mult;
    int h = (by - ty) * mult;
    int stride = img_width*3;
#pragma omp parallel for
    for (int y = 0; y < img_height; y++) {
        int n_expanded = y*img_width*3;
        int yy = h*y/img_height;
        int fraction_y = yy % mult;
        yy = ty + (yy/mult);
        for (int x = 0; x < img_width; x++, n_expanded += 3) {
            int xx = w*x/img_width;
            int fraction_x = xx % mult;
            xx = tx + (xx/mult);
            int n = (yy*img_width + xx)*3;

            expanded[n_expanded] =
                ((img[n] * fraction_x) + (img[n+3] * (mult-fraction_x)) +
                 (img[n] * fraction_y) + (img[n+stride] * (mult-fraction_y))) /
                mult2;
            expanded[n_expanded+1] =
                ((img[n+1] * fraction_x) + (img[n+1+3] * (mult-fraction_x)) +
                 (img[n+1] * fraction_y) + (img[n+1+stride] * (mult-fraction_y))) /
                mult2;
            expanded[n_expanded+2] =
                ((img[n+2] * fraction_x) + (img[n+2+3] * (mult-fraction_x)) +
                 (img[n+2] * fraction_y) + (img[n+2+stride] * (mult-fraction_y))) /
                mult2;
        }
    }
}

void elas_disparity_map(
    unsigned char * left_image,
    unsigned char * right_image,
    int image_width,
    int image_height,
    uint8_t * &I1,
    uint8_t * &I2,
    float * &left_disparities,
    float * &right_disparities,
    Elas * &elas)
{
    if (elas==NULL) {
        Elas::parameters param;
        elas = new Elas(param);
        I1 = new uint8_t[image_width*image_height];
        I2 = new uint8_t[image_width*image_height];
        left_disparities = new float[image_width*image_height];
        right_disparities = new float[image_width*image_height];
    }

    // convert to single byte format
    for (int i = 0; i < image_width*image_height; i++) {
        I1[i] = (uint8_t)left_image[i*3+2];
        I2[i] = (uint8_t)right_image[i*3+2];
    }

    const int32_t dims[3] = {image_width, image_height, image_width};
    elas->process(I1,I2,left_disparities,right_disparities,dims);
}

int main(int argc, char* argv[]) {

    int ww = 320;
    int hh = 240;
    int fps = 30;
    int skip_frames = 1;
    int prev_matches = 0;
    int image_index = 0;
    bool show_features = false;
    bool show_matches = false;
    bool show_regions = false;
    bool show_depthmap = false;
    bool show_anaglyph = false;
    bool show_histogram = false;
    bool show_lines = false;
    bool show_disparity_map = false;
    bool rectify_images = false;
    bool show_FAST = false;
    bool colour_disparity_map = true;
    bool overhead_view = false;
    bool virtual_camera_view = false;
    std::string learn_background_filename = "";
    int use_priors = 1;
    int matches;
    bool detect_obstacles = false;
    bool detect_objects = false;
    bool view_point_cloud = false;
    bool BGR = true;
    int object_format = POINT_CLOUD_FORMAT_POINTS;
    int max_range_mm = 3000;
    int no_of_calibration_images = 20;

    IplImage * background_image = NULL;
    IplImage * original_left_image = NULL;
    float * background_disparity_map = NULL;
    int * background_disparity_map_hits = NULL;

    uint8_t * I1 = NULL;
    uint8_t * I2 = NULL;
    float * left_disparities = NULL;
    float * right_disparities = NULL;
    Elas * elas = NULL;

    camcalib * camera_calibration = new camcalib();
    camera_calibration->ParseCalibrationFile("calibration.txt");
    rectify_images = camera_calibration->rectification_loaded;

    int disparity_histogram[3][SVS_MAX_IMAGE_WIDTH];

    int obstacle_map_dimension = 128;
    int obstacle_map_cell_size_mm = 20;
    float obstacle_map_relative_x_mm = 0;
    float obstacle_map_relative_y_mm = 0;
    int obstacle_map_threshold = 3000;
    int * obstacle_map = NULL;
    float tilt_angle_degrees = 40;

    int object_min_area_mm2 = 20;
    int object_max_area_mm2 = 500;

#ifdef GSTREAMER
    // Port to start streaming from - second video will be on this + 1
    int start_port = 5000;
#endif

    AnyOption *opt = new AnyOption();
    assert(opt != NULL);

    // help
    opt->addUsage( "Example: " );
    opt->addUsage( "  v4l2stereo -0 /dev/video1 -1 /dev/video0 -w 320 -h 240 --features" );
    opt->addUsage( " " );
    opt->addUsage( "Usage: " );
    opt->addUsage( "" );
    opt->addUsage( " -0  --dev0                Video device number of the left camera");
    opt->addUsage( " -1  --dev1                Video device number of the right camera");
    opt->addUsage( "     --camera              Sets a stereo camera type, eg. \"Minoru\"");
    opt->addUsage( " -w  --width               Image width in pixels");
    opt->addUsage( " -h  --height              Image height in pixels");
    opt->addUsage( " -x  --offsetx             Calibration x offset in pixels");
    opt->addUsage( " -y  --offsety             Calibration y offset in pixels");
    opt->addUsage( " -d  --disparity           Max disparity as a percent of image width");
    opt->addUsage( "     --calibrate           Calibrate a stereo camera (squares across, squares down, square size in mm)");
    opt->addUsage( "     --calibrationimages   Set the number of images gathered during camera calibration");
    opt->addUsage( "     --calibrationfile     Load a given calibration file");
    opt->addUsage( "     --intleft             Intrinsic calibration parameters for the left camera");
    opt->addUsage( "     --intright            Intrinsic calibration parameters for the left camera");
    opt->addUsage( "     --rectleft            Rectification matrix parameters for the left camera");
    opt->addUsage( "     --rectright           Rectification matrix parameters for the right camera");
    opt->addUsage( "     --translation         Extrinsic translation calibration parameters");
    opt->addUsage( "     --rotation            Extrinsic rotation calibration parameters");
    opt->addUsage( "     --pose                Camera pose 4x4 matrix");
    opt->addUsage( "     --poserotation        Three values specifying camera rotation in degrees");
    opt->addUsage( "     --posetranslation     Three values specifying camera translation in mm");
    opt->addUsage( "     --obstaclethreshold   Obstacle threshold value");
    opt->addUsage( "     --obstaclecellsize    Obstacle map cell size in millimetres");
    opt->addUsage( "     --obstacles           Detect obstacles");
    opt->addUsage( "     --objects             Detect objects");
    opt->addUsage( "     --points              Show point cloud");
    opt->addUsage( "     --minarea             Minimum area for object detection in mm2");
    opt->addUsage( "     --maxarea             Maximum area for object detection in mm2");
    opt->addUsage( "     --baseline            Baseline distance in millimetres");
    opt->addUsage( "     --equal               Perform histogram equalisation");
    opt->addUsage( "     --maxrange            Sets teh maximum range in millimetres");
    opt->addUsage( "     --ground              y coordinate of the ground plane as percent of image height");
    opt->addUsage( "     --features            Show stereo features");
    opt->addUsage( "     --disparitymap        Show dense disparity map (colour)");
    opt->addUsage( "     --disparitymapmono    Show dense disparity map (monochrome)");
    opt->addUsage( "     --background          Background image filename");
    opt->addUsage( "     --learnbackground     Filename to save background disparity map");
    opt->addUsage( "     --backgroundmodel     Loads a background disparity map");
    opt->addUsage( "     --overhead            Overhead view");
    opt->addUsage( "     --vcamera             Virtual camera view");
    opt->addUsage( "     --pointcloud          Filename in which to save point cloud data");
    opt->addUsage( "     --disparitythreshold  Threshold applied to the disparity map as a percentage of max disparity");
    opt->addUsage( "     --zoom                Zoom level given as a percentage");
    opt->addUsage( "     --matches             Show stereo matches");
    opt->addUsage( "     --regions             Show regions");
    opt->addUsage( "     --depth               Show depth map");
    opt->addUsage( "     --lines               Show lines");
    opt->addUsage( "     --anaglyph            Show anaglyph");
    opt->addUsage( "     --histogram           Show disparity histogram");
    opt->addUsage( "     --fast                Show FAST corners");
    opt->addUsage( "     --descriptors         Saves feature descriptor for each FAST corner");
    opt->addUsage( "     --fov                 Field of view in degrees");
    opt->addUsage( " -f  --fps                 Frames per second");
    opt->addUsage( " -s  --skip                Skip this number of frames");
    opt->addUsage( " -i  --input               Loads stereo matches from the given output file");
    opt->addUsage( " -o  --output              Saves stereo matches to the given output file");
    opt->addUsage( "     --log                 Logs stereo matches to the given output file (only when no file exists)");
    opt->addUsage( " -V  --version             Show version number");
    opt->addUsage( "     --save                Save raw images");
    opt->addUsage( "     --savex3d             Save mesh model in X3D format");
    opt->addUsage( "     --savestl             Save mesh model in STL format");
    opt->addUsage( "     --saveperiod          Save images repeatedly every x seconds");
    opt->addUsage( "     --flipright           Flip the right image");
    opt->addUsage( "     --flipleft            Flip the left image");
#ifdef GSTREAMER
    opt->addUsage( "     --stream              Stream output using gstreamer");
#endif
    opt->addUsage( "     --headless            Disable video output (for use with --stream)");
    opt->addUsage( "     --help                Show help");
    opt->addUsage( "" );

    opt->setOption( "maxrange" );
    opt->setOption( "background" );
    opt->setOption( "learnbackground" );
    opt->setOption( "backgroundmodel" );
    opt->setOption( "pose" );
    opt->setOption( "camera" );
    opt->setOption( "calibrate" );
    opt->setOption( "calibrationimages" );
    opt->setOption( "calibrationfile" );
    opt->setOption( "intleft" );
    opt->setOption( "intright" );
    opt->setOption( "rectleft" );
    opt->setOption( "rectright" );
    opt->setOption( "translation" );
    opt->setOption( "rotation" );
    opt->setOption( "saveperiod" );
    opt->setOption( "ground" );
    opt->setOption( "fast" );
    opt->setOption( "descriptors" );
    opt->setOption( "save" );
    opt->setOption( "fps", 'f' );
    opt->setOption( "dev0", '0' );
    opt->setOption( "dev1", '1' );
    opt->setOption( "width", 'w' );
    opt->setOption( "height", 'h' );
    opt->setOption( "offsetx", 'x' );
    opt->setOption( "offsety", 'y' );
    opt->setOption( "disparity", 'd' );
    opt->setOption( "input", 'i' );
    opt->setOption( "output", 'o' );
    opt->setOption( "log" );
    opt->setOption( "skip", 's' );
    opt->setOption( "fov" );
    opt->setOption( "disparitythreshold" );
    opt->setOption( "zoom" );
    opt->setOption( "baseline" );
    opt->setOption( "poserotation" );
    opt->setOption( "posetranslation" );
    opt->setOption( "pointcloud" );
    opt->setOption( "obstaclethreshold" );
    opt->setOption( "obstaclecellsize" );
    opt->setOption( "minarea" );
    opt->setOption( "maxarea" );
    opt->setOption( "savestl" );
    opt->setOption( "savex3d" );
    opt->setFlag( "help" );
    opt->setFlag( "flipleft" );
    opt->setFlag( "flipright" );
    opt->setFlag( "features" );
    opt->setFlag( "regions" );
    opt->setFlag( "matches" );
    opt->setFlag( "depth" );
    opt->setFlag( "lines" );
    opt->setFlag( "anaglyph" );
    opt->setFlag( "histogram" );
    opt->setFlag( "version", 'V' );
    opt->setFlag( "headless" );
    opt->setFlag( "disparitymap" );
    opt->setFlag( "disparitymapmono" );
    opt->setFlag( "equal" );
    opt->setFlag( "overhead" );
    opt->setFlag( "vcamera" );
    opt->setFlag( "obstacles" );
    opt->setFlag( "objects" );
    opt->setFlag( "points" );
#ifdef GSTREAMER
    opt->setFlag( "stream"  );
#endif

    opt->processCommandArgs(argc, argv);

    if(!opt->hasOptions())
    {
        // print usage if no options
        opt->printUsage();
        delete opt;
        return(0);
    }

    IplImage * disparity_image = NULL;
    IplImage * points_image = NULL;
    std::string point_cloud_filename = "";

    if( opt->getFlag( "version" ) || opt->getFlag( 'V' ) )
    {
        printf("Version %f\n", VERSION);
        delete opt;
        return(0);
    }

#ifdef GSTREAMER
    bool stream = false;
    if( opt->getFlag( "stream" ) ) {
        stream = true;
    }
#endif

    if( opt->getValue( "camera" ) != NULL ) {
        camera_calibration->SetStereoCamera(opt->getValue("camera"));
        rectify_images = true;
    }

    bool headless = false;
    if( opt->getFlag( "headless" ) ) {
        headless = true;
    }

    bool flip_left_image = false;
    if( opt->getFlag( "flipleft" ) )
    {
        flip_left_image = true;
    }

    bool flip_right_image = false;
    if( opt->getFlag( "flipright" ) ) {
        flip_right_image = true;
    }

    bool histogram_equalisation = false;
    if( opt->getFlag( "equal" ) ) {
        histogram_equalisation = true;
    }

    if( opt->getFlag( "obstacles" ) ) {
        detect_obstacles = true;
        show_disparity_map = true;
        if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
    }

    if( opt->getFlag( "points" ) ) {
        view_point_cloud = true;
    }

    if( opt->getFlag( "objects" ) ) {
        detect_objects = true;
        show_disparity_map = true;
        if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
    }

    bool save_images = false;
    std::string save_filename = "";
    if( opt->getValue( "save" ) != NULL  ) {
        save_filename = opt->getValue("save");
        if (save_filename == "") save_filename = "image_";
        save_images = true;
    }

    if( opt->getFlag( "help" ) ) {
        opt->printUsage();
        delete opt;
        return(0);
    }

    if( opt->getValue("maxrange") != NULL ) {
        max_range_mm = atoi(opt->getValue("maxrange"));
    }

    if( opt->getValue("calibrationimages") != NULL ) {
        no_of_calibration_images = atoi(opt->getValue("calibrationimages"));
        if (no_of_calibration_images<10) no_of_calibration_images=10;
    }

    if( opt->getValue("obstaclethreshold") != NULL ) {
        obstacle_map_threshold = atoi(opt->getValue("obstaclethreshold"));
    }

    if( opt->getValue("obstaclecellsize") != NULL ) {
        obstacle_map_cell_size_mm = atoi(opt->getValue("obstaclecellsize"));
    }

    if( opt->getValue("minarea") != NULL ) {
        object_min_area_mm2 = atoi(opt->getValue("minarea"));
    }

    if( opt->getValue("maxarea") != NULL ) {
        object_max_area_mm2 = atoi(opt->getValue("maxarea"));
    }

    std::string save_mesh_filename = "";
    if( opt->getValue("savestl") != NULL ) {
        save_mesh_filename = opt->getValue("savestl");
        object_format = POINT_CLOUD_FORMAT_STL;
        if (skip_frames < 2) skip_frames = 2;
    }

    if( opt->getValue("savex3d") != NULL ) {
        save_mesh_filename = opt->getValue("savex3d");
        object_format = POINT_CLOUD_FORMAT_X3D;
        if (skip_frames < 2) skip_frames = 2;
    }

    if( opt->getValue("learnbackground") != NULL ) {
        learn_background_filename = opt->getValue("learnbackground");
        if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
        if (background_disparity_map==NULL) background_disparity_map = new float[ww*hh];
        if (background_disparity_map_hits==NULL) background_disparity_map_hits = new int[ww*hh];
        memset((void*)background_disparity_map,'\0',ww*hh*sizeof(float));
        memset((void*)background_disparity_map_hits,'\0',ww*hh*sizeof(int));
        if (skip_frames < 2) skip_frames = 2;
    }

    if( (opt->getFlag( "disparitymap" )) ||
            (opt->getFlag( "overhead" )) ||
            (opt->getFlag( "vcamera" )) ||
            (opt->getValue("background")) ||
            (opt->getValue("learnbackground")) ) {
        show_regions = false;
        show_features = false;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = true;
        colour_disparity_map = true;
    }

    if( opt->getFlag( "disparitymapmono" ) ) {
        show_regions = false;
        show_features = false;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = true;
        colour_disparity_map = false;
    }

    if (opt->getFlag("features")) {
        show_regions = false;
        show_features = true;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = false;
    }

    if( opt->getFlag( "histogram" ) ) {
        show_regions = false;
        show_features = false;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = true;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = false;
    }

    if( opt->getFlag( "matches" ) ) {
        show_regions = false;
        show_features = false;
        show_matches = true;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = false;
    }

    if(opt->getFlag("regions")) {
        show_regions = true;
        show_features = false;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = false;
    }

    if( opt->getFlag( "depth" ) ) {
        show_regions = false;
        show_features = false;
        show_matches = false;
        show_depthmap = true;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = false;
    }

    if(opt->getFlag("lines")) {
        show_regions = false;
        show_features = false;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = true;
        show_FAST = false;
        show_disparity_map = false;
    }

    if (opt->getFlag("anaglyph")) {
        show_regions = false;
        show_features = false;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = true;
        show_histogram = false;
        show_lines = false;
        show_FAST = false;
        show_disparity_map = false;
    }

    int save_period_sec = 0;
    if(opt->getValue("saveperiod") != NULL) {
        save_period_sec = atoi(opt->getValue("saveperiod"));
        if (save_period_sec < 1) save_period_sec=1;
    }

    int desired_corner_features = 70;
    if( opt->getValue( "fast" ) != NULL  ) {
        show_regions = false;
        show_features = false;
        show_matches = false;
        show_depthmap = false;
        show_anaglyph = false;
        show_histogram = false;
        show_lines = false;
        show_FAST = true;
        show_disparity_map = false;
        desired_corner_features = atoi(opt->getValue("fast"));
        if (desired_corner_features > 150) desired_corner_features=150;
        if (desired_corner_features < 50) desired_corner_features=50;
    }

    int enable_ground_priors = 0;
    int ground_y_percent = 50;
    if( opt->getValue( "ground" ) != NULL  ) {
        enable_ground_priors = 1;
        ground_y_percent = atoi(opt->getValue("ground"));
    }

    /*
    if( opt->getValue( "fov" ) != NULL  ) {
        FOV_degrees = atoi(opt->getValue("fov"));
    }
    */

    if( opt->getValue( "calibrationfile" ) != NULL ) {
        std::string calibration_file = opt->getValue("calibrationfile");
        camera_calibration->ParseCalibrationFile(calibration_file.c_str());
        rectify_images = camera_calibration->rectification_loaded;
    }

    std::string dev0 = "/dev/video1";
    if( opt->getValue( '0' ) != NULL  || opt->getValue( "dev0" ) != NULL  ) {
        dev0 = opt->getValue("dev0");
    }

    std::string dev1 = "/dev/video2";
    if( opt->getValue( '1' ) != NULL  || opt->getValue( "dev1" ) != NULL  ) {
        dev1 = opt->getValue("dev1");
    }

    if( opt->getValue( 'w' ) != NULL  || opt->getValue( "width" ) != NULL  ) {
        ww = atoi(opt->getValue("width"));
    }

    if( opt->getValue( 'h' ) != NULL  || opt->getValue( "height" ) != NULL  ) {
        hh = atoi(opt->getValue("height"));
    }

    int calibration_offset_x = 0;
    if( opt->getValue( 'x' ) != NULL  || opt->getValue( "offsetx" ) != NULL  ) {
        calibration_offset_x = atoi(opt->getValue("offsetx"));
    }

    int calibration_offset_y = camera_calibration->v_shift;
    if( opt->getValue( 'y' ) != NULL  || opt->getValue( "offsety" ) != NULL  ) {
        calibration_offset_y = atoi(opt->getValue("offsety"));
    }

    int max_disparity_percent = 40;
    if( opt->getValue( 'd' ) != NULL  || opt->getValue( "disparity" ) != NULL  ) {
        max_disparity_percent = atoi(opt->getValue("disparity"));
        if (max_disparity_percent < 2) max_disparity_percent = 2;
        if (max_disparity_percent > 90) max_disparity_percent = 90;
    }

    if( opt->getValue( 'f' ) != NULL  || opt->getValue( "fps" ) != NULL  ) {
        fps = atoi(opt->getValue("fps"));
    }

    std::string descriptors_filename = "";
    if( opt->getValue( "descriptors" ) != NULL  ) {
        descriptors_filename = opt->getValue("descriptors");
    }

    std::string stereo_matches_filename = "";
    if( opt->getValue( 'o' ) != NULL  || opt->getValue( "output" ) != NULL  ) {
        stereo_matches_filename = opt->getValue("output");
        skip_frames = 6;
    }

    std::string stereo_matches_input_filename = "";
    if( opt->getValue( 'i' ) != NULL  || opt->getValue( "input" ) != NULL  ) {
        stereo_matches_input_filename = opt->getValue("input");
    }

    std::string log_stereo_matches_filename = "";
    if( opt->getValue( "log" ) != NULL  ) {
        log_stereo_matches_filename = opt->getValue("log");
    }

    if( opt->getValue( 's' ) != NULL  || opt->getValue( "skip" ) != NULL  ) {
        skip_frames = atoi(opt->getValue("skip"));
    }

    // disparity map threshold as a percentage
    int disparity_threshold_percent = 0;
    if( opt->getValue( "disparitythreshold" ) != NULL  ) {
        disparity_threshold_percent = atoi(opt->getValue("disparitythreshold"));
        if (disparity_threshold_percent < 0) disparity_threshold_percent = 0;
        if (disparity_threshold_percent > 100) disparity_threshold_percent = 100;
    }

    // baseline distance
    int baseline_mm = 60;
    if( opt->getValue( "baseline" ) != NULL  ) {
        baseline_mm = atoi(opt->getValue("baseline"));
        if (baseline_mm < 10) baseline_mm = 10;
    }

    // zoom percentage
    int zoom = 0;
    if( opt->getValue( "zoom" ) != NULL  ) {
        zoom = atoi(opt->getValue("zoom"));
        if (zoom < 0) zoom = 0;
        if (zoom > 100) zoom = 100;
    }
    int zoom_tx = zoom * ((ww/2)*80/100) / 100;
    int zoom_ty = zoom * ((hh/2)*80/100) / 100;
    int zoom_bx = ww - zoom_tx;
    int zoom_by = hh - zoom_ty;

    // adjust offsets to compensate for the zoom
    /*
        calibration_offset_x = calibration_offset_x * ww / (zoom_bx - zoom_tx);
        calibration_offset_y = calibration_offset_y * hh / (zoom_by - zoom_ty);
    */

    if( opt->getValue( "intleft" ) != NULL ) {
        if (camera_calibration->ParseIntrinsic(opt->getValue("intleft"),0)==0) {
            std::cout << "9 intrinsic calibration values are ";
            std::cout << "needed for the left camera\n";
            delete opt;
            return 0;
        }
    }

    if( opt->getValue( "intright" ) != NULL ) {
        if (camera_calibration->ParseIntrinsic(opt->getValue("intright"),1) == 0) {
            std::cout << "9 intrinsic calibration values are ";
            std::cout << "needed for the right camera\n";
            delete opt;
            return 0;
        }
    }

    if( opt->getValue( "rectleft" ) != NULL ) {
        if (camera_calibration->ParseRectification(opt->getValue("rectleft"),0)==0) {
            std::cout << "9 rectification matrix values are ";
            std::cout << "needed for the left camera\n";
            delete opt;
            return 0;
        }
        rectify_images = true;
    }

    if( opt->getValue( "rectright" ) != NULL ) {
        if (camera_calibration->ParseRectification(opt->getValue("rectright"),1)==0) {
            std::cout << "9 rectification matrix values are ";
            std::cout << "needed for the right camera\n";
            delete opt;
            return 0;
        }
        rectify_images = true;
    }

    if( opt->getValue( "translation" ) != NULL ) {
        if (camera_calibration->ParseExtrinsicTranslation(opt->getValue("translation"))==0) {
            std::cout << "3 extrinsic translation calibration values are ";
            std::cout << "needed\n";
            delete opt;
            return 0;
        }
    }

    if( opt->getValue( "rotation" ) != NULL ) {
        if (camera_calibration->ParseExtrinsicRotation(opt->getValue("rotation"))==0) {
            std::cout << "9 extrinsic rotation calibration values are ";
            std::cout << "needed\n";
            delete opt;
            return 0;
        }
    }

    if( opt->getValue("pose") != NULL ) {
        camera_calibration->ParsePose(opt->getValue("pose"));
    }

    if( opt->getValue("poserotation") != NULL ) {
        bool flip = false;
        if( opt->getValue("pointcloud") != NULL ) flip = true;
        camera_calibration->ParsePoseRotation(opt->getValue("poserotation"), flip);
    }

    if( opt->getValue("posetranslation") != NULL ) {
        camera_calibration->ParsePoseTranslation(opt->getValue("posetranslation"));
    }

    if( opt->getValue("backgroundmodel") ) {
        std::string background_model_filename = opt->getValue("backgroundmodel");
        FILE * fp = fopen(background_model_filename.c_str(),"rb");
        if (fp != NULL) {
            if (background_disparity_map==NULL) background_disparity_map = new float[ww*hh];
            if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
            if (fread((void*)background_disparity_map,sizeof(float),ww*hh,fp)>0) {
	    }
            fclose(fp);
        }
    }

    if( opt->getValue("calibrate") != NULL ) {
        int pattern_squares_x=6,pattern_squares_y=9,square_size_mm=24;
        if (camera_calibration->ParseCalibrationParameters(
                    opt->getValue("calibrate"),
                    pattern_squares_x, pattern_squares_y, square_size_mm)==0) {
            std::cout << "3 Calibration parameters are needed: ";
            std::cout << "squares across, squares down, square size (mm)\n";
        }
        else {
            camera_calibration->stereo_camera_calibrate(
                ww, hh, fps,
                pattern_squares_x, pattern_squares_y,
                square_size_mm,
                dev0, dev1,
                flip_left_image,
                flip_right_image,
                no_of_calibration_images, headless);
        }
        delete opt;
        return 0;
    }

    if( opt->getFlag( "overhead" ) ) {
        overhead_view = true;
        if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
    }

    if( opt->getFlag( "vcamera" ) ) {
        virtual_camera_view = true;
        if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
    }

    if( opt->getValue( "pointcloud" ) != NULL ) {
        show_disparity_map = true;
        point_cloud_filename = opt->getValue("pointcloud");
        if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
    }

    if( opt->getValue("background") != NULL ) {
        background_image = cvLoadImage(opt->getValue("background"));
        if (original_left_image == NULL) original_left_image = cvCreateImage(cvSize(ww, hh), 8, 3);
    }

    if (((detect_obstacles) || (detect_objects)) && (opt->getValue("poserotation")!=NULL)) {
        // obtain the tilt angle from the pose rotation
        double rotation_vector[3];
        camera_calibration->GetPoseRotation(rotation_vector);
        tilt_angle_degrees = rotation_vector[0];
        rotation_vector[0] = rotation_vector[0];
        camera_calibration->SetPoseRotation(rotation_vector);
    }

    delete opt;

    if ((show_disparity_map) && (!rectify_images) ) {
        std::cout << "Images need to be rectified before using ELAS.  You may need to recalibrate using --calibrate.\n";
        return 0;
    }

    //Camera c(dev0.c_str(), ww, hh, fps);
    //Camera c2(dev1.c_str(), ww, hh, fps);
    StereoCamera stereocam(dev0.c_str(), dev1.c_str(), ww, hh, fps);

    std::string left_image_title = "Left image";
    std::string right_image_title = "Right image";

    if (show_features) {
        left_image_title = "Left image features";
        right_image_title = "Right image features";
    }
    if (show_regions) {
        left_image_title = "Left image regions";
        right_image_title = "Right image regions";
    }
    if (show_FAST) left_image_title = "FAST corners";
    if (show_matches) left_image_title = "Stereo matches";
    if (show_depthmap) left_image_title = "Depth map";
    if (show_histogram) right_image_title = "Disparity histograms (L/R/All)";
    if (show_anaglyph) left_image_title = "Anaglyph";
    if (show_disparity_map) left_image_title = "Disparity map (ELAS)";
    if (background_image!=NULL) left_image_title = "Background substitution";
    if (overhead_view) left_image_title = "Overhead";
    if (virtual_camera_view) left_image_title = "Virtual Camera";
    if (detect_obstacles) left_image_title = "Obstacles";
    if (detect_objects) left_image_title = "objects";

    //cout<<c.setSharpness(3)<<"   "<<c.minSharpness()<<"  "<<c.maxSharpness()<<" "<<c.defaultSharpness()<<endl;

    if ((!save_images) &&
            (!headless) &&
            (stereo_matches_filename == "")) {

        cvNamedWindow(left_image_title.c_str(), CV_WINDOW_AUTOSIZE);
        if ((!show_matches) &&
                (!show_FAST) &&
                (!show_depthmap) &&
                (!show_anaglyph) &&
                (!show_disparity_map)) {
            cvNamedWindow(right_image_title.c_str(), CV_WINDOW_AUTOSIZE);
        }
    }

    IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);
    unsigned char *l_=(unsigned char *)l->imageData;

    IplImage *r=cvCreateImage(cvSize(ww, hh), 8, 3);
    unsigned char *r_=(unsigned char *)r->imageData;

    /* feature detection params */
    int inhibition_radius = 6;
    unsigned int minimum_response = 25;

    /* matching params */
    int ideal_no_of_matches = 400;

    /* These weights are used during matching of stereo features.
     * You can adjust them if you wish */
    int learnDesc = 18*5;  /* weight associated with feature descriptor match */
    int learnLuma = 7*5;   /* weight associated with luminance match */
    int learnDisp = 1;   /* weight associated with disparity (bias towards smaller disparities) */
    int learnGrad = 4;  /* weight associated with horizontal gradient */
    int groundPrior = 200; /* weight for ground plane prior */

    svs* lcam = new svs(ww, hh);
    svs* rcam = new svs(ww, hh);
    //motionmodel* motion = new motionmodel();
    fast* corners_left = new fast();

    unsigned char* buffer = NULL;
    unsigned char* depthmap_buffer = NULL;

    linefit *lines = new linefit();

#ifdef GSTREAMER
    /*
     * Send the video over a network for use in embedded applications
     * using the gstreamer library.
     */
    GstElement* l_source = NULL;
    GstElement* r_source = NULL;
    GstBuffer* l_app_buffer = NULL;
    GstBuffer* r_app_buffer = NULL;
    GstFlowReturn ret;

    // Yuck
    std::stringstream lp_str;
    lp_str << start_port;
    std::stringstream rp_str;
    rp_str << start_port + 1;

    std::string caps;

    if (stream) {
        // Initialise gstreamer and glib
        gst_init( NULL, NULL );
        GError* l_error = 0;
        GError* r_error = 0;
        GstElement* l_pipeline = 0;
        GstElement* r_pipeline = 0;

        caps = "image/jpeg";

        // Can replace this pipeline with anything you like (udpsink, videowriters etc)
        std::string l_pipetext = "appsrc name=appsource caps="+ caps +
                                 " ! jpegdec ! ffmpegcolorspace ! queue ! jpegenc ! multipartmux ! tcpserversink port=" + lp_str.str();
        std::string r_pipetext = "appsrc name=appsource caps="+ caps +
                                 " ! jpegdec ! ffmpegcolorspace ! queue ! jpegenc ! multipartmux ! tcpserversink port=" + rp_str.str();

        // Create the left image pipeline
        l_pipeline = gst_parse_launch( l_pipetext.c_str(), &l_error );

        // If needed, create right image pipeline
        if ((!show_matches) &&
                (!show_FAST) &&
                (!show_depthmap) &&
                (!show_anaglyph) &&
                (!show_disparity_map)) {
            r_pipeline = gst_parse_launch( r_pipetext.c_str(), &r_error );
        }

        // Seperate errors in case of port clash
        if( l_error == NULL ) {
            l_source = gst_bin_get_by_name( GST_BIN( l_pipeline ), "appsource" );
            gst_app_src_set_caps( (GstAppSrc*) l_source, gst_caps_from_string( caps.c_str() ) );
            gst_element_set_state( l_pipeline, GST_STATE_PLAYING );
            cout << "Streaming started on port " << start_port << endl;
            cout << "Watch stream with the command:" << endl;
            cout << "gst-launch tcpclientsrc host=[ip] port=" << start_port << " ! multipartdemux ! jpegdec ! autovideosink" << endl;
        } else {
            cout << "A gstreamer error occurred: " << l_error->message << endl;
        }

        // Cannot rely on pipeline, as there maybe a situation where the pipeline is null
        if ((!show_matches) &&
                (!show_FAST) &&
                (!show_depthmap) &&
                (!show_anaglyph) &&
                (!show_disparity_map)) {
            if( r_error == NULL ) {
                r_source = gst_bin_get_by_name( GST_BIN( r_pipeline ), "appsource" );
                gst_app_src_set_caps( (GstAppSrc*) r_source, gst_caps_from_string( caps.c_str() ) );
                gst_element_set_state( r_pipeline, GST_STATE_PLAYING );
                cout << "Streaming started on port " << start_port + 1 << endl;
                cout << "Watch stream with the command:" << endl;
                cout << "gst-launch tcpclientsrc host=[ip] port=" << start_port + 1 << " ! multipartdemux ! jpegdec ! autovideosink" << endl;
            } else {
                cout << "A gstreamer error occurred: " << r_error->message << endl;
            }
        }
    }
#endif

    // dense disparity
    unsigned int* disparity_space = NULL;
    unsigned int* disparity_map = NULL;

    IplImage* hist_image0 = NULL;
    IplImage* hist_image1 = NULL;

    float * virtual_camera_depth=NULL;
    CvMat * virtual_camera_rotation_matrix=NULL;
    CvMat * virtual_camera_translation=NULL;
    CvMat * virtual_camera_rotation_vector=NULL;
    CvMat * virtual_camera_points=NULL;
    CvMat * virtual_camera_image_points=NULL;

    /*
        gridmap3d * grid = NULL;
        if (point_cloud_filename != "") {
            grid = new gridmap3d(256,256,10);
        }
    */

    while(1) {

        if (!stereocam.GrabFrames()) {
            printf("Failed to acquire images\n");
            break;
        }

        stereocam.RetrieveLeftImage(l);
        stereocam.RetrieveRightImage(r);

        if (flip_right_image) {
            if (buffer == NULL) {
                buffer = new unsigned char[ww * hh * 3];
            }
            rcam->flip(r_, buffer);
        }

        if (flip_left_image) {
            if (buffer == NULL) {
                buffer = new unsigned char[ww * hh * 3];
            }
            lcam->flip(l_, buffer);
        }

        if (rectify_images) {
			#pragma omp parallel for
            for (int cam = 0; cam <= 1; cam++) {
                if (cam == 0) {
                    camera_calibration->RectifyImage(0, ww, hh, l_, -calibration_offset_y);
                }
                else {
                    camera_calibration->RectifyImage(1, ww, hh, r_, +calibration_offset_y);
                }
            }
        }

        if (zoom > 0) {
            unsigned char *l2_ = new unsigned char[ww*hh*3];
            unsigned char *r2_ = new unsigned char[ww*hh*3];
            memcpy((void*)l2_,l_,ww*hh*3);
            memcpy((void*)r2_,r_,ww*hh*3);

            expand(l2_,ww,hh,zoom_tx,zoom_ty,zoom_bx,zoom_by,l_);
            expand(r2_,ww,hh,zoom_tx,zoom_ty,zoom_bx,zoom_by,r_);
            delete[] l2_;
            delete[] r2_;
        }

        if (histogram_equalisation) {

            if (hist_image0 == NULL) {
                hist_image0 = cvCreateImage( cvGetSize(l), IPL_DEPTH_8U, 1 );
                hist_image1 = cvCreateImage( cvGetSize(l), IPL_DEPTH_8U, 1 );
            }

			#pragma omp parallel for
            for (int i = 0; i < 2; i++) {
                unsigned char *img = l_;
                IplImage* hist_image = hist_image0;
                if (i > 0) {
                    img = r_;
                    hist_image = hist_image1;
                }
                if ((background_image!=NULL) ||
                        (background_disparity_map!=NULL) ||
                        (overhead_view) ||
                        (virtual_camera_view) ||
                        (detect_obstacles) ||
                        (detect_objects) ||
                        (point_cloud_filename!="")) {
                    memcpy((void*)(original_left_image->imageData),l_,ww*hh*3);
                }
                svs::histogram_equalise(
                    hist_image,
                    img, ww, hh);
            }
        }

        if ((show_features) || (show_matches)) {
			#pragma omp parallel for
            for (int cam = 1; cam >= 0; cam--) {

                int calib_offset_x = 0;
                int calib_offset_y = 0;
                unsigned char* rectified_frame_buf = NULL;
                int no_of_feats = 0;
                int no_of_feats_horizontal = 0;
                svs* stereocam = NULL;
                if (cam == 0) {
                    rectified_frame_buf = l_;
                    stereocam = lcam;
                    calib_offset_x = 0;
                    calib_offset_y = 0;
                }
                else {
                    rectified_frame_buf = r_;
                    stereocam = rcam;
                    calib_offset_x = calibration_offset_x;
                    calib_offset_y = calibration_offset_y;
                }

                no_of_feats = stereocam->get_features_vertical(
                                  rectified_frame_buf,
                                  inhibition_radius,
                                  minimum_response,
                                  calib_offset_x,
                                  calib_offset_y,
                                  0);

                if ((cam == 0) || (show_features) || (show_lines)) {
                    no_of_feats_horizontal = stereocam->get_features_horizontal(
                                                 rectified_frame_buf,
                                                 inhibition_radius,
                                                 minimum_response,
                                                 calib_offset_x,
                                                 calib_offset_y,
                                                 0);
                }

                if (show_lines) {
                    lines->vertically_oriented(
                        no_of_feats,
                        stereocam->feature_x,
                        stereocam->features_per_row,
                        SVS_VERTICAL_SAMPLING,
                        10*320/SVS_MAX_IMAGE_WIDTH);
                    lines->horizontally_oriented(
                        no_of_feats_horizontal,
                        stereocam->feature_y,
                        stereocam->features_per_col,
                        SVS_HORIZONTAL_SAMPLING,
                        6*320/SVS_MAX_IMAGE_WIDTH);
                    for (int line = 0; line < lines->line_vertical[0]; line++) {
                        drawing::drawLine(rectified_frame_buf,ww,hh,
                                          lines->line_vertical[line*5 + 1] - calib_offset_x,
                                          lines->line_vertical[line*5 + 2] - calib_offset_y,
                                          lines->line_vertical[line*5 + 3] - calib_offset_x,
                                          lines->line_vertical[line*5 + 4] - calib_offset_y,
                                          255,0,0,
                                          0,false);
                    }
                    for (int line = 0; line < lines->line_horizontal[0]; line++) {
                        drawing::drawLine(rectified_frame_buf,ww,hh,
                                          lines->line_horizontal[line*5 + 1] - calib_offset_x,
                                          lines->line_horizontal[line*5 + 2] - calib_offset_y,
                                          lines->line_horizontal[line*5 + 3] - calib_offset_x,
                                          lines->line_horizontal[line*5 + 4] - calib_offset_y,
                                          0,255,0,
                                          0,false);
                    }
                }

                //printf("cam %d:  %d\n", cam, no_of_feats);

                /* display the features */
                if (show_features) {

                    /* vertically oriented features */
                    int row = 0;
                    int feats_remaining = stereocam->features_per_row[row];

                    for (int f = 0; f < no_of_feats; f++, feats_remaining--) {

                        int x = (int)stereocam->feature_x[f] / SVS_SUB_PIXEL;
                        int y = 4 + (row * SVS_VERTICAL_SAMPLING);

                        if (cam == 0) {
                            drawing::drawCross(rectified_frame_buf, ww, hh, x, y, 2, 255, 0, 0, 0);
                        }
                        else {
                            x -= calibration_offset_x;
                            y += calibration_offset_y;
                            drawing::drawCross(rectified_frame_buf, ww, hh, x, y, 2, 255, 0, 0, 0);
                        }

                        /* move to the next row */
                        if (feats_remaining <= 0) {
                            row++;
                            feats_remaining = stereocam->features_per_row[row];
                        }
                    }

                    /* horizontally oriented features */
                    int col = 0;
                    feats_remaining = stereocam->features_per_col[col];

                    for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--) {

                        int y = (int)stereocam->feature_y[f];
                        int x = 4 + (col * SVS_HORIZONTAL_SAMPLING);

                        if (cam == 0) {
                            drawing::drawCross(rectified_frame_buf, ww, hh, x, y, 2, 0, 255, 0, 0);
                        }
                        else {
                            x += calibration_offset_x;
                            y -= calibration_offset_y;
                            drawing::drawCross(rectified_frame_buf, ww, hh, x, y, 2, 0, 255, 0, 0);
                        }

                        /* move to the next column */
                        if (feats_remaining <= 0) {
                            col++;
                            feats_remaining = stereocam->features_per_col[col];
                        }
                    }
                }
            }
        }

        /* set ground plane parameters */
        lcam->enable_ground_priors = enable_ground_priors;
        lcam->ground_y_percent = ground_y_percent;

        matches = 0;
        if (show_matches) {
            matches = lcam->match(
                          rcam,
                          ideal_no_of_matches,
                          max_disparity_percent,
                          learnDesc,
                          learnLuma,
                          learnDisp,
                          learnGrad,
                          groundPrior,
                          use_priors);
        }

        if (show_regions) {
            lcam->enable_segmentation = 1;
            if (lcam->low_contrast != NULL) {
                lcam->segment(l_, matches);
                memset((void*)l_, '\0', ww*hh*3);
                int min_vol = ww*hh/500;
                int r=255, g=0, b=0;
                int i = 0;
                for (int y = 0; y < hh; y++) {
                    for (int x = 0; x < ww; x++, i++) {
                        int ID = lcam->low_contrast[i];
                        if ((ID > 0) && (ID < lcam->no_of_regions )) {
                            if ((int)lcam->region_volume[ID] > min_vol) {
                                int disp = lcam->region_disparity[ID*3];
                                int slope_x = (int)lcam->region_disparity[ID*3+1] - 127;
                                int slope_y = (int)lcam->region_disparity[ID*3+2] - 127;
                                if (disp != 255) {
                                    if (!((slope_x == 0) && (slope_y == 0))) {
                                        int region_tx = lcam->region_bounding_box[ID*4];
                                        int region_ty = lcam->region_bounding_box[ID*4+1];
                                        int region_bx = lcam->region_bounding_box[ID*4+2];
                                        int region_by = lcam->region_bounding_box[ID*4+3];
                                        int disp_horizontal = 0;
                                        if (region_bx > region_tx) {
                                            disp_horizontal =
                                                -(slope_x/2) + ((x - region_tx) * slope_x /
                                                                (region_bx - region_tx));
                                        }
                                        int disp_vertical = 0;
                                        if (region_by > region_ty) {
                                            disp_vertical =
                                                -(slope_y/2) + ((y - region_ty) * slope_y /
                                                                (region_by - region_ty));
                                        }
                                        disp += disp_horizontal + disp_vertical;
                                        if (disp < 0) disp = 0;
                                    }
                                    r = 20+disp*5;
                                    if (r > 255) r = 255;
                                    g = r;
                                    b = r;
                                    l_[i*3] = b;
                                    l_[i*3+1] = g;
                                    l_[i*3+2] = r;
                                }
                                /*
                                r = lcam->region_colour[ID*3+2];
                                g = lcam->region_colour[ID*3+1];
                                b = lcam->region_colour[ID*3];
                                l_[i*3] = b;
                                l_[i*3+1] = g;
                                l_[i*3+2] = r;
                                */
                            }
                        }
                    }
                }

                /*
                for (int i = 0; i < lcam->no_of_regions; i++) {
                	if ((int)lcam->region_volume[i] > min_vol) {
                		drawing::drawCross(
                				l_, ww, hh,
                				(int)lcam->region_centre[i*2],
                				(int)lcam->region_centre[i*2+1],
                				4, 255,0,0, 1);
                	}
                } */

                if (lcam->region_history_index > -1) {
                    for (i = 0; i < lcam->prev_region_centre[lcam->region_history_index][0]; i++) {
                        int ctr = lcam->region_history_index;
                        int j0 = lcam->prev_region_centre[ctr][i*4+3];
                        int j = j0;
                        int k = lcam->prev_region_centre[ctr][i*4+4];
                        int prev_x = lcam->prev_region_centre[ctr][i*4+1];
                        int prev_y = lcam->prev_region_centre[ctr][i*4+2];

                        int n = 0;
                        while ((j != 65535) && (n < SVS_REGION_HISTORY-1)) {
                            int x = lcam->prev_region_centre[j][k*4+1];
                            int y = lcam->prev_region_centre[j][k*4+2];
                            int j2 = lcam->prev_region_centre[j][k*4+3];
                            k = lcam->prev_region_centre[j][k*4+4];
                            j = j2;
                            if (j == lcam->region_history_index) break;
                            drawing::drawLine(l_,ww,hh,prev_x,prev_y,x,y,0,255,0,1,false);
                            prev_x = x;
                            prev_y = y;
                            n++;
                        }
                    }
                }
            }
        }

        /* show disparity histogram */
        if (show_histogram) {
            memset(disparity_histogram[0], 0, SVS_MAX_IMAGE_WIDTH * sizeof(int));
            memset(disparity_histogram[1], 0, SVS_MAX_IMAGE_WIDTH * sizeof(int));
            memset(disparity_histogram[2], 0, SVS_MAX_IMAGE_WIDTH * sizeof(int));
            memset(r_, 0, ww * hh * 3 * sizeof(unsigned char));
            int hist_max[3];
            hist_max[0] = 0;
            hist_max[1] = 0;
            hist_max[2] = 0;

            for (int i = 0; i < matches; i++) {
                int x = lcam->svs_matches[i*5 + 1]/SVS_SUB_PIXEL;
                int disp = lcam->svs_matches[i*5 + 3]/SVS_SUB_PIXEL;
                disparity_histogram[2][disp]++;
                if (x < ww/2)
                    disparity_histogram[0][disp]++;
                else
                    disparity_histogram[1][disp]++;
                if (disparity_histogram[0][disp] > hist_max[0]) hist_max[0] = disparity_histogram[0][disp];
                if (disparity_histogram[1][disp] > hist_max[1]) hist_max[1] = disparity_histogram[1][disp];
                if (disparity_histogram[2][disp] > hist_max[2]) hist_max[2] = disparity_histogram[2][disp];
            }
            int max_disparity_pixels = max_disparity_percent * ww / 100;

            int mass[3];
            mass[0] = 0;
            mass[1] = 0;
            mass[2] = 0;
            int disp2[3];
            disp2[0] = 0;
            disp2[1] = 0;
            disp2[2] = 0;
            int hist_thresh[3];
            hist_thresh[0] = hist_max[0] / 4;
            hist_thresh[1] = hist_max[1] / 4;
            hist_thresh[2] = hist_max[2] / 4;
            for (int d = 3; d < max_disparity_pixels-1; d++) {
                for (int i = 0; i < 3; i++) {
                    if (disparity_histogram[i][d] > hist_thresh[i]) {
                        int m = disparity_histogram[i][d] + disparity_histogram[i][d-1] + disparity_histogram[i][d+1];
                        mass[i] += m;
                        disp2[i] += m * d;
                    }
                }
            }
            for (int i = 0; i < 3; i++) {
                if (mass[i] > 0) disp2[i] /= mass[i];
            }

            int tx=0,ty=0,bx=0,by=0;
            for (int i = 0; i < 3; i++) {
                if (hist_max[i] > 0) {
                    switch(i) {
                    case 0: {
                        tx = 0;
                        ty = 0;
                        bx = ww/2;
                        by = hh/2;
                        break;
                    }
                    case 1: {
                        tx = ww/2;
                        ty = 0;
                        bx = ww;
                        by = hh/2;
                        break;
                    }
                    case 2: {
                        tx = 0;
                        ty = hh/2;
                        bx = ww;
                        by = hh;
                        break;
                    }
                    }

                    for (int x = tx; x < bx; x++) {
                        int disp = (x-tx) * max_disparity_pixels / (bx-tx);
                        int h2 = disparity_histogram[i][disp] * (by-ty) / hist_max[i];
                        for (int y = by-1; y > by-1-h2; y--) {
                            int n = ((y * ww) + x) * 3;
                            r_[n] = 255;
                            r_[n+1] = 255;
                            r_[n+2] = 255;
                        }
                    }

                    int xx = tx + (disp2[i] * (bx-tx) / max_disparity_pixels);
                    drawing::drawLine(r_, ww, hh, xx, ty, xx, by-1, 255,0,0,0,false);
                }
            }

            drawing::drawLine(r_, ww, hh, ww/2, 0, ww/2, hh/2, 0,255,0,1,false);
            drawing::drawLine(r_, ww, hh, 0, hh/2, ww-1, hh/2, 0,255,0,1,false);
        }

        /* show disparity as spots */
        if (show_matches) {
            for (int i = 0; i < matches; i++) {
                if ((lcam->svs_matches[i*5] > 0) &&
                        (lcam->svs_matches[i*5+4] != 9999)) {
                    int x = lcam->svs_matches[i*5 + 1]/SVS_SUB_PIXEL;
                    int y = lcam->svs_matches[i*5 + 2];
                    int disp = lcam->svs_matches[i*5 + 3]/SVS_SUB_PIXEL;
                    if (disp < ww/2) drawing::drawBlendedSpot(l_, ww, hh, x, y, 1 + (disp/6), 0, 255, 0);
                }
            }
        }

        if (show_disparity_map) {
            elas_disparity_map(l_, r_, ww, hh, I1, I2, left_disparities, right_disparities, elas);

            if (learn_background_filename != "") {
                for (int i = 0; i < ww*hh; i++) {
                    if (left_disparities[i] > 1) {
                        background_disparity_map[i] += left_disparities[i];
                        background_disparity_map_hits[i]++;
                    }
                }
                if (skip_frames <= 0) {
                    for (int i = 0; i < ww*hh; i++) {
                        if (background_disparity_map_hits[i] > 0) {
                            background_disparity_map[i] /= background_disparity_map_hits[i];
                        }
                    }
                    FILE * fp = fopen(learn_background_filename.c_str(),"wb");
                    if (fp != NULL) {
                        fwrite(background_disparity_map,sizeof(float),ww*hh,fp);
                        fclose(fp);
                    }
                    break;
                }
            }

            // remove background model if necessary
            int min_disparity = disparity_threshold_percent*255/100;
            if (background_disparity_map != NULL) {
                for (int i = 1; i < ww*(hh-1); i++) {
                    if ((left_disparities[i] < 1) || (background_disparity_map[i] < 1) ||
                            (left_disparities[i]+left_disparities[i+1]+left_disparities[i+ww] <
                             (background_disparity_map[i]) * 3 *(100+min_disparity) / 100)) {
                        left_disparities[i]=0;
                    }
                }
            }

            if (point_cloud_filename != "") {
                if (skip_frames <= 0) {
                    // convert disparity map to 3D points
                    pointcloud::disparity_map_to_3d_points(
                        left_disparities,ww,hh,
                        camera_calibration->disparityToDepth,
                        camera_calibration->pose,
                        disparity_image, points_image);

                    if (buffer == NULL) {
                        buffer = new unsigned char[ww * hh * 3];
                    }
                    if (histogram_equalisation) {
                        memcpy((void*)buffer,(void*)(original_left_image->imageData),ww*hh*3);
                    }
                    else {
                        memcpy((void*)buffer,(void*)l_,ww*hh*3);
                    }

                    // stop cameras before saving
                    //c.StopCam();
                    //c2.StopCam();
                    stereocam.StopCam();

                    pointcloud::save(
                        buffer,points_image,max_range_mm,
                        camera_calibration->pose,
                        cvmGet(camera_calibration->extrinsicTranslation,0,0),
                        point_cloud_filename);
                    break;
                }
            }
            else {
                if ((virtual_camera_view) || (detect_obstacles) || (detect_objects)) {
                    // convert disparity map to 3D points
                    pointcloud::disparity_map_to_3d_points(
                        left_disparities,ww,hh,
                        camera_calibration->disparityToDepth,
                        camera_calibration->pose,
                        disparity_image, points_image);

                    if (buffer == NULL) {
                        buffer = new unsigned char[ww * hh * 3];
                    }
                    if (histogram_equalisation) {
                        memcpy((void*)buffer,(void*)(original_left_image->imageData),ww*hh*3);
                    }
                    else {
                        memcpy((void*)buffer,(void*)l_,ww*hh*3);
                    }

                    if ((detect_obstacles) || (detect_objects)) {
                        if (obstacle_map == NULL) {
                            obstacle_map = new int[obstacle_map_dimension*obstacle_map_dimension];
                        }
                        pointcloud::obstacle_map(
                            points_image,
                            obstacle_map_dimension,
                            obstacle_map_cell_size_mm,
                            camera_calibration->pose,
                            obstacle_map_relative_x_mm,
                            obstacle_map_relative_y_mm,
                            obstacle_map_threshold,
                            tilt_angle_degrees,
                            obstacle_map);

                        if (detect_objects) {

                            std::vector<std::vector<float> > objects;

                            pointcloud::find_objects(
                                object_format,
                                buffer, points_image,
                                obstacle_map_dimension,
                                obstacle_map_cell_size_mm,
                                camera_calibration->pose,
                                obstacle_map_relative_x_mm, obstacle_map_relative_y_mm,
                                obstacle_map_threshold,
                                tilt_angle_degrees,
                                obstacle_map,
                                object_min_area_mm2,
                                object_max_area_mm2,
                                BGR,
                                objects);

                            if (save_mesh_filename!="") {
                                if (skip_frames <= 0) {
                                    // stop cameras before saving
                                    //c.StopCam();
                                    //c2.StopCam();
                                	stereocam.StopCam();

                                    // save the object as a mesh model
                                    pointcloud::save_largest_object(
                                        save_mesh_filename,object_format,false,ww,hh,
                                        camera_calibration->pose,
                                        cvmGet(camera_calibration->extrinsicTranslation,0,0),
                                        objects);
                                    printf("Saved %s\n", save_mesh_filename.c_str());
                                    break;
                                }
                            }
                            //printf("Objects %d\n",(int)objects.size());
                        }
                    }
                    else {
                        if (save_mesh_filename!="") {
                            if (skip_frames <= 0) {

                                // stop cameras before saving
                                //c.StopCam();
                                //c2.StopCam();
                            	stereocam.StopCam();

                                // save all points as a mesh model
                                std::vector<float> points;
                                pointcloud::export_points(
                                    object_format,
                                    buffer, points_image,
                                    camera_calibration->pose,
                                    tilt_angle_degrees,
                                    BGR, points, max_range_mm);

                                if (object_format == POINT_CLOUD_FORMAT_X3D) {
                                    pointcloud::save_mesh_x3d(
                                        save_mesh_filename, ww,hh,
                                        camera_calibration->pose,
                                        cvmGet(camera_calibration->extrinsicTranslation,0,0),
                                        points);
                                }
                                if (object_format == POINT_CLOUD_FORMAT_STL) {
                                    pointcloud::save_stl_ascii(
                                        save_mesh_filename, ww,hh,
                                        camera_calibration->pose,
                                        cvmGet(camera_calibration->extrinsicTranslation,0,0),
                                        points);
                                }
                                printf("Saved %s\n", save_mesh_filename.c_str());
                                break;
                            }
                        }
                    }

                    pointcloud::virtual_camera(
                        buffer, points_image,
                        camera_calibration->pose,
                        camera_calibration->intrinsicCalibration_left,
                        camera_calibration->distortion_left,
                        max_range_mm,
                        virtual_camera_depth,
                        virtual_camera_rotation_matrix,
                        virtual_camera_translation,
                        virtual_camera_rotation_vector,
                        virtual_camera_points,
                        virtual_camera_image_points,
                        view_point_cloud,
                        l_);
                }
                else {
                    if (overhead_view) {
                        // convert disparity map to 3D points
                        pointcloud::disparity_map_to_3d_points(
                            left_disparities,ww,hh,
                            camera_calibration->disparityToDepth,
                            camera_calibration->pose,
                            disparity_image, points_image);

                        int max_height_mm = 2000;

                        if (buffer == NULL) {
                            buffer = new unsigned char[ww * hh * 3];
                        }
                        if (histogram_equalisation) {
                            memcpy((void*)buffer,(void*)(original_left_image->imageData),ww*hh*3);
                        }
                        else {
                            memcpy((void*)buffer,(void*)l_,ww*hh*3);
                        }

                        pointcloud::show(
                            points_image, left_disparities,
                            buffer,
                            camera_calibration->pose,
                            max_range_mm, max_height_mm,
                            2,
                            ww,hh,l_);

                        //pointcloud::save(l_,points_image,max_range_mm,ww,hh,camera_calibration->pose,point_cloud_filename);
                        //break;
                        /*
                                            grid->insert(0,0,0,
                                                (float*)points_image->imageData,ww,hh,l_);
                                            grid->show(ww,hh,l_,1);
                        */
                    }
                    else {
                        int max_disparity_pixels = SVS_MAX_IMAGE_WIDTH * max_disparity_percent / 100;

                        if (background_disparity_map != NULL) {
                            // subtract a background disparity map
                            if (histogram_equalisation) {
                                memcpy((void*)l_,(void*)(original_left_image->imageData),ww*hh*3);
                            }
                            unsigned char * background_image_ = NULL;
                            if (background_image != NULL) background_image_ = (unsigned char*)background_image->imageData;
                            int i = 0, n;
                            for (int y = 0; y < hh; y++) {
                                for (int x = 0; x < ww; x++, i++) {
                                    if (left_disparities[i] == 0) {
                                        if (background_image != NULL) {
                                            int yy = y * background_image->height / hh;
                                            int xx = x * background_image->width / ww;
                                            n = ((yy*background_image->width) + xx)*3;
                                            l_[i*3] = background_image_[n];
                                            l_[i*3+1] = background_image_[n+1];
                                            l_[i*3+2] = background_image_[n+2];
                                        }
                                        else {
                                            l_[i*3] = 0;
                                            l_[i*3+1] = 0;
                                            l_[i*3+2] = 0;
                                        }
                                    }
                                }
                            }
                        }
                        else {
                            if (background_image != NULL) {
                                // use a background image with a disparity threshold
                                unsigned char * background_image_ = (unsigned char*)background_image->imageData;

                                if (histogram_equalisation) {
                                    memcpy((void*)l_,(void*)(original_left_image->imageData),ww*hh*3);
                                }

                                int i = 0, n;
                                for (int y = 0; y < hh; y++) {
                                    int yy = y * background_image->height / hh;
                                    for (int x = 0; x < ww; x++, i++) {
                                        if (left_disparities[i] <= min_disparity) {
                                            int xx = x * background_image->width / ww;
                                            n = ((yy*background_image->width) + xx)*3;
                                            l_[i*3] = background_image_[n];
                                            l_[i*3+1] = background_image_[n+1];
                                            l_[i*3+2] = background_image_[n+2];
                                        }
                                    }
                                }
                            }
                            else {
                                // show disparity map
                                if (!colour_disparity_map) {
                                    // monochrome disparities
                                    float mult = 255.0f/max_disparity_pixels;
                                    for (int i = 0; i < ww*hh; i++) {
                                        if (left_disparities[i] > min_disparity) {
                                            l_[i*3] = (unsigned char)(left_disparities[i]*mult);
                                        }
                                        else {
                                            l_[i*3]=0;
                                        }
                                        l_[i*3+1] = l_[i*3];
                                        l_[i*3+2] = l_[i*3];
                                    }
                                }
                                else {
                                    // colour coded disparities
                                    for (int i = 0; i < ww*hh; i++) {
                                        if (left_disparities[i] > min_disparity) {
                                            float val = min(( *(((float*)left_disparities)+i) )*0.01f,1.0f);
                                            if (val <= 0) {
                                                l_[3*i+0] = 0;
                                                l_[3*i+1] = 0;
                                                l_[3*i+2] = 0;
                                            } else {
                                                float h2 = 6.0f * (1.0f - val);
                                                unsigned char x  = (unsigned char)((1.0f - fabs(fmod(h2, 2.0f) - 1.0f))*255);
                                                if (0 <= h2&&h2<1) {
                                                    l_[3*i+0] = 255;
                                                    l_[3*i+1] = x;
                                                    l_[3*i+2] = 0;
                                                }
                                                else if (1<=h2&&h2<2)  {
                                                    l_[3*i+0] = x;
                                                    l_[3*i+1] = 255;
                                                    l_[3*i+2] = 0;
                                                }
                                                else if (2<=h2&&h2<3)  {
                                                    l_[3*i+0] = 0;
                                                    l_[3*i+1] = 255;
                                                    l_[3*i+2] = x;
                                                }
                                                else if (3<=h2&&h2<4)  {
                                                    l_[3*i+0] = 0;
                                                    l_[3*i+1] = x;
                                                    l_[3*i+2] = 255;
                                                }
                                                else if (4<=h2&&h2<5)  {
                                                    l_[3*i+0] = x;
                                                    l_[3*i+1] = 0;
                                                    l_[3*i+2] = 255;
                                                }
                                                else if (5<=h2&&h2<=6) {
                                                    l_[3*i+0] = 255;
                                                    l_[3*i+1] = 0;
                                                    l_[3*i+2] = x;
                                                }
                                            }
                                        }
                                        else {
                                            l_[3*i+0] = 0;
                                            l_[3*i+1] = 0;
                                            l_[3*i+2] = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        /* show depth map */
        if (show_depthmap) {
            if (depthmap_buffer == NULL) {
                depthmap_buffer = new unsigned char[ww*hh*3];
                memset(depthmap_buffer, 0, ww*hh*3*sizeof(unsigned char));
            }
            memset(l_, 0, ww*hh*3*sizeof(unsigned char));
            if (matches == 0) matches = prev_matches;
            for (int i = 0; i < matches; i++) {
                int x = lcam->svs_matches[i*5 + 1]/SVS_SUB_PIXEL;
                int y = lcam->svs_matches[i*5 + 2];
                int disp = lcam->svs_matches[i*5 + 3]/SVS_SUB_PIXEL;
                int max_disparity_pixels = max_disparity_percent * ww / 100;
                int disp_intensity = 50 + (disp * 300 / max_disparity_pixels);
                if (disp_intensity > 255) disp_intensity = 255;
                int radius = 10 + (disp/8);
                if (use_priors != 0) {
                    int n = (y*ww+x)*3;
                    int disp_intensity2 = disp_intensity;
                    disp_intensity = (disp_intensity + depthmap_buffer[n]) / 2;
                    drawing::drawBlendedSpot(depthmap_buffer, ww, hh, x, y, radius, disp_intensity2, disp_intensity2, disp_intensity2);
                }
                drawing::drawBlendedSpot(l_, ww, hh, x, y, radius, disp_intensity, disp_intensity, disp_intensity);
            }
            prev_matches = matches;
        }

        if (show_anaglyph) {
            int n = 0;
            int max = (ww * hh * 3) - 3;
            for (int y = 0; y < hh; y++) {
                int y2 = y + calibration_offset_y;
                for (int x = 0; x < ww; x++, n += 3) {
                    int x2 = x + calibration_offset_x;
                    int n2 = ((y2 * ww) + x2) * 3;
                    if ((n2 > -1) && (n2 < max)) {
                        l_[n] = 0;
                        l_[n+1] = l_[n+2];
                        l_[n+2] = r_[n2+2];
                    }
                }
            }
        }

        /* log stereo matches */
        if ((log_stereo_matches_filename != "")) {
            if (lcam->log_matches(log_stereo_matches_filename, l_, matches, true)) {
                printf("%d stereo matches logged to %s\n", matches, log_stereo_matches_filename.c_str());
            }
        }

        if (skip_frames == 0) {

            if (save_period_sec > 0) {
                char filename[256];
                sprintf((char*)filename,"stereo_%d_0.jpg", image_index);
                cvSaveImage(filename, l);
                sprintf((char*)filename,"stereo_%d_1.jpg", image_index);
                if ((!show_matches) &&
                        (!show_FAST) &&
                        (!show_depthmap) &&
                        (!show_anaglyph) &&
                        (!show_disparity_map))
                    cvSaveImage(filename, r);
                image_index++;
                sleep(save_period_sec);
            }

            /* save left and right images to file, then quit */
            if (save_images) {
                // stop cameras before saving
                //c.StopCam();
                //c2.StopCam();
            	stereocam.StopCam();

                std::string filename = save_filename + "0.jpg";
                cvSaveImage(filename.c_str(), l);
                filename = save_filename + "1.jpg";
                if ((!show_matches) &&
                        (!show_FAST) &&
                        (!show_depthmap) &&
                        (!show_anaglyph) &&
                        (!show_disparity_map))
                    cvSaveImage(filename.c_str(), r);

                /* save stereo matches */
                if ((stereo_matches_filename != "") && (!show_FAST) &&
                        ((skip_frames == 0) || (matches > 5))) {
                    lcam->save_matches(stereo_matches_filename, l_, matches, true);
                    printf("%d stereo matches saved to %s\n", matches, stereo_matches_filename.c_str());
                }

                break;
            }
        }

        /* save stereo matches to a file, then quit */
        if ((stereo_matches_filename != "") && (!save_images) && (!show_FAST) &&
                ((skip_frames == 0) || (matches > 5))) {
            lcam->save_matches(stereo_matches_filename, l_, matches, false);
            printf("%d stereo matches saved to %s\n", matches, stereo_matches_filename.c_str());
            break;
        }

        //motion->update(l_,ww,hh);
        //motion->show(l_,ww,hh);

        if (show_FAST) {
            /* load previous matches from file */
            if (stereo_matches_input_filename != "") {
                corners_left->load_matches(stereo_matches_input_filename, true);
                stereo_matches_input_filename = "";
            }

            /* locate corner features in the left image */
            corners_left->update(l_,ww,hh, desired_corner_features,1);

            /* assign disparity values to corner features */
            corners_left->match_interocular(
                ww, hh,
                matches, lcam->svs_matches);

            /* save stereo matches to a file, then quit */
            if ((stereo_matches_filename != "") && (!save_images) &&
                    ((skip_frames == 0) || (corners_left->get_no_of_disparities() > 50))) {
                /* save the matches */
                corners_left->save_matches(stereo_matches_filename, l_, ww, true);
                break;
            }

            /* save stereo matches to a file, then quit */
            if ((descriptors_filename != "") && (!save_images) &&
                    ((skip_frames == 0) || (corners_left->get_no_of_disparities() > 50))) {
                if (corners_left->save_descriptors(descriptors_filename, l_, ww, hh) > 40) {
                    break;
                }
            }

            corners_left->show(l_,ww,hh,1);
        }

#ifdef GSTREAMER
        /*
         * The streaming bit - seems a bit hacky, someone else can try
         * and convert an IPLImage directly to something GStreamer can handle.
         * My bitbanging abilities just aren't up to the task.
         */
        if (stream) {
            CvMat* l_buf;
            l_buf = cvEncodeImage(".jpg", l);

            l_app_buffer = gst_app_buffer_new( l_buf->data.ptr, l_buf->step, NULL, l_buf->data.ptr );
            g_signal_emit_by_name( l_source, "push-buffer", l_app_buffer, &ret );

            if ((!show_matches) &&
                    (!show_FAST) &&
                    (!show_depthmap) &&
                    (!show_anaglyph) &&
                    (!show_disparity_map)) {
                CvMat* r_buf;
                r_buf = cvEncodeImage(".jpg", r);

                r_app_buffer = gst_app_buffer_new( r_buf->data.ptr, r_buf->step, NULL, r_buf->data.ptr );
                g_signal_emit_by_name( r_source, "push-buffer", r_app_buffer, &ret );
            }
        }
#endif

        /* display the left and right images */
        if ((!save_images) && (!headless) && (stereo_matches_filename == "")) {
            cvShowImage(left_image_title.c_str(), l);
            if ((!show_matches) &&
                    (!show_FAST) &&
                    (!show_depthmap) &&
                    (!show_anaglyph) &&
                    (!show_disparity_map)) {
                cvShowImage(right_image_title.c_str(), r);
            }
        }

        skip_frames--;
        if (skip_frames < 0) skip_frames = 0;

        int wait = cvWaitKey(10) & 255;

        if ((virtual_camera_view) || (detect_obstacles) || (detect_objects)) {
            double displacement_mm = 5;
            double rotation_step_degrees = 0.5;
            if (wait==',') camera_calibration->translate_pose(-displacement_mm,0);
            if (wait=='.') camera_calibration->translate_pose(displacement_mm,0);
            if ((wait=='a') || (wait=='A')) camera_calibration->translate_pose(-displacement_mm,1);
            if ((wait=='z') || (wait=='Z')) camera_calibration->translate_pose(displacement_mm,1);
            if ((wait=='s') || (wait=='S')) camera_calibration->translate_pose(displacement_mm,2);
            if ((wait=='x') || (wait=='X')) camera_calibration->translate_pose(-displacement_mm,2);
            if (wait=='1') camera_calibration->rotate_pose(-rotation_step_degrees,0);
            if (wait=='2') camera_calibration->rotate_pose(rotation_step_degrees,0);
            if (wait=='3') camera_calibration->rotate_pose(-rotation_step_degrees,1);
            if (wait=='4') camera_calibration->rotate_pose(rotation_step_degrees,1);
            if (wait=='5') camera_calibration->rotate_pose(-rotation_step_degrees,2);
            if (wait=='6') camera_calibration->rotate_pose(rotation_step_degrees,2);
        }

        if ((wait=='c') || (wait=='C'))
        {
            cvSaveImage("l.jpg", l);
            cvSaveImage("r.jpg", r);
        }

        if( wait == 27 ) break;
    }

    /* destroy the left and right images */
    if ((!save_images) &&
            (stereo_matches_filename == "")) {

        cvDestroyWindow(left_image_title.c_str());
        if ((!show_matches) &&
                (!show_FAST) &&
                (!show_depthmap) &&
                (!show_anaglyph) &&
                (!show_disparity_map)) {
            cvDestroyWindow(right_image_title.c_str());
        }
    }

    cvReleaseImage(&l);
    cvReleaseImage(&r);
    if (hist_image0 != NULL) cvReleaseImage(&hist_image0);
    if (hist_image1 != NULL) cvReleaseImage(&hist_image1);
    if (disparity_image != NULL) cvReleaseImage(&disparity_image);
    if (points_image != NULL) cvReleaseImage(&points_image);

    delete lcam;
    delete rcam;
    delete corners_left;
    delete lines;
    if (background_disparity_map != NULL) delete [] background_disparity_map;
    if (background_disparity_map_hits != NULL) delete [] background_disparity_map_hits;
    if (buffer != NULL) delete [] buffer;
    if (depthmap_buffer != NULL) delete [] depthmap_buffer;
    if (disparity_space != NULL) delete [] disparity_space;
    if (disparity_map != NULL) delete [] disparity_map;

    if (elas!=NULL) {
        delete elas;
        delete [] I1;
        delete [] I2;
        delete [] left_disparities;
        delete [] right_disparities;
    }
    delete camera_calibration;
    if (background_image != NULL) cvReleaseImage(&background_image);
    if (original_left_image != NULL) cvReleaseImage(&original_left_image);
    //if (grid!=NULL) delete grid;

    if (virtual_camera_depth!=NULL) {
        delete [] virtual_camera_depth;
        cvReleaseMat(&virtual_camera_rotation_matrix);
        cvReleaseMat(&virtual_camera_translation);
        cvReleaseMat(&virtual_camera_rotation_vector);
        cvReleaseMat(&virtual_camera_points);
        cvReleaseMat(&virtual_camera_image_points);
    }
    if (obstacle_map!=NULL) delete [] obstacle_map;

    return 0;
}



