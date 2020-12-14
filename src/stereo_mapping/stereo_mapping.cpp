/* OpenCV Includes */
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/ml.h>
#include <opencv2/video/tracking.hpp>

/* Carmen includes */
#include <carmen/carmen.h>
#include <carmen/global.h>

/* Prob Models includes */
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_transforms.h>

#include <carmen/time_profile.h>

/* Stereo includes */
#include <carmen/stereo_interface.h>
#include <carmen/stereo_util.h>

/* V-Disparity includes */
#include <carmen/v_disparity.h>

/* Road Finding includes */
#include <carmen/ml_road_finding.h>

/* Xsens includes */
#include <carmen/xsens_interface.h>

/* Fused Odometry includes */
#include <carmen/fused_odometry_interface.h>

/* Visual Odometry includes */
#include <carmen/visual_odometry_interface.h>

/* My own includes */
//#include "stereo_mapping_messages.h"
#include "stereo_mapping_interface.h"
#include "stereo_mapping_map.h"
#include "stereo_mapping_kalman_filter.h"

/* Stereo Parameters */
static int stereo_height;
static int stereo_width;
static int stereo_disparity;

/* V-Disparity parameters */
static double v_disparity_slope_threshould;

/* Road Finding parameters */
static int number_of_gaussians;
static ml_road_finding road_finding_instance;

/* Map Parameters */
static ProbabilisticMap probabilistic_map;
static ProbabilisticMapParams map_params;

/* The stereo map */
static IplImage *stereo_map = NULL;
static int *perceptual_field_mask;
static CvMemStorage *memory_storage;

/* Camera Information */
static stereo_util stereo_util_instance;
static v_disparity v_disparity_instance;

static CvScalar *samples;
static CvPoint *points;
static IplImage *right_image = NULL;
static IplImage *v_disparity_map = NULL;
static IplImage *road_profile_image = NULL;
unsigned short int *v_disparity_data;

/* Camera State */
static kalman_filter_params pitch_state, height_state;
static double horizon_line;
static cv::KalmanFilter camera_ekf;

/* Car parameters */
static double sensor_board_height, camera_relative_height, wheel_radius;

/* Stereo Mapping Message */
static carmen_stereo_mapping_message msg_stereo_mapping;

/* Camera Index */
static int camera;

/* View Mode */
static char view_mode = 'R';

/* Show Reverse of Inverse Perspective Mapping and Road Mapping */
static char show_reverse = 'N';

static void publish_stereo_mapping()
{
  IPC_RETURN_TYPE err = IPC_OK;
  char *msg_name = carmen_stereo_mapping_get_messagename(camera);

  err = IPC_publishData(msg_name, &(msg_stereo_mapping));

  carmen_test_ipc_exit(err, "Could not publish", msg_name);
}

static void
shutdown_stereo_mapping(int x)
{
  if (x == SIGINT)
  {
    carmen_verbose("Disconnecting Stereo Mapping Service.\n");
    exit(0);
  }
}


static carmen_6d_point last_state_from_sensor, new_state_from_sensor;
static double new_state_timestamp, last_state_timestamp;

static void
fused_odometry_message_handler(carmen_fused_odometry_message *message)
{
  // load position and orientation from message
  new_state_from_sensor.x = message->pose.position.x;
  new_state_from_sensor.y = message->pose.position.y;
  new_state_from_sensor.z = message->pose.position.z;
  new_state_from_sensor.roll = message->pose.orientation.roll;
  new_state_from_sensor.pitch = message->pose.orientation.pitch;
  new_state_from_sensor.yaw = message->pose.orientation.yaw;
  new_state_timestamp = message->timestamp;
}

static void
visual_odometry_message_handler(carmen_visual_odometry_pose6d_message *message)
{
  // load position and orientation from message
  new_state_from_sensor.x = message->pose_6d.x;
  new_state_from_sensor.y = message->pose_6d.y;
  new_state_from_sensor.z = message->pose_6d.z;
  new_state_from_sensor.roll = message->pose_6d.roll;
  new_state_from_sensor.pitch = message->pose_6d.pitch;
  new_state_from_sensor.yaw = message->pose_6d.yaw;
}


static void
copy_disparity_map_to_image(float *disparity_data, IplImage *disparity_map)
{
  for (int i = 0; i < stereo_height; i++)
  {
    for (int j = 0; j < stereo_width; j++)
    {
      float disparity_value = disparity_data[i * stereo_width + j];
      unsigned char scaled_disparity_value = (unsigned char) (255.0 * ((double) disparity_value) / ((double) stereo_disparity));
      scaled_disparity_value = (unsigned char) disparity_value;
      disparity_map->imageData[i * stereo_width + j] = scaled_disparity_value;
    }
  }
}


static void
alloc_image(IplImage **image, CvSize size, int depth, int n_channels)
{
  if (*image == NULL)
    *image = cvCreateImage(size, depth, n_channels);
}

static void
try_save_state(int n_road_lines, carmen_simple_stereo_disparity_message *message)
{
  static bool save_state = false;
  if (!save_state)
    return;

  static int n_messages = 0;
  static FILE *timestamps_file;

  if (n_messages >= 1000)
  {
    fclose(timestamps_file);
    return;
  }

  if (timestamps_file == NULL)
    timestamps_file = fopen("state/timestamps.txt", "w");

  fprintf(timestamps_file, "%d\t%f\n", n_messages, message->timestamp);

  // Init state
  static IplImage *st_right_image, *st_v_disparity_map, *st_road_profile_image, *st_disparity_map, *st_stereo_map, *st_probabilistic_map_image;
  static CvScalar *st_samples;
  static CvPoint *st_points;
  static char *st_filename;

  alloc_image(&st_right_image, cvGetSize(right_image), right_image->depth, right_image->nChannels);
  alloc_image(&st_v_disparity_map, cvGetSize(v_disparity_map), v_disparity_map->depth, v_disparity_map->nChannels);
  alloc_image(&st_road_profile_image, cvGetSize(road_profile_image), road_profile_image->depth, road_profile_image->nChannels);
  alloc_image(&st_disparity_map, cvSize(stereo_width, stereo_height), IPL_DEPTH_8U, 1);
  alloc_image(&st_stereo_map, cvGetSize(stereo_map), stereo_map->depth, stereo_map->nChannels);
  alloc_image(&st_probabilistic_map_image, cvGetSize(stereo_map), stereo_map->depth, 1);

  if (st_samples == NULL)
    st_samples = (CvScalar*) malloc(stereo_width * stereo_height * sizeof(CvScalar));

  if (st_points == NULL)
    st_points = (CvPoint*) malloc(stereo_width * stereo_height * sizeof(CvPoint));

  if (st_filename == NULL)
    st_filename = (char*)malloc(1024 * sizeof(char));

  // Copy data to state
  cvCopy(right_image, st_right_image);
  cvCopy(road_profile_image, st_road_profile_image);
  cvCopy(v_disparity_map, st_v_disparity_map);
  copy_disparity_map_to_image(message->disparity, st_disparity_map);
  copy_probabilistic_map_to_image_buffer(map_params, &probabilistic_map, (unsigned char*)(st_probabilistic_map_image->imageData), 1);

  // Build the stereo map
  inverse_perspective_mapping(map_params, st_stereo_map, st_right_image, height_state.value, pitch_state.value, horizon_line, perceptual_field_mask, stereo_util_instance);

  // Maximize the blue channel of the "safe window" in the projected map
  if (n_road_lines)
  {
    int n_samples = get_road_pixels_list(st_right_image, st_points, st_samples, st_road_profile_image, message->disparity, 1, v_disparity_instance);

    for (int i = 0; i < n_samples; i++)
    {
      carmen_position_t right_point;
      right_point.x = st_points[i].x;
      right_point.y = st_points[i].y;

      carmen_vector_3D_t p3D = camera_to_world(right_point, height_state.value, pitch_state.value, stereo_util_instance);

      int x_map = map_grid_x(map_params, p3D.x);
      int y_map = map_grid_y(map_params, p3D.y);
      st_points[i] = cvPoint(x_map, y_map);

      if (is_map_grid_cell_valid(map_params, x_map, y_map))
      {
        st_samples[i] = cvGet2D(st_stereo_map, y_map, x_map);
        CvScalar pixel_value = cvGet2D(st_stereo_map, y_map, x_map);
        pixel_value.val[2] = 255;//blue channel
        cvSet2D(st_stereo_map, y_map, x_map, pixel_value);
      }
    }
  }

  sprintf(st_filename, "state/%03d_road_profile_%f.bmp", n_messages, message->timestamp);
  cvSaveImage(st_filename, st_road_profile_image);

  sprintf(st_filename, "state/%03d_v_disparity_map_%f.bmp", n_messages, message->timestamp);
  cvSaveImage(st_filename, st_v_disparity_map);

  sprintf(st_filename, "state/%03d_disparity_map_%f.bmp", n_messages, message->timestamp);
  cvSaveImage(st_filename, st_disparity_map);

  sprintf(st_filename, "state/%03d_right_image_%f.bmp", n_messages, message->timestamp);
  cvCvtColor(st_right_image, st_right_image, CV_RGB2BGR);
  cvSaveImage(st_filename, st_right_image);

  sprintf(st_filename, "state/%03d_log_odds_%f.bmp", n_messages, message->timestamp);
  cvSaveImage(st_filename, st_probabilistic_map_image);

  sprintf(st_filename, "state/%03d_data_%f.txt", n_messages, message->timestamp);
  FILE *f = fopen(st_filename, "w");
  fprintf(f, "%f\t%f\n", pitch_state.value, height_state.value);
  for (int i = 0; i < message->disparity_size; i++)
    fprintf(f, "%f\t", message->disparity[i]);
  fclose(f);

  n_messages++;
}


static void
update_camera_state(double camera_pitch, double camera_height)
{
  //time since last frame
  double dt = new_state_timestamp - last_state_timestamp;
  if(dt > 1000) dt = 1; //fix for the first time it runs

  // update state with the received state
  last_state_from_sensor.x = new_state_from_sensor.x;
  last_state_from_sensor.y = new_state_from_sensor.y;
  last_state_from_sensor.z = new_state_from_sensor.z;
  last_state_from_sensor.roll = new_state_from_sensor.roll;
  last_state_from_sensor.pitch = new_state_from_sensor.pitch;
  last_state_from_sensor.yaw = new_state_from_sensor.yaw;
  last_state_timestamp = new_state_timestamp;

  // update camera state
  double measurement[2] = { camera_pitch, camera_height };
  kalman_filter_params *state[2] = { &pitch_state, &height_state };

  //update transition matrix
  camera_ekf.transitionMatrix.at<float>(0,2) = dt;
  camera_ekf.transitionMatrix.at<float>(1,3) = dt;

  //compute kalman filter
  kalman_update_state(&camera_ekf, state, measurement);
}


static CvRect
get_safe_window(IplImage *src, float *disparity_map, int *mask)
{
  int n_safe_window_samples = get_road_pixels_list(src, points, samples, road_profile_image, disparity_map, true, v_disparity_instance);

  CvSeqWriter writer;
  cvStartWriteSeq(CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), memory_storage, &writer);

  for (int i = 0; i < n_safe_window_samples; i++)
  {
    if (points[i].y < horizon_line)
      continue;

    carmen_position_t right_point;
    right_point.x = points[i].x;
    right_point.y = points[i].y;

    carmen_vector_3D_t p3D = camera_to_world(right_point, height_state.value, pitch_state.value, stereo_util_instance);

    int x_map = map_grid_x(map_params, p3D.x);
    int y_map = map_grid_y(map_params, p3D.y);

    if (!is_map_grid_cell_valid(map_params, x_map, y_map) || mask[y_map * map_params.grid_sx + x_map] <= 0)
      continue;

    points[i] = cvPoint(x_map, y_map);
    CV_WRITE_SEQ_ELEM(points[i], writer);
  }

  CvSeq *seq_pt = cvEndWriteSeq(&writer);
  CvRect rect_pt = cvBoundingRect(seq_pt, 0);
  cvClearMemStorage(memory_storage);

  return rect_pt;
}


static void
extract_zt_from_image_measurement(float *zt, int width, int height, IplImage *measurement, int *perceptual_field)
{
  memset(zt, 0, width * height * sizeof(float)); //clear all

  //copy from image to measurement vector
  for (int y = 0 ; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int _y = measurement->height - y - 1;
      _y = y;
      zt[y * width + x] = perceptual_field[y * width + x] <= 0 ? -1.0 : (float)cvGet2D(measurement, _y, x).val[0];
    }
  }
}


static void
disparity_map_handler(carmen_simple_stereo_disparity_message *message)
{
  struct timeval begin;
  gettimeofday(&begin, NULL);

  // get the last processed state xt_1 and the new received state xt
  carmen_point_t xt, xt_1;
  xt.x = new_state_from_sensor.x;
  xt.y = new_state_from_sensor.y;
  xt_1.x = last_state_from_sensor.x;
  xt_1.y = last_state_from_sensor.y;

  // fill the message
  msg_stereo_mapping.timestamp = message->timestamp;
  msg_stereo_mapping.map_size = stereo_map->imageSize;
  msg_stereo_mapping.measurement_size = map_params.grid_sx * map_params.grid_sy;

  // copy the reference image
  memcpy(right_image->imageData, message->reference_image, message->reference_image_size * sizeof(unsigned char));

  // v-disparity analysis
  double camera_height, camera_pitch;
  compute_v_disparity_info(v_disparity_data, v_disparity_map, v_disparity_slope_threshould, message->disparity, &camera_height, &camera_pitch, &horizon_line, v_disparity_instance);
  show_time((char*)"V-disparity analysis", begin);

  update_camera_state(camera_pitch, camera_height);
  show_time((char*)"Camera State Estimation", begin);

  if (view_mode == VIEW_MODE_BIRDS_EYE)
  {
	  //inverse_perspective_mapping(map_params, stereo_map, right_image, 2.0, carmen_degrees_to_radians(2), horizon_line, perceptual_field_mask, stereo_util_instance);

    inverse_perspective_mapping(map_params, stereo_map, right_image, height_state.value, pitch_state.value, horizon_line, perceptual_field_mask, stereo_util_instance);
  }
  else if (view_mode == VIEW_MODE_BIRDS_EYE_OPENCV)
  {
	  opencv_birds_eye_remap(map_params, right_image, stereo_map, height_state.value, pitch_state.value, stereo_util_instance);
  }
  else if (view_mode == VIEW_MODE_ROAD_FINDING)
  {
    // 1. Transforms original image to a inverse perspective image (Bird's Eye View)
    inverse_perspective_mapping(map_params, stereo_map, right_image, height_state.value, pitch_state.value, horizon_line, perceptual_field_mask, stereo_util_instance);
    show_time((char*)"Inverse Perspective Mapping", begin);

    // 2. Determines a safe window on the Inverse Perspective Image and use it to mixture of Gaussians learning
    int n_road_lines = draw_road_profile_lines_in_v_disparity(road_profile_image, v_disparity_map, v_disparity_slope_threshould, v_disparity_instance);
    if (n_road_lines)
    {
      CvRect safe_window = get_safe_window(right_image, message->disparity, perceptual_field_mask);
      rgb_gaussian *gaussian = get_gaussian_at_rect(stereo_map, safe_window, NULL);
      add_gaussian(&road_finding_instance, gaussian);
    }
    show_time((char*)"Safe Window and Training", begin);

    // 3. Recognizes the traversable area on the Inverse Perspective Image
    fill_image_based_on_gaussians(&road_finding_instance, stereo_map, stereo_map, perceptual_field_mask);
    show_time((char*)"Image Classification", begin);

    // 4. Filters the recognition result
    cvThreshold(stereo_map, stereo_map, 4, 255, CV_THRESH_BINARY);
    show_time((char*)"Image Thresholding", begin);

    // 5. Extracts and vectorizes instantaneous observation from image
    extract_zt_from_image_measurement(msg_stereo_mapping.measurement, map_params.grid_sx, map_params.grid_sy, stereo_map, perceptual_field_mask);
    show_time((char*)"Stereo Mapping", begin);

    update_probabilistic_map(probabilistic_map, map_params, xt, xt_1, msg_stereo_mapping.measurement);

    if (show_reverse == 'Y')
    {
      static IplImage *reverse_image;
      alloc_image(&reverse_image, cvGetSize(right_image), right_image->depth, right_image->nChannels);

      reverse_inverse_perspective_mapping(map_params, reverse_image, stereo_map, height_state.value, pitch_state.value, stereo_util_instance);

      cvNamedWindow("Reverse Road Finding");
      cvShowImage("Reverse Road Finding", reverse_image);
      cvWaitKey(5);
    }

    // Testing code: Try to save state (files on disk), Update Probabilistic Map, Copy probabilistic to image
    static bool testing = false;
    if (testing)
    {
      try_save_state(n_road_lines, message);
      copy_probabilistic_map_to_image_buffer(map_params, &probabilistic_map, (unsigned char*)(stereo_map->imageData), stereo_map->nChannels);
    }
  }

  msg_stereo_mapping.stereo_mapping_data = (unsigned char*) stereo_map->imageData;
  publish_stereo_mapping();
}

static int
read_parameters(int argc, char **argv)
{
  int num_items;
  char stereo_string[256];
  char sensor_board_string[256];

  if (argc < 2)
    carmen_die("%s: Wrong number of parameters. stereo_mapping requires a parameter.\nUsage:\n %s <camera_number>\n", argv[0], argv[0]);

  camera = atoi(argv[1]);
  sprintf(stereo_string, "%s%d", "stereo", camera);

  sprintf(sensor_board_string, "%s%d", "sensor_board_", 1);

  if (argc == 3)
    view_mode = argv[2][0];

  if (argc == 4)
    show_reverse = argv[3][0];

  carmen_param_t param_list[] = {
      {stereo_string, (char*) "width", CARMEN_PARAM_INT, &stereo_width, 0, NULL},
      {stereo_string, (char*) "height", CARMEN_PARAM_INT, &stereo_height, 0, NULL},
      {stereo_string, (char*) "max_disparity", CARMEN_PARAM_INT, &stereo_disparity, 0, NULL},
      {(char*) "road_finding", (char*) "number_of_gaussians", CARMEN_PARAM_INT, &number_of_gaussians, 5, NULL},
      {(char*) "v_disparity", (char*) "slope_threshould", CARMEN_PARAM_DOUBLE, &v_disparity_slope_threshould, 0, NULL},
      {(char*) "stereo_map", (char*) "width", CARMEN_PARAM_DOUBLE, &map_params.width, 0, NULL},
      {(char*) "stereo_map", (char*) "height", CARMEN_PARAM_DOUBLE, &map_params.height, 0, NULL},
      {(char*) "stereo_map", (char*) "grid_res", CARMEN_PARAM_DOUBLE, &map_params.grid_res, 0, NULL},
      {(char*) "stereo_map", (char*) "locc", CARMEN_PARAM_INT, &map_params.locc, 0, NULL},
      {(char*) "stereo_map", (char*) "lfree", CARMEN_PARAM_INT, &map_params.lfree, 0, NULL},
      {(char*) "stereo_map", (char*) "l0", CARMEN_PARAM_INT, &map_params.l0, 0, NULL},
      {(char*) "stereo_map", (char*) "log_odds_max", CARMEN_PARAM_INT, &map_params.log_odds_max, 0, NULL},
      {(char*) "stereo_map", (char*) "log_odds_min", CARMEN_PARAM_INT, &map_params.log_odds_min, 0, NULL},
      {(char*) "stereo_map", (char*) "log_odds_bias", CARMEN_PARAM_INT, &map_params.log_odds_bias, 0, NULL},
      {(char*) "stereo_map", (char*) "laser_num_beams", CARMEN_PARAM_INT, &map_params.num_ranges, 0, NULL}, // only for consistency in ProbabilisticMapParams inicialization
      {(char*) "camera", (char*) "z", CARMEN_PARAM_DOUBLE, &camera_relative_height, 0, NULL},
      {(char*) "camera", (char*) "pitch", CARMEN_PARAM_DOUBLE, &pitch_state.value, 0, NULL},
      {sensor_board_string, (char*) "z", CARMEN_PARAM_DOUBLE, &sensor_board_height, 0, NULL},
      {(char*) "robot", (char*) "wheel_radius", CARMEN_PARAM_DOUBLE, &wheel_radius, 0, NULL}

  };

  num_items = sizeof(param_list) / sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  return 0;
}

static void
init_stereo_mapping()
{
  /* Init camera height relative to the world*/
  height_state.value = wheel_radius + sensor_board_height + camera_relative_height;

  /* Init the grid parameters */
  map_params.grid_sx = (int) ceil(map_params.width / map_params.grid_res);
  map_params.grid_sy = (int) ceil(map_params.height / map_params.grid_res);
  map_params.grid_size = map_params.width * map_params.height;

  /* Init stereo_map mask */
  perceptual_field_mask = (int*)malloc(map_params.grid_sx * map_params.grid_sy * sizeof(int));

  /* Allocate memory for the map */
  stereo_map = cvCreateImage(cvSize(map_params.grid_sx, map_params.grid_sy), IPL_DEPTH_8U, 3);

  /* Create ml_road_finding */
  init_ml_road_finding(&road_finding_instance, number_of_gaussians, 3 * stereo_width, stereo_height);

  /* Create memory storage */
  memory_storage = cvCreateMemStorage(0);

  /* init our message */
  msg_stereo_mapping.host = carmen_get_host();
  msg_stereo_mapping.map_size = 3 * map_params.grid_sx * map_params.grid_sy;
  msg_stereo_mapping.measurement_size = map_params.grid_sx * map_params.grid_sy;
  if (!msg_stereo_mapping.stereo_mapping_data)
  {
    msg_stereo_mapping.stereo_mapping_data = (unsigned char*) malloc(msg_stereo_mapping.map_size * sizeof(unsigned char));
    carmen_test_alloc(msg_stereo_mapping.stereo_mapping_data);
    msg_stereo_mapping.measurement = (float*)malloc(msg_stereo_mapping.measurement_size * sizeof(float));
  }

  /* Create a stereo_util instance and a v-disparity util instance*/
  stereo_util_instance = get_stereo_instance(camera, stereo_width, stereo_height);
  v_disparity_instance = get_v_disparity_instance(stereo_util_instance, stereo_disparity);

  samples = (CvScalar*) malloc(stereo_width * stereo_height * sizeof(CvScalar));

  points = (CvPoint*) malloc(stereo_width * stereo_height * sizeof(CvPoint));

  right_image = cvCreateImage(cvSize(stereo_width, stereo_height), IPL_DEPTH_8U, 3);

  v_disparity_map = cvCreateImage(cvSize(stereo_disparity, stereo_height), IPL_DEPTH_8U, 1);

  road_profile_image = cvCreateImage(cvGetSize(v_disparity_map), IPL_DEPTH_8U, 3);

  v_disparity_data = alloc_v_disparity_map(v_disparity_instance);

  horizon_line = 240; //192;

  // create a state with two Degrees of Freedom (pitch and height with respect to the ground plane)
  init_kalman_filter_params(&pitch_state, carmen_degrees_to_radians(1.5), 2.0, carmen_degrees_to_radians(3.0));
  init_kalman_filter_params(&height_state, 0.15, 2.0, 0.3);

  // OpenCV Kalman Filter: see http://opencv.willowgarage.com/documentation/motion_analysis_and_object_tracking.html
  // Initialize Kalman filters transition, control and measurement matrixes
  camera_ekf.init(4,2,0, CV_64F);

  double transition_coefficients[] = {
		  	  	  	  	  	  	  	  1.0 , 0.0, 1.0, 0.0,
		  	  	  	  	  	  	  	  0.0 , 1.0, 0.0, 1.0,
									  0.0 , 0.0, 1.0, 0.0,
									  0.0 , 0.0, 0.0 , 1.0
  	  	  	  	  	  	  	  	  	  };
  camera_ekf.transitionMatrix = cv::Mat(4 , 4, CV_64F, transition_coefficients);
  //camera_ekf.transitionMatrix = *(cv::Mat_<double>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

  camera_ekf.statePost.at<double>(0) = 0; //initial pitch
  camera_ekf.statePost.at<double>(1) = 2; //initial height
  camera_ekf.statePost.at<double>(2) = 0; //initial velocity in pitch direction
  camera_ekf.statePost.at<double>(3) = 0; //initial velocity in height d

  cv::setIdentity(camera_ekf.measurementMatrix);
  cv::setIdentity(camera_ekf.processNoiseCov, cv::Scalar::all(1e-3));
  cv::setIdentity(camera_ekf.measurementNoiseCov, cv::Scalar::all(5e-2));
  cv::setIdentity(camera_ekf.errorCovPost, cv::Scalar::all(1e3));
}


int
main(int argc, char **argv)
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_stereo_mapping);

  /* Initialize all the relevant parameters */
  read_parameters(argc, argv);

  /* Allocate my own structures */
  init_stereo_mapping();

  /* Create and initialize a probabilistic map */
  init_probabilistic_map(&map_params, NULL, &probabilistic_map, 1);

  /* Define my own messages */
  carmen_stereo_mapping_define_messages(camera);

  /* Subscribe Stereo Service */
  carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) disparity_map_handler, CARMEN_SUBSCRIBE_LATEST);

  /* Subscribe to Localize Service */
  bool use_fused_odometry = true;
  if (use_fused_odometry)
    carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
  else
    carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) visual_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return 0;
}
