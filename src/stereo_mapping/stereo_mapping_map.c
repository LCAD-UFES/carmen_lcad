/* OpenCV Includes */
#include <opencv/cv.h>
#include <opencv/highgui.h>

/* Carmen includes */
#include <carmen/carmen.h>
#include <carmen/global.h>

/* Prob Models includes */
#include <prob_measurement_model.h>
#include <prob_map.h>

/* Stereo includes */
#include <carmen/stereo_interface.h>
#include <carmen/stereo_util.h>

#include "stereo_mapping_map.h"


// Convert from map coordinates to real-world coordinates
double grid_x_map(ProbabilisticMapParams map_params, int x)
{
  return map_params.grid_res * ((double)x);
}

double grid_y_map(ProbabilisticMapParams map_params, int y)
{
  return map_params.grid_res * ((double)y - (double)map_params.grid_sy / 2.0);
}

// Convert from real-world coordinates to map coordinates
int map_grid_x(ProbabilisticMapParams map_params, double x)
{
  return round(x / map_params.grid_res);
}

int map_grid_y(ProbabilisticMapParams map_params, double y)
{
  return round(y / map_params.grid_res + (double)map_params.grid_sy / 2.0);
}

// Check that map coordinates are valid
int is_map_grid_cell_valid(ProbabilisticMapParams map_params, int x, int y)
{
  return ((x >= 0) && (x < map_params.grid_sx) && (y >= 0) && (y < map_params.grid_sy));
}


void inverse_perspective_mapping(ProbabilisticMapParams map_params, IplImage *dst, IplImage *src, double height, double pitch, int horizon_line, int *perceptual_mask, stereo_util stereo_util_instance)
{
  carmen_vector_3D_t P_w;
  double cosseno = cos(pitch), seno = sin(pitch);
  float u_0 = stereo_util_instance.xc, v_0 = stereo_util_instance.yc;

  double den, u, v;
  int x, y, x_map, y_map;

  #ifdef UBUNTU_20_04
    cvSet(dst, cvScalar(127,127,127), NULL);// clear the map
  #else
    cvSet(dst, CV_RGB(127,127,127), NULL);
  #endif

  // clear the perceptual mask
  memset(perceptual_mask, 0, map_params.grid_sx * map_params.grid_sy * sizeof(int));

  for (P_w.x = map_params.grid_res; P_w.x <= map_params.width; P_w.x += map_params.grid_res)
  {
    den = P_w.x * cosseno + height * seno;

    v = v_0 - stereo_util_instance.fx * (P_w.x * seno - height * cosseno) / den;
    y = round(v);

    if (y < 0 || y >= src->height || y < horizon_line)
      continue;

    for (P_w.y = -0.5 * map_params.height; P_w.y <= 0.5 * map_params.height; P_w.y += map_params.grid_res)
    {
      u = u_0 + stereo_util_instance.fx * P_w.y / den;
      x = round(u);

      if (x < 0 || x >= src->width)
        continue;

      x_map = map_grid_x(map_params, P_w.x);
      y_map = map_grid_y(map_params, P_w.y);

      if (is_map_grid_cell_valid(map_params, x_map, y_map))
      {
        cvSet2D(dst, y_map, x_map, cvGet2D(src, y, x));
        perceptual_mask[y_map * map_params.grid_sx + x_map] = 1;
      }
    }

  }
}

void reverse_inverse_perspective_mapping(ProbabilisticMapParams map_params, IplImage *dst, IplImage *src, double height, double pitch, stereo_util stereo_util_instance)
{
  carmen_vector_3D_t P_w;
  double cosseno = cos(pitch), seno = sin(pitch);
  float u_0 = stereo_util_instance.xc, v_0 = stereo_util_instance.yc;
  int x, y, x_map, y_map;

  #ifdef UBUNTU_20_04
    cvSet(dst, cvScalar(127,127,127), NULL); // clear the map
  #else
    cvSet(dst, CV_RGB(127,127,127), NULL);
  #endif

  P_w.z = -height;

  for (y = 0; y < dst->height; y++)
  {
    P_w.x = -P_w.z * (stereo_util_instance.fx * cosseno + (v_0 - y) * seno) / (stereo_util_instance.fx * seno + (y - v_0) * cosseno);

    for (x = 0; x < dst->width; x++)
    {
      P_w.y = (x - u_0) * (P_w.x * cosseno - P_w.z * seno) / stereo_util_instance.fx;

      x_map = map_grid_x(map_params, P_w.x);
      y_map = map_grid_y(map_params, P_w.y);

      if (is_map_grid_cell_valid(map_params, x_map, y_map))
      {
        CvScalar pixel = cvGet2D(src, y_map, x_map);
        if (pixel.val[0] > 0)//is road?
        {
          #ifdef UBUNTU_20_04
            cvSet2D(dst, y, x, cvScalar(255, 255, 255));
          #else
            cvSet2D(dst, y, x, CV_RGB(255, 255, 255));
          #endif
        }          
        else
        {
          #ifdef UBUNTU_20_04
            cvSet2D(dst, y, x, cvScalar(0, 0, 0));
          #else
            cvSet2D(dst, y, x, CV_RGB(0, 0, 0));
          #endif
        }
      }
    }
  }
}

void camera_to_world_build_map(ProbabilisticMapParams map_params, IplImage *dst, IplImage *src, double camera_height, double camera_pitch, int horizon_line, stereo_util stereo_util_instance)
{
  for (int y = horizon_line; y < src->height; y++)
  {
    for (int x = 0; x < src->width; x++)
    {
      carmen_position_t right_point;
      right_point.x = x;
      right_point.y = y;

      carmen_vector_3D_t p3D = camera_to_world(right_point, camera_height, camera_pitch, stereo_util_instance);

      int x_map = map_grid_x(map_params, p3D.x);
      int y_map = map_grid_y(map_params, p3D.y);

      if (is_map_grid_cell_valid(map_params, x_map, y_map))
        cvSet2D(dst, y_map, x_map, cvGet2D(src, y, x));
    }
  }
}


void opencv_birds_eye_remap(ProbabilisticMapParams map, const IplImage *src_image, IplImage *dst_image, double camera_height, double camera_pitch, stereo_util stereo_util_instance)
{
  // coordinates of 4 quadrangle vertices in the source image.
  CvPoint2D32f src_pts[4] = { cvPoint2D32f(180, 400), cvPoint2D32f(180, 320), cvPoint2D32f(520, 320), cvPoint2D32f(520, 400) };

  // coordinates of the 4 corresponding quadrangle vertices in the destination image.
  CvPoint2D32f dst_pts[4];

  for (int i = 0; i < 4; i++)
  {
    carmen_position_t right_point;
    right_point.x = src_pts[i].x;
    right_point.y = src_pts[i].y;

    carmen_vector_3D_t p3D = camera_to_world(right_point, camera_height, camera_pitch, stereo_util_instance);

    int x_map = map_grid_x(map, p3D.x);
    int y_map = map_grid_y(map, p3D.y);
    dst_pts[i] = cvPoint2D32f(x_map, y_map);
  }

  // FIND THE HOMOGRAPHY
  static CvMat *homography_matrix;
  if (homography_matrix == NULL)
    homography_matrix = cvCreateMat(3, 3, CV_32F);
  cvGetPerspectiveTransform(dst_pts, src_pts, homography_matrix);

  float Z = camera_height;

  CV_MAT_ELEM(*homography_matrix, float, 2, 2) = Z;
  cvWarpPerspective(src_image,
      dst_image,
      homography_matrix,
      CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS,
      cvScalarAll(0));
}

void draw_lateral_offset(ProbabilisticMapParams map_params, IplImage *dst_image)
{
  int x_offset = (int)ceil(2.0 / map_params.grid_res);
  int x_limit = (int)ceil(48.0 / map_params.grid_res);
  int rect_size_x = (int)ceil(3.0 / map_params.grid_res);
  int rect_size_y = (int)ceil(1.0 / map_params.grid_res);

  int n_size_x = (int)(MIN(dst_image->width, x_limit + x_offset) - x_offset) / rect_size_x;
  int n_size_y = (int)dst_image->height / rect_size_y;

  static double **values;
  if (values == NULL)
  {
    values = (double**)malloc(n_size_y * sizeof(double*));
    for (int i = 0; i < n_size_y; i++)
      values[i] = (double*)malloc(n_size_x * sizeof(double));
  }

  for (int j = 0; j < n_size_y; j++)
  {
    //cvLine(dst_image, cvPoint(0, j * rect_size_y), cvPoint(stereo_width - 1, j * rect_size_y), CV_RGB(0,0,255));
    for (int i = 0; i < n_size_x; i++)
    {
      CvRect rect = cvRect(x_offset + i * rect_size_x, j * rect_size_y, rect_size_x, rect_size_y);

      //cvLine(dst_image, cvPoint(x_offset + i * rect_size_x, 0), cvPoint(x_offset + i * rect_size_x, stereo_height - 1), CV_RGB(0,0,255));

      unsigned long sum = 0;
      for (int y = rect.y; y < rect.y + rect.height; y++)
      {
        for (int x = rect.x; x < rect.x + rect.width; x++)
        {
          sum += (int)cvGet2D(dst_image, y, x).val[0];
        }
      }

      values[j][i] = (double) sum;
      //printf("%f\t", values[j][i]);
    }

    //printf("\n");
  }

  for (int j = 0; j < n_size_x; j++)
  {
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < n_size_y; i++)
    {
      if (values[i][j] > 0.0)
      {
        sum += values[i][j];
        n++;
      }
    }

    double mean = (double)sum / n;

    double acum = 0.0;
    for (int i = 0; i < n_size_y; i++)
    {
      if (values[i][j] > 0.0)
      {
        acum = acum + (values[i][j] - mean) * (values[i][j] - mean);
      }
    }

    double variance = (double)acum / n;
    double std_dev = sqrt(variance);

    double alpha = 0.8;
    int start_index = INT_MIN;
    int n_iters = 0, start_cand = -1;
    int n_end_y = 24;
    int n_max_iters = 12;

    while (start_index < 0 || start_index > n_end_y)
    {
      alpha += 0.1;
      double threshould = mean - alpha * std_dev;

      for (int i = n_end_y; i >= 0; i--)
      {
        if (values[i][j] >= threshould)
        {
          if (start_index < i && n_iters == 0)
            start_cand = i;

          if (n_iters >= 3)
          {
            start_index = start_cand;
            break;
          }

          n_iters++;
        }
        else
        {
          n_iters = 0;
          start_index = INT_MIN;
        }
      }

      n_max_iters--;
      if (n_max_iters <= 0)
        break;
    }

    if (start_index >= 0 && start_index < n_end_y)
    {
      CvRect rect_ri = cvRect(x_offset + j * rect_size_x, start_index * rect_size_y, rect_size_x, rect_size_y);
      #ifdef UBUNTU_20_04
        cvRectangle(dst_image, cvPoint(rect_ri.x, rect_ri.y), cvPoint(rect_ri.x + rect_ri.width, rect_ri.y - rect_ri.height), cvScalar(0,255,0), 1, 8, 0);
      #else
        cvRectangle(dst_image, cvPoint(rect_ri.x, rect_ri.y), cvPoint(rect_ri.x + rect_ri.width, rect_ri.y - rect_ri.height), CV_RGB(0,255,0), 1, 8, 0);
      #endif
      

      CvRect rect_ro = cvRect(x_offset + j * rect_size_x, (start_index + 1) * rect_size_y, rect_size_x, rect_size_y);
      #ifdef UBUNTU_20_04
        cvRectangle(dst_image, cvPoint(rect_ro.x, rect_ro.y), cvPoint(rect_ro.x + rect_ro.width, rect_ro.y - rect_ro.height), cvScalar(0,0,255), 1, 8, 0);
      #else
        cvRectangle(dst_image, cvPoint(rect_ro.x, rect_ro.y), cvPoint(rect_ro.x + rect_ro.width, rect_ro.y - rect_ro.height), CV_RGB(255,0,0), 1, 8, 0);
      #endif
    }
  }
}


void update_probabilistic_map(ProbabilisticMap probabilistic_map, ProbabilisticMapParams map_params, carmen_point_t xt, carmen_point_t xt_1, float *zt)
{
  carmen_point_t origini;
  origini.x = origini.y = origini.theta = 0;
  double y_offset = 0;

  double dx = -fabs(xt.x - xt_1.x);
  double dy = xt.y - xt_1.y;
  double d = sqrt(dx * dx + dy * dy);

  if (d > map_params.grid_res)
    translate_map(&probabilistic_map, map_params, dx, dy);
  else
    translate_map(&probabilistic_map, map_params, dx, 0.0);

  update_cells_in_the_camera_perceptual_field(origini, &probabilistic_map, map_params, map_params, y_offset, zt);

  if (d > map_params.grid_res)
    translate_map(&probabilistic_map, map_params, 0.0, -dy);
}

void compare_results(ProbabilisticMapParams map_params, IplImage *ground_truth, IplImage *result_map, int *mask)
{
  int n_road[5] = {0, 0, 0, 0, 0};
  int n_non_road[5] = {0, 0, 0, 0, 0};
  int n_road_match[5] = {0, 0, 0, 0, 0};
  int n_non_road_match[5] = {0, 0, 0, 0, 0};
  int n_road_fail[5] = {0, 0, 0, 0, 0};
  int n_non_road_fail[5] = {0, 0, 0, 0, 0};
  float distances[N_DISTANCES] = {10.0, 20.0, 35.0, 50.0};

  for (int y = 0; y < ground_truth->height; y++)
  {
//    double Yw = grid_y_map(map_params, y);
//    if (Yw >= 2.0 || Yw <= -9.0)
//      continue;

    for (int x = 0; x < ground_truth->width; x++)
    {
      if (mask[y * ground_truth->width + x] <= 0) //map cell is outside perceptual field
        continue;

      double Xw = grid_x_map(map_params, x);

      int index = -1;
      for (int i = 0; i < N_DISTANCES; i++)
      {
        if (Xw <= distances[i])
        {
          index = i;
          break;
        }
      }
      if (index < 0 && Xw > distances[N_DISTANCES - 1])
        index = N_DISTANCES;

      CvScalar pixel_gt = cvGet2D(ground_truth, y, x);
      int is_road_gt = 1;
      for (int n = 0; n < ground_truth->nChannels; n++)
        if (pixel_gt.val[n] > 0.0)
          is_road_gt = 0;

      CvScalar pixel_cl = cvGet2D(result_map, y, x);
      int is_road_cl = 0;
      for (int n = 0; n < ground_truth->nChannels; n++)
        if (pixel_cl.val[n] > 0.0)
          is_road_cl = 1;

      if (is_road_gt)
      {
        n_road[index]++;

        if (is_road_cl)
          n_road_match[index]++;
        else
          n_road_fail[index]++;
      }
      else
      {
        n_non_road[index]++;

        if (is_road_cl)
          n_non_road_fail[index]++;
        else
          n_non_road_match[index]++;
      }
    }
  }

  int n_all_road = 0;
  int n_all_road_match = 0;
  int n_all_road_fail = 0;
  int n_all_non_road = 0;
  int n_all_non_road_match = 0;
  int n_all_non_road_fail = 0;

  for (int i = 0; i < N_DISTANCES; i++)
  {
    n_all_road += n_road[i];
    n_all_road_match += n_road_match[i];
    n_all_road_fail += n_road_fail[i];
    n_all_non_road += n_non_road[i];
    n_all_non_road_match += n_non_road_match[i];
    n_all_non_road_fail += n_non_road_fail[i];

    if (i == 0)
      printf("Distance to %f\n", distances[i]);
    else
      printf("Distance from %f to %f\n", distances[i - 1], distances[i]);

    printf("True Positive (Road classified as Road): %d of %d. TPR = %.3f%%\n", n_road_match[i], n_road[i], (float)100.0 * n_road_match[i] / n_road[i]);
    printf("False Positive (Non Road classified as Road) %d. FPR = %.3f%%\n", n_non_road_fail[i], (float)100.0 * n_non_road_fail[i] / n_non_road[i]);
    printf("True Negative (Non Road classified as Non Road) %d of %d. TNR = %.3f%%\n", n_non_road_match[i], n_non_road[i], (float)100.0 * n_non_road_match[i] / n_non_road[i]);
    printf("False Negative (Road classified as Non Road) %d. FNR = %.3f%%\n", n_road_fail[i], (float) 100.0 * n_road_fail[i] / n_road[i]);
    printf("\n");
  }

  printf("True Positive (Road classified as Road): %d of %d. TPR = %.3f%%\n", n_all_road_match, n_all_road, (float)100.0 * n_all_road_match / n_all_road);
  printf("False Positive (Non Road classified as Road) %d. FPR = %.3f%%\n", n_all_non_road_fail, (float)100.0 * n_all_non_road_fail / n_all_non_road);
  printf("True Negative (Non Road classified as Non Road) %d of %d. TNR = %.3f%%\n", n_all_non_road_match, n_all_non_road, (float)100.0 * n_all_non_road_match / n_all_non_road);
  printf("False Negative (Road classified as Non Road) %d. FNR = %.3f%%\n", n_all_road_fail, (float) 100.0 * n_all_road_fail / n_all_road);
  printf("\n");
}
