/**
 * ATENCAO: NAO COPIAR E COLAR ESTE ARQUIVO EM OUTROS MODULOS DA SOLUCAO DO CARMEN
 * PARA UTILIZAR AS FUNCIONALIDADES DESSA BIBLIOTECA, EH PRECISO:
 * 1 - Incluir a diretiva no código do módulo
 *     #include <carmen/stereo_util.h>
 * 2 - Adicionar no Makefile na variável LFLAGS a biblioteca 'stereo_interface'
 *     LFLAGS += -lstereo_interface
 */
#include <carmen/carmen.h>
#include <carmen/global.h>
#include <opencv/cv.h>
#include "stereo_util.h"

//http://www.ptgrey.com/products/bumblebee2/bumblebee2_xb3_datasheet.pdf
const int BB2_MAX_PIXEL_WIDTH = 1032;
const int BB2_MAX_PIXEL_HEIGHT = 776;
const float BB2_PIXEL_SIZE = 0.00000465f;//pixel size (in meters)

const int XB3_MAX_PIXEL_WIDTH = 1280;
const int XB3_MAX_PIXEL_HEIGHT = 960;
const float XB3_PIXEL_SIZE = 0.00000375f;//pixel size (in meters)


stereo_util get_stereo_instance(int camera, int width, int height)
{
  stereo_util instance;

  char *width_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(width_key);

  char *height_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(height_key);

  char *fx_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(fx_key);

  char *fy_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(fy_key);

  char *xc_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(xc_key);

  char *yc_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(yc_key);

  char *baseline_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(baseline_key);

  char *camera_model_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(camera_model_key);

  char *stereo_stride_x_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(stereo_stride_x_key);

  char *stereo_stride_y_key = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(stereo_stride_y_key);

  sprintf(width_key, "basic%d_width", camera);
  sprintf(height_key, "basic%d_height", camera);
  sprintf(fx_key, "basic%d_fx", camera);
  sprintf(fy_key, "basic%d_fy", camera);
  sprintf(xc_key, "basic%d_cu", camera);
  sprintf(yc_key, "basic%d_cv", camera);
  sprintf(baseline_key, "basic%d_baseline", camera);
  sprintf(camera_model_key, "basic%d_model", camera);
  sprintf(stereo_stride_x_key, "basic%d_stereo_stride_x", camera);
  sprintf(stereo_stride_y_key, "basic%d_stereo_stride_y", camera);

  double xc_percent, yc_percent, fx_percent, fy_percent, stereo_stride_x = 1.0, stereo_stride_y = 1.0;
  char *camera_model_value = (char*)malloc(128 * sizeof(char));
  carmen_test_alloc(camera_model_value);
  char **argv = (char**)malloc(sizeof(char*));
  carmen_test_alloc(argv);

  carmen_param_t mandatory_param_list[] = {
      {(char*)"bumblebee", (char*)width_key, CARMEN_PARAM_INT, &instance.width, 0, NULL},
      {(char*)"bumblebee", (char*)height_key, CARMEN_PARAM_INT, &instance.height, 0, NULL},
      {(char*)"bumblebee", (char*)fx_key, CARMEN_PARAM_DOUBLE, &fx_percent, 0, NULL},
      {(char*)"bumblebee", (char*)fy_key, CARMEN_PARAM_DOUBLE, &fy_percent, 0, NULL},
      {(char*)"bumblebee", (char*)xc_key, CARMEN_PARAM_DOUBLE, &xc_percent, 0, NULL},
      {(char*)"bumblebee", (char*)yc_key, CARMEN_PARAM_DOUBLE, &yc_percent, 0, NULL},
      {(char*)"bumblebee", (char*)baseline_key, CARMEN_PARAM_DOUBLE, &instance.baseline, 0, NULL},
      {(char*)"bumblebee", (char*)camera_model_key, CARMEN_PARAM_STRING, &camera_model_value, 0, NULL},
  };

  carmen_param_t optional_param_list[] = {
	  {(char*)"bumblebee", (char*)stereo_stride_x_key, CARMEN_PARAM_DOUBLE, &stereo_stride_x, 0, NULL},
	  {(char*)"bumblebee", (char*)stereo_stride_y_key, CARMEN_PARAM_DOUBLE, &stereo_stride_y, 0, NULL},
  };

  carmen_param_allow_unfound_variables(1);
  carmen_param_install_params(1, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
  carmen_param_allow_unfound_variables(0);
  carmen_param_install_params(1, argv, mandatory_param_list, sizeof(mandatory_param_list) / sizeof(mandatory_param_list[0]));


  free(fx_key);
  free(fy_key);
  free(xc_key);
  free(yc_key);
  free(baseline_key);
  free(camera_model_key);
  free(stereo_stride_x_key);
  free(stereo_stride_y_key);

  free(argv);

  if ( (width > 0) && (height > 0) )
  {
	  instance.width = width;
	  instance.height = height;
  }
  instance.fx = fx_percent * instance.width;
  instance.fy = fy_percent * instance.height;
  instance.xc = xc_percent * instance.width;
  instance.yc = yc_percent * instance.height;

  // http://www.ptgrey.com/support/kb/index.asp?a=4&q=85
  instance.vfov = 2 * atan(height / (2 * instance.fy));
  instance.hfov = 2 * atan(width / (2 * instance.fx));

  instance.stereo_stride_x = stereo_stride_x;
  instance.stereo_stride_y = stereo_stride_y;

  return instance;
}


//http://www.ptgrey.com/support/kb/data/modifiedstereoaccuracy.xls
double get_accuracy_in_X(stereo_util instance, int disparity)
{
  double rms_pixel_accuracy = 0.2;
  double accuracy = (instance.fx * instance.baseline * rms_pixel_accuracy) / (disparity * disparity);

  return accuracy;
}


// http://phototour.cs.washington.edu/focal.html
double get_focal_lenght_in_meters(stereo_util instance)
{
  double ccd_width = 0.0;
  if (instance.camera_model == BB2_INDEX)
    ccd_width = BB2_MAX_PIXEL_WIDTH * BB2_PIXEL_SIZE;
  else if (instance.camera_model == XB3_INDEX)
    ccd_width = XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;

  double f_meters = instance.fx * ccd_width / instance.width;

  return f_meters;
}


/**
 * Compute the world coordinates of the image point P, given the camera height and pitch.
 * Assume the given image point P.y coordinate is on the plane given by the camera_height, P.y = camera_height
 * Return the computed world coordinate
 */
carmen_vector_3D_t camera_to_world(carmen_position_t right_point, double camera_height, double camera_pitch, stereo_util instance)
{
  carmen_vector_3D_t p3D;
  double cosseno = cos(camera_pitch), seno = sin(camera_pitch);
  float u_0 = instance.xc, v_0 = instance.yc;

  p3D.z = -camera_height;
  p3D.x = camera_height * (instance.fx * cosseno + (v_0 - right_point.y) * seno);
  p3D.x /= (instance.fx * seno + (right_point.y - v_0) * cosseno);
  p3D.y = (right_point.x - u_0) * (p3D.x * cosseno + camera_height * seno) / instance.fx;

  return p3D;
}


void reproject_to_3D(float *disparityMap, carmen_vector_3D_t *image3D, double theta, stereo_util instance)
{
  float disparity;
  float MIN_DISPARITY = 0.1f;

  int x, y;
  for(y = 0; y < instance.height; y++)
  {
    for(x = 0; x < instance.width; x++)
    {
      disparity = disparityMap[y * instance.width + x];
      disparity = fmax(disparity, MIN_DISPARITY);

      carmen_position_t P_r;
      P_r.x = x;
      P_r.y = y;

      carmen_vector_3D_p P_world_raw = reproject_single_point_to_3D(&instance, P_r, disparity);
      carmen_vector_3D_p P_world = transform_camera_to_world_frame(P_world_raw, theta);

      image3D[y * instance.width + x].x = P_world->x;
      image3D[y * instance.width + x].y = P_world->y;
      image3D[y * instance.width + x].z = P_world->z;

      free(P_world_raw);
      free(P_world);
    }
  }
}

void get_depth_map_from_disparity_map(float *disparityMap, unsigned short* depthMap, stereo_util instance, double MAX_DEPTH)
{
  float disparity;
  float MIN_DISPARITY = 0.1f;
  int x, y;

  for(y = 0; y < instance.height; y++)
  {
    for(x = 0; x < instance.width; x++)
    {
      disparity = disparityMap[y * instance.width + x];
      disparity = fmax(disparity, MIN_DISPARITY);

      carmen_position_t P_r;
      P_r.x = x;
      P_r.y = y;

      carmen_vector_3D_p P_world = reproject_single_point_to_3D(&instance, P_r, disparity);

      depthMap[y * instance.width + x] = (unsigned short)(fmin(P_world->x, MAX_DEPTH) * 1000.0);

      free(P_world);
    }
  }
}

void get_pointcloud_from_disparity_map(float *disparityMap, carmen_vector_3D_t* pointcloud, stereo_util instance, double *position, double *rotation)
{
  float disparity;
  float MIN_DISPARITY = 0.1f;
  int x, y;
  double vx, vy, vz;

  for(y = 0; y < instance.height; y++)
  {
    for(x = 0; x < instance.width; x++)
    {
      disparity = disparityMap[y * instance.width + x];
      disparity = fmax(disparity, MIN_DISPARITY);

      carmen_position_t P_r;
      P_r.x = x;
      P_r.y = y;

      carmen_vector_3D_p P_world = reproject_single_point_to_3D(&instance, P_r, disparity);

      vx = P_world->x;
      vy = P_world->y;
      vz = P_world->z;

      if(position != NULL && rotation != NULL)
      {
      	vx = (float)((rotation[0] * vx + rotation[1] * vy + rotation[2] * vz) + position[0]);
				vy = (float)((rotation[3] * vx + rotation[4] * vy + rotation[5] * vz) + position[1]);
				vz = (float)((rotation[6] * vx + rotation[7] * vy + rotation[8] * vz) + position[2]);
      }

    	pointcloud[y * instance.width + x].x = vx;
    	pointcloud[y * instance.width + x].y = vy;
    	pointcloud[y * instance.width + x].z = vz;

      free(P_world);
    }
  }
}


void getDepthMap(carmen_vector_3D_t *image3D, float *depthMap, stereo_util instance)
{
  double x, z, depth;
  double MAX_DEPTH = 8.0;

  int i, j;
  for(i = 0; i < instance.height; i++)
  {
    for(j = 0; j < instance.width; j++)
    {
      x = image3D[i * instance.width + j].x;
      z = image3D[i * instance.width + j].z;
      depth = sqrt(x * x + z * z);
      depthMap[i * instance.width + j] = fmin(depth, MAX_DEPTH);
    }
  }
}

CvMat *
get_camera_matrix(stereo_util instance)
{
	CvMat *matrix = cvCreateMat(3, 3, CV_32F);
	cvSetReal2D(matrix, 0, 1, 0.0);
	cvSetReal2D(matrix, 1, 0, 0.0);
	cvSetReal2D(matrix, 2, 0, 0.0);
	cvSetReal2D(matrix, 2, 1, 0.0);
	cvSetReal2D(matrix, 0, 0, instance.fx);
	cvSetReal2D(matrix, 1, 1, instance.fy);
	cvSetReal2D(matrix, 0, 2, instance.xc);
	cvSetReal2D(matrix, 1, 2, instance.yc);
	cvSetReal2D(matrix, 2, 2, 1.0);
	return matrix;
}

carmen_vector_3D_p
reproject_single_point_to_3D(const stereo_util *camera, carmen_position_t right_point, double disparity)
{
	carmen_vector_3D_p p3D = NULL;

	if (disparity > 0)
	{
		double xr = right_point.x - camera->xc;
		double yr = right_point.y - camera->yc;
		double xl = xr + disparity;

		double fx_fy = camera->fx / camera->fy;

		// transform 2D image point to 3D Carmen coordinate frame
		double X = -(camera->baseline * camera->fx) / (xr - xl);
		double Y = (camera->baseline * (xr + xl)) / (2.0 * (xr - xl));
		double Z = -fx_fy * (camera->baseline * yr) / (xr - xl);

		if (!(isnan(X) || isnan(Y) || isnan(Z) ||
				isinf(X) ||	isinf(Y) ||	isinf(Z)))
		{
			p3D = (carmen_vector_3D_p)malloc(sizeof(carmen_vector_3D_t));
			carmen_test_alloc(p3D);

			p3D->x = X;
			p3D->y = Y;
			p3D->z = Z;
		}
	}
	return p3D;
}

carmen_vector_3D_p
reproject_single_point_to_3D_with_depth(stereo_util instance, carmen_position_t right_point, unsigned short depth)
{
	carmen_vector_3D_p p3D = NULL;
	double z, vx, vy, vz;
	double dfx_inv, dfy_inv;

	z = depth / 1000.0f; // load and convert: mm -> meters
	dfx_inv = 1.0 / instance.fx;
	dfy_inv = 1.0 / instance.fy;

	p3D = (carmen_vector_3D_p)malloc(sizeof(carmen_vector_3D_t));
	carmen_test_alloc(p3D);

	p3D->x = 0.0;
	p3D->y = 0.0;
	p3D->z = 0.0;

	vx = z * (right_point.x - instance.xc) * dfx_inv;
	vy = z * (right_point.y - instance.yc) * dfy_inv;
	vz = z;

	if (!(isnan(vx) || isnan(vy) || isnan(vz) || isinf(vx) || isinf(vy) || isinf(vz)))
	{
		p3D->x = vx;
		p3D->y = vy;
		p3D->z = vz;
	}

	return p3D;
}


carmen_vector_3D_p
transform_camera_to_world_frame(const carmen_vector_3D_p camera_point, double theta)
{
	carmen_vector_3D_p p3D = NULL;
	if (camera_point)
	{
		//http://en.wikipedia.org/wiki/Rotation_matrix
		p3D = (carmen_vector_3D_p)malloc(sizeof(carmen_vector_3D_t));
		carmen_test_alloc(p3D);

		p3D->x = camera_point->x * cos(theta) - camera_point->y * sin(theta);
		p3D->y = camera_point->x * sin(theta) + camera_point->y * cos(theta);
		p3D->z = camera_point->z;
	}
	return p3D;
}


carmen_vector_3D_p
reproject_single_point_to_3D_in_left_camera_frame(carmen_position_t left_camera_projection, carmen_position_t right_camera_projection, double baseline, double focal_length)
{
	double disparity = left_camera_projection.x - right_camera_projection.x;

	if (disparity < 0)
		exit(printf("OPS! Disparity less than zero!\n"));

	carmen_vector_3D_p point_3D = (carmen_vector_3D_p) calloc (1, sizeof(carmen_vector_3D_t));
	carmen_test_alloc(point_3D);

	point_3D->x = (left_camera_projection.x * baseline) / disparity;
	point_3D->y = (left_camera_projection.y * baseline) / disparity;
	point_3D->z = (focal_length * baseline) / disparity;

	return point_3D;
}


carmen_vector_3D_p
reproject_single_point_to_3D_in_left_camera_frame_with_bias_reduction(carmen_position_t left_camera_projection, carmen_position_t right_camera_projection, double baseline, double focal_length, double bias_std_deviation)
{
	// a funcao abaixo usa o codigo do paper para fazer a projecao 3d
	carmen_vector_3D_p biased_point_3D = reproject_single_point_to_3D_in_left_camera_frame(left_camera_projection, right_camera_projection, baseline, focal_length);

	// a funcao abaixo usa o codigo do carmen para fazer a projecao 3d
	// stereo_util camera = get_stereo_instance(6, 1280, 960);
	// carmen_vector_3D_p biased_point_3D = reproject_single_point_to_3D(&camera, right_camera_projection, left_camera_projection.x - right_camera_projection.x);

	if (biased_point_3D != NULL)
	{
		carmen_vector_3D_p point_3D = (carmen_vector_3D_p) calloc (1, sizeof(carmen_vector_3D_t));
		carmen_test_alloc(point_3D);

		// x = x' - (b * v * (xl + xr)) / (xl - xr)^3
		point_3D->x = biased_point_3D->x -
			(baseline * bias_std_deviation * ((left_camera_projection.x + right_camera_projection.x) /
				pow(left_camera_projection.x - right_camera_projection.x, 3)));

		// y = y' - (2 * v * b * yl) / (xl - xr)^3
		point_3D->y = biased_point_3D->y -
			((2 * bias_std_deviation * baseline * left_camera_projection.y) /
					pow(left_camera_projection.x - right_camera_projection.x ,3));

		// z = z' - (2 * v * b * f) / (xl - xr)^3
		point_3D->z = biased_point_3D->z -
			((2 * bias_std_deviation * baseline * focal_length) /
					pow(left_camera_projection.x - right_camera_projection.x ,3));

		return point_3D;
	}
	else
		return NULL;
}

