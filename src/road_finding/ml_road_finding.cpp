#include "ml_road_finding.h"
#ifndef NO_CUDA
#include "ml_road_finding_GPU.h"
#endif

inline gsl_vector *get_standard_deviation_vector(rgb_gaussian *gaussian)
{
  gsl_vector *std_dev_vector = gsl_vector_alloc(gaussian->mean->size);

  for (unsigned int i = 0; i < gaussian->mean->size; i++)
  {
    double variance = gsl_matrix_get(gaussian->cov, i, i);
    double std_deviation = sqrt(fabs(variance));
    gsl_vector_set(std_dev_vector, i, std_deviation);
  }

  return std_dev_vector;
}

inline void set_gaussian_threshould(rgb_gaussian *gaussian, double std_dev_acceptance)
{
  gsl_vector *tmp_ptr = gsl_vector_alloc(gaussian->mean->size);
  gsl_vector_memcpy(tmp_ptr, gaussian->mean);

  gsl_vector *std_dev = get_standard_deviation_vector(gaussian);
  gsl_vector_scale(std_dev, std_dev_acceptance);

  gsl_vector_add(tmp_ptr, std_dev);

  gaussian->probability_threshould = gsl_get_normal(tmp_ptr, gaussian->mean, gaussian->cov);
  gaussian->probability_mean = gsl_get_normal(gaussian->mean, gaussian->mean, gaussian->cov);
  gaussian->normalizer = 1.0 / gaussian->probability_mean;

  gsl_vector_free(tmp_ptr);
  gsl_vector_free(std_dev);
}

void set_inverse(rgb_gaussian *gaussian)
{
	gsl_matrix *cov_inv = gsl_matrix_alloc(3, 3);

  // pre-compute covariance inverse
  gsl_matrix *cov_cpy = gsl_matrix_alloc(3, 3);
  gsl_matrix_memcpy(cov_cpy, gaussian->cov);
  gsl_permutation *p = gsl_permutation_alloc(3);
  int sign;

  gsl_linalg_LU_decomp(cov_cpy, p, &sign);
  gsl_linalg_LU_invert(cov_cpy, p, cov_inv);

  for (int i = 0; i < 3; i++)
  {
  	for (int j = 0; j < 3; j++)
  	{
  		gaussian->cov_inv[3 * i + j] = gsl_matrix_get(cov_inv, i, j);
  	}
  }

  // release memory
  gsl_matrix_free(cov_inv);
  gsl_matrix_free(cov_cpy);
  gsl_permutation_free(p);
}

static double get_covariance_of_xy2(long *sums, long **sums_of_products, int n_points, int x_index, int y_index)
{
  double sum_of_x = sums[x_index];
  double sum_of_y = sums[y_index];
  double sum_of_xy = sums_of_products[x_index][y_index];

  double cov_of_xy = sum_of_xy - ((sum_of_x * sum_of_y) / n_points);
  cov_of_xy = cov_of_xy / (n_points - 1);

  return cov_of_xy;
}


// calculate mean and covariance of the points location
// formula is based on http://www.wikihow.com/Calculate-Covariance
static void
get_mean_and_covariance_of_points_locations(CvPoint *points, int n_points, double *mean, double *covariance)
{
  // calculate covariance, based on http://www.wikihow.com/Calculate-Covariance
  long *sums = (long*)calloc(2, sizeof(long));
  long **sums_of_products = (long**)malloc(2 * sizeof(long*));
  for (int i = 0; i < 2; i++)
    sums_of_products[i] = (long*)calloc(2, sizeof(long));

  for (int i = 0; i < n_points; i++)
  {
    int x = points[i].x;
    int y = points[i].y;

    // sum the components to compute the mean
    sums[0] += x;
    sums[1] += y;

    // compute the upper matrix to compute the covariances between components.
    sums_of_products[0][0] += (x * x);
    sums_of_products[1][1] += (y * y);
    sums_of_products[0][1] += (x * y);
    sums_of_products[1][0] += (x * y);
  }

  // sets the means
  mean[0] = (double)sums[0] / n_points;
  mean[1] = (double)sums[1] / n_points;

  // sets the covariance matrix
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      covariance[2 * i + j] = get_covariance_of_xy2(sums, sums_of_products, n_points, i, j);
    }
  }

  for (int i = 0; i < 2; i++)
    free(sums_of_products[i]);
  free(sums);
  free(sums_of_products);
}


void find_ellipse(CvPoint *points, int n_points, CvPoint2D64f *center, CvPoint2D64f *axes)
{
  double *mean = (double*)malloc(2 * sizeof(double));
  double *covariance = (double*)malloc(4 * sizeof(double));

  get_mean_and_covariance_of_points_locations(points, n_points, mean, covariance);

  *center = cvPoint2D64f(mean[0], mean[1]);
  *axes = cvPoint2D64f(sqrt(covariance[0]), sqrt(covariance[3]));

  free(mean);
  free(covariance);
}


double get_covariance_of_xy(rgb_gaussian *gaussian, int x_index, int y_index)
{
  int n_samples = gaussian->m;

  double sum_of_x = gaussian->sums[x_index];
  double sum_of_y = gaussian->sums[y_index];
  double sum_of_xy = gaussian->sums_of_products[x_index][y_index];

  double cov_of_xy = sum_of_xy - ((sum_of_x * sum_of_y) / n_samples);
  cov_of_xy = cov_of_xy / (n_samples - 1);

  return cov_of_xy;
}


bool try_fill_in_gaussian(rgb_gaussian *gaussian)
{
  int n_samples = gaussian->m;

  // sets the means of each RGB component
  gsl_vector_set(gaussian->mean, RED_INDEX, (double)gaussian->sums[RED_INDEX] / n_samples);
  gsl_vector_set(gaussian->mean, GREEN_INDEX, (double)gaussian->sums[GREEN_INDEX] / n_samples);
  gsl_vector_set(gaussian->mean, BLUE_INDEX, (double)gaussian->sums[BLUE_INDEX] / n_samples);

  // sets the covariance matrix
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (j >= i)//the upper matrix and main diagonal
        gsl_matrix_set(gaussian->cov, i, j, get_covariance_of_xy(gaussian, i, j));
      else//the lower matrix, since it is symmetric
        gsl_matrix_set(gaussian->cov, i, j, gsl_matrix_get(gaussian->cov, j, i));
    }
  }

  gaussian->det_cov = gsl_get_determinant(gaussian->cov);
  if (gaussian->det_cov == 0.0)
    return false;

  // pre-compute a gaussian fill-in threshould and other data used to compute normal
  set_gaussian_threshould(gaussian, 3.0);
  set_inverse(gaussian);
  return true;
}

// calculate mean and covariance of the gaussian in the RGB samples
// formula is based on http://www.wikihow.com/Calculate-Covariance
rgb_gaussian *get_gaussian_in_samples(CvScalar *rgb_samples, int n_samples)
{
  rgb_gaussian *gaussian = init_rgb_gaussian(n_samples);

  // calculate covariance, based on http://www.wikihow.com/Calculate-Covariance
  for (int i = 0; i < n_samples; i++)
  {
    CvScalar pixel = rgb_samples[i];

    // get the RGB component values of the current pixel in line
    int red_value = pixel.val[RED_INDEX];
    int green_value = pixel.val[GREEN_INDEX];
    int blue_value = pixel.val[BLUE_INDEX];

    // sum the components to compute the mean
    gaussian->sums[RED_INDEX] += red_value;
    gaussian->sums[GREEN_INDEX] += green_value;
    gaussian->sums[BLUE_INDEX] += blue_value;

    // compute the upper matrix to compute the covariances between components.
    gaussian->sums_of_products[RED_INDEX][RED_INDEX] += (red_value * red_value);
    gaussian->sums_of_products[GREEN_INDEX][GREEN_INDEX] += (green_value * green_value);
    gaussian->sums_of_products[BLUE_INDEX][BLUE_INDEX] += (blue_value * blue_value);
    gaussian->sums_of_products[RED_INDEX][GREEN_INDEX] += (red_value * green_value);
    gaussian->sums_of_products[RED_INDEX][BLUE_INDEX] += (red_value * blue_value);
    gaussian->sums_of_products[GREEN_INDEX][BLUE_INDEX] += (blue_value * green_value);
  }

  // since the covariance is symmetric, compute the lower matrix
  gaussian->sums_of_products[GREEN_INDEX][RED_INDEX] = gaussian->sums_of_products[RED_INDEX][GREEN_INDEX];
  gaussian->sums_of_products[BLUE_INDEX][RED_INDEX] = gaussian->sums_of_products[RED_INDEX][BLUE_INDEX];
  gaussian->sums_of_products[BLUE_INDEX][GREEN_INDEX] = gaussian->sums_of_products[GREEN_INDEX][BLUE_INDEX];

  if (try_fill_in_gaussian(gaussian))
    return gaussian;
  else
  {
    release_gaussian(gaussian);
    return NULL;
  }
}

// merge the gaussian at index into the new gaussian.
// set the merged gaussian to the index position and also release memory of the old gaussian
inline void merge_gaussians(ml_road_finding *road_finding_instance, rgb_gaussian *new_gaussian, int index)
{
  rgb_gaussian *old_gaussian = road_finding_instance->global_gaussians[index];

  double importance = (double)old_gaussian->m_t / old_gaussian->m;

  //new_gaussian->m += old_gaussian->m;
  new_gaussian->m += old_gaussian->m_t;

  // sum the components to compute the mean
  new_gaussian->sums[RED_INDEX] += round(importance * old_gaussian->sums[RED_INDEX]);
  new_gaussian->sums[GREEN_INDEX] += round(importance * old_gaussian->sums[GREEN_INDEX]);
  new_gaussian->sums[BLUE_INDEX] += round(importance * old_gaussian->sums[BLUE_INDEX]);

  // compute the upper matrix to compute the covariances between components.
  new_gaussian->sums_of_products[RED_INDEX][RED_INDEX] += round(importance * old_gaussian->sums_of_products[RED_INDEX][RED_INDEX]);
  new_gaussian->sums_of_products[GREEN_INDEX][GREEN_INDEX] += round(importance * old_gaussian->sums_of_products[GREEN_INDEX][GREEN_INDEX]);
  new_gaussian->sums_of_products[BLUE_INDEX][BLUE_INDEX] += round(importance * old_gaussian->sums_of_products[BLUE_INDEX][BLUE_INDEX]);
  new_gaussian->sums_of_products[RED_INDEX][GREEN_INDEX] += round(importance * old_gaussian->sums_of_products[RED_INDEX][GREEN_INDEX]);
  new_gaussian->sums_of_products[RED_INDEX][BLUE_INDEX] += round(importance * old_gaussian->sums_of_products[RED_INDEX][BLUE_INDEX]);
  new_gaussian->sums_of_products[GREEN_INDEX][BLUE_INDEX] += round(importance * old_gaussian->sums_of_products[GREEN_INDEX][BLUE_INDEX]);

  // since the covariance is symmetric, compute the lower matrix
  new_gaussian->sums_of_products[GREEN_INDEX][RED_INDEX] = new_gaussian->sums_of_products[RED_INDEX][GREEN_INDEX];
  new_gaussian->sums_of_products[BLUE_INDEX][RED_INDEX] = new_gaussian->sums_of_products[RED_INDEX][BLUE_INDEX];
  new_gaussian->sums_of_products[BLUE_INDEX][GREEN_INDEX] = new_gaussian->sums_of_products[GREEN_INDEX][BLUE_INDEX];

  try_fill_in_gaussian(new_gaussian);

  road_finding_instance->global_gaussians[index] = new_gaussian;

  release_gaussian(old_gaussian);
}

inline double get_mahalanobis_distance(rgb_gaussian *g0, rgb_gaussian *g1)
{
  gsl_matrix *cov_sum = gsl_matrix_alloc(3, 3);

  gsl_matrix_memcpy(cov_sum, g0->cov);

  gsl_matrix_add(cov_sum, g1->cov);

  double mahalanobis_distance = gsl_mahalanobis_distance(g0->mean, g1->mean, cov_sum);

  gsl_matrix_free(cov_sum);

  return mahalanobis_distance;
}

// adds the gaussian to the mixture: add it and discard the less representative or merge with the closest gaussian
void add_gaussian(ml_road_finding *road_finding_instance, rgb_gaussian *new_gaussian)
{
  if (new_gaussian == NULL)
    return;

  double decay_factor = 0.95;
  int modified_index = 0;
  double min_distance = DBL_MAX;
  double threshould_distance = 2.0;

  // still have some empty slots?
  if (road_finding_instance->n_gaussians < road_finding_instance->n_max_gaussians)
  {
    modified_index = road_finding_instance->n_gaussians;
    road_finding_instance->global_gaussians[modified_index] = new_gaussian;
    road_finding_instance->n_gaussians++;
  }
  else
  {
    // find the gaussian in mixture that is closest to the new gaussian we are trying to add
    for (int i = 0; i < road_finding_instance->n_gaussians; i++)
    {
      double distance = get_mahalanobis_distance(road_finding_instance->global_gaussians[i], new_gaussian);

      if (distance < min_distance)
      {
        min_distance = distance;
        modified_index = i;
      }
    }

    if (min_distance < threshould_distance) // merge the new gaussian with the closest one
    {
      merge_gaussians(road_finding_instance, new_gaussian, modified_index);
    }
    else // discard the less representative gaussian and add the new gaussian to the collection
    {
      int min_samples = INT_MAX;
      modified_index = 0;

      //find the less representative gaussian, i.e., the gaussian with less samples
      for (int i = 0; i < road_finding_instance->n_gaussians; i++)
      {
        if (road_finding_instance->global_gaussians[i]->m < min_samples)
        {
          min_samples = road_finding_instance->global_gaussians[i]->m_t;
          modified_index = i;
        }
      }

      release_gaussian(road_finding_instance->global_gaussians[modified_index]);
      road_finding_instance->global_gaussians[modified_index] = new_gaussian;
    }
  }

  for (int i = 0; i < road_finding_instance->n_gaussians; i++)
  {
    int n_samples = road_finding_instance->global_gaussians[i]->m_t;
    road_finding_instance->global_gaussians[i]->m_t = i == modified_index ? n_samples : round(n_samples * decay_factor);
  }

#ifndef NO_CUDA
  add_gaussian_to_GPU(road_finding_instance->global_gaussians, road_finding_instance->n_gaussians);
#endif
}

inline double get_normal_opt(gsl_vector *sample, rgb_gaussian *gaussian)
{
	CvPoint3D64f X;
	double distance;

	//X = sample - MEAN
	X.x = gsl_vector_get(sample, RED_INDEX) - gsl_vector_get(gaussian->mean, RED_INDEX);
	X.y = gsl_vector_get(sample, GREEN_INDEX) - gsl_vector_get(gaussian->mean, GREEN_INDEX);
	X.z = gsl_vector_get(sample, BLUE_INDEX) - gsl_vector_get(gaussian->mean, BLUE_INDEX);

	distance = (X.x * gaussian->cov_inv[0] + X.y * gaussian->cov_inv[1] +  X.z * gaussian->cov_inv[2]) * X.x;
	distance += (X.x * gaussian->cov_inv[3] + X.y * gaussian->cov_inv[4] +  X.z * gaussian->cov_inv[5]) * X.y;
	distance += (X.x * gaussian->cov_inv[6] + X.y * gaussian->cov_inv[7] +  X.z * gaussian->cov_inv[8]) * X.z;

	double f1 = 1.0f / (SQRT_2_PI_POW_3 * sqrt(fabs(gaussian->det_cov)));
	return f1 * exp(-0.5f * distance);
}


inline double get_normal_value(int red_value, int green_value, int blue_value, rgb_gaussian *gaussian)
{
  gsl_vector *x = gsl_vector_alloc(3);

  gsl_vector_set(x, RED_INDEX, (double)red_value);
  gsl_vector_set(x, GREEN_INDEX, (double)green_value);
  gsl_vector_set(x, BLUE_INDEX, (double)blue_value);

  double normal = get_normal_opt(x, gaussian);

  gsl_vector_free(x);

  return normal;
}

inline bool fits_gaussian(int red_value, int green_value, int blue_value, rgb_gaussian *gaussian)
{
  double normal = get_normal_value(red_value, green_value, blue_value, gaussian);

  return (normal >= gaussian->probability_threshould);
}

// public interface methods

void print_matrix(gsl_matrix *m, FILE *dst)
{
  fprintf(dst, "Matrix\n");
  for (unsigned int i = 0; i < m->size1; i++)
  {
    for (unsigned int j = 0; j < m->size2; j++)
    {
      fprintf(dst, "%.2f\t\t", gsl_matrix_get(m, i, j));
    }
    fprintf(dst, "\n");
  }
}

void print_distances(ml_road_finding *road_finding_instance, rgb_gaussian *gaussian, FILE *dst)
{
  for (int i = 0; i < road_finding_instance->n_gaussians; i++)
  {
    double distance = get_mahalanobis_distance(road_finding_instance->global_gaussians[i], gaussian);
    fprintf(dst, "\nMahalanobis distance to [%d] = %.5f\n", i, distance);
  }
}

void print_gaussian(rgb_gaussian *gaussian, FILE *dst)
{
  fprintf(dst, "Gaussian from %d samples\t", gaussian->m);
  fprintf(dst, "Mean[R,G,B] = [%.2f, %.2f, %.2f]\n",
  gsl_vector_get(gaussian->mean, RED_INDEX),
  gsl_vector_get(gaussian->mean, GREEN_INDEX),
  gsl_vector_get(gaussian->mean, BLUE_INDEX));

//  fprintf(dst, "Mean, Threshould = [%.6f, %.6f]\n",
//      gaussian->probability_mean,
//      gaussian->probability_threshould);

  fprintf(dst, "Covariance ");
  print_matrix(gaussian->cov, dst);

  fprintf(dst, "Inverse Covariance ");
  for (int i = 0; i < 3 * 3; i++)
  {
    if (i % 3 == 0)
      fprintf(dst, "\n");
    fprintf(dst, "%.3f\t\t", gaussian->cov_inv[i]);
  }
}

void release_gaussian(rgb_gaussian *gaussian)
{
  gsl_vector_free(gaussian->mean);
  gsl_matrix_free(gaussian->cov);
  free(gaussian);
}

void clear_ml_road_finding(ml_road_finding *road_finding_instance)
{
  for (int i = 0; i < road_finding_instance->n_gaussians; i++)
  {
    if (road_finding_instance->global_gaussians[i] != NULL)
    {
      release_gaussian(road_finding_instance->global_gaussians[i]);
    }
  }

  road_finding_instance->n_gaussians = 0;
}

void destroy_ml_road_finding(ml_road_finding *road_finding_instance)
{
  clear_ml_road_finding(road_finding_instance);
  free(road_finding_instance->global_gaussians);
  free(road_finding_instance);
}

void init_ml_road_finding(ml_road_finding *road_finding_instance, int max_gaussians, int width_step, int stereo_height)
{
  road_finding_instance->n_gaussians = 0;
  road_finding_instance->threshould_distance = DBL_MAX;

  road_finding_instance->n_max_gaussians = max_gaussians;

  road_finding_instance->global_gaussians = (rgb_gaussian**)malloc(road_finding_instance->n_max_gaussians * sizeof(rgb_gaussian*));

  road_finding_instance->m_width_step = width_step;
  road_finding_instance->m_stereo_height = stereo_height;

#ifndef NO_CUDA
  init_cuda_road_finding(max_gaussians, width_step, stereo_height);
#endif
}

rgb_gaussian *init_rgb_gaussian(int n_samples)
{
  rgb_gaussian *gaussian = (rgb_gaussian*)malloc(sizeof(rgb_gaussian));
  gaussian->m = n_samples;
  gaussian->m_t = n_samples;
  gaussian->mean = gsl_vector_alloc(3);
  gaussian->cov = gsl_matrix_alloc(3, 3);
  gaussian->probability_threshould = 0.0;
  gaussian->probability_mean = 0.0;
  gaussian->det_cov = 0.0;

  // init all elements of the matrix used to compute covariance with zero
  memset(gaussian->sums, 0, 3 * sizeof(unsigned long));
  memset(gaussian->sums_of_products, 0, 9 * sizeof(unsigned long));

  return gaussian;
}


int get_samples_at_rect(CvScalar *rgb_samples, IplImage *image, CvRect rect, int *mask)
{
  int i = 0;

  for (int y = rect.y; y < rect.y + rect.height; y++)
  {
    for (int x = rect.x; x < rect.x + rect.width; x++)
    {
      if (mask == NULL || mask[y * image->width + x] > 0)//if it is out of perceptual field
      {
        rgb_samples[i] = cvGet2D(image, y, x);
        i++;
      }
    }
  }

  return i;
}

rgb_gaussian *get_gaussian_at_rect(IplImage *image, CvRect rect, int *mask)
{
  if (rect.height <= 0 || rect.width<= 0)
    return NULL;

  CvScalar *rgb_samples = (CvScalar*)malloc(rect.width * rect.height * sizeof(CvScalar));
  int n_samples = get_samples_at_rect(rgb_samples, image, rect, mask);
  rgb_gaussian *gaussian = get_gaussian_in_samples(rgb_samples, n_samples);
  free(rgb_samples);

  return gaussian;
}

static bool
point_in_ellipse(CvPoint P, CvPoint2D64f C, CvPoint2D64f axes)
{
  return (double)(P.x - C.x) * (P.x - C.x) / (axes.x * axes.x) + (double)(P.y - C.y) * (P.y - C.y) / (axes.y * axes.y) <= 1.0;
}

rgb_gaussian *get_gaussian_in_ellipse(IplImage *image, CvPoint2D64f C, CvPoint2D64f axes)
{
  if (axes.x < 0.0 || axes.y < 0.0)
    return NULL;

  if (isnan(axes.x ) || isnan(axes.y) || isinf(axes.x) || isinf(axes.y))
    return NULL;

  int x0 = C.x - axes.x;
  int y0 = C.y + axes.y;
  int width = 2 * axes.x;
  int height = 2 * axes.y;
  int n_samples = 0;

  CvScalar *rgb_samples = (CvScalar*)malloc(width * height * sizeof(CvScalar));

  for (int y = y0; y > y0 - height; y--)
  {
    for (int x = x0; x < x0 + width; x++)
    {
      if (point_in_ellipse(cvPoint(x,y), C, axes))
      {
        rgb_samples[n_samples] = cvGet2D(image, y, x);
        n_samples++;
      }
    }
  }

  rgb_gaussian *gaussian = get_gaussian_in_samples(rgb_samples, n_samples);

  free(rgb_samples);

  return gaussian;
}

static void fill_image_based_on_gaussians_openMP(ml_road_finding *road_finding_instance, IplImage *src, IplImage *probabilistic_dst, int *mask)
{
  double values[road_finding_instance->n_gaussians];
	CvScalar pixel;
	int x, y, i, best_gaussian;

#ifndef DEBUG
#pragma omp parallel num_threads(8) default(none) \
  private(x,y,i,best_gaussian,pixel,values)\
  shared(src,probabilistic_dst,road_finding_instance,mask)
#endif
	{
#ifndef DEBUG
    #pragma omp for
#endif
		for (y = 0; y < src->height; y++)
		{
			for (x = 0; x < src->width; x++)
			{
			  if (mask != NULL && mask[y * src->width + x] <= 0)//if pixel is out of mask
			  {
			    cvSet2D(probabilistic_dst, y, x, cvScalarAll(0));
			    continue;
			  }

				pixel = cvGet2D(src, y, x);

				best_gaussian = 0;
				values[0] = get_normal_value(pixel.val[RED_INDEX], pixel.val[GREEN_INDEX], pixel.val[BLUE_INDEX], road_finding_instance->global_gaussians[0]);

				for (i = 1; i < road_finding_instance->n_gaussians; i++)
				{
				  values[i] = get_normal_value(pixel.val[RED_INDEX], pixel.val[GREEN_INDEX], pixel.val[BLUE_INDEX], road_finding_instance->global_gaussians[i]);
				  best_gaussian = (values[i] > values[best_gaussian]) ? i : best_gaussian;
				}

				values[best_gaussian] = round(255.0 * values[best_gaussian] * road_finding_instance->global_gaussians[best_gaussian]->normalizer);
        cvSet2D(probabilistic_dst, y, x, cvScalarAll(values[best_gaussian]));
			}
		}
	}
}

void fill_image_based_on_gaussians(ml_road_finding *road_finding_instance, IplImage *src, IplImage *probabilistic_dst, int *mask)
{
#ifndef NO_CUDA
  fill_image_based_on_gaussians_GPU((unsigned char *)src->imageData, src->width, src->height, src->widthStep, road_finding_instance->n_gaussians);
#else
  fill_image_based_on_gaussians_openMP(road_finding_instance, src, probabilistic_dst, mask);
#endif
}

