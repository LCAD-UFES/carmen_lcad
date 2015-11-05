/*
  stereo
  Functions for simple sparse stereo
  Copyright (C) 2009 Bob Mottram and Giacomo Spigler
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

#include "stereo.h"

/* offsets of pixels to be compared within the patch region
 * arranged into a roughly rectangular structure */
const int pixel_offsets[] = { -2, -4, -1, -4, 1, -4, 2, -4, -5, -2, -4, -2, -3,
			      -2, -2, -2, -1, -2, 0, -2, 1, -2, 2, -2, 3, -2, 4, -2, 5, -2, -5, 2,
			      -4, 2, -3, 2, -2, 2, -1, 2, 0, 2, 1, 2, 2, 2, 3, 2, 4, 2, 5, 2, -2, 4,
			      -1, 4, 1, 4, 2, 4 };

/* lookup table used for counting the number of set bits */
const unsigned char BitsSetTable256[] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3,
					  2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3,
					  2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
					  4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4,
					  3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
					  4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3,
					  2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
					  4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5,
					  4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
					  4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5,
					  4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7,
					  6, 7, 7, 8 };

svs::svs(int width, int height) {

  imgWidth = width;
  imgHeight = height;

  /* ground plane */
  enable_ground_priors = 0;
  ground_y_percent = 50;

  /* array storing x coordinates of detected features */
  feature_x = new short int[SVS_MAX_FEATURES];
  feature_y = new short int[SVS_MAX_FEATURES];

  disparity_histogram = new unsigned short int[SVS_MAX_IMAGE_WIDTH/2];
  disparity_histogram_plane = new int[(SVS_MAX_IMAGE_WIDTH/SVS_FILTER_SAMPLING)*(SVS_MAX_IMAGE_WIDTH / 2)];
  disparity_plane_fit = new int[SVS_MAX_IMAGE_WIDTH / SVS_FILTER_SAMPLING];
  plane = new int[15*9];

  calibration_map = NULL;

  /* array storing the number of features detected on each row */
  features_per_row = new unsigned short int[SVS_MAX_IMAGE_HEIGHT
					    / SVS_VERTICAL_SAMPLING];
  features_per_col = new unsigned short int[SVS_MAX_IMAGE_WIDTH
					    / SVS_HORIZONTAL_SAMPLING];

  /* Array storing a binary descriptor, 32bits in length, for each detected feature.
   * This will be used for matching purposes.*/
  descriptor = new unsigned int[SVS_MAX_FEATURES];

  /* mean luminance for each feature */
  mean = new unsigned char[SVS_MAX_FEATURES];

  /* buffer which stores sliding sum */
  row_sum = new int[SVS_MAX_IMAGE_WIDTH];

  /* buffer used to find peaks in edge space */
  row_peaks = new unsigned int[SVS_MAX_IMAGE_WIDTH];
  temp_row_peaks = new unsigned int[SVS_MAX_IMAGE_WIDTH];

  /* low contrast areas of the image */
  enable_segmentation = 0;
  enable_region_tracking = 0;
  low_contrast = NULL;
  region_volume = NULL;
  region_centre = NULL;
  prev_region_centre = NULL;
  region_history_index = -1;
  region_bounding_box = NULL;
  region_colour = NULL;
  region_disparity = NULL;
  no_of_regions = 0;

  /* array stores matching probabilities (prob,x,y,disp) */
  svs_matches = NULL;

  /* array used during filtering */
  valid_quadrants = NULL;

}

svs::~svs() {
  delete[] feature_x;
  delete[] features_per_row;
  delete[] descriptor;
  delete[] mean;
  delete[] row_sum;
  delete[] row_peaks;
  delete[] temp_row_peaks;
  if (region_disparity != NULL)
    delete[] region_disparity;
  if (low_contrast != NULL)
    delete[] low_contrast;
  if (region_volume != NULL)
    delete[] region_volume;
  if (region_centre != NULL) {
    delete[] region_centre;
    for (int i = 0; i < SVS_REGION_HISTORY; i++) {
      delete[] prev_region_centre[i];
    }
    delete[] prev_region_centre;
  }
  if (region_bounding_box != NULL)
    delete[] region_bounding_box;
  if (region_colour != NULL)
    delete[] region_colour;
  if (svs_matches != NULL)
    delete[] svs_matches;
  if (valid_quadrants != NULL)
    delete[] valid_quadrants;
  if (disparity_histogram != NULL)
    delete[] disparity_histogram;
  if (disparity_histogram_plane != NULL)
    delete[] disparity_histogram_plane;
  if (disparity_plane_fit != NULL)
    delete[] disparity_plane_fit;
  if (calibration_map != NULL)
    delete[] calibration_map;
}

/*!
 * \brief performs histogram equalisation using openCV
 * \param hist_image mono image used for the conversion
 * \param img colour image
 * \param img_width width of the image
 * \param img_height height of the image
 */
void svs::histogram_equalise(
			     IplImage* hist_image,
			     unsigned char* img,
			     int img_width,
			     int img_height)
{
#pragma omp parallel for
  for (int i = img_width*img_height-1; i >= 0; i--) {
    hist_image->imageData[i] = (img[i*3+2] + img[i*3+1] + img[i*3])/3;
  }

  cvEqualizeHist( hist_image, hist_image );

#pragma omp parallel for
  for (int i = img_width*img_height-1; i >= 0; i--) {
    int magnitude_corrected = hist_image->imageData[i];
    img[i*3] = magnitude_corrected;
    img[i*3+1] = magnitude_corrected;
    img[i*3+2] = magnitude_corrected;
  }
}

/* Updates sliding sums and edge response values along a single row or column
 * Returns the mean luminance along the row or column */
int svs::update_sums(int cols, /* if non-zero we're dealing with columns not rows */
                     int i, /* row or column index */
                     unsigned char* rectified_frame_buf, /* image data */
                     int segment __attribute__((unused))) { /* if non zero update low contrast areas used for segmentation */

  int j, k, x, y, idx, max, sum = 0, mean = 0;

  if (cols == 0) {
    /* compute sums along the row */
    y = i;
    idx = imgWidth * y * 3 + 2;
    max = (int) imgWidth;

    row_sum[0] = rectified_frame_buf[idx];
    for (x = 1; x < max; x++, idx += 3) {
      sum += rectified_frame_buf[idx];
      row_sum[x] = sum;
    }

  } else {
    /* compute sums along the column */
    idx = i * 3 + 2;
    x = i;
    max = (int) imgHeight;
    int stride = (int) imgWidth * 3;

    row_sum[0] = rectified_frame_buf[idx];
    for (y = 1; y < max; y++, idx += stride) {
      sum += rectified_frame_buf[idx];
      row_sum[y] = sum;
    }

  }

  /* row mean luminance */
  mean = sum / max;

  /* compute peaks */
  int p0, p1, p2, p;
  av_peaks = 0;
  for (j = SVS_PEAK_WIDTH; j < max - SVS_PEAK_WIDTH; j++) {
    sum = row_sum[j];

    /* edge using 1 pixel radius */
    p0 = (sum - row_sum[j - 1]) - (row_sum[j + 1] - sum);
    if (p0 < 0)
      p0 = -p0;

    /* edge using 3 pixel radius */
    p1 = (sum - row_sum[j - 3]) - (row_sum[j + 3] - sum);
    if (p1 < 0)
      p1 = -p1;

    /* edge using 5 pixel radius */
    p2 = (sum - row_sum[j - 5]) - (row_sum[j + 5] - sum);
    if (p2 < 0)
      p2 = -p2;

    /* overall edge response */
    p = p0*8 + p1*2 + p2;
    row_peaks[j] = p;
    av_peaks += p;
  }
  av_peaks /= (max - 8);

  memcpy((void*)temp_row_peaks, (void*)row_peaks, max*sizeof(unsigned int));

  /* create a map of low contrast areas, to be used for segmentation */
  if (enable_segmentation) {
    if (low_contrast == NULL) {
      low_contrast = new unsigned short[SVS_MAX_IMAGE_WIDTH
					* SVS_MAX_IMAGE_HEIGHT];
      region_volume = new unsigned int[SVS_MAX_REGIONS];
      region_centre = new unsigned int[SVS_MAX_REGIONS * 2];
      prev_region_centre = new unsigned short*[SVS_REGION_HISTORY];
      region_disparity = new unsigned char[SVS_MAX_REGIONS * 3];
      for (j = 0; j < SVS_REGION_HISTORY; j++) {
	prev_region_centre[j] = new unsigned short[SVS_MAX_REGIONS * 4
						   + 1];
	prev_region_centre[j][0] = 0;
      }
      region_bounding_box = new unsigned short[SVS_MAX_REGIONS * 4];
      region_colour = new unsigned int[SVS_MAX_REGIONS * 3];
    }

    if (cols == 0) {
      for (j = 4; j < max - 4; j++) {
	if (row_peaks[j] < av_peaks)
	  low_contrast[i * imgWidth + j] = 65535;
      }
    } else {
      for (j = 4; j < max - 5; j++) {
	if (row_peaks[j] > av_peaks) {
	  for (k = -1; k <= 1; k++) {
	    low_contrast[j * imgWidth + i + k] = 0;
	    low_contrast[(j - 1) * imgWidth + i + k] = 0;
	  }
	}
      }
    }
  }

  return (mean);
}

/* performs non-maximal suppression on the given row or column */
void svs::non_max(int cols, /* if non-zero we're dealing with columns not rows */
		  int inhibition_radius, /* radius for non-maximal suppression */
		  unsigned int min_response) { /* minimum threshold as a percent in the range 0-100 */

  int i, r, max, max2;
  unsigned int v;

  /* maximum response */
  unsigned int max_peaks = (unsigned int)0;
  max = (int) imgWidth;
  if (cols != 0)
    max = (int) imgHeight;
  max2 = max - inhibition_radius;
  max -= SVS_PEAK_WIDTH;
  for (i = SVS_PEAK_WIDTH; i < max2; i++) {
    if (row_peaks[i] > max_peaks) max_peaks = row_peaks[i];
  }
  unsigned int threshold = max_peaks * min_response / 100;

  for (i = SVS_PEAK_WIDTH; i < max2; i++) {
    if (row_peaks[i] < threshold) {
      row_peaks[i] = (unsigned int)0;
    }
    else {
      v = row_peaks[i];
      if (v > 0) {
	for (r = 1; r < inhibition_radius; r++) {
	  if (row_peaks[i + r] < v) {
	    row_peaks[i + r] = (unsigned int)0;
	  } else {
	    row_peaks[i] = (unsigned int)0;
	    break;
	  }
	}
      }
    }
  }
}

/* creates a binary descriptor for a feature at the given coordinate
   which can subsequently be used for matching */
int svs::compute_descriptor(int px, int py, unsigned char* rectified_frame_buf,
			    int no_of_features, int row_mean) {

  unsigned char bit_count = 0;
  int pixel_offset_idx, ix, bit;
  int meanval = 0;
  unsigned int desc = 0;

  /* find the mean luminance for the patch */
  for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS * 2; pixel_offset_idx
	 += 2) {
    ix
      = rectified_frame_buf[pixindex((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
    meanval += rectified_frame_buf[ix + 2] + rectified_frame_buf[ix + 1]
      + rectified_frame_buf[ix];
  }
  meanval /= SVS_DESCRIPTOR_PIXELS;

  /* binarise */
  bit = 1;
  for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS * 2; pixel_offset_idx
	 += 2, bit *= 2) {

    ix
      = rectified_frame_buf[pixindex((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
    if (rectified_frame_buf[ix + 2] + rectified_frame_buf[ix + 1]
	+ rectified_frame_buf[ix] > meanval) {
      desc |= bit;
      bit_count++;
    }
  }

  if ((bit_count > 3) && (bit_count < SVS_DESCRIPTOR_PIXELS - 3)) {
    meanval /= 3;

    /* adjust the patch luminance relative to the mean
     * luminance for the entire row.  This helps to ensure
     * that comparisons between left and right images are
     * fair even if there are exposure/illumination differences. */
    meanval = meanval - row_mean + 127;
    if (meanval < 0)
      meanval = 0;
    if (meanval > 255)
      meanval = 255;

    meanval /= 16;
    if (meanval > 15) meanval = 15;
    mean[no_of_features] = (unsigned char)meanval;
    descriptor[no_of_features] = desc;
    return (0);
  } else {
    /* probably just noise */
    return (-1);
  }
}

/* returns a set of vertically oriented edge features suitable for stereo matching */
int svs::get_features_vertical(unsigned char* rectified_frame_buf, /* image data */
			       int inhibition_radius, /* radius for non-maximal supression */
			       unsigned int minimum_response, /* minimum threshold */
			       int calibration_offset_x, /* calibration x offset in pixels */
			       int calibration_offset_y, /* calibration y offset in pixels */
			       int segment) { /* if non zero update low contrast areas used for segmentation */

  unsigned short int no_of_feats;
  int x, y, row_mean, start_x, prev_x, mid_x, grad;
  int no_of_features = 0;
  int row_idx = 0;

  memset((void*) (features_per_row), '\0', SVS_MAX_IMAGE_HEIGHT
	 / SVS_VERTICAL_SAMPLING * sizeof(unsigned short));

  start_x = imgWidth - 15;
  if ((int) imgWidth - inhibition_radius - 1 < start_x)
    start_x = (int) imgWidth - inhibition_radius - 1;

  for (y = 4 + calibration_offset_y; y < (int) imgHeight - 4; y += SVS_VERTICAL_SAMPLING) {

    /* reset number of features on the row */
    no_of_feats = 0;

    if ((y >= 4) && (y <= (int) imgHeight - 4)) {

      row_mean = update_sums(0, y, rectified_frame_buf, segment);
      non_max(0, inhibition_radius, minimum_response);

      /* store the features */
      prev_x = start_x;
      for (x = start_x; x > 15; x--) {
	if (row_peaks[x] > 0) {

	  if (compute_descriptor(x, y, rectified_frame_buf,
				 no_of_features, row_mean) == 0) {

	    mid_x = prev_x + ((x - prev_x)/2);
	    if (x != prev_x) {
	      grad =
		((row_sum[mid_x] - row_sum[x]) -
		 (row_sum[prev_x] - row_sum[mid_x])) /
		((prev_x - x)*4);
	      /* limit gradient into a 4 bit range */
	      if (grad < -7) grad = -7;
	      if (grad > 7) grad = 7;
	      /* pack the value into the upper 4 bits */
	      mean[no_of_features] |= ((unsigned char)(grad + 8) << 4);
	    }

	    feature_x[no_of_features] = (short int) (x + calibration_offset_x)*SVS_SUB_PIXEL;

	    /* parabolic sub-pixel interpolation */
	    int denom = 2 * ((int)temp_row_peaks[x-1] - (2*(int)temp_row_peaks[x]) + (int)temp_row_peaks[x+1]);
	    if (denom != 0) {
	      int num = (int)temp_row_peaks[x-1] - (int)temp_row_peaks[x+1];
	      feature_x[no_of_features] += (num*SVS_SUB_PIXEL)/denom;
	    }

	    no_of_features++;

	    no_of_feats++;
	    if (no_of_features == SVS_MAX_FEATURES) {
	      y = (int)imgHeight;
	      printf("stereo feature buffer full\n");
	      break;
	    }
	    prev_x = x;
	  }
	}
      }
    }

    features_per_row[row_idx++] = no_of_feats;
  }

#ifdef SVS_VERBOSE
  printf("%d vertically oriented edge features located\n", no_of_features);
#endif

  return (no_of_features);
}

/* returns a set of horizontally oriented features
   these can't be matched directly, but their disparities might be infered */
int svs::get_features_horizontal(unsigned char* rectified_frame_buf, /* image data */
				 int inhibition_radius, /* radius for non-maximal supression */
				 unsigned int minimum_response, /* minimum threshold */
				 int calibration_offset_x, /* calibration x offset in pixels */
				 int calibration_offset_y, /* calibration y offset in pixels */
				 int segment) { /* if non zero update low contrast areas used for segmentation */
  unsigned short int no_of_feats;
  int x, y, start_y;
  int no_of_features = 0;
  int col_idx = 0;

  /* create arrays */
  if (features_per_col == 0) {
    features_per_col = (unsigned short int*) malloc(SVS_MAX_IMAGE_WIDTH
						    / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short int));
    feature_y = (short int*) malloc(SVS_MAX_FEATURES * sizeof(short int));
  }

  memset((void*) features_per_col, '\0', SVS_MAX_IMAGE_WIDTH
	 / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short));

  start_y = imgHeight - 15;
  if ((int) imgHeight - inhibition_radius - 1 < start_y)
    start_y = (int) imgHeight - inhibition_radius - 1;

  for (x = 4 + calibration_offset_x; x < (int) imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING) {

    /* reset number of features on the row */
    no_of_feats = 0;

    if ((x >= 4) && (x <= (int) imgWidth - 4)) {

      if (update_sums(1, x, rectified_frame_buf, segment)!=9999999) {
	non_max(1, inhibition_radius, minimum_response);
      }

      /* store the features */
      for (y = start_y; y > 15; y--) {
	if (row_peaks[y] > 0) {

	  feature_y[no_of_features++] = (short int) (y + calibration_offset_y);
	  no_of_feats++;
	  if (no_of_features == SVS_MAX_FEATURES) {
	    x = (int)imgWidth;
	    printf("stereo feature buffer full\n");
	    break;
	  }
	}
      }
    }

    features_per_col[col_idx++] = no_of_feats;
  }

#ifdef SVS_VERBOSE
  printf("%d horizontally oriented edge features located\n", no_of_features);
#endif

  return (no_of_features);
}

/* Match features from this camera with features from the opposite one.
 * It is assumed that matching is performed on the left camera CPU */
int svs::match(svs* other, int ideal_no_of_matches, /* ideal number of matches to be returned */
	       int max_disparity_percent, /* max disparity as a percent of image width */
	       int learnDesc, /* descriptor match weight */
	       int learnLuma, /* luminance match weight */
	       int learnDisp, /* disparity weight */
	       int learnGrad, /* horizontal gradient weight */
	       int groundPrior, /* prior for ground plane */
	       int use_priors) /* if non-zero then use priors, assuming time between frames is small */
{
  int x, xL = 0, xR, L, R, y, no_of_feats, no_of_feats_left,
    no_of_feats_right, row, col = 0, bit, disp_diff;
  int luma_diff, min_disp, max_disp = 0, max_disp_pixels,
    meanL, meanR, disp = 0, fL = 0, fR = 0, bestR = 0;
  unsigned int descL, descLanti, descR, desc_match;
  unsigned int correlation, anticorrelation, total, n;
  unsigned int match_prob, best_prob;
  int idx, max, curr_idx = 0, search_idx, winner_idx = 0;
  int no_of_possible_matches = 0, matches = 0;
  int itt, prev_matches;
  int grad_diff0, gradL0, grad_diff1=0, gradL1=0, grad_anti;
  int p, pmax=1, prev_right_x, right_x;
  int min_left, min_right, disp_left=0, disp_left_x=0;
  int disp_right=0, disp_right_x=0, dx, dy, x2, dist;

  unsigned int meandescL, meandescR;
  short meandesc[SVS_DESCRIPTOR_PIXELS];

  /* create arrays */
  if (svs_matches == NULL) {
    svs_matches = new unsigned int[SVS_MAX_MATCHES * 5];
    valid_quadrants = new unsigned char[SVS_MAX_MATCHES];
  }

  /* convert max disparity from percent to pixels */
  max_disp_pixels = max_disparity_percent * imgWidth / 100;
  min_disp = -10;
  max_disp = max_disp_pixels;

  /* ground plane */
  int ground_y = imgHeight * ground_y_percent/100;
  int ground_height = imgHeight - 1 - ground_y;

  row = 0;
  for (y = 4; y < (int) imgHeight - 4; y += SVS_VERTICAL_SAMPLING, row++) {

    /* number of features on left and right rows */
    no_of_feats_left = features_per_row[row];
    no_of_feats_right = other->features_per_row[row];

    /* compute mean descriptor for the left row
     * this will be used to create eigendescriptors */
    meandescL = 0;
    memset(meandesc, 0, (SVS_DESCRIPTOR_PIXELS) * sizeof(short));
    for (L = 0; L < no_of_feats_left; L++) {
      descL = descriptor[fL + L];
      n = 1;
      for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
	if (descL & n)
	  meandesc[bit]++;
	else
	  meandesc[bit]--;
      }
    }
    n = 1;
    for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
      if (meandesc[bit] >= 0)
	meandescL |= n;
    }

    /* compute mean descriptor for the right row
     * this will be used to create eigendescriptors */
    meandescR = 0;
    memset(meandesc, 0, (SVS_DESCRIPTOR_PIXELS) * sizeof(short));
    for (R = 0; R < no_of_feats_right; R++) {
      descR = other->descriptor[fR + R];
      n = 1;
      for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
	if (descR & n)
	  meandesc[bit]++;
	else
	  meandesc[bit]--;
      }
    }
    n = 1;
    for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
      if (meandesc[bit] > 0)
	meandescR |= n;
    }

    int prev_matches = no_of_possible_matches;

    /* features along the row in the left camera */
    for (L = 0; L < no_of_feats_left; L++) {

      /* x coordinate of the feature in the left camera */
      xL = feature_x[fL + L] / SVS_SUB_PIXEL;

      /* mean luminance and eigendescriptor for the left camera feature */
      meanL = mean[fL + L] & 15;
      descL = descriptor[fL + L] & meandescL;

      /* invert bits of the descriptor for anti-correlation matching */
      n = descL;
      descLanti = 0;
      for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++) {
	/* Shift result vector to higher significance. */
	descLanti <<= 1;
	/* Get least significant input bit. */
	descLanti |= n & 1;
	/* Shift input vector to lower significance. */
	n >>= 1;
      }

      gradL0 = mean[fL + L] >> 4;
      if (L < no_of_feats_left - 1) {
	gradL1 = mean[fL + L + 1] >> 4;
      }

      total = 0;

      /* features along the row in the right camera */
      for (R = 0; R < no_of_feats_right; R++) {

	/* set matching score to zero */
	row_peaks[R] = 0;

	if (((gradL0 >= 8) &&
	     ((int)(other->mean[fR + R] >> 4) >= 8)) ||
	    ((gradL0 < 8) &&
	     ((int)(other->mean[fR + R] >> 4) < 8))) {

	  /* x coordinate of the feature in the right camera */
	  xR = other->feature_x[fR + R] / SVS_SUB_PIXEL;

	  /* compute disparity */
	  disp = xL - xR;

	  /* is the disparity within range? */
	  if ((disp >= min_disp) && (disp < max_disp)) {
	    if (disp < 0)
	      disp = 0;

	    /* mean luminance for the right camera feature */
	    meanR = other->mean[fR + R] & 15;

	    /* is the mean luminance similar? */
	    luma_diff = meanR - meanL;

	    /* right camera feature eigendescriptor */
	    descR = other->descriptor[fR + R] & meandescR;

	    /* bitwise descriptor correlation match */
	    desc_match = descL & descR;

	    /* count the number of correlation bits */
	    correlation = BitsSetTable256[desc_match & 0xff]
	      + BitsSetTable256[(desc_match >> 8) & 0xff]
	      + BitsSetTable256[(desc_match >> 16) & 0xff]
	      + BitsSetTable256[desc_match >> 24];

	    /* bitwise descriptor anti-correlation match */
	    desc_match = descLanti & descR;

	    /* count the number of anti-correlation bits */
	    anticorrelation = BitsSetTable256[desc_match & 0xff]
	      + BitsSetTable256[(desc_match >> 8) & 0xff]
	      + BitsSetTable256[(desc_match >> 16) & 0xff]
	      + BitsSetTable256[desc_match >> 24];

	    grad_anti = (15 - gradL0) - (int)(other->mean[fR + R] >> 4);
	    if (grad_anti < 0) grad_anti = -grad_anti;

	    grad_diff0 = gradL0 - (int)(other->mean[fR + R] >> 4);
	    if (grad_diff0 < 0) grad_diff0 = -grad_diff0;
	    grad_diff0 = (15 - grad_diff0) - (15 - grad_anti);

	    if (R < no_of_feats_right - 1) {
	      grad_anti = (15 - gradL1) - (int)(other->mean[fR + R + 1] >> 4);
	      if (grad_anti < 0) grad_anti = -grad_anti;

	      grad_diff1 = gradL1 - (int)(other->mean[fR + R + 1] >> 4);
	      if (grad_diff1 < 0) grad_diff1 = -grad_diff1;
	      grad_diff1 = (15 - grad_diff1) - (15 - grad_anti);
	    }

	    if (luma_diff < 0)
	      luma_diff = -luma_diff;
	    int score =
	      10000 + (max_disp * learnDisp) +
	      (grad_diff0 * learnGrad) +
	      (grad_diff1 * learnGrad) +
	      (((int) correlation
		+ (int) (SVS_DESCRIPTOR_PIXELS
			 - anticorrelation))
	       * learnDesc) - (luma_diff
			       * learnLuma) - (disp * learnDisp);
	    if (use_priors) {
	      /* bias for ground_plane */
	      if (enable_ground_priors) {
		if (y > ground_y) {
		  disp_diff = disp - ((y - ground_y) * max_disp_pixels / ground_height);
		  if (disp_diff < 0)
		    disp_diff = -disp_diff;
		  score -= disp_diff * groundPrior;
		}
	      }
	    }
	    if (score < 0)
	      score = 0;

	    /* store overall matching score */
	    row_peaks[R] = (unsigned int) score;
	    total += row_peaks[R];

	  } else {
	    if ((disp < min_disp) && (disp > -max_disp)) {
	      row_peaks[R] = (unsigned int) ((max_disp - disp)
					     * learnDisp);
	      total += row_peaks[R];
	    }
	  }
	}
      }

      /* non-zero total matching score */
      if (total > 0) {

	/* several candidate disparities per feature
	   observing the principle of least commitment */
	for (p = 0; p < pmax; p++) {

	  /* convert matching scores to probabilities */
	  best_prob = 0;
	  for (R = 0; R < no_of_feats_right; R++) {
	    if (row_peaks[R] > 0) {
	      match_prob = row_peaks[R] * 1000 / total;
	      if (match_prob > best_prob) {
		best_prob = match_prob;
		bestR = R;
	      }
	    }
	  }

	  if ((best_prob > 0) &&
	      (best_prob < 1000) &&
	      (no_of_possible_matches < SVS_MAX_FEATURES)) {

	    /* x coordinate of the feature in the right camera */
	    xR = other->feature_x[fR + bestR] / SVS_SUB_PIXEL;

	    /* possible disparity */
	    disp = xL - xR;

	    if ((disp >= min_disp) &&
		(disp < max_disp_pixels)) {
	      if (disp < 0) disp = 0;

	      /* add the best result to the list of possible matches */
	      svs_matches[no_of_possible_matches * 5] = best_prob;
	      svs_matches[no_of_possible_matches * 5 + 1]
		= (unsigned int) feature_x[fL + L];
	      svs_matches[no_of_possible_matches * 5 + 2]
		= (unsigned int) y;
	      svs_matches[no_of_possible_matches * 5 + 3]
		= (unsigned int) (feature_x[fL + L] - other->feature_x[fR + bestR]);
	      if (p > 0) {
		svs_matches[no_of_possible_matches * 5 + 1] += imgWidth*SVS_SUB_PIXEL;
	      }
	      no_of_possible_matches++;

	      row_peaks[bestR] = 0;
	    }
	  }
	}
      }
    }

    /* apply ordering constraint within the right image */
    prev_right_x = 0;
    for (int m = no_of_possible_matches-1; m >= prev_matches; m--) {
      right_x = (int)svs_matches[m*5 + 1] + (int)svs_matches[m*5 + 3];
      if (right_x < prev_right_x) {
	/* set probability to zero if rays cross (occlusion) */
	if (svs_matches[(m+1) * 5] >= svs_matches[m * 5]) {
	  svs_matches[m * 5] = 0;
	}
	else {
	  svs_matches[(m+1) * 5] = 0;
	  prev_right_x = right_x;
	}
      }
      else {
	prev_right_x = right_x;
      }
    }

    /* increment feature indexes */
    fL += no_of_feats_left;
    fR += no_of_feats_right;
  }

  if (no_of_possible_matches > 20) {

    /* filter the results */
    filter_plane(no_of_possible_matches, max_disp);

    /* sort matches in descending order of probability */
    if (no_of_possible_matches < ideal_no_of_matches) {
      ideal_no_of_matches = no_of_possible_matches;
    }
    curr_idx = 0;
    search_idx = 0;
    for (matches = 0; matches < ideal_no_of_matches; matches++, curr_idx
	   += 5) {

      match_prob = svs_matches[curr_idx];
      if (match_prob > 0)
	winner_idx = curr_idx;
      else
	winner_idx = -1;

      search_idx = curr_idx + 5;
      max = no_of_possible_matches * 5;
      while (search_idx < max) {
	if (svs_matches[search_idx] > match_prob) {
	  match_prob = svs_matches[search_idx];
	  winner_idx = search_idx;
	}
	search_idx += 5;
      }
      if (winner_idx > -1) {

	/* swap */
	best_prob = svs_matches[winner_idx];
	xL = svs_matches[winner_idx + 1];
	y = svs_matches[winner_idx + 2];
	disp = svs_matches[winner_idx + 3];

	svs_matches[winner_idx] = svs_matches[curr_idx];
	svs_matches[winner_idx + 1] = svs_matches[curr_idx + 1];
	svs_matches[winner_idx + 2] = svs_matches[curr_idx + 2];
	svs_matches[winner_idx + 3] = svs_matches[curr_idx + 3];

	svs_matches[curr_idx] = best_prob;
	svs_matches[curr_idx + 1] = xL;
	svs_matches[curr_idx + 2] = y;
	svs_matches[curr_idx + 3] = disp;

	/* update your priors */
	if (svs_matches[winner_idx + 1] >= imgWidth*SVS_SUB_PIXEL) {
	  svs_matches[winner_idx + 1] -= imgWidth*SVS_SUB_PIXEL;
	}
      }

      if (svs_matches[curr_idx] == 0) {
	break;
      }

    }

    /* attempt to assign disparities to horizontally oriented features */
    memset(valid_quadrants, 0, SVS_MAX_MATCHES * sizeof(unsigned char));
    itt = 0;
    prev_matches = matches;
    for (itt = 0; itt < 10; itt++) {
      fL = 0;
      col = 0;
      for (x = 4; x < (int) imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING, col++) {

	no_of_feats = features_per_col[col];

	/* features along the row in the left camera */
	for (L = 0; L < no_of_feats; L++) {

	  if (valid_quadrants[fL + L] == 0) {
	    /* y coordinate of the feature in the left camera */
	    y = feature_y[fL + L];

	    /* lookup disparity from priors */
	    row = y / SVS_VERTICAL_SAMPLING;

	    min_left = 99999;
	    min_right = 99999;
	    for (idx = prev_matches-1; idx >= 0; idx--) {
	      if (svs_matches[idx*5] > 0) {
		x2 = (int)svs_matches[idx*5+1]/SVS_SUB_PIXEL;
		dx = x - x2;
		dy = y - (int)svs_matches[idx*5+2];
		if ((dy > -5) && (dy < 5)) {
		  if (dy < 0)
		    dist = -dy;
		  else
		    dist = dy;

		  if (dx > 0) {
		    dist += dx;
		    if ((dist < min_left) &&
			(svs_matches[idx*5+3] < 9999)) {
		      min_left = dist;
		      disp_left = (int)svs_matches[idx*5+3];
		      disp_left_x = x2;
		    }
		  }
		  else {
		    dist += -dx;
		    if ((dist < min_right) &&
			(svs_matches[idx*5+3] < 9999)) {
		      min_right = dist;
		      disp_right = (int)svs_matches[idx*5+3];
		      disp_right_x = x2;
		    }
		  }
		}
	      }
	    }

	    if ((min_left != 99999) &&
		(min_right != 99999)) {

	      disp =
		disp_left + ((x - disp_left_x) * (disp_right - disp_left) /
			     (disp_right_x - disp_left_x));

	      if ((disp > 0) &&
		  (matches < SVS_MAX_MATCHES)) {
		curr_idx = matches * 5;

		svs_matches[curr_idx] = 500;
		svs_matches[curr_idx + 1] = x*SVS_SUB_PIXEL;
		svs_matches[curr_idx + 2] = y;
		svs_matches[curr_idx + 3] = disp;
		matches++;

		valid_quadrants[fL + L] = 1;
	      }

	    }
	  }
	}
	fL += no_of_feats;
      }
      if (prev_matches == matches)
	break;
      prev_matches = matches;
    }
  }

  return (matches);
}

/* filtering function removes noise by fitting planes to the disparities
   and disguarding outliers */
void svs::filter_plane(
		       int no_of_possible_matches, /* the number of stereo matches */
		       int max_disparity_pixels) /*maximum disparity in pixels */
{
  int i, hf, hist_max, w = SVS_FILTER_SAMPLING, w2, n, horizontal = 0;
  unsigned int x, y, disp, tx = 0, ty = 0, bx = 0, by = 0;
  int hist_thresh, hist_mean, hist_mean_hits, mass, disp2;
  int min_ww, max_ww, m, ww, d;
  int ww0, ww1, disp0, disp1, cww, dww, ddisp;
  no_of_planes = 0;

  /* clear quadrants */
  memset(valid_quadrants, 0, no_of_possible_matches * sizeof(unsigned char));

  /* default plane */
  for (i = no_of_possible_matches-1; i >= 0; i--)
    svs_matches[i * 5 + 4] = 9999;

  /* create disparity histograms within different
   * zones of the image */
  for (hf = 0; hf < 15; hf++) {

    switch (hf) {

      // overall horizontal
    case 0: {
      tx = 0;
      ty = 0;
      bx = imgWidth;
      by = imgHeight;
      horizontal = 1;
      w = bx;
      break;
    }
      // overall vertical
    case 1: {
      tx = 0;
      ty = 0;
      bx = imgWidth;
      by = imgHeight;
      horizontal = 0;
      w = by;
      break;
    }
      // left hemifield 1
    case 2: {
      tx = 0;
      ty = 0;
      bx = imgWidth / 3;
      by = imgHeight;
      horizontal = 1;
      w = bx;
      break;
    }
      // left hemifield 2
    case 3: {
      tx = 0;
      ty = 0;
      bx = imgWidth / 2;
      by = imgHeight;
      horizontal = 1;
      w = bx;
      break;
    }

      // centre field (vertical)
    case 4: {
      tx = imgWidth / 3;
      bx = imgWidth * 2 / 3;
      w = imgHeight;
      horizontal = 0;
      break;
    }
      // centre above field
    case 5: {
      tx = imgWidth / 3;
      ty = 0;
      bx = imgWidth * 2 / 3;
      by = imgHeight / 2;
      w = by;
      horizontal = 0;
      break;
    }
      // centre below field
    case 6: {
      tx = imgWidth / 3;
      ty = imgHeight / 2;
      bx = imgWidth * 2 / 3;
      by = imgHeight;
      w = ty;
      horizontal = 0;
      break;
    }
      // right hemifield 0
    case 7: {
      tx = imgWidth * 2 / 3;
      ty = 0;
      bx = imgWidth;
      by = imgHeight;
      horizontal = 1;
      break;
    }
      // right hemifield 1
    case 8: {
      tx = imgWidth / 2;
      bx = imgWidth;
      w = tx;
      break;
    }
      // upper hemifield
    case 9: {
      tx = 0;
      ty = 0;
      bx = imgWidth;
      by = imgHeight / 2;
      horizontal = 0;
      w = by;
      break;
    }
      // lower hemifield
    case 10: {
      ty = by;
      by = imgHeight;
      horizontal = 0;
      break;
    }
      // left upper
    case 11: {
      tx = 0;
      ty = 0;
      bx = imgWidth / 3;
      by = imgHeight / 2;
      horizontal = 1;
      break;
    }
      // left lower
    case 12: {
      tx = 0;
      ty = imgHeight / 2;
      bx = imgWidth / 3;
      by = imgHeight;
      horizontal = 1;
      break;
    }
      // right upper
    case 13: {
      tx = imgWidth * 2 / 3;
      ty = 0;
      bx = imgWidth;
      by = imgHeight / 2;
      horizontal = 1;
      break;
    }
      // right lower
    case 14: {
      tx = imgWidth * 2 / 3;
      ty = imgHeight / 2;
      bx = imgWidth;
      by = imgHeight;
      horizontal = 1;
      break;
    }

    }

    /* clear the histogram */
    w2 = w / SVS_FILTER_SAMPLING;
    if (w2 < 1)
      w2 = 1;
    memset((void*) disparity_histogram_plane, '\0', w2
	   * max_disparity_pixels * sizeof(int));
    memset((void*) disparity_plane_fit, '\0', w2 * sizeof(int));
    hist_max = 0;

    /* update the disparity histogram */
    n = 0;
    for (i = no_of_possible_matches-1; i >= 0; i--) {
      x = svs_matches[i * 5 + 1]/SVS_SUB_PIXEL;
      if ((x > tx) && (x < bx)) {
	y = svs_matches[i * 5 + 2];
	if ((y > ty) && (y < by)) {
	  disp = svs_matches[i * 5 + 3]/SVS_SUB_PIXEL;
	  if ((int) disp < max_disparity_pixels) {
	    if (horizontal != 0) {
	      n = (((x - tx) / SVS_FILTER_SAMPLING)
		   * max_disparity_pixels) + disp;
	    } else {
	      n = (((y - ty) / SVS_FILTER_SAMPLING)
		   * max_disparity_pixels) + disp;
	    }
	    disparity_histogram_plane[n]++;
	    if (disparity_histogram_plane[n] > hist_max)
	      hist_max = disparity_histogram_plane[n];
	  }
	}
      }
    }

    /* find peak disparities along a range of positions */
    hist_thresh = hist_max / 4;
    hist_mean = 0;
    hist_mean_hits = 0;
    disp2 = 0;
    min_ww = w2;
    max_ww = 0;
    for (ww = 0; ww < (int) w2; ww++) {
      mass = 0;
      disp2 = 0;
      for (d = 1; d < max_disparity_pixels - 1; d++) {
	n = ww * max_disparity_pixels + d;
	if (disparity_histogram_plane[n] > hist_thresh) {
	  m = disparity_histogram_plane[n]
	    + disparity_histogram_plane[n - 1]
	    + disparity_histogram_plane[n + 1];
	  mass += m;
	  disp2 += m * d;
	}
	if (disparity_histogram_plane[n] > 0) {
	  hist_mean += disparity_histogram_plane[n];
	  hist_mean_hits++;
	}
      }
      if (mass > 0) {
	// peak disparity at this position
	disparity_plane_fit[ww] = disp2 / mass;
	if (min_ww == (int) w2)
	  min_ww = ww;
	if (ww > max_ww)
	  max_ww = ww;
      }
    }
    if (hist_mean_hits > 0)
      hist_mean /= hist_mean_hits;

    /* fit a line to the disparity values */
    ww0 = 0;
    ww1 = 0;
    disp0 = 0;
    disp1 = 0;
    int hits0,hits1;
    if (max_ww >= min_ww) {
      cww = min_ww + ((max_ww - min_ww) / 2);
      hits0 = 0;
      hits1 = 0;
      for (ww = min_ww; ww <= max_ww; ww++) {
	if (ww < cww) {
	  disp0 += disparity_plane_fit[ww];
	  ww0 += ww;
	  hits0++;
	} else {
	  disp1 += disparity_plane_fit[ww];
	  ww1 += ww;
	  hits1++;
	}
      }
      if (hits0 > 0) {
	disp0 /= hits0;
	ww0 /= hits0;
      }
      if (hits1 > 0) {
	disp1 /= hits1;
	ww1 /= hits1;
      }
    }
    dww = ww1 - ww0;
    ddisp = disp1 - disp0;

    /* find inliers */
    int plane_tx = imgWidth;
    int plane_ty = 0;
    int plane_bx = imgHeight;
    int plane_by = 0;
    int plane_disp0 = 0;
    int plane_disp1 = 0;
    int hits = 0;
    for (i = no_of_possible_matches-1; i >= 0; i--) {
      if (svs_matches[i * 5 + 4] == 9999) {
	x = svs_matches[i * 5 + 1]/SVS_SUB_PIXEL;
	if ((x > tx) && (x < bx)) {
	  y = svs_matches[i * 5 + 2];
	  if ((y > ty) && (y < by)) {
	    disp = svs_matches[i * 5 + 3]/SVS_SUB_PIXEL;

	    if (horizontal != 0) {
	      ww = (x - tx) / SVS_FILTER_SAMPLING;
	      n = ww * max_disparity_pixels + disp;
	    } else {
	      ww = (y - ty) / SVS_FILTER_SAMPLING;
	      n = ww * max_disparity_pixels + disp;
	    }

	    if (dww > 0) {
	      disp2 = disp0 + ((ww - ww0) * ddisp / dww);
	    }
	    else {
	      disp2 = disp0;
	    }

	    /* check how far this is from the plane */
	    if (((int)disp >= disp2-1) &&
		((int)disp <= disp2+1) &&
		((int)disp < max_disparity_pixels)) {

	      /* inlier detected - this disparity lies along the plane */
	      valid_quadrants[i]++;
	      svs_matches[i * 5 + 4] = no_of_planes;
	      hits++;

	      /* keep note of the bounds of the plane */
	      if (x < (unsigned int)plane_tx) {
		plane_tx = x;
		if (horizontal == 1) plane_disp0 = disp2;
	      }
	      if (y < (unsigned int)plane_ty) {
		plane_ty = y;
		if (horizontal == 0) plane_disp0 = disp2;
	      }
	      if (x > (unsigned int)plane_bx) {
		plane_bx = x;
		if (horizontal == 1) plane_disp1 = disp2;
	      }
	      if (y > (unsigned int)plane_by) {
		plane_by = y;
		if (horizontal == 0) plane_disp1 = disp2;
	      }
	    }
	  }
	}
      }
    }
    if (hits > 5) {
      /* add a detected plane */
      plane[no_of_planes*9+0]=plane_tx;
      plane[no_of_planes*9+1]=plane_ty;
      plane[no_of_planes*9+2]=plane_bx;
      plane[no_of_planes*9+3]=plane_by;
      plane[no_of_planes*9+4]=horizontal;
      plane[no_of_planes*9+5]=plane_disp0;
      plane[no_of_planes*9+6]=plane_disp1;
      plane[no_of_planes*9+7]=hits;
      plane[no_of_planes*9+8]=0;
      no_of_planes++;
    }
  }

  /* remove outliers */
  for (i = no_of_possible_matches-1; i >= 0; i--) {
    if (valid_quadrants[i] == 0) {
      svs_matches[i * 5] = 0;
    }
  }
}

/* calculate offsets assuming that the cameras are looking at some distant object */
void svs::calibrate_offsets(unsigned char* left_image, /* left image data */
			    unsigned char* right_image, /* right image data */
			    int x_range, /* horizontal search range in pixels */
			    int y_range, /* vertical search range in pixels */
			    int& calibration_offset_x, /* returned horizontal offset in pixels */
			    int& calibration_offset_y) { /* returned vertical offset in pixels */

  int tx = imgWidth * 25 / 100;
  int ty = imgHeight * 25 / 100;
  int bx = imgWidth - tx;
  int by = imgHeight - ty;

  int min_diff = (bx - tx) * (by - ty) * 3* 255 ;
  for (int offset_y = -y_range; offset_y < y_range; offset_y++) {
    for (int offset_x = -x_range; offset_x < x_range; offset_x++) {

      int diff = 0;
      for (int y = ty; y < by; y++) {
	int n = ((y * imgWidth) + tx) * 3;
	int n2 = (((y + offset_y) * imgWidth) + (tx + offset_x)) * 3;
	for (int x = tx; x < bx; x++, n += 3, n2 += 3) {

	  for (int col = 0; col < 3; col++) {
	    int v = left_image[n + col] - right_image[n2 + col];
	    if (v < 0)
	      v = -v;
	    diff += v;
	  }

	}
      }
      if (diff < min_diff) {
	min_diff = diff;
	calibration_offset_x = offset_x;
	calibration_offset_y = offset_y;
      }
    }
  }
}

/* creates a calibration map */
void svs::make_map(float centre_of_distortion_x, /* centre of distortion x coordinate in pixels */
		   float centre_of_distortion_y, /* centre of distortion y coordinate in pixels */
		   float coeff_0, /* lens distortion polynomial coefficient 0 */
		   float coeff_1, /* lens distortion polynomial coefficient 1 */
		   float coeff_2, /* lens distortion polynomial coefficient 2 */
		   float rotation, /* camera rotation (roll angle) in radians */
		   float scale) { /* scaling applied */

  /* free existing map */
  if (calibration_map != NULL)
    free(calibration_map);

  polynomial* distortion_curve = new polynomial();
  distortion_curve->SetDegree(3);
  distortion_curve->SetCoeff(0, 0);
  distortion_curve->SetCoeff(1, coeff_0);
  distortion_curve->SetCoeff(2, coeff_1);
  distortion_curve->SetCoeff(3, coeff_2);

  int half_width = imgWidth / 2;
  int half_height = imgHeight / 2;
  calibration_map = new int[imgWidth * imgHeight];
  for (int x = 0; x < (int) imgWidth; x++) {

    float dx = x - centre_of_distortion_x;

    for (int y = 0; y < (int) imgHeight; y++) {

      float dy = y - centre_of_distortion_y;

      float radial_dist_rectified = (float) sqrt(dx * dx + dy * dy);
      if (radial_dist_rectified >= 0.01f) {

	double radial_dist_original = distortion_curve->RegVal(
							       radial_dist_rectified);
	if (radial_dist_original > 0) {

	  double ratio = radial_dist_original / radial_dist_rectified;
	  float x2 = (float) round(centre_of_distortion_x + (dx
							     * ratio));
	  x2 = (x2 - (imgWidth / 2)) * scale;
	  float y2 = (float) round(centre_of_distortion_y + (dy
							     * ratio));
	  y2 = (y2 - (imgHeight / 2)) * scale;

	  // apply rotation
	  double x3 = x2, y3 = y2;
	  double hyp;
	  if (rotation != 0) {
	    hyp = sqrt(x2 * x2 + y2 * y2);
	    if (hyp > 0) {
	      double rot_angle = acos(y2 / hyp);
	      if (x2 < 0)
		rot_angle = (3.1415927 * 2) - rot_angle;
	      double new_angle = rotation + rot_angle;
	      x3 = hyp * sin(new_angle);
	      y3 = hyp * cos(new_angle);
	    }
	  }

	  x3 += half_width;
	  y3 += half_height;

	  if (((int) x3 > -1) && ((int) x3 < (int) imgWidth)
	      && ((int) y3 > -1) && ((int) y3 < (int) imgHeight)) {

	    int n = (y * imgWidth) + x;
	    int n2 = ((int) y3 * imgWidth) + (int) x3;

	    calibration_map[n] = n2;
	  }
	}
      }
    }
  }
  delete distortion_curve;
}

/* takes the raw image and camera calibration parameters and returns a rectified image */
void svs::rectify(unsigned char* raw_image, /* raw image grabbed from camera */
		  unsigned char* rectified_frame_buf) { /* returned rectified image */

  int max, n, i, idx;

  if (calibration_map != NULL) {

    n = 0;
    max = imgWidth * imgHeight * 3;
    for (i = 0; i < max; i += 3, n++) {
      idx = calibration_map[n] * 3;
      rectified_frame_buf[i] = raw_image[idx];
      rectified_frame_buf[i + 1] = raw_image[idx + 1];
      rectified_frame_buf[i + 2] = raw_image[idx + 2];
    }
  }
}

/* takes the raw image and camera calibration parameters and returns a rectified image */
void svs::make_map_int(long centre_of_distortion_x, /* centre of distortion x coordinate in pixels xSVS_MULT */
		       long centre_of_distortion_y, /* centre of distortion y coordinate in pixels xSVS_MULT */
		       long* coeff, /* three lens distortion polynomial coefficients xSVS_MULT_COEFF */
		       long scale_num, /* scaling numerator */
		       long scale_denom) { /* scaling denominator */

  const long SVS_MULT = 1;
  const long SVS_MULT_COEFF = 10000000;

  long v, powr, radial_dist_rectified, radial_dist_original;
  long i, j, x, y, dx, dy;
  long n, n2, x2, y2;
  long ww = imgWidth;
  long hh = imgHeight;
  long half_width = ww / 2;
  long half_height = hh / 2;
  scale_denom *= SVS_MULT;
  calibration_map = new int[imgWidth * imgHeight];
  for (x = 0; x < ww; x++) {

    dx = (x * SVS_MULT) - centre_of_distortion_x;

    for (y = 0; y < hh; y++) {

      dy = (y * SVS_MULT) - centre_of_distortion_y;

      v = dx * dx + dy * dy;
      /* integer square root */
      for (radial_dist_rectified = 0; v >= (2* radial_dist_rectified )
	     + 1; v -= (2* radial_dist_rectified ++) + 1)
	;

      //float radial_dist_rectified2 = (float) sqrt(dx * dx + dy * dy);

      //printf("radial_dist_rectified %ld %f\n", radial_dist_rectified, radial_dist_rectified2);

      if (radial_dist_rectified >= 0) {

	/* for each polynomial coefficient */
	radial_dist_original = 0;
	double radial_dist_original2 = 0;
	for (i = 0; i < 4; i++) {

	  /* pow(radial_dist_rectified, i) */
	  powr = 1;
	  if (i != 0) {
	    j = i;
	    while (j != 0) {
	      powr *= radial_dist_rectified;
	      j--;
	    }
	  } else {
	    powr = 1;
	  }

	  //printf("powr %ld %lf\n", powr, pow(radial_dist_rectified, i));

	  /*
	    if (coeff[i] >= 0) {
	    radial_dist_original += (unsigned long)coeff[i] * powr;
	    }
	    else {
	    radial_dist_original -= (unsigned long)(-coeff[i]) * powr;
	    }
	  */
	  radial_dist_original += coeff[i] * powr;
	  radial_dist_original2 += coeff[i] * pow(
						  radial_dist_rectified, i);

	  //printf("powr %ld %lf\n", coeff[i] * powr, (double)coeff[i] * pow(radial_dist_rectified, i));
	}
	//printf("radial_dist_original %ld %lf\n", radial_dist_original, radial_dist_original2);

	if (radial_dist_original > 0) {

	  radial_dist_original /= SVS_MULT_COEFF;
	  radial_dist_original2 /= SVS_MULT_COEFF;
	  //printf("radial_dist_original %ld %lf\n", radial_dist_original, radial_dist_original2);

	  //printf("radial_dist_rectified: %d\n", (int)(radial_dist_rectified/SVS_MULT));
	  //printf("radial_dist_original:  %d\n", (int)(radial_dist_original/SVS_MULT));

	  x2 = centre_of_distortion_x + (dx * radial_dist_original
					 / radial_dist_rectified);
	  x2 = (x2 - (half_width * SVS_MULT)) * scale_num
	    / scale_denom;
	  y2 = centre_of_distortion_y + (dy * radial_dist_original
					 / radial_dist_rectified);
	  y2 = (y2 - (half_height * SVS_MULT)) * scale_num
	    / scale_denom;

	  x2 += half_width;
	  y2 += half_height;

	  if ((x2 > -1) && (x2 < ww) && (y2 > -1) && (y2 < hh)) {

	    n = y * imgWidth + x;
	    n2 = y2 * imgWidth + x2;

	    //int diff = calibration_map[(int) n] - (int) n2;
	    //if (diff < 0)
	    //	diff = -diff;
	    //printf("diff: %d\n", diff);

	    calibration_map[(int) n] = (int) n2;
	  }
	}
      }
    }
  }
}

/* saves stereo matches to file for use by other programs */
void svs::save_matches(std::string filename, /* filename to save as */
		       unsigned char* rectified_frame_buf, /* left image data */
		       int no_of_matches, /* number of stereo matches */
		       bool colour) { /* whether to additionally save colour of each match */

  FILE *file = fopen(filename.c_str(), "wb");
  if (file != NULL) {

    if (!colour) {

      struct MatchData {
	unsigned short int probability;
	unsigned short int x;
	unsigned short int y;
	unsigned short int disparity;
      };

      MatchData *m = new MatchData[no_of_matches];
      for (int i = 0; i < no_of_matches; i++) {
	m[i].probability = (unsigned short int)svs_matches[i * 5];
	m[i].x = (unsigned short int)svs_matches[i * 5 + 1];
	m[i].y = (unsigned short int)svs_matches[i * 5 + 2];
	m[i].disparity = (unsigned short int)svs_matches[i * 5 + 3];
      }

      fwrite(m, sizeof(MatchData), no_of_matches, file);
      delete[] m;
    } else {
      struct MatchDataColour {
	unsigned short int probability;
	unsigned short int x;
	unsigned short int y;
	unsigned short int disparity;
	unsigned char r, g, b;
	unsigned char pack;
      };

      int n;
      MatchDataColour *m = new MatchDataColour[no_of_matches];
      for (int i = 0; i < no_of_matches; i++) {
	m[i].probability = (unsigned short int)svs_matches[i * 5];
	m[i].x = (unsigned short int)svs_matches[i * 5 + 1];
	m[i].y = (unsigned short int)svs_matches[i * 5 + 2];
	m[i].disparity = (unsigned short int)svs_matches[i * 5 + 3];
	n = ((m[i].y * imgWidth) + m[i].x) * 3;
	m[i].r = rectified_frame_buf[n + 2];
	m[i].g = rectified_frame_buf[n + 1];
	m[i].b = rectified_frame_buf[n];
      }

      fwrite(m, sizeof(MatchDataColour), no_of_matches, file);
      delete[] m;
    }

    fclose(file);
  }
}

/*!
 * \brief returns true if the given file exists
 * \param filename name of the file
 */
bool svs::FileExists(
		     std::string filename)
{
  std::ifstream inf;

  bool flag = false;
  inf.open(filename.c_str());
  if (inf.good()) flag = true;
  inf.close();
  return(flag);
}

/* logs stereo matches to file for use by other programs */
bool svs::log_matches(std::string filename, /* filename to save as */
		      unsigned char* rectified_frame_buf, /* left image data */
		      int no_of_matches, /* number of stereo matches */
		      bool colour) { /* whether to additionally save colour of each match */

  bool logged = false;

  if (!FileExists(filename)) {
    FILE *file = fopen(filename.c_str(), "wb");
    if (file != NULL) {

      if (!colour) {

	struct MatchData {
	  unsigned short int probability;
	  unsigned short int x;
	  unsigned short int y;
	  unsigned short int disparity;
	};

	MatchData *m = new MatchData[no_of_matches];
	for (int i = 0; i < no_of_matches; i++) {
	  m[i].probability = (unsigned short int)svs_matches[i * 5];
	  m[i].x = (unsigned short int)svs_matches[i * 5 + 1];
	  m[i].y = (unsigned short int)svs_matches[i * 5 + 2];
	  m[i].disparity = (unsigned short int)svs_matches[i * 5 + 3];
	}

	fwrite(m, sizeof(MatchData), no_of_matches, file);
	delete[] m;
      } else {
	struct MatchDataColour {
	  unsigned short int probability;
	  unsigned short int x;
	  unsigned short int y;
	  unsigned short int disparity;
	  unsigned char r, g, b;
	  unsigned char pack;
	};

	int n;
	MatchDataColour *m = new MatchDataColour[no_of_matches];
	for (int i = 0; i < no_of_matches; i++) {
	  m[i].probability = (unsigned short int)svs_matches[i * 5];
	  m[i].x = (unsigned short int)svs_matches[i * 5 + 1];
	  m[i].y = (unsigned short int)svs_matches[i * 5 + 2];
	  m[i].disparity = (unsigned short int)svs_matches[i * 5 + 3];
	  n = ((m[i].y * imgWidth) + m[i].x) * 3;
	  m[i].r = rectified_frame_buf[n + 2];
	  m[i].g = rectified_frame_buf[n + 1];
	  m[i].b = rectified_frame_buf[n];
	}

	fwrite(m, sizeof(MatchDataColour), no_of_matches, file);
	delete[] m;
      }

      fclose(file);
      logged = true;
    }
  }
  return(logged);
}


/* experimental plane fitting */
int svs::fit_plane(int no_of_matches, int max_deviation, int no_of_samples) {
  /*
  int x_x0 = 0, x_y0 = 0, x_x1 = 0, x_y1 = 0;
  int y_x0 = 0, y_y0 = 0, y_x1 = 0, y_y1 = 0;
  */
  int max_hits = 0;
  int idx0, idx1, xx0, yy0, xx1, yy1, dx, dy, abs_dx, abs_dy, hits;
  int grad_x, grad_y, index0, index1, sample, horizontal, deviation_sum;
  int best_index0, min_deviation, min_deviation_hits = 0,
    edge_sample;
  int axis, edge_x, edge_y, deviation, predicted_edge_x, predicted_edge_y;

  if (no_of_matches > 40) {
    /* fit to x and y axes */
    for (axis = 0; axis < 2; axis++) {
      min_deviation = 999999;
      best_index0 = -1;

      /* try a number of baselines */
      for (sample = 0; sample < no_of_samples; sample++) {
	// pick the baseline
	index0 = rand() % no_of_matches;
	index1 = rand() % no_of_matches;
	if (index0 != index1) {
	  hits = 0;
	  deviation_sum = 0;
	  idx0 = index0 * 5;
	  idx1 = index1 * 5;
	  if (axis == 0) {
	    /* oriented along the x axis */
	    xx0 = svs_matches[idx0 + 1]/SVS_SUB_PIXEL;
	    xx1 = svs_matches[idx1 + 1]/SVS_SUB_PIXEL;
	  } else {
	    /* oriented along the y axis */
	    xx0 = svs_matches[idx0 + 2];
	    xx1 = svs_matches[idx1 + 2];
	  }
	  yy0 = svs_matches[idx0 + 3];
	  yy1 = svs_matches[idx1 + 3];
	  dx = xx1 - xx0;
	  dy = yy1 - yy0;
	  if (dx >= 0)
	    abs_dx = dx;
	  else
	    abs_dx = -dx;
	  if (dy >= 0)
	    abs_dy = dy;
	  else
	    abs_dy = -dy;

	  // is the baseline horizontally oriented ?
	  horizontal = 1;
	  if (abs_dy > abs_dx) {
	    horizontal = 0;
	    grad_x = dx;
	    grad_y = dy;
	  } else {
	    grad_x = dy;
	    grad_y = dx;
	  }

	  if (grad_y != 0) {
	    for (edge_sample = 0; edge_sample < no_of_matches; edge_sample
		   += 2) {
	      edge_x = svs_matches[edge_sample * 5 + 1]/SVS_SUB_PIXEL;
	      edge_y = svs_matches[edge_sample * 5 + 2];
	      deviation = 0;

	      if (horizontal == 1) {
		predicted_edge_y = yy0 + ((edge_x - xx0)
					  * grad_x / grad_y);
		deviation = predicted_edge_y - edge_y;
	      } else {
		predicted_edge_x = xx0 + ((edge_y - yy0)
					  * grad_x / grad_y);
		deviation = predicted_edge_x - edge_x;
	      }

	      if ((deviation > -max_deviation) && (deviation
						   < max_deviation)) {
		hits++;
		if (deviation < 0)
		  deviation = -deviation;
		deviation_sum += deviation;
	      }
	    }

	    if (hits > 0) {
	      // pick the line with the maximum number of edges within the max deviation range
	      if (hits > max_hits) {
		best_index0 = index0;
		max_hits = hits;
		min_deviation = deviation_sum;
	      } else {
		// if there is a tie choose the result with the lowest deviation
		if (hits == max_hits) {
		  if (deviation_sum < min_deviation) {
		    best_index0 = index0;
		    min_deviation = deviation_sum;
		    min_deviation_hits = hits;
		  }
		}
	      }
	    }
	  }
	}
      }

      if (min_deviation_hits > 3) {

	min_deviation /= min_deviation_hits;

	if (best_index0 > -1) {
	  /*
	  if (axis == 0) {
	    x_x0 = svs_matches[best_index0 + 1];
	    x_y0 = svs_matches[best_index0 + 3];
	    x_x1 = svs_matches[best_index1 + 1];
	    x_y1 = svs_matches[best_index1 + 3];
	  } else {
	    y_x0 = svs_matches[best_index0 + 2];
	    y_y0 = svs_matches[best_index0 + 3];
	    y_x1 = svs_matches[best_index1 + 2];
	    y_y1 = svs_matches[best_index1 + 3];
	  }
	  */
	  printf("min deviation %d\n", min_deviation);
	} else {
	  break;
	}
      } else {
	break;
      }
    }
  }

  return (max_hits);
}

/* flip the given image so that the camera can be mounted upside down */
void svs::flip(unsigned char* raw_image, unsigned char* flipped_frame_buf) {
  int max = imgWidth * imgHeight * 3;
  for (int i = 0; i < max; i += 3) {
    flipped_frame_buf[i] = raw_image[(max - 3 - i)];
    flipped_frame_buf[i + 1] = raw_image[(max - 3 - i + 1)];
    flipped_frame_buf[i + 2] = raw_image[(max - 3 - i + 2)];
  }
  memcpy(raw_image, flipped_frame_buf, max * sizeof(unsigned char));
}

void svs::segment(unsigned char* rectified_frame_buf, int no_of_matches) {
  int x, y, n2, n, n3, ctr = 0, max_x, max_y, i, j;
  int stride = imgWidth * SVS_VERTICAL_SAMPLING;
  unsigned short ID = 0, next_ID = 0, v;
  int min_length = (int) imgWidth / 50;
  unsigned short curr_ID[50];
  unsigned short curr_ID_hits[50];
  int dx, dy, min_dist, min_vol, prev_region_history_index, curr_ID_index = 0;
  int r, g, b, tx, ty, bx, by, cx, cy, disp, best_disp, max_hits = 0;
  int above, below, left, right, vol;
  int above_hits, below_hits, left_hits, right_hits, state;
  no_of_regions = 0;
  if (enable_segmentation) {
    next_ID = 1;
    max_x = (int) imgWidth - 4;
    max_y = (int) imgHeight - (SVS_VERTICAL_SAMPLING * 5);
    for (y = 4; y < max_y; y += SVS_VERTICAL_SAMPLING) {
      n = y * imgWidth + 4;
      state = 0;
      for (x = 4; x < max_x; x++, n++, ctr++) {
	v = low_contrast[n];
	if (v != 0) {
	  if (state == 0) {
	    if (low_contrast[n - 1] == 0) {
	      ctr = 0;
	      curr_ID_index = 0;
	      state = 1;
	    }
	  } else {
	    if (v != 65535) {
	      for (i = 0; i < curr_ID_index; i++) {
		if (curr_ID[i] == v) {
		  curr_ID_hits[i]++;
		  break;
		}
	      }
	      if (i == curr_ID_index) {
		curr_ID[curr_ID_index] = v;
		curr_ID_hits[curr_ID_index] = 1;
		curr_ID_index++;
		if (curr_ID_index > 49)
		  curr_ID_index = 49;
	      }
	    }
	    if (low_contrast[n + 1] == 0) {
	      if (ctr > min_length) {
		if (curr_ID_index == 0) {
		  ID = next_ID;
		  region_volume[ID] = 0;
		  region_centre[ID * 2] = 0;
		  region_centre[ID * 2 + 1] = 0;
		  region_bounding_box[ID * 4] = imgWidth;
		  region_bounding_box[ID * 4 + 1] = imgHeight;
		  region_bounding_box[ID * 4 + 2] = 0;
		  region_bounding_box[ID * 4 + 3] = 0;
		  region_colour[ID * 3] = 0;
		  region_colour[ID * 3 + 1] = 0;
		  region_colour[ID * 3 + 2] = 0;
		  if (next_ID < SVS_MAX_REGIONS - 1)
		    next_ID++;
		} else {
		  max_hits = -1;
		  for (i = 0; i < curr_ID_index; i++) {
		    if (curr_ID_hits[i] > max_hits) {
		      max_hits = curr_ID_hits[i];
		      ID = curr_ID[i];
		    }
		  }
		}

		/* update volume and bounding box */
		region_volume[ID] += (unsigned short) ctr;
		n2 = ID * 4;
		if (x - ctr < region_bounding_box[n2])
		  region_bounding_box[n2]
		    = (unsigned short) x - ctr;
		if (y < region_bounding_box[n2 + 1])
		  region_bounding_box[n2 + 1]
		    = (unsigned short) y;
		if (x > region_bounding_box[n2 + 2])
		  region_bounding_box[n2 + 2]
		    = (unsigned short) x;
		if (y > region_bounding_box[n2 + 3])
		  region_bounding_box[n2 + 3]
		    = (unsigned short) y;

		r = 0;
		g = 0;
		b = 0;
		cx = 0;
		cy = 0;
		int bord = 0, border = 2;
		for (n2 = n - ctr; n2 <= n; n2++, bord++) {
		  if ((bord > border)
		      && (bord < ctr - border)) {
		    low_contrast[n2] = ID;
		    cx += x + n2 - n;
		    cy += y;
		    n3 = n2 * 3;
		    b += rectified_frame_buf[n3++];
		    g += rectified_frame_buf[n3++];
		    r += rectified_frame_buf[n3++];
		    if (low_contrast[n2 + stride] == 65535) {
		      low_contrast[n2 + stride] = ID;
		    }
		  } else {
		    low_contrast[n2] = 0;
		  }

		}
		region_centre[ID * 2] += cx;
		region_centre[ID * 2 + 1] += cy;
		region_colour[ID * 3] += b;
		region_colour[ID * 3 + 1] += g;
		region_colour[ID * 3 + 2] += r;
	      } else {
		for (n2 = n - ctr; n2 <= n; n2++) {
		  low_contrast[n2] = 0;
		}
	      }
	      state = 0;
	    }
	  }
	}

      }
    }
    if (next_ID > 1) {

      region_history_index++;
      if (region_history_index >= SVS_REGION_HISTORY)
	region_history_index = 0;

      min_vol = imgWidth * imgHeight * 1 / 500;
      n = 0;
      no_of_regions = next_ID;
      for (i = 0; i < no_of_regions; i++) {
	vol = region_volume[i];
	if (vol != 0) {
	  region_centre[i * 2] /= vol;
	  region_centre[i * 2 + 1] /= vol;
	  if (vol > min_vol) {
	    prev_region_centre[region_history_index][n * 4 + 1]
	      = region_centre[i * 2];
	    prev_region_centre[region_history_index][n * 4 + 2]
	      = region_centre[i * 2 + 1];
	    prev_region_centre[region_history_index][n * 4 + 3]
	      = 65535;
	    prev_region_centre[region_history_index][n * 4 + 4] = i;
	    n++;
	  }
	  region_colour[i * 3] /= vol;
	  region_colour[i * 3 + 1] /= vol;
	  region_colour[i * 3 + 2] /= vol;
	}
      }
      prev_region_centre[region_history_index][0] = n;

      /* compute depth of regions */
      for (i = 0; i < n; i++) {
	j = (int) prev_region_centre[region_history_index][i * 4 + 4];
	tx = region_bounding_box[j * 4];
	ty = region_bounding_box[j * 4 + 1] - 20;
	bx = region_bounding_box[j * 4 + 2];
	by = region_bounding_box[j * 4 + 3] + 20;
	cx = tx + ((bx - tx) / 2);
	cy = ty + ((by - ty) / 2);
	if (max_hits > 0)
	  memset((void*) disparity_histogram_plane, '\0',
		 (SVS_MAX_IMAGE_WIDTH / 2) * sizeof(int));
	max_hits = 0;
	best_disp = 255;
	above = 0;
	below = 0;
	left = 0;
	right = 0;
	above_hits = 0;
	below_hits = 0;
	left_hits = 0;
	right_hits = 0;
	for (n2 = 0; n2 < no_of_matches; n2++) {
	  if (svs_matches[n2 * 5] > 0) {
	    x = svs_matches[n2 * 5 + 1]/SVS_SUB_PIXEL;
	    if ((x > tx) && (x < bx)) {
	      y = svs_matches[n2 * 5 + 2];
	      if ((y > ty) && (y < by)) {
		disp = svs_matches[n2 * 5 + 3]/SVS_SUB_PIXEL;
		if (y < cy) {
		  above += disp;
		  above_hits++;
		} else {
		  below += disp;
		  below_hits++;
		}
		if (x < cx) {
		  left += disp;
		  left_hits++;
		} else {
		  right += disp;
		  right_hits++;
		}
		disparity_histogram_plane[disp]++;
		if (disparity_histogram_plane[disp] > max_hits) {
		  max_hits = disparity_histogram_plane[disp];
		  best_disp = disp;
		}
	      }
	    }
	  }
	}
	region_disparity[j * 3] = (unsigned char) best_disp;
	region_disparity[j * 3 + 1] = 127;
	region_disparity[j * 3 + 2] = 127;
	if ((left_hits > 0) && (right_hits > 0)) {
	  left /= left_hits;
	  right /= right_hits;
	  int b = right - left;
	  if ((b < 20) && (b > -20))
	    region_disparity[j * 3 + 1] = 127 + b;

	  //printf("slope: %d\n", left - right);
	}
	if ((above_hits > 0) && (below_hits > 0)) {
	  above /= above_hits;
	  below /= below_hits;
	  b = below - above;
	  if ((b < 20) && (b > -20))
	    region_disparity[j * 3 + 2] = 127 + b;
	  //printf("slope: %d\n", above - below);
	}
      }

      /* track regions */
      if (enable_region_tracking != 0) {
	min_dist = imgWidth * 4 / 100;
	for (i = 0; i < n; i++) {
	  x = (int) prev_region_centre[region_history_index][i * 4
							     + 1];
	  y = (int) prev_region_centre[region_history_index][i * 4
							     + 2];
	  prev_region_history_index = region_history_index - 1;
	  if (prev_region_history_index < 0)
	    prev_region_history_index += SVS_REGION_HISTORY;
	  ctr = 0;
	  while (ctr < SVS_REGION_HISTORY * 3 / 4) {
	    n2 = prev_region_centre[prev_region_history_index][0];
	    for (j = 0; j < n2; j++) {
	      dx
		= (int) prev_region_centre[prev_region_history_index][j
								      * 4 + 1] - x;
	      if ((dx > -min_dist) && (dx < min_dist)) {
		dy
		  = (int) prev_region_centre[prev_region_history_index][j
									* 4 + 2] - y;
		if ((dy > -min_dist) && (dy < min_dist)) {
		  prev_region_centre[region_history_index][i
							   * 4 + 3]
		    = prev_region_history_index;
		  prev_region_centre[region_history_index][i
							   * 4 + 4] = j;
		  ctr = 9999;
		  j = n2;
		}
	      }
	    }
	    prev_region_history_index--;
	    if (prev_region_history_index < 0)
	      prev_region_history_index += SVS_REGION_HISTORY;
	    ctr++;
	  }
	}
      }
    }
  }
}

