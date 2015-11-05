#include <qx_csbp.h>
#include <sys/time.h>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>

#include <vg_ram.h>

#include "elas.h"

static int max_disparity_rescale;
static double lambda = 1024;

static int stereo_width;
static int stereo_height;
static int max_disparity;
static int stereo_scalew;
static int stereo_scaleh;
static char *algorithm;
static float gaussian_radius;
static int synapses;
static int wintakeiteration;
static int numdisparity;
static int numthreads;
static int inc_width;
static int inc_height;

static int vertical_ROI_ini;
static int vertical_ROI_end;
static int horizontal_ROI_ini;
static int horizontal_ROI_end;

static int ROI_width;
static int ROI_height;

static int camera;

static int bumblebee_basic_width;
static int bumblebee_basic_height;

IplImage *image_left_copy = NULL;
IplImage *image_left_rescale;
unsigned char *raw_image_left = NULL;
unsigned char *filtered_image_left = NULL;

IplImage *image_right_copy = NULL;
IplImage *image_right_rescale = NULL;
unsigned char *raw_image_right = NULL;
unsigned char *filtered_image_right = NULL;

IplImage *depth_image = NULL;
IplImage *depth_image_rescale = NULL;
unsigned char *raw_disparity_rescale = NULL;


IplConvKernel *kernel = NULL;

static float *left_disparity = NULL;
static float *right_disparity = NULL;
static unsigned char *gray_left_roi = NULL;
static unsigned char *gray_right_roi = NULL;

void
copy_RGB_image_to_BGR_image(unsigned char *original, IplImage *copy, int nchannels)
{
  int i, j;

  for(i = 0; i < copy->height; i++)
  {
    unsigned char* data = (unsigned char*)copy->imageData + (i*copy->widthStep);
    if(nchannels==3)
      for(j = 0; j < copy->width; j++)
      {
        data[nchannels*j+2] = original[nchannels*(i*copy->width+j)+0];
        data[nchannels*j+1] = original[nchannels*(i*copy->width+j)+1];
        data[nchannels*j+0] = original[nchannels*(i*copy->width+j)+2];
      }
    else
      for(j = 0; j < copy->width; j++)
      {
        data[j] = original[i*copy->width+j];
      }
  }
}


void
copy_BGR_image_to_RGB_image(IplImage *original, unsigned char *copy, int nchannels)
{
  int i, j;

  for (i = 0; i < original->height; i++)
  {
    unsigned char *data = (unsigned char *)original->imageData + (i * original->widthStep);
    if(nchannels==3)
    {
      for(j = 0; j < original->width; j++)
      {
        copy[nchannels*(i*original->width+j)+2] = data[nchannels*j+0];
        copy[nchannels*(i*original->width+j)+1] = data[nchannels*j+1];
        copy[nchannels*(i*original->width+j)+0] = data[nchannels*j+2];
      }
    }
    else
    {
      for(j = 0; j < original->width; j++)
      {
        copy[i*original->width+j] = data[j];
      }
    }
  }
}


void copy_raw_image_to_opencv_image(unsigned char *original, IplImage *copy,
    int nchannels) {
  int i, j;

  for (i = 0; i < copy->height; i++) {
    unsigned char* data = (unsigned char*) copy->imageData + (i
        * copy->widthStep);
    if (nchannels == 3)
      for (j = 0; j < copy->width; j++) {
        data[nchannels * j] = original[nchannels
                                       * (i * copy->width + j)];
        data[nchannels * j + 1] = original[nchannels * (i * copy->width
            + j) + 1];
        data[nchannels * j + 2] = original[nchannels * (i * copy->width
            + j) + 2];
      }
    else
      for (j = 0; j < copy->width; j++) {
        data[j] = original[i * copy->width + j];
      }
  }
}

void copy_opencv_image_to_raw_image(IplImage *original, unsigned char *copy,
    int nchannels) {
  int i, j;

  for (i = 0; i < original->height; i++) {
    unsigned char *data = (unsigned char *) original->imageData + (i
        * original->widthStep);
    if (nchannels == 3) {
      for (j = 0; j < original->width; j++) {
        copy[nchannels * (i * original->width + j)] = data[nchannels
                                                           * j];
        copy[nchannels * (i * original->width + j) + 1]
             = data[nchannels * j + 1];
        copy[nchannels * (i * original->width + j) + 2]
             = data[nchannels * j + 2];
      }
    } else {
      for (j = 0; j < original->width; j++) {
        copy[i * original->width + j] = data[j];
      }
    }
  }
}

void adjust_disparity_scale_from_opencv_image(IplImage *original,
    float *adjusted_disparity) {
  int i, j;
  double scale;

  scale = 1.0f;//(double) bumblebee_basic_width / (double) stereo_scalew;
  for (i = 0; i < original->height; i++) {
    unsigned char *data = (unsigned char *) original->imageData + (i
        * original->widthStep);
    for (j = horizontal_ROI_ini; j < horizontal_ROI_end; j++) {
      adjusted_disparity[i * original->width + j] = data[j] * scale;
    }
  }
}

void convert_disparity_from_raw_image_to_float(unsigned char *raw_image,
    float *disparity) {
  int i = vertical_ROI_ini, j = 0, x = 0, y = 0;

  for (y = vertical_ROI_ini; i < vertical_ROI_end; y++, i ++) {
    for (j = horizontal_ROI_ini, x = horizontal_ROI_ini; j
    < horizontal_ROI_end; x++, j ++) {
      disparity[y * stereo_width + x] = ((float) raw_image[i * bumblebee_basic_width + j]);
    }
  }
}

void rescale_image(unsigned char *raw_image) {
  int i = 0, j = 0, x = 0, y = 0;

  for (y = 0; i < bumblebee_basic_height; y++, i += inc_height) {
    for (j = horizontal_ROI_ini, x = horizontal_ROI_ini; j
    < horizontal_ROI_end; x++, j += inc_width) {
      depth_image_rescale->imageData[y * depth_image_rescale->widthStep + x] = (raw_image[i * bumblebee_basic_width + j]);
    }
  }


  cvErode(depth_image_rescale, depth_image_rescale, kernel, 1);
  cvDilate(depth_image_rescale, depth_image_rescale, kernel, 1);

  //cvSmooth(depth_image_rescale, depth_image_rescale, CV_GAUSSIAN, 5, 0, 0, 0);


  cvResize(depth_image_rescale, depth_image, CV_INTER_LINEAR);

  copy_opencv_image_to_raw_image(depth_image, raw_image, 1);

}


//void compute_depth_map_rescale() {
//  unsigned char *right_roi_image, *left_roi_image, *disparity_roi_map;
//
//
//  copy_raw_image_to_opencv_image(stereo_image->raw_left, image_left_copy, 3);
//
//
//
//  cvResize(image_left_copy, image_left_rescale, CV_INTER_LINEAR);
//  copy_opencv_image_to_raw_image(image_left_rescale, raw_image_left_rescale,3);
//
//
//
//  copy_raw_image_to_opencv_image(stereo_image->raw_right, image_right_copy, 3);
//
//
//  cvResize(image_right_copy, image_right_rescale, CV_INTER_LINEAR);
//  copy_opencv_image_to_raw_image(image_right_rescale,
//      raw_image_right_rescale, 3);
//
//  left_roi_image = raw_image_left_rescale + vertical_ROI_ini * 3
//  * stereo_scalew;
//  right_roi_image = raw_image_right_rescale + vertical_ROI_ini * 3
//  * stereo_scalew;
//  disparity_roi_map = raw_disparity_rescale + vertical_ROI_ini
//  * stereo_scalew;
//
//  //printf("%d\n",vertical_ROI_ini*3*stereo_scalew);
//
//  switch (algorithm[0]) {
//  case 'V':
//    disparityVgRamWNN(left_roi_image, right_roi_image, ROI_height,
//        ROI_width, max_disparity_rescale, disparity_message.disparity, synapses,
//        numthreads, wintakeiteration, inc_width, inc_height);
//    break;
//  case 'C':
//    disparityCSBP(left_roi_image, right_roi_image, ROI_height, ROI_width,
//        max_disparity_rescale, disparity_roi_map);
//    break;
//  }
//
//  copy_raw_image_to_opencv_image(raw_disparity_rescale, depth_image_rescale,
//      1);
//  cvSaveImage("dips.png", depth_image_rescale, 0);
//  cvResize(depth_image_rescale, depth_image, CV_INTER_LINEAR);
//  adjust_disparity_scale_from_opencv_image(depth_image,
//      disparity_message.disparity);
//}

void rgb_to_gray(unsigned char *src, unsigned char *dst, int width, int height)
{
  int i;
  int n = width * height;

#ifndef DEBUG
  #pragma omp parallel for
#endif
  for(i = 0; i < n; i++)
  {
    //dst[i]= 0.30 * src[3 * i] + 0.59 * src[3 * i + 1] + 0.11 * src[3 * i + 2];
    dst[i]= round((src[3 * i] + src[3 * i + 1] + src[3 * i + 2]) / 3.0);
  }
}

void compute_depth_map()
{
  unsigned char *right_roi_image, *left_roi_image;
  float *disparity_roi_map;


//  cvShowImage("image_left", image_left_copy);
//  cvShowImage("image_right", image_right_copy);
//
//  cvWaitKey(-1);


  left_roi_image = raw_image_left + vertical_ROI_ini * 3
  * bumblebee_basic_width;
  right_roi_image = raw_image_right + vertical_ROI_ini * 3
  * bumblebee_basic_width;
  disparity_roi_map = right_disparity + vertical_ROI_ini
  * bumblebee_basic_width;

//  gettimeofday(&inicioTotal, NULL);

  switch (algorithm[0]) {
  case 'V':
    disparityVgRamWNN(left_roi_image, right_roi_image, ROI_height, ROI_width, max_disparity, disparity_roi_map, synapses,
        numthreads, wintakeiteration, inc_width, inc_height);
    //rescale_image(raw_disparity_rescale);
    //convert_disparity_from_raw_image_to_float(raw_disparity_rescale,
      //  disparity_message.disparity);
    break;
  case 'C':
    disparityCSBP(left_roi_image, right_roi_image, ROI_height, ROI_width,
        max_disparity, raw_disparity_rescale);
    convert_disparity_from_raw_image_to_float(raw_disparity_rescale,
    		left_disparity);
    break;
  case 'E':
    rgb_to_gray(left_roi_image, gray_left_roi, ROI_width, ROI_height);
    rgb_to_gray(right_roi_image, gray_right_roi, ROI_width, ROI_height);

    const int32_t dims[3] = {ROI_width, ROI_height, ROI_width};
    Elas::parameters param(Elas::ROBOTICS);
    param.postprocess_only_left = false;
    Elas elas(param);
    elas.process(gray_left_roi, gray_right_roi, left_disparity, right_disparity, dims);

    int i, j, x, y;
    for (i = 0, y = vertical_ROI_ini; y < vertical_ROI_end; y++, i++)
      for (j = 0, x = horizontal_ROI_ini; x < horizontal_ROI_end; x++, j++)
    	  right_disparity[y * bumblebee_basic_width + x] = MAX(right_disparity[i * ROI_width + j], 0.0);

    break;
  }

//  gettimeofday(&fim, NULL);
//  timevalSubtract(&tempoDec, &fim, &inicioTotal);

//  double p = tempoDec.tv_sec + tempoDec.tv_usec*1.0e-6;
  //printf("Time %lfs\t%lf FPS\n", p, 1.0 / p);
}


void stereo_algorithm_initialization() {

  inc_width = 1;
  inc_height = 1;


  if (stereo_scalew > bumblebee_basic_width && stereo_scaleh > bumblebee_basic_height)
  {
    vertical_ROI_ini = vertical_ROI_ini * ((float) stereo_scaleh
        / bumblebee_basic_height);
    vertical_ROI_end = vertical_ROI_end * ((float) stereo_scaleh
        / bumblebee_basic_height);

    horizontal_ROI_ini = horizontal_ROI_ini * ((float) stereo_scalew
        / bumblebee_basic_width);
    horizontal_ROI_end = horizontal_ROI_end * ((float) stereo_scalew
        / bumblebee_basic_width);

    ROI_height = vertical_ROI_end - vertical_ROI_ini;
    ROI_width = stereo_scalew;

    if (vertical_ROI_end > stereo_scaleh)
      vertical_ROI_end = stereo_scaleh;
    if (ROI_height < 0) {
      ROI_height = stereo_scaleh;
      printf(
          "Warnning: The stereo_vertical_ROI_ini is bigger than stereo_vertical_ROI_end\n");
    }

    image_left_rescale = cvCreateImage(
        cvSize(stereo_scalew, stereo_scaleh), IPL_DEPTH_8U, 3);
    image_right_rescale = cvCreateImage(
        cvSize(stereo_scalew, stereo_scaleh), IPL_DEPTH_8U, 3);
    depth_image_rescale = cvCreateImage(
        cvSize(stereo_scalew, stereo_scaleh), IPL_DEPTH_8U, 1);
//    raw_disparity_rescale = (unsigned char *) calloc((stereo_scalew
//        * stereo_scaleh), sizeof(unsigned char));
//    depth_image = cvCreateImage(cvSize(stereo_width, stereo_height),
//        IPL_DEPTH_8U, 1);

    max_disparity_rescale = (int)  (max_disparity);// * (stereo_scalew  / (double) bumblebee_basic_width));

    switch (algorithm[0]) {
    case 'V':
      WNNInitialize(stereo_scaleh, stereo_scalew, max_disparity,
          numdisparity, numthreads, synapses, gaussian_radius,
          horizontal_ROI_ini, horizontal_ROI_end, lambda);
      //WNN_initialize_mult_neuron_stereo(bumblebee_basic_height,bumblebee_basic_width,max_disparity,numdisparity,numthreads,synapses,gaussian_radius,horizontal_ROI_ini,horizontal_ROI_end,1);
      break;
    case 'C':
      init_stereo(stereo_scaleh, stereo_scalew, max_disparity,
          QX_DEF_BP_NR_PLANE,
          QX_DEF_BP_COST_DISCONTINUITY_SINGLE_JUMP,
          QX_DEF_BP_COST_MAX_DATA_TERM, QX_DEF_BP_MAX_NR_JUMP,
          QX_DEF_BP_NR_SCALES, NULL);
      break;
    }
  } else {

    inc_width = (int) (((double) bumblebee_basic_width / stereo_scalew));
    inc_height = (int) (((double) bumblebee_basic_height / stereo_scaleh));
    raw_disparity_rescale = (unsigned char *) calloc((bumblebee_basic_width
        * bumblebee_basic_height), sizeof(unsigned char));
//    printf("stereo_scalew = %d \n, ", inc_width);

    depth_image_rescale = cvCreateImage(cvSize(stereo_scalew, stereo_scaleh),
        IPL_DEPTH_8U, 1);

    depth_image = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 1);

    ROI_height = vertical_ROI_end - vertical_ROI_ini;
    ROI_width = bumblebee_basic_width;

    if (ROI_height < 0) {
      ROI_height = bumblebee_basic_height;
      printf(
          "Warnning: The stereo_vertical_ROI_ini is bigger than stereo_vertical_ROI_end\n");
    }

    switch (algorithm[0]) {
    case 'V':
      WNNInitialize(bumblebee_basic_height, bumblebee_basic_width,
          max_disparity, numdisparity, numthreads, synapses,
          gaussian_radius, horizontal_ROI_ini, horizontal_ROI_end, lambda);
      break;
    case 'C':

      init_stereo(bumblebee_basic_height, bumblebee_basic_width,
          max_disparity, QX_DEF_BP_NR_PLANE,
          QX_DEF_BP_COST_DISCONTINUITY_SINGLE_JUMP,
          QX_DEF_BP_COST_MAX_DATA_TERM, QX_DEF_BP_MAX_NR_JUMP,
          QX_DEF_BP_NR_SCALES, NULL);
      break;
    case 'E':
      left_disparity = (float*)malloc(ROI_width * ROI_height * sizeof(float));
      right_disparity = (float*)malloc(ROI_width * ROI_height * sizeof(float));
      gray_left_roi = (unsigned char*)malloc(ROI_width * ROI_height * sizeof(unsigned char));
      gray_right_roi = (unsigned char*)malloc(ROI_width * ROI_height * sizeof(unsigned char));
      break;
    }
  }

  if (left_disparity != NULL)
	  free(left_disparity);
  left_disparity = (float*)malloc(ROI_width * ROI_height * sizeof(float));

  if (right_disparity != NULL)
  	  free(right_disparity);
  right_disparity = (float*)malloc(ROI_width * ROI_height * sizeof(float));

  if (raw_image_left != NULL)
    free(raw_image_left);
  raw_image_left = (unsigned char *) calloc((ROI_width * ROI_height)* 3, sizeof(unsigned char));

  if (raw_image_right != NULL)
      free(raw_image_right);
  raw_image_right = (unsigned char *) calloc((ROI_width * ROI_height)* 3, sizeof(unsigned char));

  copy_opencv_image_to_raw_image(image_left_copy, raw_image_left, 3);

  copy_opencv_image_to_raw_image(image_right_copy, raw_image_right , 3);

}

void disparity_opmization(float **disparity, int wo, int ho, int number_of_iterations, int levels)
{
	int l, i, yo, xo, before_op, highest_opinion_index, opinion;
	float  central_pixel;
	for (l = 0; l < number_of_iterations;l++)
	{
		for (yo = 0; yo < ho; yo++)
		{
			for (xo = 0; xo < wo; xo++)
			{
				highest_opinion_index = 0;
				before_op = 1;

				for (i = 0; i < levels; i++)
				{
					central_pixel = disparity[i][yo * wo + xo];
					opinion = 0;
					//						//UP
					if (!(yo == 0))
						if (central_pixel == disparity[i][(yo - 1) * wo + xo])
							opinion++;

					//RIGHT
					if (!(xo == wo - 1))
						if (central_pixel == disparity[i][yo * wo + (xo + 1)])
							opinion++;

					//LEFT
					if (!(xo == 0))
						if (central_pixel == disparity[i][yo * wo + (xo - 1)])
							opinion++;

					//LOW
					if (!(yo == ho - 1))
						if (central_pixel == disparity[i][(yo + 1) * wo + xo])
							opinion++;

					if (opinion > before_op)
					{
						highest_opinion_index = i;
						before_op = opinion;
					}
				}
				if (highest_opinion_index)
				{
		//			aux = disparity[0][yo * wo + xo];
					disparity[0][yo * wo + xo] = disparity[highest_opinion_index][yo * wo + xo];
			//		disparity[highest_opinion_index][yo * wo + xo] = aux;
				}
			}
		}
	}
}

int main(int argc __attribute__ ((unused)), char **argv)
{
	int i, j, k, test = 0;
	int tal1 = atoi(argv[9]);
	int tal2 = atoi(argv[10]);
	int L1 = atoi(argv[11]);
	int L2 = atoi(argv[12]);
	char img_name[1000];

	int it = atoi(argv[17]);
	float scale_factor = atof(argv[18]);

	float  **disp;// *final_confidance; **confidance;

	//confidance = (float **)calloc(100,sizeof(float *));
	disp = (float **)calloc(100,sizeof(float *));

	FILE *fout = fopen(argv[14], "w");
	FILE *fmiddle_bury = fopen(argv[16], "w");

	IplImage *left = cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR);
	IplImage *right= cvLoadImage(argv[2], CV_LOAD_IMAGE_COLOR);

	image_left_copy = cvCreateImage(cvSize(left->width, left->height), IPL_DEPTH_8U, 3);
	image_right_copy = cvCreateImage(cvSize(left->width, left->height), IPL_DEPTH_8U, 3);


	cvCopyImage(left, image_left_copy);
	cvCopyImage(right, image_right_copy);

	cvSmooth(left, left, CV_GAUSSIAN, 3, 0, 0, 0);
	cvSmooth(right, right, CV_GAUSSIAN, 3, 0, 0, 0);
//	cvSmooth(left, left, CV_MEDIAN, 3, 0, 0, 0);
//	cvSmooth(right, right, CV_MEDIAN, 3, 0, 0, 0);


	stereo_width = image_right_copy->width;
	stereo_height = image_right_copy->height;

	max_disparity = atoi(argv[3]);
	gaussian_radius = atof(argv[5]);
	algorithm = argv[4];
	synapses = atoi(argv[6]);
	numthreads = 10;
	wintakeiteration = atoi(argv[7]);
	float factor = atof(argv[13]);
	numdisparity = max_disparity;
	stereo_scalew = stereo_width;
	stereo_scaleh = stereo_height;
	vertical_ROI_ini = 0;
	vertical_ROI_end = stereo_height;
	horizontal_ROI_ini = 0;
	horizontal_ROI_end = stereo_width;
	bumblebee_basic_width = stereo_width;
	bumblebee_basic_height = stereo_height;
	lambda = atof(argv[8]);


	IplImage *disp_image = cvCreateImage(cvSize(stereo_width,stereo_height), IPL_DEPTH_8U, 1);


	filtered_image_left = (unsigned char *) calloc((right->width * right->height)* 3, sizeof(unsigned char));
	filtered_image_right = (unsigned char *) calloc((right->width * right->height)* 3, sizeof(unsigned char));

	//final_confidance = (float *) calloc(right->width * right->height, sizeof(float));
	IplImage *final_disp_image = cvCreateImage(cvSize(disp_image->width,disp_image->height), IPL_DEPTH_8U, 1);

	copy_opencv_image_to_raw_image(left, filtered_image_left, 3);

	copy_opencv_image_to_raw_image(right, filtered_image_right , 3);

	set_image_filtered(filtered_image_left, filtered_image_right, right->width, right->height, right->nChannels);


//	for (gaussian_radius = 0.3; gaussian_radius < 10.0; gaussian_radius +=0.2)
	{
//		for (synapses = 32; synapses <= 1024; synapses *= 2, ic += 16)
		{
			disp[0] = (float *)calloc(disp_image->height * disp_image->width, sizeof(float));

			stereo_algorithm_initialization();
//			for (lambda = synapses / 4; lambda < synapses; lambda += ic)
			for (k = 0; k < it; k++, gaussian_radius *= scale_factor)
			{
				build_synapses_cordinates(right->width, right->height, synapses, gaussian_radius);

				disp[k] = (float *)calloc(disp_image->height * disp_image->width, sizeof(float));



				//for (L1 = 10, L2 = 1; L1 < 40; L1+=2)
				//for (L2 = 5; L1 < 40; L1+=2)
				{
					//for (tal1 = 2, tal2 = 0; tal1 < 30; tal1++)
					//for (tal2 = 2; tal2 < tal1; tal2++)
					{
						set_scan_image_param(tal1, tal2, L1, L2);
						compute_depth_map();
						//confidance[k] = get_wnn_confidance(disp_image->width, disp_image->height);



						for (i = 0; i < disp_image->height; i++)
						{
							for(j = 0; j < disp_image->width; j++)
							{
								//printf("%f", right_disparity[i * disp_image->width + j]);
								disp[k][i * disp_image->width + j] = right_disparity[i * disp_image->width + j];
								final_disp_image->imageData[i * disp_image->widthStep + j] = disp_image->imageData[i * disp_image->widthStep + j] = (unsigned char)(factor * (right_disparity[i * disp_image->width + j]));
//								if (confidance[k][i * disp_image->width + j] > final_confidance[i * disp_image->width + j])
//								{
//									//disp[0][i * disp_image->width + j] = right_disparity[i * disp_image->width + j];
//									final_confidance[i * disp_image->width + j] = confidance[k][i * disp_image->width + j];
//									final_disp_image->imageData[i * disp_image->widthStep + j] = disp_image->imageData[i * disp_image->widthStep + j];
//								}
							}
//							for(j = 0; j < 20; j++)
//							{
//								final_disp_image->imageData[i * disp_image->widthStep + j] = (unsigned char)(factor * (right_disparity[i * disp_image->width + 20]));
//							}

						}

						sprintf(img_name, "%s/%s%d.pgm", argv[15], argv[15], test);

						cvSaveImage(img_name, disp_image, 0);

						fprintf(fout, "%s %d %f %d %d %d %d %d %f\n", img_name, synapses, gaussian_radius,
								wintakeiteration, tal1, tal2, L1, L2, lambda);
						printf("test %d\n", test);
						fprintf(fmiddle_bury, "depth_map results/%s%d.pgm\n", argv[15], test);
						test++;
						cvShowImage("disparity", disp_image);


						cvShowImage("disparity2", final_disp_image);

						cvWaitKey(33);
					}
				}
			}
		}
	}

//	disparity_opmization(confidance, disp, final_disp_image->width, final_disp_image->height, 7, it);
//
//	for (i = 0; i < disp_image->height; i++)
//	{
//		for(j = 0; j < disp_image->width; j++)
//		{
//			//printf("%f", right_disparity[i * disp_image->width + j]);
//			final_disp_image->imageData[i * disp_image->widthStep + j] = (unsigned char)(factor * (disp[0][i * disp_image->width + j]));
//		}
//		for(j = 0; j < 30; j++)
//		{
//			final_disp_image->imageData[i * disp_image->widthStep + j] = (unsigned char)(factor * (disp[0][i * disp_image->width + 30]));
//		}
//	}
	cvSmooth(final_disp_image, final_disp_image, CV_MEDIAN, 7, 0, 0, 0);

	cvShowImage("disparity2", final_disp_image);

	sprintf(img_name, "%s.pgm", argv[15]);

	cvSaveImage(img_name, final_disp_image, 0);

	cvWaitKey(-1);

}
//./stereo_test ~/Downloads/stereo-pairs/tsukuba/imL.png ~/Downloads/stereo-pairs/tsukuba/imR.png 16 VGRAM
//10.0 1024 5 16 15 3 34 0 16 venus_param.txt tsukuba venus_middlebury_test.txt 30 0.95

	//		cvShowImage("disparity", disp_image);
	//
	////
	//		cvWaitKey(-1);
	//
	////		gaussian_radius +=0.2;
	//		stereo_height *= 0.9;
	//		stereo_width *= 0.9;
	//		max_disparity *= 0.9;
	//
	//		image_left_copy = cvCreateImage(cvSize(stereo_width, stereo_height), IPL_DEPTH_8U, 3);
	//		image_right_copy = cvCreateImage(cvSize(stereo_width, stereo_height), IPL_DEPTH_8U, 3);
	//		cvResize(left, image_left_copy, CV_INTER_LINEAR);
	//		cvResize(right, image_right_copy, CV_INTER_LINEAR);
