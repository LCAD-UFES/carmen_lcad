/********************************************************************************************************
\Author:	Qingxiong Yang (http://vision.ai.uiuc.edu/~qyang6/)
\Function:	Constant space BP (CPU) given two color images.
\reference: Qingxiong Yang, Liang Wang and Narendra Ahuja, A Constant-Space Belief Propagation Algorithm
			for Stereo Matching, IEEE Conference on Computer Vision and Pattern Recognition (CVPR) 2010.
*********************************************************************************************************/
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
 

#ifndef QX_CSBP_H
#define QX_CSBP_H



#ifdef __cplusplus
extern "C" {
#endif

#define QX_DEF_PADDING					10


#define max(a,b)            (((a) > (b)) ? (a) : (b))
#define min(a,b)            (((a) < (b)) ? (a) : (b))

#define QX_DEF_BP_NR_PLANE							2

#define QX_DEF_BP_NR_SCALES							5
#define QX_DEF_BP_NR_ITER0							5
#define QX_DEF_BP_NR_ITER1							5
#define QX_DEF_BP_NR_ITER2							5
#define QX_DEF_BP_NR_ITER3							5
#define QX_DEF_BP_NR_ITER4							5
#define QX_DEF_BP_NR_ITER5							5

#define QX_DEF_BP_COST_MAX_DATA_TERM				30	/*Truncation of data cost*/
#define QX_DEF_BP_COST_DISCONTINUITY_SINGLE_JUMP	10		/*discontinuity cost*/
#define QX_DEF_BP_MAX_NR_JUMP						2		/*Truncation of discontinuity cost*/

#define QX_DEF_BP_NR_NEIGHBOR						4

short  *disp;
int *m_h_pyramid;
int *m_w_pyramid;
int *m_iteration;
int *m_max_nr_plane_pyramid;
int m_cost_max_discontinuity;
int m_max_nr_message;
int m_nr_scale;
int m_nr_neighbor;
int *m_message;
int *m_selected_disparity_pyramid;
int *m_data_cost;
int *m_temp3;
int *m_temp4;
int *m_data_cost_selected;

struct timeval inicioTotal, fim, tempoDec;

int timevalSubtract (struct timeval *result, struct timeval *x, struct timeval *y)  ;

int compute_disparity(short *disparity,int scale, int *m_h_pyramid, int *m_w_pyramid, int *m_max_nr_plane_pyramid,
		      int m_nr_neighbor,int *m_temp3, int *m_data_cost_selected, int *m_message,int *m_selected_disparity_pyramid);

#ifndef NO_CUDA	
short* disparityCUDA(unsigned char*left,unsigned char*right, int m_h, int m_w, int m_nr_plane,int max_nr_plane, int m_discontinuity_cost_single_jump, int max_nr_jump, int m_cost_max_data_term);
#endif

short* disparityOMP(unsigned char*left,unsigned char*right, int m_h, int m_w, int m_nr_plane,int max_nr_plane, int m_discontinuity_cost_single_jump, int max_nr_jump, int m_cost_max_data_term);

void beliefs_computation(int scale);
void depth_computation_debug(int scale);

int bpstereo_vec_min(int *in,int len);
void bpstereo_normalize(int *in,int len);
void discontinuity_cost(unsigned char ***image);



int compute_data_cost_per_pixel_rgb(unsigned char left[3],unsigned char right[3],int cost_max_data_term);


void compute_message(int h,int w,int nr_plane,int scale,int *m_temp3, int m_cost_max_discontinuity,
		     int m_discontinuity_cost_single_jump, int m_nr_neighbor, int *m_data_cost_selected,
		     int *m_message, int *m_selected_disparity_pyramid );

void compute_message_per_pixel(int*c0,int *p0,int *p1,int *p2,int *p3,int *p4,int*d0,int*d1,int*d2,
		int*d3,int*d4,int y,int x,int nr_plane,int scale,
		int *m_temp3, int m_cost_max_discontinuity, int m_discontinuity_cost_single_jump);

void compute_message_per_pixel_per_neighbor(int *comp_func_sub,int minimum,int *disp_left,int *disp_right,int nr_plane,int scale,
					    int *m_temp3, int m_cost_max_discontinuity, int m_discontinuity_cost_single_jump);

void qx_get_first_k_element_increase(int*q1,int*q2,int*q3,int*q4,int*p1,int*p2,int*p3,int*p4,
		int*cost,int*disp,int*coarse,int*in,int*disp_in,int len,int len_in);

void init_message(int scale_index, int *m_max_nr_plane_pyramid, int *m_h_pyramid,int *m_w_pyramid,
		  int m_nr_neighbor,int *m_temp3, int *m_temp4,int *m_selected_disparity_pyramid,
		  int *m_message, int *m_data_cost, int *m_data_cost_selected);

void compute_data_cost_init(unsigned char*left,unsigned char*right,
		int h,int w,int scale,int nr_plane,int nr_plane_in,int cost_max_data_term, int *m_temp3,
		int m_w,int m_h,int *m_temp4, int *m_selected_disparity_pyramid,int *m_data_cost_selected);

void compute_data_cost(unsigned char*left,unsigned char*right,int h,int w,int scale,int nr_plane,int cost_max_data_term, int m_nr_plane,
		       int *m_h_pyramid, int *m_w_pyramid, int *m_temp3,int *m_selected_disparity_pyramid,int *m_data_cost, int m_w);

void qx_get_first_k_element_increase_special(int*cost,int *disp,int*in,int *disp_in,int len,int len_in);

void init_stereo(int h,int w,int nr_plane,int max_nr_plane, int discontinuity_cost_single_jump, int cost_max_data_term, int max_nr_jump,int nr_scales, int *iterations);

void disparityCSBP(unsigned char*image_left,unsigned char*image_right, int stereo_height, int stereo_width, int stereo_disparity, unsigned char *);

int freeAllMem();

#ifdef __cplusplus
}
#endif

#endif
