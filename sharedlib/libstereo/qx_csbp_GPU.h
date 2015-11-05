#include <stdio.h>
#include <stdlib.h>
#ifndef NO_CUDA	
#include <cuda.h>
#endif
#ifndef QX_CSBP_GPU_H
#define QX_CSBP_GPU_H

void 
compute_message_GPU(int h,int w,int nr_plane_h,int scale,short *m_data_cost_selected,short *m_message,
		    short *m_selected_disparity_pyramid,int cost_max_discontinuity,int nr_neighbor, int discontinuity_cost_single_jump);

void
bpstereo_normalize_GPU(int *in,int len,int g_val);

void allocGPUData(void **devPtr,size_t size);


void compute_message_per_pixel_per_neighbor_GPU(int *comp_func_sub,int minimum,int *disp_left,int *disp_right,int scale,int nr_plane_h,int cost_max_discontinuity, int discontinuity_cost_single_jump);

void compute_message_per_pixel_GPU(int*c0,int *p0,int *p1,int *p2,int *p3,int *p4,int*d0,int*d1,int*d2,
							int*d3,int*d4,int y,int x,int scale,int nr_plane_h,int cost_max_discontinuity, int discontinuity_cost_single_jump);

// void
// init_GPU_date(short **m_data_cost_selected, int m_data_cost_selected_size,short **m_message, int m_message_size, short **m_selected_disparity_pyramid,
// 	      int m_selected_disparity_pyramid_size,unsigned char **left,unsigned char **right,int imsize,short **m_data_cost, int m_data_cost_size);


void GPUcopy(void * dest, void *src, size_t size, int type);

void 
compute_data_cost_GPU(unsigned char*left,unsigned char*right,int h,int w,int scale,int nr_plane_h,int cost_max_data_term,short *m_selected_disparity_pyramid, 
		      int m_w, short  *m_data_cost, int m_nr_plane,int *m_h_pyramid, int *m_w_pyramid);

void 
compute_data_cost_init_GPU(unsigned char*left,unsigned char*right,
		int h,int w,int scale,int nr_plane_h,int nr_plane_in,
		int cost_max_data_term, int m_w,short *m_selected_disparity_pyramid,short *m_data_cost_selected);


void init_message_GPU (int scale_index,int *m_max_nr_plane_pyramid, int *m_h_pyramid, int *m_w_pyramid,int nr_neighbor,
		  short * m_message, short *m_data_cost,short *m_selected_disparity_pyramid,short *m_data_cost_selected);

void compute_disparity_GPU(unsigned char *disparity_GPU, unsigned char *disparity,int scale, int *m_h_pyramid, int *m_w_pyramid, int nr_neighbor, 
			   int *m_max_nr_plane_pyramid,short *m_selected_disparity_pyramid, short *m_data_cost_selected, short *m_message);

int GPUinit(int m_h,int m_w,int m_max_nr_message, int m_max_nr_plane_pyramid);
int GPUdelete();

short* disparity_GPU(unsigned char*left,unsigned char*right, int m_h, int m_w, int m_nr_plane,int max_nr_plane, int m_discontinuity_cost_single_jump, int max_nr_jump, 
	 int m_cost_max_data_term, int m_cost_max_discontinuity,int m_max_nr_message, int m_nr_scale,int m_nr_neighbor,int *m_iteration, int *m_max_nr_plane_pyramid, 
	 int *m_h_pyramid,int *m_w_pyramid, short *disp);


#endif
