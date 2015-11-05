
#define NUM_THREADS 256
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <cuda.h>
#include <sys/time.h>
//#include "qx_csbp_GPU.h"

#define max(a,b)            (((a) > (b)) ? (a) : (b))
#define min(a,b)            (((a) < (b)) ? (a) : (b))

//__constant__ int yshift;
//__constant__ int yshift2;
//__constant__ int xshift;
//__constant__ int yshiftd;
//__constant__ int th;
// __constant__ int h2_;
// __constant__ int w2_;

// __constant__ int h;
// __constant__ int w;
//__constant__ int nr_plane;
//__constant__ int idmax;

//__constant__ int h2;
//__constant__ int w2;
//__constant__ int nr_plane2;
//__constant__ int x2shift;
// __constant__ int y2shift;

// __constant__ int yshiftd2;
// __constant__ int yshiftd_finer;

// __constant__ int m_cost_max_discontinuity;
// __constant__ int m_discontinuity_cost_single_jump;

//__constant__ int m_nr_neighbor;

short *m_data_cost_selected_d;
short *m_message_d;
short *m_selected_disparity_pyramid_d;
unsigned char *left_d, *right_d;
short *m_data_cost_d;
short *disp_d;


extern "C" void allocGPUData(void **devPtr,size_t size)
{
	cudaMalloc(devPtr, size);
}


extern "C" 
void init_GPU_date(short **m_data_cost_selected, int m_data_cost_selected_size,
		   short **m_message, int m_message_size, short **m_selected_disparity_pyramid, 
		   int m_selected_disparity_pyramid_size,int **left,
		   int **right, int imsize, short **m_data_cost, int m_data_cost_size)
{
	cudaMalloc((void**)m_data_cost_selected,m_data_cost_selected_size*sizeof(int));
	cudaMalloc((void**)m_message,m_message_size*sizeof(int));
	cudaMalloc((void**)m_selected_disparity_pyramid,m_selected_disparity_pyramid_size*sizeof(int));
	cudaMalloc((void**)m_selected_disparity_pyramid,m_selected_disparity_pyramid_size*sizeof(int));
	cudaMalloc((void**)left,imsize*sizeof(unsigned char));
	cudaMalloc((void**)right,imsize*sizeof(unsigned char));

	cudaMalloc((void**)m_data_cost, m_data_cost_size*sizeof(int));
}

extern "C" void 
GPUcopy(void * dest, void *src, size_t size, int type)
{
	if(type)
	{
		cudaMemcpy(dest,src, size,cudaMemcpyHostToDevice);
	}else{

		cudaMemcpy(dest,src, size,cudaMemcpyDeviceToHost);
	}
}

void bpstereo_normalize(int *in,int len)
{
	int val=0;
	for(int i=0;i<len;i++) 
		val+=in[i];
	val/=len;
// 	printf("%d ",in[0]);
	
	//bpstereo_normalize_GPU(in,len,val);
	
	for(int i=0;i<len;i++) 
 		in[i]-=val;
// 	printf("%d\n",in[0]);
}

// __device__ void compute_message_per_pixel_per_neighbor(int *comp_func_sub, int minimum,int *disp_left,int *disp_right,int scale)
// {
// 	__shared__ int m_temp[32][32];
// 	int val=0;
// 
// 	for(int d=0;d<nr_plane;d++)
// 	{
// 		int cost_min=minimum+m_cost_max_discontinuity;
// 		for(int i=0;i<nr_plane;i++)
// 		{
// 			cost_min=min(cost_min,comp_func_sub[i]+m_discontinuity_cost_single_jump*abs(disp_left[i]-disp_right[d]));
// 		}
// 		m_temp[threadIdx.x][d]=cost_min;
// 		val+=cost_min;
// 	}
// 
// 	val/=nr_plane;
// 
// 	for(int d=0;d<nr_plane;d++)
// 	{
// 		comp_func_sub[d]=m_temp[threadIdx.x][d]-val;
// 	  
// 	}
// }

__device__ void compute_message_per_pixel_per_neighbor(short *comp_func_sub,short*c0,short *p1,short *p2,short *p3,short *disp_left,short *disp_right,int scale,int nr_plane_h,int cost_max_discontinuity,int discontinuity_cost_single_jump)
{

	__shared__ int g_val[4];
	__shared__ int minimum_s[4];
 	__shared__ short m_temp[4][64];
	short minimum=30000;
	short cost_min;
	
	__shared__ int disp_lefts[64];
	
	
	g_val[threadIdx.x]=0;
	int val=0;
	minimum_s[threadIdx.x]=30000;
	minimum=30000;
	
	int d=threadIdx.y;
	if(d<nr_plane_h)
	{
		disp_lefts[d]=disp_left[d];
		
		val=c0[d]+p1[d]+p2[d]+p3[d];
		minimum=min(minimum,val);
		m_temp[threadIdx.x][d]=val;
	
	
		atomicMin(&minimum_s[threadIdx.x],minimum);

		__syncthreads();

		minimum=minimum_s[threadIdx.x];
		val=disp_right[d];
		
		cost_min=minimum+cost_max_discontinuity;
		for(int i=0;i<nr_plane_h;i++)
		{
			cost_min=min(cost_min,m_temp[threadIdx.x][i]+discontinuity_cost_single_jump*abs(disp_lefts[i]-val));
		}
		
		atomicAdd(&g_val[threadIdx.x],cost_min);

		__syncthreads();

		comp_func_sub[d]=cost_min - g_val[threadIdx.x]/nr_plane_h;
	  
	}
}

__device__ void compute_message_per_pixel(short*c0,short *p0,short *p1,short *p2,short *p3,short *p4,short*d0,short*d1,short*d2,
							short*d3,short*d4,int scale,int nr_plane_h,int cost_max_discontinuity,int discontinuity_cost_single_jump)
{
	if(threadIdx.x==0)
	{
		compute_message_per_pixel_per_neighbor(p0,c0,p2,p3,p4,d0,d1,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
		return;
	}
	if(threadIdx.x==1)
	{
		compute_message_per_pixel_per_neighbor(&(p0[nr_plane_h]),c0,p1,p3,p4,d0,d2,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
		return;
	}
	if(threadIdx.x==2)
	{
		compute_message_per_pixel_per_neighbor(&(p0[2*nr_plane_h]),c0,p1,p2,p4,d0,d3,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
		return;
	}
	if(threadIdx.x==3)
	{
		compute_message_per_pixel_per_neighbor(&(p0[3*nr_plane_h]),c0,p1,p2,p3,d0,d4,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
		return;
	}
}


__device__ void compute_message_per_pixel_per_neighbor2(short *comp_func_sub,short*c0,short *p1,short *p2,short *p3,short *disp_left,short *disp_right,int scale,int nr_plane,int cost_max_discontinuity,int discontinuity_cost_single_jump)
{
	__shared__ short m_temp[32][32];

	__shared__ short m_temp2[32][32];
	int minimum=30000;
	int val;
	for(int d=0;d<nr_plane;d++)
	{
		val=c0[d]+p1[d]+p2[d]+p3[d];
		m_temp2[threadIdx.x][d]=val;
		minimum=min(minimum,val);
	}
	val=0;
	for(int d=0;d<nr_plane;d++)
	{
		int cost_min=minimum+cost_max_discontinuity;
		short dips_right_aux=disp_right[d];
		for(int i=0;i<nr_plane;i++)
		{
			cost_min=min(cost_min,m_temp2[threadIdx.x][i]+discontinuity_cost_single_jump*abs(disp_left[i]-dips_right_aux));
		}
		m_temp[threadIdx.x][d]=cost_min;
		val+=cost_min;
	}
	val/=nr_plane;
	for(int d=0;d<nr_plane;d++)
	{
		comp_func_sub[d]=m_temp[threadIdx.x][d]-val;
	}
	
}

__device__ void compute_message_per_pixel2(short*c0,short *p0,short *p1,short *p2,short *p3,short *p4,short*d0,short*d1,short*d2,
							short*d3,short*d4,int scale,int nr_plane_h,int cost_max_discontinuity,int discontinuity_cost_single_jump)
{
	__shared__ short m_tempu[256][4];
	__shared__ short m_templ[256][4];
	__shared__ short m_tempd[256][4];
	__shared__ short m_tempr[256][4];
		

	int minimum[4]={30000,30000,30000,30000};
	short *p0l=&(p0[nr_plane_h]);
	short *p0d=&(p0[nr_plane_h+nr_plane_h]);
	short *p0r=&(p0[nr_plane_h+nr_plane_h+nr_plane_h]);

	for(int d=0;d<nr_plane_h;d++)
	{
		m_tempu[threadIdx.x][d]=c0[d]+p2[d]+p3[d]+p4[d];
		m_templ[threadIdx.x][d]=c0[d]+p1[d]+p3[d]+p4[d];
		m_tempd[threadIdx.x][d]=c0[d]+p1[d]+p2[d]+p4[d];
		m_tempr[threadIdx.x][d]=c0[d]+p1[d]+p2[d]+p3[d];

		minimum[0] = min( minimum[0],m_tempu[threadIdx.x][d]);
		minimum[1] = min( minimum[1],m_templ[threadIdx.x][d]);
		minimum[2] = min( minimum[2],m_tempd[threadIdx.x][d]);
		minimum[3] = min( minimum[3],m_tempr[threadIdx.x][d]);
		//m_comp_func_sub_prev[d]=p1[d]+p2[d]+p3[d]+p4[d];
		
	}

	int val1=0,val2=0,val3=0,val4=0;
	for(int d=0;d<nr_plane_h;d++)
	{
		short cost_min1=minimum[0]+cost_max_discontinuity;
		short cost_min2=minimum[1]+cost_max_discontinuity;
		short cost_min3=minimum[2]+cost_max_discontinuity;
		short cost_min4=minimum[3]+cost_max_discontinuity;

		short dips_right1=d1[d];
		short dips_right2=d2[d];
		short dips_right3=d3[d];
		short dips_right4=d4[d];
		#pragma unroll 2
		for(int i=0;i<nr_plane_h;i++)
		{
			short dips_left=d0[i];
			cost_min1=min(cost_min1,m_tempu[threadIdx.x][i]+discontinuity_cost_single_jump*abs(dips_left-dips_right1));
			cost_min2=min(cost_min2,m_templ[threadIdx.x][i]+discontinuity_cost_single_jump*abs(dips_left-dips_right2));
			cost_min3=min(cost_min3,m_tempd[threadIdx.x][i]+discontinuity_cost_single_jump*abs(dips_left-dips_right3));
			cost_min4=min(cost_min4,m_tempr[threadIdx.x][i]+discontinuity_cost_single_jump*abs(dips_left-dips_right4));
			
		}
		p0[d]=cost_min1;
		p0l[d]=cost_min2;
		p0d[d]=cost_min3;
		p0r[d]=cost_min4;

		val1+=cost_min1;
		val2+=cost_min2;
		val3+=cost_min3;
		val4+=cost_min4;
	}
	
	val1/=nr_plane_h;
	val2/=nr_plane_h;
	val3/=nr_plane_h;
	val4/=nr_plane_h;
	for(int d=0;d<nr_plane_h;d++)
	{
		p0[d]-=val1;
		p0l[d]-=val2;
		p0d[d]-=val3;
		p0r[d]-=val4;
		//comp_func_sub[d]=m_temp[threadIdx.x][d]-val;
	}

// 	compute_message_per_pixel_per_neighbor2(p0,c0,p2,p3,p4,d0,d1,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
// 	compute_message_per_pixel_per_neighbor2(&(p0[nr_plane_h]),c0,p1,p3,p4,d0,d2,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
// 	compute_message_per_pixel_per_neighbor2(&(p0[2*nr_plane_h]),c0,p1,p2,p4,d0,d3,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
// 	compute_message_per_pixel_per_neighbor2(&(p0[3*nr_plane_h]),c0,p1,p2,p3,d0,d4,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
}

__global__ void 
compute_message2(int h,int w,int scale,short *m_data_cost_selected,short *m_message,short *m_selected_disparity_pyramid,int i,
		int yshift_h,int xshift_h,int yshiftd_h,int nr_plane_h,int cost_max_discontinuity,int discontinuity_cost_single_jump)
{

	int y=blockIdx.x+1;
	if(y<h)
	{
		int yl=y*yshift_h;
		int yld=y*yshiftd_h;
		for(int x=(y+i)%2+1+(2*threadIdx.x);x<w;x+=2*blockDim.x)
		{
		      	int xl=x*xshift_h+yl;
			int xld=x*nr_plane_h+yld;
			compute_message_per_pixel2(&(m_data_cost_selected[xld]),
						  &(m_message[xl]),
						  &(m_message[xl-yshift_h+2*nr_plane_h]),
						  &(m_message[xl-xshift_h+3*nr_plane_h]),
						  &(m_message[xl+yshift_h]),
						  &(m_message[xl+xshift_h+nr_plane_h]),
						  &(m_selected_disparity_pyramid[xld]),
						  &(m_selected_disparity_pyramid[xld-yshiftd_h]),
						  &(m_selected_disparity_pyramid[xld-nr_plane_h]),
						  &(m_selected_disparity_pyramid[xld+yshiftd_h]),
						  &(m_selected_disparity_pyramid[xld+nr_plane_h]),
						  scale,
						  nr_plane_h,
						  cost_max_discontinuity,
						  discontinuity_cost_single_jump);

		}
	}
}

 /*void compute_message_per_pixel(int*c0,int *p0,int *p1,int *p2,int *p3,int *p4,int*d0,int*d1,int*d2,
		int*d3,int*d4,int y,int x,int nr_plane,int scale,
 		int *m_temp3, int m_cost_max_discontinuity, int m_discontinuity_cost_single_jump)
 {
 	int minimum[4]={30000,30000,30000,30000};
 	int *p0u=p0;
 	int *p0l=&(p0[nr_plane]);
 	int *p0d=&(p0[nr_plane+nr_plane]);
 	int *p0r=&(p0[nr_plane+nr_plane+nr_plane]);
 
 	for(int d=0;d<nr_plane;d++)
 	{
 		p0u[d]=c0[d]+p2[d]+p3[d]+p4[d];
 		p0l[d]=c0[d]+p1[d]+p3[d]+p4[d];
 		p0d[d]=c0[d]+p1[d]+p2[d]+p4[d];
 		p0r[d]=c0[d]+p1[d]+p2[d]+p3[d];
 		minimum[0] = min( minimum[0],p0u[d]);
 		minimum[1] = min( minimum[1],p0l[d]);
 		minimum[2] = min( minimum[2],p0d[d]);
 		minimum[3] = min( minimum[3],p0r[d]);
 		//m_comp_func_sub_prev[d]=p1[d]+p2[d]+p3[d]+p4[d];
 		
 	}
 
  	compute_message_per_pixel_per_neighbor(p0u,minimum[0],d0,d1,nr_plane,scale,m_temp3, m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
  	compute_message_per_pixel_per_neighbor(p0l,minimum[1],d0,d2,nr_plane,scale,m_temp3, m_cost_max_discontinuity,m_discontinuity_cost_single_jump); 
  	compute_message_per_pixel_per_neighbor(p0d,minimum[2],d0,d3,nr_plane,scale,m_temp3, m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
  	compute_message_per_pixel_per_neighbor(p0r,minimum[3],d0,d4,nr_plane,scale,m_temp3, m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
// // 	
// 
 }*/



__device__ void compute_message_per_pixel_Switch(short*c0,short *p0,short *p1,short *p2,short *p3,short *p4,short*d0,short*d1,short*d2,
							short*d3,short*d4,int scale,int nr_plane_h,int cost_max_discontinuity,int discontinuity_cost_single_jump)
{
	switch(threadIdx.x)
	{
		case 0:
			compute_message_per_pixel_per_neighbor(p0,
								c0,
								p2,
								p3,
								p4,
								d0,
								d1,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
			return;
	
		case 1:
			compute_message_per_pixel_per_neighbor(&(p0[nr_plane_h]),
								c0,
								p1,
								p3,
								p4,
								d0,
								d2,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
			return;
		case 2:
		
			compute_message_per_pixel_per_neighbor(&(p0[2*nr_plane_h]),
								  c0,
								  p1,
								  p2,
								  p4,
								  d0,
								  d3,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
			return;
		case 3:
			compute_message_per_pixel_per_neighbor(&(p0[3*nr_plane_h]),
								c0,
								p1,
								p2,
								p3,
								d0,
								d4,scale,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
			return;
	}
}

__device__ void compute_message_per_pixel_Switch_new(int *m_data_cost_selected,int *m_message,int *m_selected_disparity_pyramid,int scale)
{

}

__global__ void 
compute_message(int h,int w,int scale,short *m_data_cost_selected,short *m_message,short *m_selected_disparity_pyramid,int i,
		int yshift_h,int xshift_h,int yshiftd_h,int nr_plane_h,int cost_max_discontinuity,int discontinuity_cost_single_jump)
{

	int y=blockIdx.y+1;
	if(y<h)
	{
		int x=(y+i)%2+1+(2*blockIdx.x);
		if(x<w)
		{
		  	int yl=y*yshift_h;
			int yld=y*yshiftd_h;
		      	int xl=x*xshift_h+yl;
			int xld=x*nr_plane_h+yld;
			compute_message_per_pixel(&(m_data_cost_selected[xld]),
						  &(m_message[xl]),
						  &(m_message[xl-yshift_h+2*nr_plane_h]),
						  &(m_message[xl-xshift_h+3*nr_plane_h]),
						  &(m_message[xl+yshift_h]),
						  &(m_message[xl+xshift_h+nr_plane_h]),
						  &(m_selected_disparity_pyramid[xld]),
						  &(m_selected_disparity_pyramid[xld-yshiftd_h]),
						  &(m_selected_disparity_pyramid[xld-nr_plane_h]),
						  &(m_selected_disparity_pyramid[xld+yshiftd_h]),
						  &(m_selected_disparity_pyramid[xld+nr_plane_h]),
						  scale,
						  nr_plane_h,
						  cost_max_discontinuity,
						  discontinuity_cost_single_jump);

		}
	}
}

extern "C" void 
compute_message_GPU(int h,int w,int nr_plane_h,int scale,short *m_data_cost_selected,short *m_message,
		    short *m_selected_disparity_pyramid,int cost_max_discontinuity,int nr_neighbor, int discontinuity_cost_single_jump)
{
	int i;
	dim3 dimBlock (4,nr_plane_h>=8?nr_plane_h:8);
	dim3 dimGrid (w-1,h-1);

	int yshift_h=w*nr_neighbor*nr_plane_h;
	int xshift_h=nr_neighbor*nr_plane_h;
	int yshiftd_h=w*nr_plane_h;
	

	
	for(i=0;i<2;i++) 
	{
		if(nr_plane_h>4)
		compute_message<<<dimGrid,dimBlock>>>(h-1, w-1,scale,m_data_cost_selected,m_message,m_selected_disparity_pyramid,i,yshift_h,xshift_h,yshiftd_h,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
		else
		compute_message2<<<h-1,256>>>(h-1, w-1,scale,m_data_cost_selected,m_message,m_selected_disparity_pyramid,i,yshift_h,xshift_h,yshiftd_h,nr_plane_h,cost_max_discontinuity,discontinuity_cost_single_jump);
	}

}


__device__ void qx_get_first_k_element_increase(short *q1,short *q2,short *q3, short *q4,short *p1, short *p2, short *p3,short *p4,
					short *cost,short *disp,short *coarse,short *in,short *disp_in,int len,int len_in)
{
	for(int i=0;i<len;i++)
	{
		int fmin=in[i]; int id=i;
		for(int j=i+1;j<len_in;j++)
		{
			if(in[j]<fmin)
			{
				fmin=in[j]; 
				id=j;
			}
		}
		cost[i]=coarse[id];
		disp[i]=disp_in[id];
		q1[i]=p1[id];
		q2[i]=p2[id];
		q3[i]=p3[id];
		q4[i]=p4[id];
		in[id]=in[i];
		disp_in[id]=disp_in[i];
		coarse[id]=coarse[i];
	}
}

__device__ int compute_data_cost_per_pixel_rgb(uchar3 left,unsigned char *right,int cost_max_data_term)
{
	float tr= (0.299f*abs(left.x-right[0]) + 0.587f*abs(left.y-right[1]) + 0.114f*abs(left.z-right[2]) + 0.5f); 
	return(min((int)tr,cost_max_data_term));
}

// __device__ void cost_per_pixel_init(uchar3 left_x,unsigned char *right, int cost_max_data_term,int *m_temp,int nr_plane_in,int m_w, int xi , int yi)
// {
// 	for(int d=0;d<nr_plane_in;d++)
// 	{
// 		int xr=xi-d;
// 		m_temp[d] += (d<th||xr<0) ? m_temp[d]+=cost_max_data_term : compute_data_cost_per_pixel_rgb(left_x,&(right[3*(yi*m_w+xr)]),cost_max_data_term);
// 	}
// }

__device__ int compute_data_cost_per_pixel_rgb2(unsigned char *left,unsigned char *right,int cost_max_data_term)
{
	float tr= (0.299f*abs(left[0]-right[0]) + 0.587f*abs(left[1]-right[1]) + 0.114f*abs(left[2]-right[2]) + 0.5f); 
	return(min((int)tr,cost_max_data_term));
}

/*__device__ void qx_get_first_k_element_increase_special(short*cost,short *disp,short*in,short *disp_in,int len,int len_in)
{
	__shared__ int fmin_s;
	__shared__ int id_s;
	
	if(threadIdx.x==0)
	{
		fmin_s=100000;
	}
	 
	
	for(int i=0;i<len;i++)
	{
		__syncthreads();
		int id=threadIdx.x;
		int fmin =in[id];
		
		for(int j=id+blockDim.x;j<len_in;j+=blockDim.x)
		{
			if(in[j]<fmin)
			{
				fmin=in[j]; 
				id=j;
			}
		}
		 
	        atomicMin(&fmin_s , fmin);
                __syncthreads();

                if(fmin_s == fmin)
                {
                        id_s=id;
                }


                __syncthreads();

		
		if(threadIdx.x==0)
		{
			id=id_s;
			cost[i]=fmin_s;
			disp[i]=disp_in[id];
			in[id]=in[i];
			disp_in[id]=disp_in[i];
			fmin_s=100000;
		}
		
		
	}
}*/

__device__ void qx_get_first_k_element_increase_special(short*cost,short *disp,short*in,short *disp_in,int len,int len_in)
{
 	for(int i=0;i<len;i++)
 	{
 		int fmin=in[i]; 
 		int id=i;
 		for(int j=i+1;j<len_in;j++)
 		{
 			if(in[j]<fmin)
 			{
 				fmin=in[j]; id=j;
 			}
 		}
 		cost[i]=fmin;
 		disp[i]=disp_in[id];
 		in[id]=in[i];
 		disp_in[id]=disp_in[i];
 	}
}

__global__ void 
compute_data_cost_init(unsigned char*left,unsigned char*right,
		int h,int w,int scale,int nr_plane_in,int cost_max_data_term, int m_w,short *m_selected_disparity_pyramid,short *m_data_cost_selected, int nr_plane_h, int yshift_h, int th_h)
{
	__shared__ short m_temp[256];
	__shared__ short m_temp2[256];
	
	__shared__ short selected_disparity[64];
	__shared__ short data_cost[64];
	
	int i=threadIdx.x;
	
	for(int j=i;j<nr_plane_in;j+=blockDim.x)
	{
		m_temp[j]=0;
		m_temp2[j]=j;
	}
	
	uchar3 left_x;
	
	int y=blockIdx.y;
	if(y<h)
	{
		int x=blockIdx.x;
		if(x<w) 
		{
			int x0=(x<<scale); 
			int xt=((x+1)<<scale);
			int yt=((y+1)<<scale);
			for(int yi=(y<<scale);yi<yt;yi++)
			{
				int aux=__mul24(yi,m_w);
				unsigned char*leftAux=left+3*(aux+x0);
				unsigned char *right_x=right+3*aux;
				
				for(int xi=x0;xi<xt;xi++)
				{
					left_x.x=leftAux[0];
					left_x.y=leftAux[1];
					left_x.z=leftAux[2];
					for(int d=threadIdx.x;d<nr_plane_in;d+=blockDim.x)
					{
						int xr=xi-d;
						m_temp[d]+= (d<th_h||xr<0) ? cost_max_data_term :compute_data_cost_per_pixel_rgb(left_x,right_x+3*xr,cost_max_data_term);
						//atomicAdd(&m_temp[d],aux2);
						
					}
					leftAux+=3;
				}
			}
			__syncthreads();
			if(threadIdx.x==0)
			qx_get_first_k_element_increase_special(data_cost,selected_disparity,m_temp,m_temp2,nr_plane_h,nr_plane_in);
 			__syncthreads();
			
			if(i<nr_plane_h){
				int yl=y*yshift_h+x*nr_plane_h+i;
				m_selected_disparity_pyramid[yl]=selected_disparity[i];
				m_data_cost_selected[yl]=data_cost[i];
			}
		}
	}
}

extern "C" void 
compute_data_cost_init_GPU(unsigned char*left,unsigned char*right,
		int h,int w,int scale,int nr_plane_h,int nr_plane_in,
		int cost_max_data_term, int m_w,short *m_selected_disparity_pyramid,short *m_data_cost_selected)
											 
{
	int yshift_h=w*nr_plane_h;
	int th_h= (int) nr_plane_in*0.2;

	dim3 dimGrid (w,h);
	dim3 dimBlock (64);
	
	//cudaMemcpyToSymbol (yshift, &yshift_h,  sizeof (int));
	//cudaMemcpyToSymbol (th, &th_h, sizeof (int));
	//cudaMemcpyToSymbol (nr_plane, &nr_plane_h,  sizeof (int));
	//printf("%d %d\n",h,w);
		
	compute_data_cost_init<<<dimGrid,dimBlock>>>(left,right,h,w,scale,nr_plane_in,cost_max_data_term,m_w,m_selected_disparity_pyramid,m_data_cost_selected,nr_plane_h,yshift_h,th_h );

	
	
}
__device__ void 
cost_per_pixel_rgb(int *selected_disparity, uchar3 left_x,unsigned char *right, int cost_max_data_term, int *m_temp,int m_w, int xi , int yi, int nr_plane_h, int th_h)
{
	for(int d=threadIdx.z;d<nr_plane_h;d+=blockDim.z)
	{
		int xr=xi-selected_disparity[d];
		int aux/*m_temp[d]+*/=(selected_disparity[d]<th_h||xr<0) ? cost_max_data_term :compute_data_cost_per_pixel_rgb(left_x,right + 3*xr,cost_max_data_term);
		atomicAdd(&m_temp[d],aux);
	}
}

__global__ void 
compute_data_cost(unsigned char*left,unsigned char*right,int h,int w,int scale,
  int cost_max_data_term,short *m_selected_disparity_pyramid, int m_w, short  *m_data_cost, int nr_plane_h, int yshift_h, int yshift2_h, int th_h,int h2_h,int w2_h)
{
	__shared__ int m_temp[32];
	__shared__ int selected_disparity[32];


	int y=blockIdx.y;
	uchar3 left_x;
	if(y<h)
	{
		int x=blockIdx.x;
		if(x<w)
		{
			int i=threadIdx.z*blockDim.y*blockDim.x+blockDim.x*threadIdx.y+threadIdx.x;
			if(i<nr_plane_h){
				selected_disparity[i]=m_selected_disparity_pyramid[(__mul24(min((y>>1),h2_h),yshift2_h)+__mul24(min((x>>1),w2_h),nr_plane_h))+i];
				m_temp[i]=0;
			}
			__syncthreads();

			int yt=((y+1)<<scale);
			int xt=((x+1)<<scale);
			int x0=(x<<scale);
	
			for(int yi=(y<<scale)+threadIdx.y;yi<yt;yi+=blockDim.y)
			{
				int aux=__mul24(3*yi,m_w);
				unsigned char *leftAux = left + aux;
				unsigned char *rightAux = right + aux;
				for(int xi=x0+threadIdx.x;xi<xt;xi+=blockDim.x)
				{
					int aux=__mul24(3,xi);
					left_x.x=leftAux[aux];
					left_x.y=leftAux[aux+1];
					left_x.z=leftAux[aux+2];
					cost_per_pixel_rgb(selected_disparity,left_x, rightAux, cost_max_data_term, m_temp, m_w, xi , yi, nr_plane_h,th_h);
				}
			}
		
			__syncthreads();
			if(i<nr_plane_h)
				m_data_cost[(__mul24(y,yshift_h)+__mul24(x,nr_plane_h))+i]=m_temp[i];
		}
	}
}

__device__ void 
cost_per_pixel_rgb2(short *selected_disparity, uchar3 left_x,unsigned char *right, int cost_max_data_term, short *m_temp,int m_w, int xi , int yi, int nr_plane_h, int th_h)
{
	for(int d=0;d<nr_plane_h;d++)
	{
		int xr=xi-selected_disparity[d];
		m_temp[d]+=(selected_disparity[d]<th_h||xr<0) ? cost_max_data_term :compute_data_cost_per_pixel_rgb(left_x,right + 3*xr,cost_max_data_term);
		
	}
}

__global__ void 
compute_data_cost2(unsigned char*left,unsigned char*right,int h,int w,int scale,
  int cost_max_data_term,short *m_selected_disparity_pyramid, int m_w, short  *m_data_cost, int nr_plane_h, int yshift_h, int yshift2_h, int th_h,int h2_h,int w2_h)
{
	__shared__ short m_temp[4096];
	//__shared__ short selected_disparity[64][32];


	int y=blockIdx.x;
	uchar3 left_x;
	if(y<h)
	{
		int yt=((y+1)<<scale);
		int y0=(y<<scale);
		short *temp=&m_temp[32*threadIdx.x];
		
		for(int x=threadIdx.x;x<w;x+=blockDim.x)
		{
			//int i=threadIdx.z*blockDim.y*blockDim.x+blockDim.x*threadIdx.y+threadIdx.x;
						
			short *selected_disparity = m_selected_disparity_pyramid+(__mul24(min((y>>1),h2_h),yshift2_h)+__mul24(min((x>>1),w2_h),nr_plane_h));
			for(int i=0;i<nr_plane_h;i++)
			{
				temp[i]=0;
			}
						
			int xt=((x+1)<<scale);
			int x0=(x<<scale);
	
			for(int yi=y0;yi<yt;yi++)
			{
				int aux=__mul24(3*yi,m_w);
				unsigned char *leftAux = left + aux;
				unsigned char *rightAux = right + aux;
				for(int xi=x0;xi<xt;xi++)
				{
					int aux=__mul24(3,xi);
					left_x.x=leftAux[aux];
					left_x.y=leftAux[aux+1];
					left_x.z=leftAux[aux+2];
					cost_per_pixel_rgb2(selected_disparity,left_x, rightAux, cost_max_data_term, temp, m_w, xi , yi, nr_plane_h,th_h);
				}
			}
		
			for(int i=0;i<nr_plane_h;i++)
			{
				m_data_cost[(__mul24(y,yshift_h)+__mul24(x,nr_plane_h))+i]=temp[i];
			}
		}
	}
}

extern "C" void 
compute_data_cost_GPU(unsigned char*left,unsigned char*right,int h,int w,int scale,int nr_plane_h,int cost_max_data_term,short *m_selected_disparity_pyramid, 
		      int m_w, short  *m_data_cost, int m_nr_plane,int *m_h_pyramid, int *m_w_pyramid)
{

	int yshift_h=w*nr_plane_h;
	int yshift2_h=(w>>1)*nr_plane_h;
	int th_h=(int)m_nr_plane*0.2;

	int h2_h=m_h_pyramid[scale+1]-1;
	int w2_h=m_w_pyramid[scale+1]-1;
	
	//cudaMemcpyToSymbol (yshift, &yshift_h,  sizeof (int));
	//cudaMemcpyToSymbol (yshift2, &yshift2_h,  sizeof (int));
	//cudaMemcpyToSymbol (th, &th_h, sizeof (int));
	//cudaMemcpyToSymbol (h2_, &h2_h, sizeof (int));
	//cudaMemcpyToSymbol (w2_, &w2_h, sizeof (int));
	//cudaMemcpyToSymbol (nr_plane, &nr_plane_h,  sizeof (int));
	
	dim3 dimBlock (2,2,nr_plane_h);
	dim3 dimGrid (w,h);
	
	if(nr_plane_h>8)
	compute_data_cost<<<dimGrid ,dimBlock>>>(left,right, h, w, scale, cost_max_data_term, m_selected_disparity_pyramid, m_w, 
				    m_data_cost, nr_plane_h,yshift_h,yshift2_h,th_h,h2_h, w2_h);
	else
	compute_data_cost2<<<h ,128>>>(left,right, h, w, scale, cost_max_data_term, m_selected_disparity_pyramid, m_w, 
						m_data_cost, nr_plane_h,yshift_h,yshift2_h,th_h,h2_h, w2_h);
	
}
__device__ void 
init_temp(short *m_temp,short *m_temp2,short *m_data_cost,short *p21,short *p22,short *p23,short *p24,short *disparity_pyramid,int nr_plane2_h, int xld_finer)
{
	short *data_cost=m_data_cost+xld_finer;
	
	for(int d=threadIdx.x;d<nr_plane2_h;d+=blockDim.x)
	{
		m_temp[d]=disparity_pyramid[d];
		m_temp2[d]=data_cost[d]+p21[d]+p22[d]+p23[d]+p24[d];
	}
}



__global__ void 
init_message(int h,int w,int scale_index, short *m_message, short *m_data_cost,short *m_selected_disparity_pyramid,
short *m_data_cost_selected, int nr_plane_h, int nr_plane2_h, int xshift_h, int yshift_h,int idmax_h,int yshiftd_h, int h2_H, int w2_H, int x2shift_h, int y2shift_h, int yshiftd2_h, int yshiftd_finer_h)
{

	__shared__ short m_temp2[256];
	__shared__ short m_temp[256];

	int y=h-1-blockIdx.y;
	if(y>=0)
	{


		
		//int yl2u=max(0,(y>>1)-1)*y2shift_h;
		//int yl2d=min(h2_H,(y>>1)+1)*y2shift_h;
		//int yld2=min(h2_H,(y>>1))*yshiftd2_h;
		
		int x=w-1-blockIdx.x;
		if(x>=0)
		{
			int yl2=min(h2_H,(y>>1))*y2shift_h;

			int xl=x*xshift_h+y*yshift_h;
			int xl2=min(w2_H,(x>>1))*x2shift_h;
			//int xl2l=max(0,(x>>1)-1)*x2shift_h;
			//int xl2r=min(w2_H,(x>>1)+1)*x2shift_h;
			int xld=x*nr_plane_h+y*yshiftd_h;
			int xld_finer=x*nr_plane2_h+y*yshiftd_finer_h;
			//int xld2=min(w2_H,(x>>1))*nr_plane2_h;


			short *p21,*p22,*p23,*p24;

			
			p21=&(m_message[max(0,(y>>1)-1)*y2shift_h+xl2+2*nr_plane2_h]);			
			p22=&(m_message[yl2+max(0,(x>>1)-1)*x2shift_h+3*nr_plane2_h]);
			p23=&(m_message[min(h2_H,(y>>1)+1)*y2shift_h+xl2]);
			p24=&(m_message[yl2+min(w2_H,(x>>1)+1)*x2shift_h+nr_plane2_h]);

			init_temp(m_temp,m_temp2,m_data_cost,
				p21,
				p22,
				p23,
				p24,
				m_selected_disparity_pyramid + min(h2_H,(y>>1))*yshiftd2_h + min(w2_H,(x>>1))*nr_plane2_h,
				nr_plane2_h,xld_finer);


			__syncthreads();
			if(threadIdx.x==0)
			qx_get_first_k_element_increase(&(m_message[max(0,xl-yshift_h+2*nr_plane_h)]),
							&(m_message[max(0,xl-xshift_h+3*nr_plane_h)]),
							&(m_message[min(idmax_h,xl+yshift_h)]),
							&(m_message[min(idmax_h,xl+xshift_h+nr_plane_h)]),
							p21,
							p22,
							p23,
							p24,
							&(m_data_cost_selected[xld]),
							&(m_selected_disparity_pyramid[xld]),
							&(m_data_cost[xld_finer]),
							m_temp2,m_temp,nr_plane_h,
							nr_plane2_h);
			//__syncthreads();
		}
	}
	
}



extern "C"
void init_message_GPU (int scale_index,int *m_max_nr_plane_pyramid, int *m_h_pyramid, int *m_w_pyramid,int nr_neighbor,
		  short * m_message, short *m_data_cost,short *m_selected_disparity_pyramid,short *m_data_cost_selected)
{
        
        int h_h=m_h_pyramid[scale_index];
        int w_h=m_w_pyramid[scale_index];
        int nr_plane_h=m_max_nr_plane_pyramid[scale_index];
        int xshift_h=nr_neighbor*nr_plane_h;
        int yshift_h=w_h*xshift_h;
        int idmax_h=h_h*yshift_h-1;
        int yshiftd_h=w_h*nr_plane_h;


        int h2_h=m_h_pyramid[scale_index+1];
        int w2_h=m_w_pyramid[scale_index+1];
        int h2_H=h2_h-1;
        int w2_H=w2_h-1;
        int nr_plane2_h=m_max_nr_plane_pyramid[scale_index+1];
        int x2shift_h=nr_neighbor*nr_plane2_h;
        int y2shift_h=w2_h*x2shift_h;

        int yshiftd2_h=w2_h*nr_plane2_h;
        int yshiftd_finer_h=w_h*nr_plane2_h;


        //cudaMemcpyToSymbol (nr_plane, &nr_plane_h,  sizeof (int));
        //cudaMemcpyToSymbol (xshift, &xshift_h,  sizeof (int));
       // cudaMemcpyToSymbol (yshift, &yshift_h,  sizeof (int));
        //cudaMemcpyToSymbol (idmax, &idmax_h,  sizeof (int));
        //cudaMemcpyToSymbol (yshiftd, &yshiftd_h,  sizeof (int));
        //cudaMemcpyToSymbol (h2, &h2_h,  sizeof (int));
        //cudaMemcpyToSymbol (w2, &w2_h,  sizeof (int));
       // cudaMemcpyToSymbol (h2_, &h2_H,  sizeof (int));
      //  cudaMemcpyToSymbol (w2_, &w2_H,  sizeof (int));
        //cudaMemcpyToSymbol (nr_plane2, &nr_plane2_h,  sizeof (int));
       // cudaMemcpyToSymbol (x2shift, &x2shift_h,  sizeof (int));
      //  cudaMemcpyToSymbol (y2shift, &y2shift_h,  sizeof (int));

//         cudaMemcpyToSymbol (yshiftd2, &yshiftd2_h,  sizeof (int));
//         cudaMemcpyToSymbol (yshiftd_finer, &yshiftd_finer_h,  sizeof (int));
	
	dim3 dimGrid (w_h,h_h);
	init_message<<<dimGrid,32>>>(h_h,w_h,scale_index, m_message, m_data_cost,m_selected_disparity_pyramid,m_data_cost_selected,
				     nr_plane_h,nr_plane2_h,xshift_h,yshift_h,idmax_h,yshiftd_h,h2_H,w2_H,x2shift_h,y2shift_h,yshiftd2_h,yshiftd_finer_h);


}



__device__  int
bpstereo_vec_min(short  *c0, short *p1, short *p2, short *p3, short *p4, int len)
{
	
	int m_temp=c0[0]+p1[0]+p2[0]+p3[0]+p4[0];
	int min_val=m_temp;

	int min_pos = 0;
 
	for ( int i= 1; i<len; i++)
	{
		m_temp=c0[i]+p1[i]+p2[i]+p3[i]+p4[i];
		if (m_temp<min_val)
		{
			min_val=m_temp;	
			min_pos = i;
		}
	}

	return min_pos;
}

// int bpstereo_vec_min(int *in, int len )
// {
// 	int min_val=in[0];
// 	int min_pos= 0;
// 
// 	for ( int i= 1; i<len; i++) 
// 		if (in[i]<min_val)
// 		{
// 			min_val=in[i];	
// 			min_pos= i;
// 		}
// 	return min_pos;
// }


__global__ void
compute_disparity(int w, int h, short *disparity,int scale,short *m_selected_disparity_pyramid, short *m_data_cost_selected, short *m_message, int nr_plane_h, int xshift_h, int yshift_h,int yshiftd_h)
{
	int d0;

	int y=1+blockIdx.x;
	if(y<h-1)
	{
		int yl=y*yshift_h;
		int yld=y*yshiftd_h;
		for(int x=1+threadIdx.x;x<w-1;x+=blockDim.x)
		{
			int xl=x*xshift_h+yl;
			int xld=x*nr_plane_h+yld;

			d0=bpstereo_vec_min(&(m_data_cost_selected[xld]),
					&(m_message[xl-yshift_h+2*nr_plane_h]),
					&(m_message[xl-xshift_h+3*nr_plane_h]),
					&(m_message[xl+yshift_h]),
					&(m_message[xl+xshift_h+nr_plane_h]),
					 nr_plane_h);
			disparity[y*w+x]=m_selected_disparity_pyramid[xld+d0];
		}
	}
	
}

__global__ void 
disparity_ajust(int m_w,int m_h,short *disp)
{
	int tid=blockIdx.x*blockDim.x+threadIdx.x;
	int offset=gridDim.x*blockDim.x;
// 	unsigned char *dstDisp=disp+(m_h-1)*m_w;
// 	unsigned char *srcDisp=disp+(m_h-2)*m_w;
// 	unsigned char *dstDisp2=disp;
// 	unsigned char *srcDisp2=disp+m_w;

	short *dstDisp=disp+(m_h-1)*m_w;
	short *srcDisp=disp+(m_h-2)*m_w;
	short *dstDisp2=disp;
	short *srcDisp2=disp+m_w;


	
	for(int i=tid;i<m_w;i+=offset)
	{
		dstDisp[i]=srcDisp[i];
		dstDisp2[i]=srcDisp2[i];
	}
	int aux1=m_w-1;
	int aux2=m_w-2;

	for(int y=tid;y<m_h;y+=offset) 
	{
		int aux3=y*m_w;
		disp[aux3]=disp[aux3+1];
		disp[aux3+aux1]=disp[aux3+aux2];
	}

}


extern "C"
void compute_disparity_GPU(short *disparity_GPU, short *disparity,int scale, int *m_h_pyramid, int *m_w_pyramid, int nr_neighbor, int *m_max_nr_plane_pyramid,short *m_selected_disparity_pyramid, short *m_data_cost_selected, short *m_message)
{
	int h_h=m_h_pyramid[scale];
	int w_h=m_w_pyramid[scale];
	int nr_plane_h=m_max_nr_plane_pyramid[scale];
	
	int xshift_h=nr_neighbor*nr_plane_h;
	int yshift_h=w_h*xshift_h;
	int yshiftd_h=w_h*nr_plane_h;


	compute_disparity<<<h_h,128>>>(w_h,h_h,disparity_GPU, scale, m_selected_disparity_pyramid, m_data_cost_selected, m_message,nr_plane_h,xshift_h ,yshift_h, yshiftd_h );
	disparity_ajust<<<32,32>>>(w_h,h_h,disparity_GPU);


}

extern "C"
int GPUinit(int m_h,int m_w,int m_max_nr_message, int m_max_nr_plane_pyramid)
{
	allocGPUData((void **)&m_data_cost_selected_d,sizeof(short)*m_h*m_w*m_max_nr_plane_pyramid*2);
	allocGPUData((void **)&m_message_d,sizeof(short)*m_max_nr_message);
	allocGPUData((void **)&m_selected_disparity_pyramid_d,sizeof(short)*m_h*m_w*m_max_nr_plane_pyramid);
	allocGPUData((void **)&left_d,m_h*m_w*3*sizeof(unsigned char));
	allocGPUData((void **)&m_data_cost_d,sizeof(short)*2*m_h*m_w*m_max_nr_plane_pyramid);
	allocGPUData((void **)&right_d,m_h*m_w*3*sizeof(unsigned char));
	allocGPUData((void **)&disp_d,m_h*m_w*sizeof(short));
      
	return 0;

}

extern "C"
int GPUdelete()
{
	cudaFree(m_data_cost_selected_d);
	cudaFree(left_d);
	cudaFree(m_message_d);
	cudaFree(m_selected_disparity_pyramid_d);
	cudaFree(m_data_cost_d);
	cudaFree(right_d);

	return 0;

}

__global__ void normalizeRGB(unsigned char *img,  int width, int height)
{
	int N=width*height;
	int offset=blockDim.x*gridDim.x;
	for(int i=blockIdx.x*blockDim.x+threadIdx.x;i<N;i+=offset)
	{
		int r=img[3*i];
		int g=img[3*i+1];
		int b=img[3*i+2];
		r+=g+b;
		
		img[3*i]/=r;
		img[3*i+1]/=r;
		img[3*i+2]/=r;
	}
  
}

extern "C"
short* disparity_GPU(unsigned char*left,unsigned char*right, int m_h, int m_w, int m_nr_plane,int max_nr_plane, int m_discontinuity_cost_single_jump, int max_nr_jump, 
	 int m_cost_max_data_term, int m_cost_max_discontinuity,int m_max_nr_message, int m_nr_scale,int m_nr_neighbor,int *m_iteration, int *m_max_nr_plane_pyramid, 
	 int *m_h_pyramid,int *m_w_pyramid, short *disp)
{

	int j=0;


 	GPUcopy(left_d, left,  m_h*m_w*3*sizeof(unsigned char), 1);
	GPUcopy(right_d, right,  m_h*m_w*3*sizeof(unsigned char), 1);

	//normalizeRGB<<<128,256>>>(left_d, m_w, m_h);
	//normalizeRGB<<<128,256>>>(right_d, m_w, m_h);
	
				
	for(int i=m_nr_scale-1;i>=0;i--)
	{

		  if(i==m_nr_scale-1)
		  {
			  compute_data_cost_init_GPU(left_d,right_d,m_h_pyramid[i],m_w_pyramid[i],i,m_max_nr_plane_pyramid[i],m_nr_plane,m_cost_max_data_term, m_w,m_selected_disparity_pyramid_d, m_data_cost_selected_d);
		  }else{
			compute_data_cost_GPU(left_d,right_d,m_h_pyramid[i],m_w_pyramid[i],i,m_max_nr_plane_pyramid[i+1],m_cost_max_data_term,m_selected_disparity_pyramid_d, m_w, m_data_cost_d, m_nr_plane, m_h_pyramid, m_w_pyramid);
			  
			
			
			init_message_GPU (i,m_max_nr_plane_pyramid, m_h_pyramid, m_w_pyramid, m_nr_neighbor,
						m_message_d, m_data_cost_d, m_selected_disparity_pyramid_d, m_data_cost_selected_d);
			

		  }

		

		for(j=0;j<5;j++) 
		{

			compute_message_GPU(m_h_pyramid[i],m_w_pyramid[i],m_max_nr_plane_pyramid[i],i,m_data_cost_selected_d,m_message_d,
					    m_selected_disparity_pyramid_d,m_cost_max_discontinuity,m_nr_neighbor, 
					    m_discontinuity_cost_single_jump);

		}
	}
 

	
	compute_disparity_GPU(disp_d, disp,0, m_h_pyramid, m_w_pyramid, m_nr_neighbor, m_max_nr_plane_pyramid, m_selected_disparity_pyramid_d, m_data_cost_selected_d, m_message_d);
	
	cudaMemcpy(disp,disp_d, m_h*m_w*sizeof(short),cudaMemcpyDeviceToHost);
	
	//GPUcopy(disp, disp_d,  m_h*m_w*sizeof(short), 0);

// 	int j=0;
// 	int h,w,iteration;
// 	int nr_plane_h,xshift_h,yshift_h,yshiftd_h;
// 	int th_h= (int) m_nr_plane*0.2;
// 	dim3 dimGrid;
// 	dim3 dimBlock;
// 	
// 	GPUcopy(left_d, left, m_h*m_w*3*sizeof(unsigned char), 1);
// 	GPUcopy(right_d, right, m_h*m_w*3*sizeof(unsigned char), 1);
// 
// 	int i=m_nr_scale-1;
// 
// 	 
// 
// 	iteration=m_iteration[i];
// 	w=m_w_pyramid[i];
// 	h=m_h_pyramid[i];
// 	nr_plane_h=m_max_nr_plane_pyramid[i];
// 	yshift_h=m_w_pyramid[i]*nr_plane_h;
// 
// 	dimGrid.x=w;
// 	dimGrid.y=h;
//       
// 	dimBlock.x=64;
// 
// 	compute_data_cost_init<<<dimGrid,dimBlock>>>(left_d,right_d,h,w,i,m_nr_plane,m_cost_max_data_term,m_w,m_selected_disparity_pyramid_d,m_data_cost_selected_d,nr_plane_h,yshift_h,th_h );
// 
// 	dimBlock.x=4;
// 	dimBlock.y=nr_plane_h>=8?nr_plane_h:8;
// 	dimGrid.x= w;
// 	dimGrid.y=h;
// 	yshift_h=w*m_nr_neighbor*nr_plane_h;
// 	xshift_h=m_nr_neighbor*nr_plane_h;
// 	yshiftd_h=w*nr_plane_h;
// 
// 
// 	for(j=0;j<iteration;j++)
// 	{
// 	
// 		for(int k=0;k<2;k++) 
// 		{
// 			compute_message<<<dimGrid,dimBlock>>>(h-1, w-1,j,m_data_cost_selected_d,m_message_d,m_selected_disparity_pyramid_d,k,
// 					yshift_h,xshift_h,yshiftd_h,nr_plane_h,m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
// 		}
// 	}
// 
// 	
// 	for(i--;i>=0;i--)
// 	{
// 		compute_data_cost_GPU(left_d,right_d,m_h_pyramid[i],m_w_pyramid[i],i,m_max_nr_plane_pyramid[i+1],m_cost_max_data_term,m_selected_disparity_pyramid_d, 
// 		  m_w, m_data_cost_d, m_nr_plane, m_h_pyramid, m_w_pyramid);
// 		  
// 		
// 		
// 		init_message_GPU (i,m_max_nr_plane_pyramid, m_h_pyramid, m_w_pyramid, m_nr_neighbor,
// 					m_message_d, m_data_cost_d, m_selected_disparity_pyramid_d, m_data_cost_selected_d);
// 			
// 			
// 			
// 
// 		for(j=0;j<m_iteration[i];j++) 
// 			compute_message_GPU(m_h_pyramid[i],m_w_pyramid[i],m_max_nr_plane_pyramid[i],i,m_data_cost_selected_d,m_message_d,
// 					     m_selected_disparity_pyramid_d,m_cost_max_discontinuity,m_nr_neighbor, 
// 					     m_discontinuity_cost_single_jump);
// 			
// 
// 	}
// 	
// 	h=m_h_pyramid[0];
// 	w=m_w_pyramid[0];
// 	nr_plane_h=m_max_nr_plane_pyramid[0];
// 	
// 	xshift_h=m_nr_neighbor*nr_plane_h;
// 	yshift_h=w*xshift_h;
// 	yshiftd_h=w*nr_plane_h;
// 
// 	compute_disparity<<<h,128>>>(w,h,disp_d, 0, m_selected_disparity_pyramid_d, m_data_cost_selected_d, m_message_d,nr_plane_h,xshift_h ,yshift_h, yshiftd_h );
// 	disparity_ajust<<<32,32>>>(w,h,disp_d);
// 	
	//GPUcopy(disp, disp_d,  m_h*m_w*sizeof(short), 0);

	
	return(disp);
}






