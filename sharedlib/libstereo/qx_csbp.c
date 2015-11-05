#include "qx_csbp.h"

#ifndef NO_CUDA
#include "qx_csbp_GPU.h"
#endif
/*extern short  *disp;
extern int *m_h_pyramid;
extern int *m_w_pyramid;
extern int *m_iteration;
extern int *m_max_nr_plane_pyramid;
extern int m_cost_max_discontinuity;
extern int m_max_nr_message;
extern int m_nr_scale;
extern int m_nr_neighbor;
extern int *m_message;
extern int *m_selected_disparity_pyramid;
extern int *m_data_cost;
extern int *m_temp3;
extern int *m_temp4;*/

int timevalSubtract (struct timeval *result, struct timeval *x, struct timeval *y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}



void init_stereo(int h,int w,int nr_plane,int max_nr_plane, int discontinuity_cost_single_jump,
	 int cost_max_data_term, int max_nr_jump,int nr_scales, int *iterations)
{

	(m_cost_max_discontinuity)=max_nr_jump*discontinuity_cost_single_jump*nr_plane/16.0f;
	(m_nr_neighbor)=QX_DEF_BP_NR_NEIGHBOR;

	m_nr_scale=nr_scales;
	m_iteration=iterations;
	if(iterations==NULL)
	{
		m_nr_scale=QX_DEF_BP_NR_SCALES;
		m_iteration=(int *) calloc(m_nr_scale, sizeof(int));
		if((m_nr_scale)>0) m_iteration[0]=QX_DEF_BP_NR_ITER0;
		if((m_nr_scale)>1) m_iteration[1]=QX_DEF_BP_NR_ITER1;
		if((m_nr_scale)>2) m_iteration[2]=QX_DEF_BP_NR_ITER2;
		if((m_nr_scale)>3) m_iteration[3]=QX_DEF_BP_NR_ITER3;
		if((m_nr_scale)>4) m_iteration[4]=QX_DEF_BP_NR_ITER4;
		if((m_nr_scale)>5) m_iteration[5]=QX_DEF_BP_NR_ITER5;
	}
	m_nr_scale=min(m_nr_scale,(int)(log((double)nr_plane)/log(2.0)));

	m_max_nr_plane_pyramid=(int *) calloc(sizeof(int),m_nr_scale);
	m_h_pyramid=(int *) calloc(sizeof(int),m_nr_scale);

	m_w_pyramid=  (int *) calloc(sizeof(int),m_nr_scale);

	m_max_nr_plane_pyramid[0]=max_nr_plane;
	m_h_pyramid[0]=h;
	m_w_pyramid[0]=w;

	for(int i=1;i<(m_nr_scale);i++)
	{
		m_h_pyramid[i]=(m_h_pyramid[i-1]>>1); m_w_pyramid[i]=(m_w_pyramid[i-1]>>1);
		m_max_nr_plane_pyramid[i]=(m_max_nr_plane_pyramid[i-1]<<1);
	}

	m_max_nr_message=h*w*(m_nr_neighbor)*m_max_nr_plane_pyramid[0];


	m_message=(int *) calloc(sizeof(int),m_max_nr_message*2);
	m_selected_disparity_pyramid=(int *) calloc(sizeof(int),h*w*m_max_nr_plane_pyramid[0]);
	m_data_cost=(int *) calloc(sizeof(int),h*w*m_max_nr_plane_pyramid[0]*2);
	m_data_cost_selected=(int *) calloc(sizeof(int),h*w*m_max_nr_plane_pyramid[0]*2);


	m_temp3=(int *) calloc(16,nr_plane*sizeof(int));
	m_temp4=(int *) calloc(16,nr_plane*sizeof(int));

#ifndef NO_CUDA
	GPUinit(h,w,m_max_nr_message,m_max_nr_plane_pyramid[0]);
#endif
	disp=(short *)calloc(h*w,sizeof(short));

}

int freeAllMem()
{

      free(disp);
//       free(m_image_left);
//       free(m_image_right);
      free(m_h_pyramid);
      free(m_w_pyramid);
      free(m_iteration);
      free(m_max_nr_plane_pyramid);
#ifndef NO_CUDA
      GPUdelete();
#endif
      return 0;
}

#ifndef NO_CUDA
short* disparityCUDA(unsigned char*left,unsigned char*right, int m_h, int m_w, int m_nr_plane,int max_nr_plane, int m_discontinuity_cost_single_jump, int max_nr_jump, int m_cost_max_data_term)
{


	//gettimeofday(&inicioTotal, NULL);



	disparity_GPU(left,right,m_h,m_w,m_nr_plane,max_nr_plane,m_discontinuity_cost_single_jump,max_nr_jump,
	 m_cost_max_data_term, m_cost_max_discontinuity,m_max_nr_message, m_nr_scale,m_nr_neighbor,m_iteration, m_max_nr_plane_pyramid,
	 m_h_pyramid,m_w_pyramid,disp);


	//gettimeofday(&fim, NULL);
	//timevalSubtract(&tempoDec, &fim, &inicioTotal);


	//printf("Tempo Final: %d.%06d s\n" ,(int) tempoDec.tv_sec, (int) tempoDec.tv_usec);


      return(disp);
}
#endif

void disparityCSBP(unsigned char *image_left,unsigned char *image_right, int stereo_height, int stereo_width, int stereo_disparity, unsigned char *disparity)
{

  short* tempImage;
  #ifndef NO_CUDA
    tempImage = disparityCUDA(image_left, image_right, stereo_height, stereo_width, stereo_disparity, QX_DEF_BP_NR_PLANE,
        QX_DEF_BP_COST_DISCONTINUITY_SINGLE_JUMP, QX_DEF_BP_MAX_NR_JUMP, QX_DEF_BP_COST_MAX_DATA_TERM);
  #else
    tempImage = disparityOMP(image_left, image_right, stereo_height, stereo_width, stereo_disparity, QX_DEF_BP_NR_PLANE,
        QX_DEF_BP_COST_DISCONTINUITY_SINGLE_JUMP, QX_DEF_BP_MAX_NR_JUMP, QX_DEF_BP_COST_MAX_DATA_TERM);
  #endif

    int counter = 0;
    for (int y = 0; y < stereo_height; y++)
    {
      for(int x = 0; x < stereo_width; x++)
      {
        disparity[counter]=(unsigned char)(tempImage[y*stereo_width+x]);
        counter++;
      }
    }

}

