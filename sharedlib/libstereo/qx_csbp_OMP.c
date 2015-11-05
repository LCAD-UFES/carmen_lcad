#include "qx_csbp.h"


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

short* disparityOMP(unsigned char*left,unsigned char*right, int m_h, int m_w, int m_nr_plane,int max_nr_plane, int m_discontinuity_cost_single_jump, int max_nr_jump, int m_cost_max_data_term)
{

	//int *m_data_cost_selected=&(m_data_cost[(m_h*m_w*m_max_nr_plane_pyramid[0])]);
	//gettimeofday(&inicioTotal, NULL);
	memset(m_message,0,sizeof(int)*(m_max_nr_message>>1));
	memset(m_data_cost,0,sizeof(int)*m_h*m_w*m_max_nr_plane_pyramid[0]*2);
	memset(m_data_cost_selected,0,sizeof(int)*m_h*m_w*m_max_nr_plane_pyramid[0]*2);
	int j=0;

/*  	#pragma omp parallel num_threads(8) default(none) private (j) shared(disp,right,left,\
 	 m_h, m_w, m_nr_plane,max_nr_plane, m_discontinuity_cost_single_jump, max_nr_jump, \
  	 m_cost_max_data_term, m_cost_max_discontinuity, m_max_nr_message, m_nr_scale, m_nr_neighbor,m_iteration, m_max_nr_plane_pyramid, \
  	 m_h_pyramid, m_w_pyramid, m_data_cost, m_temp3, m_temp4, m_message, m_selected_disparity_pyramid,m_data_cost_selected)
*/	{
	for(int i=m_nr_scale-1;i>=0;i--)
	{
		if(i==(m_nr_scale-1))
		{

			compute_data_cost_init(left,right,m_h_pyramid[i],m_w_pyramid[i],i,m_max_nr_plane_pyramid[i],
					       m_nr_plane,m_cost_max_data_term, m_temp3, m_w, m_h, m_temp4,
					       m_selected_disparity_pyramid,m_data_cost_selected);
		}
		else
		{

			compute_data_cost(left,right,m_h_pyramid[i],m_w_pyramid[i],i,m_max_nr_plane_pyramid[i+1],m_cost_max_data_term,m_nr_plane,
					  m_h_pyramid, m_w_pyramid, m_temp3, m_selected_disparity_pyramid, m_data_cost, m_w);


			init_message(i, m_max_nr_plane_pyramid, m_h_pyramid,m_w_pyramid, m_nr_neighbor, m_temp3, m_temp4, m_selected_disparity_pyramid,
				     m_message, m_data_cost, m_data_cost_selected);
		}

		for(j=0;j<(int)m_iteration[i];j++)
			compute_message(m_h_pyramid[i],m_w_pyramid[i],m_max_nr_plane_pyramid[i],i,
					m_temp3, (int)m_cost_max_discontinuity,m_discontinuity_cost_single_jump,
					m_nr_neighbor, m_data_cost_selected,m_message, m_selected_disparity_pyramid);

	}
	}
	compute_disparity(disp,0,m_h_pyramid, m_w_pyramid, m_max_nr_plane_pyramid, m_nr_neighbor,
			  m_temp3, m_data_cost_selected, m_message,m_selected_disparity_pyramid);

	//gettimeofday(&fim, NULL);
	//timevalSubtract(&tempoDec, &fim, &inicioTotal);

	//printf("Tempo Final: %d.%06d s\n" ,(int) tempoDec.tv_sec, (int) tempoDec.tv_usec);

      return(disp);
}


void init_message(int scale_index, int *m_max_nr_plane_pyramid, int *m_h_pyramid,int *m_w_pyramid,
		  int m_nr_neighbor,int *m_temp3, int *m_temp4,int *m_selected_disparity_pyramid, int *m_message, int *m_data_cost, int *m_data_cost_selected)
{
	int max_nr_plane_pyramid=m_max_nr_plane_pyramid[scale_index];
	int h=m_h_pyramid[scale_index];
	int w=m_w_pyramid[scale_index];
	int nr_plane=m_max_nr_plane_pyramid[scale_index];
	int xshift=m_nr_neighbor*nr_plane;
	int yshift=w*xshift;
	int idmax=h*yshift-1;
	int yshiftd=w*nr_plane;

	int h2=m_h_pyramid[scale_index+1];
	int w2=m_w_pyramid[scale_index+1];
	int h2_=h2-1;
	int w2_=w2-1;
	int nr_plane2=m_max_nr_plane_pyramid[scale_index+1];
	int x2shift=m_nr_neighbor*nr_plane2;
	int y2shift=w2*x2shift;
//	int id2max=h2*y2shift-1;
	int yshiftd2=w2*nr_plane2;
	int yshiftd_finer=w*nr_plane2;
	//#pragma omp parallel num_threads(16) shared(scale_index,max_nr_plane_pyramid,h,w,nr_plane,xshift,yshift,idmax,yshiftd,h2,w2,h2_,w2_,nr_plane2,x2shift,y2shift,id2max,yshiftd2,yshiftd_finer)
	{
	  int thread_id = 0;//omp_get_thread_num();
		int *aux_m_temp=&(m_temp3[thread_id*nr_plane2]);
		int *aux_m_temp2=&(m_temp4[thread_id*nr_plane2]);


	for(int y=h-1;y>=0;y--)
	{
		int yl=y*yshift;
		int yld=y*yshiftd;
		int yld_finer=y*yshiftd_finer;

		int yl2=min(h2_,(y>>1))*y2shift;
		int yl2u=max(0,(y>>1)-1)*y2shift;
		int yl2d=min(h2_,(y>>1)+1)*y2shift;
		int yld2=min(h2_,(y>>1))*yshiftd2;
		//#pragma omp for
		for(int x=w-1;x>=0;x--)
		{
			int xl=x*xshift;
			int xl2=min(w2_,(x>>1))*x2shift;
			int xl2l=max(0,(x>>1)-1)*x2shift;
			int xl2r=min(w2_,(x>>1)+1)*x2shift;
			int xld=x*nr_plane;
			int xld_finer=x*nr_plane2;
			int xld2=min(w2_,(x>>1))*nr_plane2;
			memcpy(aux_m_temp,&(m_selected_disparity_pyramid[yld2+xld2]),sizeof(int)*nr_plane2);
			int *p1,*p2,*p3,*p4,*p21,*p22,*p23,*p24;

			p1=&(m_message[max(0,yl-yshift+xl+2*nr_plane)]);
			p2=&(m_message[max(0,yl+xl-xshift+3*nr_plane)]);
			p3=&(m_message[min(idmax,yl+yshift+xl)]);
			p4=&(m_message[min(idmax,yl+xl+xshift+nr_plane)]);
			p21=&(m_message[yl2u+xl2+2*nr_plane2]);
			p22=&(m_message[yl2+xl2l+3*nr_plane2]);
			p23=&(m_message[yl2d+xl2]);
			p24=&(m_message[yl2+xl2r+nr_plane2]);

			for(int d=0;d<nr_plane2;d++)
			{
				aux_m_temp2[d]=m_data_cost[yld_finer+xld_finer+d]+p21[d]+p22[d]+p23[d]+p24[d];
			}

			int *data_cost_selected=&(m_data_cost_selected[yld+xld]);
			int *disparity_selected=&(m_selected_disparity_pyramid[yld+xld]);
			int *data_cost=&(m_data_cost[yld_finer+xld_finer]);
			qx_get_first_k_element_increase(p1,p2,p3,p4,p21,p22,p23,p24,data_cost_selected,disparity_selected,data_cost,
				aux_m_temp2,aux_m_temp,max_nr_plane_pyramid,m_max_nr_plane_pyramid[scale_index+1]);
		}
	}
	}
}
void qx_get_first_k_element_increase(int*q1,int*q2,int*q3,int*q4,int*p1,int*p2,int*p3,int*p4,
					int*cost,int*disp,int*coarse,int*in,int*disp_in,int len,int len_in)
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
inline int compute_data_cost_per_pixel_rgb(unsigned char left[3],unsigned char right[3],int cost_max_data_term)
{
	float tr=0.299f*abs(left[0]-right[0]);
	float tg=0.587f*abs(left[1]-right[1]);
	float tb=0.114f*abs(left[2]-right[2]);
	return(min((int)(tr+tg+tb+0.5),cost_max_data_term));
}
void compute_data_cost_init(unsigned char*left,unsigned char*right,
		int h,int w,int scale,int nr_plane,int nr_plane_in,int cost_max_data_term, int *m_temp3,
		int m_w,int m_h,int *m_temp4, int *m_selected_disparity_pyramid,int *m_data_cost_selected)

{
	int y0,yt,x0,xt; int*selected_disparity,*data_cost;
	int yshift=w*nr_plane;
	int th= (int) nr_plane_in*0.2;


	//int th=0;

	//#pragma omp parallel num_threads(16)  default(none) shared(left, right, h, w, scale, nr_plane, nr_plane_in, cost_max_data_term,yshift,th) private(y0,yt,x0,xt,selected_disparity,data_cost)
	{
		int thread_id = 0;//omp_get_thread_num();
		int *aux_m_temp=&(m_temp3[thread_id*nr_plane_in]);
		int *aux_m_temp2=&(m_temp4[thread_id*nr_plane_in]);
	//	printf("%d %d %d %d\n",h,w,nr_plane_in,nr_plane);

	//#pragma omp for
	for(int y=0;y<h;y++)
	//for(int y=h-1;y>=0;y--)
	{

		int yl=y*yshift;
		y0=(y<<scale); yt=((y+1)<<scale);

		//#pragma omp for
		for(int x=0;x<w;x++)
		//for(int x=w-1;x>=0;x--)
		{
			selected_disparity=&(m_selected_disparity_pyramid[yl+x*nr_plane]);
			data_cost=&(m_data_cost_selected[yl+x*nr_plane]);
			x0=(x<<scale); xt=((x+1)<<scale);
			memset(aux_m_temp,0,sizeof(int)*nr_plane_in);
			//printf("%d %d\n",x0,xt);

			for(int yi=y0;yi<yt;yi++)
			{
				unsigned char*left_x=&(left[3*(yi*m_w+x0)]);
				for(int xi=x0;xi<xt;xi++)
				{

					for(int d=0;d<nr_plane_in;d++)
					{
						int xr=xi-d;
						if(d<th||xr<0) aux_m_temp[d]+=cost_max_data_term;//xr=xi;
						else aux_m_temp[d]+=compute_data_cost_per_pixel_rgb(left_x,&(right[3*(yi*m_w+xr)]),cost_max_data_term);
					}
					left_x+=3;
				}
			}
			qx_get_first_k_element_increase_special(data_cost,selected_disparity,aux_m_temp,aux_m_temp2,nr_plane,nr_plane_in);
			//memcpy(data_cost,m_temp,sizeof(int)*nr_plane); for(int d=0;d<nr_plane_in;d++) selected_disparity[d]=d;
			//if(x==37&&y==23)
			//{
			//	for(int i=0;i<nr_plane;i++) printf("%d ",selected_disparity[i]); printf("\n");
			//	int aa=0;
			//}
		}
	}
	}
	//exit(0);
}

void compute_data_cost(unsigned char*left,unsigned char*right,int h,int w,int scale,int nr_plane,int cost_max_data_term, int m_nr_plane,
		       int *m_h_pyramid, int *m_w_pyramid, int *m_temp3,int *m_selected_disparity_pyramid,int *m_data_cost, int m_w)
{
	int y0,yt,x0,xt; int*data_cost,*selected_disparity;
	int yshift=w*nr_plane;
	int yshift2=(w>>1)*nr_plane;
	int th=(int)m_nr_plane*0.2;
	//int th=0;
	int h2_=m_h_pyramid[scale+1]-1;
	int w2_=m_w_pyramid[scale+1]-1;

	//#pragma omp parallel num_threads(16)  default(none) shared(left, right, h, w, scale, nr_plane, cost_max_data_term,yshift,th,w2_,h2_,yshift2) private(y0,yt,x0,xt,selected_disparity,data_cost)
	{
		int thread_id = 0;//omp_get_thread_num();
		int *aux_m_temp=&(m_temp3[thread_id*nr_plane]);
	//#pragma omp for
	for(int y=0;y<h;y++)
	{
		int yl=y*yshift;
		int yl2=min((y>>1),h2_)*yshift2;
		y0=(y<<scale); yt=((y+1)<<scale);
		//printf("%d %d\n",y0,yt);

		for(int x=0;x<w;x++)
		{
			selected_disparity=&(m_selected_disparity_pyramid[yl2+min((x>>1),w2_)*nr_plane]);
			data_cost=&(m_data_cost[yl+x*nr_plane]);
			x0=(x<<scale); xt=((x+1)<<scale);
			//printf("%d %d\n",x0,xt);
			memset(aux_m_temp,0,sizeof(int)*nr_plane);
			for(int yi=y0;yi<yt;yi++)
			{
				unsigned char*left_x=&(left[yi*m_w*3+x0*3]);
				for(int xi=x0;xi<xt;xi++)
				{
					for(int d=0;d<nr_plane;d++)
					{
						int xr=xi-selected_disparity[d];
						if(selected_disparity[d]<th||xr<0) aux_m_temp[d]+=cost_max_data_term; //xr=xi;
						else aux_m_temp[d]+=compute_data_cost_per_pixel_rgb(left_x,&(right[yi*m_w*3+xr*3]),cost_max_data_term);
					}
					left_x+=3;
				}
			}
			memcpy(data_cost,aux_m_temp,sizeof(int)*nr_plane);
		}
	}
	}
}

void qx_get_first_k_element_increase_special(int*cost,int *disp,int*in,int *disp_in,int len,int len_in)
{
	for(int j=0;j<len_in;j++) disp_in[j]=j;
	//printf("%d %d\n",len,len_in);
	for(int i=0;i<len;i++)
	{
		int fmin=in[i]; int id=i;
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

int bpstereo_vec_min(int *in, int len )
{
	int min_val=in[0];
	int min_pos= 0;

	for ( int i= 1; i<len; i++)
		if (in[i]<min_val)
		{
			min_val=in[i];
			min_pos= i;
		}
	return min_pos;
}

void bpstereo_normalize(int *in,int len)
{
	int val=0;
	for(int i=0;i<len;i++)
		val+=in[i];
	val/=len;
	for(int i=0;i<len;i++)
		in[i]-=val;
}

void compute_message_per_pixel_per_neighbor(int *comp_func_sub,int minimum,int *disp_left,int *disp_right,int nr_plane,int scale,
					    int *m_temp3, int m_cost_max_discontinuity, int m_discontinuity_cost_single_jump)
{
	int thread_id = 0;//omp_get_thread_num();
	int *aux_m_temp=&(m_temp3[thread_id*nr_plane]);
	//printf(" %d\n", nr_plane);
	for(int d=0;d<nr_plane;d++)
	{
		int cost_min=minimum+(int)m_cost_max_discontinuity;
		for(int i=0;i<nr_plane;i++)
		{
			cost_min=min(cost_min,comp_func_sub[i]+m_discontinuity_cost_single_jump*abs(disp_left[i]-disp_right[d]));
		}
		aux_m_temp[d]=cost_min;
	}
	memcpy(comp_func_sub,aux_m_temp,sizeof(int)*nr_plane);
	bpstereo_normalize(comp_func_sub,nr_plane);
}

void compute_message_per_pixel(int*c0,int *p0,int *p1,int *p2,int *p3,int *p4,int*d0,int*d1,int*d2,
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
		if(p0u[d]<minimum[0]) minimum[0]=p0u[d];
		if(p0l[d]<minimum[1]) minimum[1]=p0l[d];
		if(p0d[d]<minimum[2]) minimum[2]=p0d[d];
		if(p0r[d]<minimum[3]) minimum[3]=p0r[d];
		//m_comp_func_sub_prev[d]=p1[d]+p2[d]+p3[d]+p4[d];
	}
	compute_message_per_pixel_per_neighbor(p0u,minimum[0],d0,d1,nr_plane,scale,m_temp3, (int)m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
	compute_message_per_pixel_per_neighbor(p0l,minimum[1],d0,d2,nr_plane,scale,m_temp3, (int)m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
	compute_message_per_pixel_per_neighbor(p0d,minimum[2],d0,d3,nr_plane,scale,m_temp3, (int)m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
	compute_message_per_pixel_per_neighbor(p0r,minimum[3],d0,d4,nr_plane,scale,m_temp3, (int)m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
}
void compute_message(int h,int w,int nr_plane,int scale,int *m_temp3, int m_cost_max_discontinuity,
		     int m_discontinuity_cost_single_jump, int m_nr_neighbor, int *m_data_cost_selected,
		     int *m_message, int *m_selected_disparity_pyramid )
{
	int i,y,x,yy=h-1,xx=w-1,yl,yld,xl,xld;
	int*c0,*p0,*p1,*p2,*p3,*p4;
	int*d0,*d1,*d2,*d3,*d4;
	int yshift=w*m_nr_neighbor*nr_plane;
	int xshift=m_nr_neighbor*nr_plane;
	int yshiftd=w*nr_plane;

	//#pragma omp parallel num_threads(16) default(none) private(c0,p0,p1,p2,p3,p4,d0,d1,d2,d3,d4,g_val,i,y,x,count,yl,yld,xl,xld) shared(yy,xx,minimum,nr_plane, scale,yshift,xshift,yshiftd)
	{
		//for(i=0;i<32;i++)
// 			minimum[0/*omp_get_thread_num()*/]=(int*)malloc(4*sizeof(int));
		//printf("%d\n",nr_plane);

		for(i=0;i<2;i++)
		{
			//#pragma omp for
			for(y=1;y<yy;y++)
			{
				//printf("%d %d\n",y,x);
				yl=y*yshift;
				yld=y*yshiftd;
				/*for(x=xx-1+(y+i)%2;x>=1;x-=2)*/ for(x=(y+i)%2+1;x<xx;x+=2)
				{
					xl=x*xshift+yl;
					xld=x*nr_plane+yld;
					c0=&(m_data_cost_selected[xld]);
					p0=&(m_message[xl]);
					p1=&(m_message[xl-yshift+2*nr_plane]);
					p2=&(m_message[xl-xshift+3*nr_plane]);
					p3=&(m_message[xl+yshift]);
					p4=&(m_message[xl+xshift+nr_plane]);
					d0=&(m_selected_disparity_pyramid[xld]);
					d1=&(m_selected_disparity_pyramid[xld-yshiftd]);
					d2=&(m_selected_disparity_pyramid[xld-nr_plane]);
					d3=&(m_selected_disparity_pyramid[xld+yshiftd]);
					d4=&(m_selected_disparity_pyramid[xld+nr_plane]);

					compute_message_per_pixel(c0,p0,p1,p2,p3,p4,d0,d1,d2,d3,d4,y,x,nr_plane,scale,m_temp3,(int)m_cost_max_discontinuity,m_discontinuity_cost_single_jump);
				}
			}
		}
	}
	//cout<<count<<"/"<<(yy-1)*(xx-1)<<endl;
}
int compute_disparity(short*disparity, int scale, int *m_h_pyramid, int *m_w_pyramid, int *m_max_nr_plane_pyramid,
		      int m_nr_neighbor, int *m_temp3, int *m_data_cost_selected, int *m_message, int *m_selected_disparity_pyramid)
{
	int h=m_h_pyramid[scale];
	int w=m_w_pyramid[scale];
	memset(disparity,0,sizeof(short)*h*w);
	int nr_plane=m_max_nr_plane_pyramid[scale];
	int d0;
	int*c0,*p1,*p2,*p3,*p4;
	int y,x,d;
	int xshift=m_nr_neighbor*nr_plane;
	int yshift=w*xshift;
	int yshiftd=w*nr_plane;
/* 	#pragma omp parallel num_threads(16) default(none) private(y,x,d,c0,p1,p2,p3,p4,d0) \
 	shared(nr_plane,disparity,scale,yshiftd,xshift,yshift,h,w, m_h_pyramid, m_w_pyramid, m_max_nr_plane_pyramid,\
		      m_nr_neighbor,m_temp3, m_data_cost_selected, m_message,m_selected_disparity_pyramid)
*/	{
		d0=0;
		int thread_id = 0;//omp_get_thread_num();
		int *aux_m_temp=&(m_temp3[thread_id*nr_plane]);
	//#pragma omp for
	for(y=1;y<h-1;y++)
	{
		int yl=y*yshift;
		int yld=y*yshiftd;
		for(x=1;x<w-1;x++)
		{
			int xl=x*xshift;
			int xld=x*nr_plane;
			c0=&(m_data_cost_selected[yld+xld]);

			p1=&(m_message[yl+xl-yshift+2*nr_plane]);
			p2=&(m_message[yl+xl-xshift+3*nr_plane]);
			p3=&(m_message[yl+xl+yshift]);
			p4=&(m_message[yl+xl+xshift+nr_plane]);
			for(d=0;d<nr_plane;d++) aux_m_temp[d]=c0[d]+p1[d]+p2[d]+p3[d]+p4[d];
			d0=bpstereo_vec_min(aux_m_temp,nr_plane);
			disparity[y*w+x]=m_selected_disparity_pyramid[yld+xld+d0];
		}
	}
	}
	//if(thread_id==0)
	//{
		memcpy(&(disparity[(h-1)*w]),&(disparity[(h-2)*w]),sizeof(short)*w);
		memcpy(&(disparity[0]),&(disparity[w]),sizeof(short)*w);
	//}
	//#pragma omp barrier
	//#pragma omp for
	for(int y=0;y<h;y++)
	{
		disparity[y*w+0]=disparity[y*w+1];
		disparity[y*w+w-1]=disparity[y*w+w-2];
	}
	return(0);
}

