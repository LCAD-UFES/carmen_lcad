#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <sys/time.h>
#include <carmen/slam6d_messages.h>

__global__ void
computeVmapKernel (unsigned short* depth, unsigned char* color, double* t, double* R, float* vmap, float* cmap, int width, int height, float fx_inv, float fy_inv, float cx, float cy)
{   
    int u = threadIdx.x + blockIdx.x * blockDim.x;
    int v = threadIdx.y + blockIdx.y * blockDim.y;
	int pos = 0;
	float z, vx, vy, vz;

    if (u < width && v < height)
    {
		pos = v * width + u;			//i * width + j; 

		z = depth[pos] / 1000.0f; // load and convert: mm -> meters

		if (z >= 0.5f)
		{
			vx = z * (u - cx) * fx_inv;
			vy = z * (v - cy) * fy_inv;
			vz = z;

			vx = (float)((R[0] * vx + R[1] * vy + R[2] * vz) + t[0]);
			vy = (float)((R[3] * vx + R[4] * vy + R[5] * vz) + t[1]);
			vz = (float)((R[6] * vx + R[7] * vy + R[8] * vz) + t[2]);

			vmap[3 * pos] = vx;
			vmap[3 * pos + 1] = vy;
			vmap[3 * pos + 2] = vz;

			cmap[3 * pos] = color[3 * pos] / 255.0f;
			cmap[3 * pos + 1] = color[3 * pos + 1] / 255.0f;
 			cmap[3 * pos + 2] = color[3 * pos + 2] / 255.0f;
		}
		else
		{
			vmap[3 * pos] = 0.0f;
			vmap[3 * pos + 1] = 0.0f;
			vmap[3 * pos + 2] = 0.0f;

			cmap[3 * pos] = 0;
			cmap[3 * pos + 1] = 0.0f;
			cmap[3 * pos + 2] = 0.0f;
		}
    }
}

unsigned short* dev_depth = NULL;
unsigned char* dev_color = NULL;
double* dev_Rmat = NULL;
double* dev_tvec = NULL;

// Wrapper for the __global__ call that sets up the kernel call
extern "C" void create_point_cloud(carmen_slam6d_pointcloud_message* message, float* pos_v, float* pos_c, float focal_length)
{
  int width = message->width;
  int height = message->height;

  dim3 block (32, 8);
  dim3 grid (1, 1, 1);
  grid.x = (width + block.x - 1) / block.x;
  grid.y = (height + block.y - 1) / block.y;
  
  float fx = focal_length, cx = width / 2.0f;
  float fy = focal_length, cy = height / 2.0f;

  int depth_cuda_size = height * width * sizeof(unsigned short);
  int color_cuda_size = 3 * height * width * sizeof(unsigned char);


  if(dev_color == NULL && dev_depth == NULL)
  {
	  cudaMalloc((void**)&dev_depth, depth_cuda_size);
	  cudaMalloc((void**)&dev_color, color_cuda_size);
  }
  
  if(dev_Rmat == NULL && dev_tvec == NULL)
  {
	  cudaMalloc((void**)&dev_Rmat, 9 * sizeof(double));
	  cudaMalloc((void**)&dev_tvec, 3 * sizeof(double));
  }
  
  cudaMemcpy(dev_Rmat, message->rotation, 9 * sizeof(double), cudaMemcpyHostToDevice);
  cudaMemcpy(dev_tvec, message->position, 3 * sizeof(double), cudaMemcpyHostToDevice);

  cudaMemcpy(dev_depth, message->depth, depth_cuda_size, cudaMemcpyHostToDevice);
  cudaMemcpy(dev_color, message->image, color_cuda_size, cudaMemcpyHostToDevice);

  computeVmapKernel<<<grid, block>>>(dev_depth, dev_color, dev_tvec, dev_Rmat, pos_v, pos_c, width, height, 1.f/fx, 1.f/fy, cx, cy);
}