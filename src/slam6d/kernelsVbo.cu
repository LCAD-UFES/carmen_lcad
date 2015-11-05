#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

__global__ void
computeVmapKernel (unsigned short* depth, unsigned char* color, float* vmap, float *cmap, int depth_width, int depth_height, float dfx_inv, float dfy_inv, float rfx, float rfy, float d_cx, float d_cy, float rgb_cx, float rgb_cy, double* Rmat, double* tvec)
{
    int u = threadIdx.x + blockIdx.x * blockDim.x;
    int v = threadIdx.y + blockIdx.y * blockDim.y;
    int pos = 0, posc = 0;
    float z, vx, vy, vz, avx, avy, avz;
    int clx, cly;

    if (u < depth_width && v < depth_height)
    {
		pos = v * depth_width + u;			//i * width + j; 

		z = depth[pos] / 1000.0f; // load and convert: mm -> meters

		if (z >= 0.5f)
		{
			vx = z * (u - d_cx) * dfx_inv;
			vy = z * (v - d_cy) * dfy_inv;
			vz = z;

			clx = (int) (((vx * (rfx)) / vz) + rgb_cx);
			cly = (int) (((vy * (rfy)) / vz) + rgb_cy);

			avx = (float)((Rmat[0] * vx + Rmat[1] * vy + Rmat[2] * vz) + tvec[0]);
			avy = (float)((Rmat[3] * vx + Rmat[4] * vy + Rmat[5] * vz) + tvec[1]);
			avz = (float)((Rmat[6] * vx + Rmat[7] * vy + Rmat[8] * vz) + tvec[2]);

			vx = -avx;
			vy = -avy;
			vz = avz;			

			posc = cly * depth_width + clx;
			
			vmap[3 * pos] = vx;
			vmap[3 * pos + 1] = vy;
			vmap[3 * pos + 2] = vz;

			if((cly >=0 && cly < depth_height) && (clx >= 0 && clx < depth_width))
			{
			cmap[3 * pos] = color[3 * posc] / 255.0f;
			cmap[3 * pos + 1] = color[3 * posc + 1] / 255.0f;
			cmap[3 * pos + 2] = color[3 * posc + 2] / 255.0f;
			}
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
extern "C" void create_point_cloud(unsigned short* depth, unsigned char* color, float* posV, float* posC, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, float rgb_cx, float rgb_cy, double* Rmat, double* tvec)
{
  dim3 block (32, 8);
  dim3 grid (1, 1, 1);
  grid.x = (depth_width + block.x - 1) / block.x;
  grid.y = (depth_height + block.y - 1) / block.y;

  float dfx = depth_focal_length_x, d_cx = depth_width / 2.0f;
  float dfy = depth_focal_length_y, d_cy = depth_height / 2.0f;
  float rfx = rgb_focal_length_x;
  float rfy = rgb_focal_length_y;

  int depth_cuda_size = depth_height * depth_width * sizeof(unsigned short);
  int color_cuda_size = 3 * depth_height * depth_width * sizeof(unsigned char);


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

  cudaMemcpy(dev_depth, depth, depth_cuda_size, cudaMemcpyHostToDevice);
  cudaMemcpy(dev_color, color, color_cuda_size, cudaMemcpyHostToDevice);
  cudaMemcpy(dev_Rmat, Rmat, 9 * sizeof(double), cudaMemcpyHostToDevice);
  cudaMemcpy(dev_tvec, tvec, 3 * sizeof(double), cudaMemcpyHostToDevice);

  computeVmapKernel<<<grid, block>>>(dev_depth, dev_color, posV, posC, depth_width, depth_height, 1.f / dfx, 1.f / dfy, rfx, rfy, d_cx, d_cy, rgb_cx, rgb_cy, dev_Rmat, dev_tvec);
}
