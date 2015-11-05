//#include "v_disparity_GPU.h"
#include <stdio.h>

unsigned char *dst_image_GPU;
unsigned char *v_disparity_map_GPU;
unsigned char *road_profile_image_GPU;
float *disparity_map_GPU;

#define RED_OBSTACLES_DISTANCE  10.0
#define MINIMUM_OBSTACLE_HEIGHT(camera_height)  (0.1 - camera_height)
#define MAXIMUM_OBSTACLE_HEIGHT(camera_height)  (4.5 - camera_height)

static int image_size, v_disparity_map_size, disparity_map_size, road_profile_image_size;

extern "C" void
init_v_disparity_GPU(int width, int height, int nchannels)
{
	image_size = width *  height * nchannels;
	v_disparity_map_size = height *  height * nchannels;
	disparity_map_size = width *  height * sizeof(float);
	road_profile_image_size = height *  height * nchannels;

	cudaMalloc((void **)&dst_image_GPU,image_size);
	cudaMalloc((void **)&v_disparity_map_GPU, v_disparity_map_size);
	cudaMalloc((void **)&road_profile_image_GPU, road_profile_image_size);
	cudaMalloc((void **)&disparity_map_GPU, disparity_map_size);
}


__device__
float3 reprojectSinglePointTo3D_GPU(int2 P, int disparity, double theta, float baseline, float focal_lenght, float xc, float yc)
{
  float xr = P.x - xc;
  float xl = xr + disparity;
  float yl = -(P.y - yc);

  // compute the point in 3D coordinates
  // pag. 85-86 dissertacao Hallysson
  float X = (baseline * (xl + xr)) / (2.0 * (xl - xr));
  float Y = (baseline * yl) / (xl - xr);
  float Z = (focal_lenght * baseline) / (xl - xr);

  // rotate the coordinate system with the camera pitch (in radians)
  float3 P_3D;
  P_3D.x = X;
  P_3D.y = Y * __cosf(theta) - Z * __sinf(theta);
  P_3D.z = Y * __sinf(theta) + Z * __cosf(theta);

  return P_3D;
}

__device__ int
is_possible_obstacle_GPU(unsigned char *road_profile_image, int y, int disparity, int stereo_width, int stereo_height, int stereo_disparity)
{

  float scale_factor = (float) stereo_height / (float) stereo_disparity;

  int offset = 3 * (y * stereo_height + (int)(disparity * scale_factor + scale_factor / 2.0));
  uchar3 pixel;
  pixel.x = road_profile_image[offset];
  pixel.y = road_profile_image[offset + 1];
  pixel.z = road_profile_image[offset + 2];

  // Discard all pixels that are outside the road profile, i.e, that are colored pixels in the road profile
  return !(pixel.x + pixel.y + pixel.z);
}


__device__ int
v_disparity_obstacle(unsigned char *v_disparity_map, int y, int disparity, int stereo_width, int stereo_height, int stereo_disparity)
{
	float scale_factor = (float) stereo_height / (float) stereo_disparity;

  // consider as a possible obstacle if there are more than 16 pixels with the disparity in the same line
  // we can verify the pixel size in meters and use this threshould in meters
  unsigned char pixel;
  pixel = v_disparity_map[3 * (y * stereo_height + (int)(disparity * scale_factor + scale_factor / 2.0))];

  // one needs to test only one channel because the image is grayscale
  return (pixel > 16) ? 1 : 0;
}


__global__ void
print_obstacles_kernel(
		unsigned char *dst_image,
		unsigned char *v_disparity_map,
		unsigned char *road_profile_image,
    float *disparity_map,
    float camera_height,
    float camera_pitch,
    float baseline,
    float focal_lenght,
    float xc,
    float yc,
    int stereo_width,
    int stereo_height,
    int stereo_disparity)

{


  float min_obstacle_height = MINIMUM_OBSTACLE_HEIGHT(camera_height);
  float max_obstacle_height = MAXIMUM_OBSTACLE_HEIGHT(camera_height);
  int disparity = blockIdx.y + 1;


 //for (int disparity = 1; disparity < stereo_disparity; disparity++)
  if(disparity < stereo_disparity)
  {
  	//for (int y = 0; y < stereo_height; y++) // zero is the top
    for (int y = blockIdx.x; y < stereo_height; y+=gridDim.x) // zero is the top
    {
      if (!is_possible_obstacle_GPU(road_profile_image, y, disparity, stereo_width, stereo_height, stereo_disparity))
        break;

      if (v_disparity_obstacle(v_disparity_map, y, disparity, stereo_width, stereo_height, stereo_disparity))
      {
      	//for (int x = 0; x < stereo_width; x++)
        for (int x = threadIdx.x; x < stereo_width; x+=blockDim.x)
        {
          int disparity_value = (int)disparity_map[y * stereo_width + x];
          if (disparity_value == disparity)
          {
            int2 P;
            P.x = x;
            P.y = y;
            float3 p_world = reprojectSinglePointTo3D_GPU(P, disparity, camera_pitch, baseline, focal_lenght, xc, yc);
            float distance = sqrtf(p_world.x * p_world.x + p_world.y * p_world.y + p_world.z * p_world.z);
            if ((distance < RED_OBSTACLES_DISTANCE) && //all obstacles that are less than 10 meters are red
                (p_world.y > min_obstacle_height) && (p_world.y < max_obstacle_height))
            {
            	int offset = 3 * (y * stereo_width + x);
            	dst_image[offset] = 255; // red obstacles
            }
            else if ((p_world.y > min_obstacle_height) && (p_world.y < max_obstacle_height))
            {
            	int offset = 3 * (y * stereo_width + x);
            	dst_image[offset + 1] = 255; // green obstacles
            }
          }
        }
      }
    }
  }
}

extern "C" void
print_obstacles_GPU(unsigned char *dst_image, unsigned char *v_disparity_map, unsigned char *road_profile_image,
    float *disparity_map, float camera_height, float camera_pitch, float baseline, float focal_lenght, float xc,
    float yc, int stereo_width, int stereo_height, int stereo_disparity)
{
	dim3 griddim(128,stereo_disparity-1);

	cudaMemcpy(dst_image_GPU,dst_image,image_size,cudaMemcpyHostToDevice);
	cudaMemcpy(v_disparity_map_GPU,v_disparity_map,v_disparity_map_size,cudaMemcpyHostToDevice);
	cudaMemcpy(road_profile_image_GPU,road_profile_image, road_profile_image_size,cudaMemcpyHostToDevice);
	cudaMemcpy(disparity_map_GPU,disparity_map,disparity_map_size,cudaMemcpyHostToDevice);

	print_obstacles_kernel<<<griddim,256>>>
			(dst_image_GPU,
			v_disparity_map_GPU,
			road_profile_image_GPU,
			disparity_map_GPU,
			camera_height,
			camera_pitch,
			baseline,
			focal_lenght,
			xc,
			yc,
			stereo_width,
			stereo_height,
			stereo_disparity);
	printf("cuda %d\n",image_size);

	cudaMemcpy(dst_image,dst_image_GPU, image_size,cudaMemcpyDeviceToHost);
	cudaMemcpy(road_profile_image,road_profile_image_GPU, v_disparity_map_size,cudaMemcpyDeviceToHost);
}

