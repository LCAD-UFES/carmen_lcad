#include "ml_road_finding_basic.h"
#include <cuda.h>

rgb_gaussian_gpu gaussian_gpu;
unsigned char *img_data_gpu;


rgb_gaussian_gpu alloc_GPU_gaussian_struct(int n_gaussians)
{
	rgb_gaussian_gpu gaussian;

	cudaMalloc((void**)&gaussian.mean,3 * n_gaussians * sizeof(float));
	cudaMalloc((void**)&gaussian.cov,9 * n_gaussians * sizeof(float));
	cudaMalloc((void**)&gaussian.cov_inv,9 * n_gaussians * sizeof(float));

	cudaMalloc((void**)&gaussian.probability_threshould, n_gaussians * sizeof(float));
	cudaMalloc((void**)&gaussian.probability_mean, n_gaussians * sizeof(float));
	cudaMalloc((void**)&gaussian.det_cov, n_gaussians * sizeof(float));
	
	cudaMalloc((void**)&gaussian.normalizer, n_gaussians * sizeof(float));

	return gaussian;
}


void init_cuda_road_finding(int n_gaussians, int widthStep, int height)
{
	gaussian_gpu = alloc_GPU_gaussian_struct(n_gaussians);

	cudaMalloc((void**)&img_data_gpu, widthStep * height);
}

void add_gaussian_to_GPU(rgb_gaussian **gaussians, int n_gaussians)
{
	float mean[3 * n_gaussians];
	float cov[9 * n_gaussians];
	float cov_inv[9 * n_gaussians];
	float probability_mean[n_gaussians];
	float probability_threshould[n_gaussians];
	float det_cov[n_gaussians];
	float normalizer[n_gaussians];

	int k = 0;
	for(int w = 0; w < n_gaussians; w++)
	{
		for(int i = 0; i < 3; i++)
		{
			for(int j = 0; j < 3; j++, k++)
			{
				cov[k] = gsl_matrix_get(gaussians[w]->cov,i,j);
				cov_inv[k] = (float)gaussians[w]->cov_inv[3 * i + j];
			}
			mean[w * 3 + i] = gsl_vector_get(gaussians[w]->mean,i);
		}
		probability_mean[w] = gaussians[w]->probability_mean;
		det_cov[w] = gaussians[w]->det_cov;
		probability_threshould[w] = gaussians[w]->probability_threshould;
		normalizer[w] =  gaussians[w]->normalizer;
	}

	cudaMemcpy(gaussian_gpu.mean, mean, 3 * n_gaussians * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(gaussian_gpu.cov, cov, 9 * n_gaussians * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(gaussian_gpu.cov_inv, cov_inv, 9 * n_gaussians * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(gaussian_gpu.det_cov, det_cov, n_gaussians * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(gaussian_gpu.probability_threshould, probability_threshould, n_gaussians * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(gaussian_gpu.probability_mean ,probability_mean, n_gaussians * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(gaussian_gpu.normalizer, normalizer, n_gaussians * sizeof(float), cudaMemcpyHostToDevice);
}

__device__ float get_normal_value_GPU(float3 pixel , rgb_gaussian_gpu gaussian, int index)
{
	float3 X;
	float distance;
	float *aux_vect = gaussian.mean + 3 * index;

	//X = sample - MEAN
	X.x = pixel.x - aux_vect[RED_INDEX];
	X.y = pixel.y - aux_vect[GREEN_INDEX];
	X.z = pixel.z - aux_vect[BLUE_INDEX];

	aux_vect = gaussian.cov_inv + 9 * index;

	distance = (X.x * aux_vect[0] + X.y * aux_vect[1] +  X.z * aux_vect[2]) * X.x;
	distance += (X.x * aux_vect[3] + X.y * aux_vect[4] +  X.z * aux_vect[5]) * X.y;
	distance += (X.x * aux_vect[6] + X.y * aux_vect[7] +  X.z * aux_vect[8]) * X.z;

	float f1 = 1.0f / (SQRT_2_PI_POW_3 * __powf(fabsf(gaussian.det_cov[index]),0.5f));
	return f1 * __expf(-0.5f * distance);
}

__global__ void fill_image_based_on_gaussians_kernel(unsigned char *imageData, int width, int height, int widthStep, rgb_gaussian_gpu gaussians, int n_gaussians)
{
	float3 pixel;
	float value;


	for (int y = blockIdx.x; y < height; y+=gridDim.x)
	{
		for (int x = threadIdx.x; x < width; x+=blockDim.x)
		{
			int index = 3 * (y * width + x);
			pixel.x = imageData[index + RED_INDEX];
			pixel.y = imageData[index + GREEN_INDEX];
			pixel.z = imageData[index + BLUE_INDEX];


			float best_normal_value = 0.0;
			int best_gaussian_index = 0;

			for (int i = 0; i < n_gaussians; i++)
			{
				value = get_normal_value_GPU(pixel, gaussians, i);

				if (value > best_normal_value)
				{
					best_normal_value = value;
					best_gaussian_index = i;
				}

			}

			value = 255 * best_normal_value * gaussians.normalizer[best_gaussian_index];
			imageData[index + RED_INDEX] = value;
			imageData[index + GREEN_INDEX] = value;
			imageData[index + BLUE_INDEX] = value;

		}
	}
}


void fill_image_based_on_gaussians_GPU(unsigned char *imageData, int width, int height, int widthStep, int n_gaussians)
{
	cudaMemcpy(img_data_gpu,imageData,height * widthStep,cudaMemcpyHostToDevice);

	fill_image_based_on_gaussians_kernel<<<128,256>>>(img_data_gpu, width, height, widthStep, gaussian_gpu, n_gaussians);

	cudaMemcpy(imageData,img_data_gpu, height * widthStep,cudaMemcpyDeviceToHost);
}



