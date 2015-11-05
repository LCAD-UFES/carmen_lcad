#include "vg_ram.h"


//VG-RAM WNN Global Variables
//int 	*NeuronSynapsesCoordinate_X_GPU;
//int 	*NeuronSynapsesCoordinate_Y_GPU;
int   *NeuronSynapsesCoordinate_X_HOST;
int   *NeuronSynapsesCoordinate_Y_HOST;

unsigned int	*bitPattern_GPU;
//unsigned int	*HamDist_GPU;
//unsigned short	*disps_GPU;
vg_ram_wnn	stereo_wnn_GPU;
float gaussian_radius;
//DisparityMap disp;

// __constant__ int patternSize;
__constant__ int inc_stereo_height;
__constant__ int inc_stereo_width;

// Inline functions for Winner-Takes-it-All filter
__device__ unsigned char * CENTRAL_GPU(int plane, int w, int h,unsigned char* p) { return p + (w*h*plane); }
__device__ unsigned char * UP_GPU(int w, unsigned char* p) { return (p - w * inc_stereo_height); }
__device__ unsigned char * UP_RIGHT_GPU(int w, unsigned char* p) {return (p - w * inc_stereo_height + inc_stereo_width); }
__device__ unsigned char * RIGHT_GPU(int w, unsigned char* p) { return( p + inc_stereo_width); }
__device__ unsigned char * LOW_RIGHT_GPU(int w, unsigned char* p) { return( p + w * inc_stereo_height + inc_stereo_width); }
__device__ unsigned char * LOW_GPU(int w, unsigned char* p) { return( p + w * inc_stereo_height);  }
__device__ unsigned char * LOW_LEFT_GPU(int w, unsigned char* p) { return( p + w * inc_stereo_height - inc_stereo_width); }
__device__ unsigned char * LEFT_GPU(int w, unsigned char* p) { return( p - inc_stereo_width ); }
__device__ unsigned char * UP_LEFT_GPU(int w, unsigned char* p) { return( p - w * inc_stereo_height - inc_stereo_width); }


extern "C" void
allocGPUData_vg(void **devPtr,size_t size)
{
  cudaMalloc(devPtr, size);
}

extern "C" void
GPUcopy_vg(void * dest, void *src, size_t size, int type)
{
  if(type)
  {
    cudaMemcpy(dest,src, size,cudaMemcpyHostToDevice);
  }else{

    cudaMemcpy(dest,src, size,cudaMemcpyDeviceToHost);
  }
}

extern "C" void GPUmemset_vg(void *devPtr, int value, size_t count)
{
  cudaMemset(devPtr, value, count);
}

extern "C" void
alloc_vg_ram_WNN_multi_neuron_on_GPU(
    int number_of_neurons,
    int pattern_size,
    int number_of_layer_width,
    int number_of_layer_height,
    int number_of_synapses,
    bi_dimention_vg_ram_wnn *stereo_wnn_out)
{
  stereo_wnn_out->number_of_layer_width=number_of_layer_width;
  stereo_wnn_out->number_of_layer_height=number_of_layer_height;
  stereo_wnn_out->number_of_synapses=number_of_synapses;
  stereo_wnn_out->number_of_neurons = number_of_neurons;
  stereo_wnn_out->pattern_size=pattern_size;

  allocGPUData_vg((void**)&stereo_wnn_out->pattern,pattern_size * number_of_layer_height * number_of_layer_width * number_of_neurons * sizeof(unsigned  int));
}

extern "C"
void allocSynapsesDistribution_on_GPU(synapsesDistribution *neuronSynapses, int numSynapses)
{
	allocGPUData_vg((void**)neuronSynapses->grx, numSynapses*sizeof(float));
	allocGPUData_vg((void**)neuronSynapses->gry, numSynapses*sizeof(float));
	neuronSynapses->numSynapses=numSynapses;
}

extern "C" void
alloc_disparity_map_on_GPU(DisparityMap *disp,int w, int h, int numOfDisparity )
{
	disp->height=h;
	disp->width=w;
	disp->numMaps=numOfDisparity;

	allocGPUData_vg((void**)&disp->disps, w * h * numOfDisparity * sizeof(unsigned char));
}

extern "C" void
alloc_image_on_GPU(int width, int height, int channels, Image *img)
{
  img->width = width;
  img->height = height;
  img->channels = channels;
  allocGPUData_vg((void**)&img->data,width * height * channels * sizeof(unsigned char));
}

extern "C"
void alloc_vgram_WNN_on_GPU(int patternSize, int numNeurons, int numSynapses, int numNeuronsLayer, vg_ram_wnn *stereo_wnn_out)
{
	stereo_wnn_out->numNeurons=numNeurons;
	stereo_wnn_out->numSynapses=numSynapses;
	stereo_wnn_out->numNeuronsLayer=numNeuronsLayer;
	stereo_wnn_out->patternSize=patternSize;

	allocGPUData_vg((void**)&stereo_wnn_out->pattern,patternSize * numNeurons * sizeof(unsigned  int));
}


__global__ void
rgb_to_gray_GPU_kernel(Image img, unsigned int *gray, int stereo_vertical_ROI_ini, int gray_offset)
{
//   int y =  blockIdx.x * blockDim.x + threadIdx.x;
//   int gray_y = stereo_vertical_ROI_ini + y / img.width;
//   int gray_x = stereo_vertical_ROI_ini + y % img.width;
  int gray_y = stereo_vertical_ROI_ini + blockIdx.x;
  for(int y = blockIdx.x; y < img.height; y += gridDim.x, gray_y += gridDim.x)
  {

    int gray_x = stereo_vertical_ROI_ini + threadIdx.x;
    for(int x = threadIdx.x; x < img.width; x += blockDim.x, gray_x += blockDim.x)
    {
      int pos = 3 * (y * img.width + x);
      gray[gray_y * gray_offset + gray_x] = 0.299f * img.data[pos] + 0.587f * img.data[pos + 1] + 0.114f * img.data[pos + 2];
    }
    //gray[y] = PIXEL(img.data[pos], img.data[pos + 1], img.data[pos + 2]);
  }
}


extern "C" void
rgb_to_gray_GPU(Image img, unsigned int *gray, int stereo_vertical_ROI_ini, int gray_offset)
{
    int size = img.width * img.height;

    dim3 dimGrid (size % BLOCK_SIZE ==0 ? size / BLOCK_SIZE : (size / BLOCK_SIZE) +1);
    rgb_to_gray_GPU_kernel<<<128,BLOCK_SIZE>>>(img, gray, stereo_vertical_ROI_ini,gray_offset);
}

__device__ int
bit_count_GPU(unsigned int n)
{
  /* works for 32-bit numbers only    */
  /* fix last line for 64-bit numbers */

  unsigned int tmp;

  tmp = n - ((n >> 1) & 033333333333)
  - ((n >> 2) & 011111111111);
  return ((tmp + (tmp >> 3)) & 030707070707) % 63;
}

__device__
void calc_neuron_synapses_coordinate_on_image_GPU_func(int center_x, int center_y,synapsesDistribution neuronSynapses,
		int *Internal_NeuronSynapsesCoordinate_X, int *Internal_NeuronSynapsesCoordinate_Y,
		int ImageWidth, int ImageHeight)
{
	int synapse=0;
	int x,y;
	for(synapse=threadIdx.x;synapse<neuronSynapses.numSynapses;synapse+=blockDim.x)
	{
		x = (int)((float)center_x + neuronSynapses.grx[synapse]);
		y = (int)((float)center_y + neuronSynapses.gry[synapse]);
		x = x > 0 ? x : 0;
		y = y > 0 ? y : 0;
		x = x < ImageWidth  ? x : ImageWidth-1;
		y = y < ImageHeight ? y : ImageHeight-1;
		Internal_NeuronSynapsesCoordinate_X[synapse] = x;
		Internal_NeuronSynapsesCoordinate_Y[synapse] = y;
	}
}

__device__
void calc_bit_pattern_minchinton_cell_GPU_func(
    unsigned int *pattern,
    int pattern_offset,
    int pattern_width,
    int pattern_height,
    unsigned short *neuron_synapses_coordinate_X,
    unsigned short *neuron_synapses_coordinate_Y,
    unsigned int *bit_pattern,
    int num_synapses,
    int pattern_size,
    int center_x,
    int center_y,
    int tid,
    int tidDim)
{
  int synapse;
  int x,y;
  unsigned int pixelxi;
  unsigned int pixelxi1;
  int N=num_synapses - 1;
//   int shift;
  int current_bit_pattern_group;



  for(synapse=tid;synapse<N;synapse+=tidDim)
  {
     //shift=synapse % PATTERN_UNIT_SIZE;
    current_bit_pattern_group = synapse / PATTERN_UNIT_SIZE;

    x = neuron_synapses_coordinate_X[synapse];
    y = neuron_synapses_coordinate_Y[synapse];
    pixelxi  = pattern[y * pattern_offset + x];

    x = neuron_synapses_coordinate_X[synapse + 1];
    y = neuron_synapses_coordinate_Y[synapse + 1];
    pixelxi1  = pattern[y * pattern_offset + x];

    //atomicOr(bit_pattern + current_bit_pattern_group, ((pixelxi > pixelxi1) ? 1 : 0) << shift);

    atomicOr(bit_pattern + current_bit_pattern_group, __ballot(pixelxi > pixelxi1));
  }

  __syncthreads();

  // Minchington: Última sinapse
  if(tid == tidDim - 1)
  {
    //shift=N % PATTERN_UNIT_SIZE;
    current_bit_pattern_group=N / PATTERN_UNIT_SIZE;

    x = neuron_synapses_coordinate_X[N];
    y = neuron_synapses_coordinate_Y[N];
    pixelxi  = pattern[y * pattern_offset + x];

    x = neuron_synapses_coordinate_X[0];
    y = neuron_synapses_coordinate_Y[0];
    pixelxi1  = pattern[y * pattern_offset + x];

  //
    atomicOr(bit_pattern + current_bit_pattern_group, __ballot(pixelxi > pixelxi1));

//     atomicOr(bit_pattern + current_bit_pattern_group, ((pixelxi > pixelxi1) ? 1 : 0) << shift);
  }
}

__global__
void WNN_train_GPU_kernel(
	unsigned int *gray_image_left,
	int width,
	int height,
	int gray_offset,
	bi_dimention_vg_ram_wnn stereo_wnn,
	unsigned short *neuron_synapses_coordinate_X,
	unsigned short *neuron_synapses_coordinate_Y,
	int inc_height,
	int stereo_horizontal_ROI_ini,
	int stereo_horizontal_ROI_end,
	int stereo_vertical_ROI_ini)
{
	int  neuron;	//Neuron counter must be treated as an integer value due to loop conditions

	neuron = stereo_horizontal_ROI_ini + blockIdx.x;

	__shared__ unsigned short neuron_synapses_coordinate_X_shared[2048];
	__shared__ unsigned short neuron_synapses_coordinate_Y_shared[2048];
	__shared__ unsigned int bit_pattern[64];

	int tid = threadIdx.y * blockDim.y + threadIdx.x;
	int tidDim = blockDim.x * blockDim.y;
	int line =  blockIdx.y * inc_height;

	if(line < height)
	{
			if(neuron < stereo_horizontal_ROI_end)
			{
				int i = stereo_vertical_ROI_ini + neuron;

				for(int j = tid; j < stereo_wnn.number_of_synapses; j += tidDim)
				{
					neuron_synapses_coordinate_X_shared[j] = neuron_synapses_coordinate_X[j] + i;
					neuron_synapses_coordinate_Y_shared[j] = neuron_synapses_coordinate_Y[j] + line;
				}

				if(tid < stereo_wnn.pattern_size)
					bit_pattern[tid] = 0;

				__syncthreads();

				calc_bit_pattern_minchinton_cell_GPU_func(
							gray_image_left,
							gray_offset,
							width,
							height,
							neuron_synapses_coordinate_X_shared,
							neuron_synapses_coordinate_Y_shared,
							bit_pattern,
							stereo_wnn.number_of_synapses,
							stereo_wnn.pattern_size,
							neuron,
							line + stereo_vertical_ROI_ini,tid,tidDim);

				__syncthreads();

				if(tid < stereo_wnn.pattern_size)
					stereo_wnn.pattern[line * width * stereo_wnn.pattern_size + neuron * stereo_wnn.pattern_size + tid] = bit_pattern[threadIdx.x];

			}
	}
}

__device__ void
hamming_distance_GPU_func2(unsigned  int *bit_pattern1, unsigned  int *bit_pattern2, int pattern_size, int *ham_dist_out)
{
  unsigned int hamming_distance = 0;

  for (int i = threadIdx.y; i < pattern_size; i+=blockDim.y)
  {

   //atomicAdd(ham_dist_out, __popc(bit_difference));

    //hamming_distance +=  bit_count_GPU(bit_difference);
    hamming_distance +=  __popc(bit_pattern1[i] ^ bit_pattern2[i]);

  }

  atomicAdd(ham_dist_out, hamming_distance);
}


__device__ void
find_nearest_patterns_GPU_func2(
      unsigned  int *bit_pattern,
      unsigned  int *neuron_pattern,
      unsigned short *HamDist,
      unsigned short *disps,
      int MAX_Diparity,
      int pattern_size)
{

  __shared__ int ham_dist[64];

  if(threadIdx.y == 0)
      ham_dist[threadIdx.x] = 0;
  __syncthreads();
  for(int disparity = threadIdx.x; disparity < MAX_Diparity; disparity += blockDim.x )
  {


      hamming_distance_GPU_func2( neuron_pattern + disparity * pattern_size, bit_pattern, pattern_size, &ham_dist[threadIdx.x]);

      __syncthreads();

      if(threadIdx.y == 0)
      {
				HamDist[disparity] = ham_dist[threadIdx.x];
				disps[disparity] = disparity;
				ham_dist[threadIdx.x] = 0;
      }
      __syncthreads();
  }
}

__device__ void
select_disparity_GPU_func(DisparityMap DispMap,unsigned int *HamDist, unsigned int *disps, int MAX_Disparity, int pixels, int line, int tid, int tidDim)
{
  int i;
  __shared__ int bestHamDist;

  float *layerDisp;
  int imgSize=DispMap.width*DispMap.height;

  if(tid == 0)
	 bestHamDist=100000;
  __syncthreads();
  int offset = line * DispMap.width;


    for(i=0;i<DispMap.numMaps;i++)
    {

      layerDisp = &DispMap.disps[i * imgSize + offset];
      int aux = HamDist[tid];

      atomicMin(&bestHamDist, aux);

      __syncthreads();

      if(bestHamDist == aux)
      {
				layerDisp[pixels] = disps[tid];
				HamDist[tid]=10000;
				bestHamDist=100000;
      }

      __syncthreads();

    }

}

__global__
void WNN_disparity_line_GPU_kernel(
		    unsigned int *gray_image_right,
		    int height,
		    int width,
		    int gray_offset,
		    int MAX_Disparity,
		    bi_dimention_vg_ram_wnn stereo_wnn,
		    unsigned short *neuron_synapses_coordinate_X,
		    unsigned short *neuron_synapses_coordinate_Y,
		    int inc_height,
		    unsigned short *HamDist,
		    unsigned short *disps,
		    int stereo_horizontal_ROI_ini,
		    int stereo_horizontal_ROI_end,
		    int stereo_vertical_ROI_ini,
		    int inc_width,
		    DisparityMap DispMap)
{

	int neuron;
	int MAX_size;
	int aux;

	__shared__ unsigned short shared1[2048];
	__shared__ unsigned short shared2[2048];
	__shared__ unsigned int bit_pattern[64];

	neuron = stereo_horizontal_ROI_ini + blockIdx.x * inc_width;

	int tid = threadIdx.y * blockDim.y + threadIdx.x;
	int tidDim = blockDim.x * blockDim.y;

	int line =  blockIdx.y * inc_height;

	if(line < height)
	{
		if( neuron < stereo_horizontal_ROI_end )
		{
			int i = stereo_vertical_ROI_ini + neuron;
			int k = line + stereo_vertical_ROI_ini;

			for(int j = tid; j < stereo_wnn.number_of_synapses; j += tidDim)
			{
				shared1[j] = neuron_synapses_coordinate_X[j] + i;
				shared2[j] = neuron_synapses_coordinate_Y[j] + k;
			}

			if(tid < stereo_wnn.pattern_size)
				bit_pattern[tid] = 0;

			__syncthreads();

			calc_bit_pattern_minchinton_cell_GPU_func(
					gray_image_right,
					gray_offset,
					width,
					height,
					shared1,
					shared2,
					bit_pattern,
					stereo_wnn.number_of_synapses,
					stereo_wnn.pattern_size,
					neuron,
					k,tid,tidDim);

			//shared1[tid] = 0;
			//shared2[tid] = 0;

			__syncthreads();

			aux = width - neuron;
			MAX_size = aux > MAX_Disparity ? MAX_Disparity : aux;

			find_nearest_patterns_GPU_func2(
					bit_pattern,
					stereo_wnn.pattern + line * width * stereo_wnn.pattern_size + neuron * stereo_wnn.pattern_size,
					shared1,
					shared2,
					MAX_size,
					stereo_wnn.pattern_size);

//			if(tid < MAX_Disparity)
//				select_disparity_GPU_func(DispMap,shared1, shared2, MAX_Disparity, neuron, line, tid, tidDim);

		}
	}
}

__global__ void
disparity_calculation_GPU(
		unsigned int *gray_left,
		unsigned int *gray_right,
		int height,
		int width,
		int max_disparity,
		int number_of_synapses,
		unsigned short *Internal_NeuronSynapsesCoordinate_X,
		unsigned short *Internal_NeuronSynapsesCoordinate_Y,
		DisparityMap DispMap,
		int inc_height,
		int inc_width,
		int stereo_horizontal_ROI_ini,
		int stereo_horizontal_ROI_end,
		int gray_size_offset)
{
	int sx, sy;
	int pixelxi_left = 0;
	int pixelxi1_left = 0;
	int hamming_distance;
	int synapse;
	int disparity;

	char bit_pattern_left, bit_pattern_right;

	int y =  blockIdx.y * inc_height;
	int x = stereo_horizontal_ROI_ini + blockIdx.x * inc_width;

	int tid = threadIdx.x * blockDim.y + threadIdx.y;
	int tidDim = blockDim.x * blockDim.y;

	__shared__ unsigned int ham_dist[256];
	__shared__ unsigned int disps[256];
	__shared__ unsigned char right_pixels[2048];
	__shared__ unsigned short pos_x[2048];
	__shared__ unsigned short pos_y[2048];

	if(y < height)
	{
		if(x < stereo_horizontal_ROI_end)
		{
			for(int j = tid; j < number_of_synapses; j += tidDim)
			{
				sx = Internal_NeuronSynapsesCoordinate_X[j] + x;
				sy = Internal_NeuronSynapsesCoordinate_Y[j] + y;
				right_pixels[j] = gray_right[sy * gray_size_offset + sx];
				pos_x[j] = sx;
				pos_y[j] = sy;
			}

			if(tid < max_disparity)
						ham_dist[tid] = 0;

			__syncthreads();
			for (disparity = threadIdx.x; disparity < max_disparity; disparity += blockDim.x )
			{
				for (synapse = threadIdx.y; synapse < number_of_synapses - 1; synapse += blockDim.y)
				{
					sy = pos_y[synapse + 1];
					sx = pos_x[synapse + 1] + disparity;
					pixelxi1_left = gray_left[sy * gray_size_offset + sx];

					sy = pos_y[synapse];
					sx = pos_x[synapse] + disparity;
					pixelxi_left = gray_left[sy * gray_size_offset + sx];

					bit_pattern_left = ((pixelxi_left > pixelxi1_left) ? 1 : 0);

					bit_pattern_right =  ((right_pixels[synapse] > right_pixels[synapse + 1]) ? 1 : 0);

					hamming_distance += bit_pattern_left ^ bit_pattern_right;

				}


				__syncthreads();

				atomicAdd(&ham_dist[disparity], hamming_distance);
				if(threadIdx.y == 0)
							disps[disparity] = disparity;

			}
			__syncthreads();

			if(tid < max_disparity)
					select_disparity_GPU_func(DispMap,ham_dist, disps, max_disparity, x, y, tid, 512);
		}
	}
}



extern "C" void
WNN_disparity_GPU2(
	  unsigned int *gray_image_left,
	  unsigned int *gray_image_right,
	  int width,
	  int height,
	  int gray_offset,
	  int MAX_Disparity,
	  bi_dimention_vg_ram_wnn stereo_wnn,
	  unsigned short *neuron_synapses_coordinate_X,
	  unsigned short *neuron_synapses_coordinate_Y,
	  int image_line,
	  unsigned short *HamDist,
	  unsigned short *disps,
	  int stereo_horizontal_ROI_ini,
	  int stereo_horizontal_ROI_end,
	  int stereo_vertical_ROI_ini,
	  int inc_width,
	  int inc_height,
	  DisparityMap DispMap)
{
	//int	line;

	 int image_width = stereo_horizontal_ROI_end - stereo_horizontal_ROI_ini;

	 int size_width = image_width % inc_width == 0 ? image_width / inc_width : (image_width / inc_width) + 1;
	 int size_height = height % inc_height == 0 ? height / inc_height : (height / inc_height) + 1;

	 dim3 dimGrid_train (stereo_wnn.number_of_layer_width, size_height);
	 dim3 dimGrid_disparity (size_width, size_height);
	// printf("%d %d\n",size_width,size_height);
	 dim3 dimBlock(16,32);

	 disparity_calculation_GPU<<<dimGrid_disparity,dimBlock>>>(
			 gray_image_left,
			 gray_image_right,
			 height,
			 width,
			 MAX_Disparity,
			 stereo_wnn.number_of_synapses,
		   neuron_synapses_coordinate_X,
		   neuron_synapses_coordinate_Y,
			 DispMap,
	 		 inc_height,
	 		 inc_width,
	 		 stereo_horizontal_ROI_ini,
	 		 stereo_horizontal_ROI_end,
	 		 gray_offset);

		/*WNN_train_GPU_kernel<<<dimGrid_train,dimBlock>>>(
		    gray_image_left,
		    width,
		    height,
		    gray_offset,
		    stereo_wnn,
		    neuron_synapses_coordinate_X,
		    neuron_synapses_coordinate_Y,
		    inc_height,
		    stereo_horizontal_ROI_ini,
		    stereo_horizontal_ROI_end,
		    stereo_vertical_ROI_ini);
		    //printf("%d\n", line);

		WNN_disparity_line_GPU_kernel<<<dimGrid_disparity,dimBlock>>>(
		    gray_image_right,
		    height,
		    width,
		    gray_offset,
		    MAX_Disparity,
		    stereo_wnn,
		    neuron_synapses_coordinate_X,
		    neuron_synapses_coordinate_Y,
		    inc_height,
		    HamDist,
		    disps,
		    stereo_horizontal_ROI_ini,
		    stereo_horizontal_ROI_end,
		    stereo_vertical_ROI_ini,
		    inc_width,
		    DispMap);*/
}


/*__device__ void
hamming_distance_GPU_func2(unsigned  int *bit_pattern1, unsigned  int *bit_pattern2, int pattern_size, int *ham_dist_out)
{
  unsigned int hamming_distance = 0;

  for (int i = 0; i < pattern_size; i++)
  {

   //atomicAdd(ham_dist_out, __popc(bit_difference));

    //hamming_distance +=  bit_count_GPU(bit_difference);
    hamming_distance +=  __popc(bit_pattern1[i] ^ bit_pattern2[i]);

  }
  *ham_dist_out = hamming_distance;
}


__device__ void
find_nearest_patterns_GPU_func2(
      unsigned  int *bit_pattern,
      unsigned  int *neuron_pattern,
      unsigned short *HamDist,
      unsigned short *disps,
      int MAX_Diparity,
      int pattern_size)
{



  int tid = threadIdx.y * blockDim.y + threadIdx.x;
  int tidDim = blockDim.x * blockDim.y;

  for(int disparity = tid; disparity < MAX_Diparity; disparity += tidDim )
  {

      int h;
      hamming_distance_GPU_func2( neuron_pattern + disparity * pattern_size, bit_pattern, pattern_size, &h);
      HamDist[disparity] = h;

  }
}*/




/*__device__ void
hamming_distance_GPU_func2(unsigned  int *bit_pattern1, unsigned  int *bit_pattern2, int pattern_size, int *ham_dist_out)
{
  unsigned int i;
  unsigned  int bit_difference;
  unsigned int hamming_distance = 0;

  if(threadIdx.x == 0)
      *ham_dist_out = 0;
  __syncthreads();

  for (i = threadIdx.y; i < pattern_size; i+=blockDim.y)
  {
    bit_difference = bit_pattern1[i] ^ bit_pattern2[i];

    //atomicAdd(&hamming_distance[threadIdx.x], __popc(bit_difference));

    hamming_distance +=  bit_count_GPU(bit_difference);

  }

  atomicAdd(ham_dist_out, hamming_distance);
}


__device__ void
find_nearest_patterns_GPU_func2(
      unsigned  int *bit_pattern,
      unsigned  int *neuron_pattern,
      unsigned int *HamDist,
      unsigned short *disps,
      int MAX_Diparity,
      int pattern_size)
{

  __shared__ int ham_dist[64];

  if(threadIdx.y == 0)
      ham_dist[threadIdx.x] = 0;

  for(int disparity = threadIdx.x; disparity < MAX_Diparity; disparity += blockDim.x )
  {
      __syncthreads();
      hamming_distance_GPU_func2( neuron_pattern + disparity * pattern_size, bit_pattern, pattern_size, &ham_dist[threadIdx.x]);
      if(threadIdx.y == 0)
      {
	HamDist[disparity] = ham_dist[threadIdx.x];
	disps[disparity] = 50;
	ham_dist[threadIdx.x] = 0;
      }


  }
}

__global__
void WNN_disparity_line_GPU_kernel(
		    unsigned int *gray_image_right,
		    int height,
		    int width,
		    int MAX_Disparity,
		    bi_dimention_vg_ram_wnn stereo_wnn,
		    synapsesDistribution neuronSynapses_GPU,
		    int image_line,
		    unsigned int *HamDist,
		    unsigned short *disps,
		    int stereo_horizontal_ROI_ini,
		    int stereo_horizontal_ROI_end,
		    int inc_width)
{

	int neuron;
	int MAX_size;
	int aux;
	__shared__ unsigned int bit_pattern[64];

	neuron = stereo_horizontal_ROI_ini + blockIdx.x * inc_width;

	if( neuron < stereo_horizontal_ROI_end )
	{
		calc_bit_pattern_minchinton_cell_GPU_func(
					gray_image_right,
					width,
					height,
					neuronSynapses_GPU,
					bit_pattern,
					stereo_wnn.number_of_synapses,
					stereo_wnn.pattern_size,
					image_line,
					neuron);
		__syncthreads();

		aux = width-neuron;
		MAX_size = aux > MAX_Disparity ? MAX_Disparity : aux;
		aux = image_line * width * MAX_Disparity + neuron * MAX_Disparity;
		find_nearest_patterns_GPU_func2(
			bit_pattern,
			stereo_wnn.pattern + image_line * width * stereo_wnn.pattern_size + neuron * stereo_wnn.pattern_size ,
			HamDist + aux,
			disps + aux,
			MAX_size,
			stereo_wnn.pattern_size);
	}
}

__global__
void WNN_train_GPU_kernel(
	unsigned int *gray_image_left,
	int width,
	int height,
	bi_dimention_vg_ram_wnn stereo_wnn,
	synapsesDistribution neuronSynapses_GPU,
	int image_line,
	int stereo_horizontal_ROI_ini,
	int stereo_horizontal_ROI_end)
{
	int  neuron;	//Neuron counter must be treated as an integer value due to loop conditions

	neuron = stereo_horizontal_ROI_ini + blockIdx.x;
	if(neuron < stereo_horizontal_ROI_end)
	{

		calc_bit_pattern_minchinton_cell_GPU_func(
					gray_image_left,
					width,
					height,
					neuronSynapses_GPU,
					stereo_wnn.pattern + image_line * width * stereo_wnn.pattern_size + neuron * stereo_wnn.pattern_size,
					stereo_wnn.number_of_synapses,
					stereo_wnn.pattern_size,
					neuron,
					image_line);

	}
}


void
WNN_disparity_GPU2(
	  unsigned int *gray_image_left,
	  unsigned int *gray_image_right,
	  int MAX_Disparity,
	  bi_dimention_vg_ram_wnn stereo_wnn,
	  synapsesDistribution neuronSynapses_GPU,
	  int image_line,
	  unsigned int *HamDist,
	  unsigned short *disps,
	  int stereo_horizontal_ROI_ini,
	  int stereo_horizontal_ROI_end,
	  int inc_width,
	  int inc_height)
{
	int	line;

	 int size_width = stereo_wnn.number_of_layer_width % inc_width == 0 ? stereo_wnn.number_of_layer_width / inc_width : (stereo_wnn.number_of_layer_width / inc_width) + 1;
	 int size_height = stereo_wnn.number_of_layer_height % inc_height == 0 ? stereo_wnn.number_of_layer_height / inc_height : (stereo_wnn.number_of_layer_height / inc_height) + 1;

	 dim3 dimGrid_train (stereo_wnn.number_of_layer_width, size_height);
	 dim3 dimGrid_disparity (size_width, size_height);
	 printf("%d %d\n",size_width,size_height);
	 dim3 dimBlock(256);


	for(line = 0; line < stereo_wnn.number_of_layer_height; line += inc_height)
	{
		//Foreward train/test -> left to right

		WNN_train_GPU_kernel<<<stereo_wnn.number_of_layer_width,dimBlock>>>(
		    gray_image_left,
		    stereo_wnn.number_of_layer_height,
		    stereo_wnn.number_of_layer_width,
		    stereo_wnn,
		    neuronSynapses_GPU,
		    line,
		    stereo_horizontal_ROI_ini,
		    stereo_horizontal_ROI_end);

		WNN_disparity_line_GPU_kernel<<<size_width,dimBlock>>>(
		    gray_image_right,
		    stereo_wnn.number_of_layer_height,
		    stereo_wnn.number_of_layer_width,
		    MAX_Disparity,
		    stereo_wnn,
		    neuronSynapses_GPU,
		    line,
		    HamDist,
		    disps,
		    stereo_horizontal_ROI_ini,
		    stereo_horizontal_ROI_end,
		    inc_width);




		//WNNTrain(ImgLeft, ImgRight, MAX_Diparity, stereo_wnn[threadIdx], &NeuronSynapsesCoordinate_X [line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width], &NeuronSynapsesCoordinate_Y [line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width], line);
		//WNNDispatyLine(ImgLeft, ImgRight, DispMap, MAX_Diparity, stereo_wnn[threadIdx], &NeuronSynapsesCoordinate_X[line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width],  &NeuronSynapsesCoordinate_Y [line*stereo_wnn[threadIdx].numSynapses*ImgLeft.width], bitPattern[threadIdx], line, HamDist + line * ImgLeft.width * MAX_Diparity, disps  + line * ImgLeft.width * MAX_Diparity);
	}
}


__device__ void
WNN_train_line_GPU_func(
    unsigned int *pattern_vector,
    int pattern_width,
    int pattern_height,
    synapsesDistribution neuronSynapses_GPU,
    unsigned  int *bit_pattern_vector,
    int number_of_synapses,
    int pattern_size,
    int begin,
    int end,
    int inc,
    int center_y
    )
{
  int  x = begin + blockIdx.x; //Neuron counter must be treated as an integer value due to loop conditions


  if ( x < end )
  {

    calc_bit_pattern_minchinton_cell_GPU_func(
        pattern_vector,
        pattern_width,
        pattern_height,
	neuronSynapses_GPU,
        bit_pattern_vector + x * pattern_size,
        number_of_synapses,
        pattern_size,x,center_y);
  }
}


__global__ void
WNN_train_neuron_GPU_kernel(
  unsigned int *pattern_vector,
  int pattern_width,
  int pattern_height,
  synapsesDistribution neuronSynapses_GPU,
  unsigned  int *bit_pattern_vector,
  int number_of_synapses,
  int pattern_size,
  int neuron_begin,
  int neuron_end,
  int inc_width,
  int inc_height
  )
{
  int y = blockIdx.y * inc_height;


  if( y < pattern_height )
  {

    WNN_train_line_GPU_func(
        pattern_vector,
        pattern_width,
        pattern_height,
	neuronSynapses_GPU,
        bit_pattern_vector + y * pattern_width * pattern_size,
        number_of_synapses,
        pattern_size,
        neuron_begin,
        neuron_end,
        inc_width, y);
  }
}

void
WNN_train_neuron_GPU(
  unsigned int *pattern_vector,
  int pattern_width,
  int pattern_height,
  synapsesDistribution neuronSynapses_GPU,
  unsigned  int *bit_pattern_vector,
  int number_of_synapses,
  int pattern_size,
  int neuron_begin,
  int neuron_end,
  int inc_width,
  int inc_height
  )
{
    //int size_width = pattern_width % inc_width == 0 ? pattern_width / inc_width : (pattern_width / inc_width) + 1;
    int size_height = pattern_height % inc_height == 0 ? pattern_height / inc_height : (pattern_height / inc_height) + 1;
    int synapses_threads = number_of_synapses < BLOCK_SIZE ? number_of_synapses : BLOCK_SIZE;
    synapses_threads = synapses_threads < 32 ? 32 : synapses_threads;

    dim3 dimGrid (pattern_width, size_height);
    dim3 dimBlock(synapses_threads);

    WNN_train_neuron_GPU_kernel<<<dimGrid,dimBlock>>>(
              pattern_vector,
              pattern_width,
              pattern_height,
	      neuronSynapses_GPU,
              bit_pattern_vector,
              number_of_synapses,
              pattern_size,
              neuron_begin,
              neuron_end,
              inc_width,
              inc_height
              );

}






__device__ void
WNN_disparity_line_GPU_func(
    unsigned  int *left_pattern,
    unsigned  int *right_pattern,
    int width,
    int height,
    int pattern_size,
    int MAX_Disparity,
    unsigned int *HamDist,
    unsigned short *disps,
    int begin,
    int end,
    int inc_width)
{

  int neuron = begin + blockIdx.x * inc_width ;
  int MAX_size,aux;

  if ( neuron < end )
  {
    aux = width - neuron;
    MAX_size = aux > MAX_Disparity ? MAX_Disparity : aux;
    find_nearest_patterns_GPU_func(
			right_pattern + neuron * pattern_size,
                        left_pattern + neuron * pattern_size,
                        HamDist + neuron * MAX_Disparity,
                        disps + neuron * MAX_Disparity,
                        MAX_size,
                        pattern_size);

  }
}

__global__ void
WNN_disparity_GPU_kernel(
        bi_dimention_vg_ram_wnn left,
        bi_dimention_vg_ram_wnn right,
        unsigned int *HamDist,
        unsigned short *disps,
        int MAX_Disparity,
        int begin,
        int end,
        int inc_height,
        int inc_width)
{
  int line = blockIdx.y * inc_height;

    if( line < left.number_of_layer_height)
    {
      //Foreward train/test -> left to right
     int aux = line * left.number_of_layer_width;
     WNN_disparity_line_GPU_func(
          left.pattern + aux * left.pattern_size,
          right.pattern + aux * left.pattern_size,
          left.number_of_layer_width,
          left.number_of_layer_height,
          left.pattern_size,
          MAX_Disparity,
          HamDist + aux * MAX_Disparity,
          disps + aux * MAX_Disparity,
          begin,
          end,
          inc_width);
    }

}

void
WNN_disparity_GPU(
        bi_dimention_vg_ram_wnn left,
        bi_dimention_vg_ram_wnn right,
        unsigned int *HamDist,
        unsigned short *disps,
        int MAX_Disparity,
        int begin,
        int end,
        int inc_height,
        int inc_width)
{
    int size_width = left.number_of_layer_width % inc_width == 0 ? left.number_of_layer_width / inc_width : (left.number_of_layer_width / inc_width) + 1;
    int size_height = left.number_of_layer_height % inc_height == 0 ? left.number_of_layer_height / inc_height : (left.number_of_layer_height / inc_height) + 1;



    dim3 dimGrid (size_width, size_height);
    dim3 dimBlock(64);

    WNN_disparity_GPU_kernel<<<dimGrid, 128>>>(
        left,
        right,
        HamDist,
        disps,
        MAX_Disparity,
        begin,
        end,
        inc_height,
        inc_width);
}


// __device__ void
// reduction(unsigned int *ham_dist, unsigned short *disps, int size)
// {
// }



__global__ void
calc_disparity_maps_GPU_kernel(
      DisparityMap DispMap,
      int MAX_Disparity,
      unsigned int *HamDist,
      unsigned short *disps,
      int begin,
      int end,
      int inc_width,
      int inc_height
      )
{
  int y = blockIdx.y * inc_height;
  int tid = blockIdx.x* blockDim.x + threadIdx.x;

  if( y < DispMap.height )
  {
    int x = begin + tid * inc_width;
    if( x < end )
    {
      int aux =  y * DispMap.width * MAX_Disparity + x * MAX_Disparity;
      select_disparity_GPU_func(DispMap,HamDist + aux, disps + aux,MAX_Disparity,x,y);
    }
  }

}
void
calc_disparity_maps_GPU(
      DisparityMap DispMap,
      int MAX_Disparity,
      unsigned int *HamDist,
      unsigned short *disps,
      int begin,
      int end,
      int inc_width,
      int inc_height
      )
{
    int size_height = DispMap.height % inc_height == 0 ? DispMap.height / inc_height : (DispMap.height / inc_height) + 1;

    int size_width = DispMap.width % inc_width == 0 ? DispMap.width / inc_width : (DispMap.width / inc_width) + 1;

    if(size_width < BLOCK_SIZE)
	size_width = 1;
    else
	size_width = size_width % BLOCK_SIZE == 0 ? size_width / BLOCK_SIZE : (size_width / BLOCK_SIZE) + 1;

    dim3 dimGrid (size_width, size_height);
    dim3 dimBlock(BLOCK_SIZE);

      calc_disparity_maps_GPU_kernel<<<dimGrid,dimBlock>>>(
	    DispMap,
	    MAX_Disparity,
	    HamDist,
	    disps,
	    begin,
	    end,
	    inc_width,
	    inc_height);

}

// void WinnerTakesItAll(DisparityMap disps, int iteration)
// {
// 	int 	l,k,i,j;
// 	int	opinion;
// 	int	highest_opinion_index;
//
// 	unsigned char*	disp_pixel;
// 	unsigned char*	central_pixel;
// 	int 	aux;
//
// 	for(l=0; l<iteration; l++)
// 	{
// 		//printf("iter: %d\n",l);
// #pragma omp parallel for default(none) private(i,j,k,opinion,disp_pixel,highest_opinion_index,central_pixel,aux) shared(disps, inc_stereo_height, inc_stereo_width,stereo_horizontal_ROI_ini,stereo_horizontal_ROI_end)
// 		for(i = 0; i<disps.height; i += inc_stereo_height)
// 		{
// 			for(j = stereo_horizontal_ROI_ini; j<stereo_horizontal_ROI_end; j += inc_stereo_width)
// 			{
// 				disp_pixel = disps.disps + (i * disps.width + j);
// 				//highest_opinion = 0;
// 				highest_opinion_index = 0;
//
// 				for(k = 0; k<disps.numMaps; k++)
// 				{
// 					opinion = 0;
// 					central_pixel = CENTRAL(k, disps.width, disps.height, disp_pixel);
//
// 					//UP
// 					if(!(i==0))
// 						if(*central_pixel == *UP(disps.width,disp_pixel))
// 							opinion++;
//
// 					//UP_RIGHT
// 					if(!(i==0)&&!(j==disps.width-inc_stereo_width))
// 						if(*central_pixel == *UP_RIGHT(disps.width,disp_pixel))
// 							opinion++;
//
// 					//RIGHT
// 					if(!(j==disps.width-inc_stereo_width))
// 						if(*central_pixel == *RIGHT(disps.width,disp_pixel))
// 							opinion++;
//
// 					//LOW_RIGHT
// 					if(!(i==disps.height-inc_stereo_height)&&!(j==disps.width-inc_stereo_width))
// 						if(*central_pixel == *LOW_RIGHT(disps.width,disp_pixel))
// 							opinion++;
//
// 					//LOW
// 					if(!(i==disps.height-inc_stereo_height))
// 						if(*central_pixel == *LOW(disps.width,disp_pixel))
// 							opinion++;
//
// 					//LOW_LEFT
// 					if(!(i==disps.height-inc_stereo_height)&&!(j==0))
// 						if(*central_pixel == *LOW_LEFT(disps.width,disp_pixel))
// 							opinion++;
//
// 					//LEFT
// 					if(!(j==0))
// 						if(*central_pixel == *LEFT(disps.width,disp_pixel))
// 							opinion++;
//
// 					//UP_LEFT
// 					if(!(i==0)&&!(j==0))
// 						if(*central_pixel == *UP_LEFT(disps.width,disp_pixel))
// 							opinion++;
//
// 					// obtain the new highest opinion and the new highest opinion index
// 					if(opinion > 4)
// 					{
// 						//highest_opinion = opinion;
// 						highest_opinion_index = k;
// 					}
// 				}
//
// 				// if highest_opinion_index != 0
// 				if(highest_opinion_index)
// 				{
// 					aux = *disp_pixel;
// 					*disp_pixel = *CENTRAL(highest_opinion_index, disps.width, disps.height, disp_pixel);
// 					*CENTRAL(highest_opinion_index, disps.width, disps.height, disp_pixel) = aux;
// 				}
// 			}
// 		}
// 	}
// }



// SelectDisparity(DispMap,HamDist,disps,MAX_Disparity,neuron,center_y);*/

