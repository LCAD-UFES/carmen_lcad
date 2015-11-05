#ifndef NO_CUDA
#include <cuda.h>
#endif

#include "vg_ram.h"

#ifndef VG_RAM_GPU_H
#define VG_RAM_GPU_H

void calc_disparity_maps_GPU(
       DisparityMap DispMap,
       int MAX_Disparity,
       unsigned int *HamDist,
       unsigned short *disps,
       int begin,
       int end,
       int inc_width,
       int inc_height);

 void WNN_disparity_GPU(
         bi_dimention_vg_ram_wnn left,
         bi_dimention_vg_ram_wnn right,
         unsigned int *HamDist,
         unsigned short *disps,
         int MAX_Disparity,
         int begin,
         int end,
         int inc_height,
         int inc_width);

void WNN_train_neuron_GPU(
   unsigned int *pattern_vector,
   int pattern_width,
   int pattern_height,
   synapsesDistribution neuronSynapses_GPU,
   unsigned  int *bit_pattern_vector,
   int number_of_synapses,
   int pattern_size, /*sizeof*/
   int neuron_begin,
   int neuron_end,
   int inc_width,
   int inc_height);

void
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
	  DisparityMap DispMap);

void  alloc_disparity_map_on_GPU(DisparityMap *disp,int w, int h, int numOfDisparity );

void allocSynapsesDistribution_on_GPU(synapsesDistribution *neuronSynapses, int numSynapses);

void alloc_vg_ram_WNN_multi_neuron_on_GPU(int number_of_neurons, int pattern_size, int number_of_layer_width, int number_of_layer_height, int number_of_synapses, bi_dimention_vg_ram_wnn *stereo_wnn_out);

void rgb_to_gray_GPU(Image img, unsigned int *gray, int stereo_vertical_ROI_ini, int gray_offset);

void alloc_image_on_GPU(int width, int height, int channels, Image *img);

void allocGPUData_vg(void **devPtr,size_t size);

void GPUcopy_vg(void * dest, void *src, size_t size, int type);

void GPUmemset_vg(void *devPtr, int value, size_t count);

void alloc_disparity_map_on_GPU(DisparityMap *disp,int w, int h, int numOfDisparity);

#endif
