#include <string>
#include <vector>
#include <stdexcept>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <lua.hpp>
#include <luaT.h>
#include <TH/TH.h>
#include <THC/THC.h>
#include "localize_neural_torch.h"


class TorchModel
{
	lua_State *state;
	THCState *state_gpu;
	THCudaTensor *keytensor_gpu;
	THFloatTensor *keytensor;
	THCudaTensor *curtensor_gpu;
	THFloatTensor *curtensor;

public:
	TorchModel (std::string const &torch_config, std::string const& model_path): state(luaL_newstate())
	{
		state_gpu = (THCState*)malloc(sizeof(THCState));
		THCudaInit(state_gpu);

		if (!state)
			throw std::runtime_error("failed to initialize Lua");
		luaL_openlibs(state);
		if (luaL_loadfile(state, torch_config.c_str()) != 0)
			throw std::runtime_error("failed to load torch server");
		if (lua_pcall(state, 0, LUA_MULTRET, 0) != 0)
			throw std::runtime_error("failed to run torch server");
		lua_getglobal(state, "load");
		lua_pushstring(state, model_path.c_str());
		if (lua_pcall(state, 1, 0, 0) != 0)
			throw std::runtime_error("fail to load model");

		allocate();
	}

	~TorchModel ()
	{
		release();
		THCudaShutdown(state_gpu);
		lua_close(state);
	}

	void
	allocate ()
	{
		keytensor_gpu = THCudaTensor_newWithSize4d(state_gpu, 1, 3, 155, 320);
		keytensor = THFloatTensor_newWithSize4d(1, 3, 155, 320);
		if (!THFloatTensor_isContiguous(keytensor))
			throw std::runtime_error("Torch tensor is not contiguous.");
		curtensor_gpu = THCudaTensor_newWithSize4d(state_gpu, 1, 3, 155, 320);
		curtensor = THFloatTensor_newWithSize4d(1, 3, 155, 320);
		if (!THFloatTensor_isContiguous(curtensor))
			throw std::runtime_error("Torch tensor is not contiguous.");
	}

	void
	release ()
	{
		THFloatTensor_free(curtensor);
		THFloatTensor_free(keytensor);
		THCudaTensor_free(state_gpu, keytensor_gpu);
		THCudaTensor_free(state_gpu, curtensor_gpu);
	}

	void
	convert (IplImage * frame, THFloatTensor * tensor)
	{
		for(int i=0; i < frame->height; i++)
			for(int j=0; j < frame->width; j++)
				for(int k=0; k < frame->nChannels; k++)
				{
					float pixel = ((uchar)frame->imageData[3*i*frame->width + 3*j+k]);
					THFloatTensor_set4d(tensor, 0, k, i, j, pixel/255.0f);
				}
	}

	long
	size_byte(const THByteTensor * otensor)
	{
		long dim = THByteTensor_nDimension(otensor);
		size_t sz = 1;
		for (long i = 0; i < dim; ++i)
		{
			sz *= THByteTensor_size(otensor, i);
		}
		return sz;
	}

	long
	size_float(const THFloatTensor * otensor)
	{
		long dim = THFloatTensor_nDimension(otensor);
		size_t sz = 1;
		for (long i = 0; i < dim; ++i)
		{
			sz *= THFloatTensor_size(otensor, i);
		}
		return sz;
	}

	std::vector<float>
	forward (IplImage *curframe, IplImage *keyframe)
	{
		convert(curframe, curtensor);
		convert(keyframe, keytensor);

		THCudaTensor_copyFloat(state_gpu, keytensor_gpu, keytensor);
		THCudaTensor_copyFloat(state_gpu, curtensor_gpu, curtensor);

		// call "forward" in lua
		lua_getglobal(state, "forward");
		luaT_pushudata(state, (void*)curtensor_gpu, "torch.CudaTensor");
		luaT_pushudata(state, (void*)keytensor_gpu, "torch.CudaTensor");
		if (lua_pcall(state, 2, 1, 0) != 0)
		{
			throw std::runtime_error(lua_tostring(state, -1));
		}
		const THFloatTensor *otensor = reinterpret_cast<const THFloatTensor *>(luaT_toudata(state, -1, "torch.FloatTensor"));
		if (!THFloatTensor_isContiguous(otensor))
		{
			throw std::runtime_error("Torch output tensor is not contiguous.");
		}
		float const *ptr = THFloatTensor_data(otensor);
		size_t sz = size_float(otensor);
		std::vector<float> output(sz);
		for (unsigned int i = 0; i < sz; ++i)
		{
			output.at(i) = ptr[i];
		}
		lua_pop(state, 1);

		return output;
	}

};

void
copy_image (const char *rgb_buffer, IplImage *img, int width, int height)
{
	for(int i = 0; i < (height * width); i++)
	{
		/**
		 * A imagem da bumblebee usa o formato rgb-rgb-rgb, enquanto
		 * a imagem da opencv usa o formato bgr-bgr-bgr. As linhas
		 * abaixo fazem essa conversao.
		 */
		img->imageData[3 * i + 0] = (uchar)rgb_buffer[3 * i + 2];
		img->imageData[3 * i + 1] = (uchar)rgb_buffer[3 * i + 1];
		img->imageData[3 * i + 2] = (uchar)rgb_buffer[3 * i + 0];
	}
}


void
resize_image(IplImage **img, int width, int height)
{
	IplImage *resized_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvResize((*img), resized_image, CV_INTER_AREA);
	cvRelease((void**) img);
	(*img) = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvCopy(resized_image, (*img));
	cvRelease((void**) &resized_image);
}


static
TorchModel *torch = NULL;


void
initialize_network(const char * saved_network)
{
	torch = new TorchModel("../src/localize_neural/torch_config.lua", saved_network);
}


carmen_pose_3D_t
forward_network(const carmen_localize_neural_imagepos_message &keyframe, const carmen_localize_neural_imagepos_message &curframe)
{
	carmen_pose_3D_t delta_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	IplImage *keyframe_image = cvCreateImage(cvSize(keyframe.width, keyframe.height), IPL_DEPTH_8U, 3);
	IplImage *curframe_image = cvCreateImage(cvSize(curframe.width, curframe.height), IPL_DEPTH_8U, 3);
	int input_width = 320;
	int input_height = 155;
	cv::Rect crop;
	crop.x = 0;
	crop.y = 0;
	crop.width = input_width*4;
	crop.height = input_height*4;

	copy_image(keyframe.image_data, keyframe_image, keyframe.width, keyframe.height);
	copy_image(curframe.image_data, curframe_image, curframe.width, curframe.height);

	cvSetImageROI(keyframe_image, crop);
	cvSetImageROI(curframe_image, crop);

	resize_image(&keyframe_image, input_width, input_height);
	resize_image(&curframe_image, input_width, input_height);

	cvCvtColor(keyframe_image, keyframe_image, CV_BGR2RGB);
	cvCvtColor(curframe_image, curframe_image, CV_BGR2RGB);

	std::vector<float> output = torch->forward(keyframe_image, curframe_image);
	delta_pose.orientation.roll  = output[0];
	delta_pose.orientation.pitch = output[1];
	delta_pose.orientation.yaw 	 = output[2];
	delta_pose.position.x = output[3];
	delta_pose.position.y = output[4];
	delta_pose.position.z = output[5];

	cvRelease((void**) &keyframe_image);
	cvRelease((void**) &curframe_image);

	return delta_pose;
}


void
finalize_network()
{
	if (torch) {
		delete torch;
		torch = NULL;
	}
}
