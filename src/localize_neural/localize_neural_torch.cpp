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
//#include "localize_neural_util.h"

static int input_width = 320;
static int input_height = 240;
static int input_channels = 3;

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
	}

	~TorchModel ()
	{
		THCudaShutdown(state_gpu);
		lua_close(state);
	}

	void
	allocate ()
	{
		keytensor_gpu = THCudaTensor_newWithSize4d(state_gpu, 1, input_channels, input_height, input_width);
		keytensor = THFloatTensor_newWithSize4d(1, input_channels, input_height, input_width);
		if (!THFloatTensor_isContiguous(keytensor))
			throw std::runtime_error("Torch tensor is not contiguous.");
		curtensor_gpu = THCudaTensor_newWithSize4d(state_gpu, 1, input_channels, input_height, input_width);
		curtensor = THFloatTensor_newWithSize4d(1, input_channels, input_height, input_width);
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
	convert_image2tensor (IplImage * frame, THFloatTensor * tensor)
	{
		for(int i=0; i < frame->height; i++)
		{
			for(int j=0; j < frame->width; j++)
			{
				for(int k=0; k < frame->nChannels; k++)
				{
					float pixel = ((uchar)frame->imageData[frame->nChannels*(i*frame->width + j) + k]);
					//THFloatTensor_set4d(tensor, 0, k, i, j, (pixel-128.0f)/255.0f);
					THFloatTensor_set4d(tensor, 0, k, i, j, pixel/255.0f);
					//THFloatTensor_set4d(tensor, 0, k, i, j, pixel);
				}
			}
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
		allocate();

		convert_image2tensor(curframe, curtensor);
		convert_image2tensor(keyframe, keytensor);

		THCudaTensor_copyFloat(state_gpu, curtensor_gpu, curtensor);
		THCudaTensor_copyFloat(state_gpu, keytensor_gpu, keytensor);

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

		//release(); Lua's garbage collector takes care of it

		return output;
	}

};


static
TorchModel *torch = NULL;


void
initialize_network(const char * saved_network)
{
	torch = new TorchModel("../src/localize_neural/torch_config.lua", saved_network);
}


carmen_pose_3D_t
forward_network(const carmen_localize_neural_imagepos_message &curframe, const carmen_localize_neural_imagepos_message &keyframe)
{
	carmen_pose_3D_t delta_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	IplImage *keyframe_image = cvCreateImage(cvSize(keyframe.width, keyframe.height), IPL_DEPTH_8U, 3);
	IplImage *curframe_image = cvCreateImage(cvSize(curframe.width, curframe.height), IPL_DEPTH_8U, 3);
	IplImage *keyframe_image_input = cvCreateImage(cvSize(input_width, input_height), IPL_DEPTH_8U, input_channels);
	IplImage *curframe_image_input = cvCreateImage(cvSize(input_width, input_height), IPL_DEPTH_8U, input_channels);
	cv::Rect crop;
	crop.x = 160;//-1;
	crop.y = 140;//-1;
	crop.width = input_width;
	crop.height = input_height;

	copy_image(keyframe.image_data, keyframe_image, keyframe.width, keyframe.height);
	copy_image(curframe.image_data, curframe_image, curframe.width, curframe.height);

	cvSetImageROI(keyframe_image, crop);
	cvSetImageROI(curframe_image, crop);

	if (input_channels == 3)
	{
		cvCvtColor(keyframe_image, keyframe_image_input, CV_BGR2RGB);
		cvCvtColor(curframe_image, curframe_image_input, CV_BGR2RGB);
	}
	else
	{
		cvCvtColor(keyframe_image, keyframe_image_input, CV_BGR2GRAY);
		cvCvtColor(curframe_image, curframe_image_input, CV_BGR2GRAY);
	}

	std::vector<float> output = torch->forward(curframe_image_input, keyframe_image_input);
	delta_pose.orientation.roll  = 0;
	delta_pose.orientation.pitch = output[0];
	delta_pose.orientation.yaw 	 = 0;
	delta_pose.position.x = output[1];
	delta_pose.position.y = 0;
	delta_pose.position.z = output[2];

	/*
	delta_pose.orientation.roll  = output[0];
	delta_pose.orientation.pitch = output[1];
	delta_pose.orientation.yaw 	 = output[2];
	delta_pose.position.x = output[3];
	delta_pose.position.y = output[4];
	delta_pose.position.z = output[5];
	*/

	cvRelease((void**) &keyframe_image_input);
	cvRelease((void**) &curframe_image_input);
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
