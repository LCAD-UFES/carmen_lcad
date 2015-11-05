

#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>

#include <pcl/console/parse.h>

#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/angles.h>

#include <pcl/gpu/kinfu/tsdf_volume.h>
#include <tsdf_volume.hpp>

typedef pcl::ScopeTime ScopeTimeT;

#include <internal.h>
#include "kinfu_wrapper.h"

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{          
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms, int i) : time_ms_(time_ms), i_(i) {}
  ~SampledScopeTime()
  {
    time_ms_ += (int)stopWatch_.getTime ();        
    if (i_ % EACH == 0 && i_)
    {
      cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << endl;
      time_ms_ = 0;        
    }
  }
private:
    StopWatch stopWatch_;
    int& time_ms_;
    int i_;
};

struct DepthSource
{
	DepthSource(int width, int height, float fx, float fy, float b, unsigned short m)
	{
		width_ = width;
		height_ = height;
		data_ = (unsigned short*) malloc ((width * height) * sizeof(unsigned short));
		memset(data_, 0, width_ * height_);

		depth_focal_length_VGA_x = fx;
		depth_focal_length_VGA_y = fy;
		baseline = b;
		max_depth = m;
	}

	void set_depth_raw_data(unsigned short* data)
	{
		for(int i=0; i < width_ * height_; i++)
			data_[i] = data[i];
	}

	bool get_depth_raw_data(PtrStepSz<const unsigned short>& depth)
	{
		if(data_ != NULL)
		{
			depth.cols = width_;
			depth.rows = height_;
			depth.step = 2 * width_;
			depth.data = data_;
			return true;
		}
		else
			return false;
	}

	int width_, height_;
	unsigned short *data_;

	float depth_focal_length_VGA_x, depth_focal_length_VGA_y;
	float baseline;						//mm
	unsigned short max_depth;			//mm

};

struct KinFuApp
{
  
  KinFuApp(DepthSource* source, float vsz) : depth_source_ (source)
  {    
    //Init Kinfu Tracker
    Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);

    float fx = depth_source_->depth_focal_length_VGA_x;
    float fy = depth_source_->depth_focal_length_VGA_y;
    kinfu_.setDepthIntrinsics (fx, fy);
    kinfu_.volume().setSize (volume_size);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    kinfu_.setInitalCameraPose (pose);
    kinfu_.volume().setTsdfTruncDist (0.030f/*meters*/);    
    kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
    //kinfu_.setDepthTruncationForICP(10.f/*meters*/);
    kinfu_.setCameraMovementThreshold(0.001f);
    
    //Init KinfuApp            
    //tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);    
  }

  ~KinFuApp()
  {
  }

  bool
  execute (double* position, double* rotation, double* orientation)
  {	
    bool has_image = false;

	PtrStepSz<const unsigned short> depth;
   
	//grab frames from evaluation folder or kinect device
	bool has_frame = depth_source_->get_depth_raw_data(depth);      
    if (!has_frame)
    {
		cout << "Can't grab" << endl;
		return false;
    }

	//upload data to gpu
    depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
      
	//kinfu algorithm
    {
    //SampledScopeTime fps(time_ms, i);
    //run kinfu algorithm
		has_image = kinfu_ (depth_device_);                  
    }

	if(has_image)
	{
		Eigen::Affine3f pose6d = kinfu_.getCameraPose();
		copy_affine3d_to_flat_array(pose6d, position, rotation, orientation);

		return true;
	}

	return false;
  }

  
  void copy_affine3d_to_flat_array(Eigen::Affine3f& transform, double* position, double* rotation, double* orientation)
  {
	  const float DEGREES_TO_RADIANS = 57.2957795;

	  position[0] =  2.0 - transform.translation().x();  //x
	  position[1] =  2.0 - transform.translation().y();  //y
	  position[2] =  0.4 + transform.translation().z();  //z

	  rotation[0] = transform.rotation()(0, 0);
	  rotation[1] = transform.rotation()(0, 1);
	  rotation[2] = transform.rotation()(0, 2);
	  rotation[3] = transform.rotation()(1, 0);
	  rotation[4] = transform.rotation()(1, 1);
	  rotation[5] = transform.rotation()(1, 2);
	  rotation[6] = transform.rotation()(2, 0);
	  rotation[7] = transform.rotation()(2, 1);
	  rotation[8] = transform.rotation()(2, 2);

	  Eigen::Vector3f rot = transform.rotation().eulerAngles(0, 1, 2);

	  orientation[0] = rot[0] * DEGREES_TO_RADIANS; //pitch
	  orientation[1] = rot[2] * DEGREES_TO_RADIANS; //roll
	  orientation[2] = rot[1] * DEGREES_TO_RADIANS; //yaw

   }
  
  DepthSource* depth_source_;
  KinfuTracker kinfu_;
  KinfuTracker::DepthMap depth_device_;

  pcl::TSDFVolume<float, short> tsdf_volume_;

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

KinFuApp* app;
DepthSource* depth_source;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool
initialize_kinfu(int width, int height, float focal_length_x, float focal_lenght_y, float baseline, unsigned short max_depth, float volume_size, int gpu_device)
{
  pcl::gpu::setDevice (gpu_device);
  pcl::gpu::printShortCudaDeviceInfo (gpu_device);

  if(checkIfPreFermiGPU(gpu_device))
    return false;

  depth_source = new DepthSource(width, height, focal_length_x, focal_lenght_y, baseline, max_depth);
  app = new KinFuApp(depth_source, volume_size);

  return true;
}

bool
execute_kinfu(double* position, double* rotation, double* orientation)
{ 
  bool result = false;
  // executing
  try { result = app->execute(position, rotation, orientation); }
  catch (const std::bad_alloc& e) { cout << "Bad alloc" << endl; }
  catch (const std::exception& e) { cout << "Exception" << endl; }

  return result;
}

void 
upload_depth_to_kinfu(unsigned short* depth)
{
	app->depth_source_->set_depth_raw_data(depth);
}

