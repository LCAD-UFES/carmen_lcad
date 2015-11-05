/*
 * Copyright (c) 2009, Morgan Quigley, Clemens Eppner, Tully Foote
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Modified Apr 6, 2010 by Adam Leeper - changed to use "image_transport"

/* Much of this code is either heavily inspired by or taken directly from the
 * camera1394 ROS driver by Jack O'Quin
 */

#include <signal.h>
#include <cstdio>
#include <ros/ros.h>
#include <ros/time.h>
#include <uvc_cam/uvc_cam.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#include "uvc_cam/UVCCamConfig.h"

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
	signal(SIGSEGV, SIG_DFL);
	fprintf(stderr, "Segmentation fault, stopping uvc camera driver.\n");
	ROS_ERROR("Segmentation fault, stopping uvc camera driver.");
	ros::shutdown();                      // stop the main loop
}

class UVCCamNode {
private:
	Driver::state_t state_;               // current driver state

	ros::NodeHandle privNH_;              // private node handle
	ros::NodeHandle camera_nh_;           // camera name space handle
	sensor_msgs::Image image_;
	sensor_msgs::CameraInfo cam_info_;
	std::string device_;
	std::string camera_name_;
	sensor_msgs::CvBridge bridge_;

	/** dynamic parameter configuration */
	typedef uvc_cam::UVCCamConfig Config;
	Config config_;

	/** camera calibration information */
	CameraInfoManager cinfo_;
	bool calibration_matches_;            // cam_info_ matches video mode

	uvc_cam::Cam *cam_;

	/** image transport interfaces */
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher image_pub_;

public:
	UVCCamNode():
		privNH_("~"),
		camera_nh_("camera"),
		cinfo_(camera_nh_),
		it_(camera_nh_)
	{
		state_ = Driver::CLOSED;
		calibration_matches_ = true;
		device_ = "/dev/video0";
		camera_name_ = "camera";
	};

	~UVCCamNode() {
		if(state_ != Driver::CLOSED) {
			closeCamera();
		}
	}

	/** Close camera device
	 *
	 *  postcondition: state_ is Driver::CLOSED
	 */
	void closeCamera() {
		if (state_ != Driver::CLOSED)
		{
			ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
			if(cam_) {
				delete cam_;
			}
			state_ = Driver::CLOSED;
		}
	}

	/** Open the camera device.
	 *
	 * @param newconfig configuration parameters
	 * @return true, if successful
	 *
	 * if successful:
	 *   state_ is Driver::OPENED
	 *   camera_name_ set to camera_name string
	 */
	bool openCamera(Config &newconfig)
	{
		bool success = true;

		try
		{
			ROS_INFO("opening uvc_cam at %dx%d, %f fps", newconfig.width, newconfig.height, newconfig.frame_rate);
                        uvc_cam::Cam::mode_t mode = uvc_cam::Cam::MODE_RGB;
                        switch (newconfig.format_mode) {
                                case 1:
                                  mode = uvc_cam::Cam::MODE_RGB;
                                case 2:
                                  mode = uvc_cam::Cam::MODE_YUYV;
                                case 3:
                                  mode = uvc_cam::Cam::MODE_MJPG;
                                default:
                                  mode = uvc_cam::Cam::MODE_RGB;
                        }
			cam_ = new uvc_cam::Cam(newconfig.device.c_str(), mode, newconfig.width, newconfig.height, newconfig.frame_rate);
			if (camera_name_ != camera_name_)
			{
				camera_name_ = camera_name_;
				if (!cinfo_.setCameraName(camera_name_))
				{
					// GUID is 16 hex digits, which should be valid.
					// If not, use it for log messages anyway.
					ROS_WARN_STREAM("[" << camera_name_ << "] name not valid"
							<< " for camera_info_manger");
				}
			}
			//			ROS_INFO_STREAM("[" << camera_name_
			//					<< "] opened: " << newconfig.video_mode << ", "
			//					<< newconfig.frame_rate << " fps, "
			//					<< newconfig.iso_speed << " Mb/s");
			state_ = Driver::OPENED;
			calibration_matches_ = true;

		//	cam_->display_formats_supported();

		}
		catch (uvc_cam::Exception& e)
		{
			ROS_FATAL_STREAM("[" << camera_name_
					<< "] exception opening device: " << e.what());
			success = false;
		}

		return success;
	}

	/** Read camera data.
	 *
	 * @return true if successful
	 */
	bool read() {
		bool success = true;
		IplImage *imageIpl = cvCreateImageHeader(cvSize(config_.width,config_.height), 8, 3);
		try
		{
			// Read data from the Camera
			ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
			unsigned char *frame = NULL;
			uint32_t bytes_used;
			int buf_idx = cam_->grab(&frame, bytes_used);
			if (buf_idx < 0) {
				ROS_WARN("Could not grab image");
			}
			else if (frame)
			{
				//cv::WImageBuffer3_b image( frame );
				//cv::Mat data(height, width, CV_8UC1, frame, 3 * width);
				imageIpl->imageData = (char *)frame;
				image_ = *bridge_.cvToImgMsg( imageIpl, "bgr8");
				cam_->release(buf_idx);
			}
			ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
		}
		catch (uvc_cam::Exception& e)
		{
			ROS_WARN_STREAM("[" << camera_name_
					<< "] Exception reading data: " << e.what());
			success = false;
		}
		cvReleaseImageHeader(&imageIpl);
		return success;
	}

	/** Publish camera stream topics
	 *
	 *  @pre image_ contains latest camera frame
	 */
	void publish()
	{
		image_.header.frame_id = config_.frame_id;
		image_.header.stamp = ros::Time::now();

		// get current CameraInfo data
		cam_info_ = cinfo_.getCameraInfo();

		if (cam_info_.height != image_.height || cam_info_.width != image_.width)
		{
			// image size does not match: publish a matching uncalibrated
			// CameraInfo instead
			if (calibration_matches_)
			{
				// warn user once
				calibration_matches_ = false;
				ROS_WARN_STREAM("[" << camera_name_
						<< "] calibration does not match video mode "
						<< "(publishing uncalibrated data)");
			}
			cam_info_ = sensor_msgs::CameraInfo();
			cam_info_.height = image_.height;
			cam_info_.width = image_.width;
		}
		else if (!calibration_matches_)
		{
			// calibration OK now
			calibration_matches_ = true;
			ROS_INFO_STREAM("[" << camera_name_
					<< "] calibration matches video mode now");
		}

		cam_info_.header.frame_id = config_.frame_id;
		cam_info_.header.stamp = image_.header.stamp;

		// @todo log a warning if (filtered) time since last published
		// image is not reasonably close to configured frame_rate

		// Publish via image_transport
		image_pub_.publish(image_, cam_info_);
	}

	/** Dynamic reconfigure callback
	 *
	 *  Called immediately when callback first defined. Called again
	 *  when dynamic reconfigure starts or changes a parameter value.
	 *
	 *  @param newconfig new Config values
	 *  @param level bit-wise OR of reconfiguration levels for all
	 *               changed parameters (0xffffffff on initial call)
	 **/
	void reconfig(Config &newconfig, uint32_t level)
	{
		ROS_DEBUG("dynamic reconfigure level 0x%x", level);

		// resolve frame ID using tf_prefix parameter
		if (newconfig.frame_id == "")
			newconfig.frame_id = "camera";
		std::string tf_prefix = tf::getPrefixParam(privNH_);
		ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
		newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

		if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
		{
			// must close the device before updating these parameters
			closeCamera();                  // state_ --> CLOSED
		}

		if (state_ == Driver::CLOSED)
		{
			// open with new values
			if (openCamera(newconfig))
			{
				// update camera name string
				newconfig.camera_name = camera_name_;
			}
		}
		
		
		///THIS IS A QUICK HACK TO GET EXPOSURE ON OUR CAMERA'S THIS DOES NOT WORK FOR ALL CAMERAS

		if(config_.exposure != newconfig.exposure){
			try {
				  cam_->set_control(0x9a0901, newconfig.exposure);
			} catch (uvc_cam::Exception& e) {
				  ROS_ERROR_STREAM("Problem setting exposure. Exception was " << e.what());
			}
		}
		if(config_.absolute_exposure != newconfig.absolute_exposure){
			try {
		  cam_->set_control(0x9a0902, newconfig.absolute_exposure);
		  	} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting absolute exposure. Exception was " << e.what());
			}
		}
		if(config_.sharpness != newconfig.sharpness){
			try {
		  cam_->set_control(0x98091b, newconfig.sharpness);
			} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting sharpness. Exception was " << e.what());
			}
		}
		if(config_.power_line_frequency != newconfig.power_line_frequency){
			try {
		  cam_->set_control(0x980918, newconfig.power_line_frequency);
			} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting powerline frequency. Exception was " << e.what());
			}
		}
		if(config_.white_balance_temperature != newconfig.white_balance_temperature){
			try {
		  cam_->set_control(0x98090c, newconfig.white_balance_temperature);
		  	} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting white balance temperature. Exception was " << e.what());
			}
		}
		if(config_.gain != newconfig.gain){
			try {
		  cam_->set_control(0x980913, newconfig.gain);
		  	} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting gain. Exception was " << e.what());
			}
		}
		if(config_.saturation != newconfig.saturation){
			try {
		  cam_->set_control(0x980902, newconfig.saturation);
		  	} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting saturation. Exception was " << e.what());
			}
		}
		if(config_.contrast != newconfig.contrast){
			try {
		  cam_->set_control(0x980901, newconfig.contrast);
		  	} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting contrast. Exception was " << e.what());
			}
		}
		if(config_.brightness != newconfig.brightness){
			try {
		  cam_->set_control(0x980900, newconfig.brightness);
		  	} catch (uvc_cam::Exception& e) {
				ROS_ERROR_STREAM("Problem setting brightness. Exception was " << e.what());
			}
		}
		
		

		if (config_.camera_info_url != newconfig.camera_info_url)
		{
			// set the new URL and load CameraInfo (if any) from it
			if (cinfo_.validateURL(newconfig.camera_info_url))
			{
				cinfo_.loadCameraInfo(newconfig.camera_info_url);
			}
			else
			{
				// new URL not valid, use the old one
				newconfig.camera_info_url = config_.camera_info_url;
			}
		}

		//	    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
		//	      {
		//	        // configure IIDC features
		//	        if (level & Levels::RECONFIGURE_CLOSE)
		//	          {
		//	            // initialize all features for newly opened device
		//	            if (false == dev_->features_->initialize(&newconfig))
		//	              {
		//	                ROS_ERROR_STREAM("[" << camera_name_
		//	                                 << "] feature initialization failure");
		//	                closeCamera();          // can't continue
		//	              }
		//	          }
		//	        else
		//	          {
		//	            // update any features that changed
		//	            dev_->features_->reconfigure(&newconfig);
		//	          }
		//	      }

		config_ = newconfig;                // save new parameters

		ROS_DEBUG_STREAM("[" << camera_name_
				<< "] reconfigured: frame_id " << newconfig.frame_id
				<< ", camera_info_url " << newconfig.camera_info_url);
	}

	/** driver main spin loop */\
	void spin() {

		ros::NodeHandle node;

		// define segmentation fault handler in case the underlying uvc driver craps out
		signal(SIGSEGV, &sigsegv_handler);

		// Define dynamic reconfigure callback, which gets called
		// immediately with level 0xffffffff.  The reconfig() method will
		// set initial parameter values, then open the device if it can.
		dynamic_reconfigure::Server<Config> srv;
		dynamic_reconfigure::Server<Config>::CallbackType f
		= boost::bind(&UVCCamNode::reconfig, this, _1, _2);
		srv.setCallback(f);

		image_pub_ = it_.advertiseCamera("image_raw", 1);

		while (node.ok())
		{
			if (state_ != Driver::CLOSED)
			{
				if (read())
				{
					publish();
				}
			}

			ros::spinOnce();
		}

		closeCamera();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uvc_cam_node");

	ros::NodeHandle node;
	UVCCamNode cam;

	cam.spin();

	return 0;
}

