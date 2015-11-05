#include <ros/ros.h>
//for ImageTransport, Publisher
#include <image_transport/image_transport.h>
// for a ton of boost-related shit
#include <boost/thread.hpp>
// for CameraInfo
#include <sensor_msgs/CameraInfo.h>

class ImageSync
{
private:
	ros::NodeHandle nh_priv_;
	image_transport::ImageTransport it_;

	sensor_msgs::ImagePtr left_img_, right_img_;
	sensor_msgs::CameraInfo left_info_, right_info_;

	image_transport::Subscriber l_img_sub_, r_img_sub_;
	image_transport::Publisher l_img_pub_, r_img_pub_;

	ros::Subscriber l_info_sub_, r_info_sub_;
	ros::Publisher l_info_pub_, r_info_pub_;

	boost::mutex l_img_mutex_, r_img_mutex_, l_info_mutex_, r_info_mutex_, flag_mutex_;

	bool new_left_img_;
	bool new_right_img_;
	bool new_left_info_;
	bool new_right_info_;

public:
	ImageSync( ros::NodeHandle & nh ) :
		nh_priv_( "~" ), it_( nh_priv_ )
	{
		new_left_img_ = new_right_img_ = new_left_info_ = new_right_info_ = false;

		l_img_sub_ = it_.subscribe( "image_l", 1, &ImageSync::imageLeftCB, this );
		r_img_sub_ = it_.subscribe( "image_r", 1, &ImageSync::imageRightCB, this );

		l_info_sub_ = nh_priv_.subscribe( "info_l", 1, &ImageSync::infoLeftCB, this );
		r_info_sub_ = nh_priv_.subscribe( "info_r", 1, &ImageSync::infoRightCB, this );

		l_img_pub_ = it_.advertise( "image_l_sync", 1 );

		r_img_pub_ = it_.advertise( "image_r_sync", 1 );

		l_info_pub_ = nh_priv_.advertise<sensor_msgs::CameraInfo> ( "info_l_sync", 1 );

		r_info_pub_ = nh_priv_.advertise<sensor_msgs::CameraInfo> ( "info_r_sync", 1 );
	}

	~ImageSync()
	{

	}

	void publishSyncdImgs()
	{
		if ( new_left_img_ && new_right_img_ && new_left_info_ && new_right_info_ )
		{
			ROS_DEBUG( "publishing..." );
			boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
			boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );
			boost::lock_guard<boost::mutex> linfoguard( l_info_mutex_ );
			boost::lock_guard<boost::mutex> rinfoguard( r_info_mutex_ );

			ros::Time now = ros::Time::now();

			left_img_->header.stamp = now;
			right_img_->header.stamp = now;
			left_info_.header.stamp = now;
			right_info_.header.stamp = now;

			l_img_pub_.publish( left_img_ );
			r_img_pub_.publish( right_img_ );
			l_info_pub_.publish( left_info_ );
			r_info_pub_.publish( right_info_ );


			//Reset new flags
			new_left_img_ = false;
			new_right_img_ = false;
			new_left_info_ = false;
			new_right_info_ = false;

			ROS_DEBUG( "done" );
		}
	}

	void imageLeftCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_DEBUG( "got left img" );
		l_img_mutex_.lock();
		left_img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

		l_img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_left_img_ = true;

		publishSyncdImgs();
	}

	void imageRightCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_DEBUG( "got right img" );
		r_img_mutex_.lock();
		right_img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

		r_img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_right_img_ = true;

		publishSyncdImgs();
	}

	void infoLeftCB( const sensor_msgs::CameraInfoConstPtr& msg )
	{
		ROS_DEBUG( "got left info" );
		l_info_mutex_.lock();
		left_info_ = *msg;

		l_info_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_left_info_ = true;

		publishSyncdImgs();
	}

	void infoRightCB( const sensor_msgs::CameraInfoConstPtr& msg )
	{
		ROS_DEBUG( "got right info" );
		r_info_mutex_.lock();
		right_info_ = *msg;

		r_info_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_right_info_ = true;

		publishSyncdImgs();
	}

	void spin()
	{
		ros::spin();
	}

};

int main( int argc, char * argv[] )
{
	ros::init( argc, argv, "image_sync" );
	ros::NodeHandle nh;

	ImageSync image_sync( nh );
	image_sync.spin();

	return 0;
}
