#include <mvog_gtk_gui/camera.h>

namespace MVOG
{

Camera::Camera()
{
	look_.setX(0);
	look_.setY(0);
	look_.setZ(0);

	lookR_ = 10.0;
	
	lookTheta_ =  -45.0 * DEG_TO_RAD;
	lookPhi_    =  90.0 * DEG_TO_RAD;

	updatePos();
}

Camera::~Camera()
{

}

void Camera::updatePos()
{

	pos_.setX(-lookR_ * cos(lookTheta_)*cos(lookPhi_) + look_.getX());
	pos_.setY(-lookR_ * cos(lookTheta_)*sin(lookPhi_) + look_.getY());
	pos_.setZ(-lookR_ * sin(lookTheta_)               + look_.getZ());
}

void Camera::updateLook()
{
	look_.setX(lookR_ * cos(lookTheta_)*cos(lookPhi_) + pos_.getX());
	look_.setY(lookR_ * cos(lookTheta_)*sin(lookPhi_) + pos_.getY());
	look_.setZ(lookR_ * sin(lookTheta_)               + pos_.getZ());
}

void Camera::pan(double a)
{
	lookPhi_ += a;
	updatePos();
}

void Camera::tilt(double a)
{
	lookTheta_ += a;

	if (lookTheta_ >  89.99 * DEG_TO_RAD) lookTheta_ =  89.99 * DEG_TO_RAD;
	if (lookTheta_ < -89.99 * DEG_TO_RAD) lookTheta_ = -89.99 * DEG_TO_RAD;

	updatePos();
}

void Camera::zoom (double r)
{
	lookR_ = lookR_*r;

	if (lookR_ < 0.1) lookR_ = 0.1;

	updatePos();
}

void Camera::move (double x, double y)
{
  btVector3 d = pos_ - look_;

  btVector3 y_v = d.cross(btVector3(0.0, 0.0, 1.0));
  btVector3 z_v = y_v.cross(d);

  y_v.normalize();
  z_v.normalize();

  y_v *= d.length();
  z_v *= d.length();

  btVector3 y_d = y_v * x;
  btVector3 z_d = z_v * y; 

  pos_  -= (y_d + z_d);
  look_ -= (y_d + z_d);
}

} // namespace MVOG
