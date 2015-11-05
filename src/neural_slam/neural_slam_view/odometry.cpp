#include "odometry.h"

Odometry::Odometry(int path_size)
{
	this->path_size_ = path_size;
	this->path_.resize(this->path_size_);

	this->path_counter_ = 0;
}

void
Odometry::addOdometryPoseToPath(carmen_vector_3D_t odometry_pose)
{
	this->path_[this->path_counter_] = odometry_pose;
	this->path_counter_++;

	if(this->path_counter_ >= this->path_size_)
		this->path_counter_ = 0;
}

void
Odometry::draw()
{
	glPushMatrix();

		glColor3f(0.0, 1.0, 0.0);
		glPointSize(1.0);

		for(unsigned int i = 0; i < this->path_.size(); i++)
		{
			glBegin(GL_POINTS);
				glVertex3f(this->path_[i].x, this->path_[i].y, this->path_[i].z);
			glEnd();
		}

	glPopMatrix();
}

Odometry::~Odometry()
{

}
