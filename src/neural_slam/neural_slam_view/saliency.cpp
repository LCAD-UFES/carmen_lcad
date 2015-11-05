#include "saliency.h"

typedef struct {
	float r, g, b;
}color_table_t;

static color_table_t color_table[] = {{179.0 / 255.0, 124.0/ 255.0, 42/ 255.0},
									  {133/ 255.0, 179/ 255.0, 42/ 255.0},
									  {46/ 255.0, 179/ 255.0, 42/ 255.0},
									  {42/ 255.0, 179/ 255.0, 175/ 255.0},
									  {42/ 255.0, 92/ 255.0, 179/ 255.0}};

Saliency::Saliency(int saliencies_size, int color)
{
	this->color_ = color;
	this->saliencies_size_ = saliencies_size;
	this->saliencies.resize(this->saliencies_size_);

	for(int i = 0; i < this->saliencies_size_; i++)
		this->saliencies[i].cropped_image = NULL;

	this->saliencies_counter_ = 0;
}

Saliency::~Saliency() {
	// TODO Auto-generated destructor stub
}

void
Saliency::addSaliencyPoseToSalienciesList(carmen_vector_3D_t saliency_pose)
{
	carmen_saliency_opengl_t saliency;

	saliency.pose = saliency_pose;

	this->saliencies[this->saliencies_counter_] = saliency;
	this->saliencies_counter_++;

	if(this->saliencies_counter_ >= this->saliencies_size_)
		this->saliencies_counter_ = 0;
}

void
Saliency::draw(bool drawSaliency, bool drawDistanceInformation)
{
	if(drawSaliency)
		this->drawSaliency();

	if(drawDistanceInformation)
		this->drawDistanceInformation();
}

Eigen::Matrix<float, 3, 3>
Saliency::getSaliencyRotationMatrixPhi(carmen_saliency_opengl_t saliency)
{
	Eigen::Matrix<float, 3, 3> rotation_phi;

	//rotation
	rotation_phi(0, 0) = 1.0;
	rotation_phi(0, 1) = 0.0;
	rotation_phi(0, 2) = 0.0;
	rotation_phi(1, 0) = 0.0;
	rotation_phi(1, 1) = cos(saliency.phi);
	rotation_phi(1, 2) = -sin(saliency.phi);
	rotation_phi(2, 0) = 0.0;
	rotation_phi(2, 1) = sin(saliency.phi);
	rotation_phi(2, 2) = cos(saliency.phi);

	return rotation_phi;
}

Eigen::Matrix<float, 3, 3>
Saliency::getSaliencyRotationMatrixTheta(carmen_saliency_opengl_t saliency)
{
	Eigen::Matrix<float, 3, 3> rotation_theta;

	//rotation
	rotation_theta(0, 0) = cos(saliency.theta);
	rotation_theta(0, 1) = -sin(saliency.theta);
	rotation_theta(0, 2) = 0.0;
	rotation_theta(1, 0) = sin(saliency.theta);
	rotation_theta(1, 1) = cos(saliency.theta);
	rotation_theta(1, 2) = 0.0;
	rotation_theta(2, 0) = 0.0;
	rotation_theta(2, 1) = 0.0;
	rotation_theta(2, 2) = 1.0;

	return rotation_theta;
}

void
Saliency::drawSaliency()
{
	Eigen::Matrix<float, 3, 3> rotation_phi;
	Eigen::Matrix<float, 3, 3> rotation_theta;

	Eigen::Matrix<float, 3, 1> saliency_vertice1;
	Eigen::Matrix<float, 3, 1> saliency_vertice2;
	Eigen::Matrix<float, 3, 1> saliency_vertice3;
	Eigen::Matrix<float, 3, 1> saliency_vertice4;


	glPushMatrix();

		glPointSize(5.0);

		int k = 0;
		for(int i = this->saliencies_counter_ - 5; i < this->saliencies_counter_; i++)
		{
			glColor3f(color_table[k].r, color_table[k].g, color_table[k].b);
			k++;

			glBegin(GL_POINTS);
				glVertex3f(this->saliencies[i].pose.x, this->saliencies[i].pose.y, this->saliencies[i].pose.z);
			glEnd();
		}
	glPopMatrix();
}

void
Saliency::drawDistanceInformation()
{
	glPushMatrix();

		if(this->color_ == 0)
			glColor3f(1.0, 0.0, 0.0);
		else
			glColor3f(0.0, 1.0, 0.0);

		glLineWidth(1.0);
		glPointSize(5.0);


		tf::Transform camera_transform;
		tf::Transform robot_pose_transform;

		camera_transform.setOrigin(tf::Vector3(this->camera_pose_.position.x, this->camera_pose_.position.y, this->camera_pose_.position.z));
		camera_transform.setRotation(tf::Quaternion(this->camera_pose_.orientation.yaw, this->camera_pose_.orientation.pitch, this->camera_pose_.orientation.roll));

		robot_pose_transform.setOrigin(tf::Vector3(this->robot_pose_.position.x, this->robot_pose_.position.y, this->robot_pose_.position.z));
		robot_pose_transform.setRotation(tf::Quaternion(this->robot_pose_.orientation.yaw, this->robot_pose_.orientation.pitch, this->robot_pose_.orientation.roll));

		camera_transform = robot_pose_transform * camera_transform;

		glBegin(GL_POINTS);
			glVertex3f(camera_transform.getOrigin().x(), camera_transform.getOrigin().y(), camera_transform.getOrigin().z());
		glEnd();

		int k = 0;
		for(int i = this->saliencies_counter_ - 5; i < this->saliencies_counter_; i++)
		{
			if(this->color_ == 0)
				glColor3f(1.0, 0.0, 0.0);
			else
				glColor3f(0.0, 1.0, 0.0);

			k++;

			glBegin(GL_LINES);
				glVertex3f(camera_transform.getOrigin().x(), camera_transform.getOrigin().y(), camera_transform.getOrigin().z());
				glVertex3f(this->saliencies[i].pose.x, this->saliencies[i].pose.y, this->saliencies[i].pose.z);
			glEnd();
		}

	glPopMatrix();
}
