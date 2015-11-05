#include <robot.h>
#include <tf.h>

Robot::Robot(char* model_name)
{
	this->robot_model_ = glmReadOBJ(model_name);
	this->robot_pose_length = 0;
	this->neural_pose_length = 0;
	this->localize_pose_length = 0;

	glmUnitize(this->robot_model_);
}

void
Robot::setRobotParameters(carmen_vector_3D_t robot_size)
{
	this->robot_size_  = robot_size;

	if(this->robot_model_ != NULL)
		glmScale(this->robot_model_, this->robot_size_.x / 1.7);
}

carmen_pose_3D_t Robot::getRobotPose()
{
	carmen_pose_3D_t erro;

	if(robot_pose_length > 0)
		return robot_pose_[robot_pose_length-1];

	return erro;
}

carmen_pose_3D_t Robot::getLocalizeAckermanPose()
{
	carmen_pose_3D_t erro;

	if(localize_pose_length > 0)
		return localize_pose_[localize_pose_length-1];

	return erro;
}

carmen_pose_3D_t Robot::getNeuralPose()
{
	carmen_pose_3D_t erro;

	if(neural_pose_length > 0)
		return neural_pose_[neural_pose_length-1];

	return erro;
}


void
Robot::drawPath(bool correction, bool neural_global)
{
	glPushMatrix();
	glColor3f(0.0, 1.0, 0.0);
	glPointSize(2.0);

	for(int i = 0; i < robot_pose_length; i++)
	{
		glBegin(GL_POINTS);

		if(correction)
		{
			glColor3f(0.0, 1.0, 0.0);
			glVertex3d(robot_pose_[i].position.x, robot_pose_[i].position.y, robot_pose_[i].position.z);
		}

		if(neural_global)
		{
			glColor3f(1.0, 0.0, 0.0);
			glVertex3d(neural_pose_[i].position.x, neural_pose_[i].position.y, neural_pose_[i].position.z);
		}
		glEnd();
	}
	glPopMatrix();
}

void
Robot::drawLocalizePath()
{
	glPushMatrix();
	glColor3f(0.0, 0.3, 0.8);
	glPointSize(2.0);

	for(int i = 0; i < localize_pose_length; i++)
	{
		glBegin(GL_POINTS);
		glColor3f(0.0, 0.3, 0.8);
		glVertex3d(localize_pose_[i].position.x, localize_pose_[i].position.y, localize_pose_[i].position.z);
		glEnd();
	}
	glPopMatrix();
}

void
Robot::drawEllipse()
{
	glColor3f(0.6, 0.34, 0.8);

	tf::Transform robot_pose, opengl_to_robot_pose;
	robot_pose.setOrigin(tf::Vector3(getRobotPose().position.x, getRobotPose().position.y, getRobotPose().position.z));
	robot_pose.setRotation(tf::Quaternion(getRobotPose().orientation.yaw, getRobotPose().orientation.pitch, getRobotPose().orientation.roll));

	glPushMatrix();

	glTranslatef(robot_pose.getOrigin().x(),robot_pose.getOrigin().y(), robot_pose.getOrigin().z());
	glRotatef(this->angle_, 0.0f, 0.0f, 1.0f);


	glBegin(GL_LINE_LOOP);
	for(int i=0; i < 360; i++)
	{
		//convert degrees into radians
		float degInRad = carmen_degrees_to_radians((double)i);
		glVertex2f(cos(degInRad)*this->minor_axis_,sin(degInRad)*this->major_axis_);
	}
	glEnd();

	glPopMatrix();
}

void
Robot::draw()
{
	tf::Transform robot_pose, opengl_to_robot_pose;
	robot_pose.setOrigin(tf::Vector3(getRobotPose().position.x, getRobotPose().position.y, getRobotPose().position.z));
	robot_pose.setRotation(tf::Quaternion(getRobotPose().orientation.yaw, getRobotPose().orientation.pitch, getRobotPose().orientation.roll));

	opengl_to_robot_pose.setOrigin(tf::Vector3(1.4, 0.0, 0.0));
	opengl_to_robot_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	robot_pose = robot_pose * opengl_to_robot_pose;

	if(this->robot_model_ != NULL)
	{
		glPushMatrix();
		glTranslatef(robot_pose.getOrigin().x(),robot_pose.getOrigin().y(), robot_pose.getOrigin().z() + 0.28);

		double y, p, r;
		tf::Matrix3x3(robot_pose.getRotation()).getRPY(r, p, y);

		glRotatef(carmen_radians_to_degrees(y), 0.0f, 0.0f, 1.0f);
		glRotatef(carmen_radians_to_degrees(p), 0.0f, 1.0f, 0.0f);
		glRotatef(carmen_radians_to_degrees(r), 1.0f, 0.0f, 0.0f);
		glRotatef(90.0, 1.0, 0.0, 0.0);
		glRotatef(0.0, 0.0, 1.0, 0.0);

		glmDraw(this->robot_model_, GLM_SMOOTH | GLM_COLOR);
		glPopMatrix();
	}
}

Robot::~Robot()
{

}

