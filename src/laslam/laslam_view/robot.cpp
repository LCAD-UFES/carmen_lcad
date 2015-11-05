#include <robot.h>

Robot::Robot(char* model_name)
{
	this->robot_model_ = glmReadOBJ(model_name);

	glmUnitize(this->robot_model_);
}

void
Robot::setRobotParameters(carmen_vector_3D_t robot_size)
{
	this->robot_size_  = robot_size;

	if(this->robot_model_ != NULL)
		glmScale(this->robot_model_, this->robot_size_.x);
}

void
Robot::draw()
{
	if(this->robot_model_ != NULL)
	{
		glPushMatrix();
			glTranslatef(this->robot_pose_.position.x, this->robot_pose_.position.y, this->robot_pose_.position.z);
			glRotatef(carmen_radians_to_degrees(this->robot_pose_.orientation.yaw), 0.0f, 0.0f, 1.0f);
			glRotatef(carmen_radians_to_degrees(this->robot_pose_.orientation.pitch), 0.0f, 1.0f, 0.0f);
			glRotatef(carmen_radians_to_degrees(this->robot_pose_.orientation.roll), 1.0f, 0.0f, 0.0f);
			glRotatef(90.0, 1.0, 0.0, 0.0);
			glRotatef(180.0, 0.0, 1.0, 0.0);

			glmDraw(this->robot_model_, GLM_SMOOTH | GLM_COLOR);
		glPopMatrix();
	}
}

Robot::~Robot()
{

}

