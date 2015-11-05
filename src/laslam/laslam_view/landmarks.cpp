#include <landmarks.h>

Landmark::Landmark(int landmarks_size)
{
	this->landmarks_size_ = landmarks_size;
	this->landmarks.resize(this->landmarks_size_);

	for(int i = 0; i < this->landmarks_size_; i++)
		this->landmarks[i].cropped_image = NULL;

	this->landmarks_counter_ = 0;
}

Landmark::~Landmark() {
}

void
Landmark::addLandmarkPoseToLandmarksList(carmen_vector_3D_t landmark_pose, carmen_pose_3D_t robot_pose, unsigned char* crop)
{
	double vx, vy, vz, cat_adj;
	carmen_laslam_landmark_t landmark;

	this->robot_pose_ = robot_pose;

	landmark.pose = landmark_pose;
	landmark.distance = sqrt((landmark_pose.x - robot_pose.position.x) * (landmark_pose.x - robot_pose.position.x) +
							 (landmark_pose.y - robot_pose.position.y) * (landmark_pose.y - robot_pose.position.y) +
							 (landmark_pose.z - robot_pose.position.z) * (landmark_pose.z - robot_pose.position.z));


	vx = landmark_pose.x - robot_pose.position.x;
	vy = landmark_pose.y - robot_pose.position.y;
	vz = landmark_pose.z - robot_pose.position.z;

	cat_adj = sqrt(vx * vx + vy * vy);

	landmark.theta = atan2(vy, vx);
	landmark.phi = atan2(vz, cat_adj);

	landmark.cropped_image = crop;

	glEnable(GL_TEXTURE_2D);

	// allocate a texture name
	glGenTextures(1, &landmark.texture);
	glBindTexture(GL_TEXTURE_2D, landmark.texture);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

//	// build our texture
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 200, 200, 0, GL_RGB,
	    GL_UNSIGNED_BYTE, landmark.cropped_image);

	glDisable(GL_TEXTURE_2D);

	this->landmarks[this->landmarks_counter_] = landmark;
	this->landmarks_counter_++;

	if(this->landmarks_counter_ >= this->landmarks_size_)
		this->landmarks_counter_ = 0;
}

void
Landmark::draw(bool drawLandmark, bool drawDistanceInformation)
{
	if(drawLandmark)
		this->drawLandmark();

	if(drawDistanceInformation)
		this->drawDistanceInformation();
}

Eigen::Matrix<float, 3, 3>
Landmark::getLandmarkRotationMatrixPhi(carmen_laslam_landmark_t landmark)
{
	Eigen::Matrix<float, 3, 3> rotation_phi;

	//rotation
	rotation_phi(0, 0) = 1.0;
	rotation_phi(0, 1) = 0.0;
	rotation_phi(0, 2) = 0.0;
	rotation_phi(1, 0) = 0.0;
	rotation_phi(1, 1) = cos(landmark.phi);
	rotation_phi(1, 2) = -sin(landmark.phi);
	rotation_phi(2, 0) = 0.0;
	rotation_phi(2, 1) = sin(landmark.phi);
	rotation_phi(2, 2) = cos(landmark.phi);

	return rotation_phi;
}

Eigen::Matrix<float, 3, 3>
Landmark::getLandmarkRotationMatrixTheta(carmen_laslam_landmark_t landmark)
{
	Eigen::Matrix<float, 3, 3> rotation_theta;

	//rotation
	rotation_theta(0, 0) = cos(landmark.theta);
	rotation_theta(0, 1) = -sin(landmark.theta);
	rotation_theta(0, 2) = 0.0;
	rotation_theta(1, 0) = sin(landmark.theta);
	rotation_theta(1, 1) = cos(landmark.theta);
	rotation_theta(1, 2) = 0.0;
	rotation_theta(2, 0) = 0.0;
	rotation_theta(2, 1) = 0.0;
	rotation_theta(2, 2) = 1.0;

	return rotation_theta;
}

void
Landmark::drawLandmark()
{
	Eigen::Matrix<float, 3, 3> rotation_phi;
	Eigen::Matrix<float, 3, 3> rotation_theta;

	Eigen::Matrix<float, 3, 1> landmark_vertice1;
	Eigen::Matrix<float, 3, 1> landmark_vertice2;
	Eigen::Matrix<float, 3, 1> landmark_vertice3;
	Eigen::Matrix<float, 3, 1> landmark_vertice4;


	glPushMatrix();

		glEnable(GL_TEXTURE_2D);
		glColor3f(1.0, 1.0, 1.0);

		for(int i = 0; i < this->landmarks_size_; i++)
		{
			if(this->landmarks[i].cropped_image != NULL)
			{
				// select our current texture
				glBindTexture(GL_TEXTURE_2D, this->landmarks[i].texture);


				rotation_phi = getLandmarkRotationMatrixPhi(this->landmarks[i]);
				rotation_theta = getLandmarkRotationMatrixTheta(this->landmarks[i]);

				landmark_vertice1(0, 0) = 0.0;  //this->landmarks[i].pose.x;
				landmark_vertice1(1, 0) = -0.50; //this->landmarks[i].pose.y - 0.2;
				landmark_vertice1(2, 0) = 0.50;  //this->landmarks[i].pose.z + 0.2;

				landmark_vertice2(0, 0) = 0.0;  //this->landmarks[i].pose.x;
				landmark_vertice2(1, 0) = 0.50;  //this->landmarks[i].pose.y + 0.2;
				landmark_vertice2(2, 0) = 0.50;  //this->landmarks[i].pose.z + 0.2;

				landmark_vertice3(0, 0) = 0.0;  //this->landmarks[i].pose.x;
				landmark_vertice3(1, 0) = 0.50;  //this->landmarks[i].pose.y + 0.2;
				landmark_vertice3(2, 0) = -0.50;  //this->landmarks[i].pose.z - 0.2;

				landmark_vertice4(0, 0) = 0.0;  //this->landmarks[i].pose.x;
				landmark_vertice4(1, 0) = -0.50; //this->landmarks[i].pose.y - 0.2;
				landmark_vertice4(2, 0) = -0.50; //this->landmarks[i].pose.z - 0.2;

				glBegin(GL_QUADS);
					//glTexCoord2d(0.0,0.0);
					glTexCoord2d(1.0,0.0);
					glVertex3f(this->landmarks[i].pose.x + landmark_vertice1(0, 0), this->landmarks[i].pose.y + landmark_vertice1(1, 0), this->landmarks[i].pose.z + landmark_vertice1(2, 0));
					//glTexCoord2d(1.0,0.0);
					glTexCoord2d(0.0,0.0);
					glVertex3f(this->landmarks[i].pose.x + landmark_vertice2(0, 0), this->landmarks[i].pose.y + landmark_vertice2(1, 0), this->landmarks[i].pose.z + landmark_vertice2(2, 0));
					//glTexCoord2d(1.1,1.0);
					glTexCoord2d(0.0,1.0);
					glVertex3f(this->landmarks[i].pose.x + landmark_vertice3(0, 0), this->landmarks[i].pose.y + landmark_vertice3(1, 0), this->landmarks[i].pose.z + landmark_vertice3(2, 0));
					//glTexCoord2d(0.0,1.0);
					glTexCoord2d(1.1,1.0);
					glVertex3f(this->landmarks[i].pose.x + landmark_vertice4(0, 0), this->landmarks[i].pose.y + landmark_vertice4(1, 0), this->landmarks[i].pose.z + landmark_vertice4(2, 0));
				glEnd();

			}
		}
		glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void
Landmark::drawDistanceInformation()
{
	glPushMatrix();

		glColor3f(1.0, 0.0, 0.0);
		glLineWidth(1.0);

		for(int i = 0; i < this->landmarks_size_; i++)
		{
			double distance = sqrt((this->landmarks[i].pose.x - this->robot_pose_.position.x) * (this->landmarks[i].pose.x - this->robot_pose_.position.x) +
					 	 	   (this->landmarks[i].pose.y - this->robot_pose_.position.y) * (this->landmarks[i].pose.y - this->robot_pose_.position.y) +
					 	 	   (this->landmarks[i].pose.z - this->robot_pose_.position.z) * (this->landmarks[i].pose.z - this->robot_pose_.position.z));
			if(distance < 10.0)
			{
				glBegin(GL_LINES);
					glVertex3f(this->robot_pose_.position.x, this->robot_pose_.position.y, this->robot_pose_.position.z);
					glVertex3f(this->landmarks[i].pose.x, this->landmarks[i].pose.y, this->landmarks[i].pose.z);
				glEnd();
			}
		}

	glPopMatrix();
}
