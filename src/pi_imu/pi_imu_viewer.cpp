/*
 * pi_imu_interface.cpp
 *
 *  Created on: Aug 1, 2018
 *      Author: Marcelo
 */
#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>
#include <carmen/xsenscore.h>
#include <GL/glut.h>
#include <math.h>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define G 9.80665
#define RAD_TO_DEG (180.0 / M_PI)


carmen_xsens_global_quat_message *xsens_quat_message;
xsens_global data;
GLfloat angle, fAspect;
float AccYangle = 0.0;
float AccXangle = 0.0;
float AccZangle = 0.0;

float MagYangle = 0.0;
float MagXangle = 0.0;
float MagZangle = 0.0;

bool enableXrotation = false;
bool enableYrotation = false;
bool enableZrotation = false;
unsigned int cubemapTexture;

using namespace Eigen;
Quaternionf  quat;


//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	quat.x() = q0;
	quat.y() = q1;
	quat.z() = q2;
	quat.w() = q3;
}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	quat.x() = q0;
	quat.y() = q1;
	quat.z() = q2;
	quat.w() = q3;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update



//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


void loadCubemap( GLenum side_target, std::string faces)
{
	glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &cubemapTexture);
	cv::Mat image = cv::imread(faces.c_str(), CV_LOAD_IMAGE_COLOR);
	unsigned char *data = image.data;
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	if (data)
	{
		glTexImage2D(side_target,
					 0, GL_RGB, (GLsizei) image.cols, (GLsizei) image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, data
		);
		//image.release();
	}
	else
	{
		std::cout << "Cubemap texture failed to load at path: " << faces << std::endl;
		//image.release();
	}
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();

        printf("Imu Interface: disconnected.\n");
        exit(0);
    }
}


void
xsens_message_handler(carmen_xsens_global_quat_message *xsens_quat_message)
{
	data.acc.x = xsens_quat_message->m_acc.x;//data.acc.x + 1.0 * (xsens_quat_message->m_acc.x - data.acc.x);
	data.acc.y = xsens_quat_message->m_acc.y;//data.acc.y + 1.0 * (xsens_quat_message->m_acc.y - data.acc.y);
	data.acc.z = xsens_quat_message->m_acc.z;//data.acc.z + 1.0 * (xsens_quat_message->m_acc.z - data.acc.z) * 2.;

	data.gyr.x = xsens_quat_message->m_gyr.x;
	data.gyr.y = xsens_quat_message->m_gyr.y;
	data.gyr.z = xsens_quat_message->m_gyr.z;

	data.mag.x = xsens_quat_message->m_mag.x;
	data.mag.y = xsens_quat_message->m_mag.y;
	data.mag.z = xsens_quat_message->m_mag.z;
}


void
carmen_xsens_subscribe_xsens_global_quat_message(carmen_xsens_global_quat_message
					    *xsens_global,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
    carmen_subscribe_message((char *) CARMEN_XSENS_GLOBAL_QUAT_NAME,
    						 (char *) CARMEN_XSENS_GLOBAL_QUAT_FMT,
                             xsens_global, sizeof(carmen_xsens_global_quat_message),
            			     handler, subscribe_how);
}


void
carmen_xsens_unsubscribe_xsens_global_quat_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message((char *) CARMEN_XSENS_GLOBAL_QUAT_NAME, handler);
}

static void cubebase(void)
/*specifies a side of a cube*/
{
	glBegin(GL_POLYGON);
		glTexCoord2f(0.0,0.0);
		glVertex3d(-0.5,-0.5,0.0);

		glTexCoord2f(0.0,1.0);
		glVertex3d(-0.5,0.5,0.0);

		glTexCoord2f(1.0,1.0);
		glVertex3d(0.5,0.5,0.0);

		glTexCoord2f(1.0,0.0);
		glVertex3d(0.5,-0.5,0.0);
	glEnd();
}

void
display (void)
{
	MadgwickAHRSupdate(data.gyr.x, data.gyr.y, data.gyr.z, data.acc.x, data.acc.y, data.acc.z, data.mag.x, data.mag.y, data.mag.z);
	Vector3f euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
	std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler[0] * RAD_TO_DEG<< " " << euler[1] * RAD_TO_DEG << " " << euler[2] * RAD_TO_DEG<< std::endl;
	//printf("ACC\t%.2f\t%.2f\t%.2f\t", data.acc.x, data.acc.y, data.acc.z);

	//AccXangle = -atan2(data.acc.y / acc_magnitude_yz, data.acc.z / acc_magnitude_yz) * RAD_TO_DEG;
	//AccYangle = atan2(data.acc.z / acc_magnitude_xz, data.acc.x / acc_magnitude_xz) * RAD_TO_DEG;
	//AccZangle = 0.0;

	/*AccYangle = -atan2(sqrt(pow(data.acc.x, 2) + pow(data.acc.y, 2)), data.acc.z) * RAD_TO_DEG;
	AccXangle = atan2(data.acc.x, data.acc.y) * RAD_TO_DEG;
	AccZangle = 0.0;

	if (enableXrotation == true)
	{
		AccXangle = -atan2(data.acc.y / acc_magnitude_yz, data.acc.z / acc_magnitude_yz) * RAD_TO_DEG;
		AccYangle = 0.0; //atan2(data.acc.z / acc_magnitude_xz, data.acc.x / acc_magnitude_xz) * RAD_TO_DEG;
		AccZangle = 0.0; //atan2(data.acc.y / acc_magnitude_xy, data.acc.x / acc_magnitude_xy) * RAD_TO_DEG;
	}
	else if (enableYrotation == true)
	{
		AccXangle = 0.0;
		AccYangle = atan2(data.acc.z / acc_magnitude_xz, data.acc.x / acc_magnitude_xz) * RAD_TO_DEG; //atan2(data.acc.z / acc_magnitude_xz, data.acc.x / acc_magnitude_xz) * RAD_TO_DEG;
		AccZangle = 0.0; //atan2(data.acc.y / acc_magnitude_xy, data.acc.x / acc_magnitude_xy) * RAD_TO_DEG;
	}
	else if (enableZrotation == true)
	{
		AccXangle = 0.0;
		AccYangle = 0.0; //atan2(data.acc.z / acc_magnitude_xz, data.acc.x / acc_magnitude_xz) * RAD_TO_DEG;
		AccZangle = atan2(data.acc.x / acc_magnitude_xy , data.acc.y / acc_magnitude_xy) * RAD_TO_DEG;//atan2(data.acc.y / acc_magnitude_xy, data.acc.x / acc_magnitude_xy) * RAD_TO_DEG;
	}

	printf("ANGLES\t%.2f\t%.2f\t%.2f\n", AccXangle, AccYangle, AccZangle);
*/
	/*for (int i = 0; i < 3 ; i++)
	{
		printf ("%f  %f  %f\n", matrix(i,0) * RAD_TO_DEG, matrix(i,1) * RAD_TO_DEG, matrix(i,2)* RAD_TO_DEG);
	}
	printf ("\n\n");*/
	/*make sure we're dealing with modelview matrix*/
	glMatrixMode(GL_MODELVIEW);

	/*pushes and duplicates current matrix*/
	glPushMatrix();


	/*construct the base*/
	cubebase();

	glPushMatrix();
	/*construct side on +x axis*/
	glTranslated(0.5,0.0,0.5);
	glRotated(90.0,0.0,1.0,0.0);
	cubebase();

	glPopMatrix();

	/*construct side on -x axis*/
	glPushMatrix();
	glTranslated(-0.5,0.0,0.5);
	glRotated(-90.0,0.0,1.0,0.0);
	cubebase();
	glPopMatrix();

	/*construct side on +y axis*/
	glPushMatrix();
	glTranslated(0.0,0.5,0.5);
	glRotated(-90.0,1.0,0.0,0.0);
	cubebase();
	glPopMatrix();

	/*construct side on -y axis*/
	glPushMatrix();
	glTranslated(0.0,-0.5,0.5);
	glRotated(90.0,1.0,0.0,0.0);
	cubebase();
	glPopMatrix();

	/*construct top*/

	glBegin(GL_POLYGON);
		glTexCoord2f(0.0,0.0);
		glVertex3d(-0.5,-0.5,1.0);

		glTexCoord2f(1.0,0.0);
		glVertex3d(0.5,-0.5,1.0);

		glTexCoord2f(1.0,1.0);
		glVertex3d(0.5,0.5,1.0);

		glTexCoord2f(0.0,1.0);
		glVertex3d(-0.5,0.5,1.0);
	glEnd();


	glPopMatrix();

	glFlush();
	// Executa os comandos OpenGL
	glutSwapBuffers();
}


void
sleep_ipc()
{
	carmen_ipc_sleep(0.033333333);
	glutPostRedisplay();
}


// Inicializa parâmetros de rendering
void
Inicializa (void)
{
	GLfloat luzAmbiente[4]={0.2,0.2,0.2,1.0};
	GLfloat luzDifusa[4]={0.7,0.7,0.7,1.0};	   // "cor"
	GLfloat luzEspecular[4]={1.0, 1.0, 1.0, 1.0};// "brilho"
	GLfloat posicaoLuz[4]={0.0, 50.0, 50.0, 1.0};

	// Capacidade de brilho do material
	GLfloat especularidade[4]={1.0,1.0,1.0,1.0};
	GLint especMaterial = 80;

	// Especifica que a cor de fundo da janela será preta
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// Habilita o modelo de colorização de Gouraud
	glShadeModel(GL_SMOOTH);

	// Define a refletância do material
	glMaterialfv(GL_FRONT,GL_SPECULAR, especularidade);
	// Define a concentração do brilho
	glMateriali(GL_FRONT,GL_SHININESS,especMaterial);

	// Ativa o uso da luz ambiente
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, luzAmbiente);

	// Define os parâmetros da luz de número 0
	glLightfv(GL_LIGHT0, GL_AMBIENT, luzAmbiente);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, luzDifusa );
	glLightfv(GL_LIGHT0, GL_SPECULAR, luzEspecular );
	glLightfv(GL_LIGHT0, GL_POSITION, posicaoLuz );

	// Habilita a definição da cor do material a partir da cor corrente
	glEnable(GL_COLOR_MATERIAL);
	//Habilita o uso de iluminação
	glEnable(GL_LIGHTING);
	// Habilita a luz de número 0
	glEnable(GL_LIGHT0);
	// Habilita o depth-bufferingt
	glEnable(GL_DEPTH_TEST);

	angle=45;
}


// Função usada para especificar o volume de visualização
void
EspecificaParametrosVisualizacao(void)
{
	// Especifica sistema de coordenadas de projeção
	glMatrixMode(GL_PROJECTION);
	// Inicializa sistema de coordenadas de projeção
	glLoadIdentity();
	// Especifica a projeção perspectiva
	gluPerspective(angle,fAspect,0.1,500);
	// Especifica sistema de coordenadas do modelo
	glMatrixMode(GL_MODELVIEW);
	// Inicializa sistema de coordenadas do modelo
	glLoadIdentity();
	// Especifica posição do observador e do alvo
	gluLookAt(0,80,200, 0,0,0, 0,1,0);
}


// Função callback chamada quando o tamanho da janela é alterado
void
AlteraTamanhoJanela(GLsizei w, GLsizei h)
{
	// Para previnir uy = 0;ma divisão por zero
	if ( h == 0 ) h = 1;
	// Especifica o tamanho da viewport
	glViewport(0, 0, w, h);
	// Calcula a correção de aspecto
	fAspect = (GLfloat)w / (GLfloat)h;
	EspecificaParametrosVisualizacao();
}


void
keyboard (unsigned char key, int x __attribute__((unused)), int y __attribute__((unused)))
{
	 switch (key)
	 {
		case 'x':
			enableXrotation = true;
			enableYrotation = false;
			enableZrotation = false;
			break;

		case 'y':
			enableXrotation = false;
			enableYrotation = true;
			enableZrotation = false;
			break;

		case 'z':
			enableXrotation = false;
			enableYrotation = false;
			enableZrotation = true;
			break;

	}

}


// Função callback chamada para gerenciar eventos do mouse
void
GerenciaMouse(int button, int state, int x, int y)
{
	x += 0;
	y += 0;
	if (button == GLUT_LEFT_BUTTON)
		if (state == GLUT_DOWN)
		{  // Zoom-in
			if (angle >= 10) angle -= 5;
		}
	if (button == GLUT_RIGHT_BUTTON)
		if (state == GLUT_DOWN)
		{  // Zoom-out	double acc_magnitude_xz = sqrt(data.acc.x * data.acc.x +  data.acc.z * data.acc.z);

			if (angle <= 130) angle += 5;
		}
	EspecificaParametrosVisualizacao();
	glutPostRedisplay();
}

int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	carmen_xsens_subscribe_xsens_global_quat_message(xsens_quat_message,
			(carmen_handler_t) xsens_message_handler, CARMEN_SUBSCRIBE_LATEST);
	quat.x() = 0.0;
	quat.y() = 0.0;
	quat.z() = 0.0;
	quat.w() = 1.0;

	std::vector<std::string> faces;
	faces.push_back("/home/lcad/carmen_lcad/src/pi_imu/images/Right.png");
	faces.push_back("/home/lcad/carmen_lcad/src/pi_imu/images/Left.png");
	faces.push_back("/home/lcad/carmen_lcad/src/pi_imu/images/Top.png");
	faces.push_back("/home/lcad/carmen_lcad/src/pi_imu/images/Bottom.png");
	faces.push_back("/home/lcad/carmen_lcad/src/pi_imu/images/Front.png");
	faces.push_back("/home/lcad/carmen_lcad/src/pi_imu/images/Back.png");

	loadCubemap(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, "/home/lcad/carmen_lcad/src/pi_imu/images/Front.png");
	loadCubemap( GL_TEXTURE_CUBE_MAP_POSITIVE_Z, "/home/lcad/carmen_lcad/src/pi_imu/images/Back.png");
	loadCubemap( GL_TEXTURE_CUBE_MAP_POSITIVE_Y, "/home/lcad/carmen_lcad/src/pi_imu/images/Top.png");
	loadCubemap(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, "/home/lcad/carmen_lcad/src/pi_imu/images/Bottom.png");
	loadCubemap(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, "/home/lcad/carmen_lcad/src/pi_imu/images/Left.png");
	loadCubemap(GL_TEXTURE_CUBE_MAP_POSITIVE_X, "/home/lcad/carmen_lcad/src/pi_imu/images/Right.png");

	glutInit(&argc, argv); // Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(800, 800);
	glutCreateWindow("Visualizacao 3D");
	glutDisplayFunc(display);
	glutReshapeFunc(AlteraTamanhoJanela);
	glutMouseFunc(GerenciaMouse);
	glutKeyboardFunc(keyboard);
	Inicializa();
	glutIdleFunc(sleep_ipc);
	glutMainLoop();

	return (0);
}
