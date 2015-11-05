/*
 * Atomic6DOF.h
 *
 *  Created on: Feb 18, 2010
 *      Author: uscr
 */

#ifndef ATOMIC6DOF_H_
#define ATOMIC6DOF_H_

#define SERIAL_PORT		"/dev/ttyUSB0"
#define SERIAL_BAUD		B115200

//sensor calibration and offset constants
#define X_ACCEL_OFFSET		1.952
#define Y_ACCEL_OFFSET		2.21
#define Z_ACCEL_OFFSET		2.0992
#define X_ACCEL_COEFF		0.003931641
#define Y_ACCEL_COEFF		0.004189453
#define Z_ACCEL_COEFF		0.004125

#define GYRO_COEFF		0.9765625
#define GYRO_OFFSET		500

#define RAD_TO_DEG		57.295779506


struct currentIMUData {
	double gyro_pitch, gyro_roll, gyro_yaw;
	double accel_x, accel_y, accel_z;
	double adj_gyro_pitch, adj_gyro_roll, adj_gyro_yaw;
	double adj_accel_x, adj_accel_y, adj_accel_z;
	double adj_accel_pitch, adj_accel_roll;
	int count;
};

bool initializeAtomic6DOF();
void shutdownAtomic6DOF();

bool isIMUDataReady();  //returns true if the read resulted in

#endif /* ATOMIC6DOF_H_ */
