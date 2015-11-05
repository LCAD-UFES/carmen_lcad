/*
 * KalmanFilter.h
 *
 *  Created on: Feb 19, 2010
 *      Author: uscr
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

/*
struct KalmanFilterData {

	double X, Boff;

	double gyroVar, accelVar, gyroAlpha, accelAlpha, B, P;

	double gyroLP, accelLP;
};*/

struct KalmanFilterData {
	double x_0, x_1;
	double innov, cov;
	double K_0, K_1;
	double P_00, P_01, P_10, P_11;

	double Sz, Sw_00, Sw_11;

};


void initializeKalmanFilter(struct KalmanFilterData * filter, double sz, double sw0, double sw1);
void updateKalmanFilter(struct KalmanFilterData * filter, double adj_gyro, double adj_accel, double delta_t);
double getCircularBufferWeightedMean(double * data, double * weights, int startVal, int numVals, int maxPoints);


#endif /* KALMANFILTER_H_ */
