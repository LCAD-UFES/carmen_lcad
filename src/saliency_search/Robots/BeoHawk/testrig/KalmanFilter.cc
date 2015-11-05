/*
 * KalmanFilter.c
 *
 *  Created on: Feb 19, 2010
 *      Author: uscr
 */

#include "KalmanFilter.h"
#include <cstdio>

struct KalmanFilterData pitchFilter, rollFilter;

void initializeKalmanFilter(struct KalmanFilterData * filter, double sz, double sw0, double sw1) {

	filter->x_0 = 0.0;
	filter->x_1 = 0.0;
	filter->innov = 0.0;
	filter->cov = 0.0;
	filter->K_0 = 0.0;
	filter->K_1 = 0.0;
	filter->P_00 = 1.0;
	filter->P_01 = 0.0;
	filter->P_10 = 0.0;
	filter->P_11 = 1.0;

	filter->Sz = sz;
	filter->Sw_00 = sw0;
	filter->Sw_11 = sw1;
}


void updateKalmanFilter(struct KalmanFilterData * filter, double adj_gyro, double accel_angle, double delta_t) {

	//Update the state of our model
	//x(k+1) = A*x(k) + B*u
	filter->x_0 += delta_t*(adj_gyro - filter->x_1);

	//Get the "innovation" - AKA the diff between what we think the value is and what our second
	// data source says the value is
	filter->innov = accel_angle - filter->x_0;

	//Get the covariance of the measurement
	filter->cov = filter->P_00 + filter->Sz;

	//Get the kalman gain
	filter->K_0 = (filter->P_00 - delta_t*(filter->P_10))/(filter->cov);
	filter->K_1 = (filter->P_10)/(filter->cov);

	//Correct the prediction of the state
	filter->x_0 += (filter->K_0)*(filter->innov);
	filter->x_1 += (filter->K_1)*(filter->innov);

	//Get the covariance of the prediction error
	double new_P_00 = (filter->P_00)*(1 - filter->K_0) +
    		(filter->P_01)*delta_t*( filter->K_0 - 1) +
    		(filter->P_10)*(-delta_t) +
    		(filter->P_11)*delta_t*delta_t;
	double new_P_01 = (filter->P_01)*(1 - filter->K_0) -
			delta_t*(filter->P_11);
	double new_P_10 = filter->P_10 +
			delta_t*((filter->K_1)*(filter->P_01) - filter->P_11) -
			(filter->K_1)*(filter->P_00);
	double new_P_11 = filter->P_11 -
			(filter->K_1)*(filter->P_01);

	//add in the gyro variances:
	filter->P_00 = new_P_00 + filter->Sw_00;
	filter->P_01 = new_P_01;
	filter->P_10 = new_P_10;
	filter->P_11 = new_P_11 + filter->Sw_11;
}


//weights should be a double array of size numVals
// they do not need to sum to 1
double getCircularBufferWeightedMean(double * data, double * weights, int startVal, int numVals, int maxPoints) {
	double mean = 0;
	double weightSum = 0;

	for (int i = startVal, j = 0; j < numVals; i++, j++) {
		mean += weights[j]*data[i];
		weightSum += weights[j];
		if (i + 1 == maxPoints)
			i = -1;
	}

	return mean / weightSum;
}


