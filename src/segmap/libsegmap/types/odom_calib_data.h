
#ifndef __ODOM_CALIB_DATA_H__
#define __ODOM_CALIB_DATA_H__

class OdomCalib
{
public:
	double add_v;
	double add_phi;
	double mult_v;
	double mult_phi;
	double init_angle;

	OdomCalib();
};

#endif
