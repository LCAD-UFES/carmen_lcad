#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/localize_interface.h>
#include "xsensOdometer_interface.h"

#define GRAVITY 9.80665

static carmen_xsens_global_message xsensGlobal;
static carmen_xsens_odometry_message xsensOdometry;
static void xsens_global_handler(void);

static carmen_base_odometry_message fakeOdometry;

static carmen_localize_initialize_message initialize;
static void initialize_handler(void);

static carmen_localize_globalpos_message globalPos_message;
static void globalPos_handler(void);

// This matrix will be used to convert from the xsens frame of reference
// to the map frame of reference
static double mapMatrix[3][3];

typedef struct vec3d
{
	double x;
	double y;
	double z;
} vec3d;

static vec3d acc;
static vec3d vel;
static vec3d pos;

static double totalAcc;
static double totalVel;

// This is the angle the robot is currently facing 
// The offset is the difference between the xsens global frame of reference and the map frame of reference
static double theta;
static double thetaOffset;

// This thresholds will be used to determine if the robot is not moving
// and the sensor readings are just noise
static double accelerationThreshold;
static double velocityThreshold;
static double constantFriction;
static double velocityFriction;

static int sendFakeWheelOdometry;

int register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_XSENS_ODOMETRY_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_ODOMETRY_NAME);
  
	return 0;  
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
		{"xsensOdometer", "fake_wheel", CARMEN_PARAM_ONOFF, &sendFakeWheelOdometry, 0, NULL},
		{"xsensOdometer", "acceleration_threshold", CARMEN_PARAM_DOUBLE, &accelerationThreshold, 0, NULL},
		{"xsensOdometer", "velocity_threshold", CARMEN_PARAM_DOUBLE, &velocityThreshold, 0, NULL},
		{"xsensOdometer", "constant_friction", CARMEN_PARAM_DOUBLE, &constantFriction, 0, NULL},
		{"xsensOdometer", "velocity_friction", CARMEN_PARAM_DOUBLE, &velocityFriction, 0, NULL}
		};
	
	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
		
	return 0;
}

void initializeOdometry()
{	
	acc.x = 0;
	acc.y = 0;	
	acc.z = 0;

	vel.x = 0;
	vel.y = 0;
	vel.z = 0;

	pos.x = 0;
	pos.y = 0;
	pos.z = 0;

	theta = 0;
	totalVel = 0;
	totalAcc = 0;

	mapMatrix[0][0] = 1.0;
	mapMatrix[0][1] = 0.0;
	mapMatrix[0][2] = 0.0;
	mapMatrix[1][0] = 0.0;
	mapMatrix[1][1] = 1.0;
	mapMatrix[1][2] = 0.0;
	mapMatrix[2][0] = 0.0;
	mapMatrix[2][1] = 0.0;
	mapMatrix[2][2] = 1.0;

}

static void initialize_handler(void)
{
	pos.x = initialize.mean->x;
	pos.y = initialize.mean->y;
	pos.z = 0;

	vel.x = 0;
	vel.y = 0;
	vel.z = 0;

	acc.x = 0;
	acc.y = 0;	
	acc.z = 0;

	totalVel = 0;
	totalAcc = 0;
	
	thetaOffset = initialize.mean->theta - theta;
	
	
	mapMatrix[0][0] = cos(thetaOffset);
	mapMatrix[0][1] = -sin(thetaOffset);
	mapMatrix[0][2] = 0;
	mapMatrix[1][0] = sin(thetaOffset);
	mapMatrix[1][1] = cos(thetaOffset);
	mapMatrix[1][2] = 0;
	mapMatrix[2][0] = 0;
	mapMatrix[2][1] = 0;
	mapMatrix[2][2] = 1;
	
	/*
	printf("\n");
	printf("Init Theta:% lf\n", initialize.mean->theta);
	printf("Theta:% lf\n", theta);
	printf("Offset:% lf\n", thetaOffset);
	printf("% lf\t% lf\t% lf\n", mapMatrix[0][0], mapMatrix[0][1], mapMatrix[0][2]);
	printf("% lf\t% lf\t% lf\n", mapMatrix[1][0], mapMatrix[1][1], mapMatrix[1][2]);
	printf("% lf\t% lf\t% lf\n", mapMatrix[2][0], mapMatrix[2][1], mapMatrix[2][2]);
	printf("\n");
	*/
}

void globalPos_handler(void)
{
	pos.x = globalPos_message.globalpos.x;
	pos.y = globalPos_message.globalpos.y;
		
	thetaOffset = globalPos_message.globalpos.theta - theta;
		
	mapMatrix[0][0] = cos(thetaOffset);
	mapMatrix[0][1] = -sin(thetaOffset);
	mapMatrix[0][2] = 0;
	mapMatrix[1][0] = sin(thetaOffset);
	mapMatrix[1][1] = cos(thetaOffset);
	mapMatrix[1][2] = 0;
	mapMatrix[2][0] = 0;
	mapMatrix[2][1] = 0;
	mapMatrix[2][2] = 1;	
}

int main(int argc, char** argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	register_ipc_messages();

	read_parameters(argc, argv);
	initializeOdometry();
		
	carmen_localize_subscribe_initialize_message( 	&initialize,
							(carmen_handler_t)initialize_handler,
							CARMEN_SUBSCRIBE_LATEST);
	//carmen_localize_subscribe_globalpos_message(	&globalPos_message,
	//						(carmen_handler_t)globalPos_handler,
	//						CARMEN_SUBSCRIBE_LATEST);

	carmen_xsens_subscribe_xsens_global_message(	&xsensGlobal,
					    		(carmen_handler_t)xsens_global_handler,
					    	 	CARMEN_SUBSCRIBE_LATEST);
		
	carmen_ipc_dispatch();
	
	return 0;
}

static void computeNewPosition(double deltaT, double accX, double accY, double accZ, double pitch, double roll, double yaw)
{	
	// This matrix will transfer from the sensor frame of reference to the global frame of reference
	// (the global frame of reference is the frame of reference oriented with earth's magnetic field)
	double sensorToGlobal[3][3];
	
	double radianPitch = carmen_degrees_to_radians(pitch);
	double radianRoll = carmen_degrees_to_radians(roll);
	double radianYaw = carmen_degrees_to_radians(yaw);
	
	double sinPitch = sin(radianPitch);
	double cosPitch = cos(radianPitch);
	double sinRoll = sin(radianRoll);
	double cosRoll = cos(radianRoll);
	double sinYaw = sin(radianYaw);
	double cosYaw = cos(radianYaw);

	// This matrix can be obtained directly from the sensor, this should be changed to do that.
	sensorToGlobal[0][0] = cosPitch*cosYaw;
	sensorToGlobal[0][1] = sinRoll*sinPitch*cosYaw-cosRoll*sinYaw;
	sensorToGlobal[0][2] = cosRoll*sinPitch*cosYaw+sinRoll*sinYaw;
	sensorToGlobal[1][0] = cosPitch*sinYaw;
	sensorToGlobal[1][1] = sinRoll*sinPitch*sinYaw+cosRoll*cosYaw;
	sensorToGlobal[1][2] = cosRoll*sinPitch*sinYaw-sinRoll*cosYaw;
	sensorToGlobal[2][0] = -sinPitch;
	sensorToGlobal[2][1] = sinRoll*cosPitch;
	sensorToGlobal[2][2] = cosRoll*cosPitch;	
		
	// Find acceleration on the global frame of reference	
	double accGlobalX = sensorToGlobal[0][0]*accX + sensorToGlobal[0][1]*accY + sensorToGlobal[0][2]*accZ;
	double accGlobalY = sensorToGlobal[1][0]*accX + sensorToGlobal[1][1]*accY + sensorToGlobal[1][2]*accZ;
	double accGlobalZ = sensorToGlobal[2][0]*accX + sensorToGlobal[2][1]*accY + sensorToGlobal[2][2]*accZ;
	
	
	//printf("% lf\t% lf\t% lf\n", sensorToGlobal[0][0], sensorToGlobal[0][1], sensorToGlobal[0][2]);
	//printf("% lf\t% lf\t% lf\n", sensorToGlobal[1][0], sensorToGlobal[1][1], sensorToGlobal[1][2]);
	//printf("% lf\t% lf\t% lf\n", sensorToGlobal[2][0], sensorToGlobal[2][1], sensorToGlobal[2][2]);
	//printf("\n");
	
	//printf("x:% lf y:% lf z:% lf\n", accGlobalX, accGlobalY, accGlobalZ);

	// This will remove gravity from the acceleration so that it can be used for the inertial navigation
	accGlobalZ = accGlobalZ - GRAVITY;
	
	// This will transform from the sensor reference to the map reference
	double mapAccX = mapMatrix[0][0]*accGlobalX + mapMatrix[0][1]*accGlobalY + mapMatrix[0][2]*accGlobalZ;
	double mapAccY = mapMatrix[1][0]*accGlobalX + mapMatrix[1][1]*accGlobalY + mapMatrix[1][2]*accGlobalZ;
	double mapAccZ = mapMatrix[2][0]*accGlobalX + mapMatrix[2][1]*accGlobalY + mapMatrix[2][2]*accGlobalZ;
	
	vec3d lastVel;
	lastVel.x = vel.x;
	lastVel.y = vel.y;
	lastVel.z = vel.z;

	vec3d constFriction;	
	if(totalVel > 0)
	{
		if(constantFriction*deltaT > totalVel)
		{
			constFriction.x = lastVel.x*deltaT/totalVel;
			constFriction.y = lastVel.y*deltaT/totalVel;
			constFriction.z = lastVel.z*deltaT/totalVel;
		}
		else
		{
			constFriction.x = constantFriction*lastVel.x*deltaT/totalVel;
			constFriction.y = constantFriction*lastVel.y*deltaT/totalVel;
			constFriction.z = constantFriction*lastVel.z*deltaT/totalVel;
		}
	}
	else
	{
		constFriction.x = 0;
		constFriction.y = 0;
		constFriction.z = 0;
	}

	vel.x += (acc.x+mapAccX)*deltaT/2 - lastVel.x*velocityFriction*deltaT - constFriction.x;
	vel.y += (acc.y+mapAccY)*deltaT/2 - lastVel.y*velocityFriction*deltaT - constFriction.y;
	vel.z += (acc.z+mapAccZ)*deltaT/2 - lastVel.z*velocityFriction*deltaT - constFriction.z;
	
	pos.x += (lastVel.x+vel.x)*deltaT/2;
	pos.y += (lastVel.y+vel.y)*deltaT/2;
	pos.z += (lastVel.z+vel.z)*deltaT/2;

	acc.x = mapAccX;
	acc.y = mapAccY;
	acc.z = mapAccZ;
	
	theta = radianYaw;
	totalVel = sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
	totalAcc = sqrt(acc.x*acc.x+ acc.y*acc.y + acc.z*acc.z);
	
	printf("a =% lf, v =% lf\n",totalAcc, totalVel);
	if(totalVel < velocityThreshold && totalAcc < accelerationThreshold)
	{
		acc.x = 0;	
		acc.y = 0;
		acc.z = 0;
	
		vel.x = 0;
		vel.y = 0;
		vel.z = 0;

		totalVel = 0;
		totalAcc = 0;
	}

}

static void updateOdometryMessage(double currentTime)
{
	xsensOdometry.rv = xsensGlobal.m_gyr.z;
	xsensOdometry.tv = totalVel;

	xsensOdometry.x = pos.x;
	xsensOdometry.y = pos.y;
	xsensOdometry.theta = theta + thetaOffset;

	xsensOdometry.acceleration = totalAcc;

	xsensOdometry.timestamp = currentTime;

}

static void updateFakeOdometryMessage()
{
	fakeOdometry.rv = xsensOdometry.rv;
	fakeOdometry.tv = xsensOdometry.tv;

	fakeOdometry.x = xsensOdometry.x;
	fakeOdometry.y = xsensOdometry.y;
	fakeOdometry.theta = xsensOdometry.theta;

	fakeOdometry.acceleration = xsensOdometry.acceleration;

	fakeOdometry.timestamp = xsensOdometry.timestamp;
}

static void printMessage(void)
{

	printf("Accel: x: % lf y: % lf z:% lf\n", xsensGlobal.m_acc.x, xsensGlobal.m_acc.y, xsensGlobal.m_acc.z);	
	printf("Gyr: x: % lf y: % lf z:% lf\n", xsensGlobal.m_gyr.x, xsensGlobal.m_gyr.y, xsensGlobal.m_gyr.z);	
	printf("Pitch: % lf Roll: % lf Yaw:% lf\n\n", xsensGlobal.m_pitch, xsensGlobal.m_roll, xsensGlobal.m_yaw);
			
	printf("Fixed Accel: x: % lf y: % lf z: % lf\n", acc.x, acc.y, acc.z);
	printf("Vel: x: % lf y: % lf z:% lf\n", vel.x, vel.y, vel.z);
	printf("Theta offset: % lf\n\n", thetaOffset);
	
	printf("Xsens Odometry:\n");
	printf("x: % lf y: % lf z: % lf\n", xsensOdometry.x, xsensOdometry.y, pos.z);	
	printf("tv: % lf rv: % lf\n", xsensOdometry.tv, xsensOdometry.rv);
	printf("theta: % lf\n", xsensOdometry.theta);
	printf("Acceleration: % lf\n", xsensOdometry.acceleration);
	printf("timestamp: % lf\n\n", xsensOdometry.timestamp);	

}

static void xsens_global_handler(void) 
{
	static double lastTime = 0;
	
	if(lastTime == 0)
	{
		lastTime = carmen_get_time();
	}
	
	double currentTime = carmen_get_time();

	double dT = currentTime - lastTime;
	
	computeNewPosition(dT, xsensGlobal.m_acc.x, xsensGlobal.m_acc.y, xsensGlobal.m_acc.z, xsensGlobal.m_pitch, xsensGlobal.m_roll, xsensGlobal.m_yaw);
	updateOdometryMessage(currentTime);
	
	lastTime = currentTime;

	printMessage();
		
	int err = IPC_publishData(CARMEN_XSENS_ODOMETRY_NAME, &xsensOdometry);	
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_ODOMETRY_NAME);

	if(sendFakeWheelOdometry)
	{
		updateFakeOdometryMessage();

		int err = IPC_publishData(CARMEN_BASE_ODOMETRY_NAME, &fakeOdometry);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BASE_ODOMETRY_NAME);
	}
		
}
