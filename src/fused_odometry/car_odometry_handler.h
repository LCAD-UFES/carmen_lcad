#ifndef CAR_ODOMETRY_HANDLER_H
#define CAR_ODOMETRY_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif



typedef struct
{
	xsens_xyz_handler *xsens_handler;
} car_odometry_handler;

car_odometry_handler *create_car_odometry_handler(xsens_xyz_handler *xsens_handler, carmen_fused_odometry_parameters *parameters);
void reset_car_odometry_handler(car_odometry_handler* car_handler);
void destroy_car_odometry_handler(car_odometry_handler* car_handler);


#ifdef __cplusplus
}
#endif

#endif
