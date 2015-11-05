
#ifndef CARMEN_XSENSODOMETER_MESSAGES_H
#define CARMEN_XSENSODOMETER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double x, y, theta;
  double tv, rv;
  double acceleration;
  double timestamp;
  char *host;
} carmen_xsens_odometry_message;
  
#define      CARMEN_XSENS_ODOMETRY_NAME       "carmen_xsens_odometry_message"
#define      CARMEN_XSENS_ODOMETRY_FMT        "{double,double,double,double,double,double,double,string}"

#ifdef __cplusplus
}
#endif

#endif
// @}
