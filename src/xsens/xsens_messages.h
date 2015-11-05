#ifndef CARMEN_XSENS_MESSAGES_H
#define CARMEN_XSENS_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	double x, y, z;
} carmen_xsens_axis;

typedef struct{
    double m_pitch, m_roll, m_yaw;
} carmen_xsens_euler;

typedef struct{
    double m_data[4];
} carmen_xsens_quat;

typedef struct{
    double m_data[3][3];
} carmen_xsens_matrix;


typedef struct {
    carmen_xsens_axis m_acc;
    carmen_xsens_axis m_gyr;
    carmen_xsens_axis m_mag;

    carmen_xsens_matrix matrix_data;
    
    double m_temp;
    unsigned short m_count;

    double timestamp;
    double timestamp2;
    char* host;

} carmen_xsens_global_matrix_message;

#define CARMEN_XSENS_GLOBAL_MATRIX_NAME "carmen_xsens_global_matrix"
#define CARMEN_XSENS_GLOBAL_MATRIX_FMT  "{{double, double, double}, {double, double, double}, {double, double, double}, { [double: 3, 3] }, double, ushort, double, double, string}"

typedef struct {
    carmen_xsens_axis m_acc;
    carmen_xsens_axis m_gyr;
    carmen_xsens_axis m_mag;

    carmen_xsens_euler euler_data;
    
    double m_temp;
    unsigned short m_count;

    double timestamp;
    char* host;

} carmen_xsens_global_euler_message;

#define CARMEN_XSENS_GLOBAL_EULER_NAME "carmen_xsens_global_euler"
#define CARMEN_XSENS_GLOBAL_EULER_FMT  "{{double, double, double}, {double, double, double}, {double, double, double}, { double, double, double }, double, ushort, double, string}"

typedef struct {
    carmen_xsens_axis m_acc;
    carmen_xsens_axis m_gyr;
    carmen_xsens_axis m_mag;

    carmen_xsens_quat quat_data;
    
    double m_temp;
    unsigned short m_count;

    double timestamp;
    char* host;

} carmen_xsens_global_quat_message;
  
#define CARMEN_XSENS_GLOBAL_QUAT_NAME "carmen_xsens_global_quat"
#define CARMEN_XSENS_GLOBAL_QUAT_FMT  "{{double, double, double}, {double, double, double}, {double, double, double}, { [double: 4] }, double, ushort, double, string}"

typedef struct {
  carmen_xsens_axis m_acc;
  carmen_xsens_axis m_gyr;
  double m_pitch, m_roll, m_yaw;
  double m_temp;
  unsigned short m_count;
  
} carmen_xsens_global_message;
  
#define CARMEN_XSENS_GLOBAL_NAME "carmen_xsens_global"
#define CARMEN_XSENS_GLOBAL_FMT  "{{double, double, double},{double, double, double}, double, double, double, double, ushort}"

#ifdef __cplusplus
}
#endif

#endif
