#ifndef CARMEN_CPP_IMU_MESSAGE_H
#define CARMEN_CPP_IMU_MESSAGE_H

#include <carmen/cpp_global.h>
#include <carmen/cpp_abstractmessage.h>
#include <carmen/cpp_point.h>


class IMUMessage : public AbstractMessage {
 public:
  IMUMessage();
  IMUMessage(double accX, double accY, double accZ, 
	     double quat_0, double quat_1, double quat_2, double quat_3,
	     double magX, double magY, double magZ,
	     double gyroX, double gyroY, double gyroZ);
  IMUMessage(const IMUMessage& x);
  IMUMessage(const carmen_imu_message& x);
  IMUMessage(carmen_imu_message* x);
  IMUMessage(char* s);
  virtual ~IMUMessage();

  void init();
  void clone(const IMUMessage& x);
  void clone(const carmen_imu_message& x);
  void setIMUMessage(carmen_imu_message* x);
  void free();

  void getRollPitchYaw(double& roll, double& pitch, double& yaw); 

  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, AccX, accX);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, AccY, accY);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, AccZ, accZ);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Q0, q0);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Q1, q1);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Q2, q2);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Q3, q3);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, MagX, magX);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, MagY, magY);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, MagZ, magZ);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, GyroX, gyroX);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, GyroY, gyroY);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, GyroZ, gyroZ);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, char*, Host, host);
  STRUCT_PARAM_VIRTUAL_SET_GET(*m_msg, double, Timestamp, timestamp, public, public);

  carmen_inline carmen_imu_message* toCarmenMsg() {return m_msg;}
  carmen_inline operator carmen_imu_message*() {return m_msg;}
  carmen_inline operator const carmen_imu_message&() const {return *m_msg;}
  carmen_inline operator carmen_imu_message&() {return *m_msg;}

  virtual const char* getMessageID() const {
    return "IMU";
  };

  virtual void save(carmen_FILE *logfile, double logger_timestamp);
  virtual char*  fromString(char* s);
  virtual AbstractMessage* clone() const;

 protected:
  carmen_imu_message * m_msg;
};

#endif 
