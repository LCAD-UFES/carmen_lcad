#include "cpp_imu.h"
#include <carmen/carmen.h>

IMUMessage::IMUMessage()
  : AbstractMessage() {
  m_msg = NULL;
  init();
}

IMUMessage::IMUMessage(double accX, double accY, double accZ, 
		       double quat_0, double quat_1, double quat_2, double quat_3,
		       double magX, double magY, double magZ,
		       double gyroX, double gyroY, double gyroZ)
: AbstractMessage() {
  m_msg = NULL;
  init();

  m_msg->accX = accX;
  m_msg->accY = accY;
  m_msg->accZ = accZ;

  m_msg->q0 = quat_0;
  m_msg->q1 = quat_1;
  m_msg->q2 = quat_2;
  m_msg->q3 = quat_3;

  m_msg->magX = magX;
  m_msg->magY = magY;
  m_msg->magZ = magZ;

  m_msg->gyroX = gyroX;
  m_msg->gyroY = gyroY;
  m_msg->gyroZ = gyroZ;


}

IMUMessage::IMUMessage(const IMUMessage& x) 
  : AbstractMessage(x) {
  m_msg = NULL;
  clone(x);
}

IMUMessage::IMUMessage(const carmen_imu_message& x) {
  m_msg = NULL;
  clone(x);
}

IMUMessage::IMUMessage(carmen_imu_message* x) {
  m_msg = NULL;
  setIMUMessage(x);
}

IMUMessage::~IMUMessage() {
  this->free();
}


void IMUMessage::save(carmen_FILE *logfile, double logger_timestamp) {
  carmen_logwrite_write_imu(m_msg, logfile, logger_timestamp);
}

IMUMessage::IMUMessage(char* s) {
  m_msg = NULL;
  fromString(s);
}

char* IMUMessage::fromString(char* s) {
  if (m_msg == NULL) {
    init();
  }
  return carmen_string_to_imu_message(s, m_msg);
}

AbstractMessage* IMUMessage::clone() const {
  return new IMUMessage(*this);
}


void IMUMessage::init() {

  if (m_msg != NULL) {
    this->free();
  }
  m_msg = new carmen_imu_message;
  carmen_test_alloc(m_msg);
  carmen_erase_structure(m_msg, sizeof(carmen_imu_message));
}

void IMUMessage::free() {
  if (m_msg != NULL) {
    delete m_msg;
    m_msg = NULL;
  }
}

void IMUMessage::setIMUMessage( carmen_imu_message* x) {
  m_msg = x;
}


void IMUMessage::clone(const IMUMessage& x) {
  clone(*(x.m_msg));
}

void IMUMessage::clone(const carmen_imu_message& x) {
  m_msg->accX = x.accX;
  m_msg->accY = x.accY;
  m_msg->accZ = x.accZ;
  m_msg->q0 = x.q0;
  m_msg->q1 = x.q1;
  m_msg->q2 = x.q2;
  m_msg->q3 = x.q3;
  m_msg->magX = x.magX;
  m_msg->magY = x.magY;
  m_msg->magZ = x.magZ;
  m_msg->gyroX = x.gyroX;
  m_msg->gyroY = x.gyroY;
  m_msg->gyroZ = x.gyroZ;
  m_msg->timestamp = x.timestamp;
  m_msg->host = x.host;
}

void IMUMessage::getRollPitchYaw(double& roll, double& pitch, double& yaw) {

  double quat_w = m_msg->q0;
  double quat_x = m_msg->q1;
  double quat_y = m_msg->q2;
  double quat_z = m_msg->q3;

  double n =  sqrt(quat_w*quat_w + quat_x*quat_x + quat_y*quat_y + quat_z*quat_z);

  double s = n > 0?2./(n*n):0.;
  
  double m00, m10, m20, m21, m22;
  //double m01, m02, m11, m12
  
  double xs = quat_x*s;
  double ys = quat_y*s;
  double zs = quat_z*s;
  
  double wx = quat_w*xs;
  double wy = quat_w*ys;
  double wz = quat_w*zs;
  
  double xx = quat_x*xs;
  double xy = quat_x*ys;
  double xz = quat_x*zs;
  
  double yy = quat_y*ys;
  double yz = quat_y*zs;
  
  double zz = quat_z*zs;
  
  m00 = 1.0 - (yy + zz);
 // m11 = 1.0 - (xx + zz);
  m22 = 1.0 - (xx + yy);
  
  
  m10 = xy + wz;
 // m01 = xy - wz;
  
  m20 = xz - wy;
  //m02 = xz + wy;
  m21 = yz + wx;
 // m12 = yz - wx;
  
  roll   = atan2(m21,m22);
  pitch = atan2(-m20,sqrt(m21*m21 + m22*m22));
  yaw     = atan2(m10,m00);


}
