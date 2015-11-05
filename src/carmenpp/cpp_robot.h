#ifndef CARMEN_CPP_ROBOT_MESSAGE_H
#define CARMEN_CPP_ROBOT_MESSAGE_H

#include <carmen/cpp_global.h>
#include <carmen/cpp_abstractmessage.h>
#include <carmen/cpp_point.h>
#include <carmen/cpp_laser.h>


class RobotLaserMessage : public AbstractMessage {
 public:
  RobotLaserMessage(int num_readings=0, int num_remissions=0);
  RobotLaserMessage(int num_readings, int num_remissions, const LaserConfig& cfg);
  RobotLaserMessage(const LaserMessage& x, const OrientedPoint& robotPose);
  RobotLaserMessage(const carmen_laser_laser_message& x, const carmen_point_t& robotPose);
  RobotLaserMessage(const RobotLaserMessage& x);
  RobotLaserMessage(const carmen_robot_ackerman_laser_message& x);
  RobotLaserMessage(carmen_robot_ackerman_laser_message* x);
  RobotLaserMessage(char* s);

  virtual ~RobotLaserMessage();

  void setRobotLaserMessage(carmen_robot_ackerman_laser_message* x);
  void init(int num_readings, int num_remissions);
  void initRange(int num_readings);
  void initRemission(int num_remission);
  void clone(const RobotLaserMessage& x);
  void clone(const carmen_robot_ackerman_laser_message& x);
  void clone(const carmen_laser_laser_message& x, double lx, double ly, double ltheta);
  void free();
  void freeRange();
  void freeRemission();
  void freeTooClose();

  double* getRange();
  double* getRemission();
  char* getTooClose();
  const double* getRange() const ;
  const double* getRemission() const ;
  const char* getTooClose() const ;

  int   getNumReadings() const;
  int   getNumRanges() const;
  double getRange(int i) const;
  void  setRange(int i, double val);

  bool isMaxRange(int i) const;
  int   getNumRemissions() const;
  double getRemission(int i) const;
  void  setRemission(int i, double val);

  char getTooClose(int i) const;
  void  setTooClose(int i, char val);

  OrientedPoint getRobotPose() const;
  void setRobotPose(const OrientedPoint& robotPose );

  OrientedPoint getLaserPose() const;
  void setLaserPose(const OrientedPoint& LaserPose );

  Point endPoint(int beam) const;

  LaserConfig getConfig() const;
  void setConfig(const LaserConfig& x) ;
  void setConfig(const carmen_laser_laser_config_t& x) ;

  carmen_inline carmen_robot_ackerman_laser_message* toCarmenMsg()   {return m_msg;}
  carmen_inline operator carmen_robot_ackerman_laser_message*()      {return m_msg;}
  carmen_inline operator const carmen_robot_ackerman_laser_message&() const {return *m_msg;}
  carmen_inline operator carmen_robot_ackerman_laser_message&() {return *m_msg;}



  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, Vel, v);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, PHI, phi);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, ForwardSafetyDist, forward_safety_dist);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, SideSafetyDist, side_safety_dist);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, TurnAxis, turn_axis);

  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, int, ID, id); 
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, char*, Host, host);
  STRUCT_PARAM_VIRTUAL_SET_GET(*m_msg, double, Timestamp, timestamp, public, public);

  virtual const char* getMessageID() const {
    return "ROBOTLASER";
  };

  virtual void save(carmen_FILE *logfile, double logger_timestamp);
  virtual char*  fromString(char* s);
  virtual AbstractMessage* clone() const;

 protected:
  carmen_robot_ackerman_laser_message * m_msg;
};

#endif 
