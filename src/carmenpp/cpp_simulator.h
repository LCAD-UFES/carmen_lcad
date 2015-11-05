#ifndef CARMEN_CPP_TRUEPOS_MESSAGE_H
#define CARMEN_CPP_TRUEPOS_MESSAGE_H

#include <carmen/cpp_global.h>
#include <carmen/cpp_abstractmessage.h>
#include <carmen/cpp_point.h>




class TrueposMessage : public AbstractMessage {
 public:
  TrueposMessage();
  TrueposMessage(const OrientedPoint& truepose, const OrientedPoint& odometrypose);
  TrueposMessage(const TrueposMessage& x);
  TrueposMessage(const carmen_simulator_ackerman_truepos_message& x);
  TrueposMessage(carmen_simulator_ackerman_truepos_message* x);
  TrueposMessage(char* s);
  virtual ~TrueposMessage();

  void init();
  void clone(const TrueposMessage& x);
  void clone(const carmen_simulator_ackerman_truepos_message& x);
  void setTrueposMessage(carmen_simulator_ackerman_truepos_message* x);
  void free();

  OrientedPoint getOdometryPose() const;
  void setOdodmetryPose(const OrientedPoint& pose );

  OrientedPoint getTruePose() const;
  void setTruePose(const OrientedPoint& pose );

  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, TruePoseX, truepose.x);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, TruePoseY, truepose.y);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, TruePoseTheta, truepose.theta);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, OdometryPoseX, odometrypose.x);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, OdometryPoseY, odometrypose.y);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, double, OdometryPoseTheta, odometrypose.theta);
  DEFAULT_STRUCT_PARAM_SET_GET(*m_msg, char*, Host, host);
  STRUCT_PARAM_VIRTUAL_SET_GET(*m_msg, double, Timestamp, timestamp, public, public);

  carmen_inline carmen_simulator_ackerman_truepos_message* toCarmenMsg() {return m_msg;}
  carmen_inline operator carmen_simulator_ackerman_truepos_message*() {return m_msg;}
  carmen_inline operator const carmen_simulator_ackerman_truepos_message&() const {return *m_msg;}
  carmen_inline operator carmen_simulator_ackerman_truepos_message&() {return *m_msg;}

  virtual const char* getMessageID() const {
    return "TRUEPOS";
  };

  virtual void save(carmen_FILE *logfile, double logger_timestamp);
  virtual char*  fromString(char* s);
  virtual AbstractMessage* clone() const;

 protected:
  carmen_simulator_ackerman_truepos_message * m_msg;
};

#endif 
