#include "cpp_simulator.h"

TrueposMessage::TrueposMessage()
  : AbstractMessage() {
  m_msg = NULL;
  init();
}

TrueposMessage::TrueposMessage(const OrientedPoint& truepose, const OrientedPoint& odometrypose) 
  : AbstractMessage() {
  m_msg = NULL;
  init();
  m_msg->truepose.x = truepose.x;
  m_msg->truepose.y = truepose.y;
  m_msg->truepose.theta = truepose.theta;

  m_msg->odometrypose.x = odometrypose.x;
  m_msg->odometrypose.y = odometrypose.y;
  m_msg->odometrypose.theta = odometrypose.theta;
}

TrueposMessage::TrueposMessage(const TrueposMessage& x) 
  : AbstractMessage(x) {
  m_msg = NULL;
  clone(x);
}

TrueposMessage::TrueposMessage(const carmen_simulator_ackerman_truepos_message& x) {
  m_msg = NULL;
  clone(x);
}

TrueposMessage::TrueposMessage(carmen_simulator_ackerman_truepos_message* x) {
  m_msg = NULL;
  setTrueposMessage(x);
}

TrueposMessage::~TrueposMessage() {
  this->free();
}

void TrueposMessage::save(carmen_FILE *logfile, double logger_timestamp) {
  carmen_logwrite_write_ackerman_truepos(m_msg, logfile, logger_timestamp);
}

TrueposMessage::TrueposMessage(char* s) {
  m_msg = NULL;
  fromString(s);
}

char* TrueposMessage::fromString(char* s) {
  if (m_msg == NULL) {
    m_msg = new carmen_simulator_ackerman_truepos_message;
    carmen_erase_structure(m_msg, sizeof(carmen_simulator_ackerman_truepos_message));
  }
  return carmen_string_to_simulator_ackerman_truepos_message(s, m_msg);
}

AbstractMessage* TrueposMessage::clone() const {
  return new TrueposMessage(*this);
}

void TrueposMessage::init() {

  if (m_msg != NULL) {
    this->free();
  }
  m_msg = new carmen_simulator_ackerman_truepos_message;
  carmen_test_alloc(m_msg);
  carmen_erase_structure(m_msg, sizeof(carmen_simulator_ackerman_truepos_message));
}

void TrueposMessage::free() {
  if (m_msg != NULL) {
    delete m_msg;
    m_msg = NULL;
  }
}

void TrueposMessage::setTrueposMessage( carmen_simulator_ackerman_truepos_message* x) {
  m_msg = x;
}


void TrueposMessage::clone(const TrueposMessage& x) {
  clone(*(x.m_msg));
}

void TrueposMessage::clone(const carmen_simulator_ackerman_truepos_message& x) {
  m_msg->truepose.x = x.truepose.x;
  m_msg->truepose.y = x.truepose.y;
  m_msg->truepose.theta = x.truepose.theta;
  m_msg->odometrypose.x = x.odometrypose.x;
  m_msg->odometrypose.y = x.odometrypose.y;
  m_msg->odometrypose.theta = x.odometrypose.theta;

  m_msg->timestamp = x.timestamp;
  m_msg->host = x.host;
}

OrientedPoint TrueposMessage::getOdometryPose() const {
  return OrientedPoint(m_msg->odometrypose.x,
		       m_msg->odometrypose.y,
		       m_msg->odometrypose.theta);
}

void TrueposMessage::setOdodmetryPose(const OrientedPoint& pose ) {
  m_msg->odometrypose.x = pose.x;
  m_msg->odometrypose.y = pose.y;
  m_msg->odometrypose.theta = pose.theta;
}

OrientedPoint TrueposMessage::getTruePose() const {
  return OrientedPoint(m_msg->truepose.x,
		       m_msg->truepose.y,
		       m_msg->truepose.theta);
}

void TrueposMessage::setTruePose(const OrientedPoint& pose ) {
  m_msg->truepose.x = pose.x;
  m_msg->truepose.y = pose.y;
  m_msg->truepose.theta = pose.theta;
}
