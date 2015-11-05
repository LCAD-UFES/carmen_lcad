#include "cpp_robot.h"


RobotLaserMessage::RobotLaserMessage(int num_readings, int num_remissions)
  : AbstractMessage() {
  m_msg = NULL;
  init(num_readings, num_remissions);
}

RobotLaserMessage::RobotLaserMessage(int num_readings, int num_remissions, const LaserConfig& cfg) 
  : AbstractMessage() {
  m_msg = NULL;
  init(num_readings, num_remissions);
  m_msg->config = cfg;
}

RobotLaserMessage::RobotLaserMessage(const RobotLaserMessage& x) 
  : AbstractMessage(x) {
  m_msg = NULL;
  clone(x);
}

RobotLaserMessage::RobotLaserMessage(const LaserMessage& x, 
				     const OrientedPoint& laserPose) 
  : AbstractMessage() {
  m_msg = NULL;
  clone(x, laserPose.x, laserPose.y, laserPose.theta);
}

RobotLaserMessage::RobotLaserMessage(const carmen_laser_laser_message& x, 
				     const carmen_point_t& laserPose) 
  : AbstractMessage() {
  m_msg = NULL;
  clone(x, laserPose.x, laserPose.y, laserPose.theta );
}

RobotLaserMessage::RobotLaserMessage(const carmen_robot_ackerman_laser_message& x) {
  m_msg = NULL;
  clone(x);
}

RobotLaserMessage::RobotLaserMessage(carmen_robot_ackerman_laser_message* x) {
  m_msg = NULL;
  setRobotLaserMessage(x);
}

RobotLaserMessage::~RobotLaserMessage() {
  this->free();
}
void RobotLaserMessage::save(carmen_FILE *logfile, double logger_timestamp) {
  carmen_logwrite_write_robot_ackerman_laser(m_msg,  m_msg->id
				    , logfile, logger_timestamp);
}

RobotLaserMessage::RobotLaserMessage(char* s) {
  m_msg = NULL;
  fromString(s);
}

char* RobotLaserMessage::fromString(char* s) {
  if (m_msg == NULL) {
    m_msg = new carmen_robot_ackerman_laser_message;
    carmen_erase_structure(m_msg, sizeof(carmen_robot_ackerman_laser_message));
  }

  return carmen_string_to_robot_ackerman_laser_message(s, m_msg);
}

AbstractMessage* RobotLaserMessage::clone() const {
  return new RobotLaserMessage(*this);
}

void RobotLaserMessage::init(int num_readings, int num_remissions) {

  if (m_msg != NULL) {
    freeRange();
    freeRemission();
  }
  else {
    m_msg = new carmen_robot_ackerman_laser_message;
    carmen_test_alloc(m_msg);
  }
  carmen_erase_structure(m_msg, sizeof(carmen_robot_ackerman_laser_message));
  
  m_msg->config = LaserConfig();
  
  initRange(num_readings);
  initRemission(num_remissions);
}

void RobotLaserMessage::initRange(int num_readings) {
  freeRange();
  m_msg->num_readings = num_readings;
  if (num_readings>0) {
    m_msg->range = new double[num_readings];
    carmen_test_alloc(m_msg->range);
  }
  else
    m_msg->range = NULL;
}

void RobotLaserMessage::initRemission(int num_remissions) {
  freeRemission();
  m_msg->num_remissions = num_remissions;

  if (num_remissions>0) {
    m_msg->remission = new double[num_remissions];
    carmen_test_alloc(m_msg->remission);
  }
  else
    m_msg->remission = NULL;
}


void RobotLaserMessage::free() {
  freeTooClose();
  freeRange();
  freeRemission();
  if (m_msg != NULL) {
    delete m_msg;
    m_msg = NULL;
  }
}

void RobotLaserMessage::freeTooClose() {
  if (m_msg != NULL) {
    if (m_msg->tooclose != NULL) 
      delete [] m_msg->tooclose;
    m_msg->tooclose = NULL;
  }
}

void RobotLaserMessage::freeRange() {
  if (m_msg != NULL) {
    if (m_msg->range != NULL) 
      delete [] m_msg->range;
    m_msg->range = NULL;
  }
}

void RobotLaserMessage::freeRemission() {
  if (m_msg != NULL) {
    if (m_msg->remission != NULL) 
      delete [] m_msg->remission;
    m_msg->remission = NULL;
  }
}

void RobotLaserMessage::setRobotLaserMessage( carmen_robot_ackerman_laser_message* x) {
  m_msg = x;
}


void RobotLaserMessage::clone(const RobotLaserMessage& x) {
  clone(*(x.m_msg));
}

void RobotLaserMessage::clone(const carmen_robot_ackerman_laser_message& x) {
  this->free();
  m_msg = new carmen_robot_ackerman_laser_message;
  carmen_test_alloc(m_msg);
  *m_msg = x;
  if (x.num_readings>0) {
    m_msg->range = new double[x.num_readings];
    carmen_test_alloc(m_msg->range);

    m_msg->tooclose = new char[x.num_readings];
    carmen_test_alloc(m_msg->tooclose);
  }
  else {
    m_msg->range = NULL;
    m_msg->tooclose = NULL;
  }

  for (int i=0; i<x.num_readings; i++) {
    m_msg->range[i] = x.range[i];
    m_msg->tooclose[i] = x.tooclose[i];
  }

  if (x.num_remissions>0) {
    m_msg->remission = new double[x.num_remissions];
    carmen_test_alloc(m_msg->remission);
  }
  else 
    m_msg->remission = NULL;
  for (int i=0; i<x.num_remissions; i++) 
    m_msg->remission[i] = x.remission[i];
}

void RobotLaserMessage::clone(const carmen_laser_laser_message& x, 
			      double lx, double ly, double ltheta) {
  this->free();
  m_msg = new carmen_robot_ackerman_laser_message;
  carmen_test_alloc(m_msg);

  m_msg->config = x.config;
  m_msg->num_readings = x.num_readings;
  m_msg->range = NULL;
  m_msg->tooclose = NULL;
  m_msg->num_remissions = x.num_remissions;
  m_msg->remission = NULL;
  m_msg->laser_pose.x = lx;
  m_msg->laser_pose.y = ly;
  m_msg->laser_pose.theta = ltheta;
  m_msg->robot_pose.x = lx;
  m_msg->robot_pose.y = ly;
  m_msg->robot_pose.theta = ltheta;
  m_msg->v = 0.0;
  m_msg->phi = 0.0;
  m_msg->forward_safety_dist = 0.0;
  m_msg->side_safety_dist = 0.0;
  m_msg->turn_axis = 0.0;
  m_msg->timestamp = x.timestamp;
  m_msg->host = x.host;

  if (x.num_readings>0) {
    m_msg->range = new double[x.num_readings];
    carmen_test_alloc(m_msg->range);

    m_msg->tooclose = new char[x.num_readings];
    carmen_test_alloc(m_msg->tooclose);
  }
  else {
    m_msg->range = NULL;
    m_msg->tooclose = NULL;
  }

  for (int i=0; i<x.num_readings; i++) {
    m_msg->range[i] = x.range[i];
    m_msg->tooclose[i] = 0;
  }

  if (x.num_remissions>0) {
    m_msg->remission = new double[x.num_remissions];
    carmen_test_alloc(m_msg->remission);
  }
  else 
    m_msg->remission = NULL;

  for (int i=0; i<x.num_remissions; i++) 
    m_msg->remission[i] = x.remission[i];

}

const double* RobotLaserMessage::getRange() const{
  return m_msg->range;
}

const double* RobotLaserMessage::getRemission() const{
  return m_msg->remission;
}
 
double* RobotLaserMessage::getRange() {
  return m_msg->range;
}

double* RobotLaserMessage::getRemission() {
  return m_msg->remission;
}

char* RobotLaserMessage::getTooClose() {
  return m_msg->tooclose;
}

const char* RobotLaserMessage::getTooClose() const{
  return m_msg->tooclose;
}

char RobotLaserMessage::getTooClose(int i) const {
  return m_msg->tooclose[i];
}

void RobotLaserMessage::setTooClose(int i, char val) {
  m_msg->tooclose[i] = val;
}

int RobotLaserMessage::getNumReadings() const {
  return m_msg->num_readings;
}

int RobotLaserMessage::getNumRanges() const {
  return getNumReadings();
}

double RobotLaserMessage::getRange(int i) const {
  return m_msg->range[i];
}

void RobotLaserMessage::setRange(int i, double val) {
  m_msg->range[i] = val;
}

int RobotLaserMessage::getNumRemissions() const {
  return m_msg->num_remissions;
}

double RobotLaserMessage::getRemission(int i) const {
  return m_msg->remission[i];
}

void RobotLaserMessage::setRemission(int i, double val) {
  m_msg->remission[i] = val;
}

LaserConfig RobotLaserMessage::getConfig() const {
  return LaserConfig(m_msg->config);
}

void RobotLaserMessage::setConfig(const LaserConfig& x) {
  setConfig( (const carmen_laser_laser_config_t&) x);
}

void RobotLaserMessage::setConfig(const carmen_laser_laser_config_t& x) {
  m_msg->config = x;
}

bool RobotLaserMessage::isMaxRange(int i) const{
  return (m_msg->range[i] >= m_msg->config.maximum_range);
}


Point RobotLaserMessage::endPoint(int beam) const {

  Point endPt;

  double angle = m_msg->laser_pose.theta + m_msg->config.start_angle + m_msg->config.angular_resolution * ((double) beam);

  endPt.x = m_msg->laser_pose.x + m_msg->range[beam] * cos( angle );
  endPt.y = m_msg->laser_pose.y + m_msg->range[beam] * sin( angle );

  return endPt;
}


OrientedPoint RobotLaserMessage::getRobotPose() const {
  return OrientedPoint(m_msg->robot_pose.x,
		       m_msg->robot_pose.y,
		       m_msg->robot_pose.theta);
}

void RobotLaserMessage::setRobotPose(const OrientedPoint& robotPose ) {
  m_msg->robot_pose.x = robotPose.x;
  m_msg->robot_pose.y = robotPose.y;
  m_msg->robot_pose.theta = robotPose.theta;
}

OrientedPoint RobotLaserMessage::getLaserPose() const {
  return OrientedPoint(m_msg->laser_pose.x,
		       m_msg->laser_pose.y,
		       m_msg->laser_pose.theta);
}

void RobotLaserMessage::setLaserPose(const OrientedPoint& laserPose ) {
  m_msg->laser_pose.x = laserPose.x;
  m_msg->laser_pose.y = laserPose.y;
  m_msg->laser_pose.theta = laserPose.theta;
}
