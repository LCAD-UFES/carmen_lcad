#include "cpp_laser.h"

LaserConfig::LaserConfig()  {
  setLaserType(UMKNOWN_PROXIMITY_SENSOR);
  setStartAngle(-0.5*M_PI);
  setFOV(M_PI);
  setAngularResolution(carmen_degrees_to_radians(1.0));
  setMaximumRange(81.9);
  setAccuracy(0.01);
  setRemissionMode(REMISSION_NONE);
}

LaserConfig::~LaserConfig() {
}

LaserConfig::LaserConfig(const LaserConfig& x) {
  setLaserConfig( x.getConfig() );
}

LaserConfig::LaserConfig(const carmen_laser_laser_config_t& x) {
  setLaserConfig(x);
}

void LaserConfig::setLaserConfig(const carmen_laser_laser_config_t& x) {
  m_Config = x;
}

///////////////////////////////////////////////////////////

LaserMessage::LaserMessage(int num_readings, int num_remissions)
  : AbstractMessage() {
  m_msg = NULL;
  init(num_readings, num_remissions);
}

LaserMessage::LaserMessage(int num_readings, int num_remissions, const LaserConfig& cfg) 
  : AbstractMessage() {
  m_msg = NULL;
  init(num_readings, num_remissions);
  m_msg->config = cfg;
}

LaserMessage::LaserMessage(const LaserMessage& x) 
  : AbstractMessage(x) {
  m_msg = NULL;
  clone(x);
}

LaserMessage::LaserMessage(const carmen_laser_laser_message& x) {
  m_msg = NULL;
  clone(x);
}

LaserMessage::LaserMessage(carmen_laser_laser_message* x) {
  m_msg = NULL;
  setLaserMessage(x);
}

LaserMessage::~LaserMessage() {
  this->free();
}

LaserMessage::LaserMessage(char* s) {
  m_msg = NULL;
  fromString(s);
}

char* LaserMessage::fromString(char* s) {
  if (m_msg == NULL) {
    m_msg = new carmen_laser_laser_message;
    carmen_erase_structure(m_msg, sizeof(carmen_laser_laser_message));
  }
  return carmen_string_to_laser_laser_message(s, m_msg);
}

void LaserMessage::save(carmen_FILE *logfile, double logger_timestamp) {
  carmen_logwrite_write_laser_laser(m_msg, m_msg->id, logfile, logger_timestamp);
}

AbstractMessage* LaserMessage::clone() const {
  return new LaserMessage(*this);
}

void LaserMessage::init(int num_readings, int num_remissions) {

  if (m_msg != NULL) {
    freeRange();
    freeRemission();
  }
  else {
    m_msg = new carmen_laser_laser_message;
    carmen_test_alloc(m_msg);
  }
  carmen_erase_structure(m_msg, sizeof(carmen_laser_laser_message));

  m_msg->config = LaserConfig();
  
  initRange(num_readings);
  initRemission(num_remissions);
}

void LaserMessage::initRange(int num_readings) {
  freeRange();
  m_msg->num_readings = num_readings;
  if (num_readings>0) {
    m_msg->range = new double[num_readings];
    carmen_test_alloc(m_msg->range);
  }
  else
    m_msg->range = NULL;
}

void LaserMessage::initRemission(int num_remissions) {
  freeRemission();
  m_msg->num_remissions = num_remissions;
  if (num_remissions>0) {
    m_msg->remission = new double[num_remissions];
    carmen_test_alloc(m_msg->remission);
  }
  else 
    m_msg->remission = NULL;
}


void LaserMessage::free() {
  freeRange();
  freeRemission();
  if (m_msg != NULL) {
    delete m_msg;
    m_msg = NULL;
  }
}

void LaserMessage::freeRange() {
  if (m_msg != NULL) {
    if (m_msg->range != NULL) 
      delete [] m_msg->range;
    m_msg->range = NULL;
  }
}

void LaserMessage::freeRemission() {
  if (m_msg != NULL) {
    if (m_msg->remission != NULL) 
      delete [] m_msg->remission;
    m_msg->remission = NULL;
  }
}

void LaserMessage::setLaserMessage( carmen_laser_laser_message* x) {
  m_msg = x;
}


void LaserMessage::clone(const LaserMessage& x) {
  clone(*(x.m_msg));
}

void LaserMessage::clone(const carmen_laser_laser_message& x) {
  this->free();
  m_msg = new carmen_laser_laser_message;
  carmen_test_alloc(m_msg);
  *m_msg = x;
  if (x.num_readings>0) {
    m_msg->range = new double[x.num_readings];
    carmen_test_alloc(m_msg->range);
  }
  else 
    m_msg->range = NULL;

  for (int i=0; i<x.num_readings; i++) 
    m_msg->range[i] = x.range[i];

  if (x.num_remissions>0) {
    m_msg->remission = new double[x.num_remissions];
    carmen_test_alloc(m_msg->remission);
  }
  else 
    m_msg->remission = NULL;
  for (int i=0; i<x.num_remissions; i++) 
    m_msg->remission[i] = x.remission[i];
}

const double* LaserMessage::getRange() const{
  return m_msg->range;
}

const double* LaserMessage::getRemission() const{
  return m_msg->remission;
}
 
double* LaserMessage::getRange() {
  return m_msg->range;
}

double* LaserMessage::getRemission() {
  return m_msg->remission;
}

int LaserMessage::getNumReadings() const {
  return m_msg->num_readings;
}

int LaserMessage::getNumRanges() const {
  return getNumReadings();
}

double LaserMessage::getRange(int i) const {
  return m_msg->range[i];
}

void LaserMessage::setRange(int i, double val) {
  m_msg->range[i] = val;
}

int LaserMessage::getNumRemissions() const {
  return m_msg->num_remissions;
}

double LaserMessage::getRemission(int i) const {
  return m_msg->remission[i];
}

void LaserMessage::setRemission(int i, double val) {
  m_msg->remission[i] = val;
}

LaserConfig LaserMessage::getConfig() const {
  return LaserConfig(m_msg->config);
}

void LaserMessage::setConfig(const LaserConfig& x) {
  setConfig( (const carmen_laser_laser_config_t&) x);
}

void LaserMessage::setConfig(const carmen_laser_laser_config_t& x) {
  m_msg->config = x;
}


Point LaserMessage::endPoint(const OrientedPoint& robotPose, int beam) const {

  Point endPt;

  double angle = robotPose.theta + m_msg->config.start_angle + m_msg->config.angular_resolution * ((double) beam);

  endPt.x = robotPose.x + m_msg->range[beam] * cos( angle );
  endPt.y = robotPose.y + m_msg->range[beam] * sin( angle );

  return endPt;
}
