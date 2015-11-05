#include "cpp_logfile.h"


LogFile::LogFile() : Collection() {
}

LogFile::LogFile(char* filename) : Collection() {
  load(filename);
}

LogFile::LogFile(const LogFile& x) : Collection() {
  for (Collection::const_iterator it = x.begin();
       it != x.end(); ++it) {
    push_back((*it)->clone());
  }
}

LogFile::~LogFile() {
}

bool LogFile::load(char* filename, bool verbose) {
  char line[100000];

  carmen_FILE *logfile = NULL;
  carmen_logfile_index_p logfile_index = NULL;

  logfile = carmen_fopen(filename, "r");
  if(logfile == NULL) {
    if (verbose)
      carmen_warn("Error: could not open file %s for reading.\n", filename);
    return false;
  }

  /* index the logfile */
  logfile_index = carmen_logfile_index_messages(logfile);

  for(int i = 0; i < logfile_index->num_messages; i++) {
    
    /* read i-th line */
    carmen_logfile_read_line(logfile_index, logfile, i, 4000000, line);

    /* create messages */
    if(strncmp(line, "ODOM ", 5) == 0) {
      push_back(new OdometryMessage(line));
    }
    else if(strncmp(line, "RAWLASER", 8) == 0) {
      push_back(new LaserMessage(line));
    }
    else if(strncmp(line, "ROBOTLASER", 10) == 0) {
      push_back(new RobotLaserMessage(line));
    }
    else if(strncmp(line, "FLASER ", 7) == 0) {
      push_back(new RobotLaserMessage(line));
    }
    else if(strncmp(line, "RLASER ", 7) == 0) {
      push_back(new RobotLaserMessage(line));
    }
    else if(strncmp(line, "TRUEPOS ", 8) == 0) {
      push_back(new TrueposMessage(line));
    }
    else if(strncmp(line, "IMU ", 4) == 0) {
      push_back(new IMUMessage(line));
    }
    else if(strlen(line) > 1) {
      push_back(new UnknownMessage(line));
    }
  }
  carmen_fclose(logfile);  
  return true;
}

bool LogFile::save(char* filename, bool verbose) const {
  
  carmen_FILE *logfile = NULL;
  
  logfile = carmen_fopen(filename, "w");
  if(logfile == NULL) {
    if (verbose)
      carmen_warn("Error: could not open file %s for writing.\n", filename);
    return false;
  }
  
  double logger_starttime = -1.0;
  
  for (Collection::const_iterator it = begin();
       it != end(); ++it) {
    if (logger_starttime < 0)
      logger_starttime = (*it)->getTimestamp();
    (*it)->save(logfile, (*it)->getTimestamp() - logger_starttime) ;
  }
  carmen_fclose(logfile);
  return true;
}


