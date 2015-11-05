#ifndef CARMEN_CPP_ABSTRACT_MESSAGE_H
#define CARMEN_CPP_ABSTRACT_MESSAGE_H

#include <carmen/cpp_global.h>

class AbstractMessage {
 public: 
  AbstractMessage() {}
  virtual ~AbstractMessage() {}
  AbstractMessage(const AbstractMessage& x) {
    *this = x;
  }
 
  virtual double getTimestamp() const= 0;
  virtual const char* getMessageID() const= 0;
  virtual void save(carmen_FILE *logfile, double logger_timestamp) = 0;
  virtual char*  fromString(char* s) = 0;
  virtual AbstractMessage* clone() const = 0;
};

#endif
