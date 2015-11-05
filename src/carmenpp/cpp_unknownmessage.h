#ifndef CARMEN_CPP_UNKNOWN_MESSAGE_H
#define CARMEN_CPP_UNKNOWN_MESSAGE_H

#include <carmen/cpp_global.h>
#include <carmen/cpp_abstractmessage.h>

class UnknownMessage : public AbstractMessage {
 public: 
  UnknownMessage();
  virtual ~UnknownMessage();
  UnknownMessage(const UnknownMessage& x);
  UnknownMessage(char* s);

  char*  getString();


  virtual const char* getMessageID() const {
    return "UNKNOWN";
  };
 
  virtual double getTimestamp() const;
  virtual void save(carmen_FILE *logfile, double);
  virtual char*  fromString(char* s);
  virtual AbstractMessage* clone() const;


 protected:
  char* logstr;
};

#endif
