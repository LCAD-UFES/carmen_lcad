#include "cpp_unknownmessage.h"

UnknownMessage::UnknownMessage() : AbstractMessage() {
  logstr=NULL;
}

UnknownMessage::~UnknownMessage() {
  if (logstr!=NULL)
    delete logstr;
}


UnknownMessage::UnknownMessage(const UnknownMessage& x) : AbstractMessage(x) {
  logstr = NULL;
  
  if (x.logstr != NULL) {
    logstr = new char[strlen(x.logstr)+1];
    carmen_test_alloc(logstr);
    strcpy(logstr, x.logstr);
  }
}
  
UnknownMessage::UnknownMessage(char* s) {
  logstr = NULL;
  fromString(s);
}

double UnknownMessage::getTimestamp() const { 
  return 0; 
}

void UnknownMessage::save(carmen_FILE *logfile, double ) {
  carmen_fprintf(logfile, logstr);
}

char*  UnknownMessage::fromString(char* s) {
  if (logstr!=NULL)
    delete logstr;
  logstr = NULL;

  if (s != NULL) {
    logstr = new char[strlen(s)+1];
    carmen_test_alloc(logstr);
    strcpy(logstr, s);
    return (char*) &(logstr[strlen(s)-1]);
  }
  else
    return NULL;
    
}


AbstractMessage* UnknownMessage::clone() const {
  return new UnknownMessage(*this);
}

char*  UnknownMessage::getString() {
  return logstr;
}
