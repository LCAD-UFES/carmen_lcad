#ifndef BEOMONKEYTYPES_H_DEFINED
#define BEOMONKEYTYPES_H_DEFINED

//this is a silly little class and should be included elsewhere
class Position
{
public:
  int servo;
  float position;
  int time;
  Position(int s, float p, int t){servo = s;position = p;time = t;}
};

#endif
