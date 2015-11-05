#include <carmen/cpp_carmen.h>


int main(int, char** ) {

  LaserMessage l;
  l.setTimestamp(carmen_get_time());

  OdometryMessage o;
  RobotLaserMessage r;
  TrueposMessage t;

  CarmenMap cm;
  RefProbMap rpm;
  FloatMap fm;
  DoubleMap dm;
  IntMap im;



  return 0;
}
