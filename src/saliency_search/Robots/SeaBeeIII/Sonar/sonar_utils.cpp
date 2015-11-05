#include "sonar_utils.h"

namespace SeaBee3_Sonar {

double sign(double val) {
  if (val < 0)
    return -1.0;
  else if (val > 0)
    return 1.0;
  return 0.0;
}

double getSamplingFrequency(double bitrate, double m, double b) {
  return m * bitrate + b;
}

} // End namespace SeaBee3_sonar
