#ifndef CARMEN_CPP_MAPCONFIG_H
#define CARMEN_CPP_MAPCONFIG_H

#include <carmen/carmen.h>
#include <carmen/cpp_point.h>

class MapConfig {
 public:
  MapConfig();
  virtual ~MapConfig() {};
  MapConfig(int sizeX, int sizeY,  double res, Point offset);
  MapConfig(const MapConfig& src);
  bool isValid() const;

  int m_sizeX;
  int m_sizeY;
  double m_res;
  Point m_offset;
};

carmen_inline bool operator==(const MapConfig& cfg1, const MapConfig& cfg2)  {
  if (cfg1.m_sizeX  == cfg2.m_sizeX && 
      cfg1.m_sizeY  == cfg2.m_sizeY && 
      cfg1.m_res    == cfg2.m_res && 
      cfg1.m_offset == cfg2.m_offset)
    return true;
  else
    return false;
};

carmen_inline bool operator!=(const MapConfig& cfg1, const MapConfig& cfg2)  {
  return !(cfg1 == cfg2);
};

#endif
