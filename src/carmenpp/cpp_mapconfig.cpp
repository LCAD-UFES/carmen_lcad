#include "cpp_mapconfig.h"


MapConfig::MapConfig() { 
  m_sizeX  = -1;
  m_sizeY  = -1;
  m_res    = 0.0;
  m_offset = Point(0.0,0.0);
}

MapConfig::MapConfig(int sizeX, int sizeY, double res, Point offset) { 
  m_sizeX  = sizeX;
  m_sizeY  = sizeY;
  m_res    = res;
  m_offset = offset;
}

MapConfig::MapConfig(const MapConfig& src) {
  m_sizeX  = src.m_sizeX;
  m_sizeY  = src.m_sizeY;
  m_res    = src.m_res;
  m_offset = src.m_offset;
}

bool MapConfig::isValid() const {
  if (m_sizeX <= 0 || m_sizeY <= 0 ||  m_res <=0.0)
    return false;
  return true;
}
