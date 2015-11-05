#include "cpp_carmenmap.h"

CarmenMap::CarmenMap() : AbstractMap<double>() {
  m_map = NULL;
  m_defaultCell = -1;
}

CarmenMap::CarmenMap(const CarmenMap& x) : AbstractMap<double>(x) {
  m_defaultCell = -1;
  copyMap(x);
}

CarmenMap::CarmenMap(const MapConfig& cfg) : AbstractMap<double>(cfg) {
  m_map = NULL;
  m_defaultCell = -1;
  init(cfg);
}

bool CarmenMap::init(const MapConfig& cfg) {
  // both invalid
  if (!cfg.isValid() && !(AbstractMap<double>::m_cfg.isValid()))
    return false;

  // already the correct size
  if (m_map != NULL &&  
      cfg.isValid() && 
      AbstractMap<double>::m_cfg.isValid() &&
      AbstractMap<double>::m_cfg.m_sizeX == cfg.m_sizeX &&
      AbstractMap<double>::m_cfg.m_sizeY == cfg.m_sizeY)
    return true;

  // free the allocated memory
  if (m_map != NULL && AbstractMap<double>::m_cfg.isValid()) {
    if (m_map->complete_map != NULL)
      delete [] m_map->complete_map;
    if (m_map->map != NULL)
      delete [] m_map->map;
    m_map->complete_map = NULL;
    m_map->map = NULL;
  }

  AbstractMap<double>::m_cfg = cfg;

  // nothing to do anyumore
  if (!cfg.isValid())
    return false;

  // has the map structre bee initialized at all
  if (m_map == NULL) {
    m_map = new carmen_map_t;
    m_map->config.x_size = cfg.m_sizeX;
    m_map->config.y_size = cfg.m_sizeY;
    m_map->config.resolution = cfg.m_res;
    m_map->config.map_name = NULL;
  }

  // all right, alloc the memory
  m_map->complete_map = new double[cfg.m_sizeX*cfg.m_sizeY];
  carmen_test_alloc(m_map->complete_map);
  m_map->map = new double*[cfg.m_sizeX];
  carmen_test_alloc(m_map->map);
  for (int x=0; x < cfg.m_sizeX; x++) {
    m_map->map[x] = &m_map->complete_map[x*cfg.m_sizeY];
    carmen_test_alloc(m_map->map[x]);
  }
  return true;
}

CarmenMap::~CarmenMap() {
  if (AbstractMap<double>::m_cfg.isValid() && m_map != NULL)
    carmen_map_destroy(&m_map);
  m_map = NULL;
}

void CarmenMap::mapConfigUpdated(const Point& offset) {
  m_cfg = MapConfig(m_map->config.x_size, m_map->config.y_size, m_map->config.resolution, offset) ;
}

void CarmenMap::setMap(carmen_map_t* m) {
  m_map = m;
  mapConfigUpdated();
}

void CarmenMap::copyMap(carmen_map_t* m) {
  m_map = carmen_map_clone(m);
  mapConfigUpdated();
}

double& CarmenMap::getCell(int x, int y) const {
  return m_map->map[x][y];
}

double& CarmenMap::getCell(int x, int y) {
  return m_map->map[x][y];
}

const double& CarmenMap::defaultCell() const {
  return m_defaultCell;
}
