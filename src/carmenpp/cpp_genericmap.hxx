
template<class CELL>
GenericMap<CELL>::GenericMap() : AbstractMap<CELL>() {
  m_maplinear = NULL;
  m_map = NULL;
}

template<class CELL>
GenericMap<CELL>::GenericMap(const MapConfig& cfg) 
  : AbstractMap<CELL>(cfg) {
  m_maplinear = NULL;
  m_map = NULL;
  init(cfg);
}

template<class CELL>
GenericMap<CELL>::GenericMap(const GenericMap<CELL>& src) 
  : AbstractMap<CELL>(src) {
  m_maplinear = NULL;
  m_map = NULL;
  init(src.getConfig());
  copy(src, IntPoint(0,0));

}

template<class CELL>
GenericMap<CELL>::~GenericMap() {
  if (AbstractMap<CELL>::m_cfg.isValid()) {
    if (m_maplinear != NULL)
      delete [] m_maplinear;
    if (m_map != NULL)
      delete [] m_map;
    m_maplinear = NULL;
    m_map = NULL;
  }
}
template<class CELL>
bool GenericMap<CELL>::init(const MapConfig& cfg) {

  // both invalid
  if (!cfg.isValid() && !(AbstractMap<CELL>::m_cfg.isValid()))
    return false;

  // already the correct size
  if (m_map != NULL && m_maplinear != NULL &&
      cfg.isValid() && 
      AbstractMap<CELL>::m_cfg.isValid() && 
      AbstractMap<CELL>::m_cfg.m_sizeX == cfg.m_sizeX &&
      AbstractMap<CELL>::m_cfg.m_sizeY == cfg.m_sizeY)
    return true;

  // free the allocated memory
  if (AbstractMap<CELL>::m_cfg.isValid()) {
    if (m_maplinear != NULL)
      delete [] m_maplinear;
    if (m_map != NULL)
      delete [] m_map;
    m_maplinear = NULL;
    m_map = NULL;
  }
  AbstractMap<CELL>::m_cfg = cfg;

  // nothing to do anyumore
  if (!cfg.isValid())
    return false;

  // all right, alloc the memory
  m_maplinear = new CELL[cfg.m_sizeX*cfg.m_sizeY];
  carmen_test_alloc(m_maplinear);
  m_map = new CELL*[cfg.m_sizeX];
  carmen_test_alloc(m_map);
  for (int x=0; x < cfg.m_sizeX; x++) {
    m_map[x] = &m_maplinear[x*cfg.m_sizeY];
  }
  return true;
}

template<class CELL>
CELL& GenericMap<CELL>::getCell(int x, int y) {
  return m_map[x][y];
}


template<class CELL>
CELL& GenericMap<CELL>::getCell(int x, int y) const {
  return m_map[x][y];
}


template<class CELL>
const CELL& GenericMap<CELL>:: defaultCell() const {
  return m_defaultCell;
}
