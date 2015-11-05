#ifndef CARMEN_CPP_GENERIC_MAP_H
#define CARMEN_CPP_GENERIC_MAP_H

#include <carmen/cpp_point.h>
#include <carmen/cpp_abstractmap.h>

template<class CELL>
class GenericMap : public AbstractMap<CELL> {
 public:
  GenericMap();
  GenericMap(const MapConfig& cfg);
  GenericMap(const GenericMap<CELL>& src);
  virtual ~GenericMap();  

  virtual bool init(const MapConfig& cfg);  
  virtual const CELL& defaultCell() const;
  
  virtual CELL& getCell(int x, int y);
  virtual CELL& getCell(int x, int y) const;

 protected:
  CELL*  m_maplinear;
  CELL** m_map;
  CELL   m_defaultCell;
};

#include "cpp_genericmap.hxx"

#endif

