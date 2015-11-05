 #ifndef CARMEN_CPP_ABSTRACT_MAP_H
#define CARMEN_CPP_ABSTRACT_MAP_H

#include <carmen/carmen.h>
#include <carmen/cpp_point.h>
#include <carmen/cpp_mapconfig.h>

template <class CELL>
class AbstractMap {
 public:
  AbstractMap();
  AbstractMap(const AbstractMap& x);
  AbstractMap(const MapConfig& cfg);
  virtual ~AbstractMap();  


  virtual CELL& getCell(int x, int y) = 0;
  virtual CELL& getCell(int x, int y) const = 0;
  

  // Vitual function to be defined in subclass
  virtual bool init(const MapConfig& cfg) = 0;
  virtual const CELL& defaultCell() const = 0;

  // non-virtual function for the map handling
  bool init(int sizeX, int sizeY, double res=1.0, Point offset = Point(0.0, 0.0) ); 
  bool init(double xfrom, double xto, double yfrom, double yto, double res);
  bool initIfSmaller(int sizeX, int sizeY, double res=1.0, Point offset = Point(0.0, 0.0) ); 

  const MapConfig& getConfig() const;

  carmen_inline IntPoint getMin() const;
  carmen_inline IntPoint getMax() const;
  carmen_inline int getMapSizeX() const;
  carmen_inline int getMapSizeY() const;

  carmen_inline void setOffset(const Point& p);
  carmen_inline Point getOffset() const;

  carmen_inline void setResolution(double res);
  carmen_inline double getResolution() const;
  
  carmen_inline Point map2world_double(const Point& p) const;
  carmen_inline Point world2map_double(const Point& p) const;
  carmen_inline Point map2world(const Point& p) const;
  carmen_inline IntPoint world2map(const Point& p) const;
  carmen_inline Point map2world(const IntPoint& p) const;
  carmen_inline Point map2world(int x, int y) const;
  carmen_inline IntPoint world2map(double x, double y) const;
  
  carmen_inline bool isInside(const Point& p) const;
  carmen_inline bool isInside(const IntPoint& p) const;
  carmen_inline bool isInside(int x, int y) const;
  carmen_inline bool isInside(double x, double y) const;
  
  carmen_inline IntPoint minInside(const IntPoint& p) const;
  carmen_inline IntPoint maxInside(const IntPoint& p) const;
  carmen_inline IntPoint putInside(const IntPoint& p) const;

  carmen_inline CELL& cell(const IntPoint& p);  
  carmen_inline CELL& cell(const IntPoint& p) const;

  carmen_inline CELL& cell(int x, int y);
  carmen_inline CELL& cell(int x, int y) const;

  carmen_inline CELL& cell(double x, double y);
  carmen_inline CELL& cell(double x, double y) const;
  
  carmen_inline CELL& cell(const Point& p) const ;
  carmen_inline CELL& cell(const Point& p);

  void copy(const AbstractMap<CELL>& src);
  void copy(const AbstractMap<CELL>& src, const IntPoint& relative_offset);
  void moveMap(int dx, int dy);

  void resetCells();
  void resetCells(const CELL& val);
  void resetCells(const CELL& val, const IntPoint& from, const IntPoint& to);
 public:
  MapConfig m_cfg;
};

#include "cpp_abstractmap.hxx"

#endif

