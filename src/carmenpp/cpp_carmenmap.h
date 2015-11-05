#ifndef CARMEN_CPP_CARMENMAP_H
#define CARMEN_CPP_CARMENMAP_H

#include <carmen/carmen.h>
#include <carmen/cpp_abstractmap.h>


class CarmenMap : public AbstractMap<double> {
 public:
  CarmenMap();
  CarmenMap(const CarmenMap& x);
  CarmenMap(const MapConfig& cfg);
  virtual ~CarmenMap();  
  
  void setMap(carmen_map_t* m);
  void copyMap(carmen_map_t* m); 

  carmen_inline operator carmen_map_t()  {return *m_map;}
  carmen_inline operator carmen_map_t() const {return *m_map;}

  carmen_inline operator carmen_map_t*()  {return m_map;}
  carmen_inline operator carmen_map_t*() const {return m_map;}

  void mapConfigUpdated(const Point& offset = Point(0.0, 0.0));

  virtual bool init(const MapConfig& cfg);
 
  virtual double& getCell(int x, int y) ;
  virtual double& getCell(int x, int y) const ;

  virtual const double& defaultCell() const;


/*   virtual void moveMap(int dx, int dy) = 0; */

 protected:
  carmen_map_t* m_map;
  double m_defaultCell;
};


#endif

