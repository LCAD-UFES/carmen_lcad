
template<class CELL>
AbstractMap<CELL>::AbstractMap() : m_cfg() {
}

template<class CELL>
AbstractMap<CELL>::AbstractMap(const AbstractMap<CELL>&  src) : m_cfg(src.getConfig()) {
}

template<class CELL>
AbstractMap<CELL>::AbstractMap(const MapConfig& cfg) : m_cfg(cfg) {
}

template<class CELL>
AbstractMap<CELL>::~AbstractMap() {
}

template<class CELL>
bool AbstractMap<CELL>::init(int sizeX
			     , int sizeY
			     , double res
			     , Point offset
			     ) {
   MapConfig cfg(sizeX, sizeY, res, offset);
   return init(cfg);
}

template<class CELL>
bool AbstractMap<CELL>::initIfSmaller(int sizeX, int sizeY, double res, Point offset) {
  if (!m_cfg.isValid() || m_cfg.m_sizeX < sizeX || m_cfg.m_sizeY < sizeY) {
    MapConfig cfg(sizeX, sizeY, res, offset);
    return init(cfg);
  }
  return false;
}

template<class CELL>
bool AbstractMap<CELL>::init(double xfrom, double xto, double yfrom, double yto, double res) {
  int sizeX = (int) ceilf( (xto-xfrom)/res );
  int sizeY = (int) ceilf( (yto-yfrom)/res );
  Point offset(xfrom, yfrom);
  return init(sizeX, sizeY, res, offset);
}

template<class CELL>
const MapConfig& AbstractMap<CELL>::getConfig() const {
  return m_cfg;
}

template<class CELL>
IntPoint AbstractMap<CELL>::world2map(const Point& p) const {
  double res = getResolution();
  IntPoint ip( (int)( (p.x-m_cfg.m_offset.x)/res), (int)((p.y-m_cfg.m_offset.y)/res) );
  return ip;
}

template<class CELL>
carmen_inline Point AbstractMap<CELL>::map2world(const IntPoint& p) const {
  double res = getResolution();
  Point wp( m_cfg.m_offset.x + res * ((double)p.x), 
	    m_cfg.m_offset.y + res * ((double)p.y) );
  return wp;
}

template<class CELL>
carmen_inline Point AbstractMap<CELL>::world2map_double(const Point& p) const {
  double res = getResolution();
  Point ip((p.x-m_cfg.m_offset.x)/res, (p.y-m_cfg.m_offset.y)/res );
  return ip;
}

template<class CELL>
carmen_inline Point AbstractMap<CELL>::map2world_double(const Point& p) const {
  double res = getResolution();
  Point mp( m_cfg.m_offset.x + res * p.x, 
	    m_cfg.m_offset.y + res * p.y);
  return mp;
}

template<class CELL>
carmen_inline Point AbstractMap<CELL>::map2world(const Point& p) const {
  double res = getResolution();
  Point wp( m_cfg.m_offset.x + res * p.x, 
	    m_cfg.m_offset.y + res * p.y);
  return wp;
}

template<class CELL>
carmen_inline Point AbstractMap<CELL>::map2world(int x, int y) const {
  return map2world(IntPoint(x,y));
}

template<class CELL>
carmen_inline IntPoint AbstractMap<CELL>::world2map(double x, double y) const {
  return world2map(Point(x,y));
}


template<class CELL>
carmen_inline bool AbstractMap<CELL>::isInside(const Point& p)  const {
  return isInside (world2map(p));
}


template<class CELL>
carmen_inline bool AbstractMap<CELL>::isInside(int x, int y)  const {
  return isInside(IntPoint(x,y));
}


template<class CELL>
carmen_inline bool AbstractMap<CELL>::isInside(double x, double y)  const {
  return isInside(Point(x,y));
}


template<class CELL>
carmen_inline bool AbstractMap<CELL>::isInside(const IntPoint& p) const {
  if (p.x >= 0 && p.x < getMapSizeX() && p.y >= 0 && p.y < getMapSizeY())
    return true;
  return false;
};

template<class CELL>
carmen_inline void AbstractMap<CELL>::setResolution(double res){
  m_cfg.m_res = res;
};

template<class CELL>
carmen_inline void AbstractMap<CELL>::setOffset(const Point& p){
  m_cfg.m_offset=p;
};


template<class CELL>
carmen_inline Point AbstractMap<CELL>::getOffset() const {
  return m_cfg.m_offset;
};


template<class CELL>
carmen_inline double AbstractMap<CELL>::getResolution() const {
  return m_cfg.m_res;
}

template<class CELL>
carmen_inline IntPoint AbstractMap<CELL>::getMin() const{
    return IntPoint(0,0);
}

template<class CELL>
carmen_inline IntPoint AbstractMap<CELL>::getMax() const{
    return IntPoint(getMapSizeX(), getMapSizeY());
}

template<class CELL>
carmen_inline int AbstractMap<CELL>::getMapSizeX() const {
  return m_cfg.m_sizeX;
}

template<class CELL>
carmen_inline int AbstractMap<CELL>::getMapSizeY() const {
  return m_cfg.m_sizeY;
}

template<class CELL>
carmen_inline IntPoint AbstractMap<CELL>::minInside(const IntPoint& p) const {
  
  return  IntPoint( (p.x<0)?0:p.x ,
		    (p.y<0)?0:p.y );
}


template<class CELL>
carmen_inline IntPoint AbstractMap<CELL>::maxInside(const IntPoint& p) const {
  return  IntPoint( (m_cfg.m_sizeX-1<p.x)?m_cfg.m_sizeX-1:p.x , 
		    (m_cfg.m_sizeY-1<p.y)?m_cfg.m_sizeY-1:p.y );
}


template<class CELL>
carmen_inline IntPoint AbstractMap<CELL>::putInside(const IntPoint& p) const {
  
  IntPoint pnew = p;
  
  if (pnew.x < 0)
    pnew.x = 0;
  else if (pnew.x >= m_cfg.m_sizeX)
    pnew.x = m_cfg.m_sizeX-1;

  if (pnew.y < 0)
    pnew.y = 0;
  else if (pnew.y >= m_cfg.m_sizeY)
    pnew.y = m_cfg.m_sizeY-1;

  return  pnew;		
}


template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(const IntPoint& p) const {
  return getCell(p.x,p.y);
}

template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(const IntPoint& p) {
  return getCell(p.x,p.y);
}

template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(int x, int y)  {
  return getCell(x,y);
}

template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(int x, int y) const {
  return getCell(x,y);
}

template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(double x, double y)  {
  return getCell(x,y);
}

template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(double x, double y) const {
  return getCell(x,y);
}

template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(const Point& p) const {
  return cell(world2map(p));
}

template<class CELL>
carmen_inline CELL& AbstractMap<CELL>::cell(const Point& p) {
  return cell(world2map(p));
}


template<class CELL>
void AbstractMap<CELL>::copy(const AbstractMap<CELL>& src) {
  
  IntPoint from_dest = src.getMin();
  IntPoint to_dest   = src.getMax();
  
  for (int x=from_dest.x; x < to_dest.x; x++) {
    for (int y=from_dest.y; y < to_dest.y; y++) {
      cell(x,y) = src.cell(x,y);
    }
  }
}

template<class CELL>
void AbstractMap<CELL>::copy(const AbstractMap<CELL>& src, const IntPoint& relative_offset) {
  
  IntPoint from_src(0,0);
  IntPoint from_dest = relative_offset;
  IntPoint to_dest   = from_dest + src.getMax();

  if (from_dest.x < 0) {
    from_src.x  = -relative_offset.x;
    from_dest.x = 0;
  }
  if (from_dest.y < 0) {
    from_src.y  = -relative_offset.y;
    from_dest.y = 0;
  }
  
  if (to_dest.x >= getMapSizeX()) {
    to_dest.x = getMapSizeX()-1;
  }
  if (to_dest.y >= getMapSizeY()) {
    to_dest.y = getMapSizeY()-1;
  }

  for (int x=from_dest.x; x < to_dest.x; x++) {
    for (int y=from_dest.y; y < to_dest.y; y++) {
      cell(x,y) = src.cell(x+from_src.x-from_dest.x,y+from_src.y-from_dest.y);
    }
  }
}

template<class CELL>
void AbstractMap<CELL>::moveMap(int dx, int dy) {


  CELL* tmp = new CELL[m_cfg.m_sizeX*m_cfg.m_sizeY];
  carmen_test_alloc(tmp);
  for (int x=0; x < m_cfg.m_sizeX; x++) {
    for (int y=0; y < m_cfg.m_sizeY; y++) {
      tmp[x*m_cfg.m_sizeY+y] = cell(x,y);
      cell(x,y) = defaultCell();
    }
  }
  
  int xfrom = carmen_imax(0, -dx);
  int yfrom = carmen_imax(0, -dy);
  
  int xto = carmen_imin(m_cfg.m_sizeX, m_cfg.m_sizeX - dx);
  int yto = carmen_imin(m_cfg.m_sizeY, m_cfg.m_sizeY - dy);
  
  for (int x = xfrom; x < xto; x++ ){
    for (int y = yfrom; y < yto; y++ ){
      cell(x,y) = tmp[(x+dx)*m_cfg.m_sizeY+(y+dy)];
    } 
  } 
  double res = getResolution();
  m_cfg.m_offset.x += res * ((double) dx);
  m_cfg.m_offset.y += res * ((double) dy);

  delete [] tmp;
}

template<class CELL>
void AbstractMap<CELL>::resetCells() {
  resetCells(defaultCell());
}

template<class CELL>
void AbstractMap<CELL>::resetCells(const CELL& val) {
  resetCells( val, getMin(), getMax() );
}

template<class CELL>
void AbstractMap<CELL>::resetCells(const CELL& val, const IntPoint& from, const IntPoint& to) {
  for (int x=from.x; x< to.x; x++)
    for (int y=from.y; y< to.y; y++)
      cell(IntPoint(x,y)) = val;
}

