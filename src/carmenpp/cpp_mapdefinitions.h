#ifndef CARMEN_CPP_MAP_DEFINITIONS_H
#define CARMEN_CPP_MAP_DEFINITIONS_H

#include <carmen/cpp_genericmap.h>


class CharCell {
public:
  CharCell(char v=-1) { set(v); }
  CharCell(const CharCell& x) { *this = x; }
  void set(char v) { val = v; }
  
  carmen_inline operator char() const { return val;}
  carmen_inline CharCell& operator= (const char& v)  { val = v; return *this;}
  
  char val;
};


class IntCell {
public:
  IntCell(int v=-1) { set(v); }
  IntCell(const IntCell& x) { *this = x; }
  void set(int v) { val = v; }
  
  carmen_inline operator int() const { return val;}
  carmen_inline IntCell& operator= (const int& v)  { val = v; return *this;}
  
  int val;
};

class FloatCell {
public:
  FloatCell(float v = -1.0) { set(v); }
  FloatCell(const FloatCell& x) { *this = x; }
  void set(float v) { val = v; }

  carmen_inline operator float() const { return val;}
  carmen_inline FloatCell& operator= (const float& v)  { val = v; return *this;}
  
  float val;
};

class DoubleCell {
public:
  DoubleCell(double v = -1.0) { set(v); }
  DoubleCell(const DoubleCell& x) { *this = x; }
  void set(double v) { val = v; }

  carmen_inline operator double() const { return val;}
  carmen_inline DoubleCell& operator= (const double& v)  { val = v; return *this;}
  
  double val;
};

class RefProbCell {
public:
  RefProbCell(int hits=0, int misses=0) { set(hits, misses); }
  RefProbCell(const RefProbCell& x) {  *this = x;  }
  
  void set(int hits, int misses) { 
    this->hits = hits;  
    this->misses = misses; 
    this->val = -1; 
    if (hits+misses > 0) {
      val = ((float)hits)/((float) (hits+misses));
    }
    updated = false;
  }
	   
  void hit() { 
    hits++;
    updated = true;
  }

  void miss() { 
    misses++;
    updated = true;
    val = ((float)hits)/((float) (hits+misses));
  }

  carmen_inline operator float() {  
    if (updated) {
      val = ((float)hits)/((float) (hits+misses));
      updated = false;
    }
    return val;  
  }
  carmen_inline operator double() {      
    if (updated) {
      val = ((float)hits)/((float) (hits+misses));
      updated = false;
    }
    return val;  
  }
  
  int hits;
  int misses;
  float val;
  bool updated;
};

typedef GenericMap<CharCell>     CharMap;
typedef GenericMap<IntCell>      IntMap;
typedef GenericMap<FloatCell>    FloatMap;
typedef GenericMap<DoubleCell>   DoubleMap;
typedef GenericMap<RefProbCell>  RefProbMap;

#endif
