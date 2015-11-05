/** @file rutz/rand.h random-number generation */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Jun 25 14:09:24 1999
// commit: $Id: rand.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/rand.h $
//
// The random number generator classes here are taken from _The C++
// Programming Language_, 3rd ed., by Bjarne Stroustrup.
//
// --------------------------------------------------------------------
//
// This file is part of GroovX.
//   [http://ilab.usc.edu/rjpeters/groovx/]
//
// GroovX is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// GroovX is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GroovX; if not, write to the Free Software Foundation,
// Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
///////////////////////////////////////////////////////////////////////

#ifndef GROOVX_RUTZ_RAND_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_RAND_H_UTC20050626084020_DEFINED

#include <cstdlib> // for rand()

namespace rutz
{
  class urand;
  class urand_irange;
  class urand_frange;

  template <class T>
  inline T rand_range(const T& min, const T& max)
  {
    return T( (double(rand()) / (double(RAND_MAX)+1.0)) * (max-min) + min );
  }

  /// A hook that allows various code to start from a predictable seed.
  /** This allows code in disparate locations to all be triggered by
      the same random seed. This is useful in allowing for predictable
      and repeatable execution sequences e.g. in a testing
      context. Initial value is 0. The most sensible use case involves
      setting this value just once, at or near the beginning of
      program execution. */
  extern unsigned long default_rand_seed;
}

/// Uniform random distribution
class rutz::urand
{
private:
  unsigned long randx;

  int abs(int x) { return x & 0x7fffffff; }
  static double max() { return 2147483648.0 /* == 0x80000000u */; }

  int idraw() { return randx = randx * 0x41c64e6d + 0x3039; }

public:
  urand(long s = 0) : randx(s) {}
  void seed(long s) { randx = s; }

  /// Uniform random distribution in the interval [0.0:1.0[
  double fdraw()
  {
    return abs(idraw())/max();
  }

  /// Uniform random distribution in the interval [min:max[
  double fdraw_range(double min, double max)
  {
    return min + fdraw() * (max-min);
  }

  /// Uniform random distribution between true:false
  bool booldraw()
  {
    return fdraw() < 0.5;
  }

  /// Uniform random distribution in the interval [0:n[
  int idraw(int n)
  {
    int r = int(n*fdraw()); return (r==n) ? n-1 : r;
  }

  /// Uniform random distribution in the interval [lo:hi[
  int idraw_range(int lo, int hi)
  {
    return lo + idraw(hi - lo);
  }

  /// Uniform random distribution in the interval [0:n[
  int operator()(int n) { return idraw(n); }
};

/// uniform distribution over a specified integer range
class rutz::urand_irange
{
private:
  rutz::urand  m_generator;
  int          m_min;
  int          m_max;

public:
  /// Construct with a given min/max range and an initial seed (default 0).
  urand_irange(int lo, int hi, long s = 0)
    : m_generator(s), m_min(lo), m_max(hi)
  {}

  int draw()       { return m_generator.idraw_range(m_min, m_max); }
  int operator()() { return draw(); }
};

/// uniform distribution over a specified floating-point range
class rutz::urand_frange
{
private:
  rutz::urand  m_generator;
  double       m_min;
  double       m_max;

public:
  /// Construct with a given min/max range and an initial seed (default 0).
  urand_frange(double lo, double hi, long s = 0)
    : m_generator(s), m_min(lo), m_max(hi)
  {}

  /// Construct with a range [0.0:1.0[ and an initial seed (default 0).
  urand_frange(long s = 0)
    : m_generator(s), m_min(0.0), m_max(1.0)
  {}

  double draw()       { return m_generator.fdraw_range(m_min, m_max); }
  double operator()() { return draw(); }
};

static const char __attribute__((used)) vcid_groovx_rutz_rand_h_utc20050626084020[] = "$Id: rand.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/rand.h $";
#endif // !GROOVX_RUTZ_RAND_H_UTC20050626084020_DEFINED
