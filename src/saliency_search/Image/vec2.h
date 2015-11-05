/** @file geom/vec2.h 2-D geometric vector/point */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Jan 28 12:54:13 1999
// commit: $Id: vec2.h 12962 2010-03-06 02:13:53Z irock $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/vec2.h $
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

#ifndef GROOVX_GEOM_VEC2_H_UTC20050626084023_DEFINED
#define GROOVX_GEOM_VEC2_H_UTC20050626084023_DEFINED

#include "Util/FastMathFunctions.H"
#include "Image/geom.h"

#include <cmath>

#include "rutz/debug.h"
GVX_DBG_REGISTER

namespace geom
{
  /// Gfx::vec2 is a 2-D vector class for representing 2-D points or distances.
  template<class V>
  class vec2
  {
  public:
    vec2()         : xx(),  yy()  {} // default is zero-init

    vec2(V x, V y) : xx(x), yy(y) {}

    template <class U>
    explicit vec2(const vec2<U>& other) : xx(V(other.x())), yy(V(other.y())) {}

    template <class U>
    vec2& operator=(const vec2<U>& other)
    { xx = other.x(); yy = other.y(); return *this; }

    static vec2 zeros() { return vec2(V(0), V(0)); }
    static vec2 ones()  { return vec2(V(1), V(1)); }

    V& x() { return xx; }
    V& y() { return yy; }

    const V& x() const { return xx; }
    const V& y() const { return yy; }

    vec2 abs() const
    { return vec2(xx > 0 ? xx : -xx, yy > 0 ? yy : -yy); }

    void set(V x, V y) { xx = x; yy = y; }

    bool operator==(const vec2<V>& b)
    { return x() == b.x() && y() == b.y(); }

    //
    // Polar coordinates
    //

    double length() const { return fastSqrt(xx*xx + yy*yy); }

    void set_length(double len)
    {
      const double r = length();
      if (r != 0.0)
        scale_by(len / r);
    }

    void set_polar_rad(double r, double theta)
    {
      xx = r * cos(theta);
      yy = r * sin(theta);
    }

    double theta_deg() const
    {
      return geom::rad2deg(atan2(yy, xx));
    }

    double theta_rad() const
    {
      return atan2(yy, xx);
    }


    void set_theta_deg(double degrees)
    {
      set_polar_rad(length(), geom::deg2rad(degrees));
    }

    void rotate_deg(double degrees)
    {
      // FIXME should use a real sin(),cos() rotation matrix here?
      degrees = geom::deg_n180_180(degrees);
      if (degrees == 0.0)
        {
          return;
        }
      else if (degrees == 90.0)
        {
          double old_x = xx;
          xx = -yy;
          yy = old_x;
        }
      else if (degrees == 180.0)
        {
          xx = -xx;
          yy = -yy;
        }
      else if (degrees == -90.0)
        {
          double old_x = xx;
          xx = yy;
          yy = -old_x;
        }
      else
        {
          set_theta_deg(theta_deg() + degrees);
        }
    }

    /// Result in radians.
    double angle_to(const vec2<V>& b) const
    {
      return rad_0_2pi(atan2(b.y() - y(), b.x() - x()));
    }

    double distance_to(const vec2<V>& b) const
    {
      const double dx = x() - b.x();
      const double dy = y() - b.y();
      return fastSqrt(dx*dx + dy*dy);
    }

    //
    // vec2-scalar math
    //

    template <class U>
    void scale_by(const U& factor) { xx *= factor; yy *= factor; }

    vec2 operator*(const V& factor) const
    { return vec2<V>(xx * factor, yy * factor); }

    vec2 operator/(const V& factor) const
    { return vec2<V>(xx / factor, yy / factor); }

    template <class U>
    vec2& operator*=(const U& factor) { scale_by(factor); return *this; }

    template <class U>
    vec2& operator/=(const U& factor) { scale_by(V(1)/factor); return *this; }


    //
    // vec2-vec2 math
    //

    vec2 operator+(const vec2<V>& rhs) const
    { return vec2<V>(xx + rhs.x(), yy + rhs.y()); }

    vec2 operator-(const vec2<V>& rhs) const
    { return vec2<V>(xx - rhs.x(), yy - rhs.y()); }


    template <class U>
    vec2 operator*(const vec2<U>& rhs) const
    { return vec2(V(x() * rhs.x()), V(y() * rhs.y())); }

    template <class U>
    vec2 operator/(const vec2<U>& rhs) const
    { return vec2(V(x() / rhs.x()), V(y() / rhs.y())); }


    template <class U>
    vec2& operator+=(const vec2<U>& rhs)
    { xx += V(rhs.x()); yy += V(rhs.y()); return *this; }

    template <class U>
    vec2& operator-=(const vec2<U>& rhs)
    { xx -= V(rhs.x()); yy -= V(rhs.y()); return *this; }


    template <class U>
    vec2& operator*=(const vec2<U>& factor)
    { xx *= factor.x(); yy *= factor.y(); return *this; }

    template <class U>
    vec2& operator/=(const vec2<U>& factor)
    { xx /= factor.x(); yy /= factor.y(); return *this; }

    void debug_dump() const throw()
    {
      dbg_eval(0, xx);
      dbg_eval_nl(0, yy);
    }

  private:
    V xx;
    V yy;
  };

  template <class U>
  vec2<U> normal_to(const vec2<U>& a)
  { return vec2<U>(-a.y(), a.x()); }

  template <class U>
  vec2<U> make_unit_length(const vec2<U>& a)
  {
    const double r = a.length();
    if (r == 0.0) return a;
    return vec2<U>(a.x()/r, a.y()/r);
  }

  typedef vec2<int> vec2i;
  typedef vec2<float> vec2f;
  typedef vec2<double> vec2d;

} // end namespace Gfx

static const char __attribute__((used)) vcid_groovx_geom_vec2_h_utc20050626084023[] = "$Id: vec2.h 12962 2010-03-06 02:13:53Z irock $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/vec2.h $";
#endif // !GROOVX_GEOM_VEC2_H_UTC20050626084023_DEFINED
