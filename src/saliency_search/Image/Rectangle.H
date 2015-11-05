/*!@file Image/Rectangle.H A basic rectangle class */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Rectangle.H $
// $Id: Rectangle.H 13892 2010-09-08 16:49:40Z lior $
//

#ifndef RECTANGLE_H_DEFINED
#define RECTANGLE_H_DEFINED

#include "Image/Dims.H"
#include "Image/Point2D.H"
#include "Util/Assert.H"

#include <algorithm>
#include <string> // for string conversions

//! A basic rectangle class
/*! This is a shorthand class to represent a 2D rectangle, as defined
    by integer top, left, bottom and right coordinates.  The goal of
    this object is not to have to check if left < right, etc...  for
    each function using a Rectangle. The object checks for that at its
    creation, and is always internally coherent.  Image convention:
    top < bottom, left < right.

    There is often some ambiguity about how to represent the
    left-right and top-bottom ranges involved in a rectangle. Imagine
    a pixel grid like this:

    +---+---+---+---+
    |   |   |   |   |
    |   |   |   |   |
    +---+---+---+---+
    |   |   |   |   |
    |   |   |   |   |
    +---+---+---+---+
    |   |   |   |   |
    |   |   |   |   |
    +---+---+---+---+
    |   |   |   |   |
    |   |   |   |   |
    +---+---+---+---+


    If we want a Rectangle that covers this whole 4x4 grid, how should
    we represent the coordinates? Clearly the upper left corner is
    (0,0), but what about the bottom right corner? One answer would be
    that the lower right corner is (4,4), in which we imagine the
    coordinate numbers falling between the cracks of the pixels:

    . 0...1...2...3...4
    0 +---+---+---+---+
    . |   |   |   |   |
    . |   |   |   |   |
    1 +---+---+---+---+
    . |   |   |   |   |
    . |   |   |   |   |
    2 +---+---+---+---+
    . |   |   |   |   |
    . |   |   |   |   |
    3 +---+---+---+---+
    . |   |   |   |   |
    . |   |   |   |   |
    4 +---+---+---+---+

    Another answer is that the lower right corner should be labeled as
    (3,3), if we envision the coordinate numbers labeling the pixel
    centers, and this approach is more natural if we want to draw a
    line just inside the border of the rectangle as shown by the '*'
    here:

    . ..0...1...2...3..
    . +---+---+---+---+
    0 |***|***|***|***|
    . |***|***|***|***|
    . +---+---+---+---+
    1 |***|   |   |***|
    . |***|   |   |***|
    . +---+---+---+---+
    2 |***|   |   |***|
    . |***|   |   |***|
    . +---+---+---+---+
    3 |***|***|***|***|
    . |***|***|***|***|
    . +---+---+---+---+

    In the Rectangle class we support both approaches; the (4,4)
    coords are called 'outer' coordinates and corresponding functions
    have a 'O' suffix (bottomO(), right(), and tlbrO()), while the
    (3,3) coords are called 'inner' coordinates and corresponding
    functions have an 'I' suffix (bottomI(), rightI(), and tlbrI()).

    The internal representation of Rectangle is based on outer
    coordinates, but this has no effect on users of the class who are
    free to user whichever of inner or outer coords are more
    convenient for their purposes.

    Outer coordinates are more natural for uniformly scaling a
    rectangle up or down by a factor. For example, if we want to scale
    the 4x4 rect up by a factor of 2 giving an 8x8 rect, then in outer
    coords we just do newright=right*2=4*2=8 and
    newbottom=bottom*2=4*2=8. But with inner coords we can't just
    multiply by 2: the inner bottom-right coords are (3,3), and if we
    scale those by 2 we get (6,6) which represents a 7x7 rectangle
    rather than the correct 8x8 result.

    HISTORICAL NOTE (2007-Mar-20): The Rectangle class previously used
    only inner coordinates both for its implementation and in its
    public interface. This lead to clunky code in some places where
    many -1 or +1 offsets were needed to handle rectangle coordinates.
*/

class Rectangle {

  //! Private constructor; use the tlbrO() pseudo-constructor if you want to call this publicly
  inline Rectangle(int tt, int ll, int bb1, int rr1);

public:
  // ############################################################
  // ##### Constructors & Destructors:
  // ############################################################

  //! Build from 4 coordinates (top, left, inner-bottom, inner-right)
  static inline Rectangle tlbrI(int tt, int ll, int bb, int rr);

  //! Build from 4 coordinates (top, left, outer-bottom, outer-right)
  static inline Rectangle tlbrO(int tt, int ll, int bb, int rr);

  //! Build from center loc and dims
  static inline Rectangle centerDims(const Point2D<int>& center, const Dims& dims);

  //! Build from a top-left corner and some dims
  inline Rectangle(const Point2D<int>& topleft, const Dims& dims);

  //! Copy constructor
  inline Rectangle(const Rectangle& rect);

  //! Uninitialized constructor (useful for arrays).
  inline Rectangle();

  // Default assignment operator ok

  // ############################################################
  // ##### Access functions:
  // ############################################################

  //! get overlap between this rectangle and passed in rectangle
  inline Rectangle getOverlap(const Rectangle& r2) const;

  //! compute overlap as area of intersection / area of union
  inline double getOverlapRatio(const Rectangle& r2) const;

  //! check if point p is within rectangle
  /*! CAUTION: boundaries are included. */
  inline bool contains(const Point2D<int>& p) const;

  //! check if this rect contains another
  /*! The boundaries are allowed to coincide, such that by this
      definition a rectangle "contains" itself. */
  inline bool contains(const Rectangle& r) const;

  inline int top() const;     //!< get (inner) top coordinate
  inline int bottomI() const; //!< get inner bottom coordinate (=top+height-1)
  inline int bottomO() const; //!< get outer bottom coordinate (=top+height)
  inline int left() const;    //!< get (inner) left coordinate
  inline int rightI() const;  //!< get inner right coordinate (=left+width-1)
  inline int rightO() const;  //!< get outer right coordinate (=left+width)

  inline Point2D<int> topLeft() const;     //!< Get (outer) top left corner point
  inline Point2D<int> topRight() const;    //!< Get (outer) top right corner point
  inline Point2D<int> bottomLeft() const;  //!< Get (outer) bottom left corner point
  inline Point2D<int> bottomRight() const; //!< Get (outer) bottom right corner point

  //! Get the rectangle's width, height
  inline Dims dims() const;

  //! Access some metric info
  inline int width() const;
  //! Access some metric info
  inline int height() const;
  //! Access some metric info
  inline int area() const;
  //! Access some metric info
  inline float aspect() const;

  //!The center of the rectangle
  inline Point2D<int> center() const;

  //! whether the rectangle is valid
  inline bool isValid() const;

private:
  inline void tlbrInitOuter(int tt, int ll, int bb1, int rr1);

  bool valid;       //!< True if Rectangle contains something coherent
  int t, b1, l, r1;   //!< top, bottom, left, right
};


// ######################################################################
// Rectangle FREE FUNCTIONS:
// ######################################################################

// ######################################################################
//! Rectangle overload: format is "<int>,<int>,<int>,<int>"
/*! Format corresponds to "xtopleft,ytopleft,xbottomright,ybottomright" */
std::string convertToString(const Rectangle& val);

// ######################################################################
//! Rectangle overload: format is "<int>,<int>,<int>,<int>"
void convertFromString(const std::string& str, Rectangle& val);


// ######################################################################
//! Return a new rectangle that is a constrained version of the input.
/*! @param in The original rectangle on which to base the constrained
    output.

    @param bounds The output rectangle will be constrained to be
    contained within these bounds.

    @param minw The minimum width for the output rectangle. Caller
    must ensure minw>=0. If the output rectangle would naturally be
    narrower than this width, then it will be widened to the smaller
    of minw and bounds.width().

    @param maxw The maximum width for the output rectangle. Caller
    must ensure maxw>=minw. If the output rectangle would naturally be
    wider than this width, then it will be narrowed to the smaller of
    maxw and bounds.width().

    @param minh The minimum height for the output rectangle. Caller
    must ensure minh>=0. If the output rectangle would naturally be
    shorter than this height, then it will be lengthened to the
    smaller of minh and bounds.height().

    @param maxh The maximum height for the output rectangle. Caller
    must ensure maxh>=minh. If the output rectangle would naturally be
    taller than this height, then it will be shortened to the smaller
    of maxh and bounds.height().

    @return The result is guaranteed to be such that the following
    conditions are all true:
    \code
    bounds.contains(result);
    result.width() >= std::min(minw, bounds.width());
    result.width() <= std::min(maxw, bounds.width());
    result.height() >= std::min(minh, bounds.height());
    result.height() <= std::min(maxh, bounds.height());
    \endcode
*/
Rectangle constrainRect(const Rectangle& in,
                        const Rectangle& bounds,
                        int minw, int maxw,
                        int minh, int maxh);


// ######################################################################
//! Like the main constrainRect(), but use the same limits for width as for height
inline Rectangle constrainRect(const Rectangle& in,
                               const Rectangle& bounds,
                               int minsz, int maxsz)
{
  return constrainRect(in, bounds, minsz, maxsz, minsz, maxsz);
}

// ######################################################################
inline bool operator==(const Rectangle& r1, const Rectangle& r2)
{
  if (!r1.isValid() || !r2.isValid()) return r1.isValid() == r2.isValid();

  return (r1.top() == r2.top() &&
          r1.bottomO() == r2.bottomO() &&
          r1.left() == r2.left() &&
          r1.rightO() == r2.rightO());
}

// ######################################################################
inline Rectangle operator*(const Rectangle& r, const double s)
{
  if (!r.isValid())
    return Rectangle();
  else
    return Rectangle::tlbrO(int(r.top() * s),
                            int(r.left() * s),
                            int(r.bottomO() * s),
                            int(r.rightO() * s));
}

// ######################################################################
inline Rectangle operator*(const Rectangle& r, const int s)
{
  if (!r.isValid())
    return Rectangle();
  else
    return Rectangle::tlbrO(r.top() * s,
                            r.left() * s,
                            r.bottomO() * s,
                            r.rightO() * s);
}

// ######################################################################
inline Rectangle operator/(const Rectangle& r, const int div)
{
  if (!r.isValid() || div == 0)
    return Rectangle();
  else
    return Rectangle::tlbrO(r.top() / div,
                            r.left() / div,
                            r.bottomO() / div,
                            r.rightO() / div);
}

// ######################################################################
inline Rectangle operator+(const Rectangle& r, const Point2D<int>& p)
{
  return Rectangle::tlbrO(r.top() + p.j,
                          r.left() + p.i,
                          r.bottomO() + p.j,
                          r.rightO() + p.i);
}

// ######################################################################
inline Rectangle& operator+=(Rectangle& r, const Point2D<int>& p)
{
  return (r = r + p);
}

// ######################################################################
inline Rectangle operator-(const Rectangle& r, const Point2D<int>& p)
{
  return Rectangle::tlbrO(r.top() - p.j,
                          r.left() - p.i,
                          r.bottomO() - p.j,
                          r.rightO() - p.i);
}

// ######################################################################
inline Rectangle& operator-=(Rectangle& r, const Point2D<int>& p)
{
  return (r = r - p);
}

// ######################################################################
// Rectangle INLINE MEMBER FUNCTIONS:
// ######################################################################

// ######################################################################
inline Rectangle::Rectangle(int tt, int ll, int bb1, int rr1)
{ tlbrInitOuter(tt, ll, bb1, rr1); }

// ######################################################################
inline Rectangle Rectangle::tlbrI(int tt, int ll, int bb, int rr)
{ return Rectangle(tt, ll, bb+1, rr+1); }

// ######################################################################
inline Rectangle Rectangle::tlbrO(int tt, int ll, int bb, int rr)
{ return Rectangle(tt, ll, bb, rr); }

// ######################################################################
inline Rectangle Rectangle::centerDims(const Point2D<int>& center, const Dims& d)
{ return Rectangle(Point2D<int>(center.i - d.w()/2, center.j - d.h()/2), d); }

// ######################################################################
inline Rectangle::Rectangle(const Point2D<int>& topleft, const Dims& d)
  : valid(true),
    t(topleft.j), b1(topleft.j+d.h()),
    l(topleft.i), r1(topleft.i+d.w())
{}

// ######################################################################
inline Rectangle::Rectangle(const Rectangle& rect)
  : valid(rect.valid), t(rect.t), b1(rect.b1), l(rect.l), r1(rect.r1)
{}

// ######################################################################
inline Rectangle::Rectangle()
  : valid(false), t(-1), b1(-1), l(-1), r1(-1)
{}

// ######################################################################
inline void Rectangle::tlbrInitOuter(int tt, int ll, int bb1, int rr1)
{
  if (bb1 - tt < 0) { t = bb1; b1 = tt; } else { t = tt; b1 = bb1; }
  if (rr1 - ll < 0) { l = rr1; r1 = ll; } else { l = ll; r1 = rr1; }
  valid = true;
}

// ######################################################################
inline Rectangle Rectangle::getOverlap(const Rectangle& r2) const
{
  if (!this->isValid() || !r2.isValid()) return Rectangle();

  const int le = std::max(l, r2.left());
  const int ri = std::min(r1, r2.rightO());
  const int to = std::max(t, r2.top());
  const int bo = std::min(b1, r2.bottomO());

  if (le >= ri || to >= bo) return Rectangle();
  else return Rectangle::tlbrO(to, le, bo, ri);
}

// ######################################################################
inline double Rectangle::getOverlapRatio(const Rectangle& r2) const
{

  Rectangle ovR = getOverlap(r2);

  double ov = 0;
  if (ovR.isValid())
  {
    //compute overlap as area of intersection / area of union
    double ua = (area()+1) + (r2.area()+1) - ovR.area();
    ov = (double)ovR.area()/ua;
  }
 
  return ov;
}

// ######################################################################
inline int Rectangle::area() const
{
  return width() * height();
}

// ######################################################################
inline float Rectangle::aspect() const
{
  return (float)width() / (float)height();
}

// ######################################################################
inline Point2D<int> Rectangle::center() const
{
  return Point2D<int>(l+(r1-l)/2, t+(b1-t)/2);
  //int t, b1, l, r1;   //!< top, bottom, left, right
}

// ######################################################################
inline bool Rectangle::contains(const Point2D<int>& p) const
{
  return (p.i >= l && p.i < r1 && p.j >= t && p.j < b1);
}

// ######################################################################
inline bool Rectangle::contains(const Rectangle& r) const
{
  return (   r.l >= this->l
          && r.t >= this->t
          && r.r1 <= this->r1
          && r.b1 <= this->b1);
}

// ######################################################################
inline int Rectangle::top() const
{ ASSERT(valid); return t; }

inline int Rectangle::bottomI() const
{ ASSERT(valid); return b1-1; }

inline int Rectangle::bottomO() const
{ ASSERT(valid); return b1; }

inline int Rectangle::left() const
{ ASSERT(valid); return l; }

inline int Rectangle::rightI() const
{ ASSERT(valid); return r1-1; }

inline int Rectangle::rightO() const
{ ASSERT(valid); return r1; }

inline Point2D<int> Rectangle::topLeft() const
{ ASSERT(valid); return Point2D<int>(l, t); }

inline Point2D<int> Rectangle::topRight() const
{ ASSERT(valid); return Point2D<int>(r1, t); }

inline Point2D<int> Rectangle::bottomLeft() const
{ ASSERT(valid); return Point2D<int>(l, b1); }

inline Point2D<int> Rectangle::bottomRight() const
{ ASSERT(valid); return Point2D<int>(r1, b1); }

inline Dims Rectangle::dims() const
{
  ASSERT(valid);
  return Dims(r1 - l, b1 - t); // *** CAUTION: BOUNDARIES ARE INCLUDED
}

inline int Rectangle::width() const
{ ASSERT(valid); return r1 - l; } // *** CAUTION: BOUNDARIES ARE INCLUDED

inline int Rectangle::height() const
{ ASSERT(valid); return b1 - t; } // *** CAUTION: BOUNDARIES ARE INCLUDED

inline bool Rectangle::isValid() const
{ return valid; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
