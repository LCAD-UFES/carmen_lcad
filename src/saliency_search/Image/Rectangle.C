/*!@file Image/Rectangle.C A basic rectangle class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Rectangle.C $
// $Id: Rectangle.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "Image/Rectangle.H"

#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "Util/log.H"

#include <sstream>

// ######################################################################
// converters for Rectangle
// ######################################################################

// ######################################################################
std::string convertToString(const Rectangle& val)
{
  if (val.isValid())
    {
      std::stringstream s;
      s<<val.left()<<','<<val.top()<<','<<val.rightO()<<','<<val.bottomO();
      return s.str();
    }
  // else...
  return "-1,-1,-1,-1";
}

// ######################################################################
void convertFromString(const std::string& str, Rectangle& val)
{
  std::stringstream s; int t = -1, b = -1, l = -1, r = -1;
  char c; s<<str; s>>l>>c>>t>>c>>r>>c>>b;
  if (s.fail())
    conversion_error::raise<Rectangle>(str);

  val = Rectangle::tlbrO(t, l, b, r);
}

// ######################################################################
Rectangle constrainRect(const Rectangle& in,
                        const Rectangle& bounds,
                        int minw, int maxw,
                        int minh, int maxh)
{
  ASSERT(minw >= 0);
  ASSERT(maxw >= minw);
  ASSERT(minh >= 0);
  ASSERT(maxh >= minh);

  Rectangle result = in.getOverlap(bounds);

  ASSERT(bounds.contains(result));

  minw = std::min(minw, bounds.width());
  maxw = std::min(maxw, bounds.width());

  minh = std::min(minh, bounds.height());
  maxh = std::min(maxh, bounds.height());

  //
  // grow/shrink the width to fit within minw,maxw
  //

  if (result.width() < minw)
    {
      const int diff = minw - result.width();
      result = Rectangle::tlbrO(result.top(),
                                result.left() - diff / 2,
                                result.bottomO(),
                                result.rightO() + diff - diff / 2);
      ASSERT(result.width() == minw);
    }

  else if (result.width() > maxw)
    {
      const int diff = result.width() - maxw;
      result = Rectangle::tlbrO(result.top(),
                                result.left() + diff / 2,
                                result.bottomO(),
                                result.rightO() - diff + diff / 2);
      ASSERT(result.width() == maxw);
    }

  //
  // shift leftward/rightward to stay within bounds (we may have gone
  // out of bounds if had to grow the width to stay wider than minw)
  //

  if (result.left() < 0)
    {
      result += Point2D<int>(-result.left(), 0);
      ASSERT(result.left() == 0);
    }
  else if (result.rightO() > bounds.width())
    {
      result -= Point2D<int>(result.rightO() - bounds.width(), 0);
      ASSERT(result.rightO() == bounds.width());
    }
  ASSERT(bounds.contains(result));

  //
  // grow/shrink the height to fit within minh,maxh
  //

  if (result.height() < minh)
    {
      const int diff = minh - result.height();
      result = Rectangle::tlbrO(result.top() - diff / 2,
                                result.left(),
                                result.bottomO() + diff - diff / 2,
                                result.rightO());
      ASSERT(result.height() == minh);
    }
  else if (result.height() > maxh)
    {
      const int diff = result.height() - maxh;
      result = Rectangle::tlbrO(result.top() + diff / 2,
                                result.left(),
                                result.bottomO() - diff + diff / 2,
                                result.rightO());
      ASSERT(result.height() == maxh);
    }

  //
  // shift upward/downward to stay within bounds (we may have gone out
  // of bounds if had to grow the height to stay taller than minh)
  //

  if (result.top() < 0)
    {
      result += Point2D<int>(0, -result.top());
      ASSERT(result.top() == 0);
    }
  else if (result.bottomO() > bounds.height())
    {
      result -= Point2D<int>(0, result.bottomO() - bounds.height());
      ASSERT(result.bottomO() == bounds.height());
    }
  ASSERT(bounds.contains(result));

  //
  // now verify that we met all of our postconditions:
  //

  ASSERT(result.width() >= minw);
  ASSERT(result.width() <= maxw);
  ASSERT(result.height() >= minh);
  ASSERT(result.height() <= maxh);
  ASSERT(bounds.contains(result));

  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
