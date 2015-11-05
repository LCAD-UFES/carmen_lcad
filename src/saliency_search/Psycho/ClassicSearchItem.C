/*!@file Psycho/ClassicSearchItem.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/ClassicSearchItem.C $
// $Id: ClassicSearchItem.C 9079 2007-12-12 00:54:09Z rjpeters $
//

#ifndef PSYCHO_CLASSICSEARCHITEM_C_DEFINED
#define PSYCHO_CLASSICSEARCHITEM_C_DEFINED

#include "Psycho/ClassicSearchItem.H"

#include "Image/Image.H"
#include "Image/ShapeOps.H"

#include <cmath>

namespace
{
  Image<double> makeC(int sz, const double angle)
  {
    Image<double> result(sz, sz, NO_INIT);

    Image<double>::iterator ptr = result.beginw();

    const double ro2 = (0.25 * sz) * (0.25 * sz);
    const double ri2 = (0.2 * sz) * (0.2 * sz);
    const double xc = 0.5 * sz - 0.5;
    const double yc = 0.5 * sz - 0.5;

    for (int y = 0; y < sz; ++y)
      {
        const double y2 = (y-yc)*(y-yc);

        for (int x = 0; x < sz; ++x)
          {
            const double r2 = (x-xc)*(x-xc) + y2;

            const double theta = atan2(yc-y, x-xc);

            const double diff = geom::rad_npi_pi(geom::rad_npi_pi(angle) - theta);

            if (fabs(diff) > M_PI/8.0 &&
                r2 > ri2 && r2 <= ro2)
              *ptr++ = -1.0;
            else
              *ptr++ = 0.0;
          }
      }

    return result;
  }

  Image<double> makeO(int sz)
  {
    Image<double> result(sz, sz, NO_INIT);

    Image<double>::iterator ptr = result.beginw();

    const double ro2 = (0.25 * sz) * (0.25 * sz);
    const double ri2 = (0.2 * sz) * (0.2 * sz);
    const double xc = 0.5 * sz - 0.5;
    const double yc = 0.5 * sz - 0.5;

    for (int y = 0; y < sz; ++y)
      {
        const double y2 = (y-yc)*(y-yc);

        for (int x = 0; x < sz; ++x)
          {
            const double r2 = (x-xc)*(x-xc) + y2;

            if (r2 > ri2 && r2 <= ro2)
              *ptr++ = -1.0;
            else
              *ptr++ = 0.0;
          }
      }

    return result;
  }

  Image<double> makeQ(int sz, const double angle)
  {
    Image<double> result(sz, sz, NO_INIT);

    Image<double>::iterator ptr = result.beginw();

    const double cosa = cos(-angle);
    const double sina = sin(-angle);
    const double ro2 = (0.25 * sz) * (0.25 * sz);
    const double ri2 = (0.2 * sz) * (0.2 * sz);
    const double xc = 0.5 * sz - 0.5;
    const double yc = 0.5 * sz - 0.5;

    const double dh = 0.025 * sz;

    for (int y = 0; y < sz; ++y)
      {
        const double y2 = (y-yc)*(y-yc);

        for (int x = 0; x < sz; ++x)
          {
            const double r2 = (x-xc)*(x-xc) + y2;

            const double u = (x-xc) * cosa - (yc-y) * sina;
            const double v = (x-xc) * sina + (yc-y) * cosa;

            if ((r2 > ri2 && r2 <= ro2)
                ||
                (fabs(v) <= dh && fabs(u-0.225*sz) < 0.1*sz))
              *ptr++ = -1.0;
            else
              *ptr++ = 0.0;
          }
      }

    return result;
  }

  Image<double> makePlus(int sz, const double angle)
  {
    Image<double> result(sz, sz, NO_INIT);

    Image<double>::iterator ptr = result.beginw();

    const double cosa = cos(-angle);
    const double sina = sin(-angle);
    const double xc = 0.5 * sz - 0.5;
    const double yc = 0.5 * sz - 0.5;

    const double dh = 0.025 * sz;
    const double dw = 0.25 * sz;

    for (int y = 0; y < sz; ++y)
      for (int x = 0; x < sz; ++x)
        {
          const double u = (x-xc) * cosa - (yc-y) * sina;
          const double v = (x-xc) * sina + (yc-y) * cosa;

          if ((fabs(u) <= dw && fabs(v) <= dh)
              ||
              (fabs(v) <= dw && fabs(u) <= dh))
            *ptr++ = -1.0;
          else
            *ptr++ = 0.0;
        }

    return result;
  }

  Image<double> makeL(int sz, const double angle)
  {
    Image<double> result(sz, sz, NO_INIT);

    Image<double>::iterator ptr = result.beginw();

    const double cosa = cos(-angle);
    const double sina = sin(-angle);
    const double xc = 0.5 * sz - 0.5;
    const double yc = 0.5 * sz - 0.5;

    const double dh = 0.025 * sz;
    const double dw = 0.25 * sz;

    for (int y = 0; y < sz; ++y)
      for (int x = 0; x < sz; ++x)
        {
          const double u = (x-xc) * cosa - (yc-y) * sina;
          const double v = (x-xc) * sina + (yc-y) * cosa;

          if ((fabs(u+0.225*sz) <= dh && fabs(v) <= dw)
              ||
              (fabs(v+0.225*sz) <= dh && fabs(u) <= dw))
            *ptr++ = -1.0;
          else
            *ptr++ = 0.0;
        }

    return result;
  }

  Image<double> makeT(int sz, const double angle)
  {
    Image<double> result(sz, sz, NO_INIT);

    Image<double>::iterator ptr = result.beginw();

    const double cosa = cos(-angle);
    const double sina = sin(-angle);
    const double xc = 0.5 * sz - 0.5;
    const double yc = 0.5 * sz - 0.5;

    const double dh = 0.025 * sz;
    const double dw = 0.25 * sz;

    for (int y = 0; y < sz; ++y)
      for (int x = 0; x < sz; ++x)
        {
          const double u = (x-xc) * cosa - (yc-y) * sina;
          const double v = (x-xc) * sina + (yc-y) * cosa;

          if ((fabs(u) <= dh && fabs(v) <= dw)
              ||
              (fabs(v-0.225*sz) <= dh && fabs(u) <= dw))
            *ptr++ = -1.0;
          else
            *ptr++ = 0.0;
        }

    return result;
  }

  Image<double> makeDash(int sz, const double angle)
  {
    Image<double> result(sz, sz, NO_INIT);

    Image<double>::iterator ptr = result.beginw();

    const double cosa = cos(-angle);
    const double sina = sin(-angle);
    const double xc = 0.5 * sz - 0.5;
    const double yc = 0.5 * sz - 0.5;

    const double dh = 0.025 * sz;
    const double dw = 0.25 * sz;

    for (int y = 0; y < sz; ++y)
      for (int x = 0; x < sz; ++x)
        {
          const double u = (x-xc) * cosa - (yc-y) * sina;
          const double v = (x-xc) * sina + (yc-y) * cosa;

          if (fabs(u) <= dw && fabs(v) <= dh)
            *ptr++ = -1.0;
          else
            *ptr++ = 0.0;
        }

    return result;
  }

  Image<double> shrink(const Image<double>& img, int noctaves)
  {
    Image<double> result = img;
    for (int i = 0; i < noctaves; ++i)
      result = quickLocalAvg2x2(result);
    return result;
  }
}

// ######################################################################
ClassicSearchItem::ClassicSearchItem(Type t, int sz, double angle,
                                     int antialiasOctaves)
  :
  itsType(t),
  itsSize(sz),
  itsAngle(angle),
  itsAntialiasOctaves(antialiasOctaves)
{}

// ######################################################################
ClassicSearchItem::~ClassicSearchItem()
{}

// ######################################################################
Image<double> ClassicSearchItem::getPatch() const
{
  const int zoom = (1 << itsAntialiasOctaves);

  switch (itsType)
    {
    case ITEM_C: return shrink(makeC(itsSize*zoom, itsAngle), itsAntialiasOctaves);
    case ITEM_O: return shrink(makeO(itsSize*zoom), itsAntialiasOctaves);
    case ITEM_Q: return shrink(makeQ(itsSize*zoom, itsAngle), itsAntialiasOctaves);
    case ITEM_PLUS: return shrink(makePlus(itsSize*zoom, itsAngle), itsAntialiasOctaves);
    case ITEM_L: return shrink(makeL(itsSize*zoom, itsAngle), itsAntialiasOctaves);
    case ITEM_T: return shrink(makeT(itsSize*zoom, itsAngle), itsAntialiasOctaves);
    case ITEM_DASH: return shrink(makeDash(itsSize*zoom, itsAngle), itsAntialiasOctaves);
    default:
      break;
    }

  LFATAL("unsupported item type");
  /* can't happen */ return Image<double>();
}

// ######################################################################
ClassicSearchItem::Type ClassicSearchItem::typeFromChar(char c)
{
  switch (toupper(c))
    {
    case 'C': return ITEM_C;
    case 'O': return ITEM_O;
    case 'Q': return ITEM_Q;
    case '+': return ITEM_PLUS;
    case 'L': return ITEM_L;
    case 'T': return ITEM_T;
    case '-': return ITEM_DASH;
    }

  LFATAL("invalid search item type '%c'", c);
  /* can't happen */ return Type(-1);
}

// ######################################################################
ClassicSearchItemFactory::ClassicSearchItemFactory(SearchItem::Type c,
                                                   const std::string& types,
                                                   int sz,
                                                   const Range<double>& angleRange,
                                                   int antialiasOctaves,
                                                   int angleSeed,
                                                   int typeSeed)
  :
  itsLayer(c),
  itsTypeList(types),
  itsSize(sz),
  itsAngleRange(angleRange),
  itsAntialiasOctaves(antialiasOctaves),
  itsAngles(angleSeed),
  itsTypes(typeSeed)
{
  if (itsTypeList.size() == 0)
    LFATAL("search item typelist must include at least one type");
}

// ######################################################################
ClassicSearchItemFactory::~ClassicSearchItemFactory()
{}

// ######################################################################
rutz::shared_ptr<SearchItem> ClassicSearchItemFactory::make(const geom::vec2d& pos)
{
  ClassicSearchItem::Type t =
    ClassicSearchItem::typeFromChar(itsTypeList[itsTypes.idraw(itsTypeList.size())]);

  rutz::shared_ptr<ClassicSearchItem> el
    (new ClassicSearchItem(t, itsSize,
                           itsAngles.fdraw_range(itsAngleRange.min(), itsAngleRange.max()),
                           itsAntialiasOctaves));

  el->type = itsLayer;
  el->pos = pos;

  return el;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_CLASSICSEARCHITEM_C_DEFINED
