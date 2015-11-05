/** @file Psycho/SearchArray.C jittered array of search elements */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/SearchArray.C $
// $Id: SearchArray.C 15310 2012-06-01 02:29:24Z itti $
//

// Code herein is derived from GroovX, also licensed under the GPL
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// [http://ilab.usc.edu/rjpeters/groovx/]

#ifndef PSYCHO_SEARCHARRAY_C_DEFINED
#define PSYCHO_SEARCHARRAY_C_DEFINED

#include "Psycho/SearchArray.H"

#include "Image/CutPaste.H"
#include "Image/Image.H"
#include "Raster/Raster.H"
#include "Image/geom.h"
#include "Image/Rectangle.H"
#include "rutz/rand.h"
#include "Util/StringConversions.H"
#include <cstdio>
#include <algorithm> //for random_shuffle

using geom::vec2d;

namespace
{
  void dumpFrame(const Image<byte>& bmap)
  {
    static int framecount = 0;

    Raster::WriteGray(bmap, sformat("frame_%06d.png", framecount++));
  }
}

SearchArray::SearchArray(const Dims& dims,
                         double gridSpacing,
                         double minSpacing,
                         double margin)
  :
  itsDims(dims),
  itsGridSpacing(gridSpacing),
  itsMinSpacing(minSpacing),
  itsFillResolution(15.0),
  itsMargin(margin),

  itsArray(),

  itsDumpingFrames(false),
  itsFrameDumpPeriod(20)
{}

SearchArray::~SearchArray() throw() {}

void SearchArray::addElement(const rutz::shared_ptr<SearchItem>& e, const bool displace) const
{
  if (tooClose(e->pos, -1))
    {
    if (!displace)
      LFATAL("element @ %g,%g is too close to an existing element",
           e->pos.x(), e->pos.y());
    else
      {
        std::vector<rutz::shared_ptr<SearchItem> > isTooClose;
        const double minSpacingSqr = itsMinSpacing*itsMinSpacing;
        for (size_t n = 0; n < itsArray.size(); ++n)
          {
            const vec2d v = e->pos; 
            const double dx = itsArray[n]->pos.x() - v.x();
            const double dy = itsArray[n]->pos.y() - v.y();

            if(dx*dx+dy*dy <= minSpacingSqr)
              isTooClose.push_back(itsArray[n]);
          }
        for (size_t i = 0; i < isTooClose.size(); ++i)
          doRemoveElement(isTooClose[i]);
      }
    }
  doAddElement(e);
}

void SearchArray::replaceElementAtSamePos(size_t i, const rutz::shared_ptr<SearchItem>& e)
{
  if (i >= itsArray.size())
    LFATAL("expected idx < %" ZU ", got idx = %" ZU , itsArray.size(), i);

  const rutz::shared_ptr<SearchItem> old = itsArray[i];
  itsArray[i] = e;
  itsArray[i]->pos = old->pos;
}

void SearchArray::generateBackground(SearchItemFactory& factory,
                                     const int diffusionCycles,
                                     const bool doFinalFill,
                                     const int jitterIters,
                                     const bool doJitterForeground,
                                     const int backgSeed) const
{
  const size_t foregSize = itsArray.size();

  backgHexGrid(factory);

  rutz::urand urand(backgSeed);

  for (int i = 0; i < diffusionCycles; ++i)
    {
      backgJitter(urand, jitterIters, doJitterForeground);
      if (i+1 < diffusionCycles || doFinalFill)
        backgFill(factory);

      //      const double spacing = sqrt(2.0*itsDims.sz()/(sqrt(3.0)*itsArray.size()));
      //      LINFO("%" ZU " elements, ave spacing %f", itsArray.size(), spacing);
    }

  size_t insideNumber = foregSize;

  for (size_t n = 0; n < itsArray.size(); ++n)
    {
      if (itsArray[n]->type == SearchItem::FOREGROUND)
        continue;

      bool inside = true;

      for (size_t i = 0; i < foregSize; ++i)
        {
          const size_t j = (i+1) % foregSize;

          // This is the vector perpendicular to the line connecting
          // contour nodes i and j
          const double Yij = itsArray[i]->pos.x() - itsArray[j]->pos.x();
          const double Xij = itsArray[j]->pos.y() - itsArray[i]->pos.y();

          // This is the vector connecting from contour node i to the
          // background node
          const double Xin = itsArray[n]->pos.x() - itsArray[i]->pos.x();
          const double Yin = itsArray[n]->pos.y() - itsArray[i]->pos.y();

          // If the dot product of those two vectors is less than zero,
          // then the background node is "outside".
          const double vp = Xij*Xin + Yij*Yin;

          if (vp < 0.0) { inside = false; break; }
        }

      if (inside)
        {
          ++insideNumber;
        }
    }

  //  LINFO(" FOREG_NUMBER %" ZU "    PATCH_NUMBER %" ZU "    TOTAL_NUMBER %" ZU ,
  //      foregSize, insideNumber, itsArray.size());
}

Image<byte> SearchArray::getImage(const double lo, const double hi,
                                  const double bg,
                                  bool doTagLast) const
{
  Image<double> tmpwin(itsDims, NO_INIT);
  tmpwin.clear(bg);

  const Image<double>::iterator win = tmpwin.beginw();

  for (size_t i = 0; i < itsArray.size(); ++i)
    {
      const int xcenter = int(itsArray[i]->pos.x() + itsDims.w() / 2.0 + 0.5);
      const int ycenter = int(itsArray[i]->pos.y() + itsDims.h() / 2.0 + 0.5);
      
      const Image<double> p = itsArray[i]->getPatch();

      // top left:
      const int x0 = xcenter - p.getWidth() / 2;
      const int y0 = ycenter - p.getHeight() / 2;
      // bottom right:
      const int x1 = x0 + p.getWidth();
      const int y1 = y0 + p.getHeight();

      for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x)
          {
            if (x >= 0 && x < itsDims.w() && y >=0 && y < itsDims.h())
              win[x+y*itsDims.w()] += p.getVal(x-x0, y-y0);
          }

      if (doTagLast && (i+1) == itsArray.size())
        {
          const double outer = 0.4 * p.getWidth();
          const double inner = outer - 3;

          for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x)
              {
                if (x >= 0 && x < itsDims.w() && y >=0 && y < itsDims.h())
                  {
                    const double r = sqrt((x-xcenter)*(x-xcenter) +
                                          (y-ycenter)*(y-ycenter));

                    if (r >= inner && r <= outer)
                      {
                        win[x+y*itsDims.w()] = 1.0;
                      }
                  }
              }
        }
    }

  Image<byte> result(tmpwin.getDims(), ZEROS);

  unsigned char* bytes = result.beginw();

  bool clip = false;

  const int npix = result.getSize();

  const double rng = hi - lo;

  for (int k = 0; k < npix; ++k)
    {
      int val = int((win[k]-lo)/rng*255);

      if      (val < 0)   { clip = true; val = 0; }
      else if (val > 255) { clip = true; val = 255; }

      GVX_ASSERT(val >= 0);
      GVX_ASSERT(val <= 255);

      *bytes++ = val;
    }

  if (clip)
    LERROR("some values were clipped");

  return result;
}

void SearchArray::pareBackground(const uint shrunkSize) const
{
  // removes elements at random from the array down to the shrunkSize
  // elements need to be shuffled first.
  random_shuffle(itsArray.begin(),itsArray.end()); 
  while(itsArray.size() > shrunkSize)
    {
      const rutz::shared_ptr<SearchItem> last = itsArray.back();
      doRemoveElement(last);
      if (last->type == SearchItem::FOREGROUND)
        doAddElement(last,true);
    }
}

void SearchArray::clear() const
{
  itsArray.clear();
}

void SearchArray::saveCoords(const std::string& filename) const
{
  FILE* f = fopen(filename.c_str(), "w");
  if (f == 0)
    LFATAL("couldn't open %s for writing", filename.c_str());

  fprintf(f, "%% fg?   x0 y0   x1 y1\n");

  for (size_t i = 0; i < itsArray.size(); ++i)
    {
      const int xcenter = int(itsArray[i]->pos.x() + itsDims.w() / 2.0 + 0.5);
      const int ycenter = int(itsArray[i]->pos.y() + itsDims.h() / 2.0 + 0.5);

      const Image<double> p = itsArray[i]->getPatch();

      // top left:
      const int x0 = xcenter - p.getWidth() / 2;
      const int y0 = ycenter - p.getHeight() / 2;
      // bottom right:
      const int x1 = x0 + p.getWidth();
      const int y1 = y0 + p.getHeight();

      fprintf(f, "  %d   %d %d   %d %d\n",
              itsArray[i]->type == SearchItem::FOREGROUND ? 1 : 0,
              x0, y0, x1, y1);
    }

  fclose(f);
}

Rectangle SearchArray::itemBounds() const
{
  Rectangle a;
  return a.centerDims(Point2D<int>(0,0),itsDims-itsMargin*2);
}

void SearchArray::doAddElement(const rutz::shared_ptr<SearchItem>& e, bool toFront) const
{
  if (!toFront)
    itsArray.push_back(e);   
  else
    itsArray.insert(itsArray.begin(),e);

  // this double call is on purpose so that the added elements don't fly by
  // quite so quickly in the resulting movie
  if (itsDumpingFrames)
    {
      const Image<byte> bmap = getImage(true);

      dumpFrame(bmap);
      dumpFrame(bmap);
    }
}

bool SearchArray::tooClose(const vec2d& v, int except) const
{
  if ((v.x() + itsDims.w()*0.5) < itsMargin
      || (itsDims.w()*0.5 - v.x()) <= itsMargin
      || (v.y() + itsDims.h()*0.5) < itsMargin
      || (itsDims.h()*0.5 - v.y()) <= itsMargin)
    return true;

  const double minSpacingSqr = itsMinSpacing*itsMinSpacing;

  for (size_t n = 0; n < itsArray.size(); ++n)
    {
      const double dx = itsArray[n]->pos.x() - v.x();
      const double dy = itsArray[n]->pos.y() - v.y();

      if (dx*dx+dy*dy <= minSpacingSqr && int(n) != except)
        return true;
    }

  return false;
}

void SearchArray::backgHexGrid(SearchItemFactory& factory) const
{
  // lay down a hexagonal grid of elements

  const double dx = itsGridSpacing;
  const double dy = sqrt(3.0) * itsGridSpacing / 2.0;

  const int nx = int((itsDims.w() - itsMinSpacing) / dx - 0.5);
  const int ny = int((itsDims.h() - itsMinSpacing) / dy);

  double y = -0.5 * (ny-1) * dy;

  for (int j = 0; j < ny; ++j, y += dy)
    {
      double x = -0.5 * (nx-1) * dx - 0.25 * dx;

      // this is a hexagonal grid, so every other row is staggered by half
      // a step in the x direction
      if (j%2) x += 0.5*dx;

      for (int i = 0; i < nx; ++i, x += dx)
        {
          if (!tooClose(vec2d(x, y), -1))
            doAddElement(factory.make(vec2d(x, y)));
        }
    }
}

void SearchArray::backgFill(SearchItemFactory& factory) const
{
  const double dx = itsMinSpacing / itsFillResolution;

  const double halfX = 0.5 * itsDims.w();
  const double halfY = 0.5 * itsDims.h();

  for (double x = -halfX; x <= halfX; x += dx)
    for (double y = -halfY; y <= halfY; y += dx)
      {
        if (!tooClose(vec2d(x, y), -1))
          doAddElement(factory.make(vec2d(x, y)));
      }
}

void SearchArray::backgJitter(rutz::urand& urand, const int jitterIters,
                              const bool doJitterForeground) const
{
  const double jitter = (itsMinSpacing/16.0);

  const double halfX = 0.5 * itsDims.w();
  const double halfY = 0.5 * itsDims.h();

  for (int niter = 0; niter < jitterIters; ++niter)
    {
      for (size_t n = 0; n < itsArray.size(); ++n)
        {
          if (!doJitterForeground
              && itsArray[n]->type == SearchItem::FOREGROUND)
            continue;

          vec2d v;
          v.x() = itsArray[n]->pos.x() + jitter*(urand.fdraw_range(-1.0, 1.0));
          v.y() = itsArray[n]->pos.y() + jitter*(urand.fdraw_range(-1.0, 1.0));

          if (v.x() < -halfX) v.x() += itsDims.w();
          if (v.x() >  halfX) v.x() -= itsDims.w();
          if (v.y() < -halfY) v.y() += itsDims.h();
          if (v.y() >  halfY) v.y() -= itsDims.h();

          if (!tooClose(v, n))
            {
              itsArray[n]->pos = v;
            }
        }

      if (itsDumpingFrames &&
          niter % itsFrameDumpPeriod == 0)
        {
          const Image<byte> bmap = getImage(true);
          dumpFrame(bmap);
        }
    }
}

void SearchArray::doRemoveElement(const rutz::shared_ptr<SearchItem>& e) const
{
  for (size_t i = 0; i < itsArray.size(); ++i)
    {
      if (e->pos.x()==itsArray[i]->pos.x() && 
          e->pos.y()==itsArray[i]->pos.y())
        {
          itsArray.erase(itsArray.begin()+i);
          break;
        }
    }

  // this double call is on purpose so that the added elements don't fly by
  // quite so quickly in the resulting movie
  if (itsDumpingFrames)
    {
      const Image<byte> bmap = getImage(true);

      dumpFrame(bmap);
      dumpFrame(bmap);
    }

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_SEARCHARRAY_C_DEFINED
