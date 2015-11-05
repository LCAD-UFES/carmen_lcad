/** @file Psycho/GaborSnake.H sequences of contour-aligned gabor elements */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/GaborSnake.H $
// $Id: GaborSnake.H 9078 2007-12-11 21:11:45Z rjpeters $
//

// Code herein is derived from GroovX, also licensed under the GPL
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// [http://ilab.usc.edu/rjpeters/groovx/]

#ifndef PSYCHO_GABORSNAKE_H_DEFINED
#define PSYCHO_GABORSNAKE_H_DEFINED

#include "Psycho/SearchItem.H"
#include "Psycho/GaborPatch.H"
#include "Image/vec2.h"
#include "rutz/arrays.h"

namespace rutz
{
  class urand;
}

class GaborSnake
{
public:
  GaborSnake(int l, double spacing, int seed, int xcenter, int ycenter,
             GaborPatchItemFactory& factory);
  ~GaborSnake();

  size_t numElements() const { return size_t(itsLength); }
  rutz::shared_ptr<SearchItem> getElement(int n) const;

private:
  const int itsLength;
  rutz::fixed_block<geom::vec2<double> > itsElem;
  std::vector<rutz::shared_ptr<GaborPatchItem> > itsPatches;

  geom::vec2<double>& elem(int i)
  {
    i = i % itsLength;
    if (i < 0) i += itsLength;
    return itsElem[i];
  }

  const geom::vec2<double>& elem(int i) const
  {
    i = i % itsLength;
    if (i < 0) i += itsLength;
    return itsElem[i];
  }

  // Returns true if jiggling converged
  bool jiggle(rutz::urand& urand);
  void transformPath(int i1, const geom::vec2<double>& new1,
                     int i2, const geom::vec2<double>& new2);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_GABORSNAKE_H_DEFINED
