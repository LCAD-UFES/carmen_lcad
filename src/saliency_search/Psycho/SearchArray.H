/** @file Psycho/SearchArray.H jittered array of search elements */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/SearchArray.H $
// $Id: SearchArray.H 14200 2010-11-04 03:10:43Z jshen $
//

// Code herein is derived from GroovX, also licensed under the GPL
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// [http://ilab.usc.edu/rjpeters/groovx/]

#ifndef PSYCHO_SEARCHARRAY_H_DEFINED
#define PSYCHO_SEARCHARRAY_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/vec2.h"
#include "Psycho/SearchItem.H"
#include "rutz/arrays.h"
#include "rutz/shared_ptr.h"

#include <string>
#include <vector>

namespace rutz
{
  class urand;
}

/// SearchArray represents an 2-D spatial array of search items
/** The array has manipulable spacing properties. */
class SearchArray
{
public:
  SearchArray(const Dims& dims = Dims(512, 512),
              double gridSpacing = 48.0,
              double minSpacing = 36.0,
              double margin = 0.0);

  virtual ~SearchArray() throw();

  void addElement(const rutz::shared_ptr<SearchItem>& e, bool displace = false) const;
  size_t numElements() const { return itsArray.size(); }

  void replaceElementAtSamePos(size_t i, const rutz::shared_ptr<SearchItem>& e);

  void generateBackground(SearchItemFactory& factory,
                          const int diffusionCycles = 10,
                          const bool doFinalFill = true,
                           const int jitterIters = 1000,
                          const bool doJitterForeground = false,
                          const int backgSeed = 0) const;

  Image<byte> getImage(const double lo = -1.0, const double hi = 1.0,
                       const double bg = 0.0,
                       bool doTagLast = false) const;

  void pareBackground(const uint shrunkSize) const;
  void clear() const;
  void saveCoords(const std::string& filename) const;
  Rectangle itemBounds() const;
private:
  SearchArray(const SearchArray&);
  SearchArray& operator=(const SearchArray&);

  void doAddElement(const rutz::shared_ptr<SearchItem>& e, const bool toFront = false) const;
  bool tooClose(const geom::vec2<double>& v, int except) const;
  void backgHexGrid(SearchItemFactory& factory) const;
  void backgFill(SearchItemFactory& factory) const;
  void backgJitter(rutz::urand& urand, const int niter,
                   const bool doJitterForeground) const;
  void doRemoveElement(const rutz::shared_ptr<SearchItem>& e) const;
  Dims itsDims;
  double itsGridSpacing;
  double itsMinSpacing;
  double itsFillResolution;
  double itsMargin;

  mutable std::vector<rutz::shared_ptr<SearchItem> > itsArray;

  bool itsDumpingFrames;
  unsigned int itsFrameDumpPeriod;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_SEARCHARRAY_H_DEFINED
