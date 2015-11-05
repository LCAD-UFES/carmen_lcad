/*!@file Psycho/ClassicSearchItem.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/ClassicSearchItem.H $
// $Id: ClassicSearchItem.H 9079 2007-12-12 00:54:09Z rjpeters $
//

#ifndef PSYCHO_CLASSICSEARCHITEM_H_DEFINED
#define PSYCHO_CLASSICSEARCHITEM_H_DEFINED

#include "Image/Range.H"
#include "Psycho/SearchItem.H"
#include "rutz/rand.h"

class ClassicSearchItem : public SearchItem
{
public:
  enum Type
  {
    ITEM_C,
    ITEM_O,
    ITEM_Q,
    ITEM_PLUS,
    ITEM_L,
    ITEM_T,
    ITEM_DASH
  };

  ClassicSearchItem(Type t, int sz, double angle, int antialiasOctaves);

  virtual ~ClassicSearchItem();

  virtual Image<double> getPatch() const;

  static Type typeFromChar(char c);

private:
  Type itsType;
  int itsSize;
  double itsAngle;
  int itsAntialiasOctaves;
};

class ClassicSearchItemFactory : public SearchItemFactory
{
public:
  ClassicSearchItemFactory(SearchItem::Type c,
                           const std::string& types,
                           int sz,
                           const Range<double>& angleRange,
                           int antialiasOctaves = 3,
                           int angleSeed = 0,
                           int typeSeed = 0);

  virtual ~ClassicSearchItemFactory();

  virtual rutz::shared_ptr<SearchItem> make(const geom::vec2d& pos);

private:
  SearchItem::Type itsLayer;
  std::string itsTypeList;
  int itsSize;
  Range<double> itsAngleRange;
  int itsAntialiasOctaves;

  rutz::urand itsAngles;
  rutz::urand itsTypes;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_CLASSICSEARCHITEM_H_DEFINED
