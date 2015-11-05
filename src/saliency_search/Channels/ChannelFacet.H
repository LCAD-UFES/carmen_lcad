/*!@file Channels/ChannelFacet.H Allow user-configurable "facets" to be associated with arbitrary classes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelFacet.H $
// $Id: ChannelFacet.H 9748 2008-05-11 19:21:54Z rjpeters $
//

#ifndef CHANNELS_CHANNELFACET_H_DEFINED
#define CHANNELS_CHANNELFACET_H_DEFINED

#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/demangle.h"
#include "rutz/shared_ptr.h"

class ParamMap;

/// Base class for facets that can be held in a ChannelFacetMap
/** ChannelFacet provides a simple mechanism to add extra data and
    functions to the channels at runtime. Example usage is to add a
    set of weights for the different feature maps. The only default
    interface to the ChannelFacet is to provide readFrom() and
    writeTo() functions to dump the ChannelFacet data to a
    ParamMap. Because this base class is purely virtual, it cannot be
    directly implemented but will first need to be derived into an
    implementable class. */
class ChannelFacet
{
protected:
  /// Constructor is protected (base class should not be used on its own)
  ChannelFacet();

public:
  /// Virtual destructor for safe inheritance
  virtual ~ChannelFacet();

  /// Write our data out to a ParamMap
  virtual void writeTo(ParamMap& pmap) const = 0;

  /// Read our data back in from a ParamMap
  virtual void readFrom(const ParamMap& pmap) = 0;
};

/// Mix-in class designed to be inherited to provide facets to the subclass
/** This map can hold various ChannelFacet elements and provides a
    basic interface to add/retrieve ChannelFacet
    elements. SingleChannel and ComplexChannel both inherit from
    ChannelFacetMap. */
class ChannelFacetMap
{
public:
  /// Default constructor
  ChannelFacetMap();

  /// Virtual destructor
  virtual ~ChannelFacetMap();

  /// Write our facets out to a ParamMap
  void writeFacetsTo(ParamMap& pmap) const;

  /// Read our facets back in from a ParamMap
  void readFacetsFrom(const ParamMap& pmap);

  /// Query if we have a facet of type T
  template <class T> inline
  bool hasFacet() const;

  /// Get the facet of type T
  /** Will throw an exception if there is no such facet (check
      hasFacet() first). */
  template <class T> inline
  rutz::shared_ptr<T> getFacet() const;

  /// Install a facet of type T
  /** This will replace any previous such facet. */
  template <class T> inline
  void setFacet(rutz::shared_ptr<T> f);

  /// Number of installed facets
  unsigned int numFacets() const;

private:
  rutz::shared_ptr<ChannelFacet> getBaseFacet(const std::type_info& tp) const;

  void setBaseFacet(const std::type_info& tp,
                    rutz::shared_ptr<ChannelFacet> f);

  ChannelFacetMap(const ChannelFacetMap&); // not implemented
  ChannelFacetMap& operator=(const ChannelFacetMap&); // not implemented

  struct Impl;
  Impl* const rep;
};


// ######################################################################
// ##### Implementation of inline functions
// ######################################################################
template <class T> inline
bool ChannelFacetMap::hasFacet() const
{
  return getBaseFacet(typeid(T)).get() != 0;
}

// ######################################################################
template <class T> inline
rutz::shared_ptr<T> ChannelFacetMap::getFacet() const
{
  rutz::shared_ptr<ChannelFacet> f = this->getBaseFacet(typeid(T));
  if (f.get() == 0)
    LFATAL("no such facet: %s", rutz::demangled_name(typeid(T)));

  // else...
  rutz::shared_ptr<T> fnew = rutz::dyn_cast<T>(f);

  // this is a hard ASSERT() rather than an LFATAL() because an
  // error here represents a design flaw rather than a user error --
  // by design, the facet associated with typeid(T) should always be
  // (castable to) type T itself
  ASSERT(f.get() != 0);

  return fnew;
}

// ######################################################################
template <class T> inline
void ChannelFacetMap::setFacet(rutz::shared_ptr<T> f)
{
  this->setBaseFacet(typeid(T), f);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_CHANNELFACET_H_DEFINED
