/*!@file Channels/ChannelFacetScalar.H ChannelFacet that contains scalars */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelFacetScalar.H $
// $Id: ChannelFacetScalar.H 14376 2011-01-11 02:44:34Z pez $
//

#ifndef CHANNELS_CHANNELFACETSCALAR_H_DEFINED
#define CHANNELS_CHANNELFACETSCALAR_H_DEFINED

#include<vector>

#include "Channels/ChannelFacet.H"
#include "Image/LevelSpec.H"

class SingleChannel;
class ComplexChannel;

/// ChannelFacet that contains one scalar value per feature map
/** This is a simple derivation of ChannelFacet to add one scalar
    value to each feature map of a SingleChannel, or to each
    subchannel of a ComplexChannel. ChannelFacetScalar is a virtual
    base that cannot be implemented because it inherits pure virtual
    readFrom() and writeTo() from ChannelFacet, see
    ChannelFacetScalarSingle and ChannelFacetScalarComplex for
    implementable derivations.  This class is used, e.g., to store a
    weighing (gain) coefficient for each submap, or the mean or std
    submap values, for either targets or distractors, etc. */
class ChannelFacetScalar : public ChannelFacet
{
public:
  /// Virtual destructor for safe inheritance
  virtual ~ChannelFacetScalar();

protected:
  /// Constructor
  ChannelFacetScalar(const uint siz, const double initval);

  //!< Our data values
  std::vector<double> itsVals;
};

// ######################################################################
/// Specialization of ChannelFacetScalar for SingleChannel
/** This class holds one scalar parameter per feature map of the
    SingleChannel.  Note that since the LevelSpec is used internally
    at constructions to determine how many feature maps are in each
    SingleChannel, this should not be constructed until after the
    model has been start()'ed so that the LevelSpec is
    valid. Typically, PreFrontalCortex would be in charge of
    installing these facets by using a ChannelVisitor depending on the
    type of extra top-down attention components being used. */
class ChannelFacetScalarSingle : public ChannelFacetScalar
{
public:
  /// Destructor
  virtual ~ChannelFacetScalarSingle();

  /// Set a value
  void setVal(const uint clev, const uint slev, const double val);

  /// Set a value
  void setVal(const uint idx, const double val);

  /// Get a value
  double getVal(const uint clev, const uint slev) const;

  /// Get a value
  double getVal(const uint idx) const;

  /// Write our data out to a ParamMap
  virtual void writeTo(ParamMap& pmap) const;

  /// Read our data back in from a ParamMap
  virtual void readFrom(const ParamMap& pmap);

protected:
  /// Constructor
  ChannelFacetScalarSingle(const SingleChannel& chan, const double initval);

  const LevelSpec itsLevelSpec; //!< Keep track of our channel's info
};

// ######################################################################
/// Specialization of ChannelFacetScalar for ComplexChannel
/** This class holds one or more named scalar parameters per
     subchannel of the ComplexChannel.  Note that this should be
     called after the model has been start()'ed since we will in our
     constructor query the ComplexChannel to determine how many
     subchannels it has. Typically, PreFrontalCortex would be in
     charge of installing these facets by using a ChannelVisitor
     depending on the type of extra top-down attention components
     being used. */
class ChannelFacetScalarComplex : public ChannelFacetScalar
{
public:
  /// Destructor
  virtual ~ChannelFacetScalarComplex();

  /// Set a value
  void setVal(const uint subchan, const double val);

  /// Get a value
  double getVal(const uint subchan) const;

  /// Write our data out to a ParamMap
  virtual void writeTo(ParamMap& pmap) const;

  /// Read our data back in from a ParamMap
  virtual void readFrom(const ParamMap& pmap);

protected:
  /// Constructor
  ChannelFacetScalarComplex(const ComplexChannel& chan, const double initval);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_CHANNELFACETSCALAR_H_DEFINED
