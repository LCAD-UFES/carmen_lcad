/*!@file Channels/ChannelFacetScalar.C ChannelFacet that contains scalar */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelFacetScalar.C $
// $Id: ChannelFacetScalar.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Channels/ChannelFacetScalar.H"
#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"
#include "Component/ParamMap.H"
#include "Util/sformat.H"

// ######################################################################
// ######################################################################
ChannelFacetScalar::ChannelFacetScalar(const uint siz, const double initval)
  :
  itsVals(siz, initval)
{ }

// ######################################################################
ChannelFacetScalar::~ChannelFacetScalar()
{ }

// ######################################################################
// ######################################################################
ChannelFacetScalarSingle::
ChannelFacetScalarSingle(const SingleChannel& chan, const double initval)
  :
  ChannelFacetScalar(chan.getLevelSpec().maxIndex(), initval),
  itsLevelSpec(chan.getLevelSpec())
{ }

// ######################################################################
ChannelFacetScalarSingle::~ChannelFacetScalarSingle()
{ }

// ######################################################################
void ChannelFacetScalarSingle::
setVal(const uint clev, const uint slev, const double val)
{
  itsVals[itsLevelSpec.csToIndex(clev, slev)] = val;
}

// ######################################################################
void ChannelFacetScalarSingle::
setVal(const uint idx, const double val)
{
  ASSERT(idx < itsVals.size());
  itsVals[idx] = val;
}

// ######################################################################
double ChannelFacetScalarSingle::
getVal(const uint clev, const uint slev) const
{
  return itsVals[itsLevelSpec.csToIndex(clev, slev)];
}

// ######################################################################
double ChannelFacetScalarSingle::
getVal(const uint idx) const
{
  ASSERT(idx < itsVals.size());
  return itsVals[idx];
}

// ######################################################################
void ChannelFacetScalarSingle::writeTo(ParamMap& pmap) const
{
  for (uint i = 0; i < itsLevelSpec.maxIndex(); ++i)
    {
      uint c, s; itsLevelSpec.indexToCS(i, c, s);
      const std::string pname = sformat("value(%d,%d)", c, s);
      pmap.putDoubleParam(pname, itsVals[i]);
    }
}

// ######################################################################
void ChannelFacetScalarSingle::readFrom(const ParamMap& pmap)
{
  for (uint i = 0; i < itsLevelSpec.maxIndex(); ++i)
    {
      uint c, s; itsLevelSpec.indexToCS(i, c, s);
      const std::string pname = sformat("value(%d,%d)", c, s);
      if (!pmap.hasParam(pname))
        LFATAL("no such value in ParamMap: '%s'", pname.c_str());
      itsVals[i] = pmap.getDoubleParam(pname);
    }
}

// ######################################################################
// ######################################################################
ChannelFacetScalarComplex::
ChannelFacetScalarComplex(const ComplexChannel& chan, const double initval)
  :
  ChannelFacetScalar(chan.numChans(), initval)
{ }

// ######################################################################
ChannelFacetScalarComplex::~ChannelFacetScalarComplex()
{ }

// ######################################################################
void ChannelFacetScalarComplex::
setVal(const uint subchan, const double val)
{
  ASSERT(subchan < itsVals.size());
  itsVals[subchan] = val;
}

// ######################################################################
double ChannelFacetScalarComplex::
getVal(const uint subchan) const
{
  ASSERT(subchan < itsVals.size());
  return itsVals[subchan];
}

// ######################################################################
void ChannelFacetScalarComplex::writeTo(ParamMap& pmap) const
{
  for (uint i = 0; i < itsVals.size(); ++i)
    {
      const std::string pname = sformat("value(%d)", i);
      pmap.putDoubleParam(pname, itsVals[i]);
    }
}

// ######################################################################
void ChannelFacetScalarComplex::readFrom(const ParamMap& pmap)
{
  for (uint i = 0; i < itsVals.size(); ++i)
    {
      const std::string pname = sformat("value(%d)", i);
      if (!pmap.hasParam(pname))
        LFATAL("no such value in ParamMap: '%s'", pname.c_str());
      itsVals[i] = pmap.getDoubleParam(pname);
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
