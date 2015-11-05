/*!@file Channels/DirectFeedChannel.C the source for DirectFeedChannel and
MultiDirectFeedChannel */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DirectFeedChannel.C $
// $Id: DirectFeedChannel.C 10746 2009-02-03 07:09:00Z itti $
//

#include "Channels/DirectFeedChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ShapeOps.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/StringConversions.H"
#include "Util/sformat.H"

// ######################################################################
DirectFeedChannel::DirectFeedChannel(OptionManager& mgr, int id)
  : ChannelBase(mgr, "DirectFeed", "DirectFeed", DIRECTFEED),
    itsMapLevel(&OPT_MapLevel, this),
    itsNormType(&OPT_MaxNormType, this),
    itsUseOlderVersion(&OPT_UseOlderVersion, this), // see Channels/ChannelOpts.{H,C}
    itsOutputRangeMin(&OPT_ChannelOutputRangeMin, this),
    itsOutputRangeMax(&OPT_ChannelOutputRangeMax, this)
{
  if (id >= 0)
    {
      std::string name("DirectFeed_No_");
      name += toStr(id);
      setTagName(name);
      setDescriptiveName(name);
    }
}

// ######################################################################
DirectFeedChannel::~DirectFeedChannel()
{}

// ######################################################################
void DirectFeedChannel::start1()
{
  // in the new version, leave our output range open rather than
  // forcing it to a given range of values:
  if (itsUseOlderVersion.getVal() == false)
    {
      itsOutputRangeMin.setVal(0.0f);
      itsOutputRangeMax.setVal(0.0f);
    }

  // in the older version, we used to set the map range as we would
  // also apply spatial competition for salience to the output map,
  // only if using the MAXNORM type of competition, and otherwise we
  // would not touch the range:
  if (itsUseOlderVersion.getVal() && itsNormType.getVal() != VCXNORM_MAXNORM)
    {
      itsOutputRangeMin.setVal(0.0f);
      itsOutputRangeMax.setVal(0.0f);
    }
}

// ######################################################################
void DirectFeedChannel::reset1()
{
  itsMapDims = Dims();
  itsPyr.reset();
  itsInputTime = SimTime::ZERO();
  itsPyrTime = SimTime::SECS(-1.0);
  itsCoeff.clear();
  itsOutputCache.freeMem();

  ChannelBase::reset1();
}

// ######################################################################
void DirectFeedChannel::readFrom(const ParamMap& pmap)
{
  //FIXME
  ChannelBase::readFrom(pmap);
}

// ######################################################################
void DirectFeedChannel::writeTo(ParamMap& pmap) const
{
  //FIXME
  ChannelBase::writeTo(pmap);
}

// ######################################################################
void DirectFeedChannel::clampCoeffs(const double cmin, const double cmax)
{
  killCaches();
  for (uint i = 0; i < itsCoeff.size(); ++i)
    itsCoeff[i] = clampValue(itsCoeff[i], cmin, cmax);
}

// ######################################################################
double DirectFeedChannel::absSumCoeffs() const
{
  double sum = 0.0;
  for (uint i = 0; i < itsCoeff.size(); ++i)
    sum += std::abs(itsCoeff[i]);
  return sum;
}

// ######################################################################
void DirectFeedChannel::normalizeCoeffs(const double div)
{
  killCaches();
  for (uint i = 0; i < itsCoeff.size(); ++i)
    itsCoeff[i] /=div;
}

// ######################################################################
void DirectFeedChannel::setCoeff(const uint idx, const double val)
{
  if (itsPyr.isEmpty()) initializeCoeffs(idx+1,1.0);
  else ASSERT(idx < numSubmaps());

  killCaches();
  itsCoeff[idx] = val;
}

// ######################################################################
double DirectFeedChannel::getCoeff(const uint idx) const
{
  ASSERT(idx < numSubmaps());
  return itsCoeff[idx];
}

// ######################################################################
void DirectFeedChannel::initializeCoeffs(const uint numCoeffs,
                                         const double val)
{
  itsCoeff.resize(numCoeffs,val);
}

// ######################################################################
void DirectFeedChannel::inputPyramid(const ImageSet<float>& pyramid,
                                     const SimTime& t)
{
  if (itsCoeff.empty()) initializeCoeffs(pyramid.size());
  else ASSERT(itsCoeff.size() == pyramid.size());

  itsPyr = pyramid;
  itsPyrTime = t;
  LDEBUG("itsPyrTime: %fms", itsPyrTime.msecs());
}

// ######################################################################
void DirectFeedChannel::doInput(const InputFrame& inframe)
{
  if (inframe.grayFloat().initialized()) LINFO("using bwimg");
  else if (inframe.colorFloat().initialized()) LINFO("using colimg");
  else LFATAL("Need to have either colimg or bwimg as input!");

  itsInputTime = inframe.time();

  LDEBUG("itsInputTime: %fms", itsInputTime.msecs());

  if (itsInputTime != itsPyrTime)
    LFATAL("I don't have any direct-feed input for time=%fms "
           "(last input was at time=%fms)",
           itsInputTime.msecs(), itsPyrTime.msecs());

  const float fac = pow(0.5f,float(itsMapLevel.getVal()));
  itsMapDims = Dims(int(this->getInputDims().w()*fac),
                    int(this->getInputDims().h()*fac));
  LDEBUG("itsMapDims = %s; itsInputDims = %s",toStr(itsMapDims).c_str(),
        toStr(this->getInputDims()).c_str());
}

// ######################################################################
bool DirectFeedChannel::outputAvailable() const
{
  LDEBUG("itsPyrTime: %fms; itsInputTime: %fms",
         itsPyrTime.msecs(), itsInputTime.msecs());
  return (itsInputTime == itsPyrTime);
}

// ######################################################################
Dims DirectFeedChannel::getMapDims() const
{ return itsMapDims; }

// ######################################################################
uint DirectFeedChannel::numSubmaps() const
{ return itsPyr.size(); }

// ######################################################################
Image<float> DirectFeedChannel::getSubmap(const uint index) const
{
  ASSERT(index < numSubmaps());
  return itsPyr[index];
}

// ######################################################################
std::string DirectFeedChannel::getSubmapName(const uint index) const
{
  ASSERT(index < numSubmaps());
  return sformat("%s lev: %d", descriptiveName().c_str(),index);
}

// ######################################################################
void DirectFeedChannel::getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const
{
  //FIXME
  LFATAL("Not implemented yet.");
}

// ######################################################################
Image<float> DirectFeedChannel::getOutput()
{
  ASSERT(itsInputTime == itsPyrTime);
  if (!itsOutputCache.initialized()) computeOutput();
  return itsOutputCache;
}


// ######################################################################
void DirectFeedChannel::computeOutput()
{
  ASSERT(itsInputTime == itsPyrTime);
  Image<float> output(getMapDims(),ZEROS);

  for (uint i = 0; i < itsPyr.size(); ++i)
    {
      Image<float> submap = rescale(itsPyr[i], getMapDims());
      applyMaxNorm(submap);
      output += (submap * itsCoeff[i]);
    }
  itsOutputCache = applyMaxNorm(output);
}


// ######################################################################
Image<float> DirectFeedChannel::applyMaxNorm(Image<float>& map)
{
  ASSERT(map.initialized());
  map =  maxNormalize(map, itsOutputRangeMin.getVal(),
                      itsOutputRangeMax.getVal(),
                      itsNormType.getVal());
  return map;
}

// ######################################################################
void DirectFeedChannel::killCaches()
{
  itsOutputCache.freeMem();
  itsPyrTime = SimTime::SECS(-1.0);
}


// ######################################################################
// ##### MultiDirecFeedChannel
// ######################################################################

// ######################################################################
MultiDirectFeedChannel::MultiDirectFeedChannel(OptionManager& mgr,
                                               uint num)
  : ComplexChannel(mgr,"MultiDirectFeed","MultiDirectFeed",MULTIDIRECTFEED)
{
  setNumChans(num);
}

MultiDirectFeedChannel::~MultiDirectFeedChannel()
{}

// ######################################################################
void MultiDirectFeedChannel::setNumChans(uint num)
{
  if (num == 0) LFATAL("Need at least one subchannel.");

  // Same number that we already have? Nothing to do then
  if (num == numChans()) return;

  // Need to add additional channels?
  if (num > numChans())
    for (uint i = numChans(); i < num; ++i)
      addSubChan(makeSharedComp(new DirectFeedChannel(getManager(),i)));

  // Need to remove channels?
  else
    for (uint i = numChans()-1; i >= num; --i)
      removeSubComponent(i);
}

// ######################################################################
void MultiDirectFeedChannel::inputPyramid(uint chanNum,
                                          const ImageSet<float>& pyramid,
                                          const SimTime& t)
{
  ASSERT(chanNum < numChans());
  directFeed(chanNum).inputPyramid(pyramid,t);
}

// ######################################################################
void MultiDirectFeedChannel::inputPyramidVector(const PyrVec& pvec,
                                                const SimTime& t)
{
  ASSERT (pvec.size() == numChans());
  for (uint i = 0; i < numChans(); ++i)
    inputPyramid(i,pvec[i],t);
}

// ######################################################################
DirectFeedChannel& MultiDirectFeedChannel::directFeed(uint idx)
{
  ASSERT(idx < numChans());
  return *(dynCast<DirectFeedChannel>(subChan(idx)));
}


// ######################################################################
Image<float> MultiDirectFeedChannel::combineOutputs()
{
  ASSERT(numChans() > 0);

  if (numChans() == 1) return directFeed(0).getOutput();

  Image<float> output(getMapDims(),ZEROS);

  for (uint i = 0; i < numChans(); ++i)
    output += directFeed(i).getOutput();

  return  maxNormalize(output, itsOutputRangeMin.getVal(),
                       itsOutputRangeMax.getVal(),
                       itsNormType.getVal());
}


// ######################################################################
void MultiDirectFeedChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.colorFloat().initialized()
         || inframe.grayFloat().initialized());
  for (uint i = 0; i < numChans(); ++i)
    directFeed(i).input(inframe);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
