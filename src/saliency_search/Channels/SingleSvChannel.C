/*!@file Channels/SingleSvChannel.C Channel for a single stream of space variant processing. */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/Channels/SingleSvChannel.C $
// $Id: SingleSvChannel.C 13757 2010-08-04 22:11:39Z siagian $

#include "Channels/SingleSvChannel.H"
#include "SpaceVariant/SpaceVariantOpts.H"

// ######################################################################
SingleSvChannel::SingleSvChannel(OptionManager& mgr, const std::string& descrName,
                                 const std::string& tag,const VisualFeature vs,
                                 const SpaceVariantModule& spacevariantmodule) :
  SingleChannel(mgr, descrName, tag, vs, rutz::shared_ptr<PyrBuilder<float> >(/*NULL*/)),
  itsChanDims(&OPT_SpaceVariantDims, this),
  itsLevels(&OPT_SpaceVariantChanScales, this),
  itsTakeSquare("TakeSquareRoot", this, false),
  isPolarized("IsPolarized", this, false),
  itsUseSpaceVariantBoundary("UseSpaceVariantBoundary", this, true),
  itsTransform(new SpaceVariantModule(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);

//hide the options we don't care about from single channel
 hideOption(&OPT_LevelSpec);
}

// ######################################################################
SingleSvChannel::~SingleSvChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
ImageSet<float> SingleSvChannel::
computePyramid(const Image<float>& bwimg,
               const rutz::shared_ptr<PyramidCache<float> >& cache)
{
GVX_TRACE(__PRETTY_FUNCTION__);
// Compute our pyramid:
 if (!cache.is_valid())
   LFATAL("Space variant channels should always get a cache from the retina");
 
 ImageSet<float> py;
 itsTransform->transformFloatPyramid(bwimg, py, itsLevels.getVal(), cache->get());
 
 // Eliminate small values if and/or rectify the pyramid desired:
 if (itsLowThresh.getVal() > 0.0f)
   {
     if (itsRectifyPyramid.getVal())
       doLowThresh(py, itsLowThresh.getVal());
     else
       doLowThreshAbs(py, itsLowThresh.getVal());
   }
 else if (itsRectifyPyramid.getVal())
   doRectify(py);
 
 // return pyramid:
 return py;
}

// ######################################################################
Dims SingleSvChannel::getMapDims() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 return itsChanDims.getVal();
}

// ######################################################################
Image<float> SingleSvChannel::centerSurround(const uint cntrlev,
                                           const uint surrlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 LFATAL("We don't do center surround operations for this channel");
 return Image<float>();
}

// ######################################################################
void SingleSvChannel::centerSurround(const uint cntrlev, const uint surrlev,
                                   Image<float>& pos, Image<float>& neg) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 LFATAL("We don't do center surround operations for this channel");
}

// ######################################################################
std::string SingleSvChannel::getSubmapName(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 ASSERT(idx < numSubmaps());
 return sformat("%s lev ", descriptiveName().c_str(), idx);
}

// ######################################################################
std::string SingleSvChannel::getSubmapNameShort(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 ASSERT(idx < numSubmaps());
 return sformat("%s(%d)", tagName().c_str(), idx);
}

// ######################################################################
void SingleSvChannel::getFeatures(const Point2D<int>& locn,
                                std::vector<float>& mean) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

 if (!this->outputAvailable())
   {
     CLDEBUG("I have no input pyramid yet -- RETURNING ZEROS");
     for (uint idx = 0; idx < numSubmaps(); idx ++) mean.push_back(0.0F);
     return;
   }
 
 const ImageSet<float>& pyr = itsPq.front().pyr;
 const Dims indims = this->getInputDims();

  for (uint idx = 0; idx < numSubmaps(); idx ++)
    {
      // read center value with bilinear interpolation:
      ASSERT(pyr[idx].initialized());
      const float val = pyr[idx].getValInterpScaled(locn, indims);

      // store the submap value at the chosen location:
      mean.push_back(val);
    }
}

// ######################################################################
void SingleSvChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                     std::vector<std::vector<float> > *mean,
                                     int *count) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

 if (!this->outputAvailable())
   {
     CLDEBUG("I have no input pyramid yet -- RETURNING ZEROS");
     for (uint idx = 0; idx < numSubmaps(); idx ++)
       {
         std::vector<std::vector<float> >::iterator imean = mean->begin();
         for(int i = 0; i < *count; i++, ++imean)
           imean->push_back(0.0);
       }
     return;
   }

  // The coordinates we receive are at the scale of the original
  // image, and we will need to rescale them to the size of the
  // various submaps we read from. The first image in our first
  // pyramid has the dims of the input:
  const ImageSet<float>& pyr = itsPq.front().pyr;
  const Dims indims = this->getInputDims();
  const uint sm = numSubmaps();
  for (uint idx = 0; idx < sm; ++idx)
    {
      std::vector<Point2D<int>*>::iterator ilocn = locn->begin();
      std::vector<std::vector<float> >::iterator imean = mean->begin();

      for (int i = 0; i < *count; ++i, ++ilocn, ++imean)
        {
          // read center value with bilinear interpolation:
          ASSERT(pyr[clev].initialized());
          const float val = pyr[clev].getValInterpScaled(**ilocn, indims);
        }
    }
}

// ######################################################################
LevelSpec SingleSvChannel::getLevelSpec() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 LFATAL("Space variant channels do not use a LevelSpec");
 return itsLevelSpec.getVal();
}

// ######################################################################
void SingleSvChannel::setClipPyramid(const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // if necessary create pyramid of clipping masks
  if (clipMask.initialized())
    {
      Image<float> mask = rescale(Image<float>(clipMask)/255.0f, getMapDims());
      itsClipPyr = ImageSet<float>(maxIndex());
      for (uint ii = 0; ii < maxIndex(); ++ii)
        itsClipPyr[ii] = mask;
    }
  else
    itsClipPyr.clear();
}

// ######################################################################
void SingleSvChannel::storePyramid(const ImageSet<float>& p,const SimTime& t)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // load our pyramid into our front pyramid:
  itsPq.push_front(TPyr(p, t));

  // truncate the pyramid queue if necessary:
  while(int(itsPq.size()) > itsQlen.getVal()) itsPq.pop_back();

  ASSERT(itsPq.front().pyr.size() < this->getMaxPyrLevel());
}

// ######################################################################
Image<float> SingleSvChannel::getRawCSmap(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(itsLevelSpec.getVal().indexOK(idx));
  if (itsPq.empty()) CLFATAL("I have no input pyramid yet!");

  // determine any gain factor (possibly from a
  // ChannelFacetGainSingle). But note that we don't apply that gain
  // factor here, rather we just care to see whether we can avoid
  // computing the map if its gain factor is zero. Applying the gain
  // factors is done in combineSubmaps():
  float w = 1.0;
  if (hasFacet<ChannelFacetGainSingle>())
    w = getFacet<ChannelFacetGainSingle>()->getVal(idx);

  // if we have a zero weight, just return an empty image:
  if (w == 0.0f) return Image<float>(getMapDims(), ZEROS);

  Image<float> submap;

  // if using split center-surround model param from SingleChannel, 
  // compute both the positive and negative submaps for channels which 
  // are polarized, normalize them, sum them and we
  // will below further normalize the sum:
  if (itsUseSplitCS.getVal() && isPolarized.getVal())
    {
      Image<float> subpos, subneg;
      computeSubChanSplit(idx, subpos, subneg);

      // apply spatial competition for salience, independently to
      // both positive and negative parts of the submap,
      // preserving their original range rather than first
      // normalizing them to [MAXNORMMIN..MAXNORMMAX] as we
      // usually do, so that those maps who contain nothing do not
      // get artificially amplified:
      subpos = maxNormalize(subpos, 0.0f, 0.0f, itsNormType.getVal());
      subneg = maxNormalize(subneg, 0.0f, 0.0f, itsNormType.getVal());

      // the raw submap is the sum of the normalized positive and
      // negative sides:
      submap = subpos + subneg;
    }
  else    
    submap = computeSubChan(idx);

  // print some debug info if in debug mode:
  if (MYLOGVERB >= LOG_DEBUG)
    {
      float mi, ma; getMinMax(submap, mi, ma);
      LDEBUG("%s(%d,%d): raw range [%f .. %f]", tagName().c_str(),
             clev, slev, mi, ma);
    }

  return submap;
}

// ######################################################################
void SingleSvChannel::computeSubChanSplit(const uint idx, Image<float>& pos, Image<float>& neg) const
{
  //grab the front of the que
  const Image<float>& img = itsPq.front().pyr[idx];
  double t = itsPq.front().t.secs();
  
  splitPosNeg(img, pos, neg);

  else if (itsTakeSquare.getVal())
    {
      pos = squared(pos);
      neg = squared(neg);
    }
  
  // do additional processing with other pyramids in queue:
  for (uint i = 1; i < itsPq.size(); ++i)
    {
      const Image<float>& img2 = itsPq[i].pyr[idx];
      double t2 = itsPq[i].t.secs();

      // compute a decay factor based on how old the second pyramid is
      // compared to the latest one:
      float fac = exp( (t2 - t) * itsTimeDecay.getVal());
      
      const Image<float> pos2, neg2;
      splitPosNeg(img2, pos2, neg2);
      
      if (itsTakeSquare.getVal())
        {
          pos2 = squared(pos2);
          neg2 = squared(neg2);
        }
      pos += pos2 * fac;
      neg += neg2 * fac;
    }
}

// ######################################################################
Image<float> SingleSvChannel::computeSubChan(const uint idx) const
{
  //grab the front of the que
  Image<float> img = itsPq.front().pyr[idx];
  double t = itsPq.front().t.secs();
  
  if (itsTakeAbs.getVal())
    img = abs(img);
  else if (itsTakeSquare.getVal())
    img = squared(img);
  
  // do additional processing with other pyramids in queue:
  for (uint i = 1; i < itsPq.size(); ++i)
    {
      double t2 = itsPq[i].t.secs();

      // compute a decay factor based on how old the second pyramid is
      // compared to the latest one:
      float fac = exp( (t2 - t) * itsTimeDecay.getVal());
      
      if (itsTakeAbs.getVal())
        img = abs(itsPq[i].pyr[idx]) * fac;
      else if (itsTakeSquare.getVal())
        img = squared(itsPq[i].pyr[idx]) * fac;
    }
  return img;
}

// ######################################################################
int SingleSvChannel::getMinPyrLevel() const
{
  return 0;
}

// ######################################################################
int SingleSvChannel::getMaxPyrLevel() const
{
  return maxIndex();
}

// ######################################################################
void SingleSvChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 SingleSvChannel::start1();
}

// ######################################################################
uint SingleSvChannel::csToIndex(const uint centerlev, const uint surroundlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return centerlev;
}

// ######################################################################
void SingleSvChannel::indexToCS(const uint index, uint& centerlev, uint& surroundlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  centerlev = idx; surroundlev = maxIndex() + 1;
}

// ######################################################################
uint SingleSvChannel::maxIndex() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 return itsLevels.getVal().numLevels();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
