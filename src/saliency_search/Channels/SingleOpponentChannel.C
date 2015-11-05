/*!@file Channels/SingleOpponentChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SingleOpponentChannel.C $
// $Id: SingleOpponentChannel.C 10746 2009-02-03 07:09:00Z itti $
//

#ifndef CHANNELS_SINGLEOPPONENTCHANNEL_C_DEFINED
#define CHANNELS_SINGLEOPPONENTCHANNEL_C_DEFINED

#include "Channels/SingleOpponentChannel.H"

#include "Image/ImageSetOps.H"
#include "Image/PyramidOps.H"
#include "rutz/trace.h"

// ######################################################################
SingleOpponentChannel::
SingleOpponentChannel(OptionManager& mgr,
                      const std::string& descrName,
                      const std::string& tagName,
                      const VisualFeature vs,
                      rutz::shared_ptr<PyrBuilder<float> > pyr)
  :
  SingleChannel(mgr, descrName, tagName, vs, pyr),
  itsPq2()
{}

// ######################################################################
void SingleOpponentChannel::reset1()
{
  itsPq2.clear();
  SingleChannel::reset1();
}

// ######################################################################
Image<float> SingleOpponentChannel::
centerSurround(const uint cntrlev, const uint surrlev) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(itsLevelSpec.getVal().csOK(cntrlev, surrlev));
  if (this->hasPyramid() == false) LFATAL("I have no input pyramid yet!");
  if (itsPq2.empty()) LFATAL("I have no second input pyramid yet!");

  // in the single-opponent version, we do pyr[cntrlev] - pyr2[surrlev]

  // do basic center-surround for the front (latest) pyramid in queue:
  const ImageSet<float>& cpyr = this->pyramid(0);
  const ImageSet<float>& spyr = itsPq2.front().pyr;
  const double t = this->pyramidTime(0).secs();

  // compute single-opponent center-surround:
  Image<float> cs =
    ::centerSurroundSingleOpponent(cpyr, spyr, cntrlev, surrlev,
                                   itsTakeAbs.getVal(),
                                   &(this->clipPyramid()));

  // do additional processing with other pyramids in queue:
  for (uint i = 1; i < itsPq2.size(); ++i)
    {
      const ImageSet<float>& cpyr2 = this->pyramid(i);
      const ImageSet<float>& spyr2 = itsPq2[i].pyr;
      const double t2 = this->pyramidTime(i).secs();

      // compute a decay factor based on how old the second pyramid is
      // compared to the latest one:
      float fac = exp( (t2 - t) * itsTimeDecay.getVal());
      cs +=
        ::centerSurroundDiffSingleOpponent(cpyr, cpyr2, spyr, spyr2,
                                           cntrlev, surrlev,
                                           itsTakeAbs.getVal(),
                                           &(this->clipPyramid()))
        * fac;
    }
  return cs;
}

// ######################################################################
void SingleOpponentChannel::
centerSurround(const uint cntrlev, const uint surrlev,
               Image<float>& pos, Image<float>& neg) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(itsLevelSpec.getVal().csOK(cntrlev, surrlev));
  if (this->hasPyramid() == false) LFATAL("I have no output yet!");
  if (itsPq2.empty()) LFATAL("I have no second input pyramid yet!");

  // do basic center-surround for the front (latest) pyramid in queue:
  const ImageSet<float>& cpyr = this->pyramid(0);
  const ImageSet<float>& spyr = itsPq2.front().pyr;
  const double t = this->pyramidTime(0).secs();

  // compute center-surround:
  ::centerSurroundSingleOpponent(cpyr, spyr, cntrlev, surrlev,
                                 pos, neg, &(this->clipPyramid()));

  // do additional processing with other pyramids in queue:
  for (uint i = 1; i < itsPq2.size(); ++i)
    {
      const ImageSet<float>& cpyr2 = this->pyramid(i);
      const ImageSet<float>& spyr2 = itsPq2[i].pyr;
      double t2 = this->pyramidTime(i).secs();

      // compute a decay factor based on how old the second pyramid is
      // compared to the latest one:
      float fac = exp( (t2 - t) * itsTimeDecay.getVal());
      Image<float> pos2, neg2;
      ::centerSurroundDiffSingleOpponent(cpyr, cpyr2, spyr, spyr2,
                                         cntrlev, surrlev,
                                         pos2, neg2,
                                         &(this->clipPyramid()));
      pos += pos2 * fac;
      neg += neg2 * fac;
    }
}

// ######################################################################
void SingleOpponentChannel::getFeatures(const Point2D<int>& locn,
                                        std::vector<float>& mean) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!this->outputAvailable())
    {
      CLDEBUG("I have no input pyramids yet -- RETURNING ZEROS");
      for (uint idx = 0; idx < numSubmaps(); idx ++) mean.push_back(0.0F);
      return;
    }

  // The coordinates we receive are at the scale of the original
  // image, and we will need to rescale them to the size of the
  // various submaps we read from. The first image in our first
  // pyramid has the dims of the input:
  const ImageSet<float>& pyr = this->pyramid(0);
  const ImageSet<float>& pyr2 = itsPq2.front().pyr;
  const Dims indims = this->getInputDims();

  for (uint idx = 0; idx < numSubmaps(); idx ++)
    {
      // get center and surround scales for this submap index:
      uint clev = 0, slev = 0;
      itsLevelSpec.getVal().indexToCS(idx, clev, slev);

      // read center value with bilinear interpolation:
      ASSERT(pyr[clev].initialized());
      const float cval = pyr[clev].getValInterpScaled(locn, indims);

      // read surround value with bilinear interpolation:
      ASSERT(pyr2[slev].initialized());
      const float sval = pyr2[slev].getValInterpScaled(locn, indims);

      // compute center - surround and take absolute value if our
      // channel wants that:
      float val = cval - sval; if (itsTakeAbs.getVal()) val = fabs(val);

      // store the submap value at the chosen location:
      mean.push_back(val);
    }
}

// ######################################################################
void SingleOpponentChannel::
getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                 std::vector<std::vector<float> > *mean,
                 int *count) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!this->outputAvailable())
    {
      CLDEBUG("I have no input pyramids yet -- RETURNING ZEROS");
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
  const ImageSet<float>& pyr = this->pyramid(0);
  const ImageSet<float>& pyr2 = itsPq2.front().pyr;
  const Dims indims = this->getInputDims();
  uint sm = this->numSubmaps();
  for (uint idx = 0; idx < sm; ++idx)
    {
    // get center and surround scales for this submap index:
    uint clev = 0, slev = 0;
    itsLevelSpec.getVal().indexToCS(idx, clev, slev);
    std::vector<Point2D<int>*>::iterator ilocn = locn->begin();
    std::vector<std::vector<float> >::iterator imean = mean->begin();

    for(int i = 0; i < *count; ++i, ++ilocn, ++imean)
      {
        // read center value with bilinear interpolation:
        ASSERT(pyr[clev].initialized());
        const float cval = pyr[clev].getValInterpScaled(**ilocn, indims);

        // read surround value with bilinear interpolation:
        ASSERT(pyr2[slev].initialized());
        const float sval = pyr2[slev].getValInterpScaled(**ilocn, indims);

        const float val = cval - sval;

        // store the submap value at the chosen location:
        imean->push_back(itsTakeAbs.getVal()
                         ? fabs(val)
                         : val);
    }
  }
}

// ######################################################################
void SingleOpponentChannel::
singleOpponentInput(const Dims& dims,
                    const ImageSet<float>& centerPyr,
                    const ImageSet<float>& surroundPyr,
                    const SimTime& t,
                    const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->killCaches();
  this->setClipPyramid(clipMask);
  this->storePyramid(centerPyr, t);
  this->storePyramid2(surroundPyr, t);
  this->setInputDims(dims);
}

// ######################################################################
void SingleOpponentChannel::storePyramid2(const ImageSet<float>& p,
                                          const SimTime& t)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // check that inputs are not coming in too fast, as we may otherwise
  // run into trouble with those channels that care about the
  // inter-frame delay:
  if (itsPq2.size() && (t - itsPq2.front().t).secs() < 0.002F) // 500fps is the max
    CLFATAL("Inputs coming in too fast! -- BAILING OUT");

  // load our pyramid into our front pyramid:
  itsPq2.push_front(TPyr(p, t));

  // truncate the pyramid queue if necessary:
  while(int(itsPq2.size()) > itsQlen.getVal()) itsPq2.pop_back();

  // We only want dyadic pyramids here:
  ASSERT(isDyadic(itsPq2.front().pyr.subSet
                  (itsLevelSpec.getVal().levMin(),
                   itsLevelSpec.getVal().maxDepth())));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SINGLEOPPONENTCHANNEL_C_DEFINED
