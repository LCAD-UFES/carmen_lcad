/*!@file Channels/InputFrame.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/InputFrame.C $
// $Id: InputFrame.C 14600 2011-03-14 21:52:47Z dberg $
//

#ifndef CHANNELS_INPUTFRAME_C_DEFINED
#define CHANNELS_INPUTFRAME_C_DEFINED

#include "Channels/InputFrame.H"

#include "Image/ColorOps.H"
#include "Util/Assert.H"
#include "Util/JobWithSemaphore.H"
#include "Util/MainJobServer.H"

#include <pthread.h>
#include <vector>

// ######################################################################
struct RgbByteToLuminanceJob : public JobWithSemaphore
{
  RgbByteToLuminanceJob(const PixRGB<byte>* in_,
                        const PixRGB<byte>* end_,
                        float* lumout_)
    :
    in(in_), end(end_),
    lumout(lumout_)
  {}

  virtual ~RgbByteToLuminanceJob()
  {}

  virtual void run()
  {
    const PixRGB<byte>* in_ = in;
    const PixRGB<byte>* const end_ = end;
    float* lumout_ = lumout;

    const float one_third = 1.0f / 3.0f;

    while (in_ != end_)
      {
        *lumout_++ = one_third * (in_->p[0] + in_->p[1] + in_->p[2]);
        ++in_;
      }

    this->markFinished();
  }

  virtual const char* jobType() const { return "RgbByteToLuminanceJob"; }

  const PixRGB<byte>* const in;
  const PixRGB<byte>* const end;
  float* const lumout;
};

// ######################################################################
struct RgbByteToFloatJob : public JobWithSemaphore
{
  RgbByteToFloatJob(const PixRGB<byte>* in_,
                    const PixRGB<byte>* end_,
                    PixRGB<float>* rgbout_)
    :
    in(in_), end(end_),
    rgbout(rgbout_)
  {}

  virtual ~RgbByteToFloatJob()
  {}

  virtual void run()
  {
    const PixRGB<byte>* in_ = in;
    const PixRGB<byte>* const end_ = end;
    PixRGB<float>* rgbout_ = rgbout;

    while (in_ != end_)
      *rgbout_++ = PixRGB<float>(*in_++);

    this->markFinished();
  }

  virtual const char* jobType() const { return "RgbByteToFloatJob"; }

  const PixRGB<byte>* const in;
  const PixRGB<byte>* const end;
  PixRGB<float>* const rgbout;
};

// ######################################################################
InputFrame::InputFrame()
{}

// ######################################################################
InputFrame InputFrame::fromRgb(const Image<PixRGB<byte> >* in,
                               SimTime t,
                               const Image<byte>* clip,
                               const rutz::shared_ptr<PyramidCache<float> >& cache,
                               bool disableCache)
{
  ASSERT(in != 0); ASSERT(in->initialized());

  InputFrame result;

  result.itsTime = t;
  result.itsDims = in->getDims();
  if (clip != 0)
    result.itsClipMask = *clip;
  else
    result.itsClipMask = Image<byte>();
  result.itsColorByte = *in;

  result.itsColorFloat = Image<PixRGB<float> >(); // will be generated on-demand in colorFloat()
  result.itsGrayFloat = Image<float>(in->getDims(), NO_INIT);
  result.itsPyrCache =
    disableCache
    ? rutz::shared_ptr<PyramidCache<float> >(/* null */)
    : cache.get() != 0
    ? cache
    : rutz::shared_ptr<PyramidCache<float> >(new PyramidCache<float>);

  result.itsPyrCacheRgb = rutz::shared_ptr<PyramidCache<PixRGB<float> > >(/* null */);

  const int sz = in->getSize();

  JobServer& srv = getMainJobServer();

  const unsigned int ntiles = srv.getParallelismHint();

  LDEBUG("ntiles = %u", ntiles);

  std::vector<rutz::shared_ptr<RgbByteToLuminanceJob > > jobs;

  for (unsigned int i = 0; i < ntiles; ++i)
    {
      const int start = (i*sz)/ntiles;
      const int end = ((i+1)*sz)/ntiles;

      jobs.push_back
        (rutz::make_shared(new RgbByteToLuminanceJob
                           (in->getArrayPtr() + start,
                            in->getArrayPtr() + end,
                            result.itsGrayFloat.getArrayPtr() + start)));

      srv.enqueueJob(jobs.back());
    }

  for (size_t i = 0; i < jobs.size(); ++i)
    jobs[i]->wait();

  return result;
}

// ######################################################################
InputFrame InputFrame::fromRgbFloat(const Image<PixRGB<float> >* col,
                                    SimTime t,
                                    const Image<byte>* clip,
                                    const rutz::shared_ptr<PyramidCache<float> >& cache,
                                    bool disableCache)
{
  ASSERT(col != 0); ASSERT(col->initialized());

  InputFrame result;

  result.itsTime = t;
  result.itsDims = col->getDims();
  if (clip != 0)
    result.itsClipMask = *clip;
  else
    result.itsClipMask = Image<byte>();
  result.itsColorByte = Image<PixRGB<byte> >();
  result.itsColorFloat = *col;
  result.itsGrayFloat = luminance(result.itsColorFloat);
  result.itsPyrCache =
    disableCache
    ? rutz::shared_ptr<PyramidCache<float> >(/* null */)
    : cache.get() != 0
    ? cache
    : rutz::shared_ptr<PyramidCache<float> >(new PyramidCache<float>);
  
  result.itsPyrCacheRgb = rutz::shared_ptr<PyramidCache<PixRGB<float> > >(/* null */);
    
  return result;
}

// ######################################################################
InputFrame InputFrame::fromGrayFloat(const Image<float>* bw,
                                     SimTime t,
                                     const Image<byte>* clip,
                                     const rutz::shared_ptr<PyramidCache<float> >& cache,
                                     bool disableCache)
{
  ASSERT(bw != 0); ASSERT(bw->initialized());

  InputFrame result;

  result.itsTime = t;
  result.itsDims = bw->getDims();
  if (clip != 0)
    result.itsClipMask = *clip;
  else
    result.itsClipMask = Image<byte>();
  result.itsColorByte = Image<PixRGB<byte> >();
  result.itsColorFloat = Image<PixRGB<float> >();
  result.itsGrayFloat = *bw;
  result.itsPyrCache =
    disableCache
    ? rutz::shared_ptr<PyramidCache<float> >(/* null */)
    : cache.get() != 0
    ? cache
    : rutz::shared_ptr<PyramidCache<float> >(new PyramidCache<float>);

  result.itsPyrCacheRgb = rutz::shared_ptr<PyramidCache<PixRGB<float> > >(/* null */);

  return result;
}
// ######################################################################
InputFrame InputFrame::fromRgbAndGrayFloat(const Image<PixRGB<byte> >* rgbb,
                                           const Image<PixRGB<float> >* rgbf,
                                           const Image<float>* bw,
                                           SimTime t,
                                           const Image<byte>* clip,
                                           const rutz::shared_ptr<PyramidCache<float> >& cache,
                                           bool disableCache)
{
  ASSERT(rgbb != 0); ASSERT(rgbb->initialized());
  ASSERT(rgbf != 0); ASSERT(rgbf->initialized());
  ASSERT(bw != 0); ASSERT(bw->initialized());

  if (rgbb->getDims() != rgbf->getDims())
    LFATAL("Color-byte and color-float inputs must have the same dimensions "
           "(got %dx%d color-byte and %dx%d color-float)",
           rgbb->getWidth(), rgbb->getHeight(),
           rgbf->getWidth(), rgbf->getHeight());

  if (rgbf->getDims() != bw->getDims())
    LFATAL("Color and grayscale inputs must have the same dimensions "
           "(got %dx%d color and %dx%d grayscale)",
           rgbf->getWidth(), rgbf->getHeight(),
           bw->getWidth(), bw->getHeight());

  InputFrame result;

  result.itsTime = t;
  result.itsDims = rgbb->getDims();
  if (clip != 0)
    result.itsClipMask = *clip;
  else
    result.itsClipMask = Image<byte>();
  result.itsColorByte = *rgbb;
  result.itsColorFloat = *rgbf;
  result.itsGrayFloat = *bw;
  result.itsPyrCache =
    disableCache
    ? rutz::shared_ptr<PyramidCache<float> >(/* null */)
    : cache.get() != 0
    ? cache
    : rutz::shared_ptr<PyramidCache<float> >(new PyramidCache<float>);

  result.itsPyrCacheRgb = rutz::shared_ptr<PyramidCache<PixRGB<float> > >(/* null */);

  return result;
}
// ######################################################################
InputFrame InputFrame::fromRgbDepth(const Image<PixRGB<byte> >* in,
                                    const Image<uint16>* depth,
                                    SimTime t,
                                    const Image<byte>* clip,
                                    const rutz::shared_ptr<PyramidCache<float> >& cache,
                                    bool disableCache)
{
  ASSERT(in != 0); ASSERT(in->initialized());
  ASSERT(depth != 0); ASSERT(depth->initialized());

  InputFrame result;

  result.itsTime = t;
  result.itsDims = in->getDims();
  if (clip != 0)
    result.itsClipMask = *clip;
  else
    result.itsClipMask = Image<byte>();
  result.itsColorByte = *in;

  result.itsDepthImage = *depth;

  result.itsColorFloat = Image<PixRGB<float> >(); // will be generated on-demand in colorFloat()
  result.itsGrayFloat = Image<float>(in->getDims(), NO_INIT);
  result.itsPyrCache =
    disableCache
    ? rutz::shared_ptr<PyramidCache<float> >(/* null */)
    : cache.get() != 0
    ? cache
    : rutz::shared_ptr<PyramidCache<float> >(new PyramidCache<float>);

  result.itsPyrCacheRgb = rutz::shared_ptr<PyramidCache<PixRGB<float> > >(/* null */);

  const int sz = in->getSize();

  JobServer& srv = getMainJobServer();

  const unsigned int ntiles = srv.getParallelismHint();

  LDEBUG("ntiles = %u", ntiles);

  std::vector<rutz::shared_ptr<RgbByteToLuminanceJob > > jobs;

  for (unsigned int i = 0; i < ntiles; ++i)
    {
      const int start = (i*sz)/ntiles;
      const int end = ((i+1)*sz)/ntiles;

      jobs.push_back
        (rutz::make_shared(new RgbByteToLuminanceJob
                           (in->getArrayPtr() + start,
                            in->getArrayPtr() + end,
                            result.itsGrayFloat.getArrayPtr() + start)));

      srv.enqueueJob(jobs.back());
    }

  for (size_t i = 0; i < jobs.size(); ++i)
    jobs[i]->wait();

  return result;
}

// ######################################################################
const Image<PixRGB<float> >& InputFrame::colorFloat() const
{
  if (!itsColorFloat.initialized() && itsColorByte.initialized())
    {
      itsColorFloat = Image<float>(itsColorByte.getDims(), NO_INIT);

      const int sz = itsColorByte.getSize();

      JobServer& srv = getMainJobServer();

      const unsigned int ntiles = srv.getParallelismHint();

      LDEBUG("ntiles = %u", ntiles);

      std::vector<rutz::shared_ptr<RgbByteToFloatJob > > jobs;

      for (unsigned int i = 0; i < ntiles; ++i)
        {
          const int start = (i*sz)/ntiles;
          const int end = ((i+1)*sz)/ntiles;

          jobs.push_back
            (rutz::make_shared(new RgbByteToFloatJob
                               (itsColorByte.getArrayPtr() + start,
                                itsColorByte.getArrayPtr() + end,
                                const_cast<PixRGB<float>*>
                                (itsColorFloat.getArrayPtr() + start))));

          srv.enqueueJob(jobs.back());
        }

      for (size_t i = 0; i < jobs.size(); ++i)
        jobs[i]->wait();
    }

  return itsColorFloat;
}

const rutz::shared_ptr<PyramidCache<float> > InputFrame::emptyCache;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INPUTFRAME_C_DEFINED
