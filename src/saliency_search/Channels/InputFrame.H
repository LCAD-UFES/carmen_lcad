/*!@file Channels/InputFrame.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/InputFrame.H $
// $Id: InputFrame.H 14631 2011-03-23 19:02:41Z dberg $
//

#ifndef CHANNELS_INPUTFRAME_H_DEFINED
#define CHANNELS_INPUTFRAME_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/PyramidCache.H"
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"

class InputFrame
{
public:
  //! Uninitialized constructor
  InputFrame();

  // default dtor, copy, assignment OK

  // ######################################################################
  /*! @name Pseudo-constructors

      There are several ways to construct an input frame. These are
      provided to allow maximum efficiency, so that only the minimum
      required information must be passed in. The size of the input
      image(s) can later be queried with getDims(). The timestamp
      parameter (t, in seconds) is used to compute transient responses
      to flickering stimuli. */
  //@{

  //! Initialize to a new input frame
  /*! @param in input image -- this is a pointer for the input
      argument so as to avoid unwanted implicit conversion -- we want
      to really initialize from a byte-rgb image, and not a float-rgb
      image, nor a byte-grayscale image
  */
  static InputFrame fromRgb(const Image<PixRGB<byte> >* in,
                            SimTime t = SimTime::ZERO(),
                            const Image<byte>* clip = 0,
                            const rutz::shared_ptr<PyramidCache<float> >& cache = emptyCache,
                            bool disableCache = false);

  static InputFrame fromRgbFloat(const Image<PixRGB<float> >* col,
                                 SimTime t = SimTime::ZERO(),
                                 const Image<byte> * clip = 0,
                                 const rutz::shared_ptr<PyramidCache<float> >& cache = emptyCache,
                                 bool disableCache = false);

  static InputFrame fromGrayFloat(const Image<float>* bw,
                                  SimTime t = SimTime::ZERO(),
                                  const Image<byte> * clip = 0,
                                  const rutz::shared_ptr<PyramidCache<float> >& cache = emptyCache,
                                  bool disableCache = false);

  static InputFrame fromRgbAndGrayFloat(const Image<PixRGB<byte> >* rgbb,
                                        const Image<PixRGB<float> >* rgbf,
                                        const Image<float>* bw,
                                        SimTime t = SimTime::ZERO(),
                                        const Image<byte> * clip = 0,
                                        const rutz::shared_ptr<PyramidCache<float> >& cache = emptyCache,
                                        bool disableCache = false);

  static InputFrame fromRgbDepth(const Image<PixRGB<byte> >* in,
                                 const Image<uint16>* depth,
                                 SimTime t = SimTime::ZERO(),
                                 const Image<byte>* clip = 0,
                                 const rutz::shared_ptr<PyramidCache<float> >& cache = emptyCache,
                                 bool disableCache = false);
  //@}

  SimTime time() const { return itsTime; }

  const Dims& getDims() const { return itsDims; }
  const int getWidth() const { return itsDims.w(); }
  const int getHeight() const { return itsDims.h(); }

  const Image<byte>& clipMask() const { return itsClipMask; }

  const Image<PixRGB<byte> >& colorByte() const { return itsColorByte; }
  const Image<PixRGB<float> >& colorFloat() const;
  const Image<float>& grayFloat() const { return itsGrayFloat; }

  const rutz::shared_ptr<PyramidCache<float> >& pyrCache() const { return itsPyrCache; }

  static const rutz::shared_ptr<PyramidCache<float> > emptyCache;

  const Image<uint16> getDepthImage() const { return itsDepthImage; }

  const bool hasDepthImage() const { return itsDepthImage.initialized(); }

  void setPyrCacheRgb(const rutz::shared_ptr<PyramidCache<PixRGB<float> > >& pyr)
  { itsPyrCacheRgb = pyr; }

  void setPyrCache(const rutz::shared_ptr<PyramidCache<float> >& pyr)
  { itsPyrCache = pyr; }

private:
  SimTime itsTime;
  Dims itsDims;
  Image<byte> itsClipMask;
  Image<PixRGB<byte> > itsColorByte;
  mutable Image<PixRGB<float> > itsColorFloat;
  Image<float> itsGrayFloat;
  rutz::shared_ptr<PyramidCache<float> > itsPyrCache;
  rutz::shared_ptr<PyramidCache<PixRGB<float> > > itsPyrCacheRgb;
  Image<uint16> itsDepthImage;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INPUTFRAME_H_DEFINED
