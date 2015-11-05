/*!@file SIFT/ScaleSpace.H ScaleSpace computation for SIFT obj recognition */

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
// Primary maintainer for this file: James Bonaiuto <bonaiuto@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/ScaleSpace.H $
// $Id: ScaleSpace.H 10423 2008-11-12 22:26:07Z mviswana $
//

#ifndef SCALESPACE_H_DEFINED
#define SCALESPACE_H_DEFINED

#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "SIFT/Histogram.H"
#include "rutz/shared_ptr.h"

#include <vector>
class Keypoint;


//! ScaleSpace is like a Gaussian pyramid but with several images per level
/*! ScaleSpace is a variation of a Gaussian pyramid, where we have
  several images convolved by increasingly large Gaussian filters for
  each level (pixel size). So, while in the standard dyadic Gaussian
  pyramid we convolve an image at a given resolution, then decimate by
  a factor 2 it to yield the next lower resolution, here we convolve
  at each resolution several times with increasingly large Gaussians
  and decimate only once every several images. This ScaleSpace class
  represents the several convolved images that are obtained for one
  resolution (one so-called octave). Additionally, ScaleSpace provides
  machinery for SIFT keypoint extraction. In the SIFT algorithm we use
  several ScaleSpace objects for several image resolutions.

  Some of the code here based on the Hugin panorama software - see
  http://hugin.sourceforge.net - but substantial modifications have
  been made, espacially by carefully going over David Lowe's IJCV 2004
  paper. Some of the code also comes from http://autopano.kolor.com/
  but substantial debugging has also been made. */
class ScaleSpace
{
public:
        //Channles def
        enum ChannlesDef {LUM_CHANNEL = 0, RG_CHANNEL = 1, BY_CHANNEL = 2};

  //! Constructor
  /*! Take an input image and create a bunch of Gaussian-convolved
    versions of it as well as a bunch of DoG versions of it.  We
    obtain the following images, given 's' images per octave and a
    baseline Gaussian sigma of 'sigma'. In all that follows, [*]
    represents the convolution operator.

    We want that once we have convolved the image 's' times we end up
    with an images blurred by 2 * sigma. It is well known that
    convolving several times by Gaussians is equivalent to convolving
    once by a single Gaussian whose variance (sigma^2) is the sum of
    all the variances of the other Gaussians. It is easy to show that
    we then want to convolve each image (n >= 1) by a Gaussian of
    stdev std(n) = sigma(n-1) * sqrt(k*k - 1.0) where k=2^(1/s) and
    sigma(n) = sigma * k^n, which means that each time we multiply the
    total effective sigma by k. So, in summary, for every n >= 1,
    std(n) = sigma * k^(n-1) * sqrt(k*k - 1.0).

    For our internal Gaussian imageset, we get s+3 images; the reason
    for that is that we want to have all the possible DoGs for that
    octave:

    blur[  0] =                           in [*] G(sigma)
    blur[  1] = blur[0]   [*] G(std(1)) = in [*] G(k*sigma)
    blur[  2] = blur[1]   [*] G(std(2)) = in [*] G(k^2*sigma)
    ...
    blur[  s] = blur[s-1] [*] G(std(s)) = in [*] G(k^s*sigma) = in[*]G(2*sigma)
    blur[s+1] = blur[s]   [*] G(std(s+1)) = in [*] G(2*k*sigma)
    blur[s+2] = blur[s+1] [*] G(std(s+2)) = in [*] G(2*k^2*sigma)

    For our internal DoG imageset, we just take differences between
    two adjacent images in the blurred imageset, yielding s+2 images
    such that:

    dog[  0] = in [*] (G(k*sigma) - G(sigma))
    dog[  1] = in [*] (G(k^2*sigma) - G(k*sigma))
    dog[  2] = in [*] (G(k^3*sigma) - G(k^2*sigma))
    ...
    dog[  s] = in [*] (G(2*sigma) - G(k^(s-1)*sigma))
    dog[s+1] = in [*] (G(2*k*sigma) - G(2*sigma))

    The reason why we need to compute dog[s+1] is that to find
    keypoints we will look for extrema in the scalespace, which
    requires comparing the dog value at a given level (1 .. s) to the
    dog values at levels immediately above and immediately below.

    To chain ScaleSpaces, you should construct a first one from your
    original image, after you have ensured that it has a blur of
    'sigma'. Then do a getTwoSigmaImage(), decimate the result using
    decXY(), and construct your next ScaleSpace directly from
    that. Note that when you chain ScaleSpace objects, the effective
    blurring accelerates: the first ScaleSpace goes from sigma to
    2.0*sigma, the second from 2.0*sigma to 4.0*sigma, and so on.

    @param in the input image.
    @param octscale baseline octave scale for this scalespace compared
    to the size of our original image. For example, if 'in' has been
    decimated by a factor 4 horizontally and vertically relatively to
    the original input image, octscale should be 4.0. This is used
    later on when assembling keypoint descriptors, so that we remember
    to scale the coordinates to the keypoints from our coordinates
    (computed in a possibly reduced input image) back to the
    coordinates of the original input image.
    @param s number of Gaussian scales per octave.
    @param sigma baseline Gaussian sigma to use. */
  ScaleSpace(const ImageSet<float>& in, const float octscale,
             const int s = 3, const float sigma = 1.6F, bool useColor=false);

  //! Destructor
  ~ScaleSpace();

  //! Get the image blurred by 2*sigma
  /*! This image can then be downscaled and fed as input to the next
    ScaleSpace. */
  Image<float> getTwoSigmaImage(int channel) const;

  //! Find the keypoints in the previously provided picture
  /*! The ScaleSpace will be searched for good SIFT keypoints. Any
    keypoint found will be pushed to the back of the provided
    vector. Note that the vector is not cleared, so if you pass a
    non-empty vector, new keypoints will just be added to is. Returns
    the number of keypoints added to the vector of keypoints. */
  uint findKeypoints(std::vector< rutz::shared_ptr<Keypoint> >& keypoints);

  //! Get the keypoints in the previously provided picture
  /*! The ScaleSpace will extract keypoints in a grid setup. Those
    keypoints will be pushed to the back of the provided
    vector. Note that the vector is not cleared, so if you pass a
    non-empty vector, new keypoints will just be added to it. Returns
    the number of keypoints added to the vector of keypoints. */
  uint getGridKeypoints(std::vector<rutz::shared_ptr<Keypoint> >& keypoints);

  //! Get number of blurred images
  uint getNumBlurredImages() const;

  //! Get a blurred image
  Image<float> getBlurredImage(const uint idx) const;

  //! Get number of DoG images
  uint getNumDoGImages() const;

  //! Get a DoG image
  Image<float> getDoGImage(const uint idx) const;

  //! Get a keypoint image for saliency submap
  Image<float> getKeypointImage(std::vector< rutz::shared_ptr<Keypoint> >& keypoints);

private :
  float itsOctScale;        // scale factor relative to original image
  float itsSigma;           // our baseline Gaussian sigma
  ImageSet<float> itsLumBlur;  // gaussian blurred pictures for lum
  ImageSet<float> itsRGBlur;  // gaussian blurred pictures for rg
  ImageSet<float> itsBYBlur;  // gaussian blurred pictures for by
  ImageSet<float> itsDog;   // difference of gaussian pictures
  bool itsUseColor;                         // Should we add color?

  // return true, if it's a max or min in the local 3x3 matrix
  bool checkForMinMax(const int x, const int y, const Image<float> & im0,
                      const Image<float> & im1, const Image<float> & im2)
    const;

  // fits the paraboloid in (X, Y, S) space : 3D Hessian matrix inversion
  Image<float> fit3D(const int x, const int y, const int s, Image<float>& dDog,
                     float& dX2, float& dY2, float& dXdY) const;

  // Localization and pruning of keypoint
  //   This is really the heart of the SIFT algorithm
  //     - 3D interpolation to find the real position of the point in (x, y, s)
  //       space
  //     - handling of already done position
  //     - feature vector calculation
  //     - returns number of keypoints found and added
  uint accurateLocalizationAndPruning(const int x, const int y, const int s,
                                      Image<byte>& analyzed,
                                      const ImageSet<float>& gradmag,
                                      const ImageSet<float>& gradori,
                                      std::vector< rutz::shared_ptr<Keypoint> >&
                                      keypoints);

  void calculateOrientationVector(const float x, const float y, const float s,
                  const ImageSet<float>& gradmag, const ImageSet<float>& gradorie, Histogram& OV);

  uint createVectorsAndKeypoints(const float x, const float y, const float s, const float dogmag,
                  const ImageSet<float>& gradmag, const ImageSet<float>& gradorie,
                  std::vector < rutz::shared_ptr<Keypoint> >& keypoints, Histogram& OV);


  // create Keypoint(s) at a verified good location and add them to a
  // VisualObject. Returns number of Keypoints added.
  uint createKeypoints(const float x, const float y, const float s,
                       const float dogmag, const ImageSet<float>& gradmag,
                       const ImageSet<float>& gradorie,
                       std::vector< rutz::shared_ptr<Keypoint> >& keypoints);
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
