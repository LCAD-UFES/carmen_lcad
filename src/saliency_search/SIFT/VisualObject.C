/*!@file SIFT/VisualObject.C Visual Objects to be recognized */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/VisualObject.C $
// $Id: VisualObject.C 15310 2012-06-01 02:29:24Z itti $
//

#include "SIFT/VisualObject.H"
#include "SIFT/ScaleSpace.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Image/Kernels.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"

#include <algorithm>
#include <cmath>
#include <istream>
#include <ostream>

#include <cctype>

#include <unistd.h> // for unlink()

namespace
{
  bool isInteger(const std::string& s)
  {
    if (s.length() == 0) return false;

    if (s[0] != '-' && !isdigit(s[0])) return false;

    for (size_t i = 1; i < s.length(); ++i)
      if (!isdigit(s[i])) return false;

    return true;
  }
}
// ######################################################################
// functor to assist with keypoint sorting:
class lessKP
{
public:
  bool operator()(const rutz::shared_ptr<Keypoint>& x,
                  const rutz::shared_ptr<Keypoint>& y)
  { return (*x) < (*y); }
};

// here is an implementation of is_sorted() (which turns out to be a
// non-standard SGI extension to the STL and hence is not always
// available), ripped from
// http://lists.boost.org/MailArchives/boost/msg40406.php
template <class ForwardIterator, class StrictWeakOrdering>
bool myIsSorted(ForwardIterator begin, ForwardIterator end,
                StrictWeakOrdering comp)
{
  if (begin == end) return true;

  ForwardIterator next = begin;
  ++next;
  for (; next != end ; ++begin,++next) if (comp(*next, *begin)) return false;

  return true;
}

// ######################################################################
VisualObject::VisualObject(const std::string& name,
                           const std::string& imagefname,
                           const Image< PixRGB<byte> >& image,
                           const Point2D<int>& salpt,
                           const std::vector<float>& preattfeatures,
                           const std::vector< rutz::shared_ptr<Keypoint> >&
                           keypoints,
                           const bool useColor,
                           bool computeKP) :

  itsName(name), itsImageFname(imagefname), itsImage(image),
  itsKeypoints(keypoints), itsSalPoint(salpt), itsFeatures(preattfeatures),
  itsIsSorted(false), itsUseColor(useColor),itsImageLoaded(true)
{
  itsObjectSize = image.getDims();
  if(computeKP) computeKeypoints();
}

// ######################################################################
void VisualObject::computeKeypoints()
{
  // if we were given an image but no keypoints, let's extract them now:
  if (itsImage.initialized() && itsKeypoints.empty())
    {
      LDEBUG("%s: initializing ScaleSpace from %dx%d image...",
             itsName.c_str(), itsImage.getWidth(), itsImage.getHeight());

      // compute the luminance of the image:
      Image<float> lum = luminance(itsImage);

      // compute the opponent color space
      // and double the image
      Image<float> rg, by;
      if (itsUseColor){
        getRGBY(itsImage, rg, by, 25.0F);
        rg = interpolate(rg);
        by = interpolate(by);
      }

      // double the resolution:
      lum = interpolate(lum);

      const int nums = 3;        // recommended by David Lowe
      const double sigma = 1.6F; // recommended by David Lowe
      float octscale = 0.5F;     // since we doubled the image

      // To feed the first ScaleSpace in our series, apply some
      // initial blur so that the input image has an effective blur of
      // the desired sigma. We assume that the original image has a
      // blur of at least 0.5 by construction. Since its size has been
      // doubled (octscale=0.5), then that becomes 1.0. We assume that
      // the sigma=1.6 applies to the doubled image. Remember that the
      // variances add when we sequentially convolve by
      // Gaussians. Hence the additional blur we need is such that
      // sigma^2 = 1^2 + blursig^2:
      const float blursig = sqrtf(sigma * sigma - 1.0F);
      Image<float> kernel = gaussian<float>(1.0F, blursig,
                                            lum.getWidth(), 1.0F);
      kernel = kernel / float(sum(kernel));
      lum = sepFilter(lum, kernel, kernel, CONV_BOUNDARY_CLEAN);

      if (itsUseColor){
        // scale the color space
        rg = sepFilter(rg, kernel, kernel, CONV_BOUNDARY_CLEAN);
        by = sepFilter(by, kernel, kernel, CONV_BOUNDARY_CLEAN);
      }

      // let's do it:
      int iter = 0; uint numkp = 0;
      while (lum.getWidth() > 24 && lum.getHeight() > 24)
        {
          ImageSet<float> inImg(3);
          inImg[ScaleSpace::LUM_CHANNEL] = lum;

          if (itsUseColor){        // add the color spaces to the input image
            inImg[ScaleSpace::RG_CHANNEL] = rg;
            inImg[ScaleSpace::BY_CHANNEL] = by;
          }

          ScaleSpace ss(inImg, octscale, nums, sigma, itsUseColor);

          // get a bunch of keypoints out of the ScaleSpace:
          uint nkp = ss.findKeypoints(itsKeypoints);
          LDEBUG("%s: Found %d keypoints in ScaleSpace %d",
                 itsName.c_str(), nkp, iter);
          numkp += nkp;

          // get ready for next ScaleSpace:
          lum = decXY(ss.getTwoSigmaImage(ScaleSpace::LUM_CHANNEL));

          if (itsUseColor){
            rg = decXY(ss.getTwoSigmaImage(ScaleSpace::RG_CHANNEL));
            by = decXY(ss.getTwoSigmaImage(ScaleSpace::BY_CHANNEL));
          }

          ++ iter; octscale *= 2.0F;
        }

      LDEBUG("%s: Found total of %d keypoints over all ScaleSpaces.",
             itsName.c_str(), numkp);
    }
}

// ######################################################################
VisualObject::VisualObject(const VisualObject& vo)
{
  itsName = vo.itsName; itsImageFname = vo.itsImageFname;
  if (vo.itsImage.initialized()) itsImage = vo.itsImage; else itsImage.freeMem();
  itsKeypoints = vo.itsKeypoints;
  itsFeatures = vo.itsFeatures;
  itsIsSorted = vo.itsIsSorted;
}

// ######################################################################
VisualObject::~VisualObject()
{  }

// ######################################################################
void VisualObject::deleteImageFile() const
{
  if (Raster::fileExists(itsImageFname, RASFMT_PNG))
    if (unlink(itsImageFname.c_str()) == -1)
      PLERROR("Could not delete '%s' -- IGNORING", itsImageFname.c_str());
}

// ######################################################################
VisualObject& VisualObject::operator=(const VisualObject& vo)
{
  itsName = vo.itsName; itsImageFname = vo.itsImageFname;

  itsImage.freeMem();
  if (vo.itsImage.initialized()) itsImage = vo.itsImage;

  itsKeypoints = vo.itsKeypoints;
  itsFeatures = vo.itsFeatures;
  itsIsSorted = vo.itsIsSorted;

  return *this;
}

// ######################################################################
double VisualObject::getFeatureDistSq(const rutz::shared_ptr<VisualObject>& obj) const
{
  ASSERT(itsFeatures.size() == obj->itsFeatures.size());

  double distSq = 0.0;
  std::vector<float>::const_iterator
    src1 = itsFeatures.begin(), stop = itsFeatures.end(),
    src2 = obj->itsFeatures.begin();

  while (src1 != stop)
    {
      const double diff = double(*src1++) - double(*src2++);
      distSq += diff * diff;
    }

  return distSq;
}

// ######################################################################
void VisualObject::sortKeypoints()
{
  if (itsIsSorted) return; // we are already sorted

  // do the sorting:
  std::sort(itsKeypoints.begin(), itsKeypoints.end(), lessKP());
  itsIsSorted = true;
}

// ######################################################################
std::ostream& operator<<(std::ostream& os, const VisualObject& v)
{
  os<<v.itsName<<std::endl<<v.itsImageFname<<std::endl;
  if (v.itsImageFname != "NULL" && v.itsImageFname != "" && Raster::fileExists(v.itsImageFname, RASFMT_PNG) == false)
    {
      LINFO("Writing image file: %s", v.itsImageFname.c_str());
      Raster::WriteRGB(v.itsImage, v.itsImageFname, RASFMT_PNG);
    }

  if (v.itsImageFname == "NULL" || v.itsImageFname == "")
    os<<v.itsObjectSize.w()<<std::endl<<v.itsObjectSize.h()<<std::endl;

  os<<v.itsSalPoint.i<<std::endl<<v.itsSalPoint.j<<std::endl;
  const uint featureSize = v.itsFeatures.size();
  os<<featureSize<<std::endl;
  for (uint i = 0; i < featureSize; i++) os<<v.itsFeatures[i]<<' ';

  const uint keySize = v.itsKeypoints.size();
  os<<keySize<<std::endl;
  for (uint i = 0; i < keySize; i++) os<<*(v.itsKeypoints[i]);

  return os;
}

// ######################################################################
std::istream& operator>>(std::istream& is, VisualObject& v)
{

  v.createVisualObject(is, v);
  return is;
}

// ######################################################################
void VisualObject::createVisualObject
(std::istream& is, VisualObject &v, bool loadImage)
{
  is >> std::ws;
  std::getline(is, v.itsName);
  std::getline(is, v.itsImageFname);
  v.itsImageLoaded = loadImage;

  // if the passed in filename is "" -> the entry is the blank
  // then we will go to a different entry,
  // the i val of salient point (an integer)
  // if the passed in filename is a string "NULL" we also skip
  uint featureSize;
  if (v.itsImageFname != "NULL" && v.itsImageFname != "")
    {
      // only load image when the user asked for
      if (loadImage)
        {
          LINFO("Opening image file %s", v.itsImageFname.c_str());
          v.itsImage = Raster::ReadRGB(v.itsImageFname);
        }
    }
  else
    {
      LDEBUG("Image file %s not opened", v.itsImageFname.c_str());
      v.itsImageFname = std::string("NULL");
      int objW = 0, objH = 0;
      is>>objW; is>>objH;
      LINFO("%d %d", objW, objH);
      v.itsObjectSize = Dims(objW, objH);
    }
  is>>v.itsSalPoint.i;
  is>>v.itsSalPoint.j;

  is>>featureSize;
  v.itsFeatures.clear(); v.itsFeatures.resize(featureSize);
  for (uint i = 0; i < featureSize; i++) is>>v.itsFeatures[i];

  uint keySize; is>>keySize;
  v.itsKeypoints.clear(); v.itsKeypoints.resize(keySize);

  std::vector< rutz::shared_ptr<Keypoint> >::iterator
    k = v.itsKeypoints.begin(), stop = v.itsKeypoints.end();

  while (k != stop)
    {
      rutz::shared_ptr<Keypoint> newkey(new Keypoint());
      is>>(*newkey); *k++ = newkey;
    }

  v.itsIsSorted =
    myIsSorted(v.itsKeypoints.begin(), v.itsKeypoints.end(), lessKP());
}

// ######################################################################
Image<PixRGB<byte> > VisualObject::
getKeypointImage(const float scale, const float vmag,
                 const PixRGB<byte> col)
{
  std::vector<rutz::shared_ptr<Keypoint> >::const_iterator
    k = itsKeypoints.begin(),
    stop = itsKeypoints.end();

  Image< PixRGB<byte> > image(getImage());
  if (scale != 1.0F)
    image = rescale(image, int(image.getWidth() * scale),
                    int(image.getHeight() * scale));

  while(k != stop)
    {
      const float x = (*k)->getX() * scale;
      const float y = (*k)->getY() * scale;
      const float s = (*k)->getS() * scale * vmag;
      const float o = (*k)->getO();

      Point2D<int> loc(int(x + 0.5F), int(y + 0.5F));
      drawDisk(image, loc, 2, PixRGB<byte>(255,0,0));
      if (s > 0.0f) drawLine(image, loc,
                             Point2D<int>(int(x + s * cosf(o) + 0.5F),
                                     int(y + s * sinf(o) + 0.5F)),
                             PixRGB<byte>(255, 0, 0));
      ++k;
    }
  return image;
}

// ######################################################################
Image<PixRGB<byte> > VisualObject::
getKeypointImage2(const float scale, const float vmag,
                  const PixRGB<byte> col)
{
  std::vector<rutz::shared_ptr<Keypoint> >::const_iterator
    k = itsKeypoints.begin(),
    stop = itsKeypoints.end();

  Image< PixRGB<byte> > image(getImage());
  if (scale != 1.0F)
    image = rescale(image, int(image.getWidth() * scale),
                    int(image.getHeight() * scale));

  while(k != stop)
    {
      const float x = (*k)->getX() * scale;
      const float y = (*k)->getY() * scale;
      const float s = (*k)->getS() * scale * vmag;
      const float o = (*k)->getO();

      Point2D<int> loc(int(x + 0.5F), int(y + 0.5F));
      drawDisk(image, loc, 2, PixRGB<byte>(255,0,0));

      if (s >= 1.0f)
        drawCircle(image, loc, int(s), PixRGB<byte>(255,0,0));
      if (s > 0.0f) drawLine(image, loc,
                             Point2D<int>(int(x + s * cosf(o) + 0.5F),
                                     int(y + s * sinf(o) + 0.5F)),
                             PixRGB<byte>(255, 0, 0));
      ++k;
    }
  return image;
}

// ######################################################################
Image<PixRGB<byte> > VisualObject::
getSalAndKeypointImage(const float scale, const float vmag,
                       const PixRGB<byte> col)
{

  Image<PixRGB<byte> > image = getKeypointImage(scale,vmag,col);
  Point2D<int> salpt((int)(itsSalPoint.i*scale), (int)(itsSalPoint.j*scale));
  drawDisk(image, salpt, 2, PixRGB<byte>(255,255,0));

  return image;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
