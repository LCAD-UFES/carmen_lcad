/*!@file SpaceVariant/SpaceVariantTransform.H */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SpaceVariant/SpaceVariantTransforms.H $

#ifndef IMAGE_SPACEVARIANTTRANSFORMS_H_DEFINED
#define IMAGE_SPACEVARIANTTRANSFORMS_H_DEFINED

#include "SpaceVariant/ScaleSpaceOps.H"
#include "Image/Image.H"
#include "Util/Promotions.H"
#include "SpaceVariant/SVChanLevels.H"

//forward declare 
template <class T> class ImageSet;
//######################################################################
// SpaceVariantTransform 
//
// This is a general interface for a family of space variant image transforms. Space variant transforms remap a
// cartesian image (x,y) to a new space variant representation consisting of (u, v, r), where u and v represents new
// space dimensions and r isthe size of the receptive field (the amount of many-to-one mapping in cartesian pixels) of
// pixel u, v. The receptive field describes the spatial influcuence a single pixel in (x,y) will have in (u,v,r).
//
// ######################################################################
class SpaceVariantTransform
{ 
public:
  typedef Image<std::pair<float, float> >::iterator iterator;
  typedef Image<std::pair<float, float> >::const_iterator const_iterator;

  //! constructor
  SpaceVariantTransform();
  
  //! destructor
  ~SpaceVariantTransform();
  
  //!are we initialized?
  bool initialized() const;
  
  //!transform a point from cartesian to space variant coordinates
  void to(const int x, const int y, int& u, int& v) const;

  //!transform a point from cartesian to space variant coordinates
  void to(const int x, const int y, float& u, float& v) const;
  
  //!transform a point from space variant to cartesian coordinates
  void from(const int u, const int v, int& x, int& y) const;

  //!transform a point from space variant to cartesian coordinate
  void from(const int u, const int v, float& x, float& y) const;
  
  //!get the cartesian image size
  const Dims& getCTDims() const;

  //!get the space variant image size
  const Dims& getSVDims() const;

  //! get the rf size at the specified ring. returns the standard deviation of
  //! the gaussian used to model the RF.
  const float& getRFSize(const uint u, const uint v) const;

  //! get the rf size at the specified ring. returns the standard deviation of
  //! the gaussian used to model the RF. This version uses interpolation
  float getRFSize(const float& u, const float& v) const;

  //! get the rf size at the specified ring. returns the standard deviation of
  //! the gaussian used to model the RF.
  const float& getRFSize(const uint pos) const;

  //!get the max rf size
  float getMaxRFSize() const;

  //!get a const iterator to the lookup table
  const_iterator begin() const;
  
  //!get a const iterator to the lookup table
  const_iterator end() const;

  //!get a const iterator to the reverse lookup table
  const_iterator rbegin() const;
  
  //!get a const iterator to the reverse lookup table
  const_iterator rend() const;

  //get a const iterator to the rf lookup table
  Image<float>::const_iterator rfbegin() const;

  //get a const iterator to the rf lookup table
  Image<float>::const_iterator rfend() const;

protected:
  //!compute the distance from xpos, ypos to the nearest image
  //!boundary given the angle(radians)
  float computeDistance(const float& angle, const uint xpos, const uint ypos, 
                        const uint imgw, const uint imgh);
  
  //store our lookup tables as floats in a case we want to interpolate
  Image<std::pair<float, float> > itsLookup, itsRevLookup;
  Image<float> itsStd;
  float itsMaxStd;
  bool isInitialized;
};

// ######################################################################
//Computations for a foveal transform - SpaceVariantModule for a model 
// component which implements default params etc...
//
//FovealTransform - model of the retinal ganglial cell or visual structures 
//like the superior colliculus, thalamus or early visual cortex. We model
//the relationship between eccentricity from the fixation point and 
//density of receptor units using the basic variable resolution transform 
//described in Wiebe & Basu ('95). The default parameters are estimated from the 
//best fit to the data from ??? on ???. The receptive field (modeled as 
//a Gaussian parameterized by sigma) has a non-linear relationship between
//eccentricity and size and the default paramters fit an exponential equation
//to the data from Croner & Kaplan ('93) where the authors measured RF sizes for 
//magno- and parvocellular ganglion cells at a wide range of eccentricities. 
//the default params fit the parvo-ganglion cells data.
//######################################################################
struct FovealTransform : public SpaceVariantTransform
{
  //to indate the scaling condition
  enum SCALE_TYPE {CROP, FULL, NONE};

  //!constructor
  FovealTransform();

  //default distrucor, copy etc OK
  
  //!convert a string to scale type
  static SCALE_TYPE toScaleType(const std::string& scale_type_string);
  
  //!set transform parameters and compute lookup table. 
  void setup(const uint image_width, const uint image_height,
             const uint rings, const uint wedges,
             const float& alpha, const float& beta,
             const float& gain, const float& exponent, const float& offset,
             const float& ppdx, const float& ppdy, 
             const SCALE_TYPE& scale_type = FovealTransform::FULL, 
             const float& s = 0.0F, const float& fovea_cuttoff = 2.0F);
  
  //get the fovial size in pixels
  const uint getFoveaSize() const;

private:
  int u_FoveaSize;
};

// ######################################################################
// Compute the Superior Colliclus transform. 
//
// SCTransform maps (x,y)-> (u,v) where (x,y) is in degrees of visual space and (u,v) is the horizontal and vertical
// position on the SC surface. (u,v) will form a uniform grid. The assumption is that the warping of space is caused by
// the afferents to the superior colliculus. The model is most similliar to the basic variable resolution transform
// described in Wiebe & Basu ('95). The default paramters are chosen to best match the isotropic log-polar transform
// used by Otes '98 to map the visual field on the anatomical SC surface. The Otes transform is discontinous at the
// vertical meridian, so here a simpler continous transform is chosen which maintains neighborhood relations such that a
// uniform sized kernel can be used to compute interactions accross the whole SC map. The receptive field size (modeled
// as a Gaussian parameterized by sigma) as a function of eccentricity is modeled for Magnocellular and Parvocellular
// ganglion cells, and the paramters are estimated from the data of Croner and Kaplan ('95). The fitting was done such
// that the relationship between receptive field width and maximum response is maintained. An offset may be applied.

// ######################################################################
struct SCTransform : public SpaceVariantTransform
{
    
    //!constructor
    SCTransform();
    
    //default distrucor, copy etc OK

    //RF type
    enum RECEPTIVEFIELD_TYPE {MAGNO, PARVO};
    
    //!convert a string to scale type
    static RECEPTIVEFIELD_TYPE toRFType(const std::string& rf_type_string);
    
    //!set transform parameters and compute lookup table. 
    void setup(const uint image_width, const uint image_height,
               const uint rings, const uint wedges,
               const float& ppdx, const float& ppdy, const float& maxDeg, RECEPTIVEFIELD_TYPE const type);

    //!set transform parameters and compute lookup table.
    void setup(const uint image_width, const uint image_height,
               const uint rings, const uint wedges,
               const float& ppdx, const float& ppdy, const float& maxDeg, 
               const float& s1=0.02974, const float& s2=1.3923, const float& beta=0.81685, const float& alpha=0.32226,
               const float& rf_slope = 0.0002, const float& rf_exp = 1.7689 ,const float& rf_offset = 0.0252, const float& dog_factor = 0.0);
    
    static Point2D<float> pix2Deg(const float & x, const float & y, const float & ppdx, const float & ppdy, const float & imgx, const float & imgy);

    static Point2D<float> deg2Pix(const float & x, const float & y, const float & ppdx, const float & ppdy, const float & imgx, const float & imgy);

    static float rf2Pix(const float & rf, const float & ppdx, const float & ppdy);
};

// ######################################################################
// ######################################################################
// free functions for SpaceVariantTransform
// ######################################################################
// ######################################################################
//!return a space variant image
// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> transformTo(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL, const float& scaleoffset = 0.0F);

// ######################################################################
//!return a cartesian image from a space variant one
// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> transformFrom(const SpaceVariantTransform& sv,const Image<T_or_RGB>& input);

// ######################################################################
//!return a space variant image with center surround processing where 
// the surround size is determined by mutliplying the center sigma by 
// surround_mult (asumes a SpaceVariantTransform where getRf() returns 
// the size in units of sigma). From Croner & Kaplan ('95) a value 
// of ~6 gives the average retinal ganglion cell
// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToDoG(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& surround_mult, 
               const bool centeron, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL, const float& scaleoffset = 0.0F);

// ###################################################################### 
//!return a space variant image with center surround divisive processing where
//the surround size is determined by mutliplying the center sigma by
//surround_mult (asumes a SpaceVariantTransform where getRf() returns the size
//in units of sigma).
//######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToDivG(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& surround_mult, 
                const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL, const float& scaleoffset = 0.0F);

// ######################################################################
/*!return a space variant image with edge processing

  Get value of a local edge by pooling nearby on only and off only
  responses, but allowing the centers to differ. The tree shrew 
  apparently creates edge detecting cells in this fashion see
  Hirsh and Martiniz, Trends Neuro Sci, 2006 for a review.
  The result is squared.
*/
// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToEdge(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const Image<LocalEdge>& edgemap, 
                const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL, const float& scaleoffset = 0.0F);

// ######################################################################
/*!return a space variant image with edge processing, surround_mult is as in 
// transformToDoG

The wiring diagram is inspired by: 
Hirsh and Martiniz, Trends Neuro Sci, 2006 (for a reviw).
We model the orientation of a pixel as an on-center DiffScaleSpacePixel 
adjacent to an off-center one, creating a local orientation estimation. 
When the parameters are chosen correctly this looks simmilar to a odd gabor or
a third derivitive Gaussian An edge is then modeled as a length of local co-oriented local 
orientations at a specific density. The result is squared.
*/

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToEdge(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& surround_mult, 
                const Image<LocalEdge>& edgemap, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL, const float& scaleoffset = 0.0F);


// ######################################################################
//!return a space variant pyramid 
// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToPyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL);

// ######################################################################
//!return a space variant pyramid with DoG processing
// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToDoGPyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& surround_mult, const bool centeron, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL);

// ######################################################################
//!return a space variant pyramid with Edge processing
// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToEdgePyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const Image<LocalEdge>& edgemap, const float& orientation, const uint& length, const float& density, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL);

// ######################################################################
//!return a space variant pyramid with Edge processing
// ######################################################################
template <class T_or_RGB>
ImageSet<typename promote_trait<T_or_RGB, float>::TP>
transformToEdgePyramid(const SpaceVariantTransform& sv, const Image<T_or_RGB>& input, const float& surround_mult, const Image<LocalEdge>& edgemap, const float& orientation, const uint& length, const float& density, const SVChanLevels& scales, const ImageSet<typename promote_trait<T_or_RGB, float>::TP>* const cache = NULL);

// ######################################################################
// ######################################################################
// Needs updating to work with std instead of variance 
// free functions for FovealTransform: 
// These are different implementations of the transformTo and transformFrom 
// If the variance of the largest ring is small, this can be faster and
// interpolation can be turned off. 
// ######################################################################
// ######################################################################
//!return a space variant image
// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> transformToFoveal(const FovealTransform& sv, const Image<T_or_RGB>& input, const bool interp);

// ######################################################################
//!return a space variant image with center surround processing
// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP> 
transformToDoGFoveal(const FovealTransform& sv, const Image<T_or_RGB>& input,
                     const float& smult, const bool centeron, const bool interp);

// ######################################################################
// ######################################################################
// Other Functions
// ######################################################################
// ######################################################################
/*!On each hemifields top and bottom, extend the image by 
   replicating a flipped version of the opposite hemifields pixels. 
   This enables standard image processing algorithms to be applied
   with appropriate boarder conditions. The replicated pixels can then 
   be removed after processing. 

  Note: This is NOT optimized. The preffered method is to filter
  such that boundaries extend to the opposite hemifield. See Image/LowPassLpt.H */
// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> replicateHemifield(const Image<T_or_RGB>& inp, const uint pix = 48);


// ######################################################################
// get separate the fovial and peripherial image
// ######################################################################
template <class T_or_RGB>
void getFoveaPeriphery(const FovealTransform& sv, const Image<T_or_RGB>& ret_image, 
                       Image<T_or_RGB>& fovia, Image<T_or_RGB>& periphery);

// ######################################################################
//returns a vector of local edges
// ######################################################################
Image<LocalEdge> LocalEdgeMap(const SpaceVariantTransform& sv, const float& surround_mult, 
                              const float& orientation, const uint& length, const uint density);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_SPACEVARIANTTRANSFORMS_H_DEFINED
