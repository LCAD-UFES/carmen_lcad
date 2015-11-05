/*!@file Image/MathOps.H Mathematical operations on Image
 */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/MathOps.H $
// $Id: MathOps.H 14592 2011-03-11 23:19:12Z jshen $
//

#ifndef IMAGE_MATHOPS_H_DEFINED
#define IMAGE_MATHOPS_H_DEFINED

#include "Image/Image.H"
#include "Util/Promotions.H"
#include "Image/Pixels.H" //out when we templatize the power-noise subroutine
#include <vector>

class Dims;
template <class T> class Point2D;
class Rectangle;
template <class T> class Image;
template <class T> class Range;

//! fill with a generating function: x = func() for each x in image
/*! returns a copy of the functor object after having been called */
template <class T, class F>
inline F fill(Image<T>& x, F func);

//! apply a function in-place: x = func(x) for each x in image
/*! returns a copy of the functor object after having been called */
template <class T, class F>
inline F apply(Image<T>& x, F func);

//! apply a function out-of-place: y = func(x) for x in input, y in output
template <class T2, class T, class F>
inline Image<T2> transform(const Image<T>& x, F func);

//! Returns the sum of all input pixels.
template <class T>
double sum(const Image<T>& a);

//! Get the average of all input pixels.
template <class T>
double mean(const Image<T>& a);

//! compute standard deviation of pixel values over image
template <class T>
double stdev(const Image<T>& a);

//! Get the min/max range of the input pixels.
/*! This provides similar functionality to Image<T>::getMinMax(). */
template <class T>
Range<T> rangeOf(const Image<T>& img);

//! Linearly remap pixels from the source to dest dynamic ranges.
/*! This is similar to, but more general than, Image<T>::normalize(). In
    effect, Image<T>::normalize() always assumes that the source range is
    the image's own min/max range. In contrast, remapRange() allows one to
    find the global min/max across a range of images, and then use that as
    the source range while normalizing each the images using that range. */
template <class T>
Image<T> remapRange(const Image<T>& img,
                    const Range<T>& from, const Range<T>& to);

//! Square each element of the input image.
template <class T>
Image<T> squared(const Image<T>& a);

//! Raise each element of the input image to the given power.
template <class T>
Image<typename promote_trait<T,float>::TP>
toPower(const Image<T>& a, double pow);

//! Raise pixels in given region of an input image to the given power. Pass a vector of 2D points that define the region of the image that you want to enhance.
template <class T>
Image<typename promote_trait<T,float>::TP>
toPowerRegion(const Image<T>& a, double pow, std::vector<Point2D<int> > region);

//! Take absolute value of every pixel.
template <class T>
Image<T> abs(const Image<T>& a);

//! Compute exp(pixel / (2 * sigma^2)) for every pixel.
template <class T>
Image<typename promote_trait<T,float>::TP>
hmaxActivation(const Image<T>& a, const float sigma);
/*
//! Returns abs(b - c).
//template <class T>
//Image<T> absDiff(const Image<T>& b, const Image<T>& c);
*/
//! Returns abs(b - c) for each color channel
template <class T_or_RGB>
Image<T_or_RGB> absDiff(const Image<T_or_RGB>& b,
                        const Image<T_or_RGB>& c);

//! Returns (b - c), clamping to zero all negative values.
//template <class T>
//Image<T> clampedDiff(const Image<T>& b, const Image<T>& c);

//! Returns (b - c) for each color channel, clamping to zero all neg. values
template <class T_or_RGB>
Image<T_or_RGB> clampedDiff(const Image<T_or_RGB>& b,
                            const Image<T_or_RGB>& c);

//! Returns (val - a).
template <class T>
Image<T> binaryReverse(const Image<T>& a, const T val);

//! Returns (b + c) / 2, clean & prevents overflows.
template <class T>
Image<T> average(const Image<T>& b, const Image<T>& c);

//! Returns (a*w + b) / 2, the weighted average between images
template <class T>
Image<T> averageWeighted(Image<T>& a, const Image<T>& b, T *aWeight);

//! Returns max(a, b), element by element.
template <class T_or_RGB>
Image<T_or_RGB> takeMax(const Image<T_or_RGB>& a,
                        const Image<T_or_RGB>& b);

//! Take element-wise max of a1,a2 and use that to choose b1,b2, storing a[12] result into aout and b[12] result into bout
template <class T> 
void takeLinkedMax(const Image<T>& a1, const Image<T>& a2, const Image<T>& b1, const Image<T>& b2, Image<T>& aout, Image<T>& bout);

//! Returns min(a, b), element by element.
template <class T>
Image<T> takeMin(const Image<T>& a, const Image<T>& b);

//! Compute energy for quadrature pair (sqrt(img1^2 + img2^2)).
template <class T_or_RGB>
Image<T_or_RGB> quadEnergy(const Image<T_or_RGB>& img1,
                           const Image<T_or_RGB>& img2);

//! RMS error between arr1 and arr2.
template <class T>
double RMSerr(const Image<T>& arr1, const Image<T>& arr2);

//! Create transparency overlay of two images
/*! by laying top over bottom with transparency of top set to
 trans. A new image will be returned (use 0 to 100 for percent
 transparency). NOTE: top and bottom must be the same size!
 this function only works for grey scale images */
template <class T_or_RGB>
Image<T_or_RGB> overlay(const Image<T_or_RGB>& top, const Image<T_or_RGB>& bottom,
                 const float trans);

//! sum pixels along the X and Y axes (sumX, sumY) and in total (return value)
/*!@param img is a w x h size image
  @param sumX will return a vector of length w with the sum over the rows of img
  @param sumY will return a vector of length h with the sum over the columns*/
template <class T>
float sumXY(const Image<T>& img, std::vector<float>& sumX,
            std::vector<float>& sumY);

//! Returns Euclidean distance between two images
/*! Returns sqrt(sum_over_pixels((img1[i] - img2[i])^2)) */
template <class T>
double distance(const Image<T> &img1, const Image<T> &img2);

//! Returns weighted Euclidean distance between two images
/*! Returns sqrt(sum_over_pixels(weight[i] * (img1[i] - img2[i])^2)) */
template <class T>
double distance(const Image<T> &img1, const Image<T> &img2,
                const Image<float> &weight);

//! Returns the correlation coefficient r^2 of the two images.
double corrcoef(const Image<float>& img1, const Image<float>& img2);

//! Returns the cross-correlation coefficient r between two patches
/*! Note that the computations done here differ from what is done in
  corrcoeff, in that a signed correlation coefficient is returned
  (between -1 and 1), and we add a convention that if both patches
  have zero variance then the correlation is 1. If only one patch has
  zero variance then the correlation is zero. Current implementation
  requires that the patch fits within both image bounds, and assumes a
  small patch size so that it will not attempt to use a faster
  FFT-based implementation which can save time with larger patches.
  @param img1 first image
  @param topleft1 coordinates of top-left corner of patch in img1
  @param patchdims dimensions of patch
  @param img2 second image
  @param topleft2 coordinates of top-left corner of patch in img2
  @param eps epsilon used to determine whether patch variance is zero */
template <class T>
double corrpatch(const Image<T>& img1, const Point2D<int>& topleft1,
                 const Dims& patchdims, const Image<T>& img2,
                 const Point2D<int>& topleft2, const double eps = 1.0e-20);

//! return correlation matrix and means for input samples
/*! This class will take a set of images and produce the eigen / correlation
    matrices. The input provided consists of a set of images:

    (1) An image for each feature dimension
    (2) Multiple sets of (1) for each frame sample

    That is, you provide a cardinal set of images that contains multiple
    image frames as well as feature maps for each frame.

    This class returns an image that is the eigen matrix given the samples
    you have provided. It also returns the mean values per dimension. The
    usefulness of this is that one can compute and fit a gaussian to
    the data. This can be used for instance in likelyhood estimation.

    The return eigen matrix is either a basic covariance matrix with
    un-normalized values useful for likelyhood estimation OR you can
    opt to have it normalize the matrix by the product of the standard
    deviations. This gives a pearson r correlation coefficent.

    note on input: The set is a nest vector of images. It should be of the form
    in c++ notation

          baseImages[frames][features]

    Thus, the first nesting should index each frame while the second should
    index each feature image.

    We compute covariance using an augmented method from Hayes 5th Ed. 1991

    @param baseImages The set of all input images
    @param baseCorr a return image that is the eigen matrix
    @param baseMean a return image that is the set of means
    @param baseSTD a return image that is the set of independant standard dev.
    @param baseSS a return image that contains the sum of squares
    @param baseN a return uint that is the total sample size
    @param returnR Should we return the normalized R or the eigen matrix
*/

template <class T>
void corrEigenMatrix(const std::vector<std::vector<Image<T> > > &baseImages,
                     Image<T> &baseCorr, Image<T> &baseMean,
                     Image<T> &baseSTD,  Image<T> &baseSS,
                     uint &baseN, bool returnR = false);

//! Given an eigen matrix and mean matrix, compute a likelyhood image
/*! In this function we take in an image and the statistics of an
    exemplar image set. We then use this to find a likelyhood image
    which is how much each pixel in the input image is like the exempler
    based on probability. High return values mean that a pixel has a high
    probability of belonging to the exemplar class.

    Basically, we compute P(x|C) for each pixel assuming baseImages as the
    samples and baseCorr as the eigen matrix that describes the distribution
    of the original sample. This computes P from the multi-variate gaussian.
    The results can be taken and fed into getNormalized bayes image with
    the results from a second class to give a true bayes P for each pixel
    belonging to one class or the other.

    Note: the non-normalized likelhood image is usefull for certain things
    but should not be used in deriving the direct bayed probability

    @param baseImage the image set you wish to find the likelyhood over
    @param baseCorr The eigen matrix correlation from the training set
    @param baseMean The set of mean values from the training set
    @param returnLogLikelyhood Keep numbers sane and return log not e^
    @param likelyhoodImage this is the returned likelyhood image
    @param nonNormalizedLImage this is a non-normalized likelyhood image

*/
template <class T>
void getLikelyhoodImage(const std::vector<Image<T> > &baseImages,
                        const Image<T> &baseCorr, const Image<T> &baseMean,
                        const bool returnLogLikelyhood,
                        Image<T> &likelyhoodImage,
                        Image<T> &nonNormalizedLImage);

//! Take in two likelyhood images and compute bayes p between them
/*! The idea here is to compute the true bayesian probability of
    membership in one image or another given the likelyhood computed in
    getLikelyhoodImage. Here we compute the p(C|x) for two classes.

    The idea here is to compute the likelyhood that a pixel belongs to one
    class v. another in an input image. You must input two likelyhood images
    of the form P(x|C) (as computed in getLikelyhoodImage). Also, you may
    input the prior probability P(C) if P(C1) is not P(C2). Otherwise,
    it is assumed P(C1) = P(C2).

    The return image has the P for each pixel being in class 1 v. class 2
    to get the result the other way, compute 1 - P (pretty easy).

    To compute this, we perform P(C|x) = g(a) where:

    a = ln((P(x|C1)*P(C1))/(P(x|C2)*P(C2)))

    g(a) = 1/(1 + exp(-a))

    This is known as logistic discrimination (Bishop 1995)

    @param classImage1 the likelyhood image of class 1
    @param classImage2 the likelyhood image of class 2
    @usingLogLikelyhood set to true if using log rather than e^ likelyhood
    @param classPrior1 the prior of class 1 (sample numbers)
    @param classPrior2 the prior of class 2 (sample numbers)
    @param bias This is a simple bias if desired, P(x)' = P(x)*bias
*/
template <class T>
Image<T> getNormalizedBayesImage(const Image<T> classImage1,
                                 const Image<T> classImage2,
                                 const bool usingLogLikelyhood,
                                 const T beta,
                                 const T classPrior1 = 1.0,
                                 const T classPrior2 = 1.0,
                                 const T bias        = 1.0);

//! Take in an image of correlation and standard deviation, give r
/*! You can use this function to give r from an already existing eigen matrix
    for instance, one computed from corrEigenMatrix. Thus, you can post hoc
    compute r if you opted not to when using corrEigenMatrix. This function
    just basically normalizes the eigen matrix by the standard deviation
    to give a true pearson r correlation matrix

    @param eigenMatrix The input eigen correlation matrix
    @param STDMatrix The input independant standard deviations
*/
template <class T>
Image<T> getPearsonRMatrix(const Image<T> &eigenMatrix,
                           const Image<T> &STDMatrix);

//! Augment the bayes image with some beliefs
/*! The belief images should be from 0 to 1 where 0 signifies
    complete belief in the reliability of class1 results while 0
    signifies absolutly no reliability. This basicially takes the product
    of the two as the final augmented belief.

    Using the median point, beliefs can be contracted to single values
    for instance, if the input bayes image ranges from 0 to 2, if the median
    is specified as 1, then values below 1 will tend to 1 as beleifs decrease
    while values above 1, will tend to 1 as beliefs decrease.

    @param bayesImage An image from getNormalizedBayesImage
    @param beliefImage1 the belief for class image 1
    @param beliefImage2 the belief for class image 2
    @param medianPoint a middle point values are squashed to
    @param beliefImage The augemented belief image
    @param beliefValue a record of the beleif products
*/

template <class T>
void getAugmentedBeliefBayesImage(const Image<T> &bayesImage,
                                  const Image<T> &beliefImage1,
                                  const Image<T> &beliefImage2,
                                  const T medianPoint,
                                  Image<T> &beliefImage,
                                  Image<T> &beliefValue);



//! Returns peak signal-to-noise ratio
/*! pSNR is computed as 10.log_10(255^2/sigma^2) where
  sigma^2=/N.sum(x1_i - x2_i)^2 and is a measure of distortion between
  two images */
template <class T>
double pSNR(const Image<T> &img1, const Image<T> &img2);

//! Returns weighted peak signal-to-noise ratio
/*! This is like the other pSNR() except that it uses the weighted
  version of distance() internally */
template <class T>
double pSNR(const Image<T> &img1, const Image<T> &img2,
            const Image<float>& weight);

//! Take square root of all elements
template <class T>
Image<typename promote_trait<T,float>::TP> sqrt(const Image<T>& img);

//! Take inverse of all elements > eps in abs, 0 otherwise
template <class T>
Image<typename promote_trait<T,float>::TP> inverse(const Image<T>& img,
                                                   const T eps);

//! Take exponential of all pixels
template <class T>
Image<typename promote_trait<T,float>::TP> exp(const Image<T>& img);

//! Take negative exponential of all pixels, that is exp(-x)
template <class T>
Image<typename promote_trait<T,float>::TP> negexp(const Image<T>& img);

//! Take natural logarithm of all pixels
template <class T>
Image<typename promote_trait<T,float>::TP> log(const Image<T>& img);

//! Take base-10 logarithm of all pixels
template <class T>
Image<typename promote_trait<T,float>::TP> log10(const Image<T>& img);

//! determine the first and last non-zero values and the centroid
/*!@param vect a vector of numbers
  @param centroid returns the centroid of vect
  @param first returns the position of the first non-zero value in vect
  @param last returns the position of the last non-zero value in vect
  @return true if we actually found all these values, false if vect
  only contains zeros*/
template <class T>
bool getCentroidFirstLast(std::vector<T> vect, float& centroid,
                          int& first, int& last);

//! determine the centroid of all the points in the image
/*! The centroid is rounded to integer pixel positions.
  When there are no values > 0 in the img, (-1,-1) is returned
  @param boundingBox the bounding box of the object(s) contained in
  the image is returned
  @param cenX returns the float value of the centroid along the x axis
  @param cenY returns the float value of the centroid along the y axis
  @return the centroid rounded to integer coordinates*/
template <class T>
Point2D<int> centroid(const Image<T>& img, Rectangle& boundingBox,
                 float& cenX, float& cenY);

//! determine the center of mass of all the points in the image
/*! The centroid is rounded to integer pixel positions.
  When there are no values > 0 in the img, (-1,-1) is returned */
template <class T>
Point2D<int> centroid(const Image<T>& img);

//! Apply a squashing function to the pixel values
/*! The squashing function is a 4th-order polynomial with sigmoidal
  shape. Denoting by x a pixel value in the original image and by
  y=f(x) the transformed pixel value that will be put in the result
  image, the contraints on the polynomial are: 1) f(oldmin)=newmin, 2)
  f(oldmax)=newmax, 3) f(oldmid)=newmid, 4) f'(oldmin)=0, and 5)
  f'(oldmax)=0. So, typically, the polynomial will have horizontal
  slope at both ends of the input range, and will remap the old
  min/max to the new min/max while also remapping a user-chosen
  mid-point oldmid to newmid. Playing with this mid-point allows the
  user to give more emphasis to higher or lower values while
  remapping. This is the full form, but two simplified forms are also
  available that will compute some of the arguments here from the
  input image. Because the main use of this function is when oldmin is
  the true minimum value of the input image, and oldmax its true
  maximum, here we will not waste time checking for input values
  outside this range, and their remapped values are unclear as the
  polynomial is unconstrained outside the input range. Internal
  remapping is done using floats, and results are converted back (with
  possible clamping) to the same type as the original image, so that
  this function may be efficiently used to do contrast adjustments in
  Image<byte> data. */
template<class T>
Image<T> squash(const Image<T>& src,
                const T oldmin, const T newmin,
                const T oldmid, const T newmid,
                const T oldmax, const T newmax);

//! Apply a squashing function to the pixel values
/*! In this specialization of the general form of squash(), the old
  min/max are computed using getMinMax(). */
template<class T>
Image<T> squash(const Image<T>& src, const T newmin,
                const T oldmid, const T newmid, const T newmax);

//! Apply a squashing function to the pixel values
/*! In this specialization of the general form of squash(), the old
  min/max are computed using getMinMax(), and the new min/max will be
  kept equal to the old ones, so that there is no overall change in
  range (unless your mid-point is outside that range). */
template<class T>
Image<T> squash(const Image<T>& src, const T oldmid, const T newmid);

//! Mix between two images based on comparing mask values to a threshold
/*! On every pixel, if mask >= thresh, the returned value is taken
  from higher, otherwise it is taken from lower. All three of lower,
  highr and mask must have same dims. */
template <class T, class T_or_RGB>
Image<T_or_RGB> thresholdedMix(const Image<T>& mask, const T& thresh,
                               const Image<T_or_RGB>& lower,
                               const Image<T_or_RGB>& higher);

//! take the logistic sigmoid 1/(1+e^x(...)) over the image
/*! This is a standard logistic sigmoid with offset o and slope b
  @param ima the input image
  @param o Offset for this sigmoid
  @param b Slope for this sigmoid
 */
Image<float> logSig(const Image<float>& ima, float o, float b);

//! randomly scramble image pixels
template <class T_or_RGB>
Image<T_or_RGB> scramble(const Image<T_or_RGB>& ima);

//! take in an image and return it's statistically most relevent points
/*!
  Input an image to find a monte carlo like map. This will in essence cut
  out about half the pixels in a simple quick fashion by applying  the formula:
  select if -> pow(pixelValue,bias) > minVal+(maxVal*rand()/RAND_MAX+1.0)
  The bias is used to scew the distribution using an exponent
  if no bias is given, this method will work more efficently by skipping
  the exponent computation over each pixel.
  @param ima This is the input probability map
  @param coords this is a pointer to a vector which will hold selected coords
  @param bais this scews the probability map by pow(pixVal,bias)

*/
int32 findMonteMap(Image<float>& ima,
                   std::vector<Point2D<int> >* coords,
                   int decimation, float bias);

//! Take in a vector and decimate it according how many points you want back
/*!
  In essence this will decimate a list of coordinates attempting to create a
  list of the size in outPoints. Thus, if you have list of 23890 points and
  want a list of 300 points, this will take every nth point and create a
  list of about this size.
  @param coords This is the list of input coordinates
  @param cmap This is the final list of coordinates after decimation
  @param inPoints this is the input list size
  @param outPoints this is how many points you would like out
*/

int32 makeSparceMap(std::vector<Point2D<int> >* coords, std::vector<Point2D<int>*>* cmap,
                    std::vector<Point2D<int> >* Oldcoords,
                    std::vector<bool>* keep, int inPoints, int outPoints);


//! a += b * coeff
template <class T>
void inplaceAddWeighted(Image<T>& a,
                        const Image<T>& b, const float coeff);

//! a = a*a
template <class T>
void inplaceSquare(Image<T>& a);

//! Replace all occurences of a value 'from' by another value 'to'
void inplaceReplaceVal(Image<byte>& dest,
                       const byte& from, const byte& to);

//! Progressive attenuation of borders by "size" pixels
template <class T_or_RGB>
void inplaceAttenuateBorders(Image<T_or_RGB>& a, int size);

//! Set borders of "size" pixels to given value
/*! Default value is T's default, which should be T's representation
    of zero. */
template <class T_or_RGB>
void inplaceSetBorders(Image<T_or_RGB>& a, const int size,
                       const T_or_RGB value = T_or_RGB());

//! Add speckle noise to array; thresh = 1 for 100% noise
void inplaceSpeckleNoise(Image<byte>& dest,
                         const float thresh, const int size,
                         const byte noise_color,
                         bool random_color=false);

//! Add power-law noise to array
//! Beta is a power-law factor
//! T must be either a float or a double
template <class T>
Image<typename promote_trait<T,float>::TP> 
addPowerNoise(const Image<T>& src, double beta);

//! Get a sample which is the max value within a circular aperture
/*! This only works for monochromatic images.
  @param center coordinates of the center of the aperture
  @param radius radius of the aperture, in pixels */
float getLocalMax(const Image<float>& src,
                  const Point2D<int>& center, const int radius);

//! Get a sample which is the average value within a circular aperture
/*! This only works for monochromatic images.
  @param center coordinates of the center of the aperture
  @param radius radius of the aperture, in pixels */
float getLocalAvg(const Image<float>& src,
                  const Point2D<int>& center, const int radius);

//! Get min and max values
template <class T>
void getMinMax(const Image<T>& src, T& mini, T& maxi);

//! Get min and max values inside and outside a mask
/*! Only pixels with non-zero mask value are considered. */
template <class T>
void getMaskedMinMax(const Image<T>& src, const Image<byte>& mask,
                     T& min_in, T& max_in, T& min_out, T& max_out);

//! Get min, max and average values
template <class T>
void getMinMaxAvg(const Image<T>& src, T& mini, T& maxi, T& avg);

//! Get min, max, avg within a binary mask
/*! Only the pixels where mask is non-zero are considered. */
template <class T>
void getMaskedMinMaxAvg(const Image<T>& src, const Image<byte> &mask,
                        T& mi, T& ma, T& avg);

//! Get min, max, sum and area values from a continuously-masked image
/*! The sum is the weighted sum of src by mask. Area is the sum of all
  non-zero mask values. */
template <class T>
void getMaskedMinMaxSumArea(const Image<T>& src, const Image<float> &mask,
                            T& mi, T& ma,
                            typename promote_trait<T,float>::TP &sum,
                            typename promote_trait<T,float>::TP &area);

//! Get min, max, average std deviation and some other stats
template <class T>
void getMinMaxAvgEtc(const Image<T>& src, T& xmini, T& xmaxi, T& xavg, T& xstd,
                     ushort& minPosX, ushort& minPosY,
                     ushort& maxPosX, ushort& maxPosY,
                     uint& pixCount);

//! Check wether all pixels have finite values
/*! This relies on isFinite() being defined for your pixel type
  T. In Types.H we define it for the canonical types, and you can
  use these canonical definitions to define it for more complex
  types (see, for example, the definition in Pixels.H). */
template <class T>
bool isFinite(const Image<T>& src);

//! find point of max activity and also what that max activity is
template <class T>
void findMax(const Image<T>& src, Point2D<int>& p, T& maxval);

//! find point of min activity and also what that min activity is
template <class T>
void findMin(const Image<T>& src, Point2D<int>& p, T& minval);

//! Saturate values < cmin to cmin and > cmax to cmax
template <class T>
void inplaceClamp(Image<T>& dst, const T cmin, const T cmax);

//! Normalize values between nmin and nmax
template <class T>
void inplaceNormalize(Image<T>& dst, const T nmin, const T nmax);

//! Normalize values between nmin and nmax, also return oldmin and oldmax
template <class T>
void inplaceNormalize(Image<T>& dst, const T nmin, const T nmax,
                      T& oldmin, T& oldmax);

//! Return true if point p is a local maximum
/*! We just check that the value at p is >= the values of its 4 neighbors */
template <class T>
bool isLocalMax(const Image<T>& src, const Point2D<int>& p);

//! Saturate values < 0
template <class T>
void inplaceRectify(Image<T>& dst);

//! Put all values >= 0 into pos and the negated of all vals <= 0 into neg
template <class T>
void splitPosNeg(const Image<T>& src,
                 Image<T>& pos, Image<T>& neg);

//! Cut values < thresh and replace them by val
template <class T>
void inplaceLowThresh(Image<T>& dst,
                      const T thresh, const T val = T());

//! Cut values whose abs is < thresh and replace them by val
template <class T>
void inplaceLowThreshAbs(Image<T>& dst,
                         const T thresh, const T val = T());

//! Pass image through sigmoid: f(x) = x^g / (s + x^h)
template <class T>
void inplaceSigmoid(Image<T>& dst,
                    const float g, const float h, const float s);

//! Tells how many pixels are zero
template <class T>
int emptyArea(const Image<T>& src);

//! Counts how many pixels are > thresh (in absolute value if absol true)
template <class T>
int countThresh(const Image<T>& src,
                const T thresh, const bool absol = true);

//! Return a row vector containing the within-column mean of each input column
Image<float> meanRow(const Image<float>& inp);

//! Return a row vector containing the within-column standard deviation of each input column
Image<float> stdevRow(const Image<float>& inp);

//! Return the result of adding vector v to each row of matrix M
Image<float> addRow(const Image<float>& M, const Image<float>& v);

//! Return the result of subtracting vector v from each row of matrix M
Image<float> subtractRow(const Image<float>& M, const Image<float>& v);

//! Return the result of multiplying each row of matrix M by vector v
Image<float> multiplyRow(const Image<float>& M, const Image<float>& v);

//! Return the result of dividing each row of matrix M by vector v
Image<float> divideRow(const Image<float>& M, const Image<float>& v);

// ######################################################################
// ######################################################################
// ##### Inline function definitions
// ######################################################################
// ######################################################################

// ######################################################################
template <class T, class F>
inline F fill(Image<T>& x, F func)
{
  // optimization; see comment in Image<T>::clear()
  if (x.isShared())
    x = Image<T>(x.getDims(), NO_INIT);

  for (typename Image<T>::iterator itr = x.beginw(), stop = x.endw();
       itr != stop; ++itr)
    *itr = T(func());

  return func;
}

// ######################################################################
template <class T, class F>
inline F apply(Image<T>& x, F func)
{
  for (typename Image<T>::iterator itr = x.beginw(), stop = x.endw();
       itr != stop; ++itr)
    *itr = T(func(*itr));

  return func;
}

// ######################################################################
template <class T2, class T, class F> inline
Image<T2> transform(const Image<T>& x, F func)
{
  Image<T2> result(x.getDims(), NO_INIT);

  typename Image<T>::const_iterator sptr = x.begin();
  typename Image<T2>::iterator dptr = result.beginw();
  typename Image<T2>::iterator stop = result.endw();

  while (dptr != stop)
    *dptr++ = func(*sptr++);

  return result;
}

// ######################################################################
//! Approximate a polyline with a simpler polyline using the Douglas-Peucker algorithm
// This code was derived from softSurfer (www.softsurfer.com)
std::vector<Point2D<int> >  approxPolyDP(std::vector<Point2D<int> >& points, float tol);

//The recursion for the DP
void recursePolyDP(float tol, std::vector<Point2D<int> >& v, int j, int k, std::vector<int>& mk);

//! Approximate 2D hull algorithm using the BFP algorithm
// Copyright 2001, softSurfer (www.softsurfer.com)
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.
std::vector<Point2D<float> > approximateHull(std::vector<Point2D<float> > points, int accuracy);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_MATHOPS_H_DEFINED
