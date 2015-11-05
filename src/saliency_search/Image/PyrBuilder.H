/*!@file Image/PyrBuilder.H Classes for building dyadic pyramids */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PyrBuilder.H $
// $Id: PyrBuilder.H 14471 2011-02-03 01:24:37Z dberg $
//

#ifndef PYRBUILDER_H_DEFINED
#define PYRBUILDER_H_DEFINED

#include "Image/Convolutions.H"
#include "Image/Image.H"
#include "Image/PyramidTypes.H"

#include <deque>

template <class T> class ImageSet;
template <class T> class PixRGB;
template <class T> class PyramidCache;

// ######################################################################
//! An interface class for creating dyadic pyramids from input images.
/*! A dyadic image pyramid is an array of images such that the next image
    is obtained from the previous one by applying some filter and
    decimating the image by a factor 2 horizontally and vertically.

    The PyrBuilder classes essentially wrap one of the buildPyr*()
    functions, binding preset values to all of the trailing arguments. This
    allows all of the buildPyr*() functions to be presented behind a
    uniform interface.

    PyrBuilder defers the actual handling of input images through the
    abstract function build(). Different subclasses may implement this to
    apply different filters (e.g., gaussian, laplacian, gabor) before
    decimation. GenericPyrBuilder offers a generic implementation, while
    GaussianPyrBuilder, LaplacianPyrBuilder, and OrientedPyrBuilder are
    specialized for single filter types.
*/
template <class T>
class PyrBuilder
{
public:
  //! Create an empty PyrBuilder
  PyrBuilder();

  // default copy contructor and assignment are okay

  //! Virtual destructor for safe inheritance.
  virtual ~PyrBuilder();

  //! Cloning constructor useful to make clones from pointers:
  virtual PyrBuilder<T>* clone() const = 0;

  //! Create a pyramid from an input image, with given depth (=nb levels)
  /*! @param firstlevel if non-zero, then pyramid levels prior to
      firstlevel may be skipped and left empty, as long as the
      remaining pyramid levels contain the same values that they would
      normally contain -- this parameter is only an optimization hint,
      and subclasses are free to ignore it if they so choose

      @param depth number of levels the pyramid should contain (note
      that if firstlevel is non-zero, then some of those levels may be
      empty)
   */
  virtual ImageSet<T> build(const Image<T>& image,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0) = 0;

  //! reset the pyramid - whatever that means for a specific pyramid
  /*! a no op implementation is given in this base class - so if pyramids
    don't need this, they don't have to worry about it */
  virtual void reset();

private:
};


// ######################################################################
//! Builds pyramids based on Gaussian filters.
template <class T>
class GaussianPyrBuilder : public PyrBuilder<T>
{
public:
  //! Constructor.
  GaussianPyrBuilder(const int filter_size);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual GaussianPyrBuilder<T>* clone() const;

  //! Builds a Gaussian pyramid based on \a img.
  virtual ImageSet<T> build(const Image<T>& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

private:
  int itsFiltSize;
};

// ######################################################################
//! Builds pyramids based on Gaussian filters with boundary conditions for radially transformed images
template <class T>
class GaussianRadialPyrBuilder : public PyrBuilder<T>
{
public:
  //! Constructor.
  GaussianRadialPyrBuilder();

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual GaussianRadialPyrBuilder<T>* clone() const;

  //! Builds a Gaussian pyramid based on \a img.
  virtual ImageSet<T> build(const Image<T>& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);
};

// ######################################################################
//! Builds pyramids based on arbitrary filters.
template <class T>
class ConvolvePyrBuilder : public PyrBuilder<T>
{
public:
  //! Constructor.
  ConvolvePyrBuilder(const Image<float>& filt,
                     ConvolutionBoundaryStrategy boundary);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual ConvolvePyrBuilder<T>* clone() const;

  //! Builds a pyramid based on \a img.
  virtual ImageSet<T> build(const Image<T>& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

private:
  Image<float> itsFilt;
  ConvolutionBoundaryStrategy itsBoundary;
};

// ######################################################################
//! Builds pyramids based on arbitrary color filters.
template <class T>
class RGBConvolvePyrBuilder : public PyrBuilder<T>
{
public:
  //! Constructor.
  RGBConvolvePyrBuilder(const Image<float>& rfilt,
                        const Image<float>& gfilt,
                        const Image<float>& bfilt,
                        ConvolutionBoundaryStrategy boundary);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual RGBConvolvePyrBuilder<T>* clone() const;

  //! Builds a pyramid based on a img.
  virtual ImageSet<T> build(const Image< PixRGB<T> >& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

  //! Don't use this one!
  virtual ImageSet<T> build(const Image<T>& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

  //! Builds a color pyramid based on a img.
  //! Only used to display results, filters behaviour...
  ImageSet< PixRGB<T> > build2(const Image< PixRGB<T> >& img,
                               const int firstlevel,
                               const int depth,
                               PyramidCache<T>* cache = 0);

private:
  Image<float> itsRFilt, itsGFilt, itsBFilt;
  ConvolutionBoundaryStrategy itsBoundary;
};

// ######################################################################
//! Builds pyramids based on Laplacian filters.
template <class T>
class LaplacianPyrBuilder : public PyrBuilder<T>
{
public:
  //! Constructor.
  LaplacianPyrBuilder(const int filter_size);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual LaplacianPyrBuilder<T>* clone() const;

  //! Builds a Laplacian pyramid based on \a img.
  virtual ImageSet<T> build(const Image<T>& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

private:
  int itsFiltSize;
};


// ######################################################################
//! Builds pyramid based on oriented Laplacian filters.
/*! This is a fast approximation to Gabor convolution. */
template <class T>
class OrientedPyrBuilder : public PyrBuilder<T>
{
public:
  //! Construct with a given gabor filter orientation and intensity.
  OrientedPyrBuilder(const int filter_size, const float theta = 0.0F,
                     const float intens = 10.0F,
                     const bool usetab = false);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual OrientedPyrBuilder<T>* clone() const;

  //! Builds an oriented pyramid based on \a img.
  virtual ImageSet<T> build(const Image<T>& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

private:
  int itsFiltSize;
  float itsAngle;
  float itsGaborIntens;
  bool itsUseTab;
};


// ######################################################################
//! Builds pyramids of a type specified by a PyramidType.
/*! This is a chameleon class; it can build any kind of pyramid (Gaussian,
    Laplacian, Gabor, QuickLocalAvg, QuickLocalMax) depending on the
    PyramidType passed to its constructor.
 */
template <class T>
class GenericPyrBuilder : public PyrBuilder<T>
{
public:
  //! Build with depth & type, but no input image yet.
  /*! \a gabor_theta and \a intens are used for Gabor pyramids. */
  GenericPyrBuilder(const PyramidType typ, const float gabor_theta = 0.0F,
                    const float intens = 10.0F);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor:
  virtual GenericPyrBuilder<T>* clone() const;

  //! Create a pyramid from an input image.
  /*! The type of pyramid that is built is determined by the PyramidType
      that was given to the constructor. */
  virtual ImageSet<T> build(const Image<T>& image,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

private:
  PyramidType itsPtype;     // Gaussian, Laplacian, Gabor, etc.
  float itsGaborAngle;      // orientation tuning for Gabor pyr
  float itsGaborIntens;     // filter strength for Gabor pyr
};


// ######################################################################
//! This class implements Reichardt motion detections
/*! The class uses the filter results of pyramids of other types
  to implement Reichardt motion detection. The pyramids of the previous
  time step are stored and used with the pyramid of this time step and
  a version of the current pyramid, in which each level is shifted by the
  same amount dx and dy. The results are stored in the Reichardt pyramid and
  represent motion at different speeds and spatial resolutions. */
template <class T>
class ReichardtPyrBuilder : public PyrBuilder<T>
{
public:
  // ######################################################################
  // #### Constructors
  // ######################################################################
  //! build with depth, dx, dy and type, but no input image yet
  /*! @param dx the number of pixels to shift into the x-direction
    (can be fractional value)
    @param dy the number of pixels to shift into the y-direction
    @param typ the type of pyramid to use for the filtering
    @param gabor_theta orientation of a gabor filter (if typ is GaborX)
    @param intens used for gabor filter ... */
  ReichardtPyrBuilder(const float dx,
                      const float dy,
                      const PyramidType typ,
                      const float gabor_theta = 0.0F,
                      const float intens = 10.0F);

  // default copy constructor and assignment are okay
  // std::vector and std::deque handle the copying of the individual images

  //! cloning constructor
  virtual ReichardtPyrBuilder<T>* clone() const;

  //! Builds a Reichardt pyramid based on a image
  virtual ImageSet<T> build(const Image<T>& image,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

  //! reset the stored queues - overrides PyrBuilder<T>::reset()
  virtual void reset();

private:
  float itsDX, itsDY;
  PyramidType itsPtype;     // Gaussian, Laplacian, Gabor, etc.
  float itsGaborAngle;      // orientation tuning for Gabor pyr
  float itsGaborIntens;     // filter strength for Gabor pyr
  std::deque< ImageSet<T> > shifted, unshifted;
};

// ######################################################################
//! A PyrBuilder that does template matching
class TemplateMatchPyrBuilder : public PyrBuilder<float>
{
public:
  // Constructor
  TemplateMatchPyrBuilder(const Image<float>& templ);

  //! Cloning constructor useful to make clones from pointers:
  virtual TemplateMatchPyrBuilder* clone() const;

  //! Create a pyramid from an input image.
  virtual ImageSet<float> build(const Image<float>& image,
                                const int firstlevel, const int depth,
                                PyramidCache<float>* cache = 0);

private:
  // return an image where higher values indicate better matches:
  Image<float> templateMatch(const Image<float>& img);

  Image<float> itsFilt;
};

// ######################################################################
//! A pyramid based on convolution with oriented Gabor filters
template <class T>
class GaborPyrBuilder : public PyrBuilder<T>
{
public:
  //! Constructor.
  /*! @param angle Angle (in degrees); parallel to the Gabor's "ridges".
      @param filter_period Period (in pixels) of underlying grating.
      @param elongation Ratio of major/minor axis lengths of Gaussian envelope.
      @param size determines the filter size (in pixels) - in the default case
      (-1), the size is computed from the filter_preiod and elongation
      @param buildFlags see Pyramid_Ops.H for possible values
  */
  GaborPyrBuilder(double angle,
                  double filter_period,
                  double elongation = 1.0,
                  int size = -1,
                  int buildFlags = 0);

  // Default copy constructor and assignment operator ok.

  //! Cloning constructor.
  virtual GaborPyrBuilder<T>* clone() const;

  //! Builds a Gabor pyramid based on an image.
  virtual ImageSet<T> build(const Image<T>& img,
                            const int firstlevel, const int depth,
                            PyramidCache<T>* cache = 0);

  //! Builds a Gabor pyramid from the levels in the given pyramid.
  ImageSet<T> input(const ImageSet<T>& pyr);

private:
  double itsAngle;
  double itsPeriod;
  double itsElongation;
  int itsSize;
  int itsBuildFlags;
};

#endif // !PYRBUILDER_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
