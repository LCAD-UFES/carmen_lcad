/*!@file Image/fancynorm.H Intrafeature competition with maxNormalize(). */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/fancynorm.H $
// $Id: fancynorm.H 8821 2007-10-11 19:45:28Z rjpeters $
//

#ifndef FANCYNORM_H_DEFINED
#define FANCYNORM_H_DEFINED

#include "Util/log.H"

#include <string>

/*! \defgroup MaxNorm max-normalization algorithms */
//@{

/*! @name Types of normalization (normtyp) */
//@{

enum MaxNormType {
  VCXNORM_NONE          = 0,  //!< no max-normalization, but may change range
  VCXNORM_MAXNORM       = 1,  //!< non-iterative maxnorm
  VCXNORM_FANCY         = 2,  //!< full implementation of fancy maxnorm
  VCXNORM_FANCYFAST     = 3,  //!< fast implementation of fancy maxnorm
  VCXNORM_FANCYONE      = 4,  //!< one-iteration only of fancy maxnorm
  VCXNORM_FANCYLANDMARK = 5,  //!< fancy maxnorm to find landmark
  VCXNORM_LANDMARK      = 6,  //!< to find landmarks
  VCXNORM_FANCYWEAK     = 7,  //!< Weak version of FancyOne
  VCXNORM_IGNORE        = 8,  //!< Do absolutely nothing, not even change range
  VCXNORM_SURPRISE      = 9,  //!< Like ignore but VCX will do something
  VCXNORM_FANCYVWEAK    = 10, //!< Very weak version of FancyOne
  VCXNORM_STDEV         = 11, //!< normalize map to have stdev=1, minval=0
  VCXNORM_STDEV0        = 12  //!< normalize map to have stdev=1, mean=0
  // if you add some new types here, also update the name list below!
};

/*! Have this value always equal to the largest of the VCXNORMs (used for
  range checking) */
#define NBMAXNORMTYPES 13

//! Get a name in clear for a given type
inline const char* maxNormTypeName(const MaxNormType m)
{
  static const char n[NBMAXNORMTYPES][14] = {
    "None", "Maxnorm", "Fancy", "FancyFast", "FancyOne", "FancyLandmark",
    "Landmark", "FancyWeak", "Ignore", "Surprise", "FancyVWeak", "Stdev",
    "Stdev0" };

  if (int(m) < 0 || int(m) >= NBMAXNORMTYPES)
    LFATAL("Invalid MaxNormType value %d (valid range is 0-%d inclusive)",
           int(m), NBMAXNORMTYPES);

  return n[int(m)];
}

/*! Always points to the default maxnorm operation */
#define VCXNORM_DEFAULT VCXNORM_FANCY

//@}

/*! @name Additional parameters for fancynorm versions of the algorithm. */
//@{
const int FANCYITER = 5;   //!< default number of iterations
const double FANCYESIG = 2;   //!< excitatory sigma as % of image size
const double FANCYISIG = 25;  //!< inhibitory sigma as % of image size
const double FANCYCOEX = 0.5; //!< excitatory coefficient (strength)
const double FANCYCOIN = 1.5; //!< inhibitory coefficient (strength)
const double FANCYINHI = 2.0; //!< strength of global inhibition

const double FANCYG = 2.1; //!< for sigmoid normalization
const double FANCYH = 2.0; //!< for sigmoid normalization
const double FANCYS = 1.0; //!< for sigmoid normalization

const int LRLEVEL = 2; //!< for tuned long-range excitation:

//@}

/*! @name Standard min/max bounds for maxNormalize(). */
//@{

//! upper bound for maxNormalize()
const float MAXNORMMAX = 10.0f;
//! lower bound for maxNormalize()
const float MAXNORMMIN = 0.0f;
//! upper bound for maxNormalizeFancyLandmark()
const float MAXNORMLANDMARK = 255.0f;

//@}


template <class T> class Image;

//! Generic implementation (select algorithm with \a normtyp).
/*! Normalize between mi and ma and multiply by (ma - mean). Versions with
    more arguments implement the core within-feature spatial competition
    for salience. See papers for details. */
template <class T>
Image<T> maxNormalize(const Image<T>& src,
                      const T mi, const T ma,
                      const MaxNormType normtyp = VCXNORM_DEFAULT,
                      int nbiter = FANCYITER,
                      const Image<float>* lrexcit = 0);

//! Specialized implementation (corresponds to VCXNORM_NONE).
/*! No max-normalization, just normalize between \a mi and \a ma. */
template <class T>
Image<T> maxNormalizeNone(const Image<T>& src, const T mi, const T ma);

//! Specialized implementation (corresponds to VCXNORM_MAXNORM).
/*! maxNorm from Itti et al, IEEE-PAMI, 1998. */
template <class T>
Image<T> maxNormalizeStd(const Image<T>& src, const T mi, const T ma);

//! Specialized implementation (corresponds to VCXNORM_FANCYFAST).
/*! fancyNorm from Itti et al, JEI, 2001; FAST implementation. */
template <class T>
Image<T> maxNormalizeFancyFast(const Image<T>& src, const T mi, const T ma,
                               const int nbiter = FANCYITER);

//! Specialized implementation (corresponds to VCXNORM_FANCY).
/*! fancyNorm from Itti et al, JEI, 2001; FULL implementation. */
template <class T>
Image<T> maxNormalizeFancy(const Image<T>& src, const T mi, const T ma,
                           const int nbiter = FANCYITER,
                           const double weakness = 1.0,
                           const Image<float>* lrexcit = 0);

//! Specialized implementation (corresponds to VCXNORM_FANCY).
/*! fancyNorm from Itti et al, JEI, 2001; FULL implementation.
 adapted to find landmarks */
template <class T>
Image<T> maxNormalizeFancyLandmark(const Image<T>& src, const T mi, const T ma,
                           const int nbiter = FANCYITER);

//! Specialized implementation (corresponds to VCXNORM_LANDMARK).
/*! to find landmarks */
template <class T>
Image<T> maxNormalizeLandmark(const Image<T>& src, const T mi, const T ma);

//! to find number of peaks in the image (used in landmark detection)
template <class T>
int findPeaks(const Image<T>& src, const T mi, const T ma,
              double& sum);

//! to find the goodness of a map (used in landmark detection)
template <class T>
float goodness_map(const Image<T>& src);

//! Specialized implementation (corresponds to VCXNORM_STDEV).
/*! The output image is the result of normalizing the input image to
    have stdev=1 and minval=0, by simply dividing by the stdev of the
    original image and then subtracting the minval of the resulting
    image. We set minval=0, rather than the more natural mean=0,
    because maxNormalize functions traditionally return images with
    only non-negative values, since negative values will likely
    eventually be subject to a rectification. The choice of minval=0
    or mean=0 has no effect on the statistics of maps produced by
    summing several stdev-normalized maps; the resulting map itself
    will have the same stdev in either case, and the only difference
    will be a shift in its mean. */
template <class T>
Image<T> maxNormalizeStdev(const Image<T>& src);

//! Specialized implementation (corresponds to VCXNORM_STDEV0).
/*! The output image is the result of normalizing the input image to
    have mean=0 and stdev=1, by simply subtracting the mean of the
    original image and then dividing by the stdev of the original
    image. Beware that the resulting image will have negative values,
    which may be truncated by later rectification steps. */
template <class T>
Image<T> maxNormalizeStdev0(const Image<T>& src);

//@}


//! MaxNormType overload
/*! Format is "name" as defined by maxNormTypeName() in MaxNormTypes.H */
std::string convertToString(const MaxNormType val);
//! MaxNormType overload
/*! Format is "name" as defined by maxNormTypeName() in MaxNormTypes.H */
void convertFromString(const std::string& str, MaxNormType& val);


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
