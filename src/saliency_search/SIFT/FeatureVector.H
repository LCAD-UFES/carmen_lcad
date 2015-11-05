/*!@file SIFT/FeatureVector.H Feature vector for SIFT obj recognition */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/FeatureVector.H $
// $Id: FeatureVector.H 14156 2010-10-21 21:21:11Z lior $
//

#ifndef FEATUREVECTOR_H_DEFINED
#define FEATUREVECTOR_H_DEFINED

#include "Util/Types.H"
#include <vector>
#include "Image/Image.H"
#include "Image/Pixels.H"


// ########################################################################
//! The FeatureVector  class
// ########################################################################
/*! Stores temporary SIFT Keypoint features.  The final feature vector
    in a SIFT Keypoint object is composed of bytes while here we use
    floats and are an intermediary onto which we can do normalizations
    and such before injecting into a Keypoint. A FeatureVector has 128
    bins, corresponding to 4 bins for x * 4 bins for y * 4 bins for
    orientation. */
class FeatureVector
{
public :
  //! Constructor, xSize,ySize,zSize the size of bins, wrap zBin ?
  FeatureVector(int xSize=4, int ySize=4, int zSize=8, bool wrapZ=true);

  //! Destructor
  ~FeatureVector();

  //! get a value from the feature vector
  float getValue(const uint index) const;

  //! add value
  /*! Linearly add a value. Indices are as follows:
    0.0 <= x <= 4.0 : [0 .. 3]   x=2.0 falls equally between bins 1 and 2;
    0.0 <= y <= 4.0 : [0 .. 3]   y=2.0 falls equally between bins 1 and 2;
    0.0 <= o <= 8.0 : [0 .. 7]   o=4.0 falls equally between bins 3 and 4 */
  void addValue(const float x, const float y, const float orientation,
                const float value);

  //! normalize Vector
  void normalize();

  //! threshold Vector and renormalize
  void threshold(const float limit);

  //! Convert to byte, to get ready for injection into a Keypoint
  /*! CAUTION: this calls normalize(), then threshold(0.2f), then
    normalize() again (all of which modify the FeatureVector), then
    converts all values multiplied by 512 to byte with clamping. Call
    this only when you are ready to discard the FeatureVector and only
    keep the final SIFT Keypoint byte data. */
  void toByteKey(std::vector<byte>& dest, float thresh=0.2, bool norm=true);

  int size(){ return itsFV.size(); }

  //! Return the x Bin Size
  inline int getXSize(){ return itsXSize; }

  //! Return the y Bin Size
  inline int getYSize(){ return itsYSize; }

  //! Return the z Bin Size
  inline int getZSize(){ return itsZSize; }

  //! Get the feature vector image
  Image<PixRGB<byte> > getFeatureVectorImage(std::vector<byte> &fv);

  //! Get the magnitude of the feature vector
  double getMag();

  //! Get the raw feature vector
  std::vector<float> getFeatureVector() const { return itsFV; } 


private:
  std::vector<float> itsFV;        // vector of features
  int itsXSize;                                                                // size of X bin;
  int itsYSize;                                                                // size of Y bin;
  int itsZSize;                                                                // size of Z bin;
  bool itsWrapZ;                                                          // do we wrap the Z bins

};


// ######################################################################
// Inlined member functions
// ######################################################################



#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
