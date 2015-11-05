/*!@file SIFT/VisualObject.H Header file for visual objects to be recognized */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/VisualObject.H $
// $Id: VisualObject.H 14116 2010-10-08 08:34:50Z siagian $
//

#ifndef VISUALOBJECT_H_DEFINED
#define VISUALOBJECT_H_DEFINED

#include "Image/Pixels.H"
#include "Image/Image.H"
#include "SIFT/Keypoint.H"
#include "rutz/shared_ptr.h"

#include <vector>
#include <iosfwd>
#include <string>

class Keypoint;

//! Describes name and attributes of a visual object
/*! A VisualObject is a visual view of a given real-world object. As
  such, it is described by a name, an Image representing the view, and
  possibly a preattentive feature vector (as obtained, for example,
  from VisualCortex::getFeatures() ), and a list of SIFT Keypoint
  objects. */
class VisualObject
{
public:
  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! Constructor assuming precomputed keypoints
  /*! @param name the name of the object.
    @param imagefname the filename to store our image into; see operator<<
    and operator>> for details on how this will be used.
    @param image an image of the object. Also see operator<< and operator>>
    for details.
    @param salpt the salient point location of the object.
    @param preattfeatures preattentive features, as obtained, for example,
    from VisualCortex::getFeatures().
    @param keypoints a vector of SIFT Keypoint descriptors. If this is
    empty, then a ScaleSpace will be built from the image and the SIFT
    keypoints will be extracted from it. */
  VisualObject(const std::string& name = "NULL",
               const std::string& imagefname = "NULL",
               const Image< PixRGB<byte> >& image = Image< PixRGB<byte> >(),
               const Point2D<int>& salpt = Point2D<int>(-1,-1),
               const std::vector<float>& preattfeatures = std::vector<float>(),
               const std::vector< rutz::shared_ptr<Keypoint> >& keypoints =
               std::vector< rutz::shared_ptr<Keypoint> >(),
               const bool useColor = false,
               bool computeKP = true);

  //! this is for computing keypoints at a later time
  //! only compute if the image is initialized and keypoints vector is empty
  void computeKeypoints();

  //! Copy constructor
  VisualObject(const VisualObject& vo);

  //! Destructor
  ~VisualObject();

  //! Assignment
  VisualObject& operator=(const VisualObject& vo);

  //@}

  // ######################################################################
  //! @name Access functions
  //@{

  //! name getter
  inline const std::string& getName() const;

  //! change the name, e.g., after we have recognized the object
  inline void setName(const std::string& name);

  //! image filename getter
  inline const std::string& getImageFname() const;

  //! change the image file name, e.g., after we have recognized the object
  inline void setImageFname(const std::string& fname);

  //! change the size of the object
  inline void setObjectSize(const Dims size);

  //! delete the image file from disk
  /*! This is useful if you want to make sure that the current image
    will be saved, as by default operator<< will not overwrite the
    image file. */
  void deleteImageFile() const;

  //! image getter
  inline const Image<PixRGB<byte> >& getImage() ;

  //! get the object dims
  inline const Dims getObjectSize() const;

  //! get the salient point location
  inline const Point2D<int>& getSalPoint() const;

  //! get all preattentive features
  inline const std::vector<float>& getFeatures() const;

  //! get number of pre-attentive features
  inline uint numFeatures() const;

  //! get a given pre-attentive feature
  inline float getFeature(const uint index) const;

  //! get all SIFT keypoints
  inline const std::vector< rutz::shared_ptr<Keypoint> >& getKeypoints() const;

  //! get number of SIFT keypoints
  inline uint numKeypoints() const;

  //! get a given SIFT keypoint
  inline const rutz::shared_ptr<Keypoint>& getKeypoint(const uint index) const;

  //! load the visual object image without returning it   
  inline void loadImage() ;

  //@}

  // ######################################################################
  //! @name Compute functions
  //@{

  //! Sort our Keypoints in ascending order of their DoG magnitude
  /*! We internally keep track of when we are sorted, so calling this
      multiple times will not impact performance. */
  void sortKeypoints();

  //! get squared distance between our and obj's pre-attentive features
  double getFeatureDistSq(const rutz::shared_ptr<VisualObject>& obj) const;

  //! Get an image that shows our SIFT keypoints
  /*! This returns our internal image onto which our keypoints are
    drawn as little vectors.
    @param scale optional scale factor to apply to the image, default
    is to zoom in by a factor 4, which makes the keypoints more visible.
    @param vmag magnitute factor for the length of the scale vectors.
    @param col color of the vectors. */
  Image<PixRGB<byte> > getKeypointImage(const float scale = 2.0F,
                                        const float vmag = 5.0F,
                                        const PixRGB<byte> col =
                                        PixRGB<byte>(255, 0, 0)) ;

  //! same as getKeypointImage with scale displayed as well
  Image<PixRGB<byte> > getKeypointImage2(const float scale = 2.0F,
                                         const float vmag = 5.0F,
                                         const PixRGB<byte> col =
                                         PixRGB<byte>(255, 0, 0)) ;

  //! same as getKeypointImage except + salient point
  Image<PixRGB<byte> > getSalAndKeypointImage(const float scale = 2.0F,
                                              const float vmag = 5.0F,
                                              const PixRGB<byte> col =
                                              PixRGB<byte>(255, 0, 0)) ;

  //@}


  static void createVisualObject(std::istream& is, VisualObject &v, bool loadImage = true);

protected:
  std::string itsName;                             // name of this object
  std::string itsImageFname;                       // filename of our image
  Image<PixRGB<byte> > itsImage;                   // object image
  std::vector< rutz::shared_ptr<Keypoint> > itsKeypoints; // keypoints
  Point2D<int> itsSalPoint;                        // the salient point location
  std::vector<float> itsFeatures;                  // pre-attentive features
  bool itsIsSorted;                                // are our Keypoints sorted?
  bool itsUseColor;                                // should we add color
  bool itsImageLoaded;                                   // Does the image already loaded?
  Dims  itsObjectSize;                             // This correspond to object size in the real world

private:
  friend std::istream& operator>>(std::istream& is, VisualObject& v);
  friend std::ostream& operator<<(std::ostream& os, const VisualObject& v);
};

// ######################################################################
// VisualObject I/O:
// ######################################################################

//! Load a VisualObject from an istream
std::istream& operator>>(std::istream& is, VisualObject& v);

//! Save a VisualObject to an ostream
/*! If our image file name is known also save the image, but only if
  there is not already a file by than name. You should delete the file
  first if you want to ensure that the image will be saved. You can
  use VisualObject::deleteImageFile() for that. */
std::ostream& operator<<(std::ostream& os, const VisualObject& v);


// ######################################################################
// Implementation for VisualObject inline functions
// ######################################################################
inline const std::string& VisualObject::getName() const
{ return itsName; }

inline void VisualObject::setName(const std::string& name)
{ itsName = name; }

inline const std::string& VisualObject::getImageFname() const
{ return itsImageFname; }

inline void VisualObject::setImageFname(const std::string& fname)
{ itsImageFname = fname; }

inline void VisualObject::setObjectSize(const Dims size)
{ itsObjectSize = size; }

inline const Image<PixRGB<byte> >& VisualObject::getImage()
{
  if(!itsImageLoaded)
    {
      itsImage = Raster::ReadRGB(itsImageFname);
      itsImageLoaded = true;
    }
  return itsImage;
}

inline void VisualObject::loadImage()
{
  if(!itsImageLoaded)
    {
      itsImage = Raster::ReadRGB(itsImageFname);
      itsImageLoaded = true;
    }
}

inline const Dims VisualObject::getObjectSize() const
{
  return itsObjectSize;
}

inline const Point2D<int>& VisualObject::getSalPoint() const
{ return itsSalPoint; }

inline const std::vector<float>& VisualObject::getFeatures() const
{ return itsFeatures; }

inline uint VisualObject::numFeatures() const
{ return itsFeatures.size(); }

inline float VisualObject::getFeature(const uint index) const
{
  ASSERT(index < itsFeatures.size());
  return itsFeatures[index];
}

inline const std::vector< rutz::shared_ptr<Keypoint> >&
VisualObject::getKeypoints() const
{ return itsKeypoints; }

inline uint VisualObject::numKeypoints() const
{ return itsKeypoints.size(); }

inline const rutz::shared_ptr<Keypoint>&
VisualObject::getKeypoint(const uint index) const
{
  ASSERT(index < itsKeypoints.size());
  return itsKeypoints[index];
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
