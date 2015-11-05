/*!@file SIFT/Keypoint.H Keypoint for SIFT obj recognition */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/Keypoint.H $
// $Id: Keypoint.H 10462 2008-11-20 06:36:31Z rand $
//

#ifndef KEYPOINT_H_DEFINED
#define KEYPOINT_H_DEFINED

#include "Component/ModelManager.H"
#include "Raster/Raster.H"
#include "Util/Types.H"
#include "Util/Assert.H"
#include "rutz/shared_ptr.h"
#include <vector>
#include <iosfwd>

// ########################################################################
//! The keypoint base class
// ########################################################################
/*! The sift keypoint feature descriptor
    3D position (x, y, scale)
    Mean orientation, magnitude
    Feature vector */
class Keypoint
{
public:
        enum CHANNEL {ORI, COL};
  // ######################################################################
  /*! @name Constructors, destructor, copy, assigment */
  //@{

  //! Uninitialized constructor; will net to call reset() later
  Keypoint();

  //! Constructor
  Keypoint(const std::vector<byte>& features, const float x, const float y,
           const float s, const float o, const float dogmag);

  //! Constructor with Color
  Keypoint(const std::vector<byte>& features,
           const std::vector<byte>& colFeatures,
           const float x, const float y,
           const float s, const float o, const float dogmag,
           float oriWeight=1.0F, float colWeight=1.0F);

  //! Copy-constructor
  Keypoint(const Keypoint& k);

  //! Reset all internal values
  void reset(const std::vector<byte>& features,
             const float x, const float y,
             const float s, const float o, const float dogmag);

  //! Destructor
  ~Keypoint();

  //! Assignment
  Keypoint& operator=(const Keypoint& k);

  //@}

  // ######################################################################
  /*! @name Iterators and access functions
    The iterators iterate on our feature vector data only, while the
    other access functions can be used to access the other data. */

  //@{

  //! standard iterator
  typedef std::vector<byte>::iterator iterator;

  //! const iterator
  typedef std::vector<byte>::const_iterator const_iterator;

  //! Returns a read-only iterator to the beginning of the feature vec data
  inline const_iterator begin(CHANNEL channel=ORI) const;

  //! Returns a read-only iterator to one-past-the-end of the feature vec data
  inline const_iterator end(CHANNEL channel=ORI) const;

  //! Returns a read-write iterator to the beginning of the feature vec data
  inline iterator beginw(CHANNEL channel=ORI);

  //! Returns a read-write iterator to one-past-the-end of the feature vec data
  inline iterator endw(CHANNEL channel=ORI);

  //! x getter
  inline float getX() const;

  //! y getter
  inline float getY() const;

  //! scale getter
  inline float getS() const;

  //! orientation getter
  inline float getO() const;

  //! DoG magnitude getter
  inline float getM() const;

  //! Return the length of this keypoint's feature vector
  inline uint getFVlength(CHANNEL channel=ORI) const;

  //! Return the value of the feature vector at the given index
  inline byte getFVelement(const uint index, CHANNEL channel=ORI) const;

  //! Return a copy of the orientation feature vector
  inline std::vector<byte> getOriFV() { return itsOriFV; };

  //! Return a copy of the color feature vector
  inline std::vector<byte> getColFV() { return itsColFV; };
  //@}

  // ######################################################################
  /*! Comparisons and feature distance computation */
  //@{

  //! Compute squared distance between two keypoint descriptors
  inline int distSquared(const rutz::shared_ptr<Keypoint>& k) const;

  //! Return the maximum possible dist sq between two feature vectors
  /*! This is just a function of the size of our feature vector */
  inline int maxDistSquared(CHANNEL channel=ORI) const;

  //! Compares the magnitude of this keypoint to the given keypoint
  /*! This is useful, for example, to use std::sort() on an
    std::vector< rutz::shared_ptr<Keypoint> >. Just remember that the
    comparison is based on the magnitude of the DoG only, not on any
    other internal data of the Keypoint. */
  inline bool operator<(const Keypoint& k) const;


  //@}

protected:
  float itsX;                        //!< our x coordinate
  float itsY;                        //!< our y coordinate
  float itsS;                        //!< our scale coordinate
  float itsO;                        //!< our orientation
  float itsM;                        //!< our DoG magnitude, for sorting
  std::vector<byte> itsOriFV;        //!< our Ori feature vector
  std::vector<byte> itsColFV;        //!< our color feature vector
  float itsOriWeight;                //!< The weight for ori
  float itsColWeight;                //!< The weight for col


private:
  friend std::istream& operator>>(std::istream& is, Keypoint& k);
  friend std::ostream& operator<<(std::ostream& os, const Keypoint& k);
};

// ######################################################################
// Keypoint I/O:
// ######################################################################

//! Load a Keypoint from an istream
std::istream& operator>>(std::istream& is, Keypoint& k);

//! Save a Keypoint to an ostream
std::ostream& operator<<(std::ostream& os, const Keypoint& k);

// ######################################################################
// Inlined member functions
// ######################################################################
inline Keypoint::const_iterator Keypoint::begin(CHANNEL channel) const
{
  switch (channel){
  case ORI:
    return itsOriFV.begin();
  case COL:
    return itsColFV.begin();
  default:
    return itsOriFV.begin();  //default
  }
}

inline Keypoint::const_iterator Keypoint::end(CHANNEL channel) const
{
  switch (channel){
  case ORI:
    return itsOriFV.end();
  case COL:
    return itsColFV.end();
  default:
    return itsOriFV.end();  //default
  }
}

//TODO: should these be beginw and endw?
inline Keypoint::iterator Keypoint::beginw(CHANNEL channel)
{
  switch (channel){
  case ORI:
    return itsOriFV.begin();
  case COL:
    return itsColFV.begin();
  default:
    return itsOriFV.begin();  //default
  }
}

inline Keypoint::iterator Keypoint::endw(CHANNEL channel)
{
  switch (channel){
  case ORI:
    return itsOriFV.end();
  case COL:
    return itsColFV.end();
  default:
    return itsOriFV.end();  //default
  }
}

///////////////////// Color FV access ///////////////////////////
inline float Keypoint::getX() const
{ return itsX; }

inline float Keypoint::getY() const
{ return itsY; }

inline float Keypoint::getS() const
{ return itsS; }

inline float Keypoint::getO() const
{ return itsO; }

inline float Keypoint::getM() const
{ return itsM; }

inline uint Keypoint::getFVlength(CHANNEL channel) const
{
  switch (channel){
  case ORI:
    return itsOriFV.size();
  case COL:
    return itsColFV.size();
  default:
    return itsOriFV.size();  //default
  }
}

inline byte Keypoint::getFVelement(const uint index,CHANNEL channel) const
{
  switch (channel){
  case ORI:
    ASSERT(index < itsOriFV.size());
    return itsOriFV[index];
  case COL:
    ASSERT(index < itsColFV.size());
    return itsColFV[index];
  default:
    ASSERT(index < itsOriFV.size());
    return itsOriFV[index];
  }
}

inline int Keypoint::distSquared(const rutz::shared_ptr<Keypoint>& k) const
{
  int oriDistsq = 0;
  int colDistsq = 0;

  ASSERT(itsOriFV.size() == k->itsOriFV.size());
  if (itsColFV.size() > 0)
     ASSERT(itsColFV.size() == k->itsColFV.size());

  Keypoint::const_iterator oriF1 = begin(ORI), oriF2 = k->begin(ORI),
          oriStop = end(ORI);

  while(oriF1 != oriStop)
    {
      const int of1 = int(*oriF1++);
      const int of2 = int(*oriF2++);
      const int dif =  of1 - of2;
      oriDistsq += dif * dif;
    }

  if (itsColFV.size() > 0) {
    Keypoint::const_iterator colF1 = begin(COL), colF2 = k->begin(COL),
      colStop = end(COL);
    while(colF1 != colStop)
      {
        const int dif = int(*colF1++) - int(*colF2++);
        colDistsq += dif * dif;
      }
  }

  return int(itsOriWeight*float(oriDistsq) + itsColWeight*float(colDistsq));
  // return oriDistsq + colDistsq;
}

inline int Keypoint::maxDistSquared(CHANNEL channel) const
{
  switch (channel){
  case ORI:
    return 255 * 255 * itsOriFV.size();
  case COL:
    return 255 * 255 * itsColFV.size();
  default:
    return 255 * 255 * itsOriFV.size();
  }
}

inline bool Keypoint::operator<(const Keypoint& k) const
{ return (itsM < k.itsM); }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
