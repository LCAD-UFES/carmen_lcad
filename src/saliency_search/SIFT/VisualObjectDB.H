/*!@file SIFT/VisualObjectDB.H Visual object database */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/VisualObjectDB.H $
// $Id: VisualObjectDB.H 14116 2010-10-08 08:34:50Z siagian $
//

#ifndef VISUALOBJECTDB_DEFINED
#define VISUALOBJECTDB_DEFINED

#include "Image/Image.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"

#include <cstring>
#include <iosfwd>
#include <vector> // for std::pair

class KDTree;

//! VisualObjectDatabase
class VisualObjectDB
{
public:
  //! Constructor; use operator>> and operator<< to load/save
  VisualObjectDB();

  //! Destructor
  ~VisualObjectDB();

  //! Easy loading from file
  /*! This will just open a file and use operator>> on it. Returns true on success. */
  bool loadFrom(const std::string& fname, bool preloadImage = true);

  //! Easy saving to file
  /*! This will just open a file and use operator<< on it. Returns true on success. */
  bool saveTo(const std::string& fname);

  //! name access methods
  inline std::string getName() const;
  inline void setName(const std::string& name);

  //! get number of objects in the database
  inline uint numObjects() const;

  //! object getter
  inline const rutz::shared_ptr<VisualObject>& getObject(const uint index) const;

  //! object setter
  inline void setObject(const uint index, const rutz::shared_ptr<VisualObject>& obj);

  //! erase object
  inline void eraseObject(const uint index);

  inline void clearObjects(uint size = 0);

  //! object getter by name
  /*! Returns an uninitialized rutz::shared_ptr is not found. */
  inline rutz::shared_ptr<VisualObject> getObject(const std::string objectName) const;

  //! add object
  /*! @return true if the object was added, false if it was not added
    because an object with the same name already exists (uniqueName=true) in the
    database. This invalidates any internal KDTree we may have. */
  bool addObject(const rutz::shared_ptr<VisualObject>& obj, bool uniqueName = true);

  //! Build a giant internal KDTree from all our Keypoints
  /*! This will automatically be called by getObjectMatches() if an
    up-to-date KDTree is not internally available. This function is
    made public because it may take a while to run, and is best run
    just after loading a DB, rather than on the first match
    attempt. If an up-to-date KDTree is already internally available,
    this is a no-op. */
  void buildKDTree();

  //! find match for the keypoints in the scene
  /*! @param obj the VisualObject to match against the objects in our database.
    @param matches list of matches which satisfy all criteria, sorted
    by matching score. Note: the list is initially cleared.
    @param algo the algorithm to use for matching (see VisualObjectMatch.H).
    @param maxn the maximum number of object matches to return. NOTE:
    we will return the first maxn matches found without exploring the
    database any further. If this is not desirable, use a large
    matchn. You can also use sortbypf below to attempt that these
    first matches will indeed be the best ones.
    @param kcoeff coefficient for keypoint distance, used to score a
    match. See VisualObjectMatch::getScore().
    @param acoeff coefficient for affine distance, used to score a
    match. See VisualObjectMatch::getScore().
    @param minscore min acceptable VisualObjectMatch::getScore(kcoeff, acoeff).
    @param mink minimum number of matching keypoints in an object match.
    @param kthresh threshold to use for keypoint selection in obj; see
    VisualObject constructor and ScaleSpace.
    @param sortbypf sort our database by similarity of preattentive
    visual features if true. This is incompatible with KDTree-based
    matching algos, which would not benefit from this sorting.
    @return the number of matches found (== matches.size()) */
  uint getObjectMatches(const rutz::shared_ptr<VisualObject> obj,
                        std::vector< rutz::shared_ptr<VisualObjectMatch> >& matches,
                        const VisualObjectMatchAlgo algo = VOMA_SIMPLE,
                        const uint maxn = 5U,
                        const float kcoeff = 0.5F,
                        const float acoeff = 0.5F,
                        const float minscore = 1.0F,
                        const uint mink = 3U,
                        const uint kthresh = 6U,
                        const bool sortbypf = false);

  //! find match for the keypoints in the scene, parallel version
  uint getObjectMatchesParallel(const rutz::shared_ptr<VisualObject> obj,
                                std::vector< rutz::shared_ptr<VisualObjectMatch> >& matches,
                                const uint numthreads = 4,
                                const float kcoeff = 0.5F,
                                const float acoeff = 0.5F,
                                const float minscore = 1.0F,
                                const uint mink = 3U,
                                const uint kthresh = 6U,
                                const bool sortbypf = false);


  void createVisualObjectDB(std::istream& is, VisualObjectDB& vdb, bool preloadImage = true);

private:
  std::string itsName;
  std::vector< rutz::shared_ptr<VisualObject> > itsObjects;  // objects
  rutz::shared_ptr<KDTree> itsKDTree;
  std::vector< std::pair<uint,uint> > itsKDindices; // <objidx, kpidx>

  // sort object db according to preattentive feature vector similarity
  void computeSortedIndices(std::vector<uint>& indices,
                            const rutz::shared_ptr<VisualObject>& obj) const;

  friend std::istream& operator>>(std::istream& is, VisualObjectDB& vdb);
  friend std::ostream& operator<<(std::ostream& os, const VisualObjectDB& vdb);
};

// ######################################################################
// VisualObjectDB I/O functions
// ######################################################################

//! Load a VisualObjectDB from an istream
std::istream& operator>>(std::istream& is, VisualObjectDB& vdb);

//! Save a VisualObjectDB to an ostream
std::ostream& operator<<(std::ostream& os, const VisualObjectDB& vdb);


// ######################################################################
// VisualObjectDB inline function implementations
// ######################################################################

inline std::string VisualObjectDB::getName() const
{ return itsName; }

inline void VisualObjectDB::setName(const std::string& name)
{ itsName = name; }

inline uint VisualObjectDB::numObjects() const
{ return itsObjects.size(); }

inline void VisualObjectDB::eraseObject(const uint index)
{
  ASSERT(index < itsObjects.size());
  itsObjects.erase(itsObjects.begin() + index);
}

inline void VisualObjectDB::clearObjects(uint size)
{
  itsObjects.clear(); 
  if(size > 0) itsObjects.resize(size);
}

inline const rutz::shared_ptr<VisualObject>&
VisualObjectDB::getObject(const uint index) const
{
  ASSERT(index < itsObjects.size());
  return itsObjects[index];
}

inline rutz::shared_ptr<VisualObject>
VisualObjectDB::getObject(const std::string objectName) const
{
  std::vector< rutz::shared_ptr<VisualObject> >::const_iterator
    vo = itsObjects.begin(), stop = itsObjects.end();

  while (vo != stop)
    {
      if ((*vo)->getName().compare(objectName) == 0) return *vo;
      ++ vo;
    }

  // not found, return an uninitialized rutz::shared_ptr:
  return rutz::shared_ptr<VisualObject>();
}

inline void VisualObjectDB::setObject(const uint index, const rutz::shared_ptr<VisualObject>& obj)
{
  ASSERT(index < itsObjects.size());
  itsObjects[index] = obj;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
