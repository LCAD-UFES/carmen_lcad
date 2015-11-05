/*!@file SIFT/KDTree.H k-d tree implementation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/KDTree.H $
// $Id: KDTree.H 9298 2008-02-21 18:11:04Z rjpeters $
//
#ifndef KDTREE_H_DEFINED
#define KDTREE_H_DEFINED

#include "SIFT/Keypoint.H"
#include <queue>

// #######################################################################
//! Represents a rectangle in n-dimensional space
/*! The Hyperrectangle has n dimensions which are assumed to each have
  byte type, as in the case in Keypoint. */
class HyperRectangle
{
public:
  //! Constructor; the HyperRectangle will be uninitialized
  inline HyperRectangle(const uint dim);

  //! Constructor with initial min and max values for all dimensions
  inline HyperRectangle(const uint dim, const byte mini, const byte maxi);

  //! Copy constructor
  inline HyperRectangle(const HyperRectangle& other);

  //! Creates an all-encompassing rectangle in the dimension specified
  inline static HyperRectangle createUniverseRectangle(const uint dim);

  //! Splits the rectangle at the specified dimension and value
  inline HyperRectangle splitAt(const uint splitDim, const byte splitVal);

  //! Returns true if the target point is within this HR
  inline bool isIn(const rutz::shared_ptr<Keypoint>& target) const;

  //! Return true if any part of this HR is reachable from target
  /*! Must be reachable by no more than 'sqdist', false otherwise.  */
  inline bool isInReach(const rutz::shared_ptr<Keypoint>& target,
                        const int sqdist) const;

  //! Return the dist from nearest point within the HR to the target point.
  inline int distSq(const rutz::shared_ptr<Keypoint>& target) const;

private:
  std::vector<byte> itsLeftTop;
  std::vector<byte> itsRightBottom;
};

// ######################################################################
//! A simple KD tree implementation
/*! This class recursively constructs a KD tree from a flat list of
  exemplars. Then, it allows for efficient finding of the exemplar in
  the tree that is nearest a given target point, of of the set of
  exemplars in the tree that are within some distance of a given
  target point. */
class KDTree
{
public:
  //! Constructor from a flat list of Keypoint objects
  /*! Note that it is okay to destroy 'keys' as an internal copy will
    be kept. You may want to keep it around, however, and to not
    modify it, since the nearest neighbor functions in KDTree return
    indices in that original array of keys. Optionally, the array
    'objindices' may be passed which contains an object number for
    each Keypoint; if it is specified, then second-matches will only
    be considered which belong the the same object as the best
    match. */
  KDTree(const std::vector< rutz::shared_ptr<Keypoint> >& keys,
         const std::vector<uint>& objindices = std::vector<uint>());

  //! Destructor
  ~KDTree();

  //! Return true if we are empty (no pivot, no subtrees)
  inline bool isEmpty() const;

  //! Return true if we are a leaf (valid pivot but no subtrees)
  inline bool isLeaf() const;

  //! Find the nearest neighbor to hyperspace point 'target' in kd-tree.
  /*! The index of the nearest keypoint in the list passed at
    construction is returned. After return 'distsq1' contains the
    absolute squared distance between the target point and the
    returned nearest neighbor point. 'distsq2' contains the squared
    distance between the target point and the second best match found
    during the search (the second best match is not returned; distsq2
    is useful to check the quality of the best match). */
  uint nearestNeighbor(const rutz::shared_ptr<Keypoint>& target,
                       int& distsq1, int& distsq2) const;

  //! Limited Best-Bin-First k-d-tree nearest neighbour search.
  /*! The index of the nearest keypoint in the list passed at
    construction is returned. Find the approximate nearest neighbour
    to the hyperspace point 'target' within the kd-tree using
    'searchSteps' tail recursions at most (each recursion deciding
    about one neighbours' fitness). After return 'distsq1' contains
    the absolute squared distance between the target point and the
    returned nearest neighbor point. 'distsq2' contains the squared
    distance between the target point and the second best match found
    during the search (the second best match is not returned; distsq2
    is useful to check the quality of the best match). */
  uint nearestNeighborBBF(const rutz::shared_ptr<Keypoint>& target,
                          const int searchSteps, int& distsq1,
                          int& distsq2) const;

private:
  rutz::shared_ptr<Keypoint> itsPivot;      // copy of our splitting Keypoint
  uint itsSplitDim;                  // the splitting dimension for subtrees
  uint itsPivotIndex;                // index of pivot in c'tor list of keys
  uint itsObjIndex;                  // index of object our pivot belongs to
  rutz::shared_ptr<KDTree> itsLeftSubTree;  // our left subtree
  rutz::shared_ptr<KDTree> itsRightSubTree; // our right subtree

  // a small helper struct for the BBF search:
  struct BBFdata {
    inline BBFdata(const HyperRectangle& h, const rutz::shared_ptr<KDTree>& t,
                   const uint px, const uint pox,
                   const rutz::shared_ptr<Keypoint>& p, const int d) :
      hr(h), tree(t), pividx(px), pivobji(pox), pivot(p), distsq(d) { }

    HyperRectangle hr;
    rutz::shared_ptr<KDTree> tree;
    uint pividx;
    uint pivobji;
    rutz::shared_ptr<Keypoint> pivot;
    int distsq;

    // CAUTION! "less priority" means "higher distance" hence, to sort
    // our BBF priority queue, operator< is weird!
    inline bool operator<(const BBFdata& other) const
    { return distsq > other.distsq; }
  };

  // used internally by our recursive constructor:
  KDTree(const std::vector< rutz::shared_ptr<Keypoint> >& keys,
         const std::vector<uint>& indices,
         const std::vector<uint>& objindices);

  // Internal recursive algorithm for the kd-tree nearest neighbour search
  uint nearestNeighborI(const rutz::shared_ptr<Keypoint>& target,
                        const HyperRectangle& hr,
                        int maxDistSq, int& distsq1, int& distsq2,
                        const int maxdsq, uint& objidx) const;

  // Internal recursive algorithm for BBF kd-tree nearest neighbour search
  uint nearestNeighborBBFI(const rutz::shared_ptr<Keypoint>& target,
                           const HyperRectangle& hr, int& searchSteps,
                           int maxDistSq, int& distsq1, int& distsq2,
                           const int maxdsq,
                           std::priority_queue<BBFdata>& bbfq,
                           uint& objidx) const;

  // find a good candidate for splitting, and splitting dimension
  uint goodCandidate(const std::vector< rutz::shared_ptr<Keypoint> >& exset,
                     uint& splitDim);
};


// #######################################################################
// #######################################################################
// ############# HyperRectangle implementation
// #######################################################################
// #######################################################################

// #######################################################################
inline HyperRectangle::HyperRectangle(const uint dim) :
  itsLeftTop(dim), itsRightBottom(dim)
{ }

// #######################################################################
inline HyperRectangle::HyperRectangle(const uint dim, const byte mini,
                                      const byte maxi) :
  itsLeftTop(dim, mini), itsRightBottom(dim, maxi)
{ }

// ######################################################################
inline HyperRectangle::HyperRectangle(const HyperRectangle& other) :
  itsLeftTop(other.itsLeftTop), itsRightBottom(other.itsRightBottom)
{ }

// #######################################################################
inline HyperRectangle
HyperRectangle::createUniverseRectangle(const uint dim)
{ return HyperRectangle(dim, 0, 255); }

// #######################################################################
inline HyperRectangle
HyperRectangle::splitAt(const uint splitDim, const byte splitVal)
{
  HyperRectangle r2(*this);
  itsRightBottom[splitDim] = splitVal;
  r2.itsLeftTop[splitDim] = splitVal;

  return r2;
}

// #######################################################################
inline bool HyperRectangle::isIn(const rutz::shared_ptr<Keypoint>& target) const
{
  const uint dim = itsLeftTop.size();
  //ASSERT(target->getFVlength() == dim);

  for (uint n = 0 ; n < dim ; ++n)
    {
      const int targD = target->getFVelement(n);
      if (targD < itsLeftTop[n] || targD >= itsRightBottom[n]) return false;
    }

  return true;
}

// #######################################################################
inline bool HyperRectangle::isInReach(const rutz::shared_ptr<Keypoint>& target,
                                      const int distsq) const
{ return (distSq(target) < distsq); }

// #######################################################################
int HyperRectangle::distSq(const rutz::shared_ptr<Keypoint>& target) const
{
  const uint dim = itsLeftTop.size();
  //ASSERT(target->getFVlength() == dim);
  int distsq = 0;

  for (uint n = 0 ; n < dim ; ++n) {
    const int tI = target->getFVelement(n);
    const int hrMin = itsLeftTop[n];
    const int hrMax = itsRightBottom[n];

    if (tI <= hrMin) { distsq += (tI - hrMin) * (tI - hrMin); }
    else if (tI >= hrMax) { distsq += (tI - hrMax) * (tI - hrMax); }
  }

  return distsq;
}

// #######################################################################
// #######################################################################
// ############# KDTree implementation
// #######################################################################
// #######################################################################

inline bool KDTree::isEmpty() const
{ return itsPivot.is_invalid(); }

// ######################################################################
inline bool KDTree::isLeaf() const
{
  return itsPivot.is_valid() &&
    itsLeftSubTree.is_invalid() &&
    itsRightSubTree.is_invalid();
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
