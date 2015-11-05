/*!@file SIFT/KDTree.C k-d tree implementation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/KDTree.C $
// $Id: KDTree.C 6990 2006-08-11 18:13:51Z rjpeters $
//

#include "SIFT/KDTree.H"
#include "Util/log.H"

// #######################################################################
KDTree::KDTree(const std::vector< rutz::shared_ptr<Keypoint> >& keys,
               const std::vector<uint>& objindices) :
  itsPivot(), itsSplitDim(0U), itsPivotIndex(0U), itsObjIndex(0U),
  itsLeftSubTree(), itsRightSubTree()
{
  const uint numkeys = keys.size();

  // NOTE: a leaf tree contains a Pivot but no splitting dimension and
  // no left or right trees.

  // if we are given an empty set of keys, bail out:
  if (numkeys == 0)
    { LFATAL("Given list of keys is empty! -- ABORT"); return; }

  // if we are given object indices, make sure the number of correct:
  if (objindices.empty() == false) ASSERT(objindices.size() == keys.size());

  // if we have only one key, we are a leaf:
  if (numkeys == 1)
    {
      itsPivot = keys[0];
      if (objindices.empty() == false) itsObjIndex = objindices[0];
      return;
    }

  // ok, we have more than one keys. Find a splitting point and dimension:
  itsPivotIndex = goodCandidate(keys, itsSplitDim);
  itsPivot = keys[itsPivotIndex]; // keep a copy of the key for later matching

  // split the exemplar set into left/right elements relative to the
  // splitting dimension:
  byte bound = itsPivot->getFVelement(itsSplitDim);
  std::vector< rutz::shared_ptr<Keypoint> > leftElems, rightElems;
  std::vector<uint> leftInd, rightInd;
  std::vector<uint> leftObjInd, rightObjInd;

  for (uint i = 0; i < numkeys; i++)
    {
      if (i == itsPivotIndex) continue; // ignore the splitting element (pivot)

      rutz::shared_ptr<Keypoint> dom = keys[i];

      if (dom->getFVelement(itsSplitDim) <= bound)
        {
          leftElems.push_back(dom);
          leftInd.push_back(i);
          if (objindices.empty() == false)
            leftObjInd.push_back(objindices[i]);
        }
      else
        {
          rightElems.push_back(dom);
          rightInd.push_back(i);
          if (objindices.empty() == false)
            rightObjInd.push_back(objindices[i]);
        }
    }

  // recurse:
  if (leftElems.size())
    itsLeftSubTree.reset(new KDTree(leftElems, leftInd, leftObjInd));
  if (rightElems.size())
    itsRightSubTree.reset(new KDTree(rightElems, rightInd, rightObjInd));
}

// #######################################################################
KDTree::KDTree(const std::vector< rutz::shared_ptr<Keypoint> >& keys,
               const std::vector<uint>& indices,
               const std::vector<uint>& objindices) :
  itsPivot(), itsSplitDim(0U), itsPivotIndex(0U), itsObjIndex(0U),
  itsLeftSubTree(), itsRightSubTree()
{
  const uint numkeys = keys.size();

  // NOTE: a leaf tree contains a Pivot but no splitting dimension and
  // no left or right trees.

  // if we are given an empty set of keys, bail out:
  if (numkeys == 0)
    { LFATAL("Given list of keys is empty! -- ABORT"); return; }

  // if we are given object indices, make sure the number of correct:
  if (objindices.empty() == false) ASSERT(objindices.size() == keys.size());

  // if we have only one key, we are a leaf:
  if (numkeys == 1)
    {
      itsPivot = keys[0];
      itsPivotIndex = indices[0];
      if (objindices.empty() == false) itsObjIndex = objindices[0];
      return;
    }

  // ok, we have more than one keys. Find a splitting point and dimension:
  uint idx = goodCandidate(keys, itsSplitDim);
  itsPivotIndex = indices[idx]; // index in the original array of keys
  itsPivot = keys[idx];         // keep a copy of the key for later matching

  // split the exemplar set into left/right elements relative to the
  // splitting dimension:
  byte bound = itsPivot->getFVelement(itsSplitDim);
  std::vector< rutz::shared_ptr<Keypoint> > leftElems, rightElems;
  std::vector<uint> leftInd, rightInd;
  std::vector<uint> leftObjInd, rightObjInd;

  for (uint i = 0; i < numkeys; i++)
    {
      if (i == idx) continue; // ignore the splitting element (pivot)

      rutz::shared_ptr<Keypoint> dom = keys[i];

      if (dom->getFVelement(itsSplitDim) <= bound)
        {
          leftElems.push_back(dom);
          leftInd.push_back(indices[i]);
          if (objindices.empty() == false)
            leftObjInd.push_back(objindices[i]);
        }
      else
        {
          rightElems.push_back(dom);
          rightInd.push_back(indices[i]);
          if (objindices.empty() == false)
            rightObjInd.push_back(objindices[i]);
        }
    }

  // recurse:
  if (leftElems.size())
    itsLeftSubTree.reset(new KDTree(leftElems, leftInd, leftObjInd));
  if (rightElems.size())
    itsRightSubTree.reset(new KDTree(rightElems, rightInd, rightObjInd));
}

// ######################################################################
KDTree::~KDTree()
{ }

// #######################################################################
uint KDTree::nearestNeighbor(const rutz::shared_ptr<Keypoint>& target,
                             int& distsq1, int& distsq2) const
{
  HyperRectangle hr(target->getFVlength(), 0, 255);
  const int maxdsq = target->maxDistSquared();
  uint objidx = 0U; // used internally to enforce second-best in same obj
  return nearestNeighborI(target, hr, maxdsq, distsq1, distsq2,
                          maxdsq, objidx);
}

// ######################################################################
uint KDTree::nearestNeighborBBF(const rutz::shared_ptr<Keypoint>& target,
                                const int searchSteps, int& distsq1,
                                int& distsq2) const
{
  HyperRectangle hr(target->getFVlength(), 0, 255);
  const int maxdsq = target->maxDistSquared();
  std::priority_queue<BBFdata> bbfq;
  int ssteps = searchSteps;
  uint objidx = 0U; // used internally to enforce second-best in same obj
  return nearestNeighborBBFI(target, hr, ssteps, maxdsq,
                             distsq1, distsq2, maxdsq, bbfq, objidx);
}

// ######################################################################
uint KDTree::nearestNeighborI(const rutz::shared_ptr<Keypoint>& target,
                              const HyperRectangle& hr, int maxDistSq,
                              int& distsq1, int& distsq2,
                              const int maxdsq, uint& objidx) const
{
  // if we are a leaf, just return our pivot and its distance to the target:
  if (isLeaf())
    {
      distsq1 = itsPivot->distSquared(target);
      distsq2 = maxdsq;
      objidx = itsObjIndex;
      return itsPivotIndex;
    }

  // ok, we are not empty nor a leaf. So let's check out what's going
  // on in our left and right subtrees:
  const byte splitval = itsPivot->getFVelement(itsSplitDim);
  const uint numdims = target->getFVlength();

  // Assign the nearer and further HRs and associated subtrees (steps 5-7):
  HyperRectangle nearerHr(numdims), furtherHr(numdims);
  rutz::shared_ptr<KDTree> nearerKd, furtherKd;

  if (target->getFVelement(itsSplitDim) <= splitval)
    {
      nearerKd = itsLeftSubTree;
      furtherKd = itsRightSubTree;
      nearerHr = hr;
      furtherHr = nearerHr.splitAt(itsSplitDim, splitval);
    }
  else
    {
      nearerKd = itsRightSubTree;
      furtherKd = itsLeftSubTree;
      furtherHr = hr;
      nearerHr = furtherHr.splitAt(itsSplitDim, splitval);
    }

  // Recursively get the nearest neighbor which could lie in the
  // nearer half tree (step 8):
  int distSq1 = maxdsq, distSq2 = maxdsq; uint nearest = 0U, nobjidx = 0U;
  if (nearerKd.is_valid())
    nearest = nearerKd->nearestNeighborI(target, nearerHr, maxDistSq,
                                         distSq1, distSq2, maxdsq, nobjidx);

  // If the second best match we just found is nearer than maxDistSq,
  // then reduce maxDistSq (modified step 9). Note: in the original
  // algorithm (which does not consider second-best matches), this
  // test would be done on distSq1 instead. So here we have a
  // performance impact for wanting to know the exact distance to the
  // second best match:

  //  if (distSq2 < maxDistSq) maxDistSq = distSq2;

  // NOTE2: the overhead is really too high. By using distSq1 as in
  // the original algo we sacrifice some accuracy on our estimation of
  // the second best distance, but accelerate the overall search a lot:
  if (distSq1 < maxDistSq) maxDistSq = distSq1;

  // If the further HR is too far away, it will be hopeless and we
  // don't even need to look at it (step 10):
  if (furtherHr.isInReach(target, maxDistSq))
    {
      // ok the further HR may contain a better (or second better)
      // match. First let's check how close our pivot is to the
      // target (modified steps 10.1.1 to 10.1.3):
      int ptDistSq = itsPivot->distSquared(target);

      if (ptDistSq < distSq1)
        {
          // our pivot is closer than our nearest, so make the pivot
          // our new nearest:
          nearest = itsPivotIndex;

          // if our pivot is in the same object than our previous
          // nearest, then just slide the distances; otherwise, make
          // distsq2 maximal:
          if (nobjidx == itsObjIndex)
            { distSq2 = distSq1; distSq1 = ptDistSq; }
          else
            { distSq1 = ptDistSq; distSq2 = maxdsq; nobjidx = itsObjIndex; }

          // update the max distance for further search:
          maxDistSq = distSq1;
        }
      else if (ptDistSq < distSq2)
        {
          // our pivot is farther than the nearest but closer than the
          // second nearest, so just update the second nearest
          // distance and max distance, but only if our pivot is in
          // the same object as our nearest:
          if (nobjidx == itsObjIndex)
            { distSq2 = ptDistSq; maxDistSq = distSq2; }
        }

      // Recursively explore the further HR (modified step 10.2):
      int tempDistSq1 = maxdsq, tempDistSq2 = maxdsq;
      uint tempNearest = 0U, tempobjidx = 0U;
      if (furtherKd.is_valid())
        tempNearest =
          furtherKd->nearestNeighborI(target, furtherHr, maxDistSq,
                                      tempDistSq1, tempDistSq2, maxdsq,
                                      tempobjidx);

      // If we found a better nearest or second nearest, update
      // accordingly (modified step 10.3):
      if (tempDistSq1 < distSq1)
        {
          // tempNearest is better than nearest, so use tempNearest:
          nearest = tempNearest;

          // now, who is the second best?
          if (tempobjidx == nobjidx)
            {
              // temp is in the same object as nearest used to be. So
              // let's just slide the second-best distances:
              if (tempDistSq2 < distSq1) distSq2 = tempDistSq2;
              else distSq2 = distSq1;
            }
          else
            {
              // temp in different object from old nearest:
              distSq2 = maxdsq;
              nobjidx = tempobjidx;
            }

          // also update our best distance:
          distSq1 = tempDistSq1;
        }
      else if (tempDistSq1 < distSq2)
        {
          // tempNearest is worse than nearest but better than our
          // second nearest, so just update our second nearest
          // distance, but only if in same object:
          if (tempobjidx == nobjidx)
            distSq2 = tempDistSq1;
        }
    }

  // return the best we have found. Note: nearest may be empty if we
  // did not find anything:
  distsq1 = distSq1; distsq2 = distSq2; objidx = nobjidx;
  return nearest;
}

// ######################################################################
uint KDTree::nearestNeighborBBFI(const rutz::shared_ptr<Keypoint>& target,
                                 const HyperRectangle& hr, int& searchSteps,
                                 int maxDistSq, int& distsq1, int& distsq2,
                                 const int maxdsq,
                                 std::priority_queue<BBFdata>& bbfq,
                                 uint& objidx) const
{
  // if we are a leaf, just return our pivot and its distance to the target:
  if (isLeaf())
    {
      distsq1 = itsPivot->distSquared(target);
      distsq2 = maxdsq;
      objidx = itsObjIndex;
      return itsPivotIndex;
    }

  // ok, we are not empty nor a leaf. So let's check out what's going
  // on in our left and right subtrees:
  const byte splitval = itsPivot->getFVelement(itsSplitDim);
  const uint numdims = target->getFVlength();

  // Assign the nearer and further HRs and associated subtrees (steps 5-7):
  HyperRectangle nearerHr(numdims), furtherHr(numdims);
  rutz::shared_ptr<KDTree> nearerKd, furtherKd;

  if (target->getFVelement(itsSplitDim) <= splitval)
    {
      nearerKd = itsLeftSubTree;
      furtherKd = itsRightSubTree;
      nearerHr = hr;
      furtherHr = nearerHr.splitAt(itsSplitDim, splitval);
    }
  else
    {
      nearerKd = itsRightSubTree;
      furtherKd = itsLeftSubTree;
      furtherHr = hr;
      nearerHr = furtherHr.splitAt(itsSplitDim, splitval);
    }

  // store the further guys into our priority queue:
  BBFdata fu(furtherHr, furtherKd, itsPivotIndex, itsObjIndex, itsPivot,
             furtherHr.distSq(target));
  bbfq.push(fu);

  // Recursively get the nearest neighbor which could lie in the
  // nearer half tree (step 8):
  int distSq1 = maxdsq, distSq2 = maxdsq; uint nearest = 0U, nobjidx = 0U;
  if (nearerKd.is_valid())
    nearest = nearerKd->
      nearestNeighborBBFI(target, nearerHr, searchSteps, maxDistSq,
                          distSq1, distSq2, maxdsq, bbfq, nobjidx);

  // If the second best match we just found is nearer than maxDistSq,
  // then reduce maxDistSq (modified step 9). Note: in the original
  // algorithm (which does not consider second-best matches), this
  // test would be done on distSq1 instead. So here we have a
  // performance impact for wanting to know the exact distance to the
  // second best match:

  //  if (distSq2 < maxDistSq) maxDistSq = distSq2;

  // NOTE2: the overhead is really too high. By using distSq1 as in
  // the original algo we sacrifice some accuracy on our estimation of
  // the second best distance, but accelerate the overall search a lot:
  if (distSq1 < maxDistSq) maxDistSq = distSq1;

  // before we even think about checking out the further HR, let's see
  // if we have one in our BBF queue:
  rutz::shared_ptr<Keypoint> pivot; uint pividx, pivobji;
  if (bbfq.size() > 0)
    {
      BBFdata fu = bbfq.top(); bbfq.pop();
      furtherHr = fu.hr; furtherKd = fu.tree; pividx = fu.pividx;
      pivot = fu.pivot; pivobji = fu.pivobji;
    }
  else
    { pivot = itsPivot; pividx = itsPivotIndex; pivobji = itsObjIndex; }

  // decrement our search count anf forget about further exploration
  // if we have reached 0:
  -- searchSteps;

  // If the further HR is too far away, it will be hopeless and we
  // don't even need to look at it (step 10):
  if (searchSteps > 0 && furtherHr.isInReach(target, maxDistSq))
    {
      // ok the further HR may contain a better (or second better)
      // match. First let's check how close our pivot is to the
      // target (modified steps 10.1.1 to 10.1.3):
      int ptDistSq = pivot->distSquared(target);
      if (ptDistSq < distSq1)
        {
          // our pivot is closer than our nearest, so make the pivot
          // our new nearest:
          nearest = pividx;

          // if our pivot is in the same object than our previous
          // nearest, then just slide the distances; otherwise, make
          // distsq2 maximal:
          if (nobjidx == pivobji)
            { distSq2 = distSq1; distSq1 = ptDistSq; }
          else
            { distSq1 = ptDistSq; distSq2 = maxdsq; nobjidx = pivobji; }

          // update the max distance for further search:
          maxDistSq = distSq1;
        }
      else if (ptDistSq < distSq2)
        {
          // our pivot is farther than the nearest but closer than the
          // second nearest, so just update the second nearest
          // distance and max distance, but only if our pivot is in
          // the same object as our nearest:
          if (nobjidx == pivobji)
            { distSq2 = ptDistSq; maxDistSq = distSq2; }
        }

      // Recursively explore the further HR (modified step 10.2):
      int tempDistSq1 = maxdsq, tempDistSq2 = maxdsq;
      uint tempNearest = 0U, tempobjidx = 0U;
      if (furtherKd.is_valid())
        tempNearest = furtherKd->
          nearestNeighborBBFI(target, furtherHr, searchSteps, maxDistSq,
                              tempDistSq1, tempDistSq2, maxdsq,
                              bbfq, tempobjidx);

      // If we found a better nearest or second nearest, update
      // accordingly (modified step 10.3):
      if (tempDistSq1 < distSq1)
        {
          // tempNearest is better than nearest, so use tempNearest:
          nearest = tempNearest;

          // now, who is the second best?
          if (tempobjidx == nobjidx)
            {
              // temp is in the same object as nearest used to be. So
              // let's just slide the second-best distances:
              if (tempDistSq2 < distSq1) distSq2 = tempDistSq2;
              else distSq2 = distSq1;
            }
          else
            {
              // temp in different object from old nearest:
              distSq2 = maxdsq;
              nobjidx = tempobjidx;
            }

          // also update our best distance:
          distSq1 = tempDistSq1;
        }
      else if (tempDistSq1 < distSq2)
        {
          // tempNearest is worse than nearest but better than our
          // second nearest, so just update our second nearest
          // distance, but only if in same object:
          if (tempobjidx == nobjidx)
            distSq2 = tempDistSq1;
        }
    }

  // return the best we have found. Note: nearest may be empty if we
  // did not find anything:
  distsq1 = distSq1; distsq2 = distSq2; objidx = nobjidx;
  return nearest;
}

// ######################################################################
uint KDTree::goodCandidate(const std::vector< rutz::shared_ptr<Keypoint> >& exset,
                           uint& splitDim)
{
  const uint numkeys = exset.size();
  if (numkeys == 0) { LFATAL("Keypoint list given is empty!"); return 0; }

  // get the dimensionality of our keypoints:
  uint dim = exset[0]->getFVlength();

  // initialize temporary hr search min/max values:
  std::vector<byte> minHr(dim, 255), maxHr(dim, 0);

  // go over the examplar set and adjust min/max HR:
  for (uint i = 0; i < numkeys; i++)
    {
      rutz::shared_ptr<Keypoint> dom = exset[i];
      for (uint k = 0; k < dim; k++)
        {
          const byte val = dom->getFVelement(k);
          if (val < minHr[k]) minHr[k] = val;
          if (val > maxHr[k]) maxHr[k] = val;
        }
    }

  // find the dim with maximum range; will be our splitting dimension:
  int maxDiff = 0;
  for (uint k = 0; k < dim; k++)
    {
      const int diffHr = int(maxHr[k]) - int(minHr[k]);
      if (diffHr > maxDiff) { maxDiff = diffHr; splitDim = k; }
    }

  // the splitting dimension is splitDim. Now find an exemplar as
  // close to the arithmetic middle as possible:
  const int middle = (maxDiff >> 1) + int(minHr[splitDim]);
  int exemMinDiff = 256; uint middleidx = 0;

  for (uint i = 0; i < numkeys; i++)
    {
      const int curDiff = std::abs(exset[i]->getFVelement(splitDim) - middle);
      if (curDiff < exemMinDiff) { exemMinDiff = curDiff; middleidx = i; }
    }

  // return the middle exemplar (and splitDim):
  return middleidx;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
