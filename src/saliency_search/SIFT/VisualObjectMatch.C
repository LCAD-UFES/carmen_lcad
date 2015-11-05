/*!@file SIFT/VisualObjectMatch.C Visual Object matches */

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
// Primary maintainer for this file: Philip Williams <plw@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/VisualObjectMatch.C $
// $Id: VisualObjectMatch.C 15310 2012-06-01 02:29:24Z itti $
//

#include "SIFT/VisualObjectMatch.H"
#include "SIFT/VisualObject.H"
#include "SIFT/KDTree.H"
#include "SIFT/SIFThough.H"
#include "Image/MatrixOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"

// ######################################################################
VisualObjectMatch::VisualObjectMatch(const rutz::shared_ptr<VisualObject>& voref,
                                     const rutz::shared_ptr<VisualObject>& votest,
                                     const VisualObjectMatchAlgo algo,
                                     const uint thresh) :
  itsVoRef(voref), itsVoTest(votest), itsMatches(), itsKDTree(),
  itsHasAff(false), itsAff(),
  itsHasKpAvgDist(false), itsHasAfAvgDist(false)
{
  uint nm = 0U;

  switch (algo)
    {
    case VOMA_SIMPLE: nm = matchSimple(thresh); break;
    case VOMA_KDTREE: nm = matchKDTree(thresh, 0); break;
    case VOMA_KDTREEBBF: nm = matchKDTree(thresh, 40); break;
    }
  LDEBUG("Got %u KP matches (th=%d) btw %s and %s",
         nm, thresh, voref->getName().c_str(), votest->getName().c_str());
}

// ######################################################################
VisualObjectMatch::VisualObjectMatch(const rutz::shared_ptr<KDTree>& kdref,
                                     const rutz::shared_ptr<VisualObject>& votest,
                                     const VisualObjectMatchAlgo algo,
                                     const uint thresh) :
  itsVoRef(new VisualObject("KDref")), itsVoTest(votest), itsMatches(),
  itsKDTree(kdref), itsHasAff(false), itsAff(),
  itsHasKpAvgDist(false), itsHasAfAvgDist(false)
{
  uint nm = 0U;

  switch (algo)
    {
    case VOMA_SIMPLE:
      LFATAL("Can't use Simple match when constructing from a KDTree"); break;
    case VOMA_KDTREE: nm = matchKDTree(thresh, 0); break;
    case VOMA_KDTREEBBF: nm = matchKDTree(thresh, 40); break;
    }
  LDEBUG("Got %u KP matches (th=%d) btw %s and %s",
         nm, thresh, itsVoRef->getName().c_str(), votest->getName().c_str());
}

// ######################################################################
VisualObjectMatch::VisualObjectMatch(const rutz::shared_ptr<VisualObject>& voref,
                                     const rutz::shared_ptr<VisualObject>& votest,
                                     const std::vector<KeypointMatch>& kpm) :
  itsVoRef(voref), itsVoTest(votest), itsMatches(kpm),
  itsKDTree(), itsHasAff(false), itsAff(),
  itsHasKpAvgDist(false), itsHasAfAvgDist(false)
{
  LDEBUG("Got %" ZU " KP matches btw %s and %s",
         itsMatches.size(), itsVoRef->getName().c_str(),
         itsVoTest->getName().c_str());
}

// ######################################################################
VisualObjectMatch::~VisualObjectMatch()
{ }

// ######################################################################
uint VisualObjectMatch::prune(const uint maxn, const uint minn)
{
  // do not go below a min number of matches:
  if (itsMatches.size() <= minn) return 0U;
  uint ndel = 0U;

  // if we have lots of matches, start by cutting some of them easily
  // by enforcing a better best-to-second-best keypoint match
  // ratio. But don't go too far in this direction, as sometimes
  // poorer matches may be the correct ones:
  const uint targetn1 = maxn * 2U; uint distthresh = 9U;
  while (itsMatches.size() > targetn1 && distthresh > 5)
    ndel += pruneByDist(distthresh--, targetn1);
  LDEBUG("After pruning by distance: total %d outliers pruned.", ndel);

  // let's now do a Hough-based pruning:
  const uint targetn2 = (maxn + minn) / 2; uint iter = 2U;
  while (itsMatches.size() > targetn2 && iter > 0)
    { ndel += pruneByHough(0.6F, targetn2); --iter; }
  LDEBUG("After pruning by Hough: total %d outliers pruned.", ndel);

  // finally a few passes of pruning by inconsistency with the full
  // affine transform. Hopefully by now gross outliers have been
  // eliminated (especially by the Hough transform) and this will work:
  const uint targetn3 = minn; float dist = 5.0F; iter = 3U;
  while (itsMatches.size() > targetn3 && iter > 0)
    { ndel += pruneByAff(dist, targetn3); dist *= 0.75F; --iter; }
  LDEBUG("After pruning by affine: total %d outliers pruned.", ndel);

  return ndel;
}

// ######################################################################
uint VisualObjectMatch::pruneByDist(const uint thresh, const uint minn)
{
  // do not go below a min number of matches:
  if (itsMatches.size() <= minn) return 0U;
  std::vector<KeypointMatch>::iterator itr = itsMatches.begin();
  const uint t2 = thresh * thresh; uint ndel = 0U;

  while (itr < itsMatches.end())
    {
      if (100U * itr->distSq >= t2 * itr->distSq2)
        {
          itr = itsMatches.erase(itr); ++ ndel;
          if (itsMatches.size() <= minn) return ndel;
        }
      else ++ itr;
      // note: the behavior of vector::erase() guarantees this code works...
    }
 return ndel;
}

// ######################################################################
uint VisualObjectMatch::pruneByHough(const float rangefac, const uint minn)
{
  // do not go below a min number of matches:
  if (itsMatches.size() <= minn) return 0U;
  std::vector<KeypointMatch>::iterator
    itr = itsMatches.begin(), stop = itsMatches.end();

  // do a first pass over the matches to determine the range:
  float dxmi = 1.0e30F, dxma = -1.0e30F; // difference in X position
  float dymi = 1.0e30F, dyma = -1.0e30F; // difference in Y position
  float domi = 1.0e30F, doma = -1.0e30F; // difference in orientation
  float dsmi = 1.0e30F, dsma = -1.0e30F; // difference in scale
  while (itr < stop)
    {
      // get the keypoint differences: dx is the difference between X
      // of the test keypoint and X of the ref keypoint, etc:
      float dx, dy, doo, ds;
      getKdiff(*itr, dx, dy, doo, ds);

      if (dx < dxmi) dxmi = dx; else if (dx > dxma) dxma = dx;
      if (dy < dymi) dymi = dy; else if (dy > dyma) dyma = dy;
      if (ds < dsmi) dsmi = ds; else if (ds > dsma) dsma = ds;
      if (doo < domi) domi = doo; else if (doo > doma) doma = doo;

      ++ itr;
    }
  //LINFO("dx = [%f .. %f]", dxmi, dxma);
  //LINFO("dy = [%f .. %f]", dymi, dyma);
  //LINFO("do = [%f .. %f]", domi, doma);
  //LINFO("ds = [%f .. %f]", dsmi, dsma);

  // make sure our ranges are not empty:
  if (dxma - dxmi < 1.0F) dxma = dxmi + 1.0F;
  if (dyma - dymi < 1.0F) dyma = dymi + 1.0F;
  if (doma - domi < 1.0e-3F) doma = domi + 1.0e-3F;
  if (dsma - dsmi < 1.0e-3F) dsma = dsmi + 1.0e-3F;

  // we are going to divide each range into eight bins:
  const float facx = 8.0F / (dxma - dxmi);
  const float facy = 8.0F / (dyma - dymi);
  const float faco = 8.0F / (doma - domi);
  const float facs = 8.0F / (dsma - dsmi);

  // let's populate a SIFThough:
  SIFThough h;
  itr = itsMatches.begin();
  while (itr < stop)
    {
      // get the keypoint differences:
      float dx, dy, doo, ds;
      getKdiff(*itr, dx, dy, doo, ds);

      // add to our Hough accumulator:
      h.addValue((dx - dxmi) * facx, (dy - dymi) * facy,
                 (doo - domi) * faco, (ds - dsmi) * facs, 1.0F);

      ++ itr;
    }

  // all right, let's get the peak out:
  float peakx, peaky, peako, peaks;
  h.getPeak(peakx, peaky, peako, peaks);

  // convert back from bin to real coordinates:
  peakx = peakx / facx + dxmi;
  peaky = peaky / facy + dymi;
  peako = peako / faco + domi;
  peaks = peaks / facs + dsmi;
  //LINFO("Peak at dx=%f, dy=%f, do=%f, ds=%f", peakx, peaky, peako, peaks);

  // compute the acceptable range:
  const float rxmi = peakx - rangefac * (dxma - dxmi);
  const float rxma = peakx + rangefac * (dxma - dxmi);
  const float rymi = peaky - rangefac * (dyma - dymi);
  const float ryma = peaky + rangefac * (dyma - dymi);
  const float romi = peako - rangefac * (doma - domi);
  const float roma = peako + rangefac * (doma - domi);
  const float rsmi = peaks - rangefac * (dsma - dsmi);
  const float rsma = peaks + rangefac * (dsma - dsmi);

  // let's prune away matches that are more than some fraction of the
  // range from the peak:
  uint ndel = 0U; itr = itsMatches.begin();
  while (itr < itsMatches.end())  // do not use 'stop' as size will shrink
    {
      // get the keypoint differences:
      float dx, dy, doo, ds;
      getKdiff(*itr, dx, dy, doo, ds);

      // prune that outlier?
      if (dx < rxmi || dx > rxma ||
          dy < rymi || dy > ryma ||
          doo < romi || doo > roma ||
          ds < rsmi || ds > rsma)
        {
          itr = itsMatches.erase(itr); ++ ndel;
          if (itsMatches.size() <= minn) return ndel;
        }
      else
        ++ itr;
    }
  return ndel;
}

// ######################################################################
uint VisualObjectMatch::pruneByAff(const float dist, const uint minn)
{
  // do not go below a min number of matches:
  if (itsMatches.size() <= minn) return 0U;
  uint ndel = 0U; const float dist2 = dist * dist;

  // get our affine transform given our current matches:
  computeAffine();

  // loop over our matches and find the outliers:
  std::vector<KeypointMatch>::iterator itr = itsMatches.begin();
  while (itr < itsMatches.end())
    {
      // get residual distance between affine-transformed ref
      // keypoint and test keypoint:
      const float d = itsAff.getResidualDistSq(*itr);

      if (d > dist2)
        {
          // delete that outlier match:
          itr = itsMatches.erase(itr); ++ndel;

          // do not go below a min number of remaining matches:
          if (itsMatches.size() <= minn) return ndel;
        }
      else
        ++ itr;
    }
  return ndel;
}

// ######################################################################
void VisualObjectMatch::computeAffine()
{
  const uint nmatches = itsMatches.size();

  // we require at least 3 matches for this to work:
  if (nmatches < 3)
    {
      LDEBUG("Too few matches (%u) -- RETURNING IDENTITY", nmatches);
      itsAff = SIFTaffine();  // default constructor is identity
      return;
    }

  // we are going to solve the linear system Ax=b in the least-squares sense
  Image<float> A(3, nmatches, NO_INIT);
  Image<float> b(2, nmatches, NO_INIT);

  for (uint i = 0; i < nmatches; i ++)
    {
      rutz::shared_ptr<Keypoint> refkp = itsMatches[i].refkp;
      rutz::shared_ptr<Keypoint> tstkp = itsMatches[i].tstkp;

      A.setVal(0, i, refkp->getX());
      A.setVal(1, i, refkp->getY());
      A.setVal(2, i, 1.0f);

      b.setVal(0, i, tstkp->getX());
      b.setVal(1, i, tstkp->getY());
    }

  try
    {
      // the solution to Ax=b is x = [A^t A]^-1 A^t b:
      Image<float> At = transpose(A);

      Image<float> x =
        matrixMult(matrixMult(matrixInv(matrixMult(At, A)), At), b);

      // store into our SIFTaffine:
      itsAff.m1 = x.getVal(0, 0); itsAff.m3 = x.getVal(1, 0);
      itsAff.m2 = x.getVal(0, 1); itsAff.m4 = x.getVal(1, 1);
      itsAff.tx = x.getVal(0, 2); itsAff.ty = x.getVal(1, 2);

      // ok, we have it:
      itsHasAff = true;
    }
  catch (SingularMatrixException& e)
    {
      LDEBUG("Couldn't invert matrix -- RETURNING IDENTITY");
      itsAff = SIFTaffine(); // default constructor is identity
      itsHasAff = false;
    }
}

// ######################################################################
bool VisualObjectMatch::checkSIFTaffine(const float maxrot,
                                        const float maxscale,
                                        const float maxshear)
{
  if (itsHasAff == false) computeAffine();
  if (itsAff.isInversible() == false) return false;

  float theta, sx, sy, str;
  itsAff.decompose(theta, sx, sy, str);

  LDEBUG("theta=%fdeg sx=%f sy=%f shx=%f shy=%f",
         theta * 180.0F / M_PI, sx, sy, str/sx, str/sy);

  // check the rotation:
  if (fabsf(theta) > maxrot) return false;

  // check the scaling:
  if (fabsf(sx) > maxscale || fabsf(sx) < 1.0F / maxscale) return false;
  if (fabsf(sy) > maxscale || fabsf(sy) < 1.0F / maxscale) return false;

  // check the shearing. Note: from the previous check, we are
  // guaranteed that sx and sy are non-zero:
  if (fabsf(str/sx) > maxshear) return false;
  if (fabsf(str/sy) > maxshear) return false;

  // if we get here, the affine is not weird:
  return true;
}

// ######################################################################
float VisualObjectMatch::getScore(const float kcoeff,
                                  const float acoeff)
{
  if(!itsHasKpAvgDist) getKeypointAvgDist();
  if(!itsHasAfAvgDist) getAffineAvgDist();

  // keypoint average distance
  // FIX: figure out a good ratio with the affine
  //      cap out at .05 for now
  float kp = itsKpAvgDist; if(kp < .05) kp = .05;

  // affine average distance: capped out at .05
  // object that close in affine transform is probably a good match
  // no need to insert in a lower value
  // otherwise it will saturates the other factor
  float af = itsAfAvgDist; if(af < .05) af = .05;

  // number of keypoint matches score
  // cap at 20 matches: .05 * 20 = 1.0
  // any bigger should not add more weight; they're all good matches
  float nm =  0.05F * float(itsMatches.size()); if(nm > 1.0) nm = 1.0F;

  // matching score:
  // max value: 21.0: .5/.05 + .5/.05 + 1.0 = 10.0 + 10.0 + 1.0
  float score = kcoeff/kp + acoeff/af + nm;

  return score;
}

// ######################################################################
float VisualObjectMatch::getSalScore(const float wcoeff,
                                     const float hcoeff )
{
  // score = feature similarity * distance
  float sscore = getSalDiff();
  if(sscore == -1.0F) return sscore;
  float sdist = getSalDist();
  if(sdist == -1.0F) return sscore;

  float maxDist = 0.0;
  rutz::shared_ptr<VisualObject> obj1 = getVoRef();
  rutz::shared_ptr<VisualObject> obj2 = getVoTest();
  if(wcoeff == 0.0F && hcoeff == 0.0F)
    {
      uint w1 = obj1->getImage().getWidth();
      uint h1 = obj1->getImage().getHeight();
      float dist1 = sqrt(w1*w1 + h1*h1);
      uint w2 = obj2->getImage().getWidth();
      uint h2 = obj2->getImage().getHeight();
      float dist2 = sqrt(w2*w2 + h2*h2);
      maxDist = dist1 + dist2;
    }
  else
    {
      maxDist = sqrt(wcoeff * wcoeff + hcoeff * hcoeff);
    }
  float dscore = 1.0 - sdist/maxDist;
  LINFO("dist score : 1.0 - %f/%f = %f", sdist, maxDist, dscore);

  float score = dscore * sscore;
  LINFO("dist * sim:  %f * %f = %f", dscore, sscore, score);

  return score;
}

// ######################################################################
float VisualObjectMatch::getSalDiff()
{
  // check if both has salient points and feature vectors
  rutz::shared_ptr<VisualObject> obj1 = getVoRef();
  rutz::shared_ptr<VisualObject> obj2 = getVoTest();
  const std::vector<float>& feat1 = obj1->getFeatures();
  const std::vector<float>& feat2 = obj2->getFeatures();
  bool compfeat = ((feat1.size() > 0) && (feat1.size() == feat2.size()));
  if (!compfeat) return -1.0F;

  // feature similarity  [ 0.0 ... 1.0 ]:
  float cval = 0.0;
  for(uint i = 0; i < feat1.size(); i++)
    {
      const float val = feat1[i] - feat2[i];
      cval += val * val;
    }
  cval = sqrtf(cval / feat1.size());
  float sscore =  1.0F - cval;
  LDEBUG("cval: %f: score: %f", cval, sscore);
  return sscore;
}

// ######################################################################
float VisualObjectMatch::getSalDist()
{
  // check if both has salient points and feature vectors
  rutz::shared_ptr<VisualObject> obj1 = getVoRef();
  rutz::shared_ptr<VisualObject> obj2 = getVoTest();
  Point2D<int> salpt1 = obj1->getSalPoint();
  Point2D<int> salpt2 = obj2->getSalPoint();
  bool compfeat = ((salpt1.i != -1) && (salpt2.i != -1));
  if(!compfeat) return -1.0F;

  // forward affine transform [A * ref -> tst ]
  SIFTaffine aff = getSIFTaffine();
  float u, v; aff.transform(salpt1.i, salpt1.j, u, v);
  float dist = salpt2.distance(Point2D<int>(int(u+0.5F),int(v+0.5F)));
  LDEBUG("pos1: (%d,%d) -> (%f,%f) & pos2: (%d,%d): dist: %f",
         salpt1.i, salpt1.j, u, v, salpt2.i, salpt2.j, dist);
  return dist;
}

// ######################################################################
float VisualObjectMatch::getKeypointAvgDist()
{
  if(itsHasKpAvgDist) return itsKpAvgDist;
  if (itsMatches.size() == 0U) return 1.0e30F;

  float d = 0.0F;
  std::vector<KeypointMatch>::const_iterator
    itr = itsMatches.begin(), stop = itsMatches.end();
  float fac = 1.0F; if (itr != stop) fac /= float(itr->refkp->getFVlength());

  while(itr != stop)
    {
      d += sqrtf(float(itr->refkp->distSquared(itr->tstkp)) * fac); ++itr;
    }

  itsKpAvgDist = 0.1F * d / float(itsMatches.size());
  itsHasKpAvgDist = true;

  return itsKpAvgDist;
}

// ######################################################################
float VisualObjectMatch::getAffineAvgDist()
{
  if(itsHasAfAvgDist) return itsAfAvgDist;
  if (itsMatches.size() < 3U) return 1.0e30F;
  if (itsHasAff == false) computeAffine();

  float d = 0.0F;
  std::vector<KeypointMatch>::const_iterator
    itr = itsMatches.begin(), stop = itsMatches.end();

  while(itr != stop)
    { d += sqrtf(itsAff.getResidualDistSq(*itr)); ++itr; }

  itsAfAvgDist = d / float(itsMatches.size());
  itsHasAfAvgDist = true;

  return itsAfAvgDist;
}

// ######################################################################
uint VisualObjectMatch::matchSimple(const uint thresh)
{
  const uint refnkp = itsVoRef->numKeypoints();
  const uint tstnkp = itsVoTest->numKeypoints();
  if (refnkp == 0 || tstnkp == 0) return 0U;

  const int maxdsq = itsVoRef->getKeypoint(0)->maxDistSquared();
  uint nmatches = 0; uint thresh2 = thresh * thresh;

  // loop over all of the test object's keypoints:
  for (uint i = 0; i < tstnkp; i++)
    {
      int distsq1 = maxdsq, distsq2 = maxdsq;
      rutz::shared_ptr<Keypoint> tstkey = itsVoTest->getKeypoint(i);
      rutz::shared_ptr<Keypoint> refkey;

      // loop over all of the ref object's keypoints:
      for (uint j = 0; j < refnkp; j ++)
        {
          rutz::shared_ptr<Keypoint> rkey = itsVoRef->getKeypoint(j);
          const int distsq = rkey->distSquared(tstkey);

          // is this better than our best one?
          if (distsq < distsq1)
            {
              distsq2 = distsq1; // old best becomes second best
              distsq1 = distsq;  // we got a new best
              refkey = rkey;     // remember the best keypoint
            }
          else if (distsq < distsq2)  // maybe between best and second best?
            distsq2 = distsq;
        }

      // Check that best distance less than thresh of second best distance:
      if (100U * distsq1 < thresh2 * distsq2)
        {
          KeypointMatch m;
          m.refkp = refkey; m.tstkp = tstkey;
          m.distSq = distsq1; m.distSq2 = distsq2;
          itsMatches.push_back(m);
          ++ nmatches;
        }
    }

  // return number of matches:
  return nmatches;
}

// ######################################################################
uint VisualObjectMatch::matchKDTree(const uint thresh, const int bbf)
{
  const uint refnkp = itsVoRef->numKeypoints();
  const uint tstnkp = itsVoTest->numKeypoints();
  if (refnkp == 0 || tstnkp == 0) return 0U;
  const int maxdsq = itsVoRef->getKeypoint(0)->maxDistSquared();
  uint nmatches = 0; uint thresh2 = thresh * thresh;

  // do we already have a valid KDTree for our ref object? Otherwise
  // let's build one:
  if (itsKDTree.is_invalid())
    {
      LINFO("Building KDTree for VisualObject '%s'...",
            itsVoRef->getName().c_str());
      itsKDTree.reset(new KDTree(itsVoRef->getKeypoints()));
      LINFO("KDTree for VisualObject '%s' complete.",
            itsVoRef->getName().c_str());
    }

  // loop over all of our test object's keypoints:
  for (uint i = 0; i < tstnkp; i++)
    {
      int distsq1 = maxdsq, distsq2 = maxdsq;
      rutz::shared_ptr<Keypoint> tstkey = itsVoTest->getKeypoint(i);

      // find nearest neighbor in our KDTree:
      uint matchIndex = bbf > 0 ?
        itsKDTree->nearestNeighborBBF(tstkey, bbf, distsq1, distsq2) :
        itsKDTree->nearestNeighbor(tstkey, distsq1, distsq2);

      // Check that best distance less than 0.6 of second best distance:
      if (100U * distsq1 < thresh2 * distsq2)
        {
          KeypointMatch m;
          m.refkp = itsVoRef->getKeypoint(matchIndex); m.tstkp = tstkey;
          m.distSq = distsq1; m.distSq2 = distsq2;
          itsMatches.push_back(m);
          ++ nmatches;
        }
    }

  // return number of matches:
  return nmatches;
}

// ######################################################################
Image< PixRGB<byte> >
VisualObjectMatch::getMatchImage(const float scale) const
{
  // get a keypoint image (without vectors) for both objects:
  Image< PixRGB<byte> > refimg = itsVoRef->getKeypointImage(scale, 0.0F);
  Image< PixRGB<byte> > tstimg = itsVoTest->getKeypointImage(scale, 0.0F);
  LDEBUG("r[%d %d] t[%d %d]", 
         refimg.getWidth(), refimg.getHeight(),
         tstimg.getWidth(), tstimg.getHeight());

  // put ref image on top of the other:
  int refdx, tstdx;
  const int w = std::max(refimg.getWidth(), tstimg.getWidth());
  const int dy = refimg.getHeight();
  const PixRGB<byte> greycol(128), linkcol(255, 200, 100);
  Image< PixRGB<byte> > combo(w, dy + tstimg.getHeight(), ZEROS);

  if (refimg.getWidth() > tstimg.getWidth())
    { refdx = 0; tstdx = (w - tstimg.getWidth()) / 2; }
  else
    { refdx = (w - refimg.getWidth()) / 2; tstdx = 0; }

  if (refimg.coordsOk(itsVoRef->getSalPoint()))
    drawDisk(refimg, itsVoRef->getSalPoint(),  3, PixRGB<byte>(255,255,0));

  if (tstimg.coordsOk(itsVoTest->getSalPoint()))
    drawDisk(tstimg, itsVoTest->getSalPoint(), 3, PixRGB<byte>(255,255,0));

  inplacePaste(combo, refimg, Point2D<int>(refdx, 0));
  inplacePaste(combo, tstimg, Point2D<int>(tstdx, dy));

  drawLine(combo, Point2D<int>(0, dy-1), Point2D<int>(w-1, dy-1), greycol);
  drawLine(combo, Point2D<int>(0, dy), Point2D<int>(w-1, dy), greycol);

  // let's link the matched keypoints:
  const uint nm = itsMatches.size();
  for (uint i = 0; i < nm; i ++)
    {
      rutz::shared_ptr<Keypoint> refk = itsMatches[i].refkp;
      rutz::shared_ptr<Keypoint> tstk = itsMatches[i].tstkp;

      drawLine(combo,
               Point2D<int>(int(refk->getX() * scale + 0.5F) + refdx,
                            int(refk->getY() * scale + 0.5F)),
               Point2D<int>(int(tstk->getX() * scale + 0.5F) + tstdx,
                            int(tstk->getY() * scale + 0.5F) + dy),
               linkcol);
    }

  return combo;
}

// ######################################################################
Image< PixRGB<byte> >
VisualObjectMatch::getMatchImage(Dims frameSize,
                                 Point2D<int> refOffset, Point2D<int> testOffset,
                                 const float scale) const
{
  int w = frameSize.w();
  int h = frameSize.h();

  // get a keypoint image (without vectors) for both objects:
  Image< PixRGB<byte> > refimg =
    getVoRef()->getKeypointImage(scale, 0.0F);
  Image< PixRGB<byte> > tstimg =
    getVoTest()->getKeypointImage(scale, 0.0F);

  // draw the salient point locations
  drawDisk(refimg, getVoRef()->getSalPoint(), 3, PixRGB<byte>(255,255,0));
  drawDisk(tstimg, getVoTest()->getSalPoint(), 3, PixRGB<byte>(255,255,0));

  const PixRGB<byte> greycol(128), linkcol(255, 200, 100);
  Image< PixRGB<byte> > combo(w, 2*h, ZEROS);

  // put ref image on top of the other:
  inplacePaste(combo, refimg, Point2D<int>(0, 0)+ refOffset);
  inplacePaste(combo, tstimg, Point2D<int>(0, h)+ testOffset);

  drawLine(combo, Point2D<int>(0, h-1), Point2D<int>(w-1, h-1), greycol);

  // let's link the matched keypoints:
  std::vector<KeypointMatch> matches = getKeypointMatches();

  const uint nm = matches.size();
  for (uint i = 0; i < nm; i ++)
    {
      rutz::shared_ptr<Keypoint> refk = matches[i].refkp;
      rutz::shared_ptr<Keypoint> tstk = matches[i].tstkp;

      drawLine
        (combo,
         Point2D<int>(int(refk->getX() * scale + 0.5F),
                 int(refk->getY() * scale + 0.5F))
         + refOffset,
         Point2D<int>(int(tstk->getX() * scale + 0.5F),
                 int(tstk->getY() * scale + 0.5F))
         + testOffset + Point2D<int>(0,h),
         linkcol);
    }

  return combo;
}

// ######################################################################
Image< PixRGB<byte> > VisualObjectMatch::
getTransfTestImage(const Image< PixRGB<byte> >& im)
{
  SIFTaffine aff = getSIFTaffine();

  // we loop over all pixel locations in the ref image, transform the
  // coordinates using the forward affine transform, get the pixel
  // value in the test image, and plot it:
  Image< PixRGB<byte> > result(im);
  if (result.initialized() == false)
    result.resize(itsVoRef->getImage().getDims(), true);
  Image< PixRGB<byte> > tsti = itsVoTest->getImage();

  uint w = result.getWidth(), h = result.getHeight();
  Image< PixRGB<byte> >::iterator dptr = result.beginw();

  for (uint j = 0; j < h; j ++)
    for (uint i = 0; i < w; i ++)
      {
        float u, v;
        aff.transform(float(i), float(j), u, v);

        if (tsti.coordsOk(u, v))
          *dptr++ = tsti.getValInterp(u, v);
        else
          ++dptr;
      }
  return result;
}

// ######################################################################
void VisualObjectMatch::
getTransfTestOutline(Point2D<int>& tl, Point2D<int>& tr, Point2D<int>& br, Point2D<int>& bl)
{
  SIFTaffine a = getSIFTaffine();
  SIFTaffine aff = a.inverse();

  // transform the four corners of the test image using the inverse affine:

  const Dims objSize = itsVoTest->getObjectSize();
  const uint w = objSize.w();
  const uint h = objSize.h();
  float u, v;

  aff.transform(0.0F, 0.0F, u, v);
  tl.i = int(u + 0.5F); tl.j = int(v + 0.5F);

  aff.transform(float(w-1), 0.0F, u, v);
  tr.i = int(u + 0.5F); tr.j = int(v + 0.5F);

  aff.transform(float(w-1), float(h-1), u, v);
  br.i = int(u + 0.5F); br.j = int(v + 0.5F);

  aff.transform(0.0F, float(h-1), u, v);
  bl.i = int(u + 0.5F); bl.j = int(v + 0.5F);
}

// ######################################################################
Image< PixRGB<byte> > VisualObjectMatch::getFusedImage(const float mix)
{
  SIFTaffine aff = getSIFTaffine();

  // we loop over all pixel locations in the ref image, transform the
  // coordinates using the forward affine transform, get the pixel
  // value in the test image, and mix:
  Image< PixRGB<byte> > refi = itsVoRef->getImage();
  Image< PixRGB<byte> > tsti = itsVoTest->getImage();

  uint w = refi.getWidth(), h = refi.getHeight();
  Image< PixRGB<byte> > result(w, h, NO_INIT);
  Image< PixRGB<byte> >::const_iterator rptr = refi.begin();
  Image< PixRGB<byte> >::iterator dptr = result.beginw();

  for (uint j = 0; j < h; j ++)
    for (uint i = 0; i < w; i ++)
      {
        float u, v;
        aff.transform(float(i), float(j), u, v);
        PixRGB<byte> rval = *rptr++;

        if (tsti.coordsOk(u, v))
          {
            PixRGB<byte> tval = tsti.getValInterp(u, v);
            PixRGB<byte> mval = PixRGB<byte>(rval * mix + tval * (1.0F - mix));
            *dptr++ = mval;
          }
        else
          *dptr++ = PixRGB<byte>(rval * mix);
      }
  return result;
}

// ######################################################################
Point2D<int> VisualObjectMatch::getSpatialDist
( Point2D<int> offset1, Point2D<int> offset2)
{
  // get the matches
  // translation is: t = Ar + b
  //   [testX]   [ a.m1 a.m2 ] [refX]   [ a.tx ]
  //   [testY] = [ a.m3 a.m4 ] [refY] + [ a.ty ]
  SIFTaffine a = getSIFTaffine();

  // get the needed matrix
  Image<double> A(2,2, ZEROS);
  A.setVal(0, 0, a.m1);    A.setVal(1, 0, a.m2);
  A.setVal(0, 1, a.m3);    A.setVal(1, 1, a.m4);

  Image<double> b(1,2, ZEROS);
  b.setVal(0, 0, a.tx);
  b.setVal(0, 1, a.ty);

  Image<double>  r(1,2,ZEROS);
  r.setVal(0,0, -offset1.i);
  r.setVal(0,1, -offset1.j);

  LINFO("[ %7.3f %7.3f ][ %7.3f ]   [ %7.3f ]",
        A.getVal(0,0),  A.getVal(1,0),  r.getVal(0,0), b.getVal(0,0));
  LINFO("[ %7.3f %7.3f ][ %7.3f ] + [ %7.3f ]",
        A.getVal(0,1),  A.getVal(1,1),  r.getVal(0,1), b.getVal(0,1));

  Image<double> diff = matrixMult(A,r) + b;
  Point2D<int> diffPt = Point2D<int>(int(diff.getVal(0,0)),
                           int(diff.getVal(0,1)));
  Point2D<int> res = diffPt + offset2;

  LINFO("diff:[%d %d] + offset:[%d %d] = [%d %d]",
        diffPt.i, diffPt.j, offset2.i, offset2.j, res.i, res.j);
  return res;
}

// ######################################################################
Rectangle VisualObjectMatch::getOverlapRect()
{
  // NOTE: overlap assume only translational transformation

  SIFTaffine aff = getSIFTaffine();

  // we loop over all pixel locations in the ref image, transform the
  // coordinates using the forward affine transform [A * ref -> tst ]
  // get the pixel value in the ref image and estimate

  Image< PixRGB<byte> > refi = getVoRef()->getImage();
  Image< PixRGB<byte> > tsti = getVoTest()->getImage();

  uint wr = refi.getWidth(), hr = refi.getHeight();
  uint wt = tsti.getWidth(), ht = tsti.getHeight();
  LDEBUG("r[%d,%d]  t[%d,%d]", wr, hr, wt, ht);

  // get the affine transforms of the test image corners
  // top-left corner of refI
  float u, v;
  aff.transform(float(0), float(0), u, v);
  float t = v, l = u, b = v, r = u;
  LDEBUG("t-l: [%f,%f]: [%f,%f,%f,%f]", u, v, t, l, b, r);

  // top-right corner of refI
  aff.transform(float(wr-1), float(0), u, v);
  if(u < l) l = u;  if(u > r) r = u;
  if(v < t) t = v;  if(v > b) b = v;
  LDEBUG("t-r: [%f,%f]: [%f,%f,%f,%f]", u, v, t, l, b, r);

  // bottom-left corner of refI
  aff.transform(float(0), float(hr-1), u, v);
  if(u < l) l = u;  if(u > r) r = u;
  if(v < t) t = v;  if(v > b) b = v;
  LDEBUG("b-l: [%f,%f]: [%f,%f,%f,%f]", u, v, t, l, b, r);

  // bottom-right corner of refI
  aff.transform(float(wr-1), float(hr-1), u, v);
  if(u < l) l = u;  if(u > r) r = u;
  if(v < t) t = v;  if(v > b) b = v;
  LDEBUG("b-r: [%f,%f]: [%f,%f,%f,%f]", u, v, t, l, b, r);

  const Rectangle refr = Rectangle::tlbrI(int(t+0.5F), int(l+0.5F),
                                         int(b+0.5F), int(r+0.5F));
  const Rectangle tstr = tsti.getBounds();
  const Rectangle ovl = refr.getOverlap(tstr);
  LINFO("overlap at: [%d, %d, %d, %d], %d, %d",
        ovl.left(), ovl.top(), ovl.rightI(), ovl.bottomI(),
        ovl.width(),ovl.height());

  return ovl;
}

// ######################################################################
bool VisualObjectMatch::isOverlapping()
{
  Image< PixRGB<byte> > refi = getVoRef()->getImage();
  Image< PixRGB<byte> > tsti = getVoTest()->getImage();

  const Rectangle a = refi.getBounds();
  const Rectangle b = tsti.getBounds();

  // print the rectangles
  const Rectangle ovl = getOverlapRect();

  LINFO("o:[%d, %d, %d, %d], [%d,%d] dbo:[%d, %d, %d, %d], [%d,%d]",
        a.left(), a.top(), a.rightI(), a.bottomI(), a.width(), a.height(),
        b.left(), b.top(), b.rightI(), b.bottomI(), b.width(), b.height() );
  LINFO("overlap at: [%d, %d, %d, %d], %d, %d",
        ovl.left(), ovl.top(), ovl.rightI(), ovl.bottomI(),
        ovl.width(),ovl.height());

  // check percentage of overlap 50%
  if(!ovl.isValid()) { LINFO("do not overlap"); return false; }
  float ovlA = (ovl.width() * ovl.height())/(a.width() * a.height()+0.0);
  float ovlB = (ovl.width() * ovl.height())/(b.width() * b.height()+0.0);
  LINFO("ovl: a=  %f, b=  %f", ovlA, ovlB);
  if((ovlA > .5 && ovlB > .5) || (ovlA > .8) || (ovlB > .8))
    { LINFO("the objects overlap"); return true; }
  else
    { LINFO("objects do not significantly overlap"); return false; }
      // maybe later check which one is the smaller one
}

// ######################################################################
bool VisualObjectMatch::isOverlapping2()
{
  rutz::shared_ptr<VisualObject> obj1 = getVoRef();
  rutz::shared_ptr<VisualObject> obj2 = getVoTest();
  LINFO("obj1 size %d, obj2 size: %d match size: %d",
        obj1->numKeypoints(), obj2->numKeypoints(), size());

  // get object 1 borders - from keypoints
  float t1 = -1.0f, b1 = -1.0f, l1 = -1.0f, r1 = -1.0f;
  if(obj1->numKeypoints() > 0)
    {
      float x = obj1->getKeypoint(0)->getX();
      float y = obj1->getKeypoint(0)->getY();
      t1 = y, b1 = y, l1 = x, r1 = x;
      //LINFO("[%f %f]",x,y);
    }
  for(uint i = 1; i < obj1->numKeypoints(); i++)
    {
      float x = obj1->getKeypoint(i)->getX();
      float y = obj1->getKeypoint(i)->getY();

      if(t1 > y) t1 = y; else if(b1 < y) b1 = y;
      if(l1 > x) l1 = x; else if(r1 < x) r1 = x;
      //LINFO("[%f %f]",x,y);
    }
  float area1 = (b1 - t1)*(r1 - l1);
  LINFO("obj1[%d]: t1: %f, b1: %f, l1: %f, r1: %f; A: %f",
        obj1->numKeypoints(), t1, b1, l1, r1, area1);

  // get object 1 borders - from keypoints
  float t2 = -1.0f, b2 = -1.0f, l2 = -1.0f, r2 = -1.0f;
  if(obj2->numKeypoints() > 0)
    {
      float x = obj2->getKeypoint(0)->getX();
      float y = obj2->getKeypoint(0)->getY();
      t2 = y, b2 = y, l2 = x, r2 = x;
      //LINFO("[%f %f]",x,y);
    }
  for(uint i = 1; i < obj2->numKeypoints(); i++)
    {
      float x = obj2->getKeypoint(i)->getX();
      float y = obj2->getKeypoint(i)->getY();

      if(t2 > y) t2 = y; else if(b2 < y) b2 = y;
      if(l2 > x) l2 = x; else if(r2 < x) r2 = x;
      //LINFO("[%f %f]",x,y);
    }
  float area2 = (b2 - t2)*(r2 - l2);
  LINFO("obj2[%d]: t2: %f, b2: %f, l2: %f, r2: %f; A: %f",
        obj2->numKeypoints(), t2, b2, l2, r2, area2);

  // go through all the matches from the ref side
  float tm1 = -1.0f, bm1 = -1.0f, lm1 = -1.0f, rm1 = -1.0f;
  if(size() > 0)
    {
      float x = itsMatches[0].refkp->getX();
      float y = itsMatches[0].refkp->getY();
      tm1 = y, bm1 = y, lm1 = x, rm1 = x;
      //LINFO("[%f %f]",x,y);
    }
  for(uint i = 1; i < size(); i++)
    {
      float x = itsMatches[i].refkp->getX();
      float y = itsMatches[i].refkp->getY();

      if(tm1 > y) tm1 = y; else if(bm1 < y) bm1 = y;
      if(lm1 > x) lm1 = x; else if(rm1 < x) rm1 = x;
      //LINFO("[%f %f]",x,y);
    }
  float aream1 = (bm1 - tm1)*(rm1 - lm1);
  LINFO("m1[%d]: tm1: %f, bm1: %f, lm1: %f, rm1: %f; Am: %f",
        size(), tm1, bm1, lm1, rm1, aream1);

  // go through all the matches from the tst side
  float tm2 = -1.0f, bm2 = -1.0f, lm2 = -1.0f, rm2 = -1.0f;
  if(size() > 0)
    {
      float x = itsMatches[0].tstkp->getX();
      float y = itsMatches[0].tstkp->getY();
      tm2 = y, bm2 = y, lm2 = x, rm2 = x;
      //LINFO("[%f %f]",x,y);
    }
  for(uint i = 1; i < size(); i++)
    {
      float x = itsMatches[i].tstkp->getX();
      float y = itsMatches[i].tstkp->getY();

      if(tm2 > y) tm2 = y; else if(bm2 < y) bm2 = y;
      if(lm2 > x) lm2 = x; else if(rm2 < x) rm2 = x;
      //LINFO("[%f %f]",x,y);
    }
  float aream2 = (bm2 - tm2)*(rm2 - lm2);
  LINFO("m2[%d]: tm2: %f, bm2: %f, lm2: %f, rm2: %f; Am: %f",
        size(), tm2, bm2, lm2, rm2, aream2);

  // if the incoming object overlaps by less than 20%
  // of its middle quarter
  float lmid = l2 + (r2-l2)/4;  float rmid = r2 - (r2-l2)/4;
  float lo = lm2; if(lo < lmid) lo = lmid;
  float ro = rm2; if(ro > rmid) ro = rmid;
  float wo = 0.0; if(ro > lo) wo = ro - lo;

  float tmid = t2 + (b2-t2)/4;  float bmid = b2 - (b2-t2)/4;
  float to = tm2; if(to < tmid) to = tmid;
  float bo = bm2; if(bo > bmid) bo = bmid;
  float ho = 0.0; if(bo > to) ho = bo - to;
  float ao = ho * wo;

  bool ret = true; if(ao/(area2/4.0) < .3) ret = false;
  LINFO("wo: %f ho: %f ao/(a2/4) = %f/%f = %f < .4 -> ret = %d",
        wo,ho, ao, area2/4.0, ao/(area2/4.0), ret);

  bool ret2 = true; if(ao/aream2 < .4) ret2 = false;
  LINFO("wo: %f ho: %f ao/(am2) = %f/%f = %f < .5 -> ret = %d",
        wo,ho, ao, aream2, ao/aream2, ret2);
  return ret || ret2;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
