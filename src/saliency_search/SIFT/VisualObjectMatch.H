/*!@file SIFT/VisualObjectMatch.H Header for visual object matches */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/VisualObjectMatch.H $
// $Id: VisualObjectMatch.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef VISUALOBJECTMATCH_H_DEFINED
#define VISUALOBJECTMATCH_H_DEFINED

#include "Image/Pixels.H"
#include "Image/Image.H"
#include "SIFT/Keypoint.H"
#include "SIFT/KeypointMatch.H"
#include "SIFT/SIFTaffine.H"
#include "SIFT/VisualObjectMatchAlgo.H"
#include <vector>
class KDTree;
class VisualObject;

//! Simple class to store lists of Keypoint matches between two VisualObject
class VisualObjectMatch {
public:

  //! Constructor from two visual objects
  /*! If using a KDTree matching algo, the KDTree will be built in
    'voref' (with caching). So voref should usually be the object that
    has more keypoints than votest. */
  VisualObjectMatch(const rutz::shared_ptr<VisualObject>& voref,
                    const rutz::shared_ptr<VisualObject>& votest,
                    const VisualObjectMatchAlgo algo,
                    const uint thresh = 7U);

  //! Constructor from a pre-built KDTree and a VisualObject
  /*! The KDTree should have been filled already and is considered the
    'reference' object. Matching algo must be either VOMA_KDTREE or
    VOMA_KDTREEBBF. */
  VisualObjectMatch(const rutz::shared_ptr<KDTree>& kdref,
                    const rutz::shared_ptr<VisualObject>& votest,
                    const VisualObjectMatchAlgo algo,
                    const uint thresh = 7U);

  //! Build from a precomputed list of KeypointMatch matches
  /*! Normally you would not want to call this unless you got your
    KeypointMatch matches from somewhere. This is used by
    VisualObjectDB during KDTree-based matching. */
  VisualObjectMatch(const rutz::shared_ptr<VisualObject>& voref,
                    const rutz::shared_ptr<VisualObject>& votest,
                    const std::vector<KeypointMatch>& kpm);

  //! Destructor
  ~VisualObjectMatch();

  //! Add a Keypoint match to the back of our internal list
  inline void push_back(const KeypointMatch& km);

  //! Access a KeypointMatch, read-only version
  inline const KeypointMatch& operator[](const uint idx) const;

  //! Access a KeypointMatch, read/write version
  inline KeypointMatch& operator[](const uint idx);

  //! get a given KeypointMatch
  inline const KeypointMatch& getKeypointMatch(const uint index) const;

  //! Get number of Keypoint matches we currently have
  inline uint size() const;

  //! Get the reference VisualObject
  inline const rutz::shared_ptr<VisualObject>& getVoRef() const;

  //! Get the test VisualObject
  inline const rutz::shared_ptr<VisualObject>& getVoTest() const;

  //! Apply a standard series of prunings
  /*! This is a heuristic combination of calls to pruneByDist(),
    pruneByHough(), and pruneByAff(), so as to prune outlier matches
    and allow the recovery of a clean affine transform through
    getSIFTaffine(). Do not prune to fewer than 'minn' matches but try
    to prune down to fewer than 'maxn' matches (the latter is not
    guaranteed, as all matches may be very very good). Returns the
    number of outlier matches pruned away. */
  uint prune(const uint maxn = 25U, const uint minn = 3U);

  //! Prune our matches by ratio of best to second best distance
  /*! Normally you should just use prune() but this is made public for
    people who want finer control. The given thresh is in units of
    10%. For example, if thresh=8, matches where the match distance is
    more than 0.8 the second best distance will be eliminated. Returns
    the number of matches pruned. Do not prune to fewer than 'minn'
    matches. */
  uint pruneByDist(const uint thresh = 7U, const uint minn = 3U);

  //! Prune the matches using a Hough transform
  /*! Normally you should just use prune() but this is made public for
    people who want finer control. Returns the number of matches
    pruned. Do not prune to fewer than 'minn' matches. Prune matches
    that disagree with the most popular transform by more than
    'rangefac' times the range in any dimension; sensible values are
    between 0.05 (extremely strict; not recommended since the family
    of geometric transformations used here is very approximative) and
    0.5 (may not prune anything). */
  uint pruneByHough(const float rangefac = 0.25F, const uint minn = 3U);

  //! Prune our matches by consistency with affine transform
  /*! Normally you should just use prune() but this is made public for
    people who want finer control. Compute the affine transform from
    the matches, and eliminate the matches that disagree with it in
    that the distance in the test image between an affine-transformed
    reference keypoint and the matching test keypoint is larger than
    'dist'. During this process, we will not continue if fewer than
    'minn' matches remain. The number of outlier matches that were
    deleted is returned. */
  uint pruneByAff(const float dist = 5.0F, const uint minn = 3U);

  //! Compute least-squares affine transform between matches
  /*! If we have not already done so, we will compute the affine from
    all our Keypoint matches. Hence it is recommended that you call
    prune() prior to invoking this member function. */
  inline SIFTaffine getSIFTaffine();

  //! Check the SIFT affine for weirdness
  /*! This returns false if our affine represents too gross of a
    distortion between the two matched objects.
    @param maxrot max allwoed rotation; valid values are in [0..pi];
      default is to allow any rotation.
    @param maxscale max allowed scaling; valid values are > 0.0;
      default is to allow scaling from 1:10 to 10:1.
    @param maxshear max allowed shearing; valid values are >= 0.0;
      default is to allow shearing between -0.5 and 0.5. */
  bool checkSIFTaffine(const float maxrot = 10.0F, const float maxscale = 30.0F,
                       const float maxshear = 10.0F);

  //! Get a match score, higher scores are better
  /*! Score returned here is defined as kcoeff / (1 +
    getKeypointAvgDist()) + acoeff / (1 + getAffineAvgDist()) +
    0.05 * numKeypointMatches */
  float getScore(const float kcoeff = 0.5F, const float acoeff = 0.5F);

  //! get a match score based on the salient feature difference *
  //! spatial distance
  float getSalScore(const float wcoeff = 0.0F, const float hcoeff = 0.0F);

  //! spatial distance of the salient points
  float getSalDist();

  //! salient feature vector difference
  float getSalDiff();

  //! Compute a matching score based on average residual distsq btw keys
  /*! Normally you would just use getScore() but this is made public
    for people who want finer control. The distance here is scaled so
    as to become comparable to that of getKeypointAvgDist(). */
  float getKeypointAvgDist();

  //! Compute matching score based on average residual distance between keys
  /*! Normally you would just use getScore() but this is made public
    for people who want finer control.  You should call this after you
    have pruned. We here just get the SIFTaffine and compute the
    average residual distance between each ref keypoint transformed by
    the affine and the corresponding test keypoint. Units hence are
    pixels in the test image. */
  float getAffineAvgDist();

  //! Get a combo image with SIFT Keypoint matches
  Image< PixRGB<byte> > getMatchImage(const float scale = 1.0F) const;

  //! Get a combo image with SIFT Keypoint matches
  //! this one has a frame around it so that different size images
  //! can be reconciled and the offset can also be added
  Image< PixRGB<byte> > getMatchImage(Dims frameSize,
                                      Point2D<int> refOffset, Point2D<int> testOffset,
                                      const float scale = 1.0F) const;

  //! Get the image of the test object transformed to match the ref object
  /*! If an uninitialized image is given, the resulting image has the
    size of the ref object's, and contains zeros everywhere except
    where the test object is. Otherwise, the test object is just
    painted into the given image (which must have the dims of the ref
    image. */
  Image< PixRGB<byte> > getTransfTestImage(const Image< PixRGB<byte> >&
                                           im = Image< PixRGB<byte> >());

  //! Get the transformed coords of the 4 corners of the test image
  /*! The returned points have coords in the coord system of the ref
    image, but may fall outside that image. */
  void getTransfTestOutline(Point2D<int>& tl, Point2D<int>& tr,
                            Point2D<int>& br, Point2D<int>& bl);

  //! Get the image of the test object fused on top of that of the ref obj
  /*! The resulting image has the size of the ref object's. A mixing
    factor of 1.0 means that the ref image gets a coeff 1.0 and the
    test image 0.0. To get the affine transform between the two
    images, getSIFTaffine will be called, without any pruning. So you
    may want to prune and clean the matches before you get that final
    display. */
  Image< PixRGB<byte> > getFusedImage(const float mix = 0.5F);

  //! Get our list of Keypoint matches
  inline const std::vector<KeypointMatch>& getKeypointMatches() const;

  //! get the spatial distance between the two objects
  /*! the offsets passed in are the the coordinates of
    the top left corner of each image. The result is the distance of the
    two respective origins, which is also the coordinate change of the
    two objects.
    This is useful for visual objects that come from a cropped frame.
    The result can be the camera movement or egomotion
   */
  Point2D<int> getSpatialDist(Point2D<int> objOffset1 = Point2D<int>(0,0),
                         Point2D<int> objOffset2 = Point2D<int>(0,0));

  //! get the overlap rectangle of the match
  Rectangle getOverlapRect();

  //! check overlap using the image rectangle overlap
  bool isOverlapping();

  //! check overlap using keypoint bounding box overlap
  bool isOverlapping2();

private:
  VisualObjectMatch(const VisualObjectMatch& m);  //!< forbid copy-contruction
  VisualObjectMatch& operator=(const VisualObjectMatch& m); //!< forbid copy

  //! our first (reference) object
  rutz::shared_ptr<VisualObject> itsVoRef;

  //! our second (test) object
  rutz::shared_ptr<VisualObject> itsVoTest;

  //! matches between our objects
  std::vector<KeypointMatch> itsMatches;

  //! KDTree for our reference object
  rutz::shared_ptr<KDTree> itsKDTree;

  //! did we already compute an affine?
  bool itsHasAff;

  //! our current affine transform
  SIFTaffine itsAff;

  //! did we already compute the keypoint average distance?
  bool itsHasKpAvgDist;

  //! current keypoint average distance
  float itsKpAvgDist;

  //! did we already compute the affine average distance?
  bool itsHasAfAvgDist;

  //! current affine average distance
  float itsAfAvgDist;

  // Find matching Keypoints using simple brute-force algo. Returns
  // number of Keypoint matches.
  uint matchSimple(const uint thresh);

  // Find matching Keypoints using a KDTree to accelerate search for
  // matches. The KDTree will be built in the itsVoRef object. Hence,
  // the ref object should usually be one with many more keypoints
  // than the test object. Note that KDTrees are cached, so only the
  // first time matchKDTree() is called will the KDTree actually be
  // built. Returns number of Keypoint matches. If bbf is 0, then
  // normal (very slow) KDTree matching is done; otherwise, BBF
  // matching is done with up to bbf steps.
  uint matchKDTree(const uint thresh, const int bbf);

  // compute our affine transform based on all our matches. This
  // should be called only once outliers have been removed.
  void computeAffine();

  // get keypoint differences; used internally by pruneByHough()
  inline void getKdiff(const KeypointMatch& km, float& dx,
                       float& dy, float& doo, float& ds) const;
};

// ######################################################################
// ########## INLINED FUNCTIONS
// ######################################################################
inline uint VisualObjectMatch::size() const
{ return itsMatches.size(); }

inline void VisualObjectMatch::push_back(const KeypointMatch& km)
{ itsMatches.push_back(km); }

inline const KeypointMatch&
VisualObjectMatch::operator[](const uint idx) const
{ return itsMatches[idx]; }

inline KeypointMatch&
VisualObjectMatch::operator[](const uint idx)
{ return itsMatches[idx]; }

inline const KeypointMatch&
VisualObjectMatch::getKeypointMatch(const uint index) const
{
  ASSERT(index < itsMatches.size());
  return itsMatches[index];
}

inline const rutz::shared_ptr<VisualObject>&
VisualObjectMatch::getVoRef() const
{ return itsVoRef; }

inline const rutz::shared_ptr<VisualObject>&
VisualObjectMatch::getVoTest() const
{ return itsVoTest; }

inline SIFTaffine VisualObjectMatch::getSIFTaffine()
{
  if (itsHasAff == false) computeAffine();
  return itsAff;
}

inline void VisualObjectMatch::getKdiff
( const KeypointMatch& km, float& dx,
  float& dy, float& doo, float& ds) const
{
  rutz::shared_ptr<Keypoint> refkp = km.refkp;
  rutz::shared_ptr<Keypoint> tstkp = km.tstkp;

  dx = tstkp->getX() - refkp->getX();
  dy = tstkp->getY() - refkp->getY();
  ds = tstkp->getS() - refkp->getS();
  doo = tstkp->getO() - refkp->getO();
  while (doo < -M_PI) doo += 2.0F * M_PI;
  while (doo > M_PI) doo -= 2.0F * M_PI;
}

inline const std::vector<KeypointMatch>&
VisualObjectMatch::getKeypointMatches() const
{ return itsMatches; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
