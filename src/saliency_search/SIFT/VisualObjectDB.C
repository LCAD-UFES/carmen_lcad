/*!@file SIFT/VisualObjectDB.C Visual object database */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/VisualObjectDB.C $
// $Id: VisualObjectDB.C 15310 2012-06-01 02:29:24Z itti $
//

#include "SIFT/VisualObjectDB.H"
#include "SIFT/Keypoint.H"
#include "SIFT/KDTree.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/Timer.H"
#include "Util/WorkThreadServer.H"

#include <fstream>


// #######################################################################
VisualObjectDB::VisualObjectDB() :
  itsName(), itsObjects(), itsKDTree(), itsKDindices()
{ }

// #######################################################################
VisualObjectDB::~VisualObjectDB()
{ }

// ######################################################################
bool VisualObjectDB::loadFrom(const std::string& fname, bool preloadImage)
{
  const char *fn = fname.c_str();
  LINFO("Loading Visual Object database: '%s'...", fn);

  std::ifstream inf(fn);
  if (inf.is_open() == false) { LERROR("Cannot open '%s' -- USING EMPTY", fn); return false; }

  inf>>(*this);

  inf.close();
  LINFO("Done. Loaded %u VisualObjects.", numObjects());
  return true;
}

// ######################################################################
bool VisualObjectDB::saveTo(const std::string& fname)
{
  const char *fn = fname.c_str();
  LINFO("Saving database '%s'...", fn);

  std::ofstream outf(fn);
  if (outf.is_open() == false) { LERROR("Cannot open %s for writing -- NOT SAVED", fn); return false; }

  outf<<(*this);

  outf.close();
  LINFO("Done. Saved %u VisualObjects.", numObjects());
  return true;
}

// #######################################################################
bool VisualObjectDB::addObject(const rutz::shared_ptr<VisualObject>& obj, bool uniqueName)
{
  if (uniqueName)
  {
    std::string objectName = obj->getName();

    std::vector< rutz::shared_ptr<VisualObject> >::const_iterator
      vo = itsObjects.begin(), stop = itsObjects.end();

    while(vo != stop)
    {
      if ((*vo)->getName().compare(objectName) == 0) return false;
      ++ vo;
    }
  }

  // not found, add it to the end of our list:
  itsObjects.push_back(obj);

  // any KDTree we may have had is now invalid:
  itsKDTree.reset();
  itsKDindices.clear();

  return true;
}

// ######################################################################
// functor to assist with VisualObjectMatch sorting:
class moreVOM
{
public:
  moreVOM(const float kcoeff, const float acoeff) :
    itsKcoeff(kcoeff), itsAcoeff(acoeff)
  { }

  bool operator()(const rutz::shared_ptr<VisualObjectMatch>& x,
                  const rutz::shared_ptr<VisualObjectMatch>& y)
  { return ( x->getScore(itsKcoeff, itsAcoeff) >
             y->getScore(itsKcoeff, itsAcoeff) ); }

private:
  float itsKcoeff, itsAcoeff;
};

// ######################################################################
void VisualObjectDB::buildKDTree()
{
  // if we have one, no-op:
  if (itsKDTree.is_valid()) return;

  LINFO("Building KDTree for %" ZU " objects...", itsObjects.size());

  // go over all our objects and build a giant vector of keypoints,
  // remembering the mapping between indices:
  itsKDindices.clear(); uint objidx = 0U;
  std::vector< rutz::shared_ptr<Keypoint> > allkps;

  std::vector< rutz::shared_ptr<VisualObject> >::const_iterator
    obj = itsObjects.begin(), stop = itsObjects.end();

  while (obj != stop)
    {
      const std::vector< rutz::shared_ptr<Keypoint> >& kps = (*obj)->getKeypoints();
      uint kidx = 0U;
      std::vector< rutz::shared_ptr<Keypoint> >::const_iterator
        kp = kps.begin(), stopk = kps.end();

      while(kp != stopk)
        {
          // get the keypoint:
          allkps.push_back(*kp);

          // remember its object number and index within that object:
          itsKDindices.push_back(std::pair<uint, uint>(objidx, kidx));

          ++ kp; ++kidx;
        }
        ++ obj; ++objidx;
    }

  // all right, 'allkps' now has all our keypoints, and itsKDindices
  // their object number and indices within objects. Let's build a
  // KDTree from allkps:
  itsKDTree.reset(new KDTree(allkps));

  LINFO("Done. KDTree initialized with %" ZU " keypoints.", allkps.size());
}

// ######################################################################
uint VisualObjectDB::
getObjectMatches(const rutz::shared_ptr<VisualObject> obj,
                 std::vector< rutz::shared_ptr<VisualObjectMatch> >& matches,
                 const VisualObjectMatchAlgo algo, const uint maxn,
                 const float kcoeff, const float acoeff,
                 const float minscore, const uint mink,
                 const uint kthresh, const bool sortbypf)
{
  LDEBUG("Matching '%s' against database...", obj->getName().c_str());
  Timer tim(1000000);
  matches.clear(); uint nm = 0U;

  switch(algo)
    {
      // ####################################################################
    case VOMA_SIMPLE:                    // #################### simple match
      {
        const uint nobj = itsObjects.size();

        std::vector<uint> sidx;
        if (sortbypf) computeSortedIndices(sidx, obj);

        for (uint i = 0; i < nobj; i ++)
          {
            // get the index:
            const uint index = sortbypf ? sidx[i] : i;

            // attempt a match:
            rutz::shared_ptr<VisualObjectMatch>
              match(new VisualObjectMatch(obj, itsObjects[index],
                                          algo, kthresh));

            // apply some standard pruning:
            match->prune(std::max(25U, mink * 5U), mink);

            // if the match is good enough, store it:
            if (match->size() >= mink &&
                match->getScore(kcoeff, acoeff) >= minscore &&
                match->checkSIFTaffine())
              {
                matches.push_back(match); ++nm;

                // have we found enough matches?
                if (nm >= maxn) break;
              }

            // otherwise, match runs out of scope here and its memory is
            // deallocated thanks to rutz::shared_ptr
          }
      }
      break;

      // ####################################################################
    case VOMA_KDTREE:                    // #################### KDTree match
    case VOMA_KDTREEBBF:
      {
        const uint nobj = itsObjects.size();
        const uint kthresh2 = kthresh * kthresh;

        // build a KDTree if we don't already have one:
        buildKDTree();

        // get matches between our KDTree and the test object:
        const uint tstnkp = obj->numKeypoints();
        if (tstnkp == 0U) break; // nothing to do
        const int maxdsq = obj->getKeypoint(0)->maxDistSquared();

        // prepare a list to get our keypoint matches: first index is
        // object number, second keypoint match number:
        std::vector< std::vector<KeypointMatch> > kpm(nobj);

        // the code here is similar to VisualObjectMatch::matchKDTree()
        // loop over all of our test object's keypoints:
        for (uint i = 0; i < tstnkp; i++)
          {
            int distsq1 = maxdsq, distsq2 = maxdsq;
            rutz::shared_ptr<Keypoint> tstkey = obj->getKeypoint(i);

            // find nearest neighbor in our KDTree:
            uint matchIndex = (algo == VOMA_KDTREEBBF) ?
              itsKDTree->nearestNeighborBBF(tstkey, 40, distsq1, distsq2) :
              itsKDTree->nearestNeighbor(tstkey, distsq1, distsq2);

            // Check that best distance less than 0.6 of second best distance:
            if (100U * distsq1 < kthresh2 * distsq2)
              {
                const uint refobjnum = itsKDindices[matchIndex].first;
                const uint refkpnum = itsKDindices[matchIndex].second;

                // note how we swap the ref/test here. This is for
                // compatibility with the Simple matching above. In
                // the list of object matches we will eventually
                // return, 'obj' is our ref object while an object in
                // the database is considered test object:
                KeypointMatch m;
                m.refkp = tstkey;
                m.tstkp = itsObjects[refobjnum]->getKeypoint(refkpnum);
                m.distSq = distsq1;
                m.distSq2 = distsq2;
                kpm[refobjnum].push_back(m);
              }
          }

        // let's see which objects accumulated enough keypoint matches:
        for (uint i = 0U; i < nobj; i ++)
          {
            // do we have nough keypoint matches for that object?
            if (kpm[i].size() >= mink)
              {
                // ok, let's make a pre-build VisualObjectMatch out of it:
                rutz::shared_ptr<VisualObjectMatch>
                  match(new VisualObjectMatch(obj, itsObjects[i], kpm[i]));

                // apply some standard pruning:
                match->prune(std::max(25U, mink * 5U), mink);

                // if the match is good enough, store it:
                if (match->size() >= mink &&
                    match->getScore(kcoeff, acoeff) >= minscore &&
                    match->checkSIFTaffine())
                  {
                    matches.push_back(match); ++nm;

                    // have we found enough matches?
                    if (nm >= maxn) break;
                  }
              }
          }
      }
      break;

      // ####################################################################
    default:
      LFATAL("Unknown matching algo %d", int(algo));
    }

  // finally, sort our matches:
  std::sort(matches.begin(), matches.end(), moreVOM(kcoeff, acoeff));

  uint64 t = tim.get();
  LDEBUG("Found %u database object matches for '%s' in %.3fms",
        nm, obj->getName().c_str(), float(t) * 0.001F);

  return nm;
}

// ######################################################################
class MatchJob : public JobServer::Job {
public:
  MatchJob(const rutz::shared_ptr<VisualObject> obj_, const rutz::shared_ptr<VisualObject> obj2_,
           pthread_mutex_t *mut_, std::vector< rutz::shared_ptr<VisualObjectMatch> >& matches_,
           const float kcoeff_, const float acoeff_, const float minscore_, const uint mink_, const uint kthresh_) :
    JobServer::Job(), obj(obj_), obj2(obj2_), mut(mut_), matches(matches_), kcoeff(kcoeff_),
    acoeff(acoeff_), minscore(minscore_), mink(mink_), kthresh(kthresh_) { }

  virtual ~MatchJob() { }

  virtual void run() {
    // do the matching:
    rutz::shared_ptr<VisualObjectMatch> match(new VisualObjectMatch(obj, obj2, VOMA_SIMPLE, kthresh));

    // apply some standard pruning:
    match->prune(std::max(25U, mink * 5U), mink);

    // if the match is good enough, store it:
    if (match->size() >= mink && match->getScore(kcoeff, acoeff) >= minscore && match->checkSIFTaffine())
      {
        pthread_mutex_lock(mut);
        matches.push_back(match);
        pthread_mutex_unlock(mut);
      }
  }

  virtual const char* jobType() const { return "MatchJob"; }

private:
  const rutz::shared_ptr<VisualObject> obj;
  const rutz::shared_ptr<VisualObject> obj2;
  pthread_mutex_t *mut;
  std::vector< rutz::shared_ptr<VisualObjectMatch> >& matches;
  const float kcoeff;
  const float acoeff;
  const float minscore;
  const uint mink;
  const uint kthresh;
};

// ######################################################################
uint VisualObjectDB::
getObjectMatchesParallel(const rutz::shared_ptr<VisualObject> obj,
                 std::vector< rutz::shared_ptr<VisualObjectMatch> >& matches,
                 const uint numthreads, const float kcoeff, const float acoeff,
                 const float minscore, const uint mink, const uint kthresh, const bool sortbypf)
{
  LDEBUG("Parallel matching '%s' against database...", obj->getName().c_str());
  Timer tim(1000000); matches.clear();

  pthread_mutex_t mut;
  if (pthread_mutex_init(&mut, NULL)) PLFATAL("Error creating mutex");
  WorkThreadServer wts("Match Server", numthreads);

  const uint nobj = itsObjects.size();

  std::vector<uint> sidx;
  if (sortbypf) computeSortedIndices(sidx, obj);

  for (uint i = 0; i < nobj; i ++)
    {
      // get the index:
      const uint index = sortbypf ? sidx[i] : i;

      // Enqueue a match job:
      wts.enqueueJob(rutz::make_shared(new MatchJob(obj, itsObjects[index], &mut, matches, kcoeff, acoeff,
                                                    minscore, mink, kthresh)));
    }

  // wait until all completed:
  wts.flushQueue();
  if (pthread_mutex_destroy(&mut)) PLERROR("Error in pthread_mutex_destroy");

  // finally, sort our matches:
  std::sort(matches.begin(), matches.end(), moreVOM(kcoeff, acoeff));

  uint64 t = tim.get();
  LDEBUG("Found %" ZU " database object matches for '%s' in %.3fms", matches.size(),
         obj->getName().c_str(), float(t) * 0.001F);

  return matches.size();
}

// #######################################################################
void VisualObjectDB::computeSortedIndices(std::vector<uint>& indices,
                                          const rutz::shared_ptr<VisualObject>&
                                          obj) const
{
  // delete any old indices:
  indices.clear();

  // get a list of pairs <featuredistsq, index>:
  std::vector< std::pair<double, uint> > lst;

  for(uint i = 0; i < itsObjects.size(); i++)
    lst.push_back(std::pair<double, uint>(itsObjects[i]->
                                          getFeatureDistSq(obj), i));

  // sort the list. NOTE: operator< on std::pair is lexicographic,
  // i.e., here, first by our first element (the distance), then by
  // our second element (the index, in the unlikely event we have two
  // equal distances):
  std::sort(lst.begin(), lst.end());

  // pull all the indices out:
  std::vector< std::pair<double, uint> >::const_iterator
    ptr = lst.begin(), stop = lst.end();
  while (ptr != stop)
    { indices.push_back(ptr->second); ++ptr; }
}

// #######################################################################
std::istream& operator>>(std::istream& is, VisualObjectDB& vdb)
{
  vdb.createVisualObjectDB(is, vdb);
  return is;
}


// ###############
void VisualObjectDB::
createVisualObjectDB(std::istream& is, VisualObjectDB& vdb, bool preloadImage)
{
  std::string name;
  std::getline(is, name);

  vdb.setName(name);

  uint siz; is>>siz;

  vdb.itsObjects.clear(); vdb.itsObjects.resize(siz);

  std::vector< rutz::shared_ptr<VisualObject> >::iterator
    vo = vdb.itsObjects.begin(), stop = vdb.itsObjects.end();

  while (vo != stop)
    {
      rutz::shared_ptr<VisualObject> newvo(new VisualObject());
      is>>(*newvo);
      *vo++ = newvo;
    }
}

// #######################################################################
std::ostream& operator<<(std::ostream& os, const VisualObjectDB& vdb)
{
  os<<vdb.getName()<<std::endl;
  os<<vdb.itsObjects.size()<<std::endl;

  std::vector< rutz::shared_ptr<VisualObject> >::const_iterator
    vo = vdb.itsObjects.begin(), stop = vdb.itsObjects.end();

  while (vo != stop) { os<<(**vo); ++ vo; }

  return os;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
