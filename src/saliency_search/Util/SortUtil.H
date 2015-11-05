/*!@file Util/SortUtil.H Utilities that relate to sorting stuff */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/SortUtil.H $
// $Id: SortUtil.H 14814 2011-06-06 08:15:25Z kai $
//

#ifndef UTIL_SORTUTIL_H_DEFINED
#define UTIL_SORTUTIL_H_DEFINED

#include <vector>
#include <utility>

namespace util {
  //! Get ranks of elements in a vector
  /*! Compute the rank of each element ina vector and return all the
      ranks in a vector. ranks should initially be empty and will be
      filled with rank values for each element in vec, hence upon return
      it will have the same size as vec, and ranks[0] is the rank of
      vec[0], etc. Rank of 0 is for the smallest element in vec, rank 1
      for the second smallest, etc. You need to have operator< defined on T. */
  template <class T> inline
  void sortrank(const std::vector<T>& vec, std::vector<size_t>& ranks);

  template <class T> inline
  void sortrank(const std::deque<T>& vec, std::deque<size_t>& ranks);


  // ######################################################################
  // inline function implementations
  // ######################################################################
  template <class T> inline
  bool sortrankcmp(const std::pair<const T*, size_t>& a, const std::pair<const T*, size_t>& b)
  { return ( (*(a.first)) < (*(b.first)) ); }

  template <class T> inline
  void sortrank(const std::vector<T>& vec, std::vector<size_t>& ranks)
  {
    typedef typename std::pair<const T*, size_t> TT;

    std::vector<TT> v;
    const size_t n = vec.size();

    for (size_t i = 0; i < n; ++i) v.push_back(TT(&vec[i], i));

    std::sort(v.begin(), v.end(), sortrankcmp<T>);

    ranks.clear();
    for (size_t i = 0; i < n; ++i) ranks.push_back(v[i].second);
  }

  template <class T> inline
  void sortrank(const std::vector<T>& vecA, std::vector<T>& vecB, std::vector<size_t>& ranks)
  {

    typedef typename std::pair<const T*, size_t> TT;

    std::vector<TT> v;
    const size_t n = vecA.size();

    for (size_t i = 0; i < n; ++i) v.push_back(TT(&vecA[i], i));

    std::sort(v.begin(), v.end(), sortrankcmp<T>);

    ranks.clear();
    for (size_t i = 0; i < n; ++i) ranks.push_back(v[i].second);


    for (size_t i = 0; i < n; ++i) vecB.push_back(vecA[ranks[i]]);

  }


  //Matlab style sort [vecB,vecIX] = sort(vecA)
  template <class T> inline
  void sortindex(const std::vector<T>& vecA, std::vector<T>& vecB, std::vector<size_t>& vecIX)
  {
    typedef typename std::pair<const T*, size_t> TT;

    std::vector<TT> v;
    std::vector<size_t> ranks;
    const size_t n = vecA.size();

    for (size_t i = 0; i < n; ++i) v.push_back(TT(&vecA[i], i));

    std::sort(v.begin(), v.end(), sortrankcmp<T>);

    ranks.clear();
    for (size_t i = 0; i < n; ++i) ranks.push_back(v[i].second);

    vecIX.resize(vecA.size());
    for (size_t i = 0; i < n; ++i) vecIX[ranks[i]] = i;

    for (size_t i = 0; i < n; ++i) vecB.push_back(vecA[ranks[i]]);

	
  }



  template <class T> inline
  void sortrank(const std::deque<T>& vec, std::deque<size_t>& ranks)
  {
    typedef typename std::pair<const T*, size_t> TT;

    std::deque<TT> v;
    const size_t n = vec.size();

    for (size_t i = 0; i < n; ++i) v.push_back(TT(&vec[i], i));

    std::sort(v.begin(), v.end(), sortrankcmp<T>);

    ranks.clear();
    for (size_t i = 0; i < n; ++i) ranks.push_back(v[i].second);
  }


}; // namespace util

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_SORTUTIL_H_DEFINED
