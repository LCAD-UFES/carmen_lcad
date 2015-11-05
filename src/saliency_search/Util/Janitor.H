/*!@file Util/Janitor.H Simple utility class for running a cleanup function during stack unwinding */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Janitor.H $
// $Id: Janitor.H 7958 2007-02-20 23:30:52Z rjpeters $
//

#ifndef UTIL_JANITOR_H_DEFINED
#define UTIL_JANITOR_H_DEFINED

/// Simple utility class for running a cleanup function during stack unwinding
/** This is useful in wrapping C APIs where resources are managed with
    acquisition and release functions. In that case, you can create a
    Janitor<SomeType> with a pointer to the C release function, so
    that the release function will be automatically called when the
    enclosing scope is exited (whether by function return or by
    exception propagation). */
template <class T>
struct Janitor
{
  typedef void (void_func)(void*);

  typedef void (free_func)(T, void_func*);

  template <class F>
  static void typed_freeit(T t, void_func* vf)
  {
    if (vf != 0 && t != 0)
      (*reinterpret_cast<F>(vf))(t);
  }

  template <class F>
  Janitor(T t, F f)
    :
    it(t),
    vfunc(reinterpret_cast<void_func*>(f)),
    freeit(&typed_freeit<F>)
  {}

  ~Janitor()
  {
    if (freeit != 0 && it != 0)
      (*freeit)(it, vfunc);
  }

  T release()
  {
    freeit = 0;
    return it;
  }

  T it;

private:
  void_func* vfunc;
  free_func* freeit;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_JANITOR_H_DEFINED
