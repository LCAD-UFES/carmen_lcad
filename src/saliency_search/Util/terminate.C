/*!@file Util/terminate.C fancy termination routine */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/terminate.C $
// $Id: terminate.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef UTIL_TERMINATE_C_DEFINED
#define UTIL_TERMINATE_C_DEFINED

#include "Util/terminate.H"

#include <exception>
#ifdef HAVE_EXECINFO_H
#  include <execinfo.h>
#endif
#include <pthread.h>

namespace
{
  typedef void (termination_func)();

  termination_func* orig_termination_func = 0;

  // A fancy termination function that first prints a backtrace, and
  // then calls the original std terminate function (a pointer to
  // which we have stored in orig_termination_func)
  void fancy_terminate()
  {
    /* Grab a backtrace and print it -- it will be in a fairly raw
       format, e.g. with lines like the following:

       /path/to/saliency/build/obj/nub/refdetail.so(_ZN3nub6detail22throw_soft_ref_invalidERKSt9type_infoRKN4rutz8file_posE+0xc3)[0xb79301f3]
       /path/to/saliency/build/obj/Neuro/ShapeEstimator.so(_ZN14ShapeEstimator7computeERK7Point2D+0x60b)[0xb7a556bb]
       /path/to/saliency/build/obj/Neuro/Brain.so(_ZN5Brain6evolveER7SimTimeR7Point2DRiRf+0x9ed)[0xb7afb0ad]
       /path/to/saliency/build/obj/Neuro/getSaliency.so(_ZN11GetSaliency7computeERK5ImageI6PixRGBIhEERK7SimTime+0x1ef)[0xb79e8cef]

       That is, each line gives the name of the object file, followed
       by the symbol name of the function that was executing in that
       file, plus a code offset relative to the start of the function,
       plus a global address:

       file.so(symbolname+offset)[address]

       You can get somewhat prettier looking output if you pipe the
       backtrace through the c++filt program, which will demangle the
       function names and give you something like this:

       /path/to/saliency/build/obj/nub/refdetail.so(nub::detail::throw_soft_ref_invalid(std::type_info const&, rutz::file_pos const&)+0xc3)[0xb79301f3]
       /path/to/saliency/build/obj/Neuro/ShapeEstimator.so(ShapeEstimator::compute(Point2D<int> const&)+0x60b)[0xb7a556bb]
       /path/to/saliency/build/obj/Neuro/Brain.so(Brain::evolve(SimTime&, Point2D<int>&, int&, float&)+0x9ed)[0xb7afb0ad]
       /path/to/saliency/build/obj/Neuro/getSaliency.so(GetSaliency::compute(Image<PixRGB<unsigned char> > const&, SimTime const&)+0x1ef)[0xb79e8cef]
    */
#ifdef HAVE_EXECINFO_H
    void* addresses[256];
    const int n = ::backtrace(&addresses[0], 256);
    ::backtrace_symbols_fd(&addresses[0], n, 2);
#endif

    (*orig_termination_func)();
  }

  pthread_once_t install_termination_func_once = PTHREAD_ONCE_INIT;

  void install_termination_func()
  {
    orig_termination_func = std::set_terminate(&fancy_terminate);
  }
}

void invt_install_fancy_terminate()
{
  pthread_once(&install_termination_func_once,
               &install_termination_func);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_TERMINATE_C_DEFINED
