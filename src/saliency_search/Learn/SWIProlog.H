/*!@file Learn/SWIProlog.H interface to SWI-Prolog */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SWIProlog.H $
// $Id: SWIProlog.H 9308 2008-02-22 19:04:41Z rjpeters $
//

#ifndef LEARN_SWIPROLOG_H_DEFINED
#define LEARN_SWIPROLOG_H_DEFINED

#include "Util/Types.H" // for uint
#include <vector>
#include <string>

#ifdef HAVE_SWI_PROLOG_H
#include <SWI-Prolog.h>
#endif

class SWIProlog
{
public:

  //! init the prolog engine
  SWIProlog(int argc, char **argv);

  //! Destructor
  ~SWIProlog();

  //! Consult a KB file (assume a .pl extention)
  bool consult(const char *filename);

  //! Query the predicate given args
  //! Note args might change to a value inferd by prolog,
  //! For variables, insert an empty string
  //! Return the first match and then cuts the tree
  bool query(const char *predicate, std::vector<std::string> &args);

private:


};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_SWIPROLOG_H_DEFINED
