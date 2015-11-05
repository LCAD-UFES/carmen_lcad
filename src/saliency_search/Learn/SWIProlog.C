/*!@file Learn/SWIProlog.C interface to SWI-Prolog */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SWIProlog.C $
// $Id: SWIProlog.C 9308 2008-02-22 19:04:41Z rjpeters $
//

#ifndef LEARN_SWIPROLOG_C_DEFINED
#define LEARN_SWIPROLOG_C_DEFINED

#include "Learn/SWIProlog.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>

// ######################################################################
SWIProlog::SWIProlog(int argc, char **argv)
{
  const char *av[10];
  int ac = 0;

 // av[ac++] = argv[0];
  av[ac++] = "DefaultProg";
  av[ac++] = "-q";
  av[ac++] = "-nosignals";
  av[ac]   = NULL;

#ifdef HAVE_SWI_PROLOG_H
  if (!PL_initialise(ac, av))
  {
    PL_halt(1);
    LFATAL("Failed to init prolog");
  }
#else
  LINFO("SWI prolog not found");
#endif

}

// ######################################################################
SWIProlog::~SWIProlog()
{

}

// ######################################################################
bool SWIProlog::consult(const char *filename)
{

  bool ret = false;
#ifdef HAVE_SWI_PROLOG_H
  term_t a0 = PL_new_term_refs(1);
  predicate_t p = NULL;

  p = PL_predicate("consult", 1, NULL);

  PL_put_atom_chars(a0, filename);

  qid_t query_id= PL_open_query(NULL, (PL_Q_NORMAL|PL_Q_CATCH_EXCEPTION), p, a0);

  ret =  PL_next_solution(query_id);
  PL_close_query(query_id);
#else
  LINFO("SWI prolog not found");
#endif

  return ret;
}

// ######################################################################
bool SWIProlog::query(const char *predicate, std::vector<std::string> &args)
{
  bool ret=false;
#ifdef HAVE_SWI_PROLOG_H
  term_t a0 = PL_new_term_refs(args.size());
  predicate_t p = NULL;

  p = PL_predicate(predicate, args.size(), NULL);

  for(uint i=0; i<args.size(); i++)
  {
    if (args[i].size() != 0)
      PL_put_atom_chars(a0+i, args[i].c_str());
  }

  qid_t query_id= PL_open_query(NULL, (PL_Q_NORMAL|PL_Q_CATCH_EXCEPTION), p, a0);

  ret =  PL_next_solution(query_id);

  if (ret)
  {
    //fill in the results in the place holdes
    for(uint i=0; i<args.size(); i++)
    {
      if (args[i].size() == 0)
      {
        char *data;
        PL_get_atom_chars(a0+i, &data);
        args[i] = std::string(data);
      }
    }
  }

  PL_close_query(query_id);
#else
  LINFO("SWI prolog not found");
#endif

  return ret;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_SWIPROLOG_C_DEFINED
