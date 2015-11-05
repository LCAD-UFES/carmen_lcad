/*!@file Transport/ParseArg.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/ParseArg.H $
// $Id: ParseArg.H 7877 2007-02-08 21:57:00Z rjpeters $
//

#ifndef TRANSPORT_PARSEARG_H_DEFINED
#define TRANSPORT_PARSEARG_H_DEFINED

#include "Util/StringUtil.H"
#include "Util/log.H"

#include <string>

/// Parse a command-line arg and return a product from the appropriate factory
/** This is where all of the parsing of --in, --out, and --io options
    happens. In principle, it is a simple procedure, looking for
    formats of the form type:spec, except it's not simple because we
    also implement a number of special cases to allow for less verbose
    command-line options. The algorithm is like this:

    (1) if we find a colon, then we split 'type:spec' on the colon and
    we're done.

    (2) With no colon, we try to see if we can infer that the user
    has passed either a bare 'type' or a bare 'spec', as
    follows:

    (3) Check if it's a bare 'spec' -- see if there is a filename
    extension that we recognize and for which we can guess an
    appropriate FrameIstream type; if so, we're done.

    (4) Check if it's a bare 'type', one of the particular type
    names that we know; if so, we're done.

    (5) Give up with an LFATAL() -- in particular, we DON'T try to
    do some default option, which could cause a future backward
    compatibility headache.

    Note that the sets of known types and filename extensions are set
    up in makeFrameIstream() and makeFrameOstream() below; if you
    want to add a new FrameIstream or FrameOstream type, you should
    add it to one of those functions.
*/
template <class Factory>
typename Factory::ProductType
parseStreamArg(const std::string& arg,
               std::string& extrainfo,
               const Factory* typeFactory,
               const Factory* extFactory)
{
  using std::string;

  const string::size_type colon = arg.find(':');

  // (1) try to split on the colon
  if (colon != string::npos)
    {
      // ok, we have a colon, so just split on that
      const string type = arg.substr(0, colon);
      extrainfo = arg.substr(colon+1, string::npos);
      return typeFactory->createObj(type.c_str());
    }

  // (2) ok, no colon, so let's search the filename for clues; first
  // look for a filename extension:
  const string::size_type dot = arg.rfind('.');

  if (dot != string::npos)
    {
      // (3) ok, found a filename extension, let's see if it's one
      // that we recognize and can associate with a particular
      // FrameIstream type
      const string ext = arg.substr(dot+1, string::npos);

      if (extFactory->is_valid_key(ext.c_str()))
        {
          extrainfo = arg;
          return extFactory->createObj(ext.c_str());
        }

      // (3a) ok, we didn't find a known matching filename extension,
      // but now let's try to find a two-part extension
      // (e.g. ".ext.gz" or ".ext.bz2") that matches
      if (dot > 0)
        {
          const string::size_type dot2 = arg.rfind('.', dot-1);

          if (dot2 != string::npos)
            {
              const string ext2 = arg.substr(dot2+1, string::npos);

              if (extFactory->is_valid_key(ext2.c_str()))
                {
                  extrainfo = arg;
                  return extFactory->createObj(ext2.c_str());
                }
            }
        }

      // ok, here we didn't find any known filename extension, but
      // don't give up yet, we still have one more thing to try...
    }

  // (4) ok, no colon and no (special) filename extension, last
  // thing is to check if the single word is a keyword with no
  // extrainfo
  if (typeFactory->is_valid_key(arg.c_str()))
    {
      extrainfo = "";
      return typeFactory->createObj(arg.c_str());
    }

  // (5) give up
  LFATAL("couldn't parse <type:spec> from '%s':\n"
         "\t* no colon was found\n"
         "\t* no bare 'spec' was found with a known filename extension\n"
         "\t     (known extensions are\n"
         "%s)\n"
         "\t* no bare 'type' was found with a known type name\n"
         "\t     (known types are\n"
         "%s)\n",
         arg.c_str(),
         stdLineWrap(extFactory->get_known_keys(", ").c_str(), 60, "\t\t").c_str(),
         stdLineWrap(typeFactory->get_known_keys(", ").c_str(), 60, "\t\t").c_str());

  /* can't happen */ return *((typename Factory::ProductType*)0);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_PARSEARG_H_DEFINED
