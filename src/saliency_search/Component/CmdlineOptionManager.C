/*!@file Component/CmdlineOptionManager.C OptionManager implementation for command-line parsing */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/CmdlineOptionManager.C $
// $Id: CmdlineOptionManager.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef COMPONENT_CMDLINEOPTIONMANAGER_C_DEFINED
#define COMPONENT_CMDLINEOPTIONMANAGER_C_DEFINED

#include "Component/CmdlineOptionManager.H"

#include "Component/GlobalOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/ModelParamBase.H"
#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H" // for levenshteinDistance()
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/demangle.h"
#include "rutz/error_context.h"
#include "rutz/pipe.h"
#include "rutz/sfmt.h"

#include <iostream>
#include <list>
#include <map>
#include <algorithm>
#include <string>
#include <sys/ioctl.h>     // for TIOCGWINSZ
#include <unistd.h>        // for isatty()
#include <vector>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace dummy_namespace_to_avoid_gcc411_bug_CmdlineOptionManager_C
{
  // ##############################################################
  // types and typedefs

  struct OptionInfo;

  typedef std::list<OptionedModelParam*> ParamList;
  typedef std::map<const ModelOptionDef*, OptionInfo> OptMapType;

  // ##############################################################
  //! OptionInfo holds mutable information related to a ModelOptionDef
  /*! We associate one OptionInfo with each ModelOptionDef that
      ModelManager hears about. Each OptionInfo keeps track of the
      current value of that option in the current program run, as well
      as the list of params who are listening for that option's
      value. */
  struct OptionInfo
  {
    OptionInfo(const ModelOptionDef* d)
      :
      sortid(nextid++),
      val(d->defval),
      params()
    {}

    static int nextid; //!< next available value for sortid

    int        sortid; //!< used to sort OptionInfo's in creation order
    string     val;    //!< current value in string representation
    ParamList  params; //!< the components that requested this option
  };

  int OptionInfo::nextid = 0;

  // ##############################################################
  string formatHelpString(const string& h,
                          const string::size_type spc1,
                          const string::size_type spc2,
                          const string::size_type ncols)
  {
    // let's start with a bunch of spaces:
    string ret = string(spc1, ' ');
    string::size_type pos = 0;
    string::size_type spc = spc1;

    // get a chunk of the incoming string:
    while(pos < h.length())
      {
        // if the rest of h fits in our available space, AND doesn't
        // have any hard newlines, then just add it and we're done:
        if (h.length() - pos + spc < ncols
            && h.find_first_of("\n", pos) == h.npos)
          { ret += h.substr(pos); break; }

        // here is the max we could fit in the current line of output:
        string::size_type pos2 = pos + ncols - spc - 1;

        // do we have an LF?
        string::size_type poslf = h.find('\n', pos);
        if (poslf != h.npos && poslf < pos2) pos2 = poslf;

        string::size_type pos3 = h.find_last_of(" \t\n", pos2);
        if (pos3 != h.npos && pos3 >= pos)
          {
            if (h[pos3] != '\n')
              ret += (h.substr(pos, pos3 - pos + 1)
                      + "\n"
                      + string(spc2, ' '));
            else
              ret += (h.substr(pos, pos3 - pos + 1)
                      + string(spc2, ' '));

            pos = pos3 + 1; spc = spc2;
            if (h[pos] == '\n') pos++; // skip any LF
          }
        else
          {
            // cannot cut -- abort this mess
            ret += h.substr(pos); break;
          }
      }
    return ret;
  }

  // ##############################################################
  // make a human-readable string out of a short-option name (e.g.,
  // replacing the null value '\0' with the two literal characters
  // {backslash, '0'}
  std::string makeReadableShortopt(const char c)
  {
    if (c == 0)
      return "\\0";

    else if (!isprint(c))
      return sformat("0x%02x", int(c));

    // else ...
    return sformat("%c", c);
  }

  // ##############################################################
  void assertNoCollision(const ModelOptionDef* opt1,
                         const ModelOptionDef* opt2)
  {
    // if they're the exact same option object, then it's not
    // considered a collision:
    if (opt1 == opt2) return;

    // otherwise, make sure that neither the param name, the long
    // options name, nor the short option name are the same

    if (strcmp(opt1->name, opt2->name) == 0)
      LFATAL("ModelOptionDef collision:\n"
             "ERROR: two options have the same name \"%s\"\n"
             "\twith &opt1 = %p\n"
             "\t and &opt2 = %p\n"
             "\t and opt1->longoptname = \"%s\"\n"
             "\t and opt2->longoptname = \"%s\"\n"
             "\t and opt1->shortoptname = '%s'\n"
             "\t and opt2->shortoptname = '%s'\n",
             opt1->name,
             opt1, opt2,
             opt1->longoptname, opt2->longoptname,
             makeReadableShortopt(opt1->shortoptname).c_str(),
             makeReadableShortopt(opt2->shortoptname).c_str());

    if (opt1->longoptname != 0
        && opt2->longoptname != 0
        && opt1->longoptname[0] != '\0'
        && strcmp(opt1->longoptname, opt2->longoptname) == 0)
      LFATAL("ModelOptionDef collision:\n"
             "ERROR: two options have the same longoptname \"%s\"\n"
             "\twith &opt1 = %p\n"
             "\t and &opt2 = %p\n"
             "\t and opt1->name = \"%s\"\n"
             "\t and opt2->name = \"%s\"\n"
             "\t and opt1->shortoptname = '%s'\n"
             "\t and opt2->shortoptname = '%s'\n",
             opt1->longoptname,
             opt1, opt2,
             opt1->name, opt2->name,
             makeReadableShortopt(opt1->shortoptname).c_str(),
             makeReadableShortopt(opt2->shortoptname).c_str());

    if (opt1->shortoptname != '\0'
        && opt1->shortoptname == opt2->shortoptname)
      LFATAL("ModelOptionDef collision:\n"
             "ERROR: two options have the same shortoptname '%s'\n"
             "\twith &opt1 = %p\n"
             "\t and &opt2 = %p\n"
             "\t and opt1->name = \"%s\"\n"
             "\t and opt2->name = \"%s\"\n"
             "\t and opt1->longoptname = \"%s\"\n"
             "\t and opt2->longoptname = \"%s\"\n",
             makeReadableShortopt(opt1->shortoptname).c_str(),
             opt1, opt2,
             opt1->name, opt2->name,
             opt1->longoptname, opt2->longoptname);
  }

  // ##############################################################
  void assertNoOptionCollisions(const OptMapType& opts,
                                const ModelOptionDef* newopt)
  {
    for (OptMapType::const_iterator
           mitr = opts.begin(),
           mstop = opts.end();
         mitr != mstop;
         ++mitr)
      assertNoCollision((*mitr).first, newopt);
  }

  // ##############################################################
  void showErrMsg(const OptMapType& opts,
                  const std::string& context,
                  const std::string& errormsg,
                  const std::string& err2 = std::string(),
                  const bool spellcheck = false)
  {
    // if we have an error message, let's start with that:
    if (errormsg.length() > 0 && err2.length() > 0)
      {
        cerr << endl << "ERROR: " << context << errormsg << err2;

        // is it a "--something" option that we want to spell-check?
        if (spellcheck &&
            err2.length() > 2 && err2[0] == '-' && err2[1] == '-')
          {
            const std::string option = err2.substr(2);

            const unsigned int levthresh =
              std::min(6u, (unsigned int)(option.length() / 2));

            typedef std::multimap<unsigned int, std::string>
              match_map_type;

            match_map_type matches;

            for (OptMapType::const_iterator
                   itr = opts.begin(), stop = opts.end();
                 itr != stop; ++itr)
              {
                const ModelOptionDef* odef = (*itr).first;

                const unsigned int dist1 =
                  damerauLevenshteinDistance(option, odef->longoptname);

                if (dist1 <= levthresh)
                  {
                    matches.insert(match_map_type::value_type
                                   (dist1, odef->longoptname));
                  }
                else if (odef->type.kind == MOK_FLAG)
                  {
                    const std::string othername =
                      std::string("no") + odef->longoptname;

                    const unsigned int dist2 =
                      damerauLevenshteinDistance(option, othername);

                    if (dist2 <= levthresh)
                      {
                        matches.insert(match_map_type::value_type
                                       (dist2, othername));
                      }
                  }
              }

            if (matches.size() > 0)
              {
                cerr << " (did you mean";
                for (match_map_type::const_iterator
                       itr = matches.begin(), stop = matches.end();
                     itr != stop; ++itr)
                  cerr << " --" << (*itr).second << '?';
                cerr << ')';
              }
          }
      }
    else if (errormsg.length() > 0)
      cerr << endl << "ERROR: " << errormsg;

    cerr << endl << endl;

    cerr << "Try re-running with --help to see a "
         << "full list of available options"
         << endl;
  }

  // ##############################################################
  typedef std::map< const ModelOptionCateg*, std::map<int, string> > CategMap;

  typedef std::pair<const ModelOptionCateg*, std::map<int, string> > CategoryInfo;

  bool cmpcateg(const CategoryInfo& v1,
                const CategoryInfo& v2)
  {
    return v1.first->sortpriority < v2.first->sortpriority;
  }

  // ##############################################################
  void printFullHelp(const OptMapType& opts, std::ostream& out)
  {
    // figure out number of columns in our terminal:
    unsigned int ncols = 80;  // default width
#ifdef TIOCGWINSZ
    if (isatty(STDIN_FILENO)) {
      struct winsize ws;
      if (!ioctl(STDIN_FILENO, TIOCGWINSZ, &ws) && ws.ws_col > 0)
        ncols = ws.ws_col;
    }
#endif

    // we'll pile up the relevant help messages for each category; so
    // here we need a vector (one entry per category) of maps (one entry
    // per help message) of strings (each help message) -- note that we
    // use a std::map keyed on each OptionInfo's sortid, which in turn
    // is determined by the order of construction of the OptionInfo's;
    // the bottom line is that OptionInfo's that were registered first
    // will be sorted first, and will thus show up first in the help
    // listing
    CategMap bycateg;

    // Go over all possible options and add some help to the correct
    // category if we have at least one ModelComponent that has
    // requested the option:
    OptMapType::const_iterator mitr = opts.begin();
    for ( ; mitr != opts.end(); ++mitr) {
      const ModelOptionCateg* categ = (*mitr).first->categ;
      const ModelOptionDef* mo = (*mitr).first;
      string help = "";

      // does this option have any params?
      if ((*mitr).second.params.empty() == false
          || mo->type.kind == MOK_ALIAS)
        {
          switch (mo->type.kind)
            {
            case MOK_OBSOLETE:
              {
                if (mo->shortoptname != '\0')
                  help += string("-") + mo->shortoptname+string(", ");
                help += mo->longoptname + string(" [OBSOLETE]");
                help += string("\n") + mo->descr;
                if (MYLOGVERB >= LOG_DEBUG)
                  help += string(" [") + mo->name + string("]");
              }
              break;
            case MOK_ALIAS:
              if (mo->shortoptname != '\0')
                help += string("-") + mo->shortoptname + string(", ");
              help += string("--") + mo->longoptname +
                string("\n") + mo->descr +
                string(". EQUIVALENT TO: ") + mo->defval;
                if (MYLOGVERB >= LOG_DEBUG)
                  help += string(" [") + mo->name + string("]");
              break;
            case MOK_FLAG:
              {
                if (mo->shortoptname != '\0')
                  help += string("-") + mo->shortoptname+string(", ");
                help += string("--[no]") + mo->longoptname;
                bool on; convertFromString((*mitr).second.val, on);
                if (on) help += " [yes]"; else help += " [no]";
                help += string("\n") + mo->descr;
                if (MYLOGVERB >= LOG_DEBUG)
                  help += string(" [") + mo->name + string("]");
              }
              break;
            case MOK_ARG:
              {
                if (mo->shortoptname != '\0')
                  help += string("-") + mo->shortoptname+string(", ");
                ASSERT(mo->type.argtype != 0);
                help += string("--") + mo->longoptname + string("=")
                  + mo->validvals + string(" [") + (*mitr).second.val
                  + string("]  (")
                  + string(rutz::demangled_name(*(mo->type.argtype)))
                  + string(")\n") + mo->descr;
                if (MYLOGVERB >= LOG_DEBUG)
                  help += string(" [") + mo->name + string("]");
              }
              break;
            }
          bycateg[categ][(*mitr).second.sortid] = help;
        }
    }

    // now loop over all categories and print their options if any:
    out << "\nCOMMAND-LINE OPTION SYNTAX:\n";
    vector<CategoryInfo>
      sortedcategs(bycateg.begin(), bycateg.end());

    std::sort(sortedcategs.begin(), sortedcategs.end(),
              cmpcateg);

    for (unsigned int c = 0; c < sortedcategs.size(); c ++)
      if (sortedcategs[c].second.empty() == false)
        {
          // let's print out the category heading:
          out << '\n' << sortedcategs[c].first->description << ":\n\n";

          // now loop over all options in that category (see note above
          // explaining how the std::map will contain the options listed
          // in their order of registration):
          for (std::map<int, string>::iterator
                 itr = sortedcategs[c].second.begin(),
                 stop = sortedcategs[c].second.end();
               itr != stop; ++itr) {
            // let's format the help message to terminal width:
            string h = (*itr).second;
            // we have 2 lines, separated by a \n; let's split them:
            unsigned int pos = h.find('\n');
            string h1 = h.substr(0, pos), h2 = h.substr(pos + 1);
            // let's print this out:
            out << formatHelpString(h1, 2, 4, ncols) << '\n';
            out << formatHelpString(h2, 6, 6, ncols) << "\n\n";
          }
        }

    out << std::flush;
  }

  // ##############################################################
  void printFullHelp(const OptMapType& opts)
  {
    if (isatty(STDOUT_FILENO))
      {
        // Try to find a pager program (such as /usr/bin/less or
        // /usr/bin/more)

        // (1) by default, start with the user's PAGER environment
        // variable, if it exists:
        const char* pager = getenv("PAGER");

        // (2) next, see if we picked up a PAGER_PROG from the
        // configure script:
#ifdef PAGER_PROG
        if (pager == 0 || access(pager, X_OK) != 0)
          {
            pager = PAGER_PROG;
          }
#endif

        // (3) finally, just try /usr/bin/less
        if (pager == 0 || access(pager, X_OK) != 0)
          pager = "/usr/bin/less";

        // now check if the pager program actually exists and is
        // executable
        if (pager != 0)
          {
            if (access(pager, X_OK) != 0)
              // ok, we wouldn't be able to execute it, so forget
              // about it
              pager = 0;
          }

        if (pager != 0)
          {
            rutz::exec_pipe p("w", pager, (const char*) 0);

            printFullHelp(opts, p.stream());

            p.close();

            const int result = p.exit_status();

            if (result != 0)
              LERROR("error in child process (%s)", pager);
            else
              return;
          }

        // ok, the pager failed, so just dump the help to stdout:
        printFullHelp(opts, cout);
      }
    else
      {
        printFullHelp(opts, cout);
      }
  }

  // ##############################################################
  /* Find the OptionInfo for a given ModelOptionDef*. This finds our
     OptionInfo object associated with the given ModelOptionDef*; if
     there is no such OptionInfo yet, then we create it on the fly
     here. */
  OptionInfo& findOptionInfo(OptMapType& opts,
                             const ModelOptionDef* def)
  {
    OptMapType::iterator mitr = opts.find(def);
    if (mitr == opts.end())
      {
        // Make sure the new option doesn't collide with any existing
        // options
        assertNoOptionCollisions(opts, def);

        std::pair<OptMapType::iterator, bool> result =
          opts.insert(OptMapType::value_type
                      (def, OptionInfo(def)));
        return (*result.first).second;
      }
    else
      return (*mitr).second;
  }
}

using namespace dummy_namespace_to_avoid_gcc411_bug_CmdlineOptionManager_C;

// ##############################################################
struct CmdlineOptionManager::Impl
{
  Impl()
    :
    opts(),
    exportMask(OPTEXP_ALL),
    extraArgs()
  {}

  OptMapType opts;          //!< link OptionInfo's to ModelOptionDef's
  int exportMask;           //!< bit mask for allowed ModelOptionDef::exportFlag
  vector<string> extraArgs; //!< non-option command-line args
  vector<string> allArgs;   //!< all commad-line args including options and non-options
};

// ##############################################################
CmdlineOptionManager::CmdlineOptionManager()
  :
  rep(new Impl)
{}

// ##############################################################
CmdlineOptionManager::~CmdlineOptionManager()
{
  delete rep;

  // ugly const_cast to set rep=0 here, but the ugliness is justified
  // the fact that it allows us to ASSERT(rep!=0) in our other
  // functions to make sure that people don't try to use us after we
  // have already been destroyed
  *(const_cast<Impl**>(&rep)) = 0;
}

// ##############################################################
void CmdlineOptionManager::allowOptions(const int mask)
{
  ASSERT(rep != 0);
  rep->exportMask = mask;
}

// ##############################################################
void CmdlineOptionManager::requestOption(OptionedModelParam& p,
                                         const bool useMyVal)
{
  ASSERT(rep != 0);

  // if the option doesn't match our exportMask, then do nothing
  if ((p.getOptionDef()->exportFlag & rep->exportMask) == false)
    {
      LDEBUG("option %s (--%s) IGNORED (does not match export mask)",
             p.getOptionDef()->name, p.getOptionDef()->longoptname);
      return;
    }

  LDEBUG("requesting option %s (--%s)",
         p.getOptionDef()->name, p.getOptionDef()->longoptname);

  // find the option based on the name of the model param:
  OptionInfo& opt = findOptionInfo(rep->opts, p.getOptionDef());

  // ok, found it. See if we already have this component for this
  // option:
  bool gotit = false;
  ParamList::iterator itr = opt.params.begin();
  while(itr != opt.params.end()) {
    if (*itr == &p) { gotit = true; break; }
    itr ++;
  }

  // Add this component to our client list for that option:
  if (gotit == false) opt.params.push_back(&p);

  // Do we want to use its val as our new default? Otherwise, reset
  // its val to our default:
  if (useMyVal) setOptionValString(p.getOptionDef(), p.getValString());
  else
    {
      const bool ok = p.setValString(opt.val);
      if (!ok)
        LFATAL("rejected setting option %s=%s",
               p.getOptionDef()->name, opt.val.c_str());
    }
}

// ##############################################################
void CmdlineOptionManager::unRequestOption(OptionedModelParam& p)
{
  ASSERT(rep != 0);

  LDEBUG("unrequesting option %s (--%s)",
         p.getOptionDef()->name, p.getOptionDef()->longoptname);

  // find the option based on the name of the model param:
  OptionInfo& opt = findOptionInfo(rep->opts, p.getOptionDef());

  // ok, found it. See if we already have this component for this
  // option, and if so, let's remove it from the list of params for
  // the option:
  ParamList::iterator itr = opt.params.begin();
  while (itr != opt.params.end())
    if (&p == *itr) itr = opt.params.erase(itr); else itr ++;
}

// ##############################################################
void CmdlineOptionManager::requestOptionAlias(const ModelOptionDef* def)
{
  ASSERT(rep != 0);

  // if the option alias doesn't match our exportMask, then do nothing
  if ((def->exportFlag & rep->exportMask) == 0)
    {
      LDEBUG("option alias %s (--%s) IGNORED (does not match export mask)",
             def->name, def->longoptname);
      return;
    }

  LDEBUG("requesting option alias %s (--%s)",
         def->name, def->longoptname);

  // find the option based on the name (and force the OptionInfo to be
  // created if it doesn't yet exist):
  OptionInfo& opt = findOptionInfo(rep->opts, def);

  if (!opt.params.empty())
    LFATAL("model params can't be associated with option aliases");
}

// ##############################################################
void CmdlineOptionManager::setOptionValString(const ModelOptionDef* def,
                                              const string& val)
{
  ASSERT(rep != 0);

  // find the option by name:
  OptionInfo& opt = findOptionInfo(rep->opts, def);

  // keep the new value so that we can return it via
  // getOptionValString:
  opt.val = val;

  LDEBUG("setting option %s=%s", def->name, val.c_str());

  // tell the new value to all our params:
  for (ParamList::iterator itr = opt.params.begin();
       itr != opt.params.end(); ++itr)
    {
      ASSERT(*itr != 0);
      const bool ok = (*itr)->setValString(val);
      if (!ok)
        LFATAL("rejected setting option %s=%s",
               def->name, val.c_str());
    }
}

// ##############################################################
string CmdlineOptionManager::getOptionValString(const ModelOptionDef* def)
{
  ASSERT(rep != 0);

  // find the option by name:
  OptionInfo& opt = findOptionInfo(rep->opts, def);

  // return the value:
  return opt.val;
}

// ##############################################################
bool CmdlineOptionManager::isOptionRequested(const ModelOptionDef* def) const
{
  ASSERT(rep != 0);

  return (rep->opts.find(def) != rep->opts.end());
}

// ##############################################################
uint CmdlineOptionManager::numOptionDefs() const
{
  ASSERT(rep != 0);

  return rep->opts.size();
}

// ##############################################################
uint CmdlineOptionManager::getOptionDefs(const ModelOptionDef** arr,
                                         uint narr)
{
  ASSERT(rep != 0);

  uint nfilled = 0;

  OptMapType::const_iterator
    mitr = rep->opts.begin(),
    stop = rep->opts.end();

  for (uint i = 0; i < narr; ++i)
    {
      if (mitr == stop)
        break;

      arr[i] = (*mitr).first;

      ++mitr;
      ++nfilled;
    }

  return nfilled;
}

// ##############################################################
const ModelOptionDef* CmdlineOptionManager::findOptionDef(const char* name) const
{
  ASSERT(rep != 0);

  for (OptMapType::const_iterator mitr = rep->opts.begin();
       mitr != rep->opts.end();
       ++mitr)
    if (!strcmp((*mitr).first->name, name))
      return (*mitr).first;

  LFATAL("Unknown option named '%s'", name);

  /* can't happen */ return (ModelOptionDef*) 0;
}

// ##############################################################
bool CmdlineOptionManager::isOptionDefUsed(const ModelOptionDef* def) const
{
  ASSERT(rep != 0);

  OptMapType::const_iterator mitr = rep->opts.find(def);

  if (mitr == rep->opts.end())
    return false;

  // FIXME do we even want to treat MODOPT_ALIAS options as "used"?
  // The only place isOptionDefUsed() is called is in the qt
  // ModelManagerWizard, which maybe doesn't even want to deal with
  // aliases?
  return ((*mitr).second.params.empty() == false
          || (*mitr).first->type.kind == MOK_ALIAS);
}

// ##############################################################
bool CmdlineOptionManager::parseCommandLine(const int argc,
                                            const char** argv,
                                            const char* usage,
                                            const int minarg,
                                            const int maxarg)
{
  ASSERT(rep != 0);

  // do a few resets:
  rep->extraArgs.clear();

  // do the core of the parsing:
  bool ret = this->parseCommandLineCore(argc, argv);
  if (ret == false) return false;

  // let's finally check that the number of leftover args is within
  // what we want:
  if (int(rep->extraArgs.size()) < minarg ||
      (maxarg >= 0 && int(rep->extraArgs.size()) > maxarg))
    {
      if (maxarg == -1)
        LERROR("Incorrect number of (non-opt) arg: %" ZU " [%d..Inf]",
               rep->extraArgs.size(), minarg);
      else
        LERROR("Incorrect number of (non-opt) arg: %" ZU " [%d..%d]",
               rep->extraArgs.size(), minarg, maxarg);

      cerr << "USAGE: " << argv[0] << " " << usage << endl;
      cerr << "Try '" << argv[0]
           << " --help' for additional information." << endl;
      return false;
    }

  // all went well
  return true;
}

// ##############################################################
bool CmdlineOptionManager::parseCommandLineCore(const int argc,
                                                const char** argv,
                                                const std::string& context)
{
  ASSERT(rep != 0);

  GVX_ERR_CONTEXT("parsing the command-line arguments");

  rep->allArgs.clear();
  rep->allArgs.push_back(argv[0] ? argv[0] : "");

  // loop over all the options and trigger the appropriate parsing.
  // There are several special cases in here. First, the
  // ModelParamBase objects take no option arguments and are processed
  // as special cases.
  int nextarg = 1;  // position of next argument for short options

  // Let's loop over the args and parse each one:
  bool ignoreOptions = false;
  for (int i = 1; i < argc; i ++)
    {
      GVX_ERR_CONTEXT(rutz::sfmt("considering argv[%d]=\"%s\"",
                                 i, argv[i]));

      LDEBUG("considering argv[%d]=\"%s\"", i, argv[i]);

      rep->allArgs.push_back(argv[i]);

      // if it's not an option, let's add it as an extra arg:
      if (argv[i][0] != '-' || ignoreOptions) {
        LDEBUG("argv[%d]=\"%s\" is an extra arg", i, argv[i]);
        if (i < nextarg)
          continue; // already processed as a short opt arg
        else {
          rep->extraArgs.push_back(string(argv[i]));
          nextarg = i + 1;
          continue;
        }
      }

      // is it a single '-'?
      if (strlen(argv[i]) < 2 && !ignoreOptions)
        { showErrMsg(rep->opts, context, "Unknown option: -"); return false; }

      // is it a short option (or a collection of short options)?
      if (argv[i][1] != '-' && !ignoreOptions) {
        LDEBUG("argv[%d]=\"%s\" is a short option", i, argv[i]);

        // loop over all short options specified here:
        for (int j = 1; j < int(strlen(argv[i])); j ++) {
          // let's look it up:
          OptMapType::iterator mitr = rep->opts.begin();
          for ( ; mitr != rep->opts.end(); ++mitr)
            if ((*mitr).first->shortoptname == argv[i][j]) break;

          // if not found, let's give an error:
          if (mitr == rep->opts.end())
            { showErrMsg(rep->opts, context, "Unknown option: ", argv[i]); return false; }

          // if it's an obsolete option, let's print a message and
          // bail out:
          if ((*mitr).first->type.kind == MOK_OBSOLETE)
            {
              showErrMsg(rep->opts, context,
                         sformat("Obsolete option: -%c:\n\t%s",
                                 argv[i][j], (*mitr).first->descr));
              return false;
            }

          // if it's special option number 0, let's print the help:
          if ((*mitr).first == &OPT_ShowHelpMessage)
            { printFullHelp(rep->opts); return false; }

          // if it's a MODOPT_ALIAS, let's recursively parse its value
          // and we'll be done with that option:
          if ((*mitr).first->type.kind == MOK_ALIAS)
            {
              if (!parseCommandLineCore
                  ((*mitr).first->defval,
                   sformat("While parsing option alias %s: %s",
                           argv[i], context.c_str())))
                return false;
              continue;
            }

          // did anybody request this option?
          if ((*mitr).second.params.empty())
            { showErrMsg(rep->opts, context, "Unimplemented option: ",argv[i]); return false; }

          // if it's a MODOPT_FLAG, let's turn it on:
          if ((*mitr).first->type.kind == MOK_FLAG)
            setOptionValString((*mitr).first, "true");
          else {
            // otherwise, we need an argument for the option:
            while (nextarg < argc && argv[nextarg][0] == '-')
              nextarg ++;
            if (nextarg >= argc)
              {
                showErrMsg(rep->opts, context, "Missing argument for option: ", argv[i]);
                return false;
              }

            // ok, let's set the value:
            setOptionValString((*mitr).first, argv[nextarg]);

            // ready for the next short option in our current arg
            nextarg ++;
          }
        }
      } else {
        LDEBUG("argv[%d]=\"%s\" is a long option", i, argv[i]);
        // it's a long option

        // is it just '--'?
        if (strlen(argv[i]) < 3)
          { LDEBUG("Ignore options: --"); ignoreOptions=true; }

        if (ignoreOptions) continue;

        // do we have an '=' symbol in the string?
        const char* eq = strchr(argv[i], '=');
        if (eq == NULL) {
          // is it a MODOPT_FLAG or MODOPT_ALIAS? let's look it up:
          OptMapType::iterator mitr = rep->opts.begin();
          const char* cval = "true";
          for ( ; mitr != rep->opts.end(); ++mitr)
            if (strcmp((*mitr).first->longoptname,
                       &(argv[i][2]))==0) break;

          // if not found, since there is no '=', it could still be
          // --noXXX
          if (mitr == rep->opts.end()) {
            // check that we have 'no' as prefix, plus something else:
            if (argv[i][2] != 'n'
                || argv[i][3] != 'o'
                || strlen(argv[i]) == 4)
              { showErrMsg(rep->opts, context, "Unknown option: ", argv[i], true); return false; }

            // let's look up that --noXXX:
            for (mitr = rep->opts.begin();
                 mitr != rep->opts.end();
                 ++mitr)
              if (strcmp((*mitr).first->longoptname,
                         &(argv[i][4])) == 0) break;

            // if not found, let's give up:
            if (mitr == rep->opts.end())
              { showErrMsg(rep->opts, context, "Unknown option: ", argv[i], true); return false; }

            // we'll continue, knowing that we want to set the value
            // to false:
            cval = "false";
          }

          // if it's an obsolete option, let's print a message and
          // bail out:
          if ((*mitr).first->type.kind == MOK_OBSOLETE)
            {
              showErrMsg(rep->opts, context,
                         sformat("Obsolete option: %s:\n\t%s",
                                 argv[i], (*mitr).first->descr));
              return false;
            }

          // if it's special option number 0, let's print the help:
          if ((*mitr).first == &OPT_ShowHelpMessage)
            { printFullHelp(rep->opts); return false; }

          // if it's a MODOPT_ALIAS, let's recursively parse its value
          // and we'll be done for that option:
          if ((*mitr).first->type.kind == MOK_ALIAS)
            {
              if (!parseCommandLineCore
                  ((*mitr).first->defval,
                   sformat("While parsing option alias %s: %s",
                           argv[i], context.c_str())))
                return false;
              continue;
            }

          // did anybody request this option?
          if ((*mitr).second.params.empty())
            { showErrMsg(rep->opts, context, "Unimplemented option: ", argv[i]); return false; }

          // if it's a MODOPT_FLAG, let's turn it on/off, otherwise
          // trouble:
          if ((*mitr).first->type.kind == MOK_FLAG)
            setOptionValString((*mitr).first, cval);
          else
            {
              showErrMsg(rep->opts, context, "Missing required argument for option: ",
                         argv[i]);
              return false;
            }
        } else {
          // ok, we have an '='; let's isolate the option name:
          char oname[eq - argv[i] - 1];
          strncpy(oname, &(argv[i][2]), eq - argv[i] - 2);
          oname[eq - argv[i] - 2] = '\0';

          LDEBUG("argv[%d]=\"%s\" has long option name \"%s\"",
                 i, argv[i], oname);

          // ok. let's look it up:
          OptMapType::iterator mitr = rep->opts.begin();
          for ( ; mitr != rep->opts.end(); ++mitr)
            if (strcmp((*mitr).first->longoptname, oname) == 0)
              break;

          // if not found, it's a bogus option name:
          if (mitr == rep->opts.end())
            { showErrMsg(rep->opts, context, "Unknown option: ", std::string("--") + oname, true); return false; }

          // if it's an obsolete option, let's print a message and
          // bail out:
          if ((*mitr).first->type.kind == MOK_OBSOLETE)
            {
              showErrMsg(rep->opts, context,
                         sformat("Obsolete option: --%s:\n\t%s",
                                 oname, (*mitr).first->descr));
              return false;
            }

          // if it's a MODOPT_ALIAS, user is goofing off:
          if ((*mitr).first->type.kind == MOK_ALIAS)
            {
              showErrMsg(rep->opts, context, "No argument permitted "
                         "for option alias --", oname);
              return false;
            }

          // did anybody request this option?
          if ((*mitr).second.params.empty())
            {
              showErrMsg(rep->opts, context, "Unimplemented option: --", oname);
              return false;
            }

          // is there anything after the '='?
          if (eq - argv[i] == int(strlen(argv[i])) - 1)
            {
              showErrMsg(rep->opts, context, "Missing value for option: --", oname);
              return false;
            }

          LDEBUG("argv[%d]=\"%s\" has long option value \"%s\"",
                 i, argv[i], &(eq[1]));

          // all right, let's set the value:
          setOptionValString((*mitr).first, &(eq[1]));
        }
      }
    }

  // all went well:
  return true;
}

// ##############################################################
bool CmdlineOptionManager::parseCommandLineCore(const char* args,
                                                const std::string& context)
{
  // let's just split our single string into an argv-like array and
  // call the other version of parseCommandLineCore:
  char* a = new char[strlen(args) + 1];
  strcpy(a, args);

  const int maxnum = 100; // max number of options in the alias
  typedef const char* const_char_ptr;
  const char** argv = new const_char_ptr[maxnum];
  argv[0] = NULL;  // this usually is the exec name and will be ignored

  int argc = 1;               // current argv number
  char* ptr = a;              // pointer to next option
  char* end = a + strlen(a); // pointer to end

  while(ptr < end)
    {
      argv[argc++] = ptr;  // store address of option
      if (argc >= maxnum)
        LFATAL("Wooooo, your alias is too long! ('%s')",
               args);
      while (!isspace(*ptr) && ptr != end) // find next white space
        ++ptr;
      *ptr++ = '\0';  // blank it out and get ready for next one
    }

  // ok, let's parse them all!
  bool ret = parseCommandLineCore(argc, argv, context);

  // free our local memory and return:
  delete [] argv; delete [] a;
  return ret;
}

// ##############################################################
uint CmdlineOptionManager::numArgs() const
{
  ASSERT(rep != 0);
  return rep->allArgs.size();
}

// ##############################################################
string CmdlineOptionManager::getArg(const uint num) const
{
  ASSERT(rep != 0);
  if (num >= rep->allArgs.size()) {
    LERROR("Invalid arg number %d (0..%" ZU ") -- IGNORED",
           num, rep->allArgs.size());
    return string("");
  }
  return rep->allArgs[num];
}

// ##############################################################
uint CmdlineOptionManager::numExtraArgs() const
{
  ASSERT(rep != 0);
  return rep->extraArgs.size();
}

// ##############################################################
string CmdlineOptionManager::getExtraArg(const uint num) const
{
  ASSERT(rep != 0);
  if (num >= rep->extraArgs.size()) {
    LERROR("Invalid extra arg number %d (0..%" ZU ") -- IGNORED",
           num, rep->extraArgs.size());
    return string("");
  }
  return rep->extraArgs[num];
}

// ##############################################################
void CmdlineOptionManager::clearExtraArgs()
{
  ASSERT(rep != 0);
  rep->extraArgs.clear();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_CMDLINEOPTIONMANAGER_C_DEFINED
