/*!@file Learn/test-SWIProlog.C test the SWI Prolog class
*/

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/test-SWIProlog.C $
// $Id: test-SWIProlog.C 9308 2008-02-22 19:04:41Z rjpeters $
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Learn/SWIProlog.H"

int main(int argc, char **argv)
{

    SWIProlog pl(argc, argv);

    const char *kbFilename = "src/Learn/testKB";

    if (!pl.consult(kbFilename))
    {
      LINFO("Can not consult the %s file", kbFilename);
    }

    //is mia a woman?
    {
    const char *predicate = "woman";
    std::vector<std::string> args;
    args.push_back(std::string("mia"));
    bool val = pl.query(predicate, args);
    LINFO("Is mia a woman? %i", val);
    }

    //is jody a woman?
    {
    const char *predicate = "woman";
    std::vector<std::string> args;
    args.push_back(std::string("jody"));
    bool val = pl.query(predicate, args);
    LINFO("Is jody a woman? %i", val);
    }

    //Who is a woman?
    {
    const char *predicate = "woman";
    std::vector<std::string> args;
    args.push_back(std::string());
    bool val = pl.query(predicate, args);
    LINFO("%s is a woman (%i)", args[0].c_str(), val);
    }

    //Who loves mia? loves(X,pumpkin).
    {
    const char *predicate = "loves";
    std::vector<std::string> args;
    args.push_back(std::string());
    args.push_back(std::string("pumpkin"));
    bool val = pl.query(predicate, args);
    LINFO("%s loves %s (%i)", args[0].c_str(), args[1].c_str(), val);
    }

    //Who loves who? loves(X,Y).
    {
    const char *predicate = "loves";
    std::vector<std::string> args;
    args.push_back(std::string());
    args.push_back(std::string());
    bool val = pl.query(predicate, args);
    LINFO("%s loves %s (%i)", args[0].c_str(), args[1].c_str(), val);
    }

    //is Marcellus jealous of sombody? jealous(marcellus,W).
    {
    const char *predicate = "jealous";
    std::vector<std::string> args;
    args.push_back(std::string("marcellus"));
    args.push_back(std::string());
    bool val = pl.query(predicate, args);
    LINFO("%s is jealous of %s (%i)", args[0].c_str(), args[1].c_str(), val);
    }


}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
