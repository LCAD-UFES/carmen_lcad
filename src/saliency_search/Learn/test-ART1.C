/*!@file Learn/test-Art1.C test the ART1 network class
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/test-ART1.C $
// $Id: test-ART1.C 12963 2010-03-06 02:18:23Z irock $
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Learn/ART1.H"

#include <vector>
#include <string>

std::vector<std::string> getPatterns()
{
  std::vector<std::string> patt;
  patt.push_back("   O ");
  patt.push_back("  O O");
  patt.push_back("    O");
  patt.push_back("  O O");
  patt.push_back("    O");
  patt.push_back("  O O");
  patt.push_back("    O");
  patt.push_back(" OO O");
  patt.push_back(" OO  ");
  patt.push_back(" OO O");
  patt.push_back(" OO  ");
  patt.push_back("OOO  ");
  patt.push_back("OO   ");
  patt.push_back("O    ");
  patt.push_back("OO   ");
  patt.push_back("OOO  ");
  patt.push_back("OOOO ");
  patt.push_back("OOOOO");
  patt.push_back("O    ");
  patt.push_back(" O   ");
  patt.push_back("  O  ");
  patt.push_back("   O ");
  patt.push_back("    O");
  patt.push_back("  O O");
  patt.push_back(" OO O");
  patt.push_back(" OO  ");
  patt.push_back("OOO  ");
  patt.push_back("OO   ");
  patt.push_back("OOOO ");
  patt.push_back("OOOOO");

  return patt;
}

int main()
{


  ART1 net(5, 10); //Constract an art1 network with 5 input features and 10 hidden units


  std::vector<std::string> patterns = getPatterns();

  for(uint i=0; i<patterns.size(); i++)
  {
    int cls = net.evolveNet(patterns[i]);
    LINFO("%s -> Class %i", patterns[i].c_str(), cls);
  }


}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
