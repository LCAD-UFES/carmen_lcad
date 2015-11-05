/*!@file SpaceVariance/SVChanLevels.C space variant channel info
 */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/SVChanLevels.C $

#include "SpaceVariant/SVChanLevels.H"
#include "Util/StringConversions.H"
#include <sstream>
// ######################################################################
// converters for SVChanLevels
// #####################################################################
std::string convertToString(const SVChanLevels& val)
{ 
  std::stringstream s; 
  if (val.numLevels() > 0)
    {
      s<<val.getVariance(0);
      for (uint i = 1; i < val.numLevels(); ++i)
        s<<','<<val.getVariance(i);
    }
  return s.str(); 
}

// ######################################################################
void convertFromString(const std::string& str, SVChanLevels& val)
{
  std::vector<float> loc;
  std::stringstream s; 
  s<<str;
  while (!s.eof())
    {
      float t;
      char c;
      
      s>>t;      
      loc.push_back(t);
      
      if (t < 0.0F)
        conversion_error::raise<SVChanLevels>(str);
      
      if (s.eof())
        break;
      
      s>>c;
      if (c != ',')
        conversion_error::raise<SVChanLevels>(str);
    }
  val = SVChanLevels(loc);
}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
