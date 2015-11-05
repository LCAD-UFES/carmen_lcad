//!Structures and methods for free viewing data

//////////////////////////////////////////////////////////////////////////
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
//////////////////////////////////////////////////////////////////////////
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
//////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////
//
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/FreeViewingData.C $

#include "ModelNeuron/FreeViewingData.H"
#include <sstream>

// ######################################################################
std::string convertToString(const vectoruint& val)
{ 
  std::stringstream s; 
  for (vectoruint::const_iterator ii = val.begin(); ii != val.end(); ++ii)  
  {
    if (ii == val.end()-1)
      s<<*ii; 
    else
      s<<*ii<<','; 
    ++ii;
  }

  return s.str(); 
}

// ######################################################################
void convertFromString(const std::string& str, vectoruint& val)
{
  std::stringstream s; 
  s<<str; 
  while (!s.eof())
  {
    int t = -1;
    s>>t; 

    char c = ',';
    if (!s.eof())
      s>>c; 
    
    if (t == -1 || c != ',')
      conversion_error::raise<Dims>(str);
    
    val.push_back((uint)t);
  }
}

//#######################################################################
std::string convertToString(const ProbeType val)
{
  if (val == /*ProbeType::*/Probe)
    return "Probe";
  else if (val == /*ProbeType::*/Avg)
    return "Avg";
  else if (val == /*ProbeType::*/Max)
    return "Max";
  else
  {
    LFATAL("Cannot convert to string");
    return "";
  }
}

//#######################################################################
void convertFromString(const std::string& str, ProbeType& val)
{
  if (str.compare("Probe") == 0)
    val = /*ProbeType::*/Probe;
  
  else if (str.compare("Avg") == 0)
    val = /*ProbeType::*/Avg;
  
  else if (str.compare("Max") == 0)
    val = /*ProbeType::*/Max;
  
  else
  {
    conversion_error::raise<ProbeType>(str);
  }
}
