/*!@file Psycho/PixPerDeg.H An eye movement trace */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/Psycho/PixPerDeg.H $
//

#ifndef PSYCHO_PIXPERDEG_H_DEFINED
#define PSYCHO_PIXPERDEG_H_DEFINED

//A class to hold pixel per degree information
class PixPerDeg 
{
public:
  PixPerDeg();
  PixPerDeg(const float& ppdx, const float& ppdy);
  ~PixPerDeg();

  //Get pixels-per-degree of the display
  float ppd() const;
  
  //Get x pixels-per-degree of the display
  float ppdx() const;

  //Get y pixels-per-degree of the display
  float ppdy() const;

  //! See if two PixPerDeg are the same.
  bool operator==(const PixPerDeg& x) const throw();
  
  //! See if two PixPerDeg are different.
  bool operator!=(const PixPerDeg& x) const throw();


private:
  float itsPPDx, itsPPDy;
};

//! PixPerDeg overload: format is "<float>,<float>"
std::string convertToString(const PixPerDeg& val);

//! PixPerDeg overload: format is "<float>,<float>"
void convertFromString(const std::string& str, PixPerDeg& val);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_PIXPERDEG_H_DEFINED
