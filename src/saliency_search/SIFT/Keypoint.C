/*!@file SIFT/Keypoint.C Keypoint for SIFT obj recognition */

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
// Primary maintainer for this file: James Bonaiuto <bonaiuto@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/Keypoint.C $
// $Id: Keypoint.C 7506 2006-12-08 06:28:33Z siagian $
//

#include "SIFT/Keypoint.H"
#include "Util/log.H"

#include <istream>
#include <ostream>

// #######################################################################
// #######################################################################
// ########## Keypoint implementation
// #######################################################################
// #######################################################################

// #######################################################################
Keypoint::Keypoint() :
  itsX(0.0F), itsY(0.0F), itsS(0.0F), itsM(0.0), itsOriFV(),
  itsOriWeight(1.0), itsColWeight(0.0)
{ }

// #######################################################################
Keypoint::Keypoint(const std::vector<byte>& features, const float x,
                   const float y, const float s, const float o,
                   const float mag) :
  itsX(x), itsY(y), itsS(s), itsO(o), itsM(mag), itsOriFV(features),
  itsOriWeight(1.0), itsColWeight(0.0)
{
  // note: copy-constructor of std::vector copies the elements from
  // 'features' into 'itsOriFV', also resizing itsOriFV.
}

// #######################################################################
Keypoint::Keypoint(const std::vector<byte>& features,
                const std::vector<byte>& colFeatures,
                const float x, const float y, const float s, const float o,
                const float mag, float oriWeight, float colWeight) :
  itsX(x), itsY(y), itsS(s), itsO(o), itsM(mag),
  itsOriFV(features), itsColFV(colFeatures),
  itsOriWeight(oriWeight), itsColWeight(colWeight)
{
  // note: copy-constructor of std::vector copies the elements from
  // 'features' into 'itsOriFV' and Color features, also resizing itsOriFV.
}

// ######################################################################
Keypoint::Keypoint(const Keypoint& k) :
  itsX(k.itsX), itsY(k.itsY), itsS(k.itsS), itsO(k.itsO),
  itsM(k.itsM), itsOriFV(k.itsOriFV)
{ }

// ######################################################################
void Keypoint::reset(const std::vector<byte>& features, const float x,
                     const float y, const float s, const float o,
                     const float mag)
{
  itsOriFV = features;  // std::vector assignment copies all values
  itsX = x; itsY = y; itsS = s; itsO = o; itsM = mag;
}


// ######################################################################
Keypoint& Keypoint::operator=(const Keypoint& k)
{
  itsX = k.itsX; itsY = k.itsY; itsS = k.itsS; itsO = k.itsO;
  itsM = k.itsM; itsOriFV = k.itsOriFV; // std::vector assignment copies all values
  return *this;
}

// #######################################################################
Keypoint::~Keypoint()
{  }

// #######################################################################
std::ostream& operator<<(std::ostream& os, const Keypoint& k)
{
  const uint siz = k.itsOriFV.size();

  os<<k.itsX<<' '<<k.itsY<<' '<<k.itsS<<' '<<k.itsO<<' '
    <<k.itsM<<' '<<siz<<std::endl;

  for (uint i = 0; i < siz; i++) os<<int(k.itsOriFV[i])<<' ';
  os<<std::endl;

  return os;
}

// #######################################################################
std::istream& operator>>(std::istream& is, Keypoint& k)
{
  uint siz;
  is>>k.itsX>>k.itsY>>k.itsS>>k.itsO>>k.itsM>>siz;

  k.itsOriFV.clear(); k.itsOriFV.resize(siz);
  for (uint j = 0; j < siz; j++)
    {
      int val; is>>val;
      if (val < 0 || val > 255) LFATAL("Bogus file format!");
      k.itsOriFV[j] = byte(val);
    }
  return is;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
