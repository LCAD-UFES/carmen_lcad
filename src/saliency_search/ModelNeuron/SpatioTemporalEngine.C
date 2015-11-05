/*!@file ModelNeuron/SpatioTemporalEngine.C Implementation */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SpatioTemporalEngine.C $

#include "ModelNeuron/SpatioTemporalEngine.H"
#include "Image/LowPass.H"
// ######################################################################
// spatiotemporal engine
// ######################################################################
SpatioTemporalEnergyEngine::SpatioTemporalEnergyEngine()
    : itsScales(), itsWorker(16), maxScale(0), itsFilters(), itsIndex()
{
}

// ######################################################################
void SpatioTemporalEnergyEngine::start(std::vector<uint> const & scales)
{
  itsScales = scales;
  itsFilters.resize(scales.size());
  std::vector<SpatioTemporalEnergy>::iterator ff = itsFilters.begin();
  for (std::vector<uint>::iterator ii = itsScales.begin(); ii != itsScales.end(); ++ii)
  {
    if (*ii > maxScale)
      maxScale = *ii;

    ff->setupFilters();
    ++ff;
  }

  itsIndex.resize(maxScale+1);
  uint cc = 0;
  for (std::vector<uint>::iterator ii = itsScales.begin(); ii != itsScales.end(); ++ii)
    itsIndex[*ii] = cc++;
}

// ######################################################################
SpatioTemporalEnergyEngine::~SpatioTemporalEnergyEngine()
{ }

// ######################################################################
std::vector<SpatioTemporalEnergy> const & SpatioTemporalEnergyEngine::getFilters() const
{
  return itsFilters;
}

// ######################################################################
uint const SpatioTemporalEnergyEngine::scaleToIndex(uint const & scale)
{
  ASSERT(scale < itsIndex.size());
  return itsIndex[scale];
}

// ######################################################################
void SpatioTemporalEnergyEngine::update(Image<float> const & input)
{
  //compute scale space
  ImageSet<float> pyr;
  Image<float> inp = input;
  pyr.push_back(inp);

  for (uint ii = 1; ii <= maxScale; ++ii)
  {
    inp = lowPass5yDecY(lowPass5xDecX(inp));
    pyr.push_back(inp);    
  }

  for (uint ii = 0; ii < itsScales.size(); ++ii)
    itsFilters[ii].getNextOutput(pyr[itsScales[ii]], itsWorker);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
