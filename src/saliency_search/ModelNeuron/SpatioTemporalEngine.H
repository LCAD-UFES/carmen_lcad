/*!@file ModelNeuron/SpatioTemporalEngine.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SpatioTemporalEngine.H $

#ifndef SPATIOTEMPORALENGINE_H_DEFINED
#define SPATIOTEMPORALENGINE_H_DEFINED

#include "ModelNeuron/SimulationWorkQueue.H"
#include "Image/ImageSet.H"
#include "Image/SteerableFilters.H"

// ###################################################################### 
// A SpatioTemporalEnergyEngine
//
// Multi-thread computation of basis sets at multiple scales
// ######################################################################
class SpatioTemporalEnergyEngine
{
  public:
    //! constructor
    SpatioTemporalEnergyEngine();

    //! destructor
    ~SpatioTemporalEnergyEngine();

    //!get things started with the specified scales
    void start(std::vector<uint> const & scales);

    //!return the filter sets, one for each scale
    std::vector<SpatioTemporalEnergy> const & getFilters() const;

    //! get the scale to array position index
    uint const scaleToIndex(uint const & scale);

    //! update the basis sets
    void update(Image<float> const & input);
    
  private:
    std::vector<uint> itsScales;
    nsu::SimulationWorkQueue itsWorker;
    uint maxScale;
    std::vector<SpatioTemporalEnergy> itsFilters; //one for each scale
    std::vector<uint> itsIndex;
};

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
