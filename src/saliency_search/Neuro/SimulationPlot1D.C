/*!@file Neuro/SimulationPlot1D.C 1d data plot used by simulation viewer */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: David J Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationPlot1D.C $

#include "Neuro/SimulationPlot1D.H"
#include "Image/DrawOps.H"  //for LinePlot()

// ######################################################################
SimulationPlot1D::SimulationPlot1D() 
  : itsBufLength(SimTime::ZERO()), itsMin(0.0F), itsMax(0.0F), 
    itsRate(SimTime::ZERO()), itsUseAutoScale(true),
    itsData(), itsTimes() 
{

}

// ######################################################################
SimulationPlot1D::SimulationPlot1D(const SimTime& buflength, const float minscale, const float maxscale,
                            const SimTime& sample_rate) 
    : itsBufLength(buflength), itsMin(minscale), itsMax(maxscale), 
      itsRate(sample_rate), itsUseAutoScale((minscale == 0.0F) && (maxscale == 0.0F)), 
      itsData(int(itsRate.hertz() * itsBufLength.secs()), 0.0F), itsTimes()
{
  if (itsRate != SimTime::ZERO())
    for (SimTime t = -1 * itsBufLength; t <= SimTime::ZERO(); t += itsRate)
    {
      itsTimes.push_back(t);
      itsData.push_back(0.0F);
    }
}

// ######################################################################
void SimulationPlot1D::push(const SimTime& time, const float& data)
{
  if (itsUseAutoScale)
  {
    if (data > itsMax)
      itsMax = data;
    else if (data < itsMin)
      itsMin = data;
  }
     
  itsTimes.push_back(time);
  itsData.push_back(data);
  SimTime d = itsTimes.back() - itsTimes.front();
  while (d > itsBufLength)
    {
      itsData.pop_front();
      itsTimes.pop_front();
      
      d = itsTimes.back() - itsTimes.front();
      
      if (d.nsecs() < 0)
        LFATAL("Cannot push past events into the buffer");
    }
}


// ######################################################################
void SimulationPlot1D::reset(const SimTime& buflength, const float minscale, const float maxscale,
                               const SimTime& sample_rate)
{
  itsBufLength = buflength;
  itsMin = minscale;
  itsMax = maxscale;
  itsRate = sample_rate;
  this->reset();
}

// ######################################################################
void SimulationPlot1D::reset()
{
  itsUseAutoScale = (itsMin == 0.0F) && (itsMax == 0.0F);
  itsData.clear();
  itsTimes.clear();
  if (itsRate != SimTime::ZERO())
    for (SimTime t = -1 * itsBufLength; t <= SimTime::ZERO(); t += itsRate)
      {
        itsTimes.push_back(t);
        itsData.push_back(0.0F);
      }
}

// ######################################################################
Image<PixRGB<byte> > SimulationPlot1D::draw(const uint w, const uint h, 
                                              const char* title, 
                                              const char* ylabel, 
                                              const char* xlabel, 
                                              const PixRGB<byte>& linecol, 
                                              const int numticks, 
                                              const bool reverse)
{
  return linePlot(itsData, w, h, itsMin, itsMax, 
                  title, ylabel, xlabel, linecol, 
                  PixRGB<byte>(255,255,255), numticks, reverse);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
