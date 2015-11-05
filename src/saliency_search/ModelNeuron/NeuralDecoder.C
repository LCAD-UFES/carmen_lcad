/*!@file ModelNeuron/NeuralDecoder.C Class declaration for a neural
   simulation module*/

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL:svn://ilab.usc.edu/trunk/saliency/src/ModelNeuron/NeuralDecoder.C$

#include "ModelNeuron/NeuralDecoder.H"

#include <cmath>

// ######################################################################
//!  implementations of NeuralDecoder
// #####################################################################
nsu::NeuralDecoder::NeuralDecoder(const nsu::NeuralDecoder& rhs) : 
  itsWindowSize(rhs.itsWindowSize), itsSamples(rhs.itsSamples)
{
}

// ######################################################################
nsu::NeuralDecoder& nsu::NeuralDecoder::operator=(const nsu::NeuralDecoder& rhs)
{
  if (this != &rhs)
    {
      itsWindowSize = rhs.itsWindowSize;
      itsSamples = rhs.itsSamples;
    }
  return *this;
}

// ######################################################################
//!  implementations of HoldDecoder
// ######################################################################
void nsu::HoldDecoder::push(const double& data) 
{
  if (itsSignal.size() < 1)
    itsLastSpike = data;
  else
    {
      itsSignal.push_back(data);
      itsLastSpike = itsSignal.front();
      itsSignal.pop_front();
    }
}

// ######################################################################
const double nsu::HoldDecoder::getOutput() const 
{
  return itsLastSpike;
}

// ######################################################################
void nsu::HoldDecoder::reset()
{
  itsLastSpike = 0.0;
  itsSignal.clear();
  itsSignal.resize(itsSamples, 0.0);
}

// ######################################################################
//!  implementations of HistDecoder
// ######################################################################
void nsu::HistDecoder::push(const double& data) 
{
  
  itsSpikeCount += data;
  itsSampleCount++;
  
  if (itsSampleCount == itsSamples)
    {
      itsSampleCount = 0;
      itsLastCount = itsSpikeCount;
      itsSpikeCount = 0.0;
    }
}

// ######################################################################
const double nsu::HistDecoder::getOutput() const 
{
  return itsLastCount / itsWindowSize.secs();
}

// ######################################################################
void nsu::HistDecoder::reset()
{
  itsSpikeCount = 0;
  itsLastCount = 0.0;
  itsSampleCount = 0;
}

// ######################################################################
//!  implementations of RectDecoder
// ######################################################################
void nsu::RectDecoder::push(const double& data) 
{
  itsSpikeRate += data - itsSignal.front();
  itsSignal.pop_front();
  itsSignal.push_back(data);
}

// ######################################################################
const double nsu::RectDecoder::getOutput() const 
{
  return itsSpikeRate / itsWindowSize.secs();
}

// ######################################################################
void nsu::RectDecoder::reset()
{
  itsSpikeRate = 0.0;
  itsSignal.clear();
  itsSignal.resize(itsSamples, 0.0);
}

// ######################################################################
//!  implementations of ExpDecoder
// ######################################################################
void nsu::ExpDecoder::push(const double& data) 
{
  itsSpikeRate *= itsAlpha;
  itsSpikeRate += (1.0-itsAlpha)*data;
}

// ######################################################################
const double nsu::ExpDecoder::getOutput() const 
{
  return itsSpikeRate;
}

// ######################################################################
void nsu::ExpDecoder::reset()
{
  itsSpikeRate = 0.0;
}

// ######################################################################
//! implementation of AlphaDecoder
// ######################################################################
nsu::AlphaDecoder::AlphaDecoder(const SimTime& timeStep,
                           const SimTime& windowSize) :
  NeuralDecoderDerived<AlphaDecoder>(timeStep,windowSize), 
  itsSignal(), itsKernel()
{
  const double a = 1.0 / windowSize.secs();  
  //find the optimal array size of the filter if we truncate at a
  //small value, C. No easy analytical solution exists to this problem,
  //so lets just use newtons method of roots to approximate
  double C = 0.0001;              
  //our initial estimate b. Since 1/a should define the peak of the
  //function in time, a good place to start is just a little beond that.
  double b = 1.0/a + 0.1 * (1.0 / a);
  for (int i = 0; i < 20; i++)
    {
      //our function
      double func = (a*a) * b * exp(-1 * a * b);
      //its second derivitive
      double func2 = ( (a*a) - (a * a * a) * b ) / exp(a * b);
      b = b - ( func - C ) / func2;
    }
  SimTime end = SimTime::SECS(b);
  SimTime ii =  SimTime::SECS(-1*b);  
  for (; ii  < end; ii += timeStep)
    {
      if (ii > SimTime::ZERO())
        {
          double  temp = (a * a) * ii.secs() * exp(-1.0 * a * ii.secs());
          itsKernel.push_back(temp);
        }
    } 
  itsSignal.resize(itsKernel.size(),0.0);
}

// ######################################################################
void nsu::AlphaDecoder::push(const double& data) 
 {
   itsSignal.push_back(data);
   itsSignal.pop_front();
 }

// ######################################################################
const double nsu::AlphaDecoder::getOutput() const 
{
  std::vector<double>::const_reverse_iterator iter = itsKernel.rbegin();
  std::deque<double>::const_iterator iters = itsSignal.begin();
  double val = 0.0;
  while (iters != itsSignal.end())
    val += (*iter++) * (*iters++);
  return val;
}

// ######################################################################
void nsu::AlphaDecoder::reset()
{
  itsSignal.clear();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
