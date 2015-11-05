/*!@file Learn/ART1.C Adaptive Resonance Theory */


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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/ART1.C $
// $Id: ART1.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Learn/ART1.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include <math.h>
#include <fcntl.h>
#include <limits>
#include <string>

ART1::ART1(const int inputSize, const int numClasses) :
  itsInputSize(inputSize),
  itsNumClasses(numClasses)
{

  //Build layer 1
  itsF1.units.resize(itsInputSize);
  for(uint i=0; i<itsF1.units.size(); i++)
    itsF1.units[i].weights.resize(itsNumClasses);

  //Build layer 2
  itsF2.units.resize(itsNumClasses);
  for(uint i=0; i<itsF2.units.size(); i++)
    itsF2.units[i].weights.resize(itsInputSize);

  //This should be application spacific
  itsA1  = 1;
  itsB1  = 1.5;
  itsC1  = 5;
  itsD1  = 0.9;
  itsL   = 3;
  itsRho = 0.9;

  //Initalize weights
  for(uint i=0; i<itsF1.units.size(); i++)
    for(uint j=0; j<itsF2.units.size(); j++)
    {
      itsF1.units[i].weights[j] = (itsB1 - 1) / itsD1 + 0.2;
      itsF2.units[j].weights[i] = itsL / (itsL - 1 + itsInputSize) - 0.1;
    }

}

ART1::~ART1()
{
}

void ART1::setInput(const std::vector<bool> input)
{
  double act;
  for(uint i=0; i<itsF1.units.size(); i++)
  {
    act = input[i] / (1 + itsA1 * (input[i] + itsB1) + itsC1);
    itsF1.units[i].output = (act > 0);
  }
}

int ART1::propagateToF2()
{

  double maxOut = -HUGE_VAL;
  int winner = -1;
  for (uint i=0; i<itsF2.units.size(); i++) {
    if (!itsF2.units[i].inhibited) {
      double sum = 0;
      for (uint j=0; j<itsF1.units.size(); j++) {
        sum += itsF2.units[i].weights[j] * itsF1.units[j].output;
      }
      if (sum > maxOut) {
        maxOut = sum;
        winner = i;
      }
    }
    itsF2.units[i].output = false;
  }
  if (winner != -1)
    itsF2.units[winner].output = true;

  return winner;
}

void ART1::propagateToF1(const std::vector<bool> input, const int winner)
{
  for (uint i=0; i<itsF1.units.size(); i++) {
    double sum = itsF1.units[i].weights[winner] *
                 itsF2.units[winner].output;
    double act = (input[i] + itsD1 * sum - itsB1) /
                 (1 + itsA1 * (input[i] + itsD1 * sum) + itsC1);
    itsF1.units[i].output = (act > 0);
  }
}


void ART1::adjustWeights(const int winner)
{

  for (uint i=0; i<itsF1.units.size(); i++) {
    if (itsF1.units[i].output) {
      double mag = 0;
      for (uint j=0; j<itsF1.units.size(); j++)
        mag += itsF1.units[j].output;

      itsF1.units[i].weights[winner] = 1;
      itsF2.units[winner].weights[i] = itsL / (itsL - 1 + mag);
    } else {
      itsF1.units[i].weights[winner] = 0;
      itsF2.units[winner].weights[i] = 0;
    }
  }
}




int ART1::evolveNet(std::string in)
{

  std::vector<bool> input(itsInputSize);

  for(uint i=0; i<in.size(); i++)
    input[i] = (in[i] == 'O');

  for(uint i=0; i<itsF2.units.size(); i++)
    itsF2.units[i].inhibited = false;

  bool resonance = false;
  bool exhausted = false;
  int winner = -1;
  do
  {
    setInput(input);

    winner = propagateToF2();
    if (winner != -1) { //we have a winner
      propagateToF1(input, winner);

      //Calculate the magnitude of the input
      double magInput = 0;
      for(uint i=0; i<input.size(); i++)
        magInput += input[i];

      //Calculate the magnitude of the input after propagation
      double magInput_ = 0;
      for(uint i=0; i<itsF1.units.size(); i++)
        magInput_ += itsF1.units[i].output;

      if ((magInput_ / magInput) < itsRho)
        itsF2.units[winner].inhibited = true;
      else
        resonance = true;
    } else {
      exhausted = true;
    }
  } while (! (resonance || exhausted));

  if (resonance)
    adjustWeights(winner);

  if (exhausted)
    LINFO("New input and all Classes exhausted");

  return winner;

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
