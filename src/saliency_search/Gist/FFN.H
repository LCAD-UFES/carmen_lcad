/*!@file Gist/FFN.H Feed Forward Network  for use to train any mapping       */

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
// Created by Chris Ackerman
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/FFN.H $
// $Id: FFN.H 12309 2009-12-18 01:26:50Z beobot $
//

#ifndef GIST_FFN_H
#define GIST_FFN_H

#include "Image/Image.H"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string>

#define FFN_RW_RANGE 1.0    // range of initial random weights [-.5 ... .5]

// ######################################################################
//! A Feed Forward Network
class FeedForwardNetwork
{

public:

  // ######################################################################
  /*! @name Constructors, assignment, and Destructors */
  //@{

  //! Constructor
  FeedForwardNetwork();

  //! Destructor
  ~FeedForwardNetwork();

  //! Initialize a two-weight-layer network with saved weights
  //! momentum is unimplemented
  void init(std::string wh_file, std::string wo_file,
            int inunits, int hidunits, int outunits,
            double lrate, double mrate);

  //! Initialize a two-weight-layer network with small random weights
  //! momentum is unimplemented
  void init(int inunits, int hidunits, int outunits,
            double lrate, double mrate);

  //! Initialize a two-weight-layer network with Image<double> weights
  void init(Image<double> wh, Image<double> wo,
            double lrate, double mrate);

  //! Initialize a three-weight-layer network with saved weights
  //! momentum is unimplemented
  void init3L(std::string wh1_file, std::string wh2_file, std::string wo_file,
              int inunits, int hid1units, int hid2units, int outunits,
              double lrate, double mrate);

  //! Initialize a two-weight-layer network with small random weights
  //! momentum is unimplemented
  void init3L(int inunits, int hid1units, int hid2units,
              int outunits, double lrate, double mrate);

  //! Initialize a three-weight-layer network with Image<double> weights
  void init3L(Image<double> wh, Image<double> wh2, Image<double> wo,
              double lrate, double mrate);

  //! Run a two-weight-layer network with passed in input
  Image<double> run(Image<double> input);

  //! Run a three-weight-layer network with passed in input
  Image<double> run3L(Image<double> input);

  //! backpropagate the error in the two-weight-layer network
  //! with passed in target output (note: call run first)
  void backprop(Image<double> target);

  //! backpropagate the error in the three-weight-layer network
  //! with passed in target output (note: call run3L first)
  void backprop3L(Image<double> target);

  //! Write the two-weight-layer network weights to a specified file
  void write(std::string wh_file,
             std::string wo_file);

  //! Write the three-weight-layer network weights to a specified file
  void write3L(std::string wh1_file,
               std::string wh2_file,
               std::string wo_file);

  //! Set Learning rate - for simulated annealing
  void setLearningRate(double newLR);

  //! get the most recent output
  inline Image<double> getOutput();

private:

  //! sigmoid function
  void inPlaceSigmoid(Image<double>& dst);

  //! input layer
  Image<double> itsInputLayerPotential;

  //! hidden layer
  Image<double> itsHiddenLayerPotential;
  Image<double> itsHiddenLayerWeight;
  Image<double> itsHiddenLayerMomentum;
  Image<double> itsHiddenLayer2Potential;
  Image<double> itsHiddenLayer2Weight;

  //! output layer
  Image<double> itsOutputLayerPotential;
  Image<double> itsOutputLayerWeight;
  Image<double> itsOutputLayerMomentum;

  // error information
  Image<double> itsError;
  Image<double> itsOutputLayerDelta;
  Image<double> itsHiddenLayerDelta;
  Image<double> itsHiddenLayer2Delta;

  //! the network's learning rate
  double itsLearningRate;

  //! the network's learning rate
  double itsMomentumRate;

  //! the number of hidden layers + output layer
  int itsNumLayer;
};

// ######################################################################
// Implementation for FeedForwardNetwork inline functions
// ######################################################################
inline Image<double> FeedForwardNetwork::getOutput()
{
  return itsOutputLayerPotential;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
