/*!@file Gist/FFN.C Feed Forward Network  for use to train any mapping  */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/FFN.C $
// $Id: FFN.C 13712 2010-07-28 21:00:40Z itti $
//

// ######################################################################
/*! This implements a simple feedforward network with backpropagation.
  Supports 2 or 3-weight-layer networks through separate function calls
  NOTE: please allow the class to create the initial weight files
        (use the init function that does not ask for weight files).
        Also user should add an extra input for bias (set it to 1.0)   */

#include "Gist/FFN.H"
#include "Image/MatrixOps.H"
#include "Image/CutPaste.H"
#include "Util/log.H"

// ######################################################################
// ######################################################################
// Feed Forward Network member definitions:
// ######################################################################
// ######################################################################

// ######################################################################
FeedForwardNetwork::FeedForwardNetwork()
{ }

// ######################################################################
FeedForwardNetwork::~FeedForwardNetwork()
{ }

// ######################################################################
// sigmoid function
void FeedForwardNetwork::inPlaceSigmoid(Image<double>& dst)
{
  Image<double>::iterator aptr = dst.beginw(), stop = dst.endw();

  while (aptr != stop) {
    double val;
    if(*aptr >  30.0) val = 1.0;
    else if(*aptr < -30.0) val = 0.0;
    else val =  1.0 / (1.0 + exp(-1.0 * (*aptr)));
    *aptr++ = val;
  }
}

// ######################################################################
// 2-weight-layer network initialization
void FeedForwardNetwork::init
(int inunits, int hidunits, int outunits, double lrate, double mrate)
{
  std::string fname("none");
  init(fname, fname, inunits, hidunits, outunits, lrate, mrate);
}

// ######################################################################
// 2-weight-layer network initialization
void FeedForwardNetwork::init(std::string wh_file, std::string wo_file,
                              int inunits, int hidunits, int outunits,
                              double lrate, double mrate)
{
  // two layer network flag
  itsNumLayer = 2;

  // allocate input, hidden, output layer nodes
  itsInputLayerPotential.resize(1, inunits+1);
  itsHiddenLayerPotential.resize(1, hidunits+1);
  itsOutputLayerPotential.resize(1, outunits);

  // allocating input layer -> hidden layer weights
  // an extra input for inserting bias constant
  itsHiddenLayerWeight.resize(inunits+1, hidunits);

  // allocating hidden layer -> output layer weights
  itsOutputLayerWeight.resize(hidunits+1, outunits);

  // momentum not inserted as it did not help in simulation
  // and it costs time and memory
  //itsHiddenLayerMomentum.resize(inunits+1, hidunits);
  //itsOutputLayerMomentum.resize(hidunits+1, outunits);

  // initialize the bias nodes, they remain at 1.0
  itsInputLayerPotential.setVal(0, inunits, 1.0);
  itsHiddenLayerPotential.setVal(0, hidunits, 1.0);

  // allocate errors storage in the 3 layers
  // for error back propagations
  itsError.resize(1, outunits);
  itsHiddenLayerDelta.resize(1, hidunits);
  itsOutputLayerDelta.resize(1, outunits);

  // laerning and momentum rates
  itsLearningRate = lrate;
  itsMomentumRate = mrate;

  // initialize the input layer -> hidden layer weights
  FILE *fp; Image<double>::iterator aptr;
  if((fp = fopen(wh_file.c_str(),"rb")) == NULL)
    {
      LINFO("initializing wh with random weights");
      srand((unsigned)time(0));
      aptr =  itsHiddenLayerWeight.beginw();
      for(int i = 0; i < inunits+1; i++)
        for(int j = 0; j < hidunits; j++)
          *aptr++ = (-FFN_RW_RANGE/2.0) +
            (rand()/(RAND_MAX + 1.0) * FFN_RW_RANGE);
    }
    else
    {
      LINFO("reading whfile");
      Image<double> temp(hidunits, inunits+1, NO_INIT);
      double val; aptr =  temp.beginw();
      for(int i = 0; i < inunits+1; i++)
        for(int j = 0; j < hidunits; j++)
          { if (fread(&val, sizeof(double), 1, fp) != 1) LERROR("fread error"); *aptr++ = val; }
      itsHiddenLayerWeight = transpose(temp);
      fclose(fp);
    }

  // initialize the hidden layer -> output layer weights
  if((fp=fopen(wo_file.c_str(),"rb")) == NULL)
    {
      LINFO("initializing wo with random weights");
      srand((unsigned)time(0));
      aptr =  itsOutputLayerWeight.beginw();
      for(int i = 0; i < hidunits+1; i++)
        for(int j = 0; j < outunits; j++)
          *aptr++ = (-FFN_RW_RANGE/2.0) +
            (rand()/(RAND_MAX + 1.0) * FFN_RW_RANGE);
    }
  else
    {
      LINFO("reading wofile");
      Image<double> temp(outunits, hidunits+1, NO_INIT);
      double val; aptr =  temp.beginw();
      for(int i = 0; i < hidunits+1; i++)
        for(int j = 0; j < outunits; j++)
          { if (fread(&val, sizeof(double), 1, fp) != 1) LERROR("fread error"); *aptr++ = val; }
      itsOutputLayerWeight = transpose(temp);
      fclose(fp);
    }
}

// ######################################################################
// 2-weight-layer network initialization
void FeedForwardNetwork::init(Image<double> wh, Image<double> wo,
                              double lrate, double mrate)
{
  // two layer network flag
  itsNumLayer = 2;

  ASSERT(wh.getHeight() == wo.getWidth() - 1);
  int inunits  = wh.getWidth() - 1;
  int hidunits = wh.getHeight();
  int outunits = wo.getHeight();

  // allocate input, hidden, output layer nodes
  itsInputLayerPotential.resize(1, inunits+1);
  itsHiddenLayerPotential.resize(1, hidunits+1);
  itsOutputLayerPotential.resize(1, outunits);

  // allocating input layer -> hidden layer weights
  // an extra input for inserting bias constant
  itsHiddenLayerWeight.resize(inunits+1, hidunits);

  // allocating hidden layer -> output layer weights
  itsOutputLayerWeight.resize(hidunits+1, outunits);

  // momentum not inserted as it did not help in simulation
  // and it costs time and memory
  //itsHiddenLayerMomentum.resize(inunits+1, hidunits);
  //itsOutputLayerMomentum.resize(hidunits+1, outunits);

  // initialize the bias nodes, they remain at 1.0
  itsInputLayerPotential.setVal(0, inunits, 1.0);
  itsHiddenLayerPotential.setVal(0, hidunits, 1.0);

  // allocate errors storage in the 3 layers
  // for error back propagations
  itsError.resize(1, outunits);
  itsHiddenLayerDelta.resize(1, hidunits);
  itsOutputLayerDelta.resize(1, outunits);

  // laerning and momentum rates
  itsLearningRate = lrate;
  itsMomentumRate = mrate;

  // assign the input layer -> hidden layer weights
  itsHiddenLayerWeight = wh;

  // assign the hidden layer -> output layer weights
  itsOutputLayerWeight = wo;
}

// ######################################################################
// 3-weight-layer network initialization
void FeedForwardNetwork::init3L
( int inunits, int hid1units, int hid2units,
  int outunits, double lrate, double mrate)
{
  std::string fname("none");
  init3L(fname, fname, fname, inunits, hid1units, hid2units, outunits,
         lrate, mrate);
}

// ######################################################################
// 3-weight-layer initialization
void FeedForwardNetwork::init3L
( std::string wh_file, std::string wh2_file, std::string wo_file,
  int inunits, int hidunits, int hid2units,
  int outunits, double lrate, double mrate)
{
  // flag for three layer network
  itsNumLayer = 3;

  // allocate input, hidden1, hidden2, output layer nodes
  itsInputLayerPotential.resize  (1, inunits+1);
  itsHiddenLayerPotential.resize (1, hidunits+1);
  itsHiddenLayer2Potential.resize(1, hid2units+1);
  itsOutputLayerPotential.resize (1, outunits);

  // allocating input layer -> hidden layer1 weights
  // an extra input for inserting bias constant
  itsHiddenLayerWeight.resize(inunits+1, hidunits);

  // allocating hidden layer1 -> hidden2 layer weights
  itsHiddenLayer2Weight.resize(hidunits+1, hid2units);

  // allocating hidden layer2 -> output layer weights
  itsOutputLayerWeight.resize(hid2units+1, outunits);

  // initialize the bias nodes, they remain at 1.0
  itsInputLayerPotential.setVal(0, inunits, 1.0);
  itsHiddenLayerPotential.setVal(0, hidunits, 1.0);
  itsHiddenLayer2Potential.setVal(0, hid2units, 1.0);

  // allocate errors storage in the 4 layers
  // for error back propagations
  itsError.resize(1, outunits);
  itsHiddenLayerDelta.resize(1, hidunits);
  itsHiddenLayer2Delta.resize(1, hid2units);
  itsOutputLayerDelta.resize(1, outunits);

  // initialize learning rate
  itsLearningRate = lrate;
  itsMomentumRate = mrate;

  // initialize the input layer -> hidden layer1 weights
  FILE *fp; Image<double>::iterator aptr;
  if((fp = fopen(wh_file.c_str(),"rb")) == NULL)
    {
      LINFO("initializing wh with random weights");
      srand((unsigned)time(0));
      aptr =  itsHiddenLayerWeight.beginw();
      for(int i = 0; i < inunits+1; i++)
        for(int j = 0; j < hidunits; j++)
          *aptr++ = (-FFN_RW_RANGE/2.0) +
            (rand()/(RAND_MAX + 1.0) * FFN_RW_RANGE);
    }
    else
    {
      LINFO("reading whfile");
      Image<double> temp(hidunits, inunits+1, NO_INIT);
      double val; aptr =  temp.beginw();
      for(int i = 0; i < inunits+1; i++)
        for(int j = 0; j < hidunits; j++)
          { if (fread(&val, sizeof(double), 1, fp) != 1) LERROR("fread error"); *aptr++ = val; }
      itsHiddenLayerWeight = transpose(temp);
      fclose(fp);
    }

  // initialize the hidden layer1 -> hidden layer2 weights
  if((fp = fopen(wh2_file.c_str(),"rb")) == NULL)
    {
      LINFO("initializing wh2 with random weights");
      srand((unsigned)time(0));
      aptr =  itsHiddenLayer2Weight.beginw();
      for(int i = 0; i < hidunits+1; i++)
        for(int j = 0; j < hid2units; j++)
          *aptr++ = (-FFN_RW_RANGE/2.0) +
            (rand()/(RAND_MAX + 1.0) * FFN_RW_RANGE);
    }
    else
    {
      LINFO("reading wh2file");
      Image<double> temp(hid2units, hidunits+1, NO_INIT);
      double val; aptr =  temp.beginw();
      for(int i = 0; i < hidunits+1; i++)
        for(int j = 0; j < hid2units; j++)
          { if (fread(&val, sizeof(double), 1, fp) != 1) LERROR("fread error"); *aptr++ = val; }
      itsHiddenLayer2Weight = transpose(temp);
      fclose(fp);
    }

  // initialize the hidden layer2 -> output layer weights
  if((fp=fopen(wo_file.c_str(),"rb")) == NULL)
    {
      LINFO("initializing wo with random weights");
      srand((unsigned)time(0));
      aptr =  itsOutputLayerWeight.beginw();
      for(int i = 0; i < hid2units+1; i++)
        for(int j = 0; j < outunits; j++)
          *aptr++ = (-FFN_RW_RANGE/2.0) +
            (rand()/(RAND_MAX + 1.0) * FFN_RW_RANGE);
    }
  else
    {
      LINFO("reading wofile");
      Image<double> temp(outunits, hid2units+1, NO_INIT);
      double val; aptr =  temp.beginw();
      for(int i = 0; i < hid2units+1; i++)
        for(int j = 0; j < outunits; j++)
          { if (fread(&val, sizeof(double), 1, fp) != 1) LERROR("fread error"); *aptr++ = val; }
      itsOutputLayerWeight = transpose(temp);
      fclose(fp);
    }
}

// ######################################################################
// 3-weight-layer initialization
void FeedForwardNetwork::init3L
( Image<double> wh, Image<double> wh2, Image<double> wo,
  double lrate, double mrate)
{
  // flag for three layer network
  itsNumLayer = 3;

  ASSERT(wh.getHeight() == wh2.getWidth() - 1);
  ASSERT(wh2.getHeight() == wo.getWidth() - 1);
  int inunits   = wh.getWidth() - 1;
  int hidunits  = wh.getHeight();
  int hid2units = wh2.getHeight();
  int outunits  = wo.getHeight();

  // allocate input, hidden1, hidden2, output layer nodes
  itsInputLayerPotential.resize  (1, inunits+1);
  itsHiddenLayerPotential.resize (1, hidunits+1);
  itsHiddenLayer2Potential.resize(1, hid2units+1);
  itsOutputLayerPotential.resize (1, outunits);

  // allocating input layer -> hidden layer1 weights
  // an extra input for inserting bias constant
  itsHiddenLayerWeight.resize(inunits+1, hidunits);

  // allocating hidden layer1 -> hidden2 layer weights
  itsHiddenLayer2Weight.resize(hidunits+1, hid2units);

  // allocating hidden layer2 -> output layer weights
  itsOutputLayerWeight.resize(hid2units+1, outunits);

  // initialize the bias nodes, they remain at 1.0
  itsInputLayerPotential.setVal(0, inunits, 1.0);
  itsHiddenLayerPotential.setVal(0, hidunits, 1.0);
  itsHiddenLayer2Potential.setVal(0, hid2units, 1.0);

  // allocate errors storage in the 4 layers
  // for error back propagations
  itsError.resize(1, outunits);
  itsHiddenLayerDelta.resize(1, hidunits);
  itsHiddenLayer2Delta.resize(1, hid2units);
  itsOutputLayerDelta.resize(1, outunits);

  // initialize learning rate
  itsLearningRate = lrate;
  itsMomentumRate = mrate;

  // assign the input layer -> hidden layer weights
  itsHiddenLayerWeight = wh;

  // assign the hidden layer -> hidden layer 2 weights
  itsHiddenLayer2Weight = wh2;

  // assign the hidden layer 2 -> output layer weights
  itsOutputLayerWeight = wo;
}

// ######################################################################
// run the 2-weight-layer network
Image<double> FeedForwardNetwork::run(Image<double> input)
{
  // set the input layer potential
  Image<double>::iterator aptr = input.beginw(), stop = input.endw();
  Image<double>::iterator bptr = itsInputLayerPotential.beginw();
  while (aptr != stop) *bptr++ = *aptr++;

  // compute hidden layer (bias stays at 1.0)
  Image<double> thlp =
    matrixMult(itsHiddenLayerWeight, itsInputLayerPotential);
  inPlaceSigmoid(thlp);
  aptr = thlp.beginw(), stop = thlp.endw();
  bptr = itsHiddenLayerPotential.beginw();
  while (aptr != stop) *bptr++ = *aptr++;

  // compute output layer
  itsOutputLayerPotential =
    matrixMult(itsOutputLayerWeight, itsHiddenLayerPotential);
  inPlaceSigmoid(itsOutputLayerPotential);

  return itsOutputLayerPotential;
}

// ######################################################################
// run the 3-weight-layer network
Image<double> FeedForwardNetwork::run3L(Image<double> input)
{
  // set the input layer potential
  Image<double>::iterator aptr = input.beginw(), stop = input.endw();
  Image<double>::iterator bptr = itsInputLayerPotential.beginw();
  while (aptr != stop) *bptr++ = *aptr++;

  // compute hidden layer (bias stays at 1.0)
  Image<double> thlp =
    matrixMult(itsHiddenLayerWeight, itsInputLayerPotential);
  inPlaceSigmoid(thlp);
  aptr = thlp.beginw(), stop = thlp.endw();
  bptr = itsHiddenLayerPotential.beginw();
  while (aptr != stop) *bptr++ = *aptr++;

  // compute hidden2 layer (bias stays at 1.0)
  Image<double> thl2p =
    matrixMult(itsHiddenLayer2Weight, itsHiddenLayerPotential);
  inPlaceSigmoid(thl2p);
  aptr = thl2p.beginw(), stop = thl2p.endw();
  bptr = itsHiddenLayer2Potential.beginw();
  while (aptr != stop) *bptr++ = *aptr++;

  // compute output layer
  itsOutputLayerPotential =
    matrixMult(itsOutputLayerWeight, itsHiddenLayer2Potential);
  inPlaceSigmoid(itsOutputLayerPotential);

  return itsOutputLayerPotential;
}

// ######################################################################
void FeedForwardNetwork::backprop(Image<double> target)
{
  // propagate error from the output to hidden layer
  itsError = target - itsOutputLayerPotential;
  Image<double>
    onesO(1,itsOutputLayerPotential.getSize(), ZEROS); onesO += 1.0;
  itsOutputLayerDelta = itsError * itsOutputLayerPotential *
    (onesO - itsOutputLayerPotential);

  // propagate error from hidden layer to input layer
  Image<double>
    onesH(1,itsHiddenLayerPotential.getSize(), ZEROS); onesH += 1.0;
  Image<double> tempDh =
    matrixMult(transpose(itsOutputLayerWeight), itsOutputLayerDelta) *
    itsHiddenLayerPotential * (onesH - itsHiddenLayerPotential);
  itsHiddenLayerDelta =
    crop(tempDh,Point2D<int>(0,0), itsHiddenLayerDelta.getDims());

  // update weights in hidden -> output layer
  itsOutputLayerWeight +=
    (matrixMult(itsOutputLayerDelta, transpose(itsHiddenLayerPotential))
     * itsLearningRate);

  // update weights in input layer -> hidden layer
  itsHiddenLayerWeight +=
    matrixMult(itsHiddenLayerDelta, transpose(itsInputLayerPotential))
    * itsLearningRate;
}

// ######################################################################
void FeedForwardNetwork::backprop3L(Image<double> target)
{
  // propagate error from the output to hidden layer 2
  itsError = target - itsOutputLayerPotential;
  Image<double>
    onesO(1,itsOutputLayerPotential.getSize(), ZEROS); onesO += 1.0;
  itsOutputLayerDelta = itsError * itsOutputLayerPotential *
    (onesO - itsOutputLayerPotential);

  // propagate error from hidden layer 2 to hidden layer
  Image<double>
    onesH2(1,itsHiddenLayer2Potential.getSize(), ZEROS); onesH2 += 1.0;
  Image<double> tempDh2 =
    matrixMult(transpose(itsOutputLayerWeight), itsOutputLayerDelta) *
    itsHiddenLayer2Potential * (onesH2 - itsHiddenLayer2Potential);
  itsHiddenLayer2Delta =
    crop(tempDh2, Point2D<int>(0,0), itsHiddenLayer2Delta.getDims());

  // propagate error from hidden layer to input layer
  Image<double>
    onesH(1,itsHiddenLayerPotential.getSize(), ZEROS); onesH += 1.0;
  Image<double> tempDh =
    matrixMult(transpose(itsHiddenLayer2Weight), itsHiddenLayer2Delta) *
    itsHiddenLayerPotential * (onesH - itsHiddenLayerPotential);
  itsHiddenLayerDelta =
    crop(tempDh, Point2D<int>(0,0), itsHiddenLayerDelta.getDims());

  // update weights in hidden layer 2 -> output layer
  itsOutputLayerWeight +=
    (matrixMult(itsOutputLayerDelta, transpose(itsHiddenLayer2Potential))
     * itsLearningRate);

  // update weights in hidden layer -> hidden layer 2
  itsHiddenLayer2Weight +=
    matrixMult(itsHiddenLayer2Delta, transpose(itsHiddenLayerPotential))
    * itsLearningRate;

  // update weights in input layer -> hidden layer
  itsHiddenLayerWeight +=
    matrixMult(itsHiddenLayerDelta, transpose(itsInputLayerPotential))
    * itsLearningRate;
}

// ######################################################################
// store current weights of 2-weight-layer network to the passed in files
void FeedForwardNetwork::write(std::string wh_file, std::string wo_file)
{
  FILE *fp;

  // store the hidden layer weights
  if((fp = fopen(wh_file.c_str(),"wb")) == NULL)
    LFATAL("can't open wh1");
  Image<double> temp = transpose(itsHiddenLayerWeight);
  Image<double>::iterator aptr = temp.beginw();
  Image<double>::iterator stop = temp.endw();
  while (aptr != stop)
    { double val = *aptr++; fwrite(&val, sizeof(double), 1, fp); }
  fclose(fp);

  // store the output layer weights
  if((fp = fopen(wo_file.c_str(),"wb")) == NULL)
    LFATAL("can't open wo");
  temp = transpose(itsOutputLayerWeight);
  aptr = temp.beginw(); stop = temp.endw();
  while (aptr != stop)
    { double val = *aptr++; fwrite(&val, sizeof(double), 1, fp); }
  fclose(fp);
  //fwrite(temp.getArrayPtr(), sizeof(double), temp.getSize(), fp);
}

// ######################################################################
// store current weights of 3-weight-layer network
// to the passed in files
void FeedForwardNetwork::write3L
(std::string wh_file, std::string wh2_file, std::string wo_file)
{
  FILE *fp;

  // store the hidden layer1 weights
  if((fp = fopen(wh_file.c_str(),"wb")) == NULL)
    LFATAL("can't open wh1");
  Image<double> temp = transpose(itsHiddenLayerWeight);
  Image<double>::iterator aptr = temp.beginw();
  Image<double>::iterator stop = temp.endw();
  while (aptr != stop)
    { double val = *aptr++; fwrite(&val, sizeof(double), 1, fp); }
  fclose(fp);

  // store the hidden layer2 weights
  if((fp = fopen(wh2_file.c_str(),"wb")) == NULL)
    LFATAL("can't open wh2");
  temp = transpose(itsHiddenLayer2Weight);
  aptr = temp.beginw(); stop = temp.endw();
  while (aptr != stop)
    { double val = *aptr++; fwrite(&val, sizeof(double), 1, fp); }
  fclose(fp);

  // store the output layer weights
  if((fp = fopen(wo_file.c_str(),"wb")) == NULL)
    LFATAL("can't open wo");
  temp = transpose(itsOutputLayerWeight);
  aptr = temp.beginw(); stop = temp.endw();
  while (aptr != stop)
    { double val = *aptr++; fwrite(&val, sizeof(double), 1, fp); }
  fclose(fp);
}

// ######################################################################
void FeedForwardNetwork::setLearningRate(double newLR)
{
  itsLearningRate = newLR;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
