/*!@file Learn/SOFM.H Self-Organizing Map neural network  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SOFM.H $
// $Id: SOFM.H 13223 2010-04-15 00:52:46Z lior $
//

#ifndef LEARN_SOFM_H_DEFINED
#define LEARN_SOFM_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/ColorOps.H"

class SOFM
{
public:
  enum DISTANCE_MEASURE {EUCLIDEAN, KL,L2GMM};

  SOFM(const char *netname, int InputSize, int x, int y);                        //init with map size x y
  ~SOFM();
  void SetLearningRate(unsigned long long learning_time);
  void Train(float *Input, float *Target, double score);
  void organize(std::vector<double> &input);
  void organize(const Image<float>& input);
  Image<float> getMap();
  Image<float> getActMap();
  Image<PixRGB<byte> > getWeightsImage();
  std::vector<float> getWeights(const Point2D<int> loc);

  void ReadNet(const char *filename);
  void WriteNet(const char *filename);
  double RandomRange(double Low, double High);
  double RandomBinaryRange(double Low, double High);
  double Neighborhood(int i);
  void RandomWeights(int min = 0, int max = 255);
  void ZeroWeights();
  void SetInput(float *in);
  void setInput(const std::vector<double> &in);
  void setInput(const Image<float> &in);
  void Propagate(DISTANCE_MEASURE dm = EUCLIDEAN);
  Point2D<int> getWinner(double& val);


  //private:
  struct Layer {
    int Units;                                //number of units
    double *Output;                //output of i unit
    float **Weight;                //connection weightto i unit
    double *StepSizeX;                //size of search steps of i unit
    double *StepSizeY;                //size of search steps of i unit
    double *dScoreMean;        //mean score delta of i unit
    double *score;                        // the previous score for this move
    int          *BadDim;                // the dimintion that we need to work on
    double *Lambda;                //for displyaing the neigborhood of units
  };

  Layer                *InputLayer;
  Layer                *KohonenLayer;
  int                itsWinner;
  double             itsWinnerValue;
  int                itsLooser;
  double             itsLooserValue;
  int                MapSizeX;
  int                MapSizeY;

  const char    *name;                                        // the name of the network for saving
  double        KohonenAlpha;                                //learning rate of Kohonen layer
  double        OutAlpha;                                        //learning rate of output layer
  double        StepAlpha;                                        //learning rate for  step sizes
  double        Gamma;                                                //smoothing factor for score deltas
  double        Sigma;                                                //width of neighoborhood
  unsigned long long itsLearningTime;          //The current learning time


  void InitSofm();


};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_SOFM_H_DEFINED
