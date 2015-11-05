/*!@file Learn/SOFM.C Self-Organizing Map network */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/SOFM.C $
// $Id: SOFM.C 14390 2011-01-13 20:17:22Z pez $
//

#ifndef LEARN_SOFM_C_DEFINED
#define LEARN_SOFM_C_DEFINED

#include "Learn/SOFM.H"
#include "Image/CutPaste.H"
#include "GUI/DebugWin.H"
#include <fcntl.h>
#include <stdio.h>

SOFM::SOFM(const char *net_name, int InputSize, int x, int y){


  name = net_name;
  MapSizeX = x;
  MapSizeY = y;

  InputLayer = new Layer;
  KohonenLayer = new Layer;

  //alocate input
  InputLayer->Units = InputSize;
  InputLayer->Output = new double[InputSize];

  //alocate map
  KohonenLayer->Units = x*y;
  KohonenLayer->Output = new double[x*y];
  KohonenLayer->Weight = new float*[x*y];
  KohonenLayer->StepSizeX = new double[x*y];
  KohonenLayer->StepSizeY = new double[x*y];
  KohonenLayer->dScoreMean = new double[x*y];
  KohonenLayer->score = new double[x*y];
  KohonenLayer->BadDim = new int[x*y];
  KohonenLayer->Lambda = new double[x*y];

  for (int i=0; i<KohonenLayer->Units; i++){
    KohonenLayer->Weight[i] = new float[InputLayer->Units];
  }


  itsWinner = -1; //no winner
  itsLooser = -1; //no winner
  KohonenAlpha = 0;
  OutAlpha = 0 ;
  StepAlpha = 0;
  Gamma = 0;
  Sigma = 0.1;

  itsLearningTime = 0;
}

void SOFM::InitSofm(){

  for(int i=0; i<KohonenLayer->Units; i++){
    KohonenLayer->StepSizeX[i] = -20;
    KohonenLayer->StepSizeY[i] = -20;
    KohonenLayer->dScoreMean[i] = 0;
    KohonenLayer->score[i] = 0;
    KohonenLayer->BadDim[i] = 0;
    KohonenLayer->Lambda[i] = 0;
  }
}

double SOFM::RandomRange(double Low, double High){
  return ( (double) rand() / RAND_MAX) * (High - Low) + Low;
}

double SOFM::RandomBinaryRange(double Low, double High){
  int i;
  i = (int)(1.0*rand()/(RAND_MAX+1.0));
  if (i)
    return High;
  else
    return Low;
}


void SOFM::RandomWeights(int min, int max){
  initRandomNumbers();
  for (int i=0; i<KohonenLayer->Units; i++){
    for (int j=0; j<InputLayer->Units; j++){
      KohonenLayer->Weight[i][j] = (float)RandomRange(min, max);
    }
  }
}

void SOFM::ZeroWeights(){

  for (int i=0; i<KohonenLayer->Units; i++){
    for (int j=0; j<InputLayer->Units; j++){
      KohonenLayer->Weight[i][j] = 0;
    }
  }
}


SOFM::~SOFM(){
  delete InputLayer;
  delete KohonenLayer;
}


void SOFM::SetInput(float *in){
  for (int i=0; i<InputLayer->Units; i++){
    InputLayer->Output[i] = in[i];
  }
}

void SOFM::setInput(const std::vector<double> &in){
  for (int i=0; i<InputLayer->Units; i++){
    InputLayer->Output[i] = in[i];
  }
}

void SOFM::setInput(const Image<float> &in){
  for (int i=0; i<InputLayer->Units; i++){
    InputLayer->Output[i] = in[i];
  }
}

Image<float> SOFM::getMap(){

  Image<float> sofmOut(MapSizeX,MapSizeY,NO_INIT);

  //convert the SOFM to image
  Image<float>::iterator dest_itr = sofmOut.beginw();

  int i=0;
  for (Image<float>::iterator itr = sofmOut.beginw(), stop = sofmOut.endw();
      itr != stop; itr++, dest_itr++, i++) {
    *dest_itr = (float)KohonenLayer->Output[i];
  }

  return sofmOut;

}

Image<float> SOFM::getActMap(){

  Image<float> sofmOut(MapSizeX,MapSizeY,NO_INIT);

  //convert the SOFM to image
  Image<float>::iterator dest_itr = sofmOut.beginw();

  int i=0;
  for (Image<float>::iterator itr = sofmOut.beginw(), stop = sofmOut.endw();
      itr != stop; itr++, dest_itr++, i++) {
    *dest_itr = (float)KohonenLayer->Output[i];
  }

  return sofmOut;

}

Image<PixRGB<byte> > SOFM::getWeightsImage(){


  int i=0;
  Image<PixRGB<byte> > sofmOut;

  if (InputLayer->Units == 3)
  {
    sofmOut = Image<PixRGB<byte> >(MapSizeX,MapSizeY,NO_INIT);

    //convert the SOFM to image
    Image<PixRGB<byte> >::iterator dest_itr = sofmOut.beginw();

    for (Image<PixRGB<byte> >::iterator itr = sofmOut.beginw(), stop = sofmOut.endw();
        itr != stop; itr++, dest_itr++, i++) {
      *dest_itr = PixRGB<byte>(KohonenLayer->Weight[i][0], KohonenLayer->Weight[i][1], KohonenLayer->Weight[i][2]);
    }
  } else {
    int patchSize = (int)sqrt(InputLayer->Units);
    int w = MapSizeX*patchSize;
    int h = MapSizeY*patchSize;
    Image<float> tmp(w,h,NO_INIT);

    for(int j=0; j<MapSizeY; j++)
      for(int i=0; i<MapSizeX; i++)
      {
        int weightIdx = j*MapSizeY + i;
        Image<float> patch(patchSize, patchSize, NO_INIT);
        for(uint pix=0; pix<patch.size(); pix++)
          patch[pix] = KohonenLayer->Weight[weightIdx][pix];
        inplaceNormalize(patch, 0.0F, 255.0F);
        inplacePaste(tmp, patch, Point2D<int>(i*patchSize,j*patchSize));
      }

    sofmOut = toRGB(tmp);

  }


  return sofmOut;

}

std::vector<float> SOFM::getWeights(const Point2D<int> loc)
{
  std::vector<float> weights;

  int weightIdx = loc.j*MapSizeY + loc.i;
  for(int i=0; i<InputLayer->Units; i++)
    weights.push_back(KohonenLayer->Weight[weightIdx][i]);

  return weights;

}


void SOFM::Propagate(DISTANCE_MEASURE dm){

  double minOut = +HUGE_VAL, maxOut = -HUGE_VAL;
  int winner = -1, looser = -1;
  //Caculate Output of all units

  for (int i=0; i<KohonenLayer->Units; i++){

    switch (dm)
    {
      case EUCLIDEAN:
        {
          //find the sum of the input*wight^2
          double sum = 0;
          for(int j=0; j<InputLayer->Units; j++)
          {
            double out = InputLayer->Output[j];
            double weight = KohonenLayer->Weight[i][j];
            sum += (out - weight)*(out - weight);
          }
          KohonenLayer->Output[i] = sqrt(sum);
        }
        break;
      case KL:
        {
          //use the kl Distance
          double sum = 0;
          for(int j=0; j<InputLayer->Units; j++){
            double out = InputLayer->Output[j];
            double weight = KohonenLayer->Weight[i][j];
            sum += weight*log(weight/out);
          }
          KohonenLayer->Output[i] = fabs(sum);
        }
        break;
      case L2GMM:
        {



        }
    }

    //find the winner
    if (KohonenLayer->Output[i] < minOut){
      minOut = KohonenLayer->Output[i];
      winner = i;
    }
    if (KohonenLayer->Output[i] > maxOut){
      maxOut = KohonenLayer->Output[i];
      looser = i;
    }

  }

  itsWinner = winner;
  itsWinnerValue = minOut;
  itsLooser = looser;
  itsLooserValue = maxOut;

}

double SOFM::Neighborhood(int i){

  double Distance;
  int iRow, iCol, jRow, jCol;

  iRow = i / MapSizeX; jRow = itsWinner / MapSizeX;
  iCol = i % MapSizeX; jCol = itsWinner % MapSizeX;

  Distance = sqrt(((iRow-jRow)*(iRow-jRow)) + ((iCol-jCol)*(iCol-jCol)));

  return exp(-(Distance*Distance) / (2*(Sigma*Sigma)));
}

Point2D<int> SOFM::getWinner(double& val)
{
  int y = itsWinner/MapSizeX;
  int x = itsWinner - (y*MapSizeX);

  val = itsWinnerValue;

  return Point2D<int>(x,y);
}


void SOFM::Train(float *Input, float *Target, double score){

  double Out, Weight, Lambda=0;

  //set the map to the input
  for(int i=0; i<KohonenLayer->Units; i++){
    Lambda = Neighborhood(i);
    if (Lambda > 0.000001){
      for(int j=0; j<InputLayer->Units; j++){
        Out = Input[j];
        Weight = KohonenLayer->Weight[i][j];
        KohonenLayer->Weight[i][j] += KohonenAlpha * Lambda * (Out - Weight);
      }
    }
    KohonenLayer->Lambda[i] = Lambda;         //set the Lambda for display
    //StepSize = KohonenLayer->StepSize[i];
    //KohonenLayer->StepSize[i] += StepAlpha * Lambda * -StepSize;
  }


  /*
  //set the score

  KohonenLayer->score[Winner] = score;
  for(int i=0; i<KohonenLayer->Units; i++){
  Lambda = Neighborhood(i);
  if (Lambda > 0.1)        //only update if not set from before
  if (KohonenLayer->score[i] == 0){
  KohonenLayer->score[i] = score;
  }
  if (i != Winner){
//KohonenLayer->score[i] += 0.01 * (double)((int)(score/Lambda)%900);        //score increses
}
if (Lambda > 0.1){
KohonenLayer->StepSizeX[i] = KohonenLayer->StepSizeX[Winner];
KohonenLayer->StepSizeY[i] = KohonenLayer->StepSizeY[Winner];
}
if (Lambda > 0.5){
KohonenLayer->BadDim[i] = KohonenLayer->BadDim[Winner];
}

} */

}

void SOFM::organize(std::vector<double> &input){

  double Out, Weight, Lambda=0;

  //set the map to the input
  for(int i=0; i<KohonenLayer->Units; i++){
    Lambda = Neighborhood(i);
    if (Lambda > 0.000001){
      for(int j=0; j<InputLayer->Units; j++){
        Out = input[j];
        Weight = KohonenLayer->Weight[i][j];
        KohonenLayer->Weight[i][j] += KohonenAlpha * Lambda * (Out - Weight);
      }
    }
    KohonenLayer->Lambda[i] = Lambda;         //set the Lambda for display
    //StepSize = KohonenLayer->StepSize[i];
    //KohonenLayer->StepSize[i] += StepAlpha * Lambda * -StepSize;
  }

}

void SOFM::organize(const Image<float>& input){

  double Out, Weight, Lambda=0;

  //increment the learning time
  SetLearningRate(itsLearningTime++);

  //set the map to the input
  for(int i=0; i<KohonenLayer->Units; i++){
    Lambda = Neighborhood(i);
    if (Lambda > 0.000001){
      for(int j=0; j<InputLayer->Units; j++){
        Out = input[j];
        Weight = KohonenLayer->Weight[i][j];
        //KohonenLayer->Weight[i][j] += KohonenAlpha * Lambda * (Out - Weight);
        KohonenLayer->Weight[i][j] += 1 * Lambda * (Out - Weight);
      }
    }
    KohonenLayer->Lambda[i] = Lambda;         //set the Lambda for display
    //StepSize = KohonenLayer->StepSize[i];
    //KohonenLayer->StepSize[i] += StepAlpha * Lambda * -StepSize;
  }

}

void SOFM::SetLearningRate(unsigned long long learning_time){

  //set the net param
  KohonenAlpha         = 0.1 * pow(0.001, (double) learning_time / 1000);
  OutAlpha             = 1; //0.5 * pow(0.01, (double) learning_time / 1000);
  StepAlpha           = 0.05;
  Gamma                     = 0.05;
  // after the leraning time, always update about 0.5 around the winner
  // this is why the 1.5 +
  Sigma                    =  0.5 + ( MapSizeX * pow(0.2, (double) learning_time / 1000));

  //LINFO("%llu: Sigma %f, Alpha %f",
  //    learning_time, Sigma, KohonenAlpha);

}

void SOFM::ReadNet(const char *filename){
  int fd;
  int in, sizeX, sizeY;

  if ((fd = open(filename, 0, 0644)) == -1) return;

  printf("Reading from %s\n", filename);
  //read the # of input, size of sofm, and output
  if(read(fd, (char *) &in, sizeof(int)) != sizeof(int)) LFATAL("fread error");
  if(read(fd, (char *) &sizeX, sizeof(int)) != sizeof(int)) LFATAL("fread error");
  if(read(fd, (char *) &sizeY, sizeof(int)) != sizeof(int)) LFATAL("fread error");
  if(read(fd, (char *) &itsLearningTime, sizeof(itsLearningTime)) != sizeof(itsLearningTime)) LFATAL("fread error");

  printf("%i %i %i %llu\n", in, sizeX, sizeY, itsLearningTime);

  //read the weights

  for (int i=0; i<KohonenLayer->Units; i++){
    int sz = sizeof(float)*InputLayer->Units;
    if(read(fd, KohonenLayer->Weight[i], sz) != sz) LFATAL("fread failed");
  }

  close(fd);

}

void SOFM::WriteNet(const char *filename){
  int fd;

  if ((fd = creat(filename, 0644)) == -1) {
    printf("Can not open %s for saving\n", filename);
    return;
  }

  //write the # of input, size of sofm, and output
  if(write(fd, (char *) &InputLayer->Units, sizeof(int)) != sizeof(int)) LFATAL("write failed");
  if(write(fd, (char *) &MapSizeX, sizeof(int)) != sizeof(int)) LFATAL("write failed");
  if(write(fd, (char *) &MapSizeY, sizeof(int)) != sizeof(int)) LFATAL("write failed");
  if(write(fd, (char *) &itsLearningTime, sizeof(itsLearningTime)) != sizeof(itsLearningTime)) LFATAL("write failed");

  //write the weights

  for (int i=0; i<KohonenLayer->Units; i++){
    int sz = sizeof(float)*InputLayer->Units;
    if(write(fd, KohonenLayer->Weight[i], sz) != sz) LFATAL("write failed");
  }

  close(fd);
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_SOFM_C_DEFINED
