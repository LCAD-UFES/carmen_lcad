/*!@file Gist/train-FFN.C train an multilayer feed-forward netwark
         with backpropagation                                           */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/train-FFN.C $
// $Id: train-FFN.C 14376 2011-01-11 02:44:34Z pez $
//

// ######################################################################
/*! training a neural network                                           */
// //////////////////////////////////////////////////////////
// train-FFN.C <*_train.txt>

// This is a general purpose neural network trainer
// in which we can specify the network architecture as well as
// a set of training parameters in the training file <*_train.txt>.

// Below is an example of a training file to convey the format used:

// // ######################################################################

// File name: Campus_train.txt
// Purpose: train a neural network to recognize different location
// at the USC campus.
// Content:

// /home2/tmp/u/christian/movies/SAL_JEP/  # training folder
// /home2/tmp/u/christian/movies/SAL_JEP2/ # testing folder
// 11                            # number of categories
// PCA                           # check if have (PCA/NO_PCA) step
// Campus.evec                   # PCA eigenvector matrix
// 714                           # number of original dimension
// 80                            # number of reduced dimension (same if NO_PCA)
// 400                           # number of hidden unit at first layer
// 200                           # number of hidden unit at second layer
// .002                          # learning rate
// Campus_gistList.txt           # training file
// Campus2_gistList.txt          # testing file
// CampusB_train_hidden1.nnwt    # first hidden layer weight file name
// CampusB_train_hidden2.nnwt    # second hidden layer weight file name
// CampusB_train_out.nnwt        # output layer weight file name

// Note:
// Most of the content is self explanatory: the first two are the absolute
// location of the training and tsting sample folders.

// The next parameters pertains to the architecture. A three layer network
// with 80, 400, 200, 11 nodes for the respective input, first hidden layer,
// second hidden layer, and output layer.

// The next two files contains list of samples along with their corresponding
// ground truths. Which will be discussed below.

// The last three file names are the file names for the training weights.

// // ######################################################################

// The following is an example of a *_gistList.txt file for the trainer to use.

// // ######################################################################

// File name: Campus_gistList.txt
// Purpose: provide 5254 samples of images of campus at prespecified locations
// ground truth.
// Content:
//
// 5254                     # number of samples (up to 1000 per file tag)
// 11                       # number of categories
// ABSOLUTE                 # ground truth type (MIXTURE/ABSOLUTE)
// tag             start     num     groundTruth extension
// SAL_JEP_1_       0         120     0          .gist
// SAL_JEP_1_       120       258     1          .gist
// SAL_JEP_2_       0         173     1          .gist
// SAL_JEP_3_       0         53      1          .gist
// SAL_JEP_3_       53        21      2          .gist
// SAL_JEP_4_       0         408     2          .gist
// SAL_JEP_5_       0         608     3          .gist
// WaterFntn_       0         292     4          .gist
// LeaveyA_         0         219     5          .gist
// LeaveyB_         0         101     6          .gist
// LeaveyC_         0         178     7          .gist
// w34st_           0         1201    8          .gist
// McClintock_      0         1221    9          .gist
// w37th_           0         401     10         .gist

// Note:
// The tags are prefixes for the file name. Each entry has 'num' files
// (consecutive indexes) from 'start' index.
// So, the first file is SAL_JEP_1_000.gist
// which corresponds to ground truth output of 1.0 for the first node and 0.0
// for the remaining nodes.

// The MIXTURE ground truth type is for desired outputs that are more
// descriptive than simply one winning node. This is NOT YET IMPLEMENTED.

// // ######################################################################

// Related files of interest: FFN.C (and .H) The feed-forward network class.

#include "Component/ModelManager.H"
#include "Gist/FFN.H"
#include "Raster/Raster.H"
#include "Util/MathFunctions.H"
#include "Gist/trainUtils.H"
#include "Image/MatrixOps.H"

#include <vector>

#define ABSOLUTE        0
#define MIXTURE         1
#define ERR_THRESHOLD   .01
#define MAX_EPOCH       1000
// CloseButtonListener wList;

void setupCases
(std::string folder, std::string fname, bool equalize);
void train();
void test();
void run(int isTest);
void diff
(Image<double> ideal, Image<double> out, double &tErr, int &tFc, int &tIc);



rutz::shared_ptr<FeedForwardNetwork> ffn;
int nSamples = 0;
std::vector<Image<double> > in;
std::vector<Image<double> > out;
Image<double> pcaIcaMatrix;

// information from training file
rutz::shared_ptr<FFNtrainInfo> info;

// ######################################################################
// training procedure
int main(const int argc, const char **argv)
{
  // Instantiate a ModelManager:
  ModelManager manager("Feed-Forward Network trainer");

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "<input_train.txt>",
                               1, 1) == false)
    return(1);

  // setup the trainer: multi-layer neural networks with back propagation
  info.reset(new FFNtrainInfo(manager.getExtraArg(0)));

  // instantiate a three-layer feed-forward network
  // initialize with the given parameters
  ffn.reset(new FeedForwardNetwork());
  ffn->init3L(info->h1Name, info->h2Name, info->oName,
              info->redFeatSize, info->h1size, info->h2size, info->nOutput,
              info->learnRate, 0.0);

  // setup PCA/ICA reduction matrix - if necessary
  if(info->isPCA)
    {
      pcaIcaMatrix = setupPcaIcaMatrix
        (info->trainFolder+info->evecFname,
         info->oriFeatSize, info->redFeatSize);
    }

  // train the network
  printf("would you like to skip training and just test the network? "
         "(y/n - default y)");
  char spC = getchar(); getchar();
  if(spC == 'n')
  {
    printf("would you like to equalize the number of samples? "
           "(y/n default y)");
    char spC = getchar(); getchar();
    bool equalize = true;
    if(spC == 'n') equalize = false;

    setupCases(info->trainFolder, info->trainSampleFile, equalize);
    train();
    Raster::waitForKey();
  }

  // test the network
  setupCases(info->testFolder, info->testSampleFile, false);
  test();
  Raster::waitForKey();

  // save the results
  ffn->write3L(info->h1Name, info->h2Name, info->oName);
}

// ######################################################################
// open a testing file containing images and corresponding ground truth
void setupCases(std::string folder, std::string fname, bool equalize)
{
  char comment[200];  FILE *fp;  char inLine[100];

  // open a file that lists the sample with ground truth
  std::string name = folder + fname;
  if((fp = fopen(name.c_str(),"rb")) == NULL)
    {
      LINFO("samples file: %s not found", name.c_str());

      // input and output vector
      out.resize(0);
      in.resize(0);
      nSamples = 0;

      return;
    }
  LINFO("tName: %s",name.c_str());

  // get number of samples
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed"); sscanf(inLine, "%d %s", &nSamples, comment);

  // the number of categories -> has to agree with the training file
  uint tNout;
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed"); sscanf(inLine, "%d %s", &tNout, comment);
  if(tNout != info->nOutput)
    LFATAL("Num categories differ: %d != %d", tNout, info->nOutput);

  // get the type of ground truth
  char gtOpt[100]; int gtType = -1;
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed"); sscanf(inLine, "%s %s", gtOpt, comment);
  if(strcmp(gtOpt,"ABSOLUTE") == 0)
    gtType = ABSOLUTE;
  else if(strcmp(gtOpt,"MIXTURE" ) == 0)
    gtType = MIXTURE;
  else
    LFATAL("unknown ground truth type %s",gtOpt);

  // set up the size input and output vector
  out.resize(nSamples);
  in.resize(nSamples);

  // skip column headers
  if (fgets(inLine, 1000, fp) == NULL) LFATAL("fgets failed");

  char cName[100]; char sName[100]; char iName[100]; char ext[100];
  int cStart, cNum; int gTruth;
  FILE *ifp;
  int count = 0;  int tSamples = 0;
  std::vector<uint> nSamples;
  while(fgets(inLine, 1000, fp) != NULL)
  {
    if(gtType == ABSOLUTE)
      {
        // get the files in this category and ground truth
        sscanf(inLine, "%s %d %d %d %s", cName, &cStart, &cNum,  &gTruth, ext);
        sprintf(sName,"%s%s", folder.c_str(), cName);
        printf("    sName: %s %d %d %d %s\n",sName, cStart, cNum, gTruth, ext);
      }
    else if(gtType == MIXTURE)
      {
        // get the files in this category and ground truth
        //char tStr[300];
        //sscanf(inLine, "%s %d %d %s %s", cName, &cStart, &cNum,  tStr, ext);
        //sprintf(sName,"%s%s", folder, cName);
        //printf(" sName: %s %d %d %d %s\n",sName, cStart, cNum, gTruth, ext);

        // change to mixture values
        LFATAL("MIXTURE ground truth type not yet implemented");
      }
    else LFATAL("unknown ground truth type %s",gtOpt);

    nSamples.push_back(cNum);

    // go through every sample
    for(int j = cStart; j < cStart+cNum; j++)
      {
        tSamples++;
        // get the corresponding vector file (if exist)
        sprintf(iName,"%s%06d%s", sName,j,ext);

        // open the file
        if((ifp = fopen(iName,"rb")) != NULL)
          {
            Image<double> tData(1,info->oriFeatSize, NO_INIT);
            Image<double>::iterator aptr = tData.beginw();

            for(int i = 0; i < tData.getSize(); i++)
              {
                double val; if (fread(&val, sizeof(double), 1, ifp) != 1) LFATAL("fread failed");
                *aptr++ = val;
              }

            LINFO("feature file found: %s (%d)",//%7.4f %7.4f %7.4f %7.4f\n",
                   iName,gTruth);//,tData[0], tData[21], tData[42], tData[63]);
            fclose(ifp);

            // calculate the reduced features
            if(info->isPCA) in[count] = matrixMult(pcaIcaMatrix, tData);
            else in[count] = tData;

            // load the ground truth
            if(gtType == ABSOLUTE)
              {
                Image<double> res(1,info->nOutput, ZEROS);
                res.setVal(0, gTruth, 1.0);
                out[count] = res;
              }
            else if(gtType == MIXTURE)
              {
                LFATAL("MIXTURE ground truth type not yet implemented");
              }
            else LFATAL("unknown ground truth type %s",gtOpt);

//             // just to test stuff
//             for(int k = 0; k < info->oriFeatSize; k++)
//                 printf("ori[%7d]: %f \n", k, tData.getVal(k));
//             printf("\n");

//             for(int k = 0; k < info->redFeatSize; k++)
//                 printf("red[%7d]: %f \n", k, in[count].getVal(k));
//             printf("\n");
//             //for(uint k = 0; k < info->nOutput; k++)
//             //    printf("%f \n",out[count].getVal(k));
//             Raster::waitForKey();

            count++;
          }
        else LFATAL("file: %s not found\n",iName);
      }
  }

  // equalize the number of samples if requested
  if(equalize)
    {
      // find the max
      uint max = 0;
//       for(uint i = 0; i < nSamples.size(); i++)
//         if(max < nSamples[i]) max = nSamples[i];
      max = *max_element(nSamples.begin(),nSamples.end());
      LINFO("max element: %d", max);

      uint offset = 0;
      for(uint i = 0; i < nSamples.size(); i++)
        {
          LINFO("extra samples for class[%3d]: %d - %d -> %d",  
                i, max,  nSamples[i], max - nSamples[i]);
          for(uint j = 0; j < max - nSamples[i]; j++)
            {
              // index to be copied
              uint index = rand()/(RAND_MAX + 1.0) * nSamples[i];
              LINFO("[%d] Duplicating class[%3d] sample[%3d]"
                    " -> actual ind: %3d", 
                    j, i, index, index + offset);
              index = index + offset;

              in.push_back(in[index]);
              out.push_back(out[index]);
            }
          offset += nSamples[i];
        }
      LINFO("Total samples before equalized: %d \n",tSamples);
      tSamples = in.size();
    }

  LINFO("Actual total samples: %d \n",tSamples);
  fclose(fp);
}

// ######################################################################
// train the network
void train() { run(0);};

// ######################################################################
// test the network
void test()  { run(1);};

// ######################################################################
// train or test the network
void run(int isTest)
{
  LINFO("Run the samples");
  double errSum = double(nSamples);
  double err; Image<double> ffnOut;
  int nfc = nSamples; int fc;
  int nfcClass[info->nOutput][info->nOutput];//confusion matrix[target][output]
  int nTrials = 0;
  int target = 0;

  if(nSamples == 0) return;
  int order[nSamples];
  for(int i = 0; i < nSamples; i++) order[i] = i;

  while(nTrials < MAX_EPOCH && !isTest && nfc > int(nSamples*ERR_THRESHOLD))
  {
    // reinitialize statistic variables
    for(uint i = 0; i < info->nOutput; i++)
      for(uint j = 0; j < info->nOutput; j++)
        nfcClass[i][j] = 0;
    errSum = 0.0; nfc = 0;

    // run the input in random order
    randShuffle(order, nSamples);

    for(int i = 0; i < nSamples; i++)
    {
      // run the input
      ffn->run3L(in[order[i]]);
      ffnOut = ffn->getOutput();

      // get the error
      diff(out[order[i]], ffnOut, err, fc, target);

      // add misclassification count
      if(fc != -1)
        {
          nfc++;
          nfcClass[target][fc]++;
        }
      else
        nfcClass[target][target]++;

      // and the numerical deviation
      errSum += err;

      if(fc != -1)
        {
          //ffn->setLearnRate(learnRate*10);
          ffn->backprop3L(out[order[i]]);
          //ffn->setLearnRate(learnRate);
        }
    }
    nTrials++;

    // periodically report progress
    if(nTrials %1 == 0)
      {
        printf("Trial_%04d_Err: %f nfc: %5d/%5d -> %f%%\n",
               nTrials, errSum/nSamples,
               nfc,nSamples,(double)(nfc)/(0.0 + nSamples)*100.0);

        printf("class |");
        for(uint k = 0;  k < info->nOutput; k++)
          printf(" %4d", k);
        printf("\n");
        for(uint k = 0;  k < info->nOutput; k++)
          printf("------");
        printf("\n");
        for(uint k = 0; k < info->nOutput; k++)
        {
          printf("%6d|",k);
          for(uint j = 0; j < info->nOutput; j++)
            printf(" %4d",nfcClass[k][j]);
          printf("\n");
        }
      }
    printf("\n");
  }

  // print the results if testing
  if(isTest)
    {
      nfc = 0; errSum = 0.0; err = 0;
      for(uint i = 0; i < info->nOutput; i++)
        for(uint j = 0; j < info->nOutput; j++)
          nfcClass[i][j] = 0;

      for(int i = 0; i < nSamples; i++)
        {
          // run the input
          ffn->run3L(in[i]);

          // get the output
          ffnOut = ffn->getOutput();

          // get the error
          diff(out[i], ffnOut, err, fc, target);

          // add misclassification count
          if(fc != -1)
            {
              nfc++;
              nfcClass[target][fc]++;
            }
          else
            nfcClass[target][target]++;

          // and the numerical deviation
          errSum += err;

          if((fc != -1) | 1)
            {
              printf("sample %5d: ",i);
              for(uint j = 0; j < info->nOutput; j++)
                printf("%.3f ",out[i][j]);
              printf(" -:- ");
              for(uint j = 0; j < info->nOutput; j++)
                printf("%.3f ",ffnOut[j]);
            }
          if(fc != -1) printf(" WRONG! NO:%d  [%d][%d] = %d \n",
                              nfc, target, fc, nfcClass[target][fc]);
          else printf("\n");
        }
    }

  // final error count
  printf("Final Trial_%04d_Err: %f nfc: %5d/%5d -> %.3f%%\n",
         nTrials,errSum/nSamples,
         nfc,nSamples,(double)(nfc)/(0.0 + nSamples)*100.0);

  printf("class |");
  for(uint k = 0;  k < info->nOutput; k++)
    printf(" %5d",k);
  printf("     Total          pct. err \n-------");
  for(uint k = 0;  k < info->nOutput; k++)
    printf("------");
  printf("\n");
  for(uint k = 0; k < info->nOutput; k++)
    {
      int t = 0, e = 0;
      printf("%6d|",k);
      for(uint j = 0; j < info->nOutput; j++)
        {
          printf(" %5d",nfcClass[k][j]);
          if(k == j)
            t = nfcClass[k][j];
          else
            e += nfcClass[k][j];
        }
      if(e+t == 0)
        printf(" %6d/%6d     N/A%%\n",0,0);
      else
        printf(" %6d/%6d  %6.2f%%\n",e,e+t, float(e)/float(e+t)*100.0);
    }

  for(uint k = 0;  k < info->nOutput; k++)
    printf("------");
  printf("-------\nFalse+|");
  for(uint k = 0; k < info->nOutput; k++)
    {
      int e = 0;
      for(uint j = 0; j < info->nOutput; j++)
        {
          if(k == j)
            ; //t = nfcClass[j][k];
          else
            e += nfcClass[j][k];
        }
      printf(" %5d",e);
    }
  printf("\ntotal |");
  for(uint k = 0; k < info->nOutput; k++)
    {
      int t = 0, e = 0;
      for(uint j = 0; j < info->nOutput; j++)
        {
          if(k == j)
            t = nfcClass[j][k];
          else
            e += nfcClass[j][k];
        }
      printf(" %5d",e+t);
    }
  printf("\nerr:  |");
  for(uint k = 0; k < info->nOutput; k++)
    {
      int t = 0, e = 0;
      for(uint j = 0; j < info->nOutput; j++)
        {
          if(k == j)
            t = nfcClass[j][k];
          else
            e += nfcClass[j][k];
        }
      if(e+t == 0)
        printf("  N/A");
      else
        printf(" %5.2f",float(e)/float(e+t)*100.0);
    }
  printf("\n");
}

// ######################################################################
//  calculate the difference between the 2 vectors
void diff
(Image<double> ideal, Image<double> out,
 double &tErr, int &tFc, int &tIc)
{
  tErr = 0.0;
  Image<double>::iterator iptr = ideal.beginw();
  Image<double>::iterator optr = out.beginw();
  for(uint i = 0; i < info->nOutput; i++)
    tErr += fabs(*iptr++ - *optr++);

  int iMaxI = 0, oMaxI = 0;
  iptr = ideal.beginw(); optr = out.beginw();
  double iMax = *iptr++, oMax = *optr++;
  for(uint i = 1; i < info->nOutput; i++)
    {
      double ival = *iptr++;
      double oval = *optr++;
      if(ival > iMax) { iMax = ival; iMaxI = i; }
      if(oval > oMax) { oMax = oval; oMaxI = i; }
    }

  // indication of incorrect output
  tFc = -1; if(iMaxI != oMaxI) tFc = oMaxI;
  tIc = iMaxI;
}

// code for best/second best stuff
//       // get output of the first and second most likely level
//       if(vout[0] > vout[1])
//       {
//         max  = 0; mval  =  vout[0];
//         max2 = 1; mval2 =  vout[1];
//       }
//       else
//       {
//         max  = 1; mval  =  vout[1];
//         max2 = 0; mval2 =  vout[0];
//       }

//       for(int j = 2; j < NUM_H_LEV; j++)
//         if(mval < vout[j])
//         {
//           max2  = max;
//           mval2 = mval;
//           max   = j;
//           mval  = vout[j];
//         }
//         else
//         if(mval2 < vout[j])
//         {
//           max2   = j;
//           mval2  = vout[j];
//         }

//       // add the level misclassification
//       if(max != rmax) nfc++;
//       if(max != rmax && max2 != rmax ) nfc2++;

//     printf("avgErr: %f real:  %d predict( %d,%d) ",
//            errSum/NUM_H_LEV, rmax, max, max2);
//     if(max != rmax)
//       printf(" WRONG %d ",nfc);
//     if(max != rmax && max2 != rmax )
//       printf(" WRONG2 %d\n",nfc2);
//     else
//       printf(" \n");
//     printf("\n");
//   }
//   printf("Test_%04d_Err: %f nfc: %d nfc2: %d\n",
//          nTrials,avgErr/nSamples/NUM_H_LEV, nfc, nfc2);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
