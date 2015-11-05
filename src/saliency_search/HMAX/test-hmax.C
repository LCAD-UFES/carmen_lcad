/*!@file HMAX/test-hmax.C This is a test of the C++ hmax object recognition */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/test-hmax.C $
// $Id: test-hmax.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "Channels/Jet.H"
#include "GUI/XWindow.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Kernels.H"   // for dogFilterHmax()
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Raster/Raster.H"
#include "Util/log.H"

#include <cmath>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// show debugging info on memory alloc/free
//#define  ARRAYDATA_DEBUG_ALLOC

// include class definitions for all classes that define or use
// templates. See MakeObjects for why we do that strange thing!

inline float sqr(float x) {
  return (x * x);
}

std::vector<std::string> readDir(std::string inName) {
  DIR *dp = opendir(inName.c_str());
  dirent *dirp;
  std::vector<std::string> fList;
  while ((dirp = readdir(dp)) != NULL ) {
    if (dirp->d_name[0] != '.')
      fList.push_back(inName + '/' + std::string(dirp->d_name));
  }
  std::cout << "# of files ...: " << fList.size() << std::endl;
  std::cout << "file list : " << std::endl;
  for (unsigned int i=0; i<fList.size(); i++)
    std::cout << "\t" << fList[i] << std::endl;
  return fList;
}

// ######################################################################
// ##### Main Program:
// ######################################################################
//! The main routine. Read images, process, display results
int main(int argc, char **argv)
{
  float currStd = 1.75;
  float stdStep = 0.5;
  float currOri = 0.0;
  float oriStep = 45.0;
  int numOri = 4;
  int numScale = 12;
  int numScaleBand = 4;
  int minFS = 7;
  int maxFS = 29;
  int sSFS = 2;
  int fieldSize = 0;
  int cropBegin, cropEnd;
  int sizeOfPR = 4; // size for spatial pooling range in c1
  int poolRange[] = {4, 6, 9, 12};
  int scaleSS[] = {1, 3, 6, 9, 13};
  int fSiz[numScale*numScaleBand];
  int c1OL = 2; // define the ovelap of pooling range in c1
  int c1R = 0; //define c1's abstraction from s1 : receptive range
  int imgHeight = 128;
  int imgWidth = 128;
  int c1RSize = 0;
  int resImg, resX, resY; // index of c1Res[]
  int c1Size, c2Size, s2Size;
  int c2x, c2y;
  float res = 0;
  float s2Res = 0;
  int angleFlag = 1;
  int s2Target = 0; //the mean of the gaussian: center
  int target = 0;
  int tItr[] = {2, 600, 1200};

  // ########## parse options:
  char imname[1024]; imname[0] = '\0';
  strncpy(imname, argv[1], 1023);
  target = atoi(argv[2]);
  std::string inName = imname;

  std::vector<std::string> fileList = readDir(inName);

  // ########## file for restore the result
  std::ofstream wFile ("weightFile");

  // weighting for IT
  float eta = 0.2;
  Image<float> wt(16, 16, NO_INIT);

  // buffer for intermediate c1 response
  Image<float> c1Buff[numScaleBand];
  // c1 result : 4 * 4 =16 features
  Image<float> c1Res[numOri*sizeOfPR];
  // c2 result : a 16 * 16 = 256 pixels image
  Image<float> c2Res(16, 16, NO_INIT);
  Image<byte> im;
  Image<float> filters[numOri*numScale];
  Image<float> tmpFilt;

  // display weights:
  XWindow xww(Dims(256, 256), -1, -1, "C2->IT Weights");
  Image<byte> xwi(256, 256, NO_INIT);
  XWindow xwimg(Dims(imgWidth, imgHeight), -1, -1, "Original Image");

  //for(unsigned int tItrInd = 0; tItrInd < 3; tItrInd++) {
  for(int itr=0; itr < tItr[0]; itr++) {
    for(unsigned int imgInd=0; imgInd<fileList.size(); imgInd++) {
      int fInd = 0;
      std::cout << "**********img..." << imgInd << std::endl;
      std::cout << "target..." << target << std::endl;

      //read input Image and convert to float
      im = Raster::ReadGray(fileList[imgInd], RASFMT_PNM);
      std::cout << "the size of image...im...." << im.getSize() << std::endl;
      rescale(im, imgWidth, imgHeight);

      xwimg.drawImage(im);

      Image<float> fim = im;
      //Raster::VisuFloat(fim,FLOAT_NORM_0_255,"float-image.pgm");

      //***************************************************
      //making filters : numOri(4) * numScale(12) = 48 filters
      for(int filtSize=minFS; filtSize<=maxFS; filtSize+=sSFS) {
        fieldSize = (int)(filtSize * ::sqrt(2) + 1);
        fieldSize = fieldSize + 1 + (fieldSize % 2);
        cropBegin = (int)((fieldSize-filtSize) / 2) + 1;
        cropEnd = cropBegin + filtSize;
        currStd += stdStep;
        currOri = 0;
        for(int oriInd = 0; oriInd < numOri; oriInd ++) {
          tmpFilt.resize(fieldSize,fieldSize,true);
          filters[fInd].resize(maxFS*maxFS,1,true);
          currOri += oriStep;
          tmpFilt = dogFilterHmax<float>(currStd,currOri,cropBegin,cropEnd);

          fSiz[fInd] = tmpFilt.getHeight();
          int filtElm = 0;
          for(int y = 0; y < filtSize; y++) {
            for(int x = 0; x < filtSize; x ++) {
              filters[fInd].setVal(filtElm,0,(tmpFilt.getVal(x,y)));
              filtElm++;
            }
          }
          //Raster::VisuFloat(filters[fInd],FLOAT_NORM_0_255,"dog-filters.pgm");
          fInd ++;
        }       //end of oriInd
      }       //end of filtSize : end of making filters
      //**********************************end of making filters

      //*************  copy img and pad with 0s : expand by maxFS size
      Image<float> fExpnd(imgHeight+maxFS, imgWidth+maxFS, NO_INIT);
      int xVal = 0;
      int yVal = 0;
      //fExpnd.resize(imgHeight+maxFS, imgWidth+maxFS, true);
      for(int y = 0; y < imgHeight+maxFS; y ++) {
        if( y < (int)(maxFS/2) || y >= imgHeight+(int)(maxFS/2)) {
          for(int x = 0; x < imgWidth+maxFS; x ++)
            fExpnd.setVal(x,y,0.0);
        } // end of if
        else {
          for(int x = 0; x < imgWidth+maxFS; x ++) {
            if( x < (int)(maxFS/2) || x >= imgWidth+(int)(maxFS/2))
              fExpnd.setVal(x,y,0.0);
            else {
              fExpnd.setVal(x,y,fim.getVal(xVal,yVal));
              xVal ++;
            }
          }
          xVal = 0;
          yVal ++;
        } // end of else
      } //end of copy img

      //*********************************** S1/C1
      int bStart = (int)(maxFS/2);
      int currBX = 0;
      int currBY = 0;
      fInd = 0;

      //loop for scales of filters
      for(int scaleInd = 0; scaleInd < numScaleBand; scaleInd ++) {
        //initialize c1Buff[]
        for(int i = 0; i < numScaleBand; i ++) {
          c1Buff[i].resize(imgHeight,imgWidth,true);
        }
        for(int currScale = scaleSS[scaleInd];
            currScale < scaleSS[scaleInd+1]; currScale++) {
          for(int oriInd = 0; oriInd < numOri; oriInd ++) {
            int fSize = fSiz[numOri * (currScale - 1) + oriInd];
            for(int y = 0; y < imgHeight; y ++) {
              for(int x = 0; x < imgWidth; x ++) {
                float intRes = 0.0;
                float imgLen = 0.0;
                currBX = bStart - (int)(fSize/2) + x;
                currBY = bStart - (int)(fSize/2) + y;
                for(int hig = 0; hig < fSize; hig ++) {
                  for(int wid = 0; wid < fSize; wid ++) {
                    imgLen += sqr(fExpnd.getVal(currBX+wid,currBY+hig));
                    intRes += fExpnd.getVal(currBX+wid,currBY+hig)
                      * filters[fInd].getVal(wid+(hig*fSize),0);
                  }
                }// end of a filter
                if(angleFlag && (imgLen > 0.0)) intRes /= ::sqrt(imgLen);
                //rectify : take the absolute value
                if(intRes < 0.0) intRes = -intRes;
                if(intRes > c1Buff[oriInd].getVal(x,y))
                  c1Buff[oriInd].setVal(x,y,intRes);
              } // x
            }// end of an img : y
            fInd ++;
          }//end of orientations of filters
        }//end of currScale

        //decide pooling range
        c1R = (int)ceil((double)poolRange[scaleInd]/c1OL);
        int c1BuffInd = 0;
        for(int oInd = 0; oInd < numOri; oInd ++) {
          //initialize c1Res[]:just for a pooling range size of scale band:4
          c1RSize = (int)ceil((double)imgHeight/c1R);
          //trick for reverse c1Res order : <3,2,1,0>, <7,6,5,4>, <11,6,5,4>
          //and <15,14,13,12>
          resImg = (scaleInd * numScaleBand) + 3 - oInd;
          c1Res[resImg].resize(c1RSize,c1RSize,true);
          resX = resY = 0;
          //loop for an c1Buff
          for(int y = 0; y < imgHeight; y += c1R) {
            for(int x = 0; x < imgWidth; x += c1R) {
              //loop for an pooling range
              for(int yInPR = y; (yInPR-y < poolRange[scaleInd])
                    && (yInPR < imgHeight); yInPR ++) {
                for(int xInPR = x; (xInPR-x < poolRange[scaleInd])
                      && (xInPR < imgWidth); xInPR ++) {
                  if(c1Buff[c1BuffInd].getVal(xInPR,yInPR)
                     > c1Res[resImg].getVal(resX, resY))
                    c1Res[resImg].setVal (resX,resY,
                                          c1Buff[c1BuffInd].getVal(xInPR,yInPR));
                } //: xInPR
              } //: yInPR
              resX ++;
            } //: x
            resY ++;
            resX = 0;
          }//end of an c1Buff loop : y
          c1BuffInd++;
        }
      }//end of scales of filters

      Image<float> c1All;
      c1All = concatArray(c1Res,16,4,64,64);
      Raster::VisuFloat(c1All,FLOAT_NORM_0_255,"c1All.pgm");

      //***************************************************** S2/C2 begin
      //The result of C1 layer is on c1Res[]
      //now for S2 layer : combine C1 results
      s2Size = c2Size = 0;
      c2x = c2y = 0;
      for(int oInd1 = 0; oInd1 < numOri; oInd1++)
        for(int oInd2 = 0; oInd2 < numOri; oInd2++)
          for(int oInd3 = 0; oInd3 < numOri; oInd3++)
            for(int oInd4 = 0; oInd4 < numOri; oInd4++) {
              c2Res.setVal(c2x,c2y,-1e10);
              res = -1e10;
              for(int scaleInd = 0; scaleInd < numScaleBand; scaleInd++) {
                c1Size = c1Res[scaleInd * numOri].getHeight();
                for(int y = c1OL; y < c1Size; y ++) {
                  for(int x = c1OL; x < c1Size; x ++) {
                    s2Res = (
                             sqr(c1Res[(scaleInd*numOri) +
                                       oInd1].getVal(y,x) - s2Target) +
                             sqr(c1Res[(scaleInd*numOri) +
                                       oInd2].getVal(y,x-c1OL) - s2Target) +
                             sqr(c1Res[(scaleInd*numOri) +
                                       oInd3].getVal(y-c1OL,x) - s2Target) +
                             sqr(c1Res[(scaleInd*numOri) +
                                       oInd4].getVal(y-c1OL,x-c1OL) - s2Target));
                    if(s2Res > res)  res = s2Res;
                    s2Size ++;
                  }// end of "x" for c1Size
                }// end of "y" for c1Size
                if(res > c2Res.getVal(c2x, c2y))
                  c2Res.setVal(c2x, c2y, res);
              } //end of scaleInd
              c2Size++;
              if(c2x >= 15) {
                c2x = 0;
                c2y++;
              }
              else c2x++;
            } //end of oInd4
      //Raster::VisuFloat(c2Res, FLOAT_NORM_0_255, "dog-c2Res.pgm");

      //*****************************************************
      //learning
      float udWt = 0;
      Image<float> wtC2Res(16, 16, NO_INIT);
      std::cout << "before the sum of ud wt is ... " << sum(wt) << std::endl;
      std::cout << "before the mean ud wt is ... " << mean(wt) << std::endl;

      //initialize the weight to "1"
      for(int y = 0; y < 16; y ++) {
        for(int x = 0; x < 16; x ++) {
          //std::cout << x << "," << y << " th.." << c2Res.getVal(x, y) << std::endl;
          wt.setVal(x,y,1);
          //std::cout << x << "," << y << " th.." << wt.getVal(x, y) << std::endl;
        }
      }

      //update the weight
      for(int y = 0; y < 16; y ++) {
        for(int x = 0; x < 16; x ++) {
          if (target == 1)
            udWt = wt.getVal(x,y) * (1 + eta*c2Res.getVal(x,y));
          else if (target == 0)
            udWt = wt.getVal(x,y) * (1 - eta*c2Res.getVal(x,y));
          wt.setVal(x,y,udWt);
          //wtC2Res.setVal(x,y,udWt*c2Res.getVal(x,y));
          //std::cout << x <<"(aUDwt)"<< y <<" th.."<< wt.getVal(x, y) << std::endl;
        }
      }

      //normalize the weight
      //wt = wt/mean(wt);
      float wVal = 0.0;
      for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 16; x++) {
          if (wVal <= wt.getVal(x,y))
            wVal = wt.getVal(x,y);
        }
      }
      wt = wt/wVal;
/*
      //check the normalized/updated weight
      for(int y = 0; y < 16; y ++) {
        for(int x = 0; x < 16; x ++) {
          std::cout << x << "(aNMwt)" << y << " th.." << wt.getVal(x, y) << std::endl;
        }
      }
*/
      //write the result to a file
      if (wFile.is_open()) {
        wFile << "########## image.." << imgInd << " iteration"<<itr<<std::endl;
        wFile << "the sum of ud wt is ... " << sum(wt);
        wFile << "   the mean ud wt is ... " << mean(wt) << std::endl;

        //print out the wight matrix
        for(int y = 0; y < 16; y ++) {
          //std::cout << "*" << y << "* ";
          for(int x = 0; x < 16; x ++) {
            wFile << wt.getVal(x, y) << "\t";
            wtC2Res.setVal(x,y,wt.getVal(x,y)*c2Res.getVal(x,y));
          }
          wFile << std::endl;
        }
        wFile << "wt sum..." << sum(wtC2Res) << std::endl;
      }//end of file writing

      // show the weights:
      Image<float> wt2(wt); inplaceNormalize(wt2, 0.0f, 255.0f);
      xwi.clear(255);
      for (int i = 1; i < 256; i ++) {
        Point2D<int> p1(i-1, (int)wt2.getVal(i-1)), p2(i, (int)wt2.getVal(i));
        drawLine(xwi, p1, p2, byte(0));
      }
      xww.drawImage(xwi);

    }//end of imgInd
  }//end of itr
  //}//end of tItrInd
  wFile.close();
}//end of main

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
