/*!@file HMAX/test-hmax2.C Test Hmax class and compare to original code */

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
// Primary maintainer for this file: Daesu Chung <dchung@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/test-hmax2.C $
// $Id: test-hmax2.C 6191 2006-02-01 23:56:12Z rjpeters $
//

#include "HMAX/Hmax.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Raster/Raster.H"
#include "Util/Timer.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <fstream>
#include <iostream>

int tItr = 1;
int c2Size = 0;
int target = 0;
std::string targetS1 = "tri";
std::string targetS2 = "Tri";
int imgHeight = 0;
int imgWidth = 0;
int trnFlag = 0;

float w[] = {1,1};
//## img.../data/tstData//cir9.pgm itr 41 eta 0.00477165
//the sum ... 206.082   the mean 0.805006

#define NORI 4

int main(int argc, char **argv)
{
        // parse options : read directory
        char imname[1024]; imname[0] = '\0';
        strncpy(imname, argv[1], 1023);
        trnFlag = atoi(argv[2]);
        std::string inName = imname;

        if (argc != 3)
                { std::cerr<<"USAGE: test-hmax2 <dir> <trnFlag(1 or 0)>"<<std::endl; exit(1); }


        // weighting for IT
        float eta = 0.3;
        std::ofstream wFile("weightFile");
        std::ofstream c2File("c2ResSum");
        Image<float> wt;
        Image<float> wtC2res;

        Image<byte> input;

/*
        Image<byte> tImg1;
        Image<byte> nImg1;
        Raster::ReadGray(tImg1, "tImg1.pgm");
        Raster::ReadGray(nImg1, "nImg1.pgm");
        Image<float> tstImg(tImg1);
        Image<float> nosImg(nImg1);
        Image<float> rstImg(tstImg+(nosImg*0.2f));
        Raster::VisuFloat(tstImg, FLOAT_NORM_0_255, "tstImg.pgm");
        Raster::VisuFloat(nosImg, FLOAT_NORM_0_255, "nosImg.pgm");
        Raster::VisuFloat(rstImg, FLOAT_NORM_0_255, "rstImg.pgm");
*/

        // pass image through Hmax model:
        std::vector<int> scss(5);
        scss[0] = 0; scss[1] = 2; scss[2] = 5; scss[3] = 8; scss[4] = 12;
        std::vector<int> spss(4);
        spss[0] = 4; spss[1] = 6; spss[2] = 9; spss[3] = 12;
        Hmax hmax(NORI, spss, scss);
        Timer tim; tim.reset();

        if (trnFlag == 0)       tItr = 1;

        //read a directory for train/rest imgs.
        std::vector<std::string> fileList = hmax.readDir(inName);
        int listSize = fileList.size();
        Image<float> oc2resp[listSize];

        //for batch process : read and do hmax for all imgs at once.
        for(int imgInd = 0; imgInd < listSize; imgInd++) {
                input = Raster::ReadGray(fileList[imgInd], RASFMT_PNM);
                Image<float> inputf(input);
                oc2resp[imgInd]=hmax.origGetC2(inputf);
                //Raster::VisuFloat(oc2resp, FLOAT_NORM_0_255, "oc2resp.pgm");
                imgHeight = inputf.getHeight();
                imgWidth = inputf.getWidth();
                c2Size = oc2resp[imgInd].getHeight();

                //write the C2 result to a file
                /*
                if (c2File.is_open()) {
                        c2File << "## image " << fileList[imgInd] << std::endl;
                        for(int y = 0; y < 16; y ++) {
                                for(int x = 0; x < 16; x ++)
                                        c2File << oc2resp[imgInd].getVal(x, y) << " ";
                                }
                        c2File << std::endl;
                        c2File <<"the sum "<<sum(oc2resp[imgInd])<<" the mean "<<mean(oc2resp[imgInd])<<std::endl;
                }//end of C2 result writing
                */
                if (c2File.is_open()) {
                        c2File << sum(oc2resp[imgInd]) << ", ";
                        if (imgInd%10 == 0)
                                c2File << std::endl;
                }//end of C2 result writing
        }

        //Initialize the weight with rand. number b/w 0 and 1
        wt.resize(c2Size, c2Size, true);
        srand(time(NULL));
        for(int y = 0; y < c2Size; y++) {
                for(int x = 0; x < c2Size; x++) {
                        float r = rand()%10;
                        wt.setVal(x,y,r/10);
                }
        }

        for(int itr = 0; itr < tItr; itr++) {
                eta = eta - (itr * (9*eta)/(10*tItr) );
                for(int imgInd = 0; imgInd < listSize; imgInd++) {
                        //training****************************
                        if(trnFlag == 1) {
                                float udWt = 0;
                                //set the target
                                std::string::size_type where1 = fileList[imgInd].find(targetS1);
                                std::string::size_type where2 = fileList[imgInd].find(targetS2);
                                if ((where1 == std::string::npos) && (where2 == std::string::npos))
                                        target = -1;
                                else target = 1;
                                //std::cout << "target...  " << target << std::endl;

                                //update the weight mat.
                                for(int y = 0; y < c2Size; y++) {
                                        for(int x = 0; x < c2Size; x++) {
                                                if (target == 1)
                                                        udWt = wt.getVal(x,y) * (1 + eta*oc2resp[imgInd].getVal(x,y));
                                                else if (target == -1)
                                                        udWt = wt.getVal(x,y) * (1 - eta*oc2resp[imgInd].getVal(x,y));
                                                wt.setVal(x,y,udWt);
                                        }
                                } // end of update the weight

                                inplaceNormalize(wt, 0.0f, 1.0f);
                        } //end of training

                        //testing*****************************
                        else {
                                wtC2res.resize(c2Size, c2Size, true);
                                for(int y = 0; y < 16; y ++) {
                                        for(int x = 0; x < 16; x ++)
                                                wtC2res.setVal(x,y,w[(y*16) + x]);
                                }
                                wt = wtC2res * oc2resp[imgInd];
                        } //end of testing

                        //write the result to a file
                        if (wFile.is_open()) {

                                wFile <<"## img.." <<fileList[imgInd]<<" itr "<<itr<<" eta "<<eta<<std::endl;
                                for(int y = 0; y < 16; y ++) {
                                        for(int x = 0; x < 16; x ++)
                                                wFile << wt.getVal(x, y) << ", ";
                                }
                                wFile << std::endl;
                                wFile << "the sum ... "<<sum(wt)<<"   the mean "<<mean(wt)<<std::endl;

                                /*
                                wFile << sum(wt) << ", ";
                                if(imgInd%10 == 0)
                                        wFile << std::endl;
                                */

                        }//end of file writing
                } // itr
        } // imgInd

  //Image<float> c2resp = hmax.getC2(inputf);
  //LDEBUG("c2Resp computed in %llums", tim.getReset());
  //Image<float> oc2resp=hmax.origGetC2(inputf);
  //LDEBUG("original c2Resp computed in %llums", tim.get());
  //Raster::VisuFloat(c2resp, FLOAT_NORM_0_255, "c2resp.pgm");
  return 0;
} //end of main

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
