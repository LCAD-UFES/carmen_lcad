/*!@file HMAX/test-hmax3.C Test Hmax class and compare to original code */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/test-hmax3.C $
// $Id: test-hmax3.C 6191 2006-02-01 23:56:12Z rjpeters $
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
//int trnFlag = 0;
int mixRate = 5;
#define NORI 4

int main(int argc, char **argv)
{
        // parse options : read directory
        char imname1[1024]; imname1[0] = '\0';
        char imname2[1024]; imname2[0] = '\0';
        strncpy(imname1, argv[1], 1023);
        strncpy(imname2, argv[2], 1023);
        //trnFlag = atoi(argv[2]);
        std::string inName = imname1;
        std::string nsName = imname2;
        if (argc != 3)
                { std::cerr<<"USAGE: test-hmax2 <dir> <trnFlag(1 or 0)>"<<std::endl; exit(1); }
        // weighting for IT
        std::ofstream c2File("c2ResSum");
        Image<byte> inputImgB; Image<byte> noiseImgB;
        // pass image through Hmax model:
        std::vector<int> scss(5);
        scss[0] = 0; scss[1] = 2; scss[2] = 5; scss[3] = 8; scss[4] = 12;
        std::vector<int> spss(4);
        spss[0] = 4; spss[1] = 6; spss[2] = 9; spss[3] = 12;
        Hmax hmax(NORI, spss, scss);
        Timer tim; tim.reset();

        //read a directory for train/rest imgs.
        std::vector<std::string> nsList = hmax.readDir(nsName);
        int nsListSize = nsList.size();
        Image<float> noiseImgF[nsListSize];
        std::vector<std::string> inList = hmax.readDir(inName);
        int inListSize = inList.size();
        Image<float> inputImgF[inListSize];

        //read the input imgs.
        for(int imgInd = 0; imgInd < inListSize; imgInd++) {
                inputImgB = Raster::ReadGray(inList[imgInd], RASFMT_PNM);
                inputImgF[imgInd] = inputImgB;
                //Raster::VisuFloat(inputImgF[imgInd], FLOAT_NORM_0_255, "inputImgF.pgm");
        }

        //read the input imgs.
        for(int nsInd = 0; nsInd < nsListSize; nsInd++) {
                noiseImgB = Raster::ReadGray(nsList[nsInd], RASFMT_PNM);
                noiseImgF[nsInd] = noiseImgB;
                //Raster::VisuFloat(inputImgF[imgInd], FLOAT_NORM_0_255, "inputImgF.pgm");
        }

        for(int nsInd = 0; nsInd < nsListSize; nsInd++) {
                Image<float> nsImg(noiseImgF[nsInd]);
                if (c2File.is_open())
                        c2File << std::endl <<"****** noise Image " << nsList[nsInd].c_str() << std::endl;
                for(int mixInd = 0; mixInd < mixRate; mixInd++) {
                        float nsRate = mixInd * (1.0f / mixRate);
                        if (c2File.is_open()) c2File << std::endl << "****** rate " << nsRate << std::endl;
                        for(int imgInd = 0; imgInd < inListSize; imgInd++) {
                                Image<float> mxImg(inputImgF[imgInd] + (nsImg * nsRate));
                                //Raster::VisuFloat(mxImg, FLOAT_NORM_0_255, "mxImg.pgm");
                                Image<float> c2Mix = hmax.origGetC2(mxImg);
                                //Raster::VisuFloat(c2Mix, FLOAT_NORM_0_255, "c2Mix.pgm");
                                if (c2File.is_open()) {
                                        c2File << sum(c2Mix) << ", ";
                                        if ((imgInd+1)%10 == 0)
                                                c2File << std::endl;
                                }//end of C2 result writing

                        }//end of imgInd
                }//end of mixInd
        }//end of nsInd

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
