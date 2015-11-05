/*!@file VFAT/test-NPclassify2.C  Test the non-parametric classifier */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/test-NPclassify2.C $
// $Id: test-NPclassify2.C 6003 2005-11-29 17:22:45Z rjpeters $
//

// ############################################################
// ############################################################
// ##### ---NPclassify---
// ##### non-parametric classifier:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Vidhya Navalpakkam - navalpak@usc.edu
// ##### partners full name - email
// ############################################################
// ############################################################

//This is the start of the execution path for the NPclassify test alg.#include "log.H"

#include "VFAT/NPclassify2.H"


#define INVALS 31

//! This is the configFile name
char* configFile;
//! This is the configFile object
readConfig configIn(25);
readConfig polySet(25);
//! number of items if training
int itemNumber;
int features;
int datasize;

int main(int argc, char* argv[])
{
  float inVals[INVALS];
  // get test file
  std::ifstream inFile(argv[1],std::ios::in);
  features = (int)atof(argv[2]);
  datasize = (int)atof(argv[3]);
  for(int i = 0; i < INVALS; i++)
  {
    inVals[i] = atof(argv[4+i]);
  }

  // create operating objects
  configIn.openFile("NPclassify.conf");
  polySet.openFile("polySet.conf");

  std::string in;
  std::vector<float> feature(features,0);
  std::vector<std::vector<float> > vectorIn(datasize,feature);

  long featureCount = 0;
  int column = 0;
  int row = 0;
  bool comment = false;

  // convert test file to vector format
  //std::cerr << "Parsing config file" << argv[1] << "\n";
  while (inFile >> in)
  {
    if(!in.compare("#")) //comment code # found
    {
      if(!comment)
      {
        comment = true;
      }
      else      //end of comment
      {
        comment = false;
      }
    }
    if((!comment) && in.compare("#")) //real line found
    {
      vectorIn[row][column] = atof(in.c_str()) * atof(argv[INVALS+4+column]);
      //std::cerr << "Adding " << in << "[" << row << "]"
      //    << "[" << column << "]\n";
      column++;
      if(column == features)
      {
        column = 0;
        row++;
        featureCount++;
      }
    }
  }

  // (1) create the NP classify object, input conf files, specify if settings
  // are from command line
  NPclassify2<float> NP(configIn,polySet,false);
  // (2) input any command line arguments if any

  NP.NPinputCommandLineSettings(inVals);

  //----------------------------------------------------------------//

  // classify space using density webs
  // (3) specify the size of your space in samples and dimensions
  NP.NPresizeSpace(featureCount,features);
  // (4) input the current vector into the NP clusterer
  NP.NPaddSpace(vectorIn);
  // (5) start the alg.
  NP.NPclassifySpaceNew(false);
  std::cerr << "DONE\n";

  long roots = NP.NPgetStemNumberEdit();
  std::cout << roots << "\n";
}
