/*! @file Gist/test-FFN.C -- testing the FFN class                      */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-FFN.C $
// $Id: test-FFN.C 13712 2010-07-28 21:00:40Z itti $

#include "rutz/shared_ptr.h"
#include "Gist/FFN.H"
#include "Raster/Raster.H"
#include "Image/MatrixOps.H"
#include "Gist/trainUtils.H"
#include <string>
#include <vector>

int main(const int argc, const char **argv)
{
  LINFO("Testing with small Neural Network");

  // create a network with 2->3->2 architecture
  rutz::shared_ptr<FeedForwardNetwork> ffn(new FeedForwardNetwork());

  Image<double> wh(10,7, NO_INIT);
  wh.setVal(0,0, 1.0); wh.setVal(0,1, 1.0); wh.setVal(0,2, 1.0);
  wh.setVal(1,0, 1.0); wh.setVal(1,1, 1.0); wh.setVal(1,2, 1.0);
  wh.setVal(2,0, 1.0); wh.setVal(2,1, 1.0); wh.setVal(2,2, 1.0);

  Image<double> wh2(8,5, NO_INIT);
  wh2.setVal(0,0, 1.0); wh2.setVal(0,1, 1.0); wh2.setVal(0,2, 1.0);
  wh2.setVal(1,0, 1.0); wh2.setVal(1,1, 1.0); wh2.setVal(1,2, 1.0);
  wh2.setVal(2,0, 1.0); wh2.setVal(2,1, 1.0); wh2.setVal(2,2, 1.0);

  Image<double> wo(6,3, NO_INIT);
  wo.setVal(0,0, 1.0);  wo.setVal(0,1, 1.0);
  wo.setVal(1,0, 1.0);  wo.setVal(1,1, 1.0);
  wo.setVal(2,0, 1.0);  wo.setVal(2,1, 1.0);
  wo.setVal(3,0, 1.0);  wo.setVal(3,1, 1.0);

  ffn->init3L(wh, wh2, wo, 0.1, 0.0);

  // set the input
  Image<double> in(1,9, NO_INIT);
  in.setVal(0, 0, 1.0);
  in.setVal(0, 1, 2.0);

  // run it
  Image<double> out = ffn->run3L(in);
  for(int i = 0; i < out.getSize(); i++)
    LINFO("%d: %f", i, out.getVal(0,i));

  // train it
  Image<double> target(1,3, NO_INIT);
  target.setVal(0, 0, 0.5);
  target.setVal(0, 1, 0.2);
  target.setVal(0, 2, 0.7);
  ffn->backprop3L(target);

  // run it again
  out = ffn->run3L(in);
  for(int i = 0; i < out.getHeight(); i++)
    LINFO("%d: %f", i, out.getVal(i));

  // ===============================================
  // ===============================================
  Raster::waitForKey();
  LINFO("Now testing with provided files");

  rutz::shared_ptr<FeedForwardNetwork> ffn2(new FeedForwardNetwork());
  std::string infoFName("../data/PAMI07data/ACB_GIST_train.txt");
  FFNtrainInfo pcInfo(infoFName);

  ffn2->init3L(pcInfo.h1Name, pcInfo.h2Name, pcInfo.oName,
               pcInfo.redFeatSize, pcInfo.h1size, pcInfo.h2size,
               pcInfo.nOutput, 0.0, 0.0);

  // setup the PCA eigenvector
  Image<double> pcaVec =
    setupPcaIcaMatrix(pcInfo.trainFolder+pcInfo.evecFname,
                      pcInfo.oriFeatSize, pcInfo.redFeatSize);

  // open an input
  Image<double> oriin(1,pcInfo.oriFeatSize,NO_INIT);
  Image<double>::iterator aptr = oriin.beginw();
  FILE *gfp;
  std::string gfname = pcInfo.trainFolder + std::string("ACB1B_000.gist");
  if((gfp = fopen(gfname.c_str(),"rb")) != NULL)
    {
      LINFO("gist file found: %s", gfname.c_str());
      for(uint i = 0; i < pcInfo.oriFeatSize; i++)
        {
          double tval;  if (fread(&tval, sizeof(double), 1, gfp) != 1) LFATAL("fread failed");
          *aptr++ = tval;
        }
      fclose(gfp);
    }
  else LFATAL("gist file NOT found: %s", gfname.c_str());

  // run the ffn
  out = ffn2->run3L(matrixMult(pcaVec,oriin));
  for(int i = 0; i < out.getHeight(); i++)
    LINFO("%d: %f", i, out.getVal(i));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
