/*!@file Learn/test-Bayes.C test the Bayes network class
*/

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/test-Bayes.C $
// $Id: test-Bayes.C 7040 2006-08-25 16:48:09Z rjpeters $
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Learn/Bayes.H"

int main()
{

    Bayes bn(4, 2); //constract a bayes network with 4 featuers and 2 classes


    std::vector<double> FV(4);

    //Class 0
    FV[0] = 752; FV[1] = 265; FV[2] = 700; FV[3] = 271; bn.learn(FV, 0u);
    FV[0] = 895; FV[1] = 355; FV[2] = 812; FV[3] = 288; bn.learn(FV, 0u);
    FV[0] = 893; FV[1] = 352; FV[2] = 790; FV[3] = 298; bn.learn(FV, 0u);
    FV[0] = 814; FV[1] = 326; FV[2] = 790; FV[3] = 296; bn.learn(FV, 0u);
    FV[0] = 532; FV[1] = 405; FV[2] = 750; FV[3] = 401; bn.learn(FV, 0u);
    FV[0] = 532; FV[1] = 405; FV[2] = 750; FV[3] = 401; bn.learn(FV, 0u);
    FV[0] = 478; FV[1] = 385; FV[2] = 750; FV[3] = 394; bn.learn(FV, 0u);
    FV[0] = 532; FV[1] = 405; FV[2] = 750; FV[3] = 401; bn.learn(FV, 0u);
    FV[0] = 565; FV[1] = 47 ; FV[2] = 710; FV[3] = 142; bn.learn(FV, 0u);
    FV[0] = 689; FV[1] = 127; FV[2] = 955; FV[3] = 162; bn.learn(FV, 0u);

    //Class 1
    FV[0] = 576; FV[1] = 726; FV[2] = 287; FV[3] =719; bn.learn(FV, 1);
    FV[0] = 718; FV[1] = 783; FV[2] = 300; FV[3] =536; bn.learn(FV, 1);
    FV[0] = 859; FV[1] = 724; FV[2] = 270; FV[3] =480; bn.learn(FV, 1);
    FV[0] = 839; FV[1] = 512; FV[2] = 246; FV[3] =657; bn.learn(FV, 1);
    FV[0] = 746; FV[1] = 343; FV[2] = 250; FV[3] =710; bn.learn(FV, 1);
    FV[0] = 660; FV[1] = 527; FV[2] = 272; FV[3] =763; bn.learn(FV, 1);
    FV[0] = 704; FV[1] = 621; FV[2] = 263; FV[3] =713; bn.learn(FV, 1);
    FV[0] = 684; FV[1] = 836; FV[2] = 287; FV[3] =213; bn.learn(FV, 1);
    FV[0] = 678; FV[1] = 800; FV[2] = 377; FV[3] =220; bn.learn(FV, 1);
    FV[0] = 624; FV[1] = 697; FV[2] = 494; FV[3] =238; bn.learn(FV, 1);


    LINFO("Class 0");
    for(uint i=0; i<bn.getNumFeatures(); i++)
      LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(0, i), bn.getStdevSq(0, i));

    LINFO("Class 1");
    for(uint i=0; i<bn.getNumFeatures(); i++)
      LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(1, i), bn.getStdevSq(1, i));

    LINFO("Class 0 frq %i prob %f", bn.getClassFreq(0), bn.getClassProb(0));
    LINFO("Class 1 frq %i prob %f", bn.getClassFreq(1), bn.getClassProb(1));


    //New FV to classify
    FV[0] = 750; FV[1] = 269; FV[2] = 720; FV[3] = 291;
    int cls = bn.classify(FV); //classify a given FV
    LINFO("FV1 belongs to class %i", cls);

    FV[0] = 458; FV[1] = 381; FV[2] = 350; FV[3] = 392;
    cls = bn.classify(FV); //classify a given FV
    LINFO("FV2 belongs to class %i", cls);


    bn.save("Bayes.net");

    bn.load("Bayes.net");

    LINFO("Class 0");
    for(uint i=0; i<bn.getNumFeatures(); i++)
      LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(0, i), bn.getStdevSq(0, i));

    LINFO("Class 1");
    for(uint i=0; i<bn.getNumFeatures(); i++)
      LINFO("Feature %i: mean %f, stddevSq %f", i, bn.getMean(1, i), bn.getStdevSq(1, i));

    LINFO("Class 0 frq %i prob %f", bn.getClassFreq(0), bn.getClassProb(0));
    LINFO("Class 1 frq %i prob %f", bn.getClassFreq(1), bn.getClassProb(1));


    //New FV to classify
    FV[0] = 750; FV[1] = 269; FV[2] = 720; FV[3] = 291;
    cls = bn.classify(FV); //classify a given FV
    LINFO("FV1 belongs to class %i", cls);

    FV[0] = 458; FV[1] = 381; FV[2] = 350; FV[3] = 392;
    cls = bn.classify(FV); //classify a given FV
    LINFO("FV2 belongs to class %i", cls);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
