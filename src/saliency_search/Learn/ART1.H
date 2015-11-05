/*!@file Learn/ART1.H Adaptive Resonance Theory */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/ART1.H $
// $Id: ART1.H 12962 2010-03-06 02:13:53Z irock $
//
//Inspired from http://www.neural-networks-at-your-fingertips.com/art1.html by Karsten Kutza

#ifndef LEARN_ART1_H_DEFINED
#define LEARN_ART1_H_DEFINED

#include "Util/Types.H" // for uint
#include <vector>
#include <string>

class ART1
{
  public:
    struct Unit
    {
      bool output;
      std::vector<double> weights;
      bool inhibited; //the inhibition status of ith F2 unit
    };


    struct Layer
    {
      std::vector<Unit> units;
    };

    //! init an ART network with inputSize and numClasses
    ART1(const int inputSize, const int numClasses);

    ~ART1();

    //! evolve the network and return the class id
    int evolveNet(std::string in);


    int propagateToF2();
    void propagateToF1(const std::vector<bool> input,const int winner);
    void adjustWeights(const int winner);
    void setInput(const std::vector<bool> input);




private:
    int itsInputSize;
    int itsNumClasses;
    Layer itsF1; //input layer
    Layer itsF2; //Output layer
    double itsA1; //A parameter for first layer
    double itsB1; //B parameter for first layer
    double itsC1; //C parameter for first layer
    double itsD1; //D parameter for first layer
    double itsL; //A parameter for network
    double itsRho; //vigilance parameter
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
