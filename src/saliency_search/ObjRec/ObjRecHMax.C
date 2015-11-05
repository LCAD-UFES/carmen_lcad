/*!@file ObjRec/ObjRecHMax.C Obj Reconition class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRecHMax.C $
// $Id: ObjRecHMax.C 9108 2007-12-30 06:14:30Z rjpeters $
//

#include "ObjRec/ObjRecHMax.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "GUI/DebugWin.H"


// number of orientations to use in Hmax
#define NORI 4

// ######################################################################
ObjRecHMax::ObjRecHMax(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsHmax(NULL)
{
  // get an Hmax object:
  std::vector<int> scss(5);
  scss[0] = 0; scss[1] = 2; scss[2] = 5; scss[3] = 8; scss[4] = 12;
  std::vector<int> spss(4);
  spss[0] = 4; spss[1] = 6; spss[2] = 9; spss[3] = 12;
  itsHmax = new Hmax(NORI, spss, scss);

}

void ObjRecHMax::start2()
{


}

ObjRecHMax::~ObjRecHMax()
{
}


void ObjRecHMax::train(const Image<PixRGB<byte> > &img, const std::string label)
{

  Image<float> input = luminance(img); //convert to gray
  input = rescale(input, 256, 256); //rescale all images to the same size
  Image<float> features = extractFeatures(input);
  SHOWIMG(features);

}

Image<float> ObjRecHMax::extractFeatures(const Image<float> &input)
{
  return  itsHmax->getC2(input);

}

std::string ObjRecHMax::test(const Image<PixRGB<byte> > &img)
{


  return std::string("Test");
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
