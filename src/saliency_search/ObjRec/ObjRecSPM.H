/*!@file ObjRec/ObjRecSPM.H Obj Reconition using spatial pyramid matching
 algorithem from Lzebnik, Schmid and Ponce
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRecSPM.H $
// $Id: ObjRecSPM.H 9108 2007-12-30 06:14:30Z rjpeters $
//

#ifndef ObjRecSPM_H_DEFINED
#define ObjRecSPM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Pixels.H"
#include "SIFT/Histogram.H"
#include "Neuro/getSaliency.H"

#include <vector>
#include <limits>

class ObjRecSPM : public ModelComponent
{
public:

  struct SiftKeypoint {
    int x;
    int y;
    std::vector<std::vector<byte> > fv;
  };

  struct Descriptor {
    std::vector<std::vector<Histogram> > featureLevelHist;
    std::vector<SiftKeypoint> siftDescriptors;
  };

  struct Object
  {
    int id;
    std::string name;
    std::vector<Descriptor> model;
  };

  //Constructor
  ObjRecSPM(OptionManager& mgr,
          const std::string& descrName = "ObjRecSPM",
          const std::string& tagName = "ObjRecSPM");
  ~ObjRecSPM();

  //###########################################################

  void start2();

  void train(const Image<PixRGB<byte> > &img, const std::string label);
  void finalizeTraining();
  std::string predict(const Image<PixRGB<byte> > &img);

  Descriptor extractFeatures(const Image<float> &input);
  Descriptor extractSiftFeatures(const Image<float> &input);

  uint getObject(const std::string name);

  //Find the object given the features
  int findObject(const Descriptor &desc);

  double matchKernel(const std::vector<Histogram>& A, const std::vector<Histogram>& B);

  double matchDescriptor(const Descriptor& descA, const Descriptor& descB);

  //For SIft descriptor
  std::vector<std::vector<byte> > getSiftDescriptor(const Image<float> &lum,
      const float x, const float y, const float s);
  void calculateOrientationVector(const float x, const float y, const float s,
      const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV);
  std::vector<std::vector<byte> > createVectorsAndKeypoints(const float x, const float y, const float s,
      const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV);





private:

  int itsNumOri;
  int itsNumScales;
  int itsNumOriArray;
  std::vector<Object> itsObjects;

  ImageSet<float> itsFilters;

  nub::soft_ref<GetSaliency> itsGetSaliency;
  bool itsUseSaliency;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
