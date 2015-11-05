/*!@file Neuro/EnvSaliencyMap.H */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvSaliencyMap.H $
// $Id: EnvSaliencyMap.H 11346 2009-06-24 22:31:27Z lior $
//

#ifndef NEURO_ENVSALIENCYMAP_H_DEFINED
#define NEURO_ENVSALIENCYMAP_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Image/Point2D.H"
#include "Util/Types.H"

extern const ModelOptionDef OPT_EsmIorHalfLife;
extern const ModelOptionDef OPT_EsmIorStrength;
extern const ModelOptionDef OPT_EsmIorRadius;
extern const ModelOptionDef OPT_EsmInertiaRadius;
extern const ModelOptionDef OPT_EsmInertiaStrength;
extern const ModelOptionDef OPT_EsmInertiaHalfLife;
extern const ModelOptionDef OPT_EsmInertiaShiftThresh;

class EnvSaliencyMap : public ModelComponent
{
public:
  EnvSaliencyMap(OptionManager& mgr);

  virtual ~EnvSaliencyMap();

  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  struct LocInfo
  {
    Point2D<int> lowres_maxpos;
    Point2D<int> fullres_maxpos;
    byte maxval;
  };

  struct State
  {
    Image<byte> salmap;
    Point2D<int> lowres_maxpos;
    Point2D<int> fullres_maxpos;
    std::vector<LocInfo> nMostSalientLoc;
    byte maxval;
  };

  State getSalmap(const Image<byte>& vcxmap,
                  const Point2D<int>& forceWinnerFullres = Point2D<int>(-1,-1));

  const Image<byte>& getInhibmap() const { return itsInhibMap; }

  const Image<float>& getInertiaMap() const { return itsInertiaMap; }

  bool getUseFixed() const { return itsUseFixed.getVal(); }
  double getInertiaHalfLife() const { return itsInertiaHalfLife.getVal(); }
  double getInertiaStrength() const { return itsInertiaStrength.getVal(); }
  double getInertiaRadius() const { return itsInertiaRadius.getVal(); }
  double getInertiaShiftThresh() const { return itsInertiaShiftThresh.getVal(); }
  double getIorHalfLife() const { return itsIorHalfLife.getVal(); }
  double getIorStrength() const { return itsIorStrength.getVal(); }
  double getIorRadius() const { return itsIorRadius.getVal(); }
  double getVcxFlicker() const { return itsVcxFlicker; }

  double getDynamicFactor() const { return itsDynamicFactor; }

  void setUseFixed(bool v) { itsUseFixed.setVal(v); }
  void setInertiaHalfLife(double v) { itsInertiaHalfLife.setVal(v); }
  void setInertiaStrength(double v) { itsInertiaStrength.setVal(v); }
  void setInertiaRadius(double v) { itsInertiaRadius.setVal(v); }
  void setInertiaShiftThresh(double v) { itsInertiaShiftThresh.setVal(v); }
  void setIorHalfLife(double v) { itsIorHalfLife.setVal(v); }
  void setIorStrength(double v) { itsIorStrength.setVal(v); }
  void setIorRadius(double v) { itsIorRadius.setVal(v); }
  void setBiasImg(const Image<byte> &img) { itsBiasImg = img; }
  void resetBiasImg() { itsBiasImg = Image<byte>(); }


private:
  Point2D<int> itsInertiaLoc;
  double itsCurrentInertiaFactor;
  Image<float> itsInertiaMap;
  Image<byte> itsInhibMap;
  OModelParam<bool>   itsUseFixed;
  OModelParam<int>    itsFixedX;
  OModelParam<int>    itsFixedY;
  OModelParam<int>    itsGetNMostSalientLoc;
  OModelParam<int>    itsInternalIORRadius;
  OModelParam<double> itsDynamicFeedback;
  OModelParam<double> itsInertiaHalfLife;
  OModelParam<double> itsInertiaStrength;
  OModelParam<double> itsInertiaRadius;
  OModelParam<double> itsInertiaShiftThresh;
  OModelParam<double> itsIorHalfLife;
  OModelParam<double> itsIorStrength;
  OModelParam<double> itsIorRadius;
  OModelParam<LevelSpec> itsLevelSpec;
  OModelParam<std::string> itsTextLogFile;

  double itsDynamicFactor;

  Image<float> itsVcxMovingAvg;
  Image<byte> itsBiasImg;
  double itsVcxFlicker;
  const double itsVcxMeanDiffCenter;
  const double itsVcxFlickerMin;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_ENVSALIENCYMAP_H_DEFINED
