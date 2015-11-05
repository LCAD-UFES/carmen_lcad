/*! @file Channels/ContourChannel.C [put description here] */

// ContourChannel.C -- Contour facilitation channel

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; filed July 23, 2001, following provisional applications     //
// No. 60/274,674 filed March 8, 2001 and 60/288,724 filed May 4, 2001).//
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ContourChannel.C $
// $Id: ContourChannel.C 12820 2010-02-11 05:44:51Z itti $
//

#include "Channels/ContourChannel.H"

#include "Channels/ChannelBase.H"
#include "Channels/ChannelOpts.H"
#include "Channels/ComplexChannel.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ImageSet.H"
#include "Image/ImageSetOps.H"
#include "Image/LevelSpec.H"
#include "Image/MathOps.H"
#include "Image/PyramidOps.H"
#include "Image/Range.H"
#include "Image/ShapeOps.H"
#include "Image/fancynorm.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameOstream.H"
#include "Util/Assert.H"
#include "Util/CpuTimer.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/shared_ptr.h"

#include <cmath>
#include <map>
#include <iostream>
#include <fstream>

namespace dummy_namespace_to_avoid_gcc411_bug_ContourChannel_C
{
  // Find the absolute value of the orientation difference between two
  // angles, such that the result is between 0 and 90 degrees (from
  // Braun J, 1996).
  float angDist(float ang1, float ang2)
  {
    while (ang1 < 0.0f) ang1 += 180.0f;
    while (ang2 < 0.0f) ang2 += 180.0f;
    while (ang1 >= 180.0f) ang1 -= 180.0f;
    while (ang2 >= 180.0f) ang2 -= 180.0f;

    float diff = fabs(ang1 - ang2);

    if (diff > 90.0f) diff = 180.0f - diff;

    return diff;
  }

  // Remap v into the range [0.0 - 1.0]
  //   v == 0.0       --> sigmoid ~= 0.12
  //   v == thresh/2  --> sigmoid  = 0.5
  //   v == thresh    --> sigmoid ~= 0.88
  float sigmoid(float v, float thresh)
  {
    return 1.0f / ( 1.0f + exp(2.0f - 4.0f*v/thresh) );
  }

  Image<float> sigmoid(const Image<float>& input, float thresh)
  {
    Image<float> result(input.getDims(), NO_INIT);

    Image<float>::iterator dptr = result.beginw();
    Image<float>::iterator stop = result.endw();
    Image<float>::const_iterator sptr = input.begin();

    while (dptr != stop)
      {
        *dptr++ = sigmoid(*sptr++, thresh);
      }

    return result;
  }

  struct Accum
  {
    double s;
    int c;

    Accum() : s(0.0), c(0) {}

    void add(double d) { s+=d; ++c; }

    int count() const { return c; }
    double sum() const { return s; }
    double mean() const { return c != 0.0 ? s/c : 0.0; }
  };

  inline double sigmoid2(double x)
  {
    return 1.0 / (1.0 + exp(-x));
  }

  inline double excitFiring(double x)
  {
    double result = 2.0*sigmoid2(0.2 * x) - 1;
    return (result < 0.0) ? 0.0 : result;
  }

  inline double PHI(double x)
  {
    return sigmoid2(10.0 * x);
  }

  inline double inhibFiring(double x)
  {
    const double R_THR = 0.5;
    const double R_CUT = 0.2;
    double result = PHI(x - R_THR) - PHI(R_CUT - R_THR);
    return (result < 0.0) ? 0.0 : result;
  }

  float getScaleBias(int scaleNumber)
  {
    switch(scaleNumber)
      {
      case 0: return 1.0f;
      case 1: return 0.9f;
      case 2: return 0.7f;
      }
    return 0.7f;
  }

  void printStats(const Image<float>& img, const char* name)
  {
    float mi, ma, me;
    getMinMaxAvg(img, mi, ma, me);

    LINFO("%s: min %.2f, max %.2f, mean %.2f", name, mi, ma, me);
  }

  Image<float> boxDownSizeClean(const Image<float>& src,
                                const Dims& new_dims,
                                const int filterWidth)
  {
    if (src.getDims() == new_dims) return src;

    ASSERT(new_dims.isNonEmpty());

    Image<float> result = src;

    Image<float> filt(filterWidth, 1, ZEROS);
    filt.clear(1.0f / filterWidth);

    while (result.getWidth() >= new_dims.w() * 2 &&
           result.getHeight() >= new_dims.h() * 2)
      {
        result = decX(sepFilter(result, filt, Image<float>(),
                                CONV_BOUNDARY_CLEAN));
        result = decY(sepFilter(result, Image<float>(), filt,
                                CONV_BOUNDARY_CLEAN));
      }

    return rescaleBilinear(result, new_dims);
  }

  struct SaveSet
  {
  private:
    typedef std::map<std::string, GenericFrame> map_type;
    map_type imgmap;

  public:
    void clear()
    {
      map_type().swap(imgmap);
    }

    void add(const std::string& nm, const GenericFrame& f)
    {
      imgmap.insert(map_type::value_type(nm, f));
    }

    void saveAll(FrameOstream& ofs, const char* savePfx) const
    {
      for (map_type::const_iterator itr = imgmap.begin(), stop = imgmap.end();
           itr != stop; ++itr)
        ofs.writeFrame((*itr).second,
                       sformat("%s.%s", savePfx, (*itr).first.c_str()));
    }
  };
}

using namespace dummy_namespace_to_avoid_gcc411_bug_ContourChannel_C;

// ######################################################################
// ######################################################################
// ##### ContourConnection class
// ######################################################################
// ######################################################################

enum ContourConnectionType
  {
    CC_CXN_CINNIC = 1,
    CC_CXN_BRAUN = 2,
    CC_CXN_BRAUN_NORM = 3
  };

std::string convertToString(const ContourConnectionType val)
{
  switch (val)
    {
    case CC_CXN_CINNIC: return "Cinnic"; break;
    case CC_CXN_BRAUN: return "Braun"; break;
    case CC_CXN_BRAUN_NORM: return "BraunNorm"; break;
    }

  LFATAL("invalid ContourConnectionType '%d'", int(val));
  /* can't happen */ return std::string();
}

void convertFromString(const std::string& str1, ContourConnectionType& val)
{
  const std::string str = toLowerCase(str1);

  if      (str.compare("cinnic") == 0) { val = CC_CXN_CINNIC; }
  else if (str.compare("braun") == 0) { val = CC_CXN_BRAUN; }
  else if (str.compare("braunnorm") == 0)  { val = CC_CXN_BRAUN_NORM; }
  else
    conversion_error::raise<ContourConnectionType>(str1);
}

//! The connection kernel between orientation-tuned units.
class ContourConnection : public ModelComponent
{
public:
  ContourConnection(OptionManager& manager);

  virtual ~ContourConnection();

  float weight(int a, int b, int x, int y) const
  {
    return itsWeights[index(a,b,x,y)];
  }

  const float* weightAXY(int a, int x, int y) const
  {
    return itsWeights + index(a, 0, x, y);
  }

  const float* excitAXY(int a, int x, int y) const
  {
    return itsExcit + index(a, 0, x, y);
  }

  const float* inhibAXY(int a, int x, int y) const
  {
    return itsInhib + index(a, 0, x, y);
  }

  int numAngles() const { return itsNumAngles.getVal(); }

  Dims getDims() const { return itsDims.getVal(); }

  Point2D<int> getCenter() const
  {
    return Point2D<int>((itsDims.getVal().w()-1) / 2,
                   (itsDims.getVal().h()-1) / 2);
  }

  float angle(int i) const { return itsAngles[i]; }

  const float* angles() const { return itsAngles; }

  Image<byte> getSummaryImage() const;

private:
  ContourConnection(const ContourConnection&);
  ContourConnection& operator=(const ContourConnection&);

  virtual void start2();

  virtual void stop1();

  virtual void setupConnectionWeights() = 0;

  void printStats() const;

protected:
  int index(int a, int b, int x, int y) const
  {
    return b + itsNA*(y + itsH*(x + a*itsW));
  }

  float& weight(int a, int b, int x, int y)
  {
    return itsWeights[index(a,b,x,y)];
  }

  OModelParam<Dims> itsDims;
  OModelParam<int> itsNumAngles;

  int itsW;  ///< set to itsDims.getVal().w() during start()
  int itsH;  ///< set to itsDims.getVal().w() during start()
  int itsNA; ///< set to itsNumAngles.getVal() during start()
  int itsNumWeights;
  float* itsAngles;
  float* itsWeights;
  float* itsExcit;
  float* itsInhib;
};

// ######################################################################
// ######################################################################
// ##### ContourConnection implementation
// ######################################################################
// ######################################################################

// Used by: ContourConnection
static const ModelOptionDef OPT_ContourConnectionDims =
  { MODOPT_ARG(Dims), "ContourConnectionDims", &MOC_CHANNEL, OPTEXP_CORE,
    "Dimensions of the individual connection kernels in the contour channel "
    "(corresponds to old ParamMap parameters 'sizeX=13' and 'sizeY=13')",
    "contour-cxn-dims", '\0', "<w>x<h>", "13x13" };

// Used by: ContourConnection
static const ModelOptionDef OPT_ContourConnectionNumOrient =
  { MODOPT_ARG(int), "ContourConnectionNumOrient", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of orientations to use in the contour channel's connection kernel "
    "(corresponds to old ParamMap parameter 'numAngles=12')",
    "contour-num-orient", '\0', "<int>", "12" };

ContourConnection::ContourConnection(OptionManager& mgr)
  :
  ModelComponent(mgr, "Contour Connection Kernel", "ContourConnection"),
  itsDims         ( &OPT_ContourConnectionDims, this ),
  itsNumAngles    ( &OPT_ContourConnectionNumOrient, this),
  itsW            ( -1 ),
  itsH            ( -1 ),
  itsNA           ( -1 ),
  itsNumWeights   ( -1 ),
  itsAngles       ( 0 ),
  itsWeights      ( 0 ),
  itsExcit        ( 0 ),
  itsInhib        ( 0 )
{}

ContourConnection::~ContourConnection()
{}

void ContourConnection::start2()
{
  // width&height must be odd
  if ((itsDims.getVal().w() % 2) == 0)
    LFATAL("Expected --%s to have odd width, but got --%s=%dx%d",
           itsDims.getOptionDef()->longoptname,
           itsDims.getOptionDef()->longoptname,
           itsDims.getVal().w(), itsDims.getVal().h());

  if ((itsDims.getVal().h() % 2) == 0)
    LFATAL("Expected --%s to have odd height, but got --%s=%dx%d",
           itsDims.getOptionDef()->longoptname,
           itsDims.getOptionDef()->longoptname,
           itsDims.getVal().w(), itsDims.getVal().h());

  if (itsNumAngles.getVal() <= 0)
    LFATAL("Expected a positive value for --%s, but got --%s=%d",
           itsNumAngles.getOptionDef()->longoptname,
           itsNumAngles.getOptionDef()->longoptname,
           itsNumAngles.getVal());

  itsW = itsDims.getVal().w();
  itsH = itsDims.getVal().h();
  itsNA = itsNumAngles.getVal();
  itsNumWeights = itsNA*itsNA*itsW*itsH;

  ASSERT(itsAngles == 0);  itsAngles = new float[itsNA];
  ASSERT(itsWeights == 0); itsWeights = new float[itsNumWeights];
  ASSERT(itsExcit == 0);   itsExcit = new float[itsNumWeights];
  ASSERT(itsInhib == 0);   itsInhib = new float[itsNumWeights];

  for (int i = 0; i < itsNA; ++i)
    {
      itsAngles[i] = i * (180.0 / itsNA);
    }

  for (int i = 0; i < itsNumWeights; ++i)
    {
      itsWeights[i] = 0.0f;
    }

  this->setupConnectionWeights();

  printStats();

  ModelComponent::start2();
}

void ContourConnection::stop1()
{
  itsW = -1;
  itsH = -1;
  itsNumWeights = -1;
  delete [] itsAngles;   itsAngles = 0;
  delete [] itsWeights;  itsWeights = 0;
  delete [] itsExcit;    itsExcit = 0;
  delete [] itsInhib;    itsInhib = 0;
}

void ContourConnection::printStats() const
{
  int countAll = 0;
  int countSuppression = 0;
  int countExcitation = 0;
  double sumAll = 0;
  double sumSuppression = 0;
  double sumExcitation = 0;

  ASSERT(itsNumWeights > 0);

  for (int i = 0; i < itsNumWeights; ++i)
    {
      const float w = itsWeights[i];

      ++countAll;
      sumAll += w;

      if (w < 0.0)
        {
          ++countSuppression;
          sumSuppression += w;
        }
      else if (w > 0.0)
        {
          ++countExcitation;
          sumExcitation += w;
        }
    }

  LINFO("%+8.2f (%5d) all", sumAll, countAll);
  LINFO("%+8.2f (%5d) suppression", sumSuppression, countSuppression);
  LINFO("%+8.2f (%5d) excitation", sumExcitation, countExcitation);
}

Image<byte> ContourConnection::getSummaryImage() const
{
  const int nori = itsNumAngles.getVal();

  Image<float> weights[(nori+1)*(nori+1)];

  weights[0] = Image<float>(itsDims.getVal() * 4, ZEROS);

  int c = 1;

  for (int b = 0; b < nori; ++b)
    {
      Image<float> tmp(itsDims.getVal() * 4, ZEROS);
      writeText(tmp, Point2D<int>(0,0),
                sformat("%.1f", angle(b)).c_str());
      inplaceNormalize(tmp, 0.0f, 1.0f);
      weights[c++] = tmp;
    }

  for (int angle_a = 0; angle_a < nori; ++angle_a)
    {
      Image<float> foo(itsDims.getVal() * 4, ZEROS);
      writeText(foo, Point2D<int>(0,0),
                sformat("%.1f", angle(angle_a)).c_str());
      inplaceNormalize(foo, 0.0f, 1.0f);
      weights[c++] = foo;

      for (int angle_b = 0; angle_b < nori; ++angle_b)
        {
          Image<float> tmp(itsDims.getVal(), NO_INIT);

          for (int y = 0; y < itsH; ++y)
            for (int x = 0; x < itsW; ++x)
              tmp.setVal(x, y, weight(angle_a, angle_b, x, y));

          weights[c++] = intXY(intXY(tmp, true), true);
        }
    }

  Image<float> img = concatArray(&weights[0],
                                 (nori+1)*(nori+1),
                                 nori+1);
  inplaceNormalize(img, 0.0f, 255.0f);

  return Image<byte>(img);
}

// ######################################################################
// ######################################################################
// ##### ContourConnectionCinnic class
// ######################################################################
// ######################################################################

// Based on CINNIC kernel (Nathan Mundhenk)
class ContourConnectionCinnic : public ContourConnection
{
public:
  ContourConnectionCinnic(OptionManager& manager);

  virtual ~ContourConnectionCinnic() {}

private:
  virtual void setupConnectionWeights();

  OModelParam<float> itsAngleDropOff;
  OModelParam<float> itsAngleSuppress;
  OModelParam<float> itsSupMult;
  OModelParam<float> itsOrthMult;
  OModelParam<float> itsOrthA;
  OModelParam<float> itsOrthB;
  OModelParam<float> itsNeuronExSize;
  OModelParam<float> itsNeuronSupSize;
  OModelParam<float> itsNeuronSupStart;
  OModelParam<float> itsNeuronOrthSize;
  OModelParam<float> itsValueCutoff;
  OModelParam<float> itsColinearDiff;
};

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicAngleDropOff =
  { MODOPT_ARG(float), "CinnicAngleDropOff", &MOC_CHANNEL, OPTEXP_CORE,
    "AngleDropOff parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'AngleDropOff=35.0')",
    "cinnic-angle-dropoff", '\0', "<float>", "35.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicAngleSuppress =
  { MODOPT_ARG(float), "CinnicAngleSuppress", &MOC_CHANNEL, OPTEXP_CORE,
    "AngleSuppress parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'AngleSuppress=60.0')",
    "cinnic-angle-suppress", '\0', "<float>", "60.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicSupMult =
  { MODOPT_ARG(float), "CinnicSupMult", &MOC_CHANNEL, OPTEXP_CORE,
    "SupMult parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'SupMult=1.5')",
    "cinnic-sup-mult", '\0', "<float>", "1.5" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicOrthMult =
  { MODOPT_ARG(float), "CinnicOrthMult", &MOC_CHANNEL, OPTEXP_CORE,
    "OrthMult parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'OrthMult=0.0')",
    "cinnic-orth-mult", '\0', "<float>", "0.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicAngleOrthRangeA =
  { MODOPT_ARG(float), "CinnicAngleOrthRangeA", &MOC_CHANNEL, OPTEXP_CORE,
    "AngleOrthRangeA parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'AngleOrthRangeA=45.0')",
    "cinnic-angle-orth-range-a", '\0', "<float>", "45.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicAngleOrthRangeB =
  { MODOPT_ARG(float), "CinnicAngleOrthRangeB", &MOC_CHANNEL, OPTEXP_CORE,
    "AngleOrthRangeB parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'AngleOrthRangeB=35.0')",
    "cinnic-angle-orth-range-b", '\0', "<float>", "35.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicNeuronExSize =
  { MODOPT_ARG(float), "CinnicNeuronExSize", &MOC_CHANNEL, OPTEXP_CORE,
    "NeuronExSize parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'NeuronExSize=12.0')",
    "cinnic-neuron-ex-size", '\0', "<float>", "12.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicNeuronSupSize =
  { MODOPT_ARG(float), "CinnicNeuronSupSize", &MOC_CHANNEL, OPTEXP_CORE,
    "NeuronSupSize parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'NeuronSupSize=10.0')",
    "cinnic-neuron-sup-size", '\0', "<float>", "10.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicNeuronSupStart =
  { MODOPT_ARG(float), "CinnicNeuronSupStart", &MOC_CHANNEL, OPTEXP_CORE,
    "NeuronSupStart parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'NeuronSupStart=1.0')",
    "cinnic-neuron-sup-start", '\0', "<float>", "1.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicNeuronOrthSize =
  { MODOPT_ARG(float), "CinnicNeuronOrthSize", &MOC_CHANNEL, OPTEXP_CORE,
    "NeuronOrthSize parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'NeuronOrthSize=12.0')",
    "cinnic-neuron-orth-size", '\0', "<float>", "12.0" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicValueCutoff =
  { MODOPT_ARG(float), "CinnicValueCutoff", &MOC_CHANNEL, OPTEXP_CORE,
    "valueCutOff parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'valueCutOff=0.0001')",
    "cinnic-value-cutoff", '\0', "<float>", "0.0001" };

// Used by: ContourConnectionCinnic
static const ModelOptionDef OPT_CinnicColinearDiff =
  { MODOPT_ARG(float), "CinnicColinearDiff", &MOC_CHANNEL, OPTEXP_CORE,
    "ColinearDiff parameter for the cinnic kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'CoLinearDiff=16.0')",
    "cinnic-colinear-diff", '\0', "<float>", "16.0" };

ContourConnectionCinnic::
ContourConnectionCinnic(OptionManager& manager)
  :
  ContourConnection(manager),
  itsAngleDropOff   (&OPT_CinnicAngleDropOff, this),
  itsAngleSuppress  (&OPT_CinnicAngleSuppress, this),
  itsSupMult        (&OPT_CinnicSupMult, this),
  itsOrthMult       (&OPT_CinnicOrthMult, this),
  itsOrthA          (&OPT_CinnicAngleOrthRangeA, this),
  itsOrthB          (&OPT_CinnicAngleOrthRangeB, this),
  itsNeuronExSize   (&OPT_CinnicNeuronExSize, this),
  itsNeuronSupSize  (&OPT_CinnicNeuronSupSize, this),
  itsNeuronSupStart (&OPT_CinnicNeuronSupStart, this),
  itsNeuronOrthSize (&OPT_CinnicNeuronOrthSize, this),
  itsValueCutoff    (&OPT_CinnicValueCutoff, this),
  itsColinearDiff   (&OPT_CinnicColinearDiff, this)
{}

void ContourConnectionCinnic::setupConnectionWeights()
{
  ASSERT(itsValueCutoff.getVal() >= 0.0f);

  ASSERT(itsW > 0);
  ASSERT(itsH > 0);

  const int nori = itsNumAngles.getVal();

  const Point2D<int> center = this->getCenter();

  for (int x = 0; x < itsW; ++x) {
    for (int y = 0; y < itsH; ++y) {

      const float distance =
        sqrt( (center.i-x)*(center.i-x) + (center.j-y)*(center.j-y) );

      if (distance == 0) // center is always 0
        continue;

      // We negate the y component to account for the difference
      // between "math" coordinates where y runs from bottom to top,
      // and "image coordinates", where y runs from top to bottom
      const float connectingAngle = (180/M_PI) * atan2(center.j-y, x-center.i);

      for (int a = 0; a < nori; ++a) {

        // Find the angle between the line pointing the the other unit
        // and the alignment of this unit
        const float angDistAC = angDist(connectingAngle, angle(a));

        for (int b = 0; b < nori; ++b) {

          const float angDistAB = angDist(angle(a), angle(b));

          // find if orthogonal values need to be calculated and
          // calculate their value
          const bool is_orth =
            (angDistAC > 90.0f-itsOrthA.getVal())
            &&
            (angDistAB > 90.0f-itsOrthB.getVal());

          if (is_orth)
            {
              if (itsOrthMult.getVal() <= 0.0f)
                continue;

              const float alphaAngleMod = 1.0f - (90.0f-angDistAC)/itsOrthA.getVal();
              ASSERT(alphaAngleMod >= 0);

              const float distanceMod = 1.0f - distance/(itsNeuronOrthSize.getVal()/2);

              if (distanceMod <= 0.0f)
                continue;

              weight(a, b, x, y) =
                itsOrthMult.getVal() * alphaAngleMod * distanceMod;
            }
          else if (angDistAC > itsAngleDropOff.getVal()) // "butterfly" suppression
            {
              ASSERT(itsAngleDropOff.getVal() > (90.0f - itsAngleSuppress.getVal()));

              // Make sure we are colinear "enough"
              if (angDistAB >= itsColinearDiff.getVal())
                continue;

              // Make sure we are in the right distance range to allow
              // suppression
              if ((distance < itsNeuronSupStart.getVal()) ||
                  (distance >= (itsNeuronSupSize.getVal()/2)))
                continue;

              const float distanceMod = 1.0f - distance/(itsNeuronSupSize.getVal()/2);

              ASSERT(distanceMod > 0.0f);

              const float alphaAngleMod =
                1.0f - (90.0f-angDistAC)/itsAngleSuppress.getVal();

              ASSERT(alphaAngleMod >= 0.0f);

              const float angleMod =
                alphaAngleMod * (1.0f - angDistAB / itsColinearDiff.getVal());

              weight(a, b, x, y) =
                -1.0f * itsSupMult.getVal() * ((angleMod + distanceMod)/2);
            }
          else // "butterfly" excitation
            {
              // Find the angle between the line pointing to the other
              // unit and the alignment of that unit.
              const float angDistBC = angDist(connectingAngle, angle(b));

              const float betaAngleMod = 1.0f - angDistBC/itsAngleDropOff.getVal();

              if (betaAngleMod <= itsValueCutoff.getVal())
                continue;

              const float alphaAngleMod = 1.0f - angDistAC/itsAngleDropOff.getVal();
              ASSERT(alphaAngleMod >= 0.0f);

              // Alpha/beta combo for excitation.
              const float angleMod = alphaAngleMod * betaAngleMod;

              if (angleMod <= itsValueCutoff.getVal())
                continue;

              const float distanceMod = 1.0f - distance/(itsNeuronExSize.getVal()/2);

              if (distanceMod <= itsValueCutoff.getVal())
                continue;

              weight(a, b, x, y) = (angleMod + distanceMod)/2;
            }
        }
      }
    }
  }
}

// ######################################################################
// ######################################################################
// ##### ContourConnectionBraun class
// ######################################################################
// ######################################################################

// based on Achim Braun's unpublished "Contour integration in striate
// cortex: a model" (submitted to Neural Computation (1996))
class ContourConnectionBraun : public ContourConnection
{
public:
  ContourConnectionBraun(OptionManager& manager,
                         bool doOneNormalize);

  virtual ~ContourConnectionBraun() {}

private:
  virtual void setupConnectionWeights();

  OModelParam<float> itsPeakDistance;
  OModelParam<float> itsExcitStrength;
  OModelParam<float> itsInhibStrength;
  OModelParam<float> itsBeta;
  OModelParam<bool>  itsOrthoIntxn;

  const bool itsDoOneNormalize;
};

// Used by: ContourConnectionBraun
static const ModelOptionDef OPT_ContourConnectionPeakDistance =
  { MODOPT_ARG(float), "ContourConnectionPeakDistance", &MOC_CHANNEL, OPTEXP_CORE,
    "PeakDistance parameter for the Braun kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'peakDistance=2.0')",
    "contour-cxn-peak-distance", '\0', "<float>", "2.0" };

// Used by: ContourConnectionBraun
static const ModelOptionDef OPT_ContourConnectionExcitStrength =
  { MODOPT_ARG(float), "ContourConnectionExcitStrength", &MOC_CHANNEL, OPTEXP_CORE,
    "ExcitStrength parameter for the Braun kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'excitStrength=1.7')",
    "contour-cxn-excit-strength", '\0', "<float>", "1.7" };

// Used by: ContourConnectionBraun
static const ModelOptionDef OPT_ContourConnectionInhibStrength =
  { MODOPT_ARG(float), "ContourConnectionInhibStrength", &MOC_CHANNEL, OPTEXP_CORE,
    "InhibStrength parameter for the Braun kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'inhibStrength=-0.3')",
    "contour-cxn-inhib-strength", '\0', "<float>", "-0.3" };

// Used by: ContourConnectionBraun
static const ModelOptionDef OPT_ContourConnectionBeta =
  { MODOPT_ARG(float), "ContourConnectionBeta", &MOC_CHANNEL, OPTEXP_CORE,
    "Beta parameter for the Braun kernel in the contour channel "
    "(corresponds to old ParamMap parameter 'beta=0.07')",
    "contour-cxn-beta", '\0', "<float>", "0.07" };

// Used by: ContourConnectionBraun
static const ModelOptionDef OPT_ContourConnectionOrthoIntxn =
  { MODOPT_FLAG, "ContourConnectionOrthoIntxn", &MOC_CHANNEL, OPTEXP_CORE,
    "Whether to include (inhibitory) interactions between (near-)orthogonal "
    "orientations in the contour channel "
    "(corresponds to old ParamMap parameter 'orthoIntxn=0')",
    "contour-ortho-interaction", '\0', "", "false" };

ContourConnectionBraun::
ContourConnectionBraun(OptionManager& manager,
                       bool doOneNormalize)
  :
  ContourConnection(manager),
  itsPeakDistance(&OPT_ContourConnectionPeakDistance, this),
  itsExcitStrength(&OPT_ContourConnectionExcitStrength, this),
  itsInhibStrength(&OPT_ContourConnectionInhibStrength, this),
  itsBeta(&OPT_ContourConnectionBeta, this),
  itsOrthoIntxn(&OPT_ContourConnectionOrthoIntxn, this),
  itsDoOneNormalize(doOneNormalize)
{}

void ContourConnectionBraun::setupConnectionWeights()
{
  ASSERT(itsW > 0);
  ASSERT(itsH > 0);

  const int nori = itsNumAngles.getVal();
  const Point2D<int> center = this->getCenter();

  for (int x = 0; x < itsW; ++x) {
    for (int y = 0; y < itsH; ++y) {

      const float distance =
        sqrt( (center.i-x)*(center.i-x) + (center.j-y)*(center.j-y) );

      const float distRatio = distance/itsPeakDistance.getVal();

      const float rho = pow(distRatio * exp(1.0 - distRatio), 2.0);

      const float connectingAngle = (180/M_PI) * atan2(center.j-y, x-center.i);

      for (int a = 0; a < nori; ++a) {

        const float angDistAC = angDist(angle(a), connectingAngle);

        for (int b = 0; b < nori; ++b) {

          const float angDistAB = angDist(angle(a), angle(b));
          const float angDistBC = angDist(angle(b), connectingAngle);

          const float avgAngle = fabs(angle(a)-angle(b)) > 90.0f
            ? (angle(a)+angle(b)-180.0f) / 2.0f
            : (angle(a)+angle(b)) / 2.0f;

          const bool colinear =
            angDistAB < 45.0f && angDistAC < 45.0f && angDistBC < 45.0f;

          const float psi = colinear
            ? exp(-itsBeta.getVal() * angDist(avgAngle, connectingAngle))
            : 0.0f;

          const float excitPart = rho * psi * itsExcitStrength.getVal();

          const bool haveInhib = (angDistAB < 45.0f || itsOrthoIntxn.getVal());

          const float inhibPart =
            haveInhib
            ? rho * itsInhibStrength.getVal()
            : 0.0f;

          weight(a, b, x, y) = excitPart + inhibPart;
          itsInhib[index(a,b,x,y)] = inhibPart;
          itsExcit[index(a,b,x,y)] = excitPart;
        }
      }
    }
  }

  if (itsDoOneNormalize)
    {
      float maxw = 0.0f;

      ASSERT(itsNumWeights > 0);

      for (int i = 0; i < itsNumWeights; ++i)
        {
          if (fabs(itsWeights[i]) > maxw) maxw = fabs(itsWeights[i]);
        }

      if (maxw > 0.0f)
        {
          for (int i = 0; i < itsNumWeights; ++i)
            {
              itsWeights[i] /= maxw;
            }
        }
    }
}

// ######################################################################
// ######################################################################
// ##### ContourLayer class
// ######################################################################
// ######################################################################

//! Represents a single scale band in which contour facilitation is run.
class ContourLayer : public ModelComponent
{
public:
  ContourLayer(OptionManager& mgr,
               const std::string& descrName,
               const std::string& tagName);

  virtual ~ContourLayer();

  Image<float> compute(const ImageSet<float>& input,
                       const ContourConnection& connection,
                       const std::string& saveSuffix,
                       int normFlags,
                       SaveSet& save,
                       const unsigned int niter,
                       const Dims& origdims);

protected:
  OModelParam<bool> itsSaveLayerOutput;
  OModelParam<bool> itsSaveLayerDetails;

private:
  virtual void startNewInput(const ImageSet<float>& input,
                             const ContourConnection& connection) = 0;

  // Returns this iteration's saliency map
  virtual Image<float> iterate(const ContourConnection& connection,
                               const char* saveSuffix,
                               int normFlags,
                               SaveSet& save) = 0;
};

// Used by: ContourLayer
static const ModelOptionDef OPT_ContourLayerSaveOutput =
  { MODOPT_FLAG, "ContourLayerSaveOutput", &MOC_CHANNEL, OPTEXP_SAVE,
    "Whether to save per-iteration output from individual scale bands "
    "in the contour channel",
    "save-contour-layer-output", '\0', "", "false" };

// Used by: ContourLayer
static const ModelOptionDef OPT_ContourLayerSaveDetails =
  { MODOPT_FLAG, "ContourLayerSaveDetails", &MOC_CHANNEL, OPTEXP_SAVE,
    "Whether to save detailed intermediate maps from individual scale "
    "bands in the contour channel",
    "save-contour-layer-details", '\0', "", "false" };

ContourLayer::ContourLayer(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName),
  itsSaveLayerOutput(&OPT_ContourLayerSaveOutput, this),
  itsSaveLayerDetails(&OPT_ContourLayerSaveDetails, this)
{}

ContourLayer::~ContourLayer() {}

Image<float> ContourLayer::compute(const ImageSet<float>& input,
                                   const ContourConnection& connection,
                                   const std::string& saveSuffix,
                                   int normFlags,
                                   SaveSet& save,
                                   const unsigned int niter,
                                   const Dims& origdims)
{
  this->startNewInput(input, connection);

  Image<float> salmap;

  for (unsigned int iter = 0; iter < niter; ++iter)
    {
      LINFO("scale %s, iteration %u / %u ... ",
            saveSuffix.c_str(), iter+1, niter);

      const std::string sfx2 =
        sformat("%s.i-%02u", saveSuffix.c_str(), iter);

      salmap = this->iterate(connection, sfx2.c_str(), normFlags,
                             save);

      if (itsSaveLayerOutput.getVal())
        save.add(sformat("potential.%s", sfx2.c_str()),
                 GenericFrame(rescale(salmap, origdims),
                              normFlags));
    }

  return salmap;
}

// ######################################################################
// ######################################################################
// ##### ContourLayerDynamic class
// ######################################################################
// ######################################################################

//! Based on Achim Braun's unpublished Neural Computation paper.
class ContourLayerDynamic : public ContourLayer
{
private:
  OModelParam<float>    itsTimestep;
  OModelParam<float>    itsFilterThresh;

  ImageSet<float>       itsInput;
  ImageSet<float>       itsNeuronsCur;
  ImageSet<float>       itsNeuronsTempSpace;

public:
  ContourLayerDynamic(OptionManager& mgr);

  virtual ~ContourLayerDynamic();

private:
  virtual void startNewInput(const ImageSet<float>& input,
                             const ContourConnection& connection);

  virtual Image<float> iterate(const ContourConnection& connection,
                               const char* saveSuffix,
                               int normFlags,
                               SaveSet& save);
};

// ######################################################################
// ######################################################################
// ##### ContourLayerDynamic implementation
// ######################################################################
// ######################################################################

// Used by: ContourLayerDynamic
static const ModelOptionDef OPT_ContourLayerDynamicTimestep =
  { MODOPT_ARG(float), "ContourLayerDynamicTimestep", &MOC_CHANNEL, OPTEXP_CORE,
    "Timestep for the contour channel with --contour-dynamics-type=Dynamic "
    "(corresponds to old ParamMap parameter 'timestep=0.5')",
    "contour-layer-timestep", '\0', "<float>", "0.5" };

// Used by: ContourLayerDynamic
static const ModelOptionDef OPT_ContourLayerDynamicFilterThresh =
  { MODOPT_ARG(float), "ContourLayerDynamicFilterThresh", &MOC_CHANNEL, OPTEXP_CORE,
    "Timestep for the contour channel with --contour-dynamics-type=Dynamic "
    "(corresponds to old ParamMap parameter 'filterThresh=0.0001')",
    "contour-layer-filter-thresh", '\0', "<float>", "0.0001" };

ContourLayerDynamic::ContourLayerDynamic(OptionManager& mgr)
  :
  ContourLayer(mgr, "Dynamic Contour Layer", "ContourLayerDynamic"),
  itsTimestep           ( &OPT_ContourLayerDynamicTimestep, this),
  itsFilterThresh       ( &OPT_ContourLayerDynamicFilterThresh, this),
  itsInput              ( ),
  itsNeuronsCur         ( ),
  itsNeuronsTempSpace   ( )
{}

ContourLayerDynamic::~ContourLayerDynamic()
{}

void ContourLayerDynamic::startNewInput(const ImageSet<float>& input,
                                        const ContourConnection& cxn)
{
  ASSERT(rangeOf(input).max() == 1.0f);

  itsInput = input;
  itsNeuronsCur = ImageSet<float>(cxn.numAngles(), input[0].getDims());
  itsNeuronsTempSpace = ImageSet<float>();
}

Image<float> ContourLayerDynamic::iterate(const ContourConnection& cxn,
                                          const char* saveSuffix,
                                          int normFlags,
                                          SaveSet& save)
{
  ASSERT(cxn.started());

  const Dims dims = itsInput[0].getDims();

  // fetch these values once and store them so that we don't need
  // repeated getVal() calls within inner loops below
  const int nori = cxn.numAngles();
  const float timestep = itsTimestep.getVal();
  const float fthresh = itsFilterThresh.getVal();
  const Dims kerndims = cxn.getDims();
  const Point2D<int> kerncenter = cxn.getCenter();

  itsNeuronsTempSpace = itsNeuronsCur;
  doAddWeighted(itsNeuronsTempSpace, itsInput, timestep);

  itsNeuronsTempSpace *= (1.0f - timestep); // electrical leak

  // We set up these arrays of iterators ahead of time so that we can avoid
  // more costly image-indexing operations inside the inner loop below.
  Image<float>::const_iterator input_itrs[nori];
  Image<float>::const_iterator prev_charge_itrs[nori];
  Image<float>::iterator       charge_itrs[nori];

  Image<float> exc[nori];
  Image<float> inh[nori];
  Image<float> chg[nori];

  for (int b = 0; b < nori; ++b)
    {
      input_itrs[b] = itsInput[b].begin();
      prev_charge_itrs[b] = itsNeuronsCur[b].begin();
      charge_itrs[b] = itsNeuronsTempSpace[b].beginw();

      exc[b] = Image<float>(itsInput[b].getDims(), ZEROS);
      inh[b] = Image<float>(itsInput[b].getDims(), ZEROS);
      chg[b] = Image<float>(itsInput[b].getDims(), ZEROS);
    }

  // Do the three-dimensional x/y/angle convolution

  for (int angle_a = 0; angle_a < nori; ++angle_a)
    {
      Image<float>& charges_a = itsNeuronsCur[angle_a];

      Image<float>::const_iterator input_a_itr = itsInput[angle_a].begin();

      Image<float>::const_iterator charges_a_itr = charges_a.begin();

      for (int a_y = 0; a_y < dims.h(); ++a_y)
        {
          for (int a_x = 0;
               a_x < dims.w();
               ++a_x, ++input_a_itr, ++charges_a_itr)
            {
              const float input_a = *input_a_itr;

              if (input_a <= fthresh)
                continue;

              const float charge_a = *charges_a_itr;

              const float excit_a = excitFiring(charge_a);

              const float inhib_a = inhibFiring(excit_a);

              chg[angle_a].setVal(a_x, a_y, charge_a);
              exc[angle_a].setVal(a_x, a_y, excit_a);
              inh[angle_a].setVal(a_x, a_y, inhib_a);

              for (int rf_x = 0; rf_x < kerndims.w(); ++rf_x)
                {
                  // Current position plus its field - center
                  const int b_x = a_x + (rf_x-kerncenter.i);

                  if (b_x < 0 || b_x >= dims.w())
                    continue;

                  for (int rf_y = 0; rf_y < kerndims.h(); ++rf_y)
                    {
                      const int b_y = a_y - (rf_y-kerncenter.j);

                      if (b_y < 0 || b_y >= dims.h()) // stay inside of image
                        continue;

                      const int offset = b_x + b_y*dims.w();

                      const float* weights_ptr =
                        cxn.weightAXY(angle_a, rf_x, rf_y);

                      const float* const w_excit_ptr =
                        cxn.excitAXY(angle_a, rf_x, rf_y);

                      const float* const w_inhib_ptr =
                        cxn.inhibAXY(angle_a, rf_x, rf_y);

                      for (int angle_b = 0; angle_b < nori; ++angle_b)
                        {
                          const float cxn_strength = *weights_ptr++;

                          if (cxn_strength == 0.0f)
                            continue;

                          const float input_b = input_itrs[angle_b][offset];

                          const float charge_b = prev_charge_itrs[angle_b][offset];

                          // excitation is modulated by fast plasticity
                          const float excit =
                            excit_a * input_b * w_excit_ptr[angle_b];

                          // suppression is modulated by group activity
                          const float inhib =
                            inhib_a * charge_b * w_inhib_ptr[angle_b];

                          // add energy to salmap
                          charge_itrs[angle_b][offset] +=
                            timestep * (excit + inhib);
                        }
                    }
                }
            }
        }
    }

  if (itsSaveLayerDetails.getVal())
    {
      const Image<float> all_chg = concatArray(&chg[0], nori, 4);
      const Image<float> all_exc = concatArray(&exc[0], nori, 4);
      const Image<float> all_inh = concatArray(&inh[0], nori, 4);

      save.add(sformat("all_chg.%s", saveSuffix), GenericFrame(all_chg, normFlags));
      save.add(sformat("all_exc.%s", saveSuffix), GenericFrame(all_exc, normFlags));
      save.add(sformat("all_inh.%s", saveSuffix), GenericFrame(all_inh, normFlags));
    }

  itsNeuronsCur.swap(itsNeuronsTempSpace);

  const Image<float> activation = sum(itsNeuronsCur);

  printStats(activation, "activation");

  return activation;
}

// ######################################################################
// ######################################################################
// ##### ContourLayerStatic class
// ######################################################################
// ######################################################################

//! Based on CINNIC (Nathan Mundhenk).
class ContourLayerStatic : public ContourLayer
{
private:
  OModelParam<float>    itsFastPlast;
  OModelParam<float>    itsGroupBottom;
  OModelParam<float>    itsGroupTop;
  OModelParam<int>      itsGroupSize;
  OModelParam<float>    itsLeak;
  OModelParam<float>    itsSuppressionAdd;
  OModelParam<float>    itsSuppressionSub;
  OModelParam<float>    itsSigmoidThresh;
  OModelParam<float>    itsFilterThresh;

  ImageSet<float>       itsInput;

  Image<float>          itsSalmap;

  Image<float>          itsEnergyMap;

  ImageSet<float>       itsNeuronsCur;
  ImageSet<float>       itsNeuronsTempSpace;

  Image<float>          itsGroupModMap;

  void convolveAll(const ContourConnection& cxn);

public:
  ContourLayerStatic(OptionManager& mgr);

  virtual ~ContourLayerStatic();

private:
  virtual void startNewInput(const ImageSet<float>& input,
                             const ContourConnection& connection);

  virtual Image<float> iterate(const ContourConnection& connection,
                               const char* saveSuffix,
                               int normFlags,
                               SaveSet& save);
};

// ######################################################################
// ######################################################################
// ##### ContourLayerStatic implementation
// ######################################################################
// ######################################################################

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerFastPlast =
  { MODOPT_ARG(float), "CinnicLayerFastPlast", &MOC_CHANNEL, OPTEXP_CORE,
    "Fast-plasticity strength under --contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'fastPlast=7.2077')",
    "cinnic-fast-plast", '\0', "<float>", "7.2077" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerGroupBottom =
  { MODOPT_ARG(float), "CinnicLayerGroupBottom", &MOC_CHANNEL, OPTEXP_CORE,
    "Threshold beneath which group suppression is removed, under "
    "--contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'groupAvgBottom=0.0')",
    "cinnic-group-bottom", '\0', "<float>", "0.0" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerGroupTop =
  { MODOPT_ARG(float), "CinnicLayerGroupTop", &MOC_CHANNEL, OPTEXP_CORE,
    "Threshold above which group suppression is added, under "
    "--contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'groupAvgTop=0.03125')",
    "cinnic-group-top", '\0', "<float>", "0.03125" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerGroupSize =
  { MODOPT_ARG(int), "CinnicLayerGroupSize", &MOC_CHANNEL, OPTEXP_CORE,
    "Width of low-pass kernel used to compute group suppression, under "
    "--contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'groupSize=15')",
    "cinnic-group-size", '\0', "<int>", "15" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerLeak =
  { MODOPT_ARG(float), "CinnicLayerLeak", &MOC_CHANNEL, OPTEXP_CORE,
    "Amount of global leak during each iteration under "
    "--contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'leakFactor=0.5')",
    "cinnic-leak", '\0', "<float>", "0.5" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerSuppressionAdd =
  { MODOPT_ARG(float), "CinnicLayerSuppressionAdd", &MOC_CHANNEL, OPTEXP_CORE,
    "Scale factor for group suppression added where the rate of change "
    "exceeds --cinnic-group-top, --contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'suppressionAdd=1.0')",
    "cinnic-suppression-add", '\0', "<float>", "1.0" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerSuppressionSub =
  { MODOPT_ARG(float), "CinnicLayerSuppressionSub", &MOC_CHANNEL, OPTEXP_CORE,
    "Scale factor for group suppression removed where the rate of change "
    "is less than --cinnic-group-bottom, --contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'suppressionSub=1.0')",
    "cinnic-suppression-sub", '\0', "<float>", "1.0" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerSigmoidThresh =
  { MODOPT_ARG(float), "CinnicLayerSigmoidThresh", &MOC_CHANNEL, OPTEXP_CORE,
    "Threshold for the output sigmoid nonlinearity, under "
    "--contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'sigmoidThresh=9.0')",
    "cinnic-sigmoid-thresh", '\0', "<float>", "9.0" };

// Used by: ContourLayerStatic
static const ModelOptionDef OPT_CinnicLayerFilterThresh =
  { MODOPT_ARG(float), "CinnicLayerFilterThresh", &MOC_CHANNEL, OPTEXP_CORE,
    "Filter values less than this threshold will be forced to 0, under "
    "--contour-dynamics-type=Static "
    "(corresponds to old ParamMap parameter 'filterThresh=0.0001')",
    "cinnic-filter-thresh", '\0', "<float>", "0.0001" };

ContourLayerStatic::ContourLayerStatic(OptionManager& mgr)
  :
  ContourLayer(mgr, "Static Contour Layer", "ContourLayerStatic"),
  itsFastPlast          ( &OPT_CinnicLayerFastPlast, this ),
  itsGroupBottom        ( &OPT_CinnicLayerGroupBottom, this ),
  itsGroupTop           ( &OPT_CinnicLayerGroupTop, this ),
  itsGroupSize          ( &OPT_CinnicLayerGroupSize, this ),
  itsLeak               ( &OPT_CinnicLayerLeak, this ),
  itsSuppressionAdd     ( &OPT_CinnicLayerSuppressionAdd, this ),
  itsSuppressionSub     ( &OPT_CinnicLayerSuppressionSub, this ),
  itsSigmoidThresh      ( &OPT_CinnicLayerSigmoidThresh, this ),
  itsFilterThresh       ( &OPT_CinnicLayerFilterThresh, this ),
  itsInput              ( ),
  itsSalmap             ( ),
  itsEnergyMap          ( ),
  itsNeuronsCur         ( ),
  itsNeuronsTempSpace   ( ),
  itsGroupModMap        ( )
{}

ContourLayerStatic::~ContourLayerStatic()
{}

void ContourLayerStatic::startNewInput(const ImageSet<float>& input,
                                       const ContourConnection& cxn)
{
  ASSERT(rangeOf(input).max() == 1.0f);

  itsInput = input;
  itsSalmap = Image<float>(input[0].getDims(), ZEROS);
  itsEnergyMap = Image<float>(input[0].getDims(), ZEROS);
  itsNeuronsCur = ImageSet<float>(cxn.numAngles(), input[0].getDims());
  itsNeuronsTempSpace = ImageSet<float>(cxn.numAngles(), input[0].getDims());
  itsGroupModMap = Image<float>(input[0].getDims(), ZEROS);
  itsGroupModMap.clear(1.0f);
}

void ContourLayerStatic::convolveAll(const ContourConnection& cxn)
{
  ASSERT(cxn.started());

  const Dims dims = itsInput[0].getDims();

  doClear(itsNeuronsTempSpace, 0.0f);

  // For monitoring fast-plasticity behavior
  Accum fpTotal;
  Accum fpHi;
  Accum fpLo;

  // fetch these values once and store them so that we don't need
  // repeated getVal() calls during inner loops below
  const int nori = cxn.numAngles();
  const float cfgFastPlast = itsFastPlast.getVal();
  const float cfgFilterThresh = itsFilterThresh.getVal();
  const Dims kerndims = cxn.getDims();
  const Point2D<int> kerncenter = cxn.getCenter();

  // We set up these arrays of iterators ahead of time so that we can avoid
  // more costly image-indexing operations inside the inner loop below.
  Image<float>::iterator       charges_b_itrs[nori];
  Image<float>::const_iterator input_b_itrs[nori];

  for (int b = 0; b < nori; ++b)
    {
      charges_b_itrs[b] = itsNeuronsTempSpace[b].beginw();
      input_b_itrs[b] = itsInput[b].begin();
    }

  // For monitoring execution speed
  CpuTimer t1, t2;

  // For monitoring progress through the convolution
  int c = 0;
  const int maxc = nori * dims.h() * dims.w();

  // Here's the actual three-dimensional x/y/angle convolution

  for (int angle_a = 0; angle_a < nori; ++angle_a)
    {
      Image<float>& charges_a = itsNeuronsCur[angle_a];

      Image<float>::const_iterator input_a_itr = itsInput[angle_a].begin();

      Image<float>::const_iterator charges_a_itr = charges_a.begin();

      for (int a_y = 0; a_y < dims.h(); ++a_y)
        {
          for (int a_x = 0;
               a_x < dims.w();
               ++a_x, ++input_a_itr, ++charges_a_itr)
            {
              const float input_a = *input_a_itr;

              if (input_a <= cfgFilterThresh)
                continue;

              const float charge_a = *charges_a_itr;

              // Fast plasticity based on potential at sending neuron
              float fastPlastMod = charge_a * cfgFastPlast;
              fpTotal.add(fastPlastMod);
              if      (fastPlastMod < 1.0f) { fpLo.add(fastPlastMod); fastPlastMod = 1.0f; }
              else if (fastPlastMod > 5.0f) { fpHi.add(fastPlastMod); fastPlastMod = 5.0f; }

              const float groupMod = itsGroupModMap.getVal(a_x, a_y);

              for (int rf_x = 0; rf_x < kerndims.w(); ++rf_x)
                {
                  // Current position plus its field - center
                  const int b_x = a_x + (rf_x-kerncenter.i);

                  if (b_x < 0 || b_x >= dims.w())
                    continue;

                  for (int rf_y = 0; rf_y < kerndims.h(); ++rf_y)
                    {
                      const int b_y = a_y - (rf_y-kerncenter.j);

                      if (b_y < 0 || b_y >= dims.h()) // stay inside of image
                        continue;

                      const int offset = b_x + b_y*dims.w();

                      const float* weights_ptr =
                        cxn.weightAXY(angle_a, rf_x, rf_y);

                      for (int angle_b = 0; angle_b < nori; ++angle_b)
                        {
                          const float cxn_strength = *weights_ptr++;

                          if (cxn_strength == 0.0f)
                            continue;

                          const float input_b = input_b_itrs[angle_b][offset];

                          if (input_b <= cfgFilterThresh)
                            continue;

                          // excitation is modulated by fast plasticity
                          // suppression is modulated by group activity
                          float charge =
                            input_a * input_b
                            * cxn_strength * fastPlastMod;

                          if (cxn_strength < 0)
                            charge *= groupMod;

                          // add energy to salmap
                          charges_b_itrs[angle_b][offset] += charge;
                        }
                    }
                }

              ++c;

              t2.mark();

              if (t2.user_secs() > 20.0)
                {
                  t1.mark();
                  t1.report("convolve");
                  t2.reset();
                  LINFO("%.2f%% complete", (100.0*c)/maxc);
                }
            }
        }
    }

  itsNeuronsCur.swap(itsNeuronsTempSpace);

  LINFO("fast plast: total N (%d), avg (%.2f)",
        fpTotal.count(), fpTotal.mean());

  LINFO("fast plast: hi    N (%d), avg (%.2f)",
        fpHi.count(), fpHi.mean());
  LINFO("fast plast: lo    N (%d), avg (%.2f)",
        fpLo.count(), fpLo.mean());
}

Image<float> ContourLayerStatic::iterate(const ContourConnection& cxn,
                                         const char* saveSuffix,
                                         int normFlags,
                                         SaveSet& save)
{
  // fetch these values once and store them so that we don't need
  // repeated getVal() calls during inner loops below
  const float cfgGroupBottom = itsGroupBottom.getVal();
  const float cfgGroupTop = itsGroupTop.getVal();
  const float cfgSuppressionAdd = itsSuppressionAdd.getVal();
  const float cfgSuppressionSub = itsSuppressionSub.getVal();

  convolveAll(cxn);

  doRectify(itsNeuronsCur);

  const Image<float> activation = sum(itsNeuronsCur);

  printStats(activation, "activation");

  // Update the salmap with (1) global leak, (2) adding the activation
  // maps for all orientations, (3) rectification.
  itsSalmap -= itsLeak.getVal();
  itsSalmap += activation;

  printStats(itsSalmap, "salmap");

  inplaceRectify(itsSalmap);

  // Save the old energy map in case we need to compute the energy
  // change for group suppression
  const Image<float> prevEnergyMap = itsEnergyMap;

  // Compute the energy map by remapping the salmap into the range [0,1]
  itsEnergyMap = sigmoid(itsSalmap, itsSigmoidThresh.getVal());

  printStats(itsEnergyMap, "energy");

  // Now modify suppression values

  Image<float> delta = lowPass(itsGroupSize.getVal(),
                               itsEnergyMap - prevEnergyMap);

  Image<float>::const_iterator delta_itr = delta.begin();
  Image<float>::iterator gmod_itr = itsGroupModMap.beginw();
  Image<float>::iterator stop = itsGroupModMap.endw();

  int numTooHigh = 0;
  int numTooLow = 0;

  while (gmod_itr != stop)
    {
      const float pctChange = *delta_itr;

      // If values are too big then add suppression...
      if (pctChange > cfgGroupTop)
        {
          *gmod_itr += cfgSuppressionAdd * (pctChange - cfgGroupTop);
          ++numTooHigh;
        }

      // ... if values are too small then remove suppression
      else if (pctChange < cfgGroupBottom)
        {
          *gmod_itr += cfgSuppressionSub * (pctChange - cfgGroupTop);
          ++numTooLow;
        }

      ++gmod_itr;
      ++delta_itr;
    }

  inplaceRectify(itsGroupModMap);

  LINFO("suppression: N (%d), + (%d), - (%d)",
        delta.getSize(), numTooHigh, numTooLow);

  if (itsSaveLayerDetails.getVal())
    save.add(sformat("gmod.%s", saveSuffix),
             GenericFrame(itsGroupModMap, normFlags));

  return itsEnergyMap;
}

// ######################################################################
// ######################################################################
// ##### ContourChannel class
// ######################################################################
// ######################################################################

enum ContourChannelFilterStyle
  {
    CC_FILTER_STEERABLE = 1,
    CC_FILTER_GABOR = 2,
    CC_FILTER_GAUSSIAN_STEERABLE = 3,
  };

std::string convertToString(const ContourChannelFilterStyle val)
{
  switch (val)
    {
    case CC_FILTER_STEERABLE: return "Steerable"; break;
    case CC_FILTER_GABOR: return "Gabor"; break;
    case CC_FILTER_GAUSSIAN_STEERABLE: return "GaussianSteerable"; break;
    }

  LFATAL("invalid ContourChannelFilterStyle '%d'", int(val));
  /* can't happen */ return std::string();
}

void convertFromString(const std::string& str1, ContourChannelFilterStyle& val)
{
  const std::string str = toLowerCase(str1);

  if      (str.compare("steerable") == 0) { val = CC_FILTER_STEERABLE; }
  else if (str.compare("gabor") == 0) { val = CC_FILTER_GABOR; }
  else if (str.compare("gaussiansteerable") == 0)  { val = CC_FILTER_GAUSSIAN_STEERABLE; }
  else
    conversion_error::raise<ContourChannelFilterStyle>(str1);
}

enum ContourChannelDynamicsType
  {
    CC_STATIC = 1,
    CC_DYNAMIC = 2
  };

std::string convertToString(const ContourChannelDynamicsType val)
{
  switch (val)
    {
    case CC_STATIC: return "Static"; break;
    case CC_DYNAMIC: return "Dynamic"; break;
    }

  LFATAL("invalid ContourChannelDynamicsType '%d'", int(val));
  /* can't happen */ return std::string();
}

void convertFromString(const std::string& str1, ContourChannelDynamicsType& val)
{
  const std::string str = toLowerCase(str1);

  if      (str.compare("static") == 0) { val = CC_STATIC; }
  else if (str.compare("dynamic") == 0) { val = CC_DYNAMIC; }
  else
    conversion_error::raise<ContourChannelDynamicsType>(str1);
}

class ContourChannel : public ChannelBase
{
public:
  ContourChannel(OptionManager& mgr,
                 const std::string& saveprefix = "contourout");
  virtual ~ContourChannel();

  virtual bool outputAvailable() const;

  virtual Dims getMapDims() const;

  virtual uint numSubmaps() const;

  //! Not implemented for ContourChannel
  virtual Image<float> getSubmap(unsigned int index) const
  {
    LFATAL("getSubmap() not implemented for ContourChannel");
    return Image<float>(); // placate compiler with a return statement
  }

  virtual std::string getSubmapName(unsigned int index) const;

  virtual std::string getSubmapNameShort(unsigned int index) const;

  //! Not implemented for ContourChannel
  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const
  {
    LFATAL("getFeatures() not implemented for ContourChannel");
  }

  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const
  {
    LFATAL("getFeatures() not implemented for ContourChannel");
  }

  virtual Image<float> getOutput();

  //! Save our various maps using a FrameOstream
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

protected:
  virtual void paramChanged(ModelParamBase* param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  virtual void start1()
  {
    ChannelBase::start1();
    itsFrameIdx = 0;
    if (itsNumIterations.getVal() <= 0)
      LFATAL("expected positive value for --%s, but got %u",
             itsNumIterations.getOptionDef()->longoptname,
             itsNumIterations.getVal());

    if (itsNumScales.getVal() <= 0)
      LFATAL("expected positive value for --%s, but got %u",
             itsNumScales.getOptionDef()->longoptname,
             itsNumScales.getVal());
  }

  virtual void doInput(const InputFrame& inframe);

  virtual void killCaches();

  virtual void saveStats(const Image<float> img, const short idx);

private:
  ContourChannel(const ContourChannel&);
  ContourChannel& operator=(const ContourChannel&);

  OModelParam<ContourConnectionType> itsCxnType;
  OModelParam<unsigned int>    itsNumIterations;
  OModelParam<unsigned int>    itsLowPassWidth;
  OModelParam<unsigned int>    itsFirstScale;
  OModelParam<unsigned int>    itsNumScales;
  OModelParam<Dims>            itsMaxInputDims;
  OModelParam<ContourChannelFilterStyle> itsFilterStyle;
  OModelParam<float>           itsGaborPeriod;
  OModelParam<unsigned int>    itsFilterReduction;
  OModelParam<ContourChannelDynamicsType> itsDynamicsType;
  OModelParam<MaxNormType>     itsNormType;
  OModelParam<int>             itsNormFlags;
  OModelParam<LevelSpec>       itsLevelSpec;
  OModelParam<int>             itsOutputBlur;
  OModelParam<bool>            itsSaveWeights;
  OModelParam<bool>            itsSaveFilterOutput;
  OModelParam<bool>            itsSaveOutput;
  const std::string            itsSavePrefix;

  //! Save basic single channel stats after combineSubMaps
  OModelParam<bool> itsGetSingleChannelStats;

  //! If saving stats, should we put each feature in its own file?
  OModelParam<bool> itsSaveStatsPerChannel;

  //! File name for single channel stats after combineSubMaps
  OModelParam<std::string> itsGetSingleChannelStatsFile;

  //! Tag name for single channel stats after combineSubMaps
  OModelParam<std::string> itsGetSingleChannelStatsTag;

  nub::ref<ContourConnection>  itsConnection;
  nub::ref<ContourLayer>       itsScaleBand;

  Image<float>                 itsRawInput;
  Image<float>                 itsFloatInput;
  Image<float>                 itsOutput;

  SaveSet                      itsSaveSet;
  uint                         itsFrameIdx; // for logging purposes
};

// ######################################################################
// ######################################################################
// ##### ContourChannel implementation
// ######################################################################
// ######################################################################

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourConnectionType =
  { MODOPT_ARG(ContourConnectionType), "ContourConnectionType", &MOC_CHANNEL, OPTEXP_CORE,
    "Type of connection kernel to use in the contour channel "
    "(corresponds to old ParamMap parameter 'cxnType=3')",
    "contour-cxn-type", '\0', "<Cinnic|Braun|BraunNorm>", "BraunNorm" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelIterations =
  { MODOPT_ARG(unsigned int), "ContourChannelIterations", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of iterations to run in the contour channel "
    "(corresponds to old ParamMap parameter 'iterations=3')",
    "contour-num-iterations", '\0', "<uint>", "3" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelLowPassWidth =
  { MODOPT_ARG(unsigned int), "ContourChannelLowPassWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "Width of low-pass filter to use in the contour channel "
    "(corresponds to old ParamMap parameter 'lowPassWidth=9')",
    "contour-lowpass-width", '\0', "<uint>", "9" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelFirstScale =
  { MODOPT_ARG(unsigned int), "ContourChannelFirstScale", &MOC_CHANNEL, OPTEXP_CORE,
    "Lowest pyramid scale to use in the contour channel "
    "(corresponds to old ParamMap parameter 'firstScale=0')",
    "contour-first-scale", '\0', "<uint>", "0" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelNumScales =
  { MODOPT_ARG(unsigned int), "ContourChannelNumScales", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of pyramid scale to use in the contour channel "
    "(corresponds to old ParamMap parameter 'scalesNumber=3')",
    "contour-num-scales", '\0', "<uint>", "3" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelMaxInputDims =
  { MODOPT_ARG(Dims), "ContourChannelMaxInputDims", &MOC_CHANNEL, OPTEXP_CORE,
    "If input to the contour channel is larger than these dims, then "
    "it will be rescaled down to fit "
    "(corresponds to old ParamMap parameters 'maxInputW=10000' and 'maxInputH=10000')",
    "contour-max-input-dims", '\0', "<w>x<h>", "10000x10000" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelFilterStyle =
  { MODOPT_ARG(ContourChannelFilterStyle), "ContourChannelFilterStyle", &MOC_CHANNEL, OPTEXP_CORE,
    "Type of orientation filtering to use in the contour channel "
    "(corresponds to old ParamMap parameter 'filterStyle=2')",
    "contour-filter-style", '\0', "<Steerable|Gabor|GaussianSteerable>", "Gabor" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelGaborPeriod =
  { MODOPT_ARG(float), "ContourChannelGaborPeriod", &MOC_CHANNEL, OPTEXP_CORE,
    "Period of the sine-wave component of the gabor filter used in the "
    "contour channel "
    "(corresponds to old ParamMap parameter 'gaborPeriod=5.0')",
    "contour-gabor-width", '\0', "<float>", "5.0" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelFilterReduction =
  { MODOPT_ARG(unsigned int), "ContourChannelFilterReduction", &MOC_CHANNEL, OPTEXP_CORE,
    "Number of octaves by which to downsize the oriented filter outputs "
    "in the contour channel "
    "(corresponds to old ParamMap parameter 'filterReduction=1')",
    "contour-filter-reduction", '\0', "<uint>", "1" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelDynamicsType =
  { MODOPT_ARG(ContourChannelDynamicsType), "ContourChannelDynamicsType", &MOC_CHANNEL, OPTEXP_CORE,
    "The type of iterative dynamics to use in the contour channel "
    "(corresponds to old ParamMap parameter 'dynamicsType=1')",
    "contour-dynamics-type", '\0', "<Static|Dynamic>", "Static" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelNormFlags =
  { MODOPT_ARG(int), "ContourChannelNormFlags", &MOC_CHANNEL, OPTEXP_CORE,
    "Bitwise-OR'ed combination of Flags to use when normaling float images "
    "in the contour channel (FLOAT_NORM_0_255=1, FLOAT_NORM_WITH_SCALE=2, "
    "FLOAT_NORM_PRESERVE=4) "
    "(corresponds to old ParamMap parameter 'normFlags=3')",
    "contour-norm-flags", '\0', "<int>", "3" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelOutputBlur =
  { MODOPT_ARG(int), "ContourChannelOutputBlur", &MOC_CHANNEL, OPTEXP_CORE,
    "Box-filter width to use when blurring+decimating to rescale "
    "the output of the contour channel",
    "contour-output-blur", '\0', "<int>", "4" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelSaveWeights =
  { MODOPT_FLAG, "ContourChannelSaveWeights", &MOC_CHANNEL, OPTEXP_SAVE,
    "Whether to save a summary image of the contour channel's kernel "
    "weights",
    "save-contour-weights", '\0', "", "false" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelSaveFilterOutput =
  { MODOPT_FLAG, "ContourChannelSaveFilterOutput", &MOC_CHANNEL, OPTEXP_SAVE,
    "Whether to save summary images of the contour channel's oriented "
    "filter outputs",
    "save-contour-filter-output", '\0', "", "false" };

// Used by: ContourChannel
static const ModelOptionDef OPT_ContourChannelSaveOutput =
  { MODOPT_FLAG, "ContourChannelSaveOutput", &MOC_CHANNEL, OPTEXP_SAVE,
    "Whether to save output maps from the contour channel",
    "save-contour-output", '\0', "", "false" };


// Used by: ContourChannel
static const ModelOptionDef OPT_ALIAScontourModel0027 =
  { MODOPT_ALIAS, "ALIAScontourModel0027", &MOC_ALIAS, OPTEXP_CORE,
    "Contour model #0027",
    "contour-model-0027", '\0', "",
    "--contour-cxn-type=BraunNorm "
    "--contour-cxn-beta=0.07 "
    "--contour-cxn-excit-strength=1.7 "
    "--contour-cxn-inhib-strength=-0.3 "
    "--contour-cxn-dims=25x25 "
    "--contour-cxn-peak-distance=2.5 "
    "--contour-dynamics-type=Static "
    "--cinnic-fast-plast=5.405775 "
    "--cinnic-group-top=0.0048828125 "
    "--cinnic-leak=0.0003662109375 "
    "--cinnic-sigmoid-thresh=35.15625 "
    "--cinnic-suppression-add=1.6875 "
    "--cinnic-suppression-sub=2.0 "
    "--contour-filter-reduction=1 "
    "--contour-first-scale=2 "
    "--contour-gabor-width=5.0 "
    "--contour-num-iterations=3 "
    "--contour-num-scales=1 "
  };

// Used by: ContourChannel
static const ModelOptionDef OPT_ALIAScontourModel0032 =
  { MODOPT_ALIAS, "ALIAScontourModel0032", &MOC_ALIAS, OPTEXP_CORE,
    "Contour model #0032",
    "contour-model-0032", '\0', "",
    "--contour-cxn-type=BraunNorm "
    "--contour-cxn-beta=0.000720977783205 "
    "--contour-cxn-dims=31x31 "
    "--contour-cxn-excit-strength=2.33459472656 "
    "--contour-cxn-inhib-strength=-0.0030920982361 "
    "--contour-cxn-peak-distance=54.931640625 "
    "--contour-dynamics-type=Static "
    "--cinnic-fast-plast=0.0791861572262 "
    "--cinnic-group-size=11 "
    "--cinnic-group-top=0.15625 "
    "--cinnic-leak=1.318359375 "
    "--cinnic-sigmoid-thresh=474.609375 "
    "--cinnic-suppression-add=0.75 "
    "--cinnic-suppression-sub=2.0 "
    "--contour-filter-reduction=2 "
    "--contour-first-scale=1 "
    "--contour-gabor-width=2.02549743327 "
    "--contour-num-iterations=1 "
    "--contour-num-scales=1 "
  };


ContourChannel::ContourChannel(OptionManager& mgr,
                               const std::string& saveprefix)
  :
  ChannelBase(mgr, "Contours", "contours", UNKNOWN),

  itsCxnType              ( &OPT_ContourConnectionType, this ),
  itsNumIterations        ( &OPT_ContourChannelIterations, this ),
  itsLowPassWidth         ( &OPT_ContourChannelLowPassWidth, this ),
  itsFirstScale           ( &OPT_ContourChannelFirstScale, this ),
  itsNumScales            ( &OPT_ContourChannelNumScales, this ),
  itsMaxInputDims         ( &OPT_ContourChannelMaxInputDims, this ),
  itsFilterStyle          ( &OPT_ContourChannelFilterStyle, this ),
  itsGaborPeriod          ( &OPT_ContourChannelGaborPeriod, this ),
  itsFilterReduction      ( &OPT_ContourChannelFilterReduction, this ),
  itsDynamicsType         ( &OPT_ContourChannelDynamicsType, this ),
  itsNormType             ( &OPT_MaxNormType, this ),
  itsNormFlags            ( &OPT_ContourChannelNormFlags, this ),
  itsLevelSpec            ( &OPT_LevelSpec, this ),
  itsOutputBlur           ( &OPT_ContourChannelOutputBlur, this ),
  itsSaveWeights          ( &OPT_ContourChannelSaveWeights, this ),
  itsSaveFilterOutput     ( &OPT_ContourChannelSaveFilterOutput, this ),
  itsSaveOutput           ( &OPT_ContourChannelSaveOutput, this ),
  itsSavePrefix           ( saveprefix ), // pmap->getStringParam("savePrefix", "contourout"),
  itsGetSingleChannelStats(&OPT_GetSingleChannelStats, this),
  itsSaveStatsPerChannel(&OPT_SaveStatsPerChannel, this),
  itsGetSingleChannelStatsFile(&OPT_GetSingleChannelStatsFile, this),
  itsGetSingleChannelStatsTag(&OPT_GetSingleChannelStatsTag, this),
  itsConnection           ( new ContourConnectionBraun(mgr, true) ),
  itsScaleBand            ( new ContourLayerStatic(mgr) ),

  itsRawInput             ( ),
  itsFloatInput           ( ),
  itsOutput               ( )
{
  mgr.requestOptionAlias(&OPT_ALIAScontourModel0027);
  mgr.requestOptionAlias(&OPT_ALIAScontourModel0032);

  this->addSubComponent(itsConnection);
  this->addSubComponent(itsScaleBand);
}

// ######################################################################
ContourChannel::~ContourChannel()
{}

// ######################################################################
bool ContourChannel::outputAvailable() const
{
  return itsOutput.initialized();
}

// ######################################################################
Dims ContourChannel::getMapDims() const
{
  const int lev = itsLevelSpec.getVal().mapLevel();

  return this->getInputDims() / (1 << lev);
}

// ######################################################################
uint ContourChannel::numSubmaps() const
{
  return itsNumScales.getVal() * itsConnection->numAngles();
}

// ######################################################################
std::string ContourChannel::getSubmapName(unsigned int index) const
{
  const unsigned int scale = index / itsNumScales.getVal();
  const unsigned int angle = index % itsNumScales.getVal();

  return sformat("%s scale: %u, angle: %u", descriptiveName().c_str(), scale, angle);
}

// ######################################################################
std::string ContourChannel::getSubmapNameShort(unsigned int index) const
{
  const unsigned int scale = index / itsNumScales.getVal();
  const unsigned int angle = index % itsNumScales.getVal();

  return sformat("%s(%u,%u)", tagName().c_str(), scale, angle);
}

// ######################################################################
Image<float> ContourChannel::getOutput()
{
  if(itsGetSingleChannelStats.getVal())
    saveStats(itsOutput, -1);

  return itsOutput;
}

// ######################################################################
void ContourChannel::saveStats(const Image<float> img, const short idx)
{
  std::string fileName;

  if(itsSaveStatsPerChannel.getVal())
  {
    std::string dot = ".";
    std::string txt = ".txt";
    fileName = itsGetSingleChannelStatsFile.getVal()+ dot + tagName() + txt;
  }
  else
  {
    fileName = itsGetSingleChannelStatsFile.getVal();
  }

  //LINFO("SAVING single channel stats %d - %s to file %s",
  //      idx, descriptiveName().c_str(),fileName.c_str());
  ushort minx = 0, miny = 0, maxx = 0, maxy = 0;
  float  min,  max,  avg,  std;
  uint   N;
  // get a whole bunch of stats about this output image
  getMinMaxAvgEtc(img, min, max, avg, std, minx, miny, maxx, maxy, N);

  std::ofstream statsFile(fileName.c_str(), std::ios::app);

  statsFile << itsGetSingleChannelStatsTag.getVal() << "\t";

  statsFile << itsFrameIdx << "\t";

  // differentiate between scale image stats and the combined max norm
  // for this channel
  if(idx == -1)
  {
    statsFile << "COMBINED\t-1\t";
    itsFrameIdx++;
  }
  else
    statsFile << "SCALE\t" << idx << "\t";

  statsFile << tagName().c_str() << "\t" << descriptiveName().c_str() << "\t";
  statsFile << min  << "\t" << max  << "\t" << avg  << "\t" << std  << "\t"
            << minx << "\t" << miny << "\t" << maxx << "\t" << maxy << "\t"
            << N    << "\n";
  statsFile.close();
}

// ######################################################################
void ContourChannel::saveResults(const nub::ref<FrameOstream>& ofs)
{
  itsSaveSet.saveAll(*ofs, itsSavePrefix.c_str());
}

// ######################################################################
void ContourChannel::paramChanged(ModelParamBase* param,
                                  const bool valueChanged,
                                  ParamClient::ChangeStatus* status)
{
  if (param == &itsCxnType)
    {
      this->removeSubComponent(*itsConnection);

      switch (itsCxnType.getVal())
        {
        case CC_CXN_CINNIC:     itsConnection.reset(new ContourConnectionCinnic(getManager())); break;
        case CC_CXN_BRAUN:      itsConnection.reset(new ContourConnectionBraun(getManager(), false)); break;
        case CC_CXN_BRAUN_NORM: itsConnection.reset(new ContourConnectionBraun(getManager(), true)); break;
        default:
          LFATAL("unknown connection type '%d'", int(itsCxnType.getVal()));
        }

      this->addSubComponent(itsConnection);
      itsConnection->exportOptions(MC_RECURSE);
    }
  else if (param == &itsDynamicsType)
    {
      this->removeSubComponent(*itsScaleBand);

      switch (itsDynamicsType.getVal())
        {
        case CC_STATIC: itsScaleBand.reset(new ContourLayerStatic(getManager())); break;
        case CC_DYNAMIC: itsScaleBand.reset(new ContourLayerDynamic(getManager())); break;
        default:
          LFATAL("unknown dynamics type '%d'", int(itsDynamicsType.getVal()));
        }

      this->addSubComponent(itsScaleBand);
      itsScaleBand->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void ContourChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());

  CpuTimer t;

  itsSaveSet.clear();

  if (itsSaveWeights.getVal())
    itsSaveSet.add("weights", GenericFrame(itsConnection->getSummaryImage()));

  itsRawInput = inframe.grayFloat();

  itsFloatInput =
    (inframe.getWidth() > itsMaxInputDims.getVal().w() ||
     inframe.getHeight() > itsMaxInputDims.getVal().h())
    ? rescale(inframe.grayFloat(), itsMaxInputDims.getVal())
    : inframe.grayFloat();

  ImageSet<float> filterSets[itsNumScales.getVal()];

  switch (itsFilterStyle.getVal())
    {
    case CC_FILTER_STEERABLE:
      {
        const ImageSet<float> lpyr =
          buildPyrLaplacian(itsFloatInput,
                            0, itsFirstScale.getVal()+itsNumScales.getVal(),
                            itsLowPassWidth.getVal());

        for (unsigned int s = 0; s < itsNumScales.getVal(); ++s)
          {
            filterSets[s] =
              orientedFilterSet(lpyr[itsFirstScale.getVal()+s],
                                itsGaborPeriod.getVal(),
                                itsConnection->angles(),
                                itsConnection->numAngles());
            doMeanNormalize(filterSets[s]);
            doRectify(filterSets[s]);
          }
      }
      break;
    case CC_FILTER_GABOR:
    case CC_FILTER_GAUSSIAN_STEERABLE:
      {
        ImageSet<float> gausspyr =
          buildPyrGaussian(itsFloatInput,
                           0, itsFirstScale.getVal()+itsNumScales.getVal(),
                           itsLowPassWidth.getVal());

        doEnergyNorm(gausspyr);

        if (itsFilterStyle.getVal() == CC_FILTER_GABOR)
          {
            ImageSet<float> gpyr[itsConnection->numAngles()];

            for (int i = 0; i < itsConnection->numAngles(); ++i)
              {
                // Note, we don't need to pass the DO_ENERGY_NORM flag
                // to buildPyrGabor() since we've already called
                // doEnergyNorm() a few lines up from here.
                gpyr[i] = buildPyrGabor(gausspyr,
                                        itsConnection->angle(i),
                                        itsGaborPeriod.getVal(),
                                        1.0 /*elongation ratio*/,
                                        -1 /*size: -1 gives auto-default*/,
                                        0 /*flags*/);

                // Memory optimization: discard any lower pyramid
                // levels, up to (but not including) itsFirstScale. It
                // is only the higher levels that will be needed in
                // the forthcoming takeSlice(), and we can save a lot
                // of memory usage by releasing the unused memory in
                // the lower levels.
                for (unsigned int n = 0; n < itsFirstScale.getVal(); ++n)
                  gpyr[i].getImageMut(n) = Image<float>();
              }

            for (unsigned int s = 0; s < itsNumScales.getVal(); ++s)
              {
                filterSets[s] = takeSlice(gpyr,
                                          itsConnection->numAngles(),
                                          itsFirstScale.getVal()+s);
                doMeanNormalize(filterSets[s]);
                doRectify(filterSets[s]);
              }
          }
        else // itsFilterStyle.getVal() == CC_FILTER_GAUSSIAN_STEERABLE
          {
            for (unsigned int s = 0; s < itsNumScales.getVal(); ++s)
              {
                filterSets[s] =
                  orientedFilterSet(gausspyr[itsFirstScale.getVal()+s],
                                    itsGaborPeriod.getVal(),
                                    itsConnection->angles(),
                                    itsConnection->numAngles());
                doMeanNormalize(filterSets[s]);
                doRectify(filterSets[s]);
              }
          }
      }
      break;
    default:
      LFATAL("invalid filterStyle '%d'", int(itsFilterStyle.getVal()));
    }

  t.mark();
  t.report();

  const Dims imgDims = itsFloatInput.getDims();

  itsOutput = Image<float>(imgDims, ZEROS);
  Image<float> maxout(imgDims, ZEROS);

  for (unsigned int s = 0; s < itsNumScales.getVal(); ++s)
    {
      filterSets[s] = reduce(filterSets[s], itsFilterReduction.getVal());

      if (itsSaveFilterOutput.getVal())
        itsSaveSet.add(sformat("filter.s-%u", s),
                       GenericFrame(makeImageArray(filterSets[s]),
                                    itsNormFlags.getVal()));

      doOneNormalize(filterSets[s]);

      const std::string saveSuffix = sformat("s-%u", s);

      const Image<float> salmap =
        itsScaleBand->compute(filterSets[s],
                              *itsConnection,
                              saveSuffix,
                              itsNormFlags.getVal(),
                              itsSaveSet,
                              itsNumIterations.getVal(),
                              imgDims);

      const Image<float> scale_sm = rescale(salmap, imgDims);

      const float bias = getScaleBias(s);
      itsOutput += (scale_sm * bias);
      maxout = takeMax(maxout, scale_sm);

      t.mark();
      t.report();
    }

  if (itsSaveOutput.getVal())
    {
      itsSaveSet.add("avg",
                     GenericFrame(itsOutput, itsNormFlags.getVal()));

      itsSaveSet.add("max",
                     GenericFrame(maxout, itsNormFlags.getVal()));
    }

  itsOutput = boxDownSizeClean(itsOutput, this->getMapDims(),
                               itsOutputBlur.getVal());

  itsOutput = maxNormalize(itsOutput,
                           MAXNORMMIN, MAXNORMMAX,
                           itsNormType.getVal());

  if (itsSaveOutput.getVal())
    {
      itsSaveSet.add("final",
                     GenericFrame(itsOutput, itsNormFlags.getVal()));
    }
}

// ######################################################################
void ContourChannel::killCaches()
{
  ChannelBase::killCaches();
}

// ######################################################################
// ######################################################################
// ##### Factory function for ContourChannel
// ######################################################################
// ######################################################################

nub::ref<ChannelBase> makeContourChannel(OptionManager& mgr,
                                         const std::string& saveprefix)
{
  return makeSharedComp(new ContourChannel(mgr, saveprefix));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
