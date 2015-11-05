/*!@file Image/SteerableFilters.C Implementation */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/SteerableFilters.C $

#include "Image/SteerableFilters.H"
#include "Image/ImageSetOps.H"
#include "Util/MathFunctions.H"

// ######################################################################
// Gaussian Derivitive Parameter implementation
// ######################################################################
Gauss2ndDerivParams::Gauss2ndDerivParams() 
  : alpha(0.0F), beta(0.0F), gamma(0.0F) 
{ }

// ######################################################################
Gauss2ndDerivParams::Gauss2ndDerivParams(float const & a, float const & b, float const & g) 
  : alpha(a), beta(b), gamma(g)
{ }
  
// ######################################################################
Gauss2ndDerivParams::Gauss2ndDerivParams(float const & theta, float const & phi) :
    alpha(cos(theta * D_PI / 180.0F) * sin(phi * D_PI / 180.0F)),
    beta(sin(theta * D_PI / 180.0F) * sin(phi * D_PI / 180.0F)),
    gamma(cos(phi * D_PI / 180.0F)) 
{ 
  if (fabs(alpha) < 0.0000000001F)
    alpha = 0.0F;

  if (fabs(beta) < 0.0000000001F)
    beta = 0.0F;

  if (fabs(gamma) < 0.0000000001F)
    gamma = 0.0F;
}
  
// ###################################################################### 
// Spatio-temporal filter implementation
// ######################################################################
  
// ######################################################################
SpatioTemporalEnergy::SpatioTemporalEnergy() 
    :   itsTimeQ(), 
        itsFilters(), itsFiltersH(), 
        itsBasisSet(), itsBasisSetH(),
        itsWeights(), itsWeightsH()
{ }

// ######################################################################
SpatioTemporalEnergy::SpatioTemporalEnergy(std::vector<Gauss2ndDerivParams> const & params) 
    :   itsTimeQ(), 
        itsFilters(), itsFiltersH(), 
        itsBasisSet(), itsBasisSetH(),
        itsWeights(), itsWeightsH()
{ 
  setupFilters(params);
}

// ######################################################################
SpatioTemporalEnergy::~SpatioTemporalEnergy() 
{ }

// ######################################################################
Image<float> SpatioTemporalEnergy::filter3D(SeperableFilter3D const & filter) const
{
  ImageSet<float> resp = itsTimeQ;
  for (uint ii = 0; ii < resp.size(); ++ii)
  {
    resp[ii] = sepFilter(resp[ii], filter.xfilt, filter.yfilt, ConvolutionBoundaryStrategy::CONV_BOUNDARY_ZERO);
    resp[ii] *= filter.zfilt[ii];
  }
  return sum(resp);
}

// ######################################################################
ImageSet<float> SpatioTemporalEnergy::getNextOutput(Image<float> const & img)
{
  //update the queue
  if (itsTimeQ.size() == 0)
    itsTimeQ = ImageSet<float>(9, img.getDims(), ZEROS);
  
  itsTimeQ.push_front(img);
  itsTimeQ.pop_back();
  
  //get the response from our filter set, normal filter first
  uint fcount = 0;
  for (std::vector<SeperableFilter3D>::iterator filter = itsFilters.begin(); filter != itsFilters.end(); ++filter)
    itsBasisSet[fcount++] = filter3D(*filter);
  
  //then with the hilbert transformed filters
  fcount = 0;
  for (std::vector<SeperableFilter3D>::iterator filter = itsFiltersH.begin(); filter != itsFiltersH.end(); ++filter)
    itsBasisSetH[fcount++] = filter3D(*filter);

  return getResponse();
}

// ######################################################################
ImageSet<float> SpatioTemporalEnergy::getNextOutput(SpatioTemporalEnergy const & ste)
{
  //loop over each of our desired filter orientations and weight the basis set
  //to get the response
  ImageSet<float> resp, respH;
  if (itsWeights.size() == 0)
    return resp;

  for (std::vector<float> const & w : itsWeights)
    resp.push_back(weightedSum(ste.itsBasisSet, w));

  for (std::vector<float> const & w : itsWeightsH)
    respH.push_back(weightedSum(ste.itsBasisSetH, w));

  doSquared(resp);
  doSquared(respH);
  resp += respH;

  return resp;
}

// ######################################################################
ImageSet<float> SpatioTemporalEnergy::getResponse() const
{
  ImageSet<float> resp, respH;
  if (itsWeights.size() == 0)
    return resp;
  
  for (std::vector<float> const & w : itsWeights)
    resp.push_back(weightedSum(itsBasisSet, w));
  
  for (std::vector<float> const & w : itsWeightsH)
    respH.push_back(weightedSum(itsBasisSetH, w));
  
  doSquared(resp);
  doSquared(respH);
  resp += respH;
  
  return resp;
}

// ######################################################################
void SpatioTemporalEnergy::setupFilters(std::vector<Gauss2ndDerivParams> const & params)
{
  itsFilters.clear();
  itsFiltersH.clear();
  itsBasisSet.clear();
  itsBasisSetH.clear();
  itsWeights.clear();
  itsWeightsH.clear();

  std::vector<std::function<float(Gauss2ndDerivParams const &)> > itsCoeffFunc;
  std::vector<std::function<float(Gauss2ndDerivParams const &)> > itsCoeffFuncH;
  
  float f1[] = {0.0084F, 0.1025F, 0.354F, -0.0537F, -0.823F, -0.0537F, 0.354F, 0.1025F, 0.0084};
  Image<float> f1i(f1,1,9);
  float f2[] = {0.0008F, 0.0176F, 0.166F, 0.6383F, 1.0F, 0.6383F, 0.166F, 0.0176F, 0.0008F};
  Image<float> f2i(f2,1,9);
  float f3[] = {-0.0034F, -0.0582F, -0.3662F, -0.7039F, -0.0F, 0.7039F, 0.3662F, 0.0582F, 0.0034F};
  Image<float> f3i(f3,1,9);
  float f4[] = {-0.002F, -0.0354F, -0.2225F, -0.4277F, -0.0F, 0.4277F, 0.2225F, 0.0354F, 0.002F};
  Image<float> f4i(f4,1,9);

  float h1[] = {-0.0088F, -0.0554F, 0.0895F, 0.6776F, -0.0F, -0.6776F, -0.0895F, 0.0554F, 0.0088F};
  Image<float> h1i(h1,1,9);
  float h2[] = {0.0043F, 0.0508F, 0.1522F, -0.1695F, -0.6595F, -0.1695F, 0.1522F, 0.0508F, 0.0043F};
  Image<float> h2i(h2,1,9);
  float h3[] = {-0.0008F, -0.0176F, -0.166F, -0.6383F, -1.0F, 0.6383F, 0.166F, 0.0176F, 0.0008F};
  Image<float> h3i(h3,1,9);
  float h4[] = {-0.0018F, -0.031F, -0.1953F, -0.3754F, -0.0F, 0.3754F, 0.1953F, 0.031F, 0.0018F};
  Image<float> h4i(h4,1,9);
  float h5[] = {-0.002F, -0.0354F, -0.2225F, -0.4277F, -0.0F, 0.4277F, 0.2225F, 0.0354F, 0.002F};
  Image<float> h5i(h5,1,9);

  SeperableFilter3D ff1 = {f1i, f2i, f2i};
  SeperableFilter3D ff2 = {f3i, f4i, f2i};
  SeperableFilter3D ff3 = {f2i, f1i, f2i};
  SeperableFilter3D ff4 = {f3i, f2i, f4i};
  SeperableFilter3D ff5 = {f2i, f3i, f4i};
  SeperableFilter3D ff6 = {f2i, f2i, f1i};

  SeperableFilter3D hh1 = {h1i, h3i, h3i};
  SeperableFilter3D hh2 = {h2i, h5i, h3i};
  SeperableFilter3D hh3 = {h5i, h2i, h3i};
  SeperableFilter3D hh4 = {h3i, h1i, h3i};
  SeperableFilter3D hh5 = {h2i, h3i, h5i};
  SeperableFilter3D hh6 = {h4i, h5i, h5i};
  SeperableFilter3D hh7 = {h3i, h2i, h5i};
  SeperableFilter3D hh8 = {h5i, h3i, h2i};
  SeperableFilter3D hh9 = {h3i, h5i, h2i};
  SeperableFilter3D hh10 = {h3i, h3i, h1i};

  itsFilters.push_back(ff1);
  itsFilters.push_back(ff2);
  itsFilters.push_back(ff3);
  itsFilters.push_back(ff4);
  itsFilters.push_back(ff5);
  itsFilters.push_back(ff6);

  itsFiltersH.push_back(hh1);
  itsFiltersH.push_back(hh2);
  itsFiltersH.push_back(hh3);
  itsFiltersH.push_back(hh4);
  itsFiltersH.push_back(hh5);
  itsFiltersH.push_back(hh6); 
  itsFiltersH.push_back(hh7);
  itsFiltersH.push_back(hh8);
  itsFiltersH.push_back(hh9);
  itsFiltersH.push_back(hh10);

  itsCoeffFunc.resize(itsFilters.size());
  itsCoeffFunc[0] = [](Gauss2ndDerivParams const & filter)->float {return filter.alpha * filter.alpha; };
  itsCoeffFunc[1] = [](Gauss2ndDerivParams const & filter)->float {return 2*filter.alpha * filter.beta; };
  itsCoeffFunc[2] = [](Gauss2ndDerivParams const & filter)->float {return filter.beta * filter.beta; };
  itsCoeffFunc[3] = [](Gauss2ndDerivParams const & filter)->float {return 2*filter.alpha * filter.gamma; };
  itsCoeffFunc[4] = [](Gauss2ndDerivParams const & filter)->float {return 2*filter.beta * filter.gamma; };
  itsCoeffFunc[5] = [](Gauss2ndDerivParams const & filter)->float {return filter.gamma * filter.gamma; };

  itsCoeffFuncH.resize(itsFiltersH.size());
  itsCoeffFuncH[0] = [](Gauss2ndDerivParams const & filter)->float {return filter.alpha * filter.alpha * filter.alpha; };
  itsCoeffFuncH[1] = [](Gauss2ndDerivParams const & filter)->float {return 3*filter.alpha * filter.alpha * filter.beta; };
  itsCoeffFuncH[2] = [](Gauss2ndDerivParams const & filter)->float {return 3*filter.alpha * filter.beta * filter.beta; };
  itsCoeffFuncH[3] = [](Gauss2ndDerivParams const & filter)->float {return filter.beta * filter.beta * filter.beta; };
  itsCoeffFuncH[4] = [](Gauss2ndDerivParams const & filter)->float {return 3*filter.alpha * filter.alpha * filter.gamma; };
  itsCoeffFuncH[5] = [](Gauss2ndDerivParams const & filter)->float {return 6*filter.alpha * filter.beta * filter.gamma; };
  itsCoeffFuncH[6] = [](Gauss2ndDerivParams const & filter)->float {return 3*filter.beta * filter.beta * filter.gamma; };
  itsCoeffFuncH[7] = [](Gauss2ndDerivParams const & filter)->float {return 3*filter.alpha * filter.gamma * filter.gamma; };
  itsCoeffFuncH[8] = [](Gauss2ndDerivParams const & filter)->float {return 3*filter.beta * filter.gamma * filter.gamma; };
  itsCoeffFuncH[9] = [](Gauss2ndDerivParams const & filter)->float {return 3*filter.gamma * filter.gamma * filter.gamma; };

  itsBasisSet.reset(itsFilters.size());
  itsBasisSetH.reset(itsFiltersH.size());

  for (Gauss2ndDerivParams const & filter : params)
  {
    std::vector<float> w,wh;
    for (std::function<float(Gauss2ndDerivParams const &)> func : itsCoeffFunc)
      w.push_back(func(filter));

    for (std::function<float(Gauss2ndDerivParams const &)> func : itsCoeffFuncH)
      wh.push_back(func(filter));

    itsWeights.push_back(w);
    itsWeightsH.push_back(wh);
  }
}

// ######################################################################
void SpatioTemporalEnergy::setupFilters()
{
  std::vector<Gauss2ndDerivParams> params;
  setupFilters(params);
}

// ######################################################################
Image<float> SpatioTemporalEnergy::weightedSum(ImageSet<float> const & imgset, std::vector<float> const & weights) const
{
  ASSERT(imgset.size() == weights.size());
  
  Image<float> sum(imgset[0].getDims(), ZEROS);
  for (uint ii = 0; ii < imgset.size(); ++ii)
  {
    if (weights[ii] != 0.0F)
    {
      Image<float> t = imgset[ii];
      t *= weights[ii];
      sum += t;
    }    
  }
  return sum;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
