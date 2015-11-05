/*!@file Neuro/SpatialMetrics.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SpatialMetrics.H $
// $Id: SpatialMetrics.H 14593 2011-03-13 00:57:50Z dberg $
//

#ifndef NEURO_SPATIALMETRICS_H_DEFINED
#define NEURO_SPATIALMETRICS_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Image/Point2D.H"
#include "Psycho/PixPerDeg.H"

class SpatialMetrics : public ModelComponent
{
public:
  //! Constructor
  SpatialMetrics(OptionManager& mgr,
                 const std::string& descrName = "Spatial Metrics",
                 const std::string& tagName = "SpatialMetrics");

  //! Virtual destructor for proper destruction of derived classes
  virtual ~SpatialMetrics();

  //! Override to respond to changes in OPT_InputFrameDims
  virtual void paramChanged(ModelParamBase* param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Get the radius of the fovea
  int getFoveaRadius() const;

  //! Get the radius of the focus of attention
  int getFOAradius() const;

  //! Set the fovea radius
  void setFoveaRadius(int val);

  //! Set the radius of the focus of attention
  void setFOAradius(int val);

  //! Helper function converting from pixels to degrees
  void pix2deg(const Point2D<int>& pixloc, double& xdeg, double& ydeg) const;

  //! Helper function converting from degrees to pixels
  void deg2pix(const double xdeg, const double ydeg, Point2D<int>& pixloc) const;

  //! Get our current pixels-per-degree value
  double getPPD() const;

  //! Get our current pixels-per-degree value
  double getPPDX() const;

  //! Get our current pixels-per-degree value
  double getPPDY() const;

  //! Get our input frame dims
  Dims getInputFrameDims() const;

private:
  OModelParam<std::string> itsLogFile;

  OModelParam<Dims> itsInputFrameDims;

  OModelParam<int> itsFoveaRadius;
  OModelParam<int> itsFOAradius;
  OModelParam<PixPerDeg> itsPPD;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SPATIALMETRICS_H_DEFINED
