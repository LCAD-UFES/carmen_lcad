/*!@file Neuro/SpatialMetrics.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SpatialMetrics.C $
// $Id: SpatialMetrics.C 14593 2011-03-13 00:57:50Z dberg $
//

#ifndef NEURO_SPATIALMETRICS_C_DEFINED
#define NEURO_SPATIALMETRICS_C_DEFINED

#include "Neuro/SpatialMetrics.H"

#include "Component/GlobalOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/Point2D.H"
#include "Media/MediaOpts.H"
#include "Neuro/NeuroOpts.H"
#include "Util/TextLog.H"
#include "Util/log.H"
#include "Util/sformat.H"

#include <algorithm>

// ######################################################################
SpatialMetrics::SpatialMetrics(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName),
  itsLogFile(&OPT_TextLogFile, this),
  itsInputFrameDims(&OPT_InputFrameDims, this),
  itsFoveaRadius(&OPT_FoveaRadius, this),
  itsFOAradius(&OPT_FOAradius, this),
  itsPPD(&OPT_PixelsPerDegree, this)
{}

// ######################################################################
SpatialMetrics::~SpatialMetrics()
{}

// ######################################################################
int SpatialMetrics::getFoveaRadius() const
{ return itsFoveaRadius.getVal(); }

// ######################################################################
int SpatialMetrics::getFOAradius() const
{ return itsFOAradius.getVal(); }

// ######################################################################
double SpatialMetrics::getPPD() const
{ return itsPPD.getVal().ppd(); }

// ######################################################################
double SpatialMetrics::getPPDX() const
{ return itsPPD.getVal().ppdx(); }

// ######################################################################
double SpatialMetrics::getPPDY() const
{ return itsPPD.getVal().ppdy(); }

// ######################################################################
void SpatialMetrics::paramChanged(ModelParamBase* param,
                                  const bool valueChanged,
                                  ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  OptionManager& mgr = this->getManager();

  if (param == &itsInputFrameDims && itsInputFrameDims.getVal().isNonEmpty())
    {
      const Dims indims = itsInputFrameDims.getVal();

      {
        // set the FOA radius if someone wants it but it is not set:
        int foaRadius = 0;
        convertFromString(mgr.getOptionValString(&OPT_FOAradius),
                          foaRadius);
        if (foaRadius == 0) {
          foaRadius = std::min(indims.w(), indims.h()) / 12;
          mgr.setOptionValString(&OPT_FOAradius,
                                 convertToString(foaRadius));
          LINFO("Using FOA radius = %d pixels", foaRadius);
          textLog(itsLogFile.getVal(),
                  "FOAradius", sformat("%d", foaRadius));
        }
      }

      {
        // set the fovea radius if someone wants it but it is not set:
        int foveaRadius = 0;
        convertFromString(mgr.getOptionValString(&OPT_FoveaRadius),
                          foveaRadius);
        if (foveaRadius == 0) {
          foveaRadius = std::min(indims.w(), indims.h()) / 12;
          mgr.setOptionValString(&OPT_FoveaRadius,
                                 convertToString(foveaRadius));
          LINFO("Using fovea radius = %d pixels", foveaRadius);
          textLog(itsLogFile.getVal(),
                  "FoveaRadius", sformat("%d", foveaRadius));
        }
      }

    }
}

// ######################################################################
void SpatialMetrics::setFoveaRadius(int val)
{
  getManager().setOptionValString(&OPT_FoveaRadius, convertToString(val));
}

// ######################################################################
void SpatialMetrics::setFOAradius(int val)
{
  getManager().setOptionValString(&OPT_FOAradius, convertToString(val));
}

// ######################################################################
void SpatialMetrics::pix2deg(const Point2D<int>& pixloc,
                             double& xdeg, double& ydeg) const
{
  const double width = double(itsInputFrameDims.getVal().w());
  const double height = double(itsInputFrameDims.getVal().h());
  const double ppdx = itsPPD.getVal().ppdx();
  const double ppdy = itsPPD.getVal().ppdy();

  xdeg = atan((2.0 * pixloc.i / width - 1.0) *
              tan(width / ppdx * M_PI / 360.0)) * 180.0 / M_PI;
  ydeg = atan((1.0 - 2.0 * pixloc.j / height) *
              tan(height / ppdy * M_PI / 360.0)) * 180.0 / M_PI;
}

// ######################################################################
void SpatialMetrics::deg2pix(const double xdeg, const double ydeg,
                             Point2D<int>& pixloc) const
{
  const double width = double(itsInputFrameDims.getVal().w());
  const double height = double(itsInputFrameDims.getVal().h());
  const double ppdx = itsPPD.getVal().ppdx();
  const double ppdy = itsPPD.getVal().ppdy();

  pixloc.i = int(0.5 * width * (1.0 + tan(xdeg * M_PI / 180.0) /
                                tan(width / ppdx * M_PI / 360.0)));
  pixloc.j = int(0.5 * height * (1.0 - tan(ydeg * M_PI / 180.0) /
                                 tan(height / ppdy * M_PI / 360.0)));
}

// ######################################################################
Dims SpatialMetrics::getInputFrameDims() const
{ return itsInputFrameDims.getVal(); }



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SPATIALMETRICS_C_DEFINED
