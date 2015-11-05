/*!@file Neuro/VisualCortexSurprise.H a VisualCortex with SingleChannelSurprise
  channels */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexSurprise.H $
// $Id: VisualCortexSurprise.H 10845 2009-02-13 08:49:12Z itti $
//

#ifndef VISUALCORTEXSURPRISE_H_DEFINED
#define VISUALCORTEXSURPRISE_H_DEFINED

#include "Channels/RawVisualCortex.H"

//! This is a derivation of VisualCortex using SingleChannelSurprise channels
/*! It computes Surprise instead of computing saliency. */
class VisualCortexSurprise : public RawVisualCortex
{
public:
  //! Constructor without any channels
  VisualCortexSurprise(OptionManager& mgr,
                       const std::string& descrName = "Visual Cortex Surprise",
                       const std::string& tagName = "VisualCortexSurprise");

  //! Virtual destructor for safe inheritance.
  virtual ~VisualCortexSurprise();

  //! Register surprise channel types with SubmapAlgorithm::getFactory()
  static void registerSurpriseTypes(OptionManager& mgr);

protected:
  OModelParam<std::string> itsStype;     //!< type of surprise models
  OModelParam<int> itsNormSurp;  //! apply weak spatial competition to output?
  OModelParam<float> itsSurpriseThresh;  //!< surprise nonlinerity threshold
  OModelParam<float> itsSurpriseSemisat; //!< surprise nonlinerity semisat
  OModelParam<float> itsSurpriseExp;     //!< surprise nonlinerity exponent

  //! post-process raw weighted sum of channel output maps, surprise-style
  virtual Image<float> postProcessOutputMap(const Image<float>& outmap);

  //! Rebuild our channels if surprise model type changes
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
