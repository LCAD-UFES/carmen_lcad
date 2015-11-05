/*!@file Component/ModelParamBase.C A tunable ModelComponent parameter base class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelParamBase.C $
// $Id: ModelParamBase.C 8758 2007-09-06 19:21:04Z rjpeters $
//

#include "Component/ModelParamBase.H"

#include "Util/StringUtil.H"

namespace
{
  pthread_once_t acronyms_init_once = PTHREAD_ONCE_INIT;
  std::set<std::string>* acronyms;

  void acronyms_init()
  {
    acronyms = new std::set<std::string>;
    acronyms->insert("AGM");
    acronyms->insert("ALIAS");
    acronyms->insert("ASAC");
    acronyms->insert("EHC");
    acronyms->insert("FOA");
    acronyms->insert("FOV");
    acronyms->insert("FPE");
    acronyms->insert("FPS");
    acronyms->insert("H2SV1");
    acronyms->insert("IOR");
    acronyms->insert("ITC");
    acronyms->insert("MPEG");
    acronyms->insert("MRV");
    acronyms->insert("NP");
    acronyms->insert("SC");
    acronyms->insert("SDL");
    acronyms->insert("SM");
    acronyms->insert("SV");
    acronyms->insert("SVCOMP");
    acronyms->insert("SVEM");
    acronyms->insert("TCP");
    acronyms->insert("TRM");
    acronyms->insert("VODB");
    acronyms->insert("VB");
    acronyms->insert("VCO");
    acronyms->insert("VCC4");
    acronyms->insert("VCEM");
    acronyms->insert("VCX");
    acronyms->insert("WTA");
  }
}

// ######################################################################
// ############# RefHolder implementation
// ######################################################################

RefHolder::RefHolder()
{}

RefHolder::~RefHolder()
{}

// ######################################################################
// ############# ModelParamBase implementation
// ######################################################################

ModelParamBase::ModelParamBase(const ParamFlag flags)
  :
  itsFlags(flags)
{}

ModelParamBase::~ModelParamBase()
{}

std::string ModelParamBase::getNameWithSpaces() const
{
  pthread_once(&acronyms_init_once, &acronyms_init);
  return camelCaseToSpaces(this->getName(), acronyms);
}

// ######################################################################
// ############# OptionedModelParam implementation
// ######################################################################

OptionedModelParam::OptionedModelParam(const ParamFlag flags)
  :
  ModelParamBase(flags)
{}

OptionedModelParam::~OptionedModelParam()
{}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
