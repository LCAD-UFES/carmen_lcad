/*!@file Media/MgzJOutputStream.C Write frames to a .mgzj file */

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
// Primary maintainer for this file: Randolph Voorhies <voories at usc dot edu>

#include "Media/MgzJOutputStream.H"
#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/MgzJEncoder.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"


const ModelOptionDef OPT_MgzJOutputStreamCompLevel = 
{ MODOPT_ARG(int), "MgzJOutputStreamCompLevel", &MOC_OUTPUT, OPTEXP_CORE,
  "Gzip compression level to use for the MgzJ file.\n"
    "  Higher values use better compression at the cost of speed and memory,"
    "  while a value of 0 will not compress the frames at all.",
  "output-mgzj-complev", '\0', "<1..9>", "6" };

// ######################################################################
MgzJOutputStream::MgzJOutputStream(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  LowLevelEncoderMap(mgr, descrName, tagName),
  itsCompLevel(&OPT_MgzJOutputStreamCompLevel, this),
  itsStem()
{}


// ######################################################################
MgzJOutputStream::~MgzJOutputStream()
{ }


// ######################################################################
void MgzJOutputStream::setConfigInfo(const std::string& filestem)
{ 

  this->setFileStem(filestem);
}

// ######################################################################
void MgzJOutputStream::setFileStem(const std::string& s)
{
  itsStem = s;
}
// ######################################################################
rutz::shared_ptr<LowLevelEncoder>   
MgzJOutputStream::makeEncoder(const GenericFrameSpec& spec,
                              const std::string& shortname,
                              const FrameInfo& auxinfo)
{
  std::string stem = itsStem.substr(0,itsStem.rfind('.'));
  

  return rutz::shared_ptr<MgzJEncoder>
    (new MgzJEncoder(stem+shortname+".mgzJ", itsCompLevel.getVal()));
}

