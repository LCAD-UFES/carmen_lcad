/*!@file ModelNeuron/Rectify.H Class declarations for a module which
   is a simple rectifier */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Rectify.H $

#ifndef MODELNEURON_RECTIFY_H_DEFINED
#define MODELNEURON_RECTIFY_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "ModelNeuron/SimUnit.H"

namespace nsu
{
  // ######################################################################
  // A simple rectification module. For this simple of a computation, it
  // is better just to use a rate decoder that performs rectification,
  // but this is included as a simple example.
  // ######################################################################
  class Rectify: public SimUnitDerived<Rectify>
  {
    public:  
      //! Constructor with default params
      Rectify(const SimTime& timestep, const std::string& name = "Rectify", const std::string& units = "");
    
      //! destructor
      ~Rectify();
  
    private:
      //! integrate a time step
      const double doIntegrate(const SimTime& dt, const double& ine, const double& inh);
  };

  // ######################################################################
  // register our objects
  // ######################################################################
  namespace 
  {
    typedef SimUnit::Factory RectFactory;
    typedef SimUnit::Creator RectCreator;
    //define creation functions
    struct RegisterRectify
    {
        RegisterRectify()
        {
          RectFactory::instance().add("Rectify", RectCreator::make<Rectify>(SimTime::MSECS(1)));
        }
    };
    static RegisterRectify registerrect;  
  }

  // ######################################################################
  // ##### implementation for Rectify
  // ######################################################################
  inline
  Rectify::Rectify(const SimTime& timestep, const std::string& name, const std::string& units) :
      SimUnitDerived<Rectify>(timestep, SimUnit::NORMAL, name, units)
  { 
  }

  // ######################################################################
  inline
  Rectify::~Rectify()
  { 
  }

  // ######################################################################
  inline
  const double Rectify::doIntegrate(const SimTime& dt, const double& ine, const double& inh)
  {
    const double in = ine + inh;
    return (in > 0.0) ? in : 0.0;
  }
}
#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
