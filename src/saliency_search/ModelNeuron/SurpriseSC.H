/*!@file Surprise/SurpriseSC.H a local (single-point) model of surprise */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SurpriseSC.H $


#ifndef SURPRISESC_H_DEFINED
#define SURPRISESC_H_DEFINED

#include "Image/Point2D.H"
#include "Image/Image.H"
#include "Util/MathFunctions.H"

// ######################################################################
//! A local (single-point) surprise model
/*! This the base class and it cannot be implemented as some of its
    functions are purely virtual. It defines the basic interface. See
    the derivative classes for actual implementations. */
class SurpriseSC
{
  public:
    //! Constructor
    /*! Build a new SurpriseSC object and initialize it as if an
        infinitely long sequence of samples had been previously received
        by the model.
        @param updatefac factor by which the current model is combine with
        a new model on an update. Valid values are in [0..1]
        @param sampleval the value of the samples previously received.
        @param samplevar a variance expressing the intrinsic variability
        in the measurement of those samples. */
    SurpriseSC(const double updatefac = 0.7, const double sampleval = 0.0, const double noise = 0.01, const bool usekl = true);

    // default copy-construction and copy operators OK.

    //! Virtual destructor ensures proper destruction of derived classes
    ~SurpriseSC();

    //! Change intial state parameters
    void init(const double updatefac = 0.7, const double sampleval = 0.0, const double noise = 0.01, const bool usekl = true);

    //! Reset our update factor
    void resetUpdFac(const double updfac);

    //! Compute surprise between us and another model
    double input(const double& sample);

    //! get our mean
    double getMean() const;

    //! get our variance
    double getVar() const;

    //! get our UpdateFac
    double getUpdateFac() const;

  protected:
    double itsUpdateFac;  //!< our update factor
    double itsInitialVal; //!< our initial value
    double itsAlpha;      //!< our alpha
    double itsBeta;       //!< our beta
    double itsNoise;      //!< the noise parameter
    bool itsUseKL;           //!< use KL in our output
};

// ######################################################################
// surprise SC map
// ######################################################################
class SurpriseMapSC : public Image<SurpriseSC>
{
public:
    //! Uninitialized constructor
    SurpriseMapSC();

    //! initialize with size
    SurpriseMapSC(const uint x, const uint y, const SurpriseSC& sc);

    // get input
    void input(const Image<double> & image);

    // get output
    Image<double> getOutput();

  private:
    Image<double> itsOut;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
