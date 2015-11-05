/*!@file SIFT/SIFTegomotion.H  Calculates egomotion given a set of
  correspondence match */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/SIFTegomotion.H $
// $Id: SIFTegomotion.H 6990 2006-08-11 18:13:51Z rjpeters $
//

#ifndef SIFTEGOMOTION_H_DEFINED
#define SIFTEGOMOTION_H_DEFINED

#include "SIFT/VisualObjectMatch.H"
#include "GUI/XWinManaged.H"
#include <complex>
#include "SIFT/CameraIntrinsicParam.H"

//! A class that calculates 3D egomotion from a set of matches
class SIFTegomotion {
public:

  //! Constructor
  SIFTegomotion(rutz::shared_ptr<VisualObjectMatch> match,
                rutz::shared_ptr<CameraIntrinsicParam> cip = rutz::shared_ptr<CameraIntrinsicParam>(),
                rutz::shared_ptr<XWinManaged> matchWin = rutz::shared_ptr<XWinManaged>());

  inline Image<double> getItsVel() const;
  inline Image<double> getItsOmega() const;

  void print(Image<double> img, const std::string& name = "name");

private:

  // calculate egomotion
  void calc();
  void leastSquaresAlgebraic();
  void sevenPointAlgorithm();

  Image<double> constraint2(Image<double> T1);
  Image<double> constraint2(Image<double> T1, Image<double> T2);
  void checkConstraint2(Image<double> U, Image<double> T);

  Image<double> getThetaVelOmega(Image<double> vel, Image<double> omega);
  Image<double> getVel(Image<double> T);
  Image<double> getOmega(Image<double> T);

  Image<double> getThetaCW(Image<double> C, Image<double> W);
  Image<double> getC(Image<double> T);
  Image<double> getW(Image<double> T);

  void getUncalibCW(Image<double> T);

  void setCuMatch();
  Image<double> getU(int nPoint = -1);

  uint solveCubic(double p, double q, double r, double s,
                    std::vector<std::complex<double> >& x);

  //! the match the egomotion is based upon
  rutz::shared_ptr<VisualObjectMatch> itsVoMatch;
  std::vector<KeypointMatch> itsCuMatch; //!< its unique coordinate matches
  rutz::shared_ptr<CameraIntrinsicParam> itsCip;
  rutz::shared_ptr<XWinManaged> itsMatchWin;

  float itsXdot;      //!< X component of the velocity
  float itsYdot;      //!< Y component of the velocity
  float itsZdot;      //!< Z component of the velocity

  Image<double> itsVel;
  Image<double> itsOmega;

};

// ######################################################################
// SIFTegomotion I/O:
// ######################################################################


// ######################################################################
// ########## Implementation of inline functions
// ######################################################################

inline Image<double> SIFTegomotion::getItsVel() const
{ return itsVel; }

inline Image<double> SIFTegomotion::getItsOmega() const
{ return itsVel; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
