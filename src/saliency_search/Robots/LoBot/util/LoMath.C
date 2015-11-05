/**
   \file  Robots/LoBot/util/LoMath.C
   \brief Math functions.
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoMath.C $
// $Id: LoMath.C 13628 2010-06-28 23:48:02Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/LoMath.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//---------------------------- BASIC STUFF ------------------------------

// Returns the next power of two >= given number
int next_power_of_two(int n)
{
   const int m = abs(n) ;

   int p = 1 ;
   while (p < m)
      p <<= 1 ;

   return (n < 0) ? -p : p ;
}

//-------------------------- FLOATING POINT -----------------------------

// Round a floating-point number to the nearest integer
int round(float t)
{
   float rounder = (t < 0) ? -0.5f : 0.5f ;
   return static_cast<int>(t + rounder) ;
}

//------------------------------ ANGLES ---------------------------------

// Clamp an angle (specified in degrees) to lie in [0, 360]
float clamp_angle(float angle)
{
   float A = angle - 360 * static_cast<int>(angle/360) ;
   return (A < 0) ? 360 + A : A ;
}

//-------------------------- RANDOM NUMBERS -----------------------------

// Return a random number in the specified range
int random(int min, int max)
{
   float n = static_cast<float>(std::rand())/RAND_MAX ;
   return static_cast<int>(min + n * (max - min)) ;
}

// Return a random number in the specified range
float randomf(float min, float max)
{
   float n = static_cast<float>(std::rand())/RAND_MAX ;
   return min + n * (max - min) ;
}

//-------------------- LOGARITHMS AND EXPONENTIALS ----------------------

// Convert a [0,1] probability to its equivalent log-odds form
float prob_to_log_odds(float p)
{
   return ln(p/(1 - p)) ;
}

// Convert log-odds likelihood to its equivalent [0,1] probability
float log_odds_to_prob(float log_odds)
{
   return 1/(1 + exp(-log_odds)) ;
}

//--------------------------- MISCELLANEOUS -----------------------------

// Evaluate Gaussian
float gaussian(float x, float mu, float sigma)
{
   const float sigma_inv = 1/sigma ;
   return 0.39894228f * sigma_inv * exp(-0.5f * sqr((x - mu) * sigma_inv)) ;
}

// Return a random number sampled from a triangular distribution centered
// at b.
//
// DEVNOTE: See page 124 of "Probabilistic Robotics" by Thrun, Burgard
// and Fox.
float sample_tri(float b)
{
   return /*sqrt(6)/2*/ 1.22474487f * (randomf(-b, b) + randomf(-b, b)) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
