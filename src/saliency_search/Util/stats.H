/*!@file Util/stats.H STATS classes */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/stats.H $
// $Id: stats.H 4663 2005-06-23 17:47:28Z rjpeters $
//


// ############################################################
// ############################################################
// ##### ---STATS---
// ##### Some basic statistical methods:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ############################################################
// ############################################################

#ifndef STATS_H_DEFINED
#define STATS_H_DEFINED

#include "Util/Assert.H"

#include <vector>

template <class T> class stats
{
private:
public:
  //! Constructor
  stats();
  //! Destructor
  ~stats();
  //! some bools for ASSERTION testing
  bool GGC;
  //basic holders for conveniance
  //! X bar - the population mean for x
  T Xb;
  //! Y bar - the population mean for y
  T Yb;
  //! S squared - population measure for variance
  T S2;
  //! S - population measure for varaince
  T Sx,Sy,S;
  //! r - the corralation coefficent
  T r;
  //! b - the regression coefficent slope
  T b;
  //! probability of A or B
  T PA, PB;
  //! decision boundrys
  T D, Dprime;
  //! SStotal, SSwithin, SSbetween
  T SStotal, SSwithin, SSbetween, DFwithin, DFbetween, MSwithin, MSbetween, F;
  //! The populational mean
  T mean(std::vector<T> &X);
  //! The standard deviation of the population
  T findS(std::vector<T> &X, T Xbar);
  //! The standard deviation of the population, adjust for negative numbers
  T findS(std::vector<T> &X, T Xbar, T adj);
  //! basic pearson r linear regression
  T rRegression(std::vector<T> &X, std::vector<T> &Y);
  //! regression coefficent slope
  /*! This can be use to fit the line of regression as
    Y' = b*(X - Xbar) + YBar
  */
  T bRegression(std::vector<T> &X, std::vector<T> &Y);
  //! raw score regression coefficent
  /*! This can be use to fit the line of regression as
    Y' = Bxy*(X - Xbar) + YBar
  */
  T Bxy(T r, T Sx, T Sy);
  //! This is a simple ANOVA for two groups
  /*! input the raw scores for the two groups, let it run
    will run on assumption of SStotal = SSwithin + SSbetween
  */
  T simpleANOVA(std::vector<T> &X, std::vector<T> &Y);
  //! find the decision boundry as described in Itti(2000) PhD Thesis pg. 145-8
  /*! Find a decision in the general gaussian case
    input mu's sigma's and P(X)'s for two events. This will return D. Use
    getDPrime() to get D' following this command.
    @param mu1 mean for condition 1
    @param mu2 mean for condition 2
    @param sigma1 std dev for condition 1
    @param sigma2 std dev for condition 2
    @param PofA probability of A (0 to 1) PofB is determined as 1 - PofA
  */
  T decisionGGC(T mu1, T mu2, T sigma1, T sigma2, T PofA);
  //! Return D' after running descisionGGC
  T getDPrime();
  //! Get the probability of Error for a 2AFC paradigm
  /*! take the decision boundrys and find the probability of error
    from them. This is for Two Alternative forced choice paradigm
    @param mu1 mean for condition 1
    @param mu2 mean for condition 2
    @param sigma1 std dev for condition 1
 @param sigma2 std dev for condition 2
    @param PofA probability of A (0 to 1) PofB is determined as 1 - PofA
  */
  T getErrorGGC_2AFC(T mu1, T mu2, T sigma1, T sigma2);
  //! return the gaussian from f(x;mu,sigma) = guassian
  /*! Return a simple P(x) based upon the gaussian distribution with an
    input sample x and gaussian defined with mu as E(x) and sigma as
    E(x^2)
  */
  T gauss(T x, T mu,T sigma);

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif
