/*!@file Util/stats.C STATS classes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/stats.C $
// $Id: stats.C 9632 2008-04-15 05:50:25Z mundhenk $
//

// ############################################################
// ############################################################
// ##### ---STATS---
// ##### Some basic statistical methods:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ############################################################
// ############################################################

#include "Util/Assert.H"
#include "Util/stats.H"
#include "Util/Types.H"
#include "Util/log.H"
#include <cmath>

template <class T>
stats<T>::stats()
{
  GGC = false;
}
template <class T>
stats<T>::~stats()
{
}



template <class T>
T stats<T>::mean(std::vector<T> &X)
{
  ASSERT(X.size() > 0);
  T Xi = 0;
  for(unsigned int i = 0; i < X.size(); i++)
  {
    Xi = Xi + X[i];
  }
  return Xb = Xi/X.size();
};

template <class T>
T stats<T>::findS(std::vector<T> &X, T Xbar)
{
  ASSERT(X.size() > 0);
  T Xi = 0;
  for(unsigned int i = 0; i < X.size(); i++)
  {
    Xi = (pow(X[i],2)/X.size()) + Xi;
  }
  S2 = Xi - pow(Xbar,2);
  if(S > 0)
    return S = sqrt(S2);
  else
    return S = 0;
};

template <class T>
T stats<T>::findS(std::vector<T> &X, T Xbar, T adj)
{
  ASSERT(X.size() > 0);
  T Xi = 0;
  for(unsigned int i = 0; i < X.size(); i++)
  {
    Xi = (pow((X[i]+adj),2)/X.size()) + Xi;
  }
  S2 = Xi - pow((Xbar+adj),2);
  if(S2 > 0)
    S = sqrt(S2);
  else
    S = 0;
  return S;
}

template <class T>
T stats<T>::rRegression(std::vector<T> &X, std::vector<T> &Y)
{
  ASSERT(X.size() == Y.size());
  T Xmean = mean(X);
  T Ymean = mean(Y);
  Sx = findS(X,Xmean);
  Sy = findS(Y,Ymean);
  T hold = 0;
  for(unsigned int i = 0; i < X.size(); i++)
  {
    hold = ((X[i] - Xmean)*(Y[i] - Ymean)) + hold;
  }
  return r = hold/(X.size()*Sx*Sy);
};

#if 0
// FIXME this function does not compile
template <class T>
T stats<T>::bRegression(std::vector<T> &X, std::vector<T> &Y)
{
  ASSERT(X.size() == Y.size());
  T Xmean = mean(X);
  T Ymean = mean(Y);
  Sx = S(X,Xmean); // <-- FIXME compilation error here
  Sy = S(Y,Ymean); // <-- FIXME compilation error here
  T Zxy = 0;
  for(unsigned int i = 0; i < X.size(); i++)
  {
    Zxy = (((X[i] - Xmean)/Sx)*((Y[i] - Ymean)/Sy)) + Zxy;
  }
  return b = Zxy/X.size();
};
#endif

template <class T>
T stats<T>::Bxy(T r, T Sx, T Sy)
{
  return r*(Sy/Sx);
};

template <class T>
T stats<T>::simpleANOVA(std::vector<T> &X, std::vector<T> &Y)
{
  ASSERT(X.size() == Y.size());
  float mean, meanX, meanY;
  float sumX = 0, sumY = 0;
  //find mean values
  for(unsigned int i = 0; i < X.size(); i++)
  {
    sumX += X[i];
    sumY += Y[i];
  }
  meanX = sumX/X.size();
  meanY = sumY/Y.size();
  mean = (sumY+sumX)/(X.size()+Y.size());
  //find SSwithin and SStotal
  SSwithin = 0;
  SStotal = 0;
  for(unsigned int i = 0; i < X.size(); i++)
  {
    SSwithin += pow((X[i]-meanX),2);
    SSwithin += pow((Y[i]-meanY),2);

    SStotal += pow((X[i]-mean),2);
    SStotal += pow((Y[i]-mean),2);
  }
  //find SSbetween
  SSbetween = 0;
  SSbetween = X.size() * (pow((meanX - mean),2));
  SSbetween += Y.size() * (pow((meanY - mean),2));

  //create anova table stuff
  DFwithin = (X.size()+Y.size())-2;
  DFbetween = 1;
  MSbetween = (SSbetween/DFbetween);
  MSwithin = (SSwithin/DFwithin);
  return F = MSbetween/MSwithin;
}

#if 0
// FIXME this function does not compile
template <class T>
T stats<T>::decisionGGC(T mu1, T mu2, T sigma1, T sigma2, T PofA)
{
  T PofB = 1 - PofA;

  //find parts of the equation first
  T LeftTop = (mu2*pow(sigma1,2))-(mu1*pow(sigma2,2));
  T Bottom = pow(sigma1,2)-pow(sigma2,2);
  T logVal = log((PofB*sigma1)/(PofA*sigma2));
  T preLog = pow((mu1-mu2),2)+(2*(pow(sigma1,2)-pow(sigma2,2)));

  //find D and D'
  // FIXME error here ('sgrt' unknown)...
  D = (LeftTop - ((sigma1*sigma2)*sgrt(preLog*logVal)))/Bottom;
  // FIXME and here ('sgrt' unknown)...
  Dprime = (LeftTop + ((sigma1*sigma2)*sgrt(preLog*logVal)))/Bottom;
  GGC = true;
  return D;
}
#endif

template <class T>
T stats<T>::getDPrime()
{
  ASSERT(GGC);
  return Dprime;
}

template <class T>
T stats<T>::getErrorGGC_2AFC(T mu1, T mu2, T sigma1, T sigma2)
{
  LINFO("INPUT 2AFC u1 = %f, u2 = %f, s1 = %f, s2 = %f",mu1,mu2,sigma1,sigma2);
  float temp = fabs((mu1-mu2)/(sqrt(2*(pow(sigma1,2)+pow(sigma2,2)))));
  LINFO("ERFC(%f)",temp);
  return erfc(temp)/2;
}

template <class T>
T stats<T>::gauss(T x, T mu,T sigma)
{
  return (1/(sqrt(2*3.14159*pow(sigma,2))))*exp((-1*pow((x-mu),2))/(2*pow(sigma,2)));
}


template class stats<float>;
template class stats<double>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
