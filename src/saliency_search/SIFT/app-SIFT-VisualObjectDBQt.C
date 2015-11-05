/*!@file SIFT/app-SIF-VisualObjectDBQt.C Simple Qt-based SIFT object database browser */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Apps/BorderWatch/BorderWatch.C $
// $Id: BorderWatch.C 13039 2010-03-23 02:06:32Z itti $

#include "QtUtil/Util.H" // for argv2qt()
#include "SIFT/VisualObjectDBQt.qt.H"
#include <QtGui/QApplication>
#include "Util/log.H"

//! SIFT VisualObjectDB GUI
int main(int argc, const char **argv)
{
  MYLOGVERB = LOG_DEBUG;

  // create a QApplication:
  int qtargc = 1; const char* qtargv[1] = { "app-SIFT-VisualObjectDBQt" };
  QApplication a(qtargc, argv2qt(qtargc, qtargv));

  // and a widget:
  VisualObjectDBQt b;
  b.setWindowTitle("iLab USC -- SIFT VisualObjectDB GUI");
  b.show();

  // main loop for QApplication:
  const int ret = a.exec();

  // cleanup and exit:
  return ret;
}
