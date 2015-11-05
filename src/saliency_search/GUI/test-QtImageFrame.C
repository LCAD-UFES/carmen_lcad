/*!@file GUI/test-QtImageFrame.C test the QtImageFrame display */

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
// Primary maintainer for this file: Laurent Itti
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/test-QtImageFrame.C $
// $Id: test-QtImageFrame.C 12962 2010-03-06 02:13:53Z irock $
//

#ifdef INVT_HAVE_QT4
# include <QtGui/QApplication>
# include "GUI/QtImageFrame.qt.H"  // use Qt4 if available
#else
# include <qapplication.h>
# include "GUI/QtImageFrame.H"
#endif
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/FrameInfo.H"
#include "QtUtil/Util.H"

//! Trivial app that displays an image in a QtImageFrame
int main(const int argc, const char** argv)
{
  if (argc != 2) LFATAL("USAGE: %s <image.png>", argv[0]);

  // read the image:
  GenericFrame im = Raster::ReadFrame(argv[1]);

  // create Qt application:
  int qargc = 1; const char* qargv[1] = { "test-QtImageFrame" };
  QApplication *a = new QApplication(qargc, argv2qt(qargc, qargv));

  // create QtImageFrame:
  const Dims prefdims(640, 480), maxdims(1600, 1200);
  QtImageFrame *f = new QtImageFrame(0, prefdims, maxdims);
  const std::string title = sformat("test-QtImageFrame: %s", argv[1]);
  const FrameInfo auxinfo;

  // pass the image to the QtImageFrame:
  f->setFrame(im, title, 0, auxinfo);

  // show the frame:
  f->show();

  // main event loop:
  const int result = a->exec();

  LDEBUG("QApplication exit status %d", result);
  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
