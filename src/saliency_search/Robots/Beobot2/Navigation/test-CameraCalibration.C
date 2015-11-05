/*!@file/Robots/Beobot2/Navigation/test-CameraCalibration.C  
calibrate image coordinate to ground plane coordinate mapping */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/test-CameraCalibration.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "GUI/XWinManaged.H"
#include "Image/DrawOps.H"

void getLocations
(Image<PixRGB<byte> > image, rutz::shared_ptr<XWinManaged> win);

// ######################################################################
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  ModelManager manager("test-CameraCalibration");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);
  
  if (manager.parseCommandLine((const int)argc, (const char**)argv,
                               "", 0, 0) == false)
    return(1);

  rutz::shared_ptr<XWinManaged> win
    (new XWinManaged(Dims(100, 100), 0, 0, "test-CameraCalibration"));

  manager.start();
  
  ifs->updateNext(); 
  Image<PixRGB<byte> > ima = ifs->readRGB();
  if(ima.initialized()) getLocations(ima, win);
  else LINFO("Not initialized");
}

// ######################################################################
void getLocations
(Image<PixRGB<byte> > image, rutz::shared_ptr<XWinManaged> win)
{
  Dims d  = win->getDims(); Dims id = image.getDims();
  win->setDims(id);
  win->drawImage(image,0,0);

  LINFO("please click on the image");
  Point2D<int> pos(-1, -1);
  while(true)
    {
      pos = win->getLastMouseClick();
      if(pos != Point2D<int>(-1,-1))
        {
          LINFO("pos: %d %d", pos.i, pos.j);
          Image< PixRGB<byte> > disp = image;
          drawCross(disp, pos, PixRGB<byte>(0,0,255), 30, 2);
          win->drawImage(disp,0,0);
          break;
        }
    }

  // save the location point
  // FILE *gtfp; 
  // LINFO("[%3d] gt file: %s: %d %d", 
  //       frame, filename.c_str(), pos.i, pos.j);
  // if((gtfp = fopen(filename.c_str(),"at")) == NULL) LFATAL("not found");
  // fputs(sformat("%5d %4d %4d \n", frame, pos.i, pos.j).c_str(), gtfp);
  // fclose (gtfp);

  // NOTE: maybe a way where if the annotator wants to quit
  //       whatever is already done within a clip is still saved
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
