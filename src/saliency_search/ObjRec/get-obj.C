/*!@file ObjRec/get-obj.C get an object from a mask
*/

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/get-obj.C $
// $Id: get-obj.C 7063 2006-08-29 18:26:55Z rjpeters $
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/ColorOps.H"
#include "Image/ShapeOps.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/Transforms.H"
#include "Media/FrameSeries.H"
#include "Raster/Raster.H"
#include "Util/log.H"
#include "Util/Timer.H"
#include "Learn/SOFM.H"
#include "GUI/XWinManaged.H"
#include "CMapDemo/SaliencyCMapMT.H"
#include "SIFT/ScaleSpace.H"
#include "SIFT/VisualObject.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObjectDB.H"
#include "Image/FourierEngine.H"
#include "RCBot/Motion/MotionEnergy.H"

#include <signal.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>




int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager manager("Test SIFT");
  XWinManaged xwin(Dims(704,480), -1, -1, "Get Object");

  if (manager.parseCommandLine(
        (const int)argc, (const char**)argv, "<Image> <Mask>", 2, 2) == false);

  char filename[255];
  for (int i=0; i<100; i++)
  {

    sprintf(filename, "%s/RPGData%03d.ppm", manager.getExtraArg(0).c_str(), i);
    struct stat statBuf;
    if (!stat(filename, &statBuf))
    {
      Image<PixRGB<byte> > img = Raster::ReadRGB(filename);

      sprintf(filename, "%s/RPGData%03dTarget.pgm", manager.getExtraArg(1).c_str(), i);
      Image<PixRGB<byte> > imgMask = Raster::ReadRGB(filename);


      Image<PixRGB<byte> >::iterator imgPtr = img.beginw();
      Image<PixRGB<byte> >::const_iterator imgMaskPtr = imgMask.beginw(),
        imgMaskStop = imgMask.end();

      while (imgMaskPtr != imgMaskStop)
      {
        if (imgMaskPtr->p[0] == 0.0f){
          *imgPtr = PixRGB<byte>((byte)0, (byte)0, (byte)0);
        }
        imgMaskPtr++; imgPtr++;
      }

      //xwin.drawImage(img);

      sprintf(filename, "%s/out%03dTarget.ppm", manager.getExtraArg(1).c_str(), i);
      Raster::WriteRGB(img, filename);

    }
  }



  return 0;

}
