/*! @file SIFT/ilabLoweSiftComp.C compare the SIFT alg to lowe's sift program */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/ilabLoweSiftComp.C $
// $Id: ilabLoweSiftComp.C 15310 2012-06-01 02:29:24Z itti $
//


#include "Image/Image.H"
#include "Image/Pixels.H"
#include "SIFT/ScaleSpace.H"
#include "SIFT/VisualObject.H"
#include "SIFT/Keypoint.H"
#include "Util/MathFunctions.H"
#include "Raster/Raster.H"
#include <stdlib.h>
#include <stdio.h>
#define USECOLOR true
#define TMPKEYFILE "/tmp/loweSift.key"

/* Read keypoints from the given file pointer and return the list of
   keypoints.  The file format starts with 2 integers giving the total
   number of keypoints and the size of descriptor vector for each
   keypoint (currently assumed to be 128). Then each keypoint is
   specified by 4 floating point numbers giving subpixel row and
   column location, scale, and orientation (in radians from -PI to
   PI).  Then the descriptor vector for each keypoint is given as a
   list of integers in range [0,255].

   changed to support Ilab format
*/

std::vector< rutz::shared_ptr<Keypoint> >& ReadKeys(const char *filename)
{

   int i, j, num, len, val;
   FILE *fp = fopen(filename, "r");
   if (!fp)
      LFATAL("Can not open %s", filename);

   std::vector< rutz::shared_ptr<Keypoint> > *keypoints = new std::vector< rutz::shared_ptr<Keypoint> >();

   if (fscanf(fp, "%d %d", &num, &len) != 2)
      LFATAL("Invalid keypoint file beginning.");

    if (len != 128)
       LFATAL("Keypoint descriptor length invalid (should be 128).");

    for (i = 0; i < num; i++) {

      float x, y, s, o;
      if (fscanf(fp, "%f %f %f %f", &y, &x, &s, &o) != 4)
         LFATAL("Invalid keypoint file format.");

      std::vector<byte> fv;

      for (j = 0; j < len; j++) {
         if (fscanf(fp, "%d", &val) != 1 || val < 0 || val > 255)
            LFATAL("Invalid keypoint file value.");
         fv.push_back((byte)val);
      }

      // create a keypoint:
      rutz::shared_ptr<Keypoint> newkey(new Keypoint(fv, x, y, s, o, 0));
      keypoints->push_back(newkey);
    }

    return *keypoints;
}



int main(const int argc, const char **argv)
{

   if (argc < 3)
      LFATAL("Usage: ilabSift <lowe's sift code> <image filename> ");


   //get the keypoints from Lowe's output file
   char cmd[255];
   sprintf(cmd, "%s < %s > %s", argv[1], argv[2], TMPKEYFILE);
   if (system(cmd) == -1)
      LFATAL("Can not run %s", cmd);

   std::vector< rutz::shared_ptr<Keypoint> > loweKeypoints = ReadKeys(TMPKEYFILE);
   LINFO("Lowe SIFT found: %" ZU " keypoints", loweKeypoints.size());


   //get the keypoints from the Ilab SIFT code
   Image< PixRGB<byte> > input = Raster::ReadRGB(argv[2]);
   rutz::shared_ptr<VisualObject>
     vo(new VisualObject("Test", "test", input,
                         Point2D<int>(-1,-1),
                         std::vector<float>(),
                         std::vector< rutz::shared_ptr<Keypoint> >(),
                         USECOLOR));
   std::vector< rutz::shared_ptr<Keypoint> > ilabKeypoints = vo->getKeypoints();

   LINFO("Ilab SIFT found: %" ZU " keypoints", ilabKeypoints.size());



   //find keypoint matches

   int numKeyDiff = 0;
   int numFvDiff = 0;
   for(unsigned int i=0; i<loweKeypoints.size(); i++){
      float lowe_x = loweKeypoints[i]->getX();
      float lowe_y = loweKeypoints[i]->getY();
      float lowe_s = loweKeypoints[i]->getS();
      float lowe_o = loweKeypoints[i]->getO() * -1; //lowe has this from -PI to PI

      //look for the closest keypoint
      double minDist = 9999;
      int minMatch = -1;
      for (unsigned int j=0; j<ilabKeypoints.size(); j++){
         float ilab_x = ilabKeypoints[j]->getX();
         float ilab_y = ilabKeypoints[j]->getY();
         float ilab_s = ilabKeypoints[j]->getS();
         float ilab_o = ilabKeypoints[j]->getO();

         double dist = sqrt(squareOf(ilab_x - lowe_x) +
                            squareOf(ilab_y - lowe_y) +
                            squareOf(ilab_s - lowe_s) +
                            squareOf(ilab_o - lowe_o) );
         if (dist < minDist){
            minDist = dist;
            minMatch = j;
         }
      }

      float fvDist = sqrt(loweKeypoints[i]->distSquared(ilabKeypoints[minMatch]));

      if (minDist > 1) numKeyDiff++; //Threshold for matches
      if (fvDist > 500) numFvDiff++; //Threshold for matches

     /* LINFO("keypoint match lowe(%0.2f,%0.2f,%0.2f,%0.2f) ilab(%0.2f,%0.2f,%0.2f,%0.2f) dist %0.2f fvDist = %f",
            lowe_x, lowe_y, lowe_s, lowe_o,
            ilabKeypoints[minMatch]->getX(), ilabKeypoints[minMatch]->getY(),
            ilabKeypoints[minMatch]->getS(), ilabKeypoints[minMatch]->getO(),
            minDist, fvDist);*/
   }
   LINFO("Number of diffrent keypoints %i", numKeyDiff);
   LINFO("Number of diffrent Feature Vectors %i", numFvDiff);


}

