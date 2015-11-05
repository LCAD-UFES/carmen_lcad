/*! @file SIFT/app-SIFT-panorama.C Build a panorama from several overlapping images */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/app-SIFT-panorama.C $
// $Id: app-SIFT-panorama.C 14118 2010-10-09 07:36:25Z itti $
//

#include "Raster/Raster.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"
#include "Image/ImageSet.H"
#include "Util/Timer.H"
#include <iostream>

//! Stitch images into a panorama.
/*! The images given on the command-line should overlap by some amount
  and should be taken in sequence; e.g., the first one may be the
  leftmost, the second one shifted slightly rightwards compared to the
  first, the third slightly shifted rightwards compared to the second,
  etc. Indeed this program is very simple and will just stitch the
  first image to the second, then the second to the third, etc,
  without enforcing any further consistency. */
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_DEBUG;

  // check command-line args:
  if (argc < 4)
    LFATAL("USAGE: app-SIFT-panorama <result.png> "
           "<image1.png> ... <imageN.png>");

  // loop over the images and get all the SIFTaffines:
  ImageSet< PixRGB<byte> > images;

  const char *nam1 = argv[2];
  Image< PixRGB<byte> > im1 = Raster::ReadRGB(nam1);
  images.push_back(im1);
  rutz::shared_ptr<VisualObject> vo1(new VisualObject(nam1, "", im1));
  std::vector<SIFTaffine> affines;
  SIFTaffine comboaff; // default constructor is identity
  affines.push_back(comboaff);
  int minx = 0, miny = 0, maxx = im1.getWidth()-1, maxy = im1.getHeight()-1;

  for (int i = 3; i < argc; i ++)
    {
      const char *nam2 = argv[i];
      Image< PixRGB<byte> > im2 = Raster::ReadRGB(nam2);
      images.push_back(im2);
      rutz::shared_ptr<VisualObject> vo2(new VisualObject(nam2, "", im2));

      // compute the matching keypoints:
      Timer tim(1000000);
      VisualObjectMatch match(vo1, vo2, VOMA_SIMPLE);
      uint64 t = tim.get();

      LINFO("Found %u matches between %s and %s in %.3fms",
            match.size(), nam1, nam2, float(t) * 0.001F);

      // let's prune the matches:
      uint np = match.prune();
      LINFO("Pruned %u outlier matches.", np);

      // show our final affine transform:
      SIFTaffine aff = match.getSIFTaffine();
      std::cerr<<aff;

      // compose with the previous affines and store:
      comboaff = comboaff.compose(aff);
      affines.push_back(comboaff);

      // update panorama boundaries, using the inverse combo aff to
      // find the locations of the four corners of our image in the
      // panorama:
      if (comboaff.isInversible() == false) LFATAL("Oooops, singular affine!");
      SIFTaffine iaff = comboaff.inverse();
      const float ww = float(im2.getWidth() - 1);
      const float hh = float(im2.getHeight() - 1);
      float xx, yy; int x, y;

      iaff.transform(0.0F, 0.0F, xx, yy); x = int(xx+0.5F); y = int(yy+0.5F);
      if (x < minx) minx = x; if (x > maxx) maxx = x;
      if (y < miny) miny = y; if (y > maxy) maxy = y;

      iaff.transform(ww, 0.0F, xx, yy); x = int(xx+0.5F); y = int(yy+0.5F);
      if (x < minx) minx = x; if (x > maxx) maxx = x;
      if (y < miny) miny = y; if (y > maxy) maxy = y;

      iaff.transform(0.0F, hh, xx, yy); x = int(xx+0.5F); y = int(yy+0.5F);
      if (x < minx) minx = x; if (x > maxx) maxx = x;
      if (y < miny) miny = y; if (y > maxy) maxy = y;

      iaff.transform(ww, hh, xx, yy); x = int(xx+0.5F); y = int(yy+0.5F);
      if (x < minx) minx = x; if (x > maxx) maxx = x;
      if (y < miny) miny = y; if (y > maxy) maxy = y;

      // get ready for next pair:
      im1 = im2; vo1 = vo2; nam1 = nam2;
    }

  // all right, allocate the panorama:
  LINFO("x = [%d .. %d], y = [%d .. %d]", minx, maxx, miny, maxy);
  const int w = maxx - minx + 1, h = maxy - miny + 1;
  LINFO("Allocating %dx%d panorama...", w, h);
  if (w < 2 || h < 2) LFATAL("Oooops, panorama too small!");
  Image< PixRGB<byte> > pano(w, h, ZEROS);
  Image< PixRGB<byte> >::iterator p = pano.beginw();

  // let's stitch the images into the panorama. This code is similar
  // to that in VisualObjectMatch::getTransfTestImage() but modified
  // for a large panorama and many images:
  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
      {
        // compute the value that should go into the current panorama
        // pixel based on all the images and affines; this is very
        // wasteful and may be optimized later:
        PixRGB<int> val(0); uint n = 0U;

        for (uint k = 0; k < images.size(); k ++)
          {
            // get transformed coordinates for image k:
            float u, v;
            affines[k].transform(float(i + minx), float(j + miny), u, v);

            // if we are within bounds of image k, accumulate the pix value:
            if (images[k].coordsOk(u, v))
              {
                val += PixRGB<int>(images[k].getValInterp(u, v));
                ++ n;
              }
          }

        if (n > 0) *p = PixRGB<byte>(val / n);

        ++ p;
      }

  // save final panorama:
  Raster::WriteRGB(pano, std::string(argv[1]));
  LINFO("Done.");

  return 0;
}
