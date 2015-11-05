/*! @file SIFT/app-match-SIFT-database.C Match an image against a database
  of VisualObject */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/app-match-SIFT-database.C $
// $Id: app-match-SIFT-database.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "Raster/Raster.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"
#include "Image/DrawOps.H"

/*! Load a database, an image, and find best matches. */
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;

  // check command-line args:
  if (argc < 3 || argc > 4)
    LFATAL("USAGE: app-match-SIFT-database <dbname.vdb> <image.png> "
           "[<fused.png>]");

  // load the database:
  VisualObjectDB vdb;
  if (vdb.loadFrom(argv[1]) == false)
    LFATAL("Cannot operate without a valid database.");

  // get input image:
  Image< PixRGB<byte> > colim = Raster::ReadRGB(argv[2]);

  // create visual object and extract keypoints:
  rutz::shared_ptr<VisualObject> vo(new VisualObject(argv[2], argv[2], colim));

  // get the matching objects:
  std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
  const uint nmatches = vdb.getObjectMatches(vo, matches, VOMA_KDTREEBBF);

  // prepare the fused image:
  Image< PixRGB<byte> > mimg;
  std::vector<Point2D<int> > tl, tr, br, bl;

  // if no match, forget it:
  if (nmatches == 0U)
    LINFO("### No matching object found.");
  else
    {
      // let the user know about the matches:
      for (uint i = 0; i < nmatches; i ++)
        {
          rutz::shared_ptr<VisualObjectMatch> vom = matches[i];
          rutz::shared_ptr<VisualObject> obj = vom->getVoTest();

          LINFO("### Object match with '%s' score=%f",
                obj->getName().c_str(), vom->getScore());

          // add to our fused image if desired:
          if (argc > 3)
            {
              mimg = vom->getTransfTestImage(mimg);

              // also keep track of the corners of the test image, for
              // later drawing:
              Point2D<int> ptl, ptr, pbr, pbl;
              vom->getTransfTestOutline(ptl, ptr, pbr, pbl);
              tl.push_back(ptl); tr.push_back(ptr);
              br.push_back(pbr); bl.push_back(pbl);
            }
        }

      // do a final mix between given image and matches:
      if (mimg.initialized())
        {
          mimg = Image<PixRGB<byte> >(mimg * 0.5F + colim * 0.5F);

          // finally draw all the object outlines:
          PixRGB<byte> col(255, 255, 0);
          for (uint i = 0; i < tl.size(); i ++)
            {
              drawLine(mimg, tl[i], tr[i], col, 1);
              drawLine(mimg, tr[i], br[i], col, 1);
              drawLine(mimg, br[i], bl[i], col, 1);
              drawLine(mimg, bl[i], tl[i], col, 1);
            }
        }
    }

  // save result image if desired:
  if (argc > 3)
    {
      if (mimg.initialized() == false)
        mimg = Image< PixRGB<byte> >(colim * 0.5F);
      Raster::WriteRGB(mimg, std::string(argv[3]));
    }

  return 0;
}
