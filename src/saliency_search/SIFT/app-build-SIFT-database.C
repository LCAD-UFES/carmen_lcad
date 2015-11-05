/*! @file SIFT/app-build-SIFT-database.C Build a database of VisualObject */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/app-build-SIFT-database.C $
// $Id: app-build-SIFT-database.C 7902 2007-02-14 01:20:09Z harel $
//

#include "Raster/Raster.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectDB.H"

/*! Load a database, enrich it with new VisualObject entities
  extracted from the given images, and save it back. */
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;

  // check command-line args:
  if (argc < 3)
    LFATAL("USAGE: app-build-SIFT-database <dbname.vdb> <image1.png> "
           "... <imageN.png>");

  // load the database:
  VisualObjectDB vdb;
  vdb.loadFrom(argv[1]);

  // compute keypoints for all objects:
  for (int i = 0; i < argc-2; i ++)
    {
      std::string name(argv[i+2]);
      uint idx = name.rfind('.'); if (idx > 0) name = name.substr(0, idx);
      LINFO("##### Processing object %d/%d: %s", i+1, argc-2, name.c_str());

      // get input image:
      Image< PixRGB<byte> > colim = Raster::ReadRGB(argv[i+2]);

      // create visual object and extract keypoints:
      rutz::shared_ptr<VisualObject> vo(new VisualObject(name, argv[i+2], colim));

      // add the object to the db:
      if (vdb.addObject(vo))
        LINFO("Added VisualObject '%s' to database.", vo->getName().c_str());
      else
        LERROR("FAILED adding VisualObject '%s' to database -- IGNORING",
               vo->getName().c_str());
    }

  // save the resulting database:
  vdb.saveTo(argv[1]);

  return 0;
}
