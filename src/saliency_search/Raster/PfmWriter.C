/*!@file Raster/PfmWriter.C Write pfm image files */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PfmWriter.C $
// $Id: PfmWriter.C 6956 2006-08-08 18:20:23Z rjpeters $
//

#ifndef RASTER_PFMWRITER_C_DEFINED
#define RASTER_PFMWRITER_C_DEFINED

#include "Raster/PfmWriter.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"

#include <fstream>

using std::deque;
using std::string;

PfmWriter::PfmWriter() {}

PfmWriter::~PfmWriter() {}

string PfmWriter::writeFrame(const GenericFrame& image,
                             const string& fname)
{
  PfmWriter::writeFloat(image.asFloat(), fname);

  return fname;
}

void PfmWriter::writeFloat(const Image<float>& image,
                           const string& fname,
                           const deque<string>& tagName,
                           const deque<string>& tagValue)
{
  if (tagName.size() != tagValue.size())
    LFATAL("the number of tag names and tag values must be equal");

  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open PFM file '%s' for writing.", fname.c_str());

  ofs << "PF\n" << image.getWidth() << ' ' << image.getHeight() << '\n';

  // write extra tags if supplied
  if (tagName.size() > 0)
    {
      deque<string>::const_iterator tagNameItr  = tagName.begin();
      deque<string>::const_iterator tagValueItr = tagValue.begin();
      while (tagNameItr != tagName.end())
        {
          ofs << '!' << *tagNameItr << '\n'
              << *tagValueItr << '\n';
          ++tagNameItr; ++tagValueItr;
        }
    }

  // 1.0 is our "max gray" value for the PFM format:
  ofs << "1.0\n";

  ofs.write(reinterpret_cast<const char*>(image.getArrayPtr()),
            image.getSize() * sizeof(float));
  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

void PfmWriter::writeFloat(const Image<float>& image,
                           const string& fname)
{
  deque<string> nothing;
  PfmWriter::writeFloat(image, fname, nothing, nothing);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_PFMWRITER_C_DEFINED
