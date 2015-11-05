/*!@file Raster/PfmParser.C Parse pfm image files. */

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
// Primary maintainer for this file: Laurent Itti <Itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PfmParser.C $
// $Id: PfmParser.C 8790 2007-09-28 22:24:10Z rjpeters $
//

#include "Raster/PfmParser.H"

#include "Util/Assert.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/FileUtil.H"
#include "Util/log.H"
#include "rutz/shared_ptr.h"

#include <istream>
#include <limits>
#include <string>
#include <deque>

using std::deque;
using std::string;

// ######################################################################
struct PfmParser::Rep
{
  Rep(const std::string& fname) :
    strm(openMaybeCompressedFile(fname)),
    mode('\0'), w(-1), h(-1), maxGrey(1), comments("")
  {
    ASSERT(strm.get() != 0);
  }

  rutz::shared_ptr<std::istream> strm;
  char mode;
  int w, h;
  float maxGrey;
  string comments;
  deque<string> tagName;
  deque<string> tagValue;
};

// ######################################################################
PfmParser::PfmParser(const std::string& fname) :
  rep(new Rep(fname))
{
  const int c = rep->strm->get();
  if (c != 'P')
    LFATAL("Missing magic number in pbm file '%s'.", fname.c_str());

  (*rep->strm) >> rep->mode >> std::ws;

  string temp;

  // copy and concatenate optional comment line(s) starting with '#'
  // into comments string
  while (rep->strm->peek() == '#')
    {
      // get the full line
      std::getline((*rep->strm), temp, '\n');
      rep->comments += temp;
    }

  (*rep->strm) >> rep->w >> rep->h >> std::ws;

  // check for extra optional tags
  while (rep->strm->peek() == '!')
    {
      // get the full line, remove leading !
      std::getline((*rep->strm), temp, '\n');
      temp = temp.substr(1);
      rep->tagName.push_back(temp);

      // the next line contains the data with this tag
      std::getline((*rep->strm), temp, '\n');
      rep->tagValue.push_back(temp);
    }

  (*rep->strm) >> rep->maxGrey;

  // read one more character of whitespace from the stream after maxGrey
  const int spc = rep->strm->get();
  if (!isspace(spc))
    LFATAL("Missing whitespace after maxGrey in pfm file '%s'.", fname.c_str());
  LDEBUG("PFM Reading Image: %s", fname.c_str());
}

// ######################################################################
PfmParser::~PfmParser()
{
  delete rep;
}

// ######################################################################
GenericFrameSpec PfmParser::getFrameSpec() const
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::GRAY_F32;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = Dims(rep->w, rep->h);
  result.floatFlags = 0;

  return result;
}

// ######################################################################
string PfmParser::getComments() const
{ return rep->comments; }

// ######################################################################
uint PfmParser::getTagCount() const
{ return rep->tagName.size(); }

// ######################################################################
bool PfmParser::getTag(uint tag, string& name, string& value) const
{
  if (tag < rep->tagName.size())
    {
      name  = rep->tagName[tag];
      value = rep->tagValue[tag];
      return true;
    }
  else
    return false;
}

// ######################################################################
GenericFrame PfmParser::getFrame()
{
  ASSERT(rep->mode == 'F');

  Image<float> img(rep->w, rep->h, NO_INIT);
  rep->strm->read(reinterpret_cast<char*>(img.getArrayPtr()),
                  img.getSize() * sizeof(float));
  return GenericFrame(img, /* flags */ 0);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
