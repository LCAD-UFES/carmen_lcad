/*!@file Raster/PfzParser.C Parse pfm image files. */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PfzParser.C $
// $Id: PfzParser.C 8027 2007-03-02 00:40:43Z rjpeters $
//

#include "Raster/PfzParser.H"

#include "Util/Assert.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/log.H"

#include <fstream>
#include <limits>
#include <string>
#include <deque>

// ######################################################################
struct PfzParser::Rep
{
  Rep(const char* fname) :
    strm(fname), mode('\0'), w(-1), h(-1), maxGrey(1), comments(""), tags(0)
  {}

  std::ifstream strm;
  std::stringstream os;
  char mode;
  int w, h;
  float maxGrey;
  std::string comments;
  std::deque<std::string> tagName;
  std::deque<std::string> tagValue;
  uint tags;
};

// ######################################################################
PfzParser::PfzParser(const char* fname) :
  rep(new Rep(fname))
{
  /* allocate inflate state */
  zstrm.zalloc   = Z_NULL;
  zstrm.zfree    = Z_NULL;
  zstrm.opaque   = Z_NULL;
  zstrm.avail_in = 0;
  zstrm.next_in  = Z_NULL;
  const int retI = inflateInit(&zstrm);
  if (retI != Z_OK)
    LFATAL("Unable to allocate memory and set up zlib compression in PFZ '%s'",
           fname);

  if (!rep->strm.is_open())
    LFATAL("Couldn't open file '%s' for reading.", fname);

  int c = rep->strm.get();
  if (c != 'P')
    LFATAL("Missing magic number in pbm file '%s'.", fname);

  rep->tagName.clear();
  rep->tagValue.clear();

  rep->strm >> rep->mode;

  rep->strm >> std::ws;

  // copy and concatenate optional UNCOMPRESSED
  // comment line(s) starting with '#'
  // into comments string

  std::string temp;

  while ( rep->strm.peek() == '#' )
    {
      //get the full line
      std::getline( rep->strm, temp, '\n');
      rep->comments += temp;
      //LINFO("%s",temp.c_str());
      //This is what we did before we were interested in the comments
    }

  rep->strm >> std::ws;

  // read in chunks from zipped PFZ file
  while(rep->strm.good())
  {
    rep->strm.read(reinterpret_cast<char*>(buff_uin),PFZ_CHUNK);
    zstrm.avail_in = rep->strm.gcount();
    zstrm.next_in = buff_uin;

    // each input chunk is taken in sizes that zlib can handle
    zstrm.avail_out = 0;
    while (zstrm.avail_out == 0)
    {
      zstrm.avail_out = PFZ_CHUNK;
      zstrm.next_out = buff_uout;

      const int ret = inflate(&zstrm, Z_NO_FLUSH);
      if(ret == Z_STREAM_ERROR)
        LFATAL("Stream Error in zlib decompression of PFZ image '%s'",fname);
      switch (ret)
      {
      case Z_NEED_DICT:
        LINFO("zlib needs dictionary in opening PFZ '%s'",fname);
      case Z_DATA_ERROR:
        LFATAL("zlib data error in opening PFZ '%s'",fname);
      case Z_MEM_ERROR:
        LFATAL("zlib memory error in opening PFZ '%s'",fname);
      }
      const uint have = PFZ_CHUNK - zstrm.avail_out;
      rep->os.write(reinterpret_cast<char*>(buff_uout),have);
    }
  }

  rep->os >> rep->w >> rep->h;

  if (rep->mode != 1 && rep->mode != 4)
    {
      rep->os >> rep->maxGrey;
    }

  // copy and concatenate optional COMPRESSED
  // tags starting with '!'
  // into comments string

  rep->os >> std::ws;

  while ( rep->os.peek() == '!' )
    {
      // get the full line, remove leading !
      std::getline( rep->os, temp, '\n');
      temp = temp.substr(1,temp.length());
      rep->tagName.push_back(temp);
      // the next line contains the data with this tag
      std::getline( rep->os, temp, '\n');
      rep->tagValue.push_back(temp);
    }

  // read one more character of whitespace from the stream after maxGrey
  c = rep->os.get();
  if ( !isspace(c) )
    LFATAL("Missing whitespace after maxGrey in PFZ file '%s'.", fname);
  LDEBUG("PFM Reading Image: %s", fname);

  inflateEnd(&zstrm);
}

// ######################################################################
PfzParser::~PfzParser()
{
  delete rep;
}

// ######################################################################
GenericFrameSpec PfzParser::getFrameSpec() const
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
std::string PfzParser::getComments() const
{ return rep->comments; }

// ######################################################################
uint PfzParser::getTagCount() const
{ return rep->tags; }

// ######################################################################
bool PfzParser::getTag(uint tag, std::string &name, std::string &value) const
{
  if(tag < rep->tagName.size())
  {
    name  = rep->tagName[tag];
    value = rep->tagValue[tag];
    return true;
  }
  else
    return false;
}

// ######################################################################
GenericFrame PfzParser::getFrame()
{ return GenericFrame(parseFloat(), /* flags */ 0); }

// ######################################################################
void PfzParser::readFloats(float* space, int count)
{
  ASSERT(rep->mode == 'Z');
  rep->os.read(reinterpret_cast<char *>(space), count * sizeof(float));
}

// ######################################################################
Image<float> PfzParser::parseFloat()
{
  ASSERT(rep->mode == 'Z');

  Image<float> img(rep->w, rep->h, NO_INIT);
  readFloats(img.getArrayPtr(), img.getSize());
  return img;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
