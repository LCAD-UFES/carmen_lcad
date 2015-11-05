/*!@file Transport/HashOutputSeries.C write a series of image hashes to an output file */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/HashOutputSeries.C $
// $Id: HashOutputSeries.C 14290 2010-12-01 21:44:03Z itti $
//

#ifndef TRANSPORT_HASHOUTPUTSERIES_C_DEFINED
#define TRANSPORT_HASHOUTPUTSERIES_C_DEFINED

#include "Transport/HashOutputSeries.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/FileUtil.H"
#include "Util/jenkinshash.H"

#include <cstdio>
#include <set>

struct HashOutputSeries::Impl
{
  Impl()
    :
    fileName(""),
    fileOwned(true),
    frameNumber(-1),
    file(0)
  {}

  ~Impl()
  {
    if (this->file != 0)
      {
        fflush(this->file);

        // we might not want to fclose() the file if the file is
        // actually stdout or stderr, so we check this->fileOwned
        // first:
        if (this->fileOwned)
          fclose(this->file);
      }
  }

  void openFile()
  {
    if (this->file == 0)
      {
        this->file = stdOutputFileOpen(this->fileName,
                                       &this->fileOwned);

        // ok, after all is said and done we should have a valid FILE*
        ASSERT(this->file != 0);
      }
  }

  void doPrintHash(const uint32 h,
                   const std::string& imgname, const std::string& imgtype)
  {
    this->openFile();

    const bool repeat = (this->hashes.find(h) != this->hashes.end());
    if (!repeat)
      this->hashes.insert(h);

    fprintf(this->file, "%06d  %08x  %d  %% %s (%s)\n",
            this->frameNumber, h, int(repeat),
            imgname.c_str(), imgtype.c_str());
    fflush(this->file);
  }

  template <class T>
  void printHash(const Image<T>& ima,
                 const std::string& imgname, const std::string& imgtype)
  {
    const uint32 h =
      jenkinshash(reinterpret_cast<const byte*>(ima.getArrayPtr()),
                  ima.getSize() * sizeof(T),
                  0);

    this->doPrintHash(h, imgname, imgtype);
  }

  void printHash(const VideoFrame& frame,
                 const std::string& imgname, const std::string& imgtype)
  {
    const uint32 h =
      jenkinshash(reinterpret_cast<const byte*>(frame.getBuffer()),
                  frame.getBufSize(),
                  0);

    this->doPrintHash(h, imgname, imgtype);
  }

  std::string      fileName;
  bool             fileOwned;
  int              frameNumber;
  FILE*            file;
  std::set<uint32> hashes;
};

// ######################################################################
HashOutputSeries::HashOutputSeries(OptionManager& mgr)
  :
  FrameOstream(mgr, "Hash Output Series", "HashOutputSeries"),
  rep(new Impl)
{}

// ######################################################################
HashOutputSeries::~HashOutputSeries()
{
  delete rep;
}

// ######################################################################
void HashOutputSeries::setConfigInfo(const std::string& filename)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  this->setFileName(filename);
}

// ######################################################################
bool HashOutputSeries::setFrameNumber(int n)
{
  rep->frameNumber = n;

  return true;
}

// ######################################################################
void HashOutputSeries::writeFrame(const GenericFrame& f,
                                  const std::string& shortname,
                                  const FrameInfo& auxinfo)
{
  const std::string nm = shortname;
  const std::string tp = f.nativeTypeName();

  switch (f.nativeType())
    {
    case GenericFrame::NONE:                                         break;
    case GenericFrame::RGB_U8:   rep->printHash(f.asRgbU8(),nm,tp);  break;
    case GenericFrame::RGBD:     rep->printHash(f.asRgbU8(),nm,tp);  break;
    case GenericFrame::RGB_F32:  rep->printHash(f.asRgbF32(),nm,tp); break;
    case GenericFrame::GRAY_U8:  rep->printHash(f.asGrayU8(),nm,tp); break;
    case GenericFrame::GRAY_F32: rep->printHash(f.asGrayF32(),nm,tp);break;
    case GenericFrame::VIDEO:    rep->printHash(f.asVideo(),nm,tp);  break;
    case GenericFrame::RGB_U16:     break;
    case GenericFrame::GRAY_U16:     break;
    }
}

// ######################################################################
void HashOutputSeries::closeStream(const std::string& shortname)
{
  /* nothing to see here, move along */
}

// ######################################################################
void HashOutputSeries::setFileName(const std::string& s)
{
  if (rep->file != 0)
    LFATAL("can't change filename while output file is already open");

  rep->fileName = s;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_HASHOUTPUTSERIES_C_DEFINED
