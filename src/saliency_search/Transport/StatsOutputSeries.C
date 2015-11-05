/*!@file Transport/StatsOutputSeries.C FrameOstream subclass that writes image statistics to an output file */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/StatsOutputSeries.C $
// $Id: StatsOutputSeries.C 14290 2010-12-01 21:44:03Z itti $
//

#ifndef TRANSPORT_STATSOUTPUTSERIES_C_DEFINED
#define TRANSPORT_STATSOUTPUTSERIES_C_DEFINED

#include "Transport/StatsOutputSeries.H"

#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Raster/GenericFrame.H"
#include "Util/FileUtil.H"

#include <cstdio>

template <class T>
Range<T> getRange(const Image<T>& x)
{
  Range<T> result;
  for (typename Image<T>::const_iterator
         itr = x.begin(), stop = x.end(); itr != stop; ++itr)
    result.merge(*itr);
  return result;
}

struct StatsOutputSeries::Impl
{
  Impl()
    :
    fileName(""),
    fileOwned(true),
    frameNumber(-1),
    file(0),
    meanR(0.0),
    meanG(0.0),
    meanB(0.0),
    count(0)
  {}

  ~Impl()
  {
    if (this->file != 0)
      {
        fflush(this->file);

        // we don't want to fclose() the file if the file is actually
        // stdout or stderr, so check this->fileOwned first:
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

  void printStatsRGB(const Image<PixRGB<byte> >& img,
                     const std::string& imgname, const std::string& imgtype)
  {
    this->openFile();

    Image<byte> r, g, b;
    getComponents(img, r, g, b);

    const double mean_r = mean(r);
    const double stdev_r = stdev(r);
    const Range<byte> range_r = getRange(r);

    const double mean_g = mean(g);
    const double stdev_g = stdev(g);
    const Range<byte> range_g = getRange(g);

    const double mean_b = mean(b);
    const double stdev_b = stdev(b);
    const Range<byte> range_b = getRange(b);

    fprintf(this->file,
            "%06d "
            "R=[%d .. %f +/- %f .. %d] "
            "G=[%d .. %f +/- %f .. %d] "
            "B=[%d .. %f +/- %f .. %d] %% %s (%dx%d %s)\n",
            this->frameNumber,
            int(range_r.min()), mean_r, stdev_r, int(range_r.max()),
            int(range_g.min()), mean_g, stdev_g, int(range_g.max()),
            int(range_b.min()), mean_b, stdev_b, int(range_b.max()),
            imgname.c_str(), img.getWidth(), img.getHeight(),
            imgtype.c_str());
    fflush(this->file);

    this->rangeR.merge(range_r);
    this->rangeG.merge(range_g);
    this->rangeB.merge(range_b);

    this->meanR += mean_r;
    this->meanG += mean_g;
    this->meanB += mean_b;
    ++(this->count);
  }

  void printStatsSummary()
  {
    this->openFile();

    fprintf(this->file,
            "OVERALL "
            "R=[%d .. %f .. %d] "
            "G=[%d .. %f .. %d] "
            "B=[%d .. %f .. %d] %% summary of %d frames\n",
            int(rangeR.min()), meanR/count, int(rangeR.max()),
            int(rangeG.min()), meanG/count, int(rangeG.max()),
            int(rangeB.min()), meanB/count, int(rangeB.max()),
            count);
    fflush(this->file);
  }

  std::string      fileName;
  bool             fileOwned;
  int              frameNumber;
  FILE*            file;

  Range<byte>      rangeR;
  Range<byte>      rangeG;
  Range<byte>      rangeB;
  double           meanR;
  double           meanG;
  double           meanB;
  int              count;
};

// ######################################################################
StatsOutputSeries::StatsOutputSeries(OptionManager& mgr)
  :
  FrameOstream(mgr, "Stats Output Series", "StatsOutputSeries"),
  rep(new Impl)
{}

// ######################################################################
StatsOutputSeries::~StatsOutputSeries()
{
  delete rep;
}

// ######################################################################
void StatsOutputSeries::setConfigInfo(const std::string& filename)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  this->setFileName(filename);
}

// ######################################################################
bool StatsOutputSeries::setFrameNumber(int n)
{
  rep->frameNumber = n;

  return true;
}

// ######################################################################
void StatsOutputSeries::writeFrame(const GenericFrame& f,
                                  const std::string& shortname,
                                  const FrameInfo& auxinfo)
{
  const std::string nm = shortname;
  const std::string tp = f.nativeTypeName();

  switch (f.nativeType())
    {
    case GenericFrame::NONE:                                           break;
    case GenericFrame::RGB_U8:  rep->printStatsRGB(f.asRgbU8(),nm,tp); break;
    case GenericFrame::RGBD:  rep->printStatsRGB(f.asRgbU8(),nm,tp);   break;
    case GenericFrame::RGB_F32: rep->printStatsRGB(f.asRgbU8(),nm,tp); break;
    case GenericFrame::GRAY_U8: rep->printStatsRGB(f.asRgbU8(),nm,tp); break;
    case GenericFrame::GRAY_F32:rep->printStatsRGB(f.asRgbU8(),nm,tp); break;
    case GenericFrame::VIDEO:   rep->printStatsRGB(f.asRgbU8(),nm,tp); break;
    case GenericFrame::RGB_U16:   break;
    case GenericFrame::GRAY_U16:  break;
    }
}

// ######################################################################
void StatsOutputSeries::closeStream(const std::string& shortname)
{
  /* nothing to see here, move along */
}

// ######################################################################
void StatsOutputSeries::setFileName(const std::string& s)
{
  if (rep->file != 0)
    LFATAL("can't change filename while output file is already open");

  rep->fileName = s;
}

// ######################################################################
void StatsOutputSeries::stop2()
{
  rep->printStatsSummary();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_STATSOUTPUTSERIES_C_DEFINED
