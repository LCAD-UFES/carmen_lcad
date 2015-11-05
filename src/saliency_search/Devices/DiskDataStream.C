/*!@file Devices/DiskDataStream.C Multi-threaded data streamer to disk */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/DiskDataStream.C $
// $Id: DiskDataStream.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Devices/DiskDataStream.H"

#include "Devices/DeviceOpts.H"
#include "Util/StringUtil.H" // for split() and join()
#include "Util/WorkThreadServer.H"
#include "Util/sformat.H"
#include "Raster/GenericFrame.H"

#include <iterator> // for back_inserter()
#include <unistd.h> // for usleep

namespace dummy_namespace_to_avoid_gcc411_bug_DiskDataStream_C
{
  class DiskDumpJob : public JobServer::Job
  {
  public:
    DiskDumpJob(const GenericFrame& f, const std::string& nm,
                bool usemmap)
      :
      itsFrame(f),
      itsFname(nm),
      itsUseMmap(usemmap)
    {}

    virtual void run()
    {
      if (itsUseMmap)
        // 1.01s cpu for 1000 frames (4.60s including framegrabbing)
        itsFrame.asVideo().diskDumpMmap(itsFname.c_str(), true);
      else
        // 1.74s cpu for 1000 frames (5.60s including framegrabbing)
        itsFrame.asVideo().diskDumpStdio(itsFname.c_str(), true);
    }

    virtual const char* jobType() const { return "DiskDumpJob"; }

    const GenericFrame itsFrame;
    const std::string itsFname;
    const bool        itsUseMmap;
  };

  struct DiskDumpStreamData
  {
    DiskDumpStreamData() : frameNumberIn(0), frameNumberOut(0) {}

    uint frameNumberIn;//!< How many input frames have we received
    uint frameNumberOut;//!< How many output frames have we saved
  };
}

using namespace dummy_namespace_to_avoid_gcc411_bug_DiskDataStream_C;

typedef std::map<std::string, DiskDumpStreamData> map_type;

// ######################################################################
struct DiskDataStream::Impl
{
  map_type streams;
};

// ######################################################################
DiskDataStream::DiskDataStream(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  FrameOstream(mgr, descrName, tagName),
  itsUseMmap(&OPT_DiskDataStreamUseMmap, this),
  itsNumThreads(&OPT_DiskDataStreamNumThreads, this),
  itsSleepUsecs(&OPT_DiskDataStreamSleepUsecs, this),
  itsSavePeriod(&OPT_DiskDataStreamSavePeriod, this),
  itsSavePath(&OPT_DiskDataStreamSavePath, this),
  itsFileStems(),
  itsServer(0),
  rep(new Impl)
{}

// ######################################################################
DiskDataStream::~DiskDataStream()
{
  delete rep;
}

// ######################################################################
void DiskDataStream::setConfigInfo(const std::string& path)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  this->setSavePath(path);
}

// ######################################################################
void DiskDataStream::setSavePath(const std::string& path)
{
  LINFO("setting save path to '%s'", path.c_str());

  itsFileStems.resize(0);

  if (path.size() == 0)
    {
      itsFileStems.push_back("./");
    }
  else
    {
      split(path, ",", std::back_inserter(itsFileStems));
    }

  LINFO("got %" ZU " stems from '%s'", itsFileStems.size(), path.c_str());

  ASSERT(itsFileStems.size() > 0);
}

// ######################################################################
void DiskDataStream::start2()
{
  if (itsUseMmap.getVal())
    {
#ifdef MMAP_IS_NOT_THREADSAFE
      itsUseMmap.setVal(false);
      LINFO("using stdio instead of mmap "
            "because mmap is not threadsafe on this system");
#else
      LINFO("using mmap for writing");
#endif
    }
  else
    LINFO("using stdio for writing");

  ASSERT(itsServer == 0);
  itsServer = new WorkThreadServer("DiskDataStream",
                                   itsNumThreads.getVal());

  itsServer->setCheckpointPeriod(100);
  itsServer->setSleepUsecs(itsSleepUsecs.getVal());
  setSavePath(itsSavePath.getVal());
}

// ######################################################################
void DiskDataStream::stop1()
{
  LINFO("Flushing data queue to disk");

  // wait until queue has been fully written to disk:
  while (true)
    {
      const uint sz = itsServer->size();

      if (sz == 0)
        break;

      LINFO("Flushing data queue to disk - %u remaining", sz);
      usleep(250000);
    }

  delete itsServer;
  itsServer = 0;
}

// ######################################################################
void DiskDataStream::writeFrame(const GenericFrame& frame,
                                const std::string& shortname,
                                const FrameInfo& auxinfo)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  DiskDumpStreamData& s = rep->streams[shortname];

  const uint nout = s.frameNumberIn++;
  if ((nout % itsSavePeriod.getVal()) != 0)
    // skip this frame if it doesn't match our save period
    return;

  // push that video frame into our queue:

  // compute the filename:
  ASSERT(itsFileStems.size() > 0);
  const uint n = (s.frameNumberOut % itsFileStems.size());
  const std::string fname =
    sformat("%s%s%06d",
            itsFileStems[n].c_str(), shortname.c_str(),
            s.frameNumberOut);

  // ready for next file:
  ++s.frameNumberOut;

  // we make a deep copy since the buffer held by frame may be
  // "short-lived", i.e., likely to be overwritten when the next frame
  // is read from the original data source (mpeg file, framegrabber,
  // etc.)
  rutz::shared_ptr<DiskDumpJob> j
    (new DiskDumpJob(GenericFrame::deepCopyOf(frame),
                     fname, itsUseMmap.getVal()));

  itsServer->enqueueJob(j);
}

// ######################################################################
void DiskDataStream::closeStream(const std::string& shortname)
{
  map_type::iterator itr = rep->streams.find(shortname);

  if (itr != rep->streams.end())
    {
      (*itr).second.frameNumberIn = 0;
      (*itr).second.frameNumberOut = 0;
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
