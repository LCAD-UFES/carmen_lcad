/*!@file Devices/SpeechSynth.C Interfaces to festival speech synth */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/SpeechSynth.C $
// $Id: SpeechSynth.C 9760 2008-05-11 22:40:13Z rjpeters $
//

#include "Devices/SpeechSynth.H"

#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Util/Assert.H"
#include "Util/JobWithSemaphore.H"
#include "Util/StringUtil.H"
#include "Util/WorkThreadServer.H"
#include "Util/sformat.H"
#include "rutz/compat_snprintf.h"
#include "rutz/stdiobuf.h"

#include <fcntl.h>

static const ModelOptionDef OPT_FestivalServerHost =
  { MODOPT_ARG_STRING, "FestivalServerHost", &MOC_AUDIO, OPTEXP_CORE,
    "IP address of festival server to use for speech synthesis",
    "festival-server-host", '\0', "<ipaddr>", "127.0.0.1" };

static const ModelOptionDef OPT_FestivalServerPort =
  { MODOPT_ARG(uint), "FestivalServerPort", &MOC_AUDIO, OPTEXP_CORE,
    "Port number of festival server to use for speech synthesis",
    "festival-server-port", '\0', "<portnum>", "1314" };

static const ModelOptionDef OPT_SpeechQueueSize =
  { MODOPT_ARG(size_t), "SpeechQueueSize", &MOC_AUDIO, OPTEXP_CORE,
    "Max queue size for speech utterances; low-priority utterances "
    "will be dropped to avoid exceeding this size",
    "speech-queue-size", '\0', "<size_t>", "1" };

namespace
{
  class SpeechUtteranceJob : public JobWithSemaphore
  {
  public:
    SpeechUtteranceJob(std::iostream& server,
                       const std::string& msg,
                       const int priority,
                       const int id,
                       time_t* timestamp)
      :
      itsServer(server),
      itsMsg(msg),
      itsPriority(priority),
      itsJobType(sformat("utterance[priority=%d]", priority)),
      itsId(id),
      itsTimestamp(timestamp)
    {}

    virtual ~SpeechUtteranceJob() {}

    virtual void run()
    {
      LINFO("          running #%d @ priority %d: %s",
            itsId, itsPriority, itsMsg.c_str());

      const std::string msg_with_newline = itsMsg + '\n';

      itsServer << msg_with_newline << std::flush;

      std::string code;
      std::getline(itsServer, code);

      LDEBUG("festival ack = '%s'", code.c_str());

      if (code == "LP" || code == "WV")
        {
          std::string data, end;
          std::getline(itsServer, data);
          std::getline(itsServer, end);
          LDEBUG("festival data = '%s'", data.c_str());
          LDEBUG("festival end = '%s'", end.c_str());
          if (end.size() < 2 || end.substr(end.size() - 2).compare("OK") != 0)
            LERROR("festival return message didn't end with 'OK'");
        }
      else if (code == "ER")
        {
          LERROR("festival returned error code 'ER'");
        }

      if (itsTimestamp)
        time(itsTimestamp);

      this->markFinished();
    }

    virtual const char* jobType() const
    { return itsJobType.c_str(); }

    virtual int priority() const
    { return itsPriority; }

    std::iostream& itsServer;
    const std::string itsMsg;
    const int itsPriority;
    const std::string itsJobType;
    const int itsId;
    time_t* itsTimestamp;
  };
}

// ######################################################################
SpeechSynth::SpeechSynth(OptionManager& mgr, const std::string& descrName,
                         const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName),
  itsServerHost(&OPT_FestivalServerHost, this),
  itsServerPort(&OPT_FestivalServerPort, this),
  itsQueueSize(&OPT_SpeechQueueSize, this, ALLOW_ONLINE_CHANGES),
  itsThreadServer(),
  itsServerFD(-1),
  itsServerStream(0),
  itsJobCounter(0)
{}

// ######################################################################
void SpeechSynth::start2()
{
   festivalConnect();
}

// ######################################################################
SpeechSynth::~SpeechSynth()
{
   festivalClose();
}

// ######################################################################
void SpeechSynth::festivalConnect()
{
  itsThreadServer.reset(new WorkThreadServer("SpeechSynth", 1));
  itsThreadServer->setDropPolicy(WorkThreadServer::DROP_OLDEST_LOWEST_PRIORITY);
  itsThreadServer->setMaxQueueSize(itsQueueSize.getVal());
  itsThreadServer->setFlushBeforeStopping(false);

  /* Return an FD to a remote server */
  struct sockaddr_in serv_addr;
  struct hostent *serverhost;
  int fd;

  fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (fd < 0)
  {
    LFATAL("can't get socket\n");
  }
  memset(&serv_addr, 0, sizeof(serv_addr));
  if ((int)(serv_addr.sin_addr.s_addr = inet_addr(itsServerHost.getVal().c_str())) == -1)
  {
    /* its a name rather than an ipnum */
    serverhost = gethostbyname(itsServerHost.getVal().c_str());
    if (serverhost == (struct hostent *)0)
    {
      LFATAL("festival_client: gethostbyname failed\n");
    }
    memmove(&serv_addr.sin_addr,serverhost->h_addr, serverhost->h_length);
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(itsServerPort.getVal());

  if (connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != 0)
  {
    LINFO("Connect to server failed: Insure that festival is running in server mode\n");
    LINFO("Starting in no sound mode\n");
    itsServerFD = -1;
    delete itsServerStream;
    itsServerStream = 0;
  } else {
    itsServerFD = fd;
    itsServerStream = new rutz::stdiostream(itsServerFD,
                                            std::ios::in | std::ios::out,
                                            true);
  }
}

// ######################################################################
void SpeechSynth::festivalClose()
{
  itsThreadServer.reset(0);

  if (itsServerFD != -1)
    {
      close(itsServerFD);
      itsServerFD = -1;
    }

  if (itsServerStream != 0)
    {
      delete itsServerStream;
      itsServerStream = 0;
    }
}

// ######################################################################
bool SpeechSynth::sayText(const std::string& text, int priority,
                          bool block)
{
  std::string cmd = sformat("(SayText \"%s\")", text.c_str());
  return sendCommand(cmd, priority, block);
}

// ######################################################################
bool SpeechSynth::sendCommand(const std::string& text, int priority,
                              bool block, time_t* timestamp)
{
  const int id = ++itsJobCounter;
  LINFO("enqueuing #%d @ priority %d: %s", id, priority, text.c_str());

  if (itsServerStream != 0 && itsThreadServer.get() != 0)
    {
      rutz::shared_ptr<SpeechUtteranceJob> j
        (new SpeechUtteranceJob(*itsServerStream, text, priority, id,
                                timestamp));

      itsThreadServer->enqueueJob(j);

      if (j->wasDropped())
        return false;

      if (block)
        j->wait();

      return true;
    }

  return false;
}

// ######################################################################
void SpeechSynth::flushQueue()
{
  if (itsThreadServer.get() != 0)
    itsThreadServer->flushQueue();
}

// ######################################################################
bool SpeechSynth::playWavFile(const std::string& fname, int priority, bool block,
                              int mindelay)
{
  static int nextid = 0;

  std::map<std::string, WavFileInfo>::iterator itr = itsWavFiles.find(fname);

  if (itr == itsWavFiles.end())
    {
      WavFileInfo info;

      info.fname = fname;
      info.token = sformat("SpeechSynthWavToken%d", nextid++);
      if (!this->sendCommand(sformat("(if (probe_file \"%s\") (set! %s (wave.load \"%s\")) (set! %s nil))",
                                     fname.c_str(),
                                     info.token.c_str(),
                                     fname.c_str(),
                                     info.token.c_str()),
                             -10, false))
        return false;

      itr = itsWavFiles.insert(std::make_pair(fname, info)).first;
    }

  const time_t now = time(NULL);

  WavFileInfo& info = (*itr).second;

  if (now - info.lasttime >= mindelay)
    {
      info.lasttime = now;

      return this->sendCommand(sformat("(if %s (wave.play %s))",
                                       info.token.c_str(),
                                       info.token.c_str()),
                               priority, block, &info.lasttime);
    }
  else
    LINFO("not speaking '%s' because the delay is %ld secs, less than the minimum %d secs",
          info.fname.c_str(), now - info.lasttime, mindelay);

  // else...
  return false;
}

// ######################################################################
void SpeechSynth::paramChanged(ModelParamBase* const param,
                               const bool valueChanged,
                               ParamClient::ChangeStatus* status)
{
  if (param == &itsQueueSize && valueChanged)
    {
      if (itsQueueSize.getVal() == 0)
        *status = ParamClient::CHANGE_REJECTED;
      else if (itsThreadServer.get() != 0)
        itsThreadServer->setMaxQueueSize(itsQueueSize.getVal());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
