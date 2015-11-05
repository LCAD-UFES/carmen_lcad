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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/SpeechSynth.H $
// $Id: SpeechSynth.H 9760 2008-05-11 22:40:13Z rjpeters $
//

#ifndef SpeechSynth_H_DEFINED
#define SpeechSynth_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/Pololu.H"
#include "Util/Types.H"
#include "rutz/shared_ptr.h"

#include <deque>
#include <istream>
#include <map>
#include <string>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

class WorkThreadServer;

class SpeechSynth : public ModelComponent
{
public:
  //Constructor
  SpeechSynth(OptionManager& mgr,
              const std::string& descrName = "SpeechSynth",
              const std::string& tagName = "SpeechSynth");

  ~SpeechSynth();

  bool sayText(const std::string& text, int priority, bool block = false);
  bool sendCommand(const std::string& text, int priority, bool block = false,
                   time_t* timestamp = NULL);

  void flushQueue();

  /// Play a wav file; the wav file must exist on the same machine as the running festival-server, which is not necessarily the same as localhost
  /** The wave file will be first loaded into the festival-server if
      it has not yet been loaded. */
  bool playWavFile(const std::string& fname, int priority, bool block = false,
                   int mindelay = 20);

  //###########################################################
  void start2();

private:
  void festivalConnect();
  void festivalClose();

  void paramChanged(ModelParamBase* const param,
                    const bool valueChanged,
                    ParamClient::ChangeStatus* status);

  OModelParam<std::string> itsServerHost;
  OModelParam<uint> itsServerPort;
  OModelParam<size_t> itsQueueSize;

  rutz::shared_ptr<WorkThreadServer> itsThreadServer;

  int itsServerFD;
  std::iostream* itsServerStream;

  int itsJobCounter;

  struct WavFileInfo
  {
    WavFileInfo()
      :
      fname(),
      token(),
      lasttime(0)
    {}

    std::string fname;
    std::string token;
    time_t lasttime;
  };

  std::map<std::string, WavFileInfo> itsWavFiles;
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
