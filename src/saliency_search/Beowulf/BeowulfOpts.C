/*!@file Beowulf/BeowulfOpts.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/BeowulfOpts.C $
// $Id: BeowulfOpts.C 6437 2006-04-07 17:16:42Z rjpeters $
//

#ifndef BEOWULF_BEOWULFOPTS_C_DEFINED
#define BEOWULF_BEOWULFOPTS_C_DEFINED

#include "Beowulf/BeowulfOpts.H"

#include "Component/ModelOptionDef.H"

#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif
#include <netinet/in.h>

const ModelOptionCateg MOC_BEOWULF = {
  MOC_SORTPRI_2, "Beowulf-Related Options" };

// Format here is:
//
// { MODOPT_TYPE, "name", &MOC_CATEG, OPTEXP_CORE,
//   "description of what option does",
//   "long option name", 'short option name', "valid values", "default value" }
//

// alternatively, for MODOPT_ALIAS option types, format is:
//
// { MODOPT_ALIAS, "", &MOC_ALIAS, OPTEXP_CORE,
//   "description of what alias does",
//   "long option name", 'short option name', "", "list of options" }
//

// NOTE: do not change the default value of any existing option unless
// you really know what you are doing!  Many components will determine
// their default behavior from that default value, so you may break
// lots of executables if you change it.

// #################### Beowulf options
// Used by: Beowulf
const ModelOptionDef OPT_BeowulfSlaveNames =
  { MODOPT_ARG_STRING, "BeowulfSlaveNames", &MOC_BEOWULF, OPTEXP_CORE,
    "Comma-separated list of names of slaves to use, in the hostname:port "
    "format (default port will be used if not specified), or single absolute "
    "path to a text file that contains the node names, one per line.",
    "beowulf-slaves", '\0', "<host:port,host:port,...|/path/to/nodelist.txt>",
    "/etc/nodes" };

// Used by: Beowulf
const ModelOptionDef OPT_BeowulfMaster =
  { MODOPT_FLAG, "BeowulfMaster", &MOC_BEOWULF, OPTEXP_CORE,
    "Flags this as the master node",
    "beowulf-master", '\0', "", "false" };

// Used by: TCPcommunicator
const ModelOptionDef OPT_TCPcommunicatorIPaddr =
  { MODOPT_ARG(in_addr), "TCPcommunicatorIPaddr", &MOC_BEOWULF, OPTEXP_CORE,
    "Our IP address to use for Beowulf communications (useful if we have "
    "several ethernet interfaces), or 0.0.0.0 to determine it automatically "
    "from our hostname",
    "ip-addr", '\0', "<int>.<int>.<int>.<int>", "0.0.0.0" };

// Used by: TCPcommunicator
const ModelOptionDef OPT_TCPcommunicatorInQlen =
  { MODOPT_ARG(int), "TCPcommunicatorInQlen", &MOC_BEOWULF, OPTEXP_CORE,
    "Queue length for incoming messages, or zero for unlimited",
    "tcp-inqlen", '\0', "<int>", "100" };

// Used by: TCPcommunicator
const ModelOptionDef OPT_TCPcommunicatorOuQlen =
  { MODOPT_ARG(int), "TCPcommunicatorOuQlen", &MOC_BEOWULF, OPTEXP_CORE,
    "Queue length for outgoing messages, or zero for unlimited",
    "tcp-outqlen", '\0', "<int>", "100" };

// Used by: TCPcommunicator
const ModelOptionDef OPT_TCPcommunicatorInDropLast =
  { MODOPT_FLAG, "TCPcommunicatorInDropLast", &MOC_BEOWULF, OPTEXP_CORE,
    "Message dropping policy when incoming queue is full; if true, the "
    "most recent (last received) message will be dropped, otherwise, the "
    "least recent will be dropped",
    "tcp-indroplast", '\0', "", "false" };

// Used by: TCPcommunicator
const ModelOptionDef OPT_TCPcommunicatorOuDropLast =
  { MODOPT_FLAG, "TCPcommunicatorOuDropLast", &MOC_BEOWULF, OPTEXP_CORE,
    "Message dropping policy when outgoing queue is full; if true, the "
    "most recent (last queued) message will be dropped, otherwise, the "
    "least recent will be dropped",
    "tcp-outdroplast", '\0', "", "false" };

// Used by: TCPcommunicator
const ModelOptionDef OPT_TCPcommunicatorDisableShm =
  { MODOPT_FLAG, "TCPcommunicatorDisableShm", &MOC_BEOWULF, OPTEXP_CORE,
    "Disable the use of shared memory for faster transfers between two "
    "Beowulf nodes running on the same physical machine",
    "tcp-disable-shm", '\0', "", "false" };

// Used by: TCPcommunicator
const ModelOptionDef OPT_SockServPort =
  { MODOPT_ARG(short), "SockServPort", &MOC_BEOWULF, OPTEXP_CORE,
    "Port on which to listen for incoming Beowulf connections, "
    "or 0 to determine it from /etc/services",
    "ip-port", '\0', "<int>", "0" };

// Used by: Beowulf
const ModelOptionDef OPT_BeowulfSelfQlen =
  { MODOPT_ARG(int), "BeowulfSelfQlen", &MOC_BEOWULF, OPTEXP_CORE,
    "Queue length for self-addressed messages, or zero for unlimited",
    "tcp-selfqlen", '\0', "<int>", "100" };

// Used by: Beowulf
const ModelOptionDef OPT_BeowulfSelfDropLast =
  { MODOPT_FLAG, "BeowulfSelfDropLast", &MOC_BEOWULF, OPTEXP_CORE,
    "Message dropping policy when self-addressed message queue is full; "
    "if true, the most recent (last received) message will be dropped, "
    "otherwise, the least recent will be dropped",
    "tcp-selfdroplast", '\0', "", "false" };

// Used by: SingleChannelBeoServer
const ModelOptionDef OPT_SingleChannelBeoServerQuickMode =
  { MODOPT_FLAG, "SingleChannelBeoServerQuickMode", &MOC_BEOWULF, OPTEXP_CORE,
    "Use quick mode where only the output map is sent back for each"
    "channel; this will break any program that attempts to access other"
    "channel results, such as the pyramid queue, submaps or clipPyramid",
    "scbserver-quickmode", '\0', "", "true" };

const ModelOptionDef OPT_BeowulfInitTimeout =
  { MODOPT_ARG(double), "BeowulfInitTimeout", &MOC_BEOWULF, OPTEXP_CORE,
    "Max time in seconds to wait for Beowulf initialization, "
    "or zero for unlimited",
    "beowulf-init-timeout", '\0', "<float>", "0" };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // BEOWULF_BEOWULFOPTS_C_DEFINED
