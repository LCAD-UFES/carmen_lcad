/*!@file Beowulf/BeowulfOpts.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/BeowulfOpts.H $
// $Id: BeowulfOpts.H 6437 2006-04-07 17:16:42Z rjpeters $
//

#ifndef BEOWULF_BEOWULFOPTS_H_DEFINED
#define BEOWULF_BEOWULFOPTS_H_DEFINED

struct ModelOptionCateg;
struct ModelOptionDef;

//! Command-line options for Beowulf et al
//@{

//! Beowulf-related options
extern const ModelOptionCateg MOC_BEOWULF;

extern const ModelOptionDef OPT_BeowulfSlaveNames;
extern const ModelOptionDef OPT_BeowulfMaster;
extern const ModelOptionDef OPT_TCPcommunicatorIPaddr;
extern const ModelOptionDef OPT_TCPcommunicatorInQlen;
extern const ModelOptionDef OPT_TCPcommunicatorOuQlen;
extern const ModelOptionDef OPT_TCPcommunicatorInDropLast;
extern const ModelOptionDef OPT_TCPcommunicatorOuDropLast;
extern const ModelOptionDef OPT_TCPcommunicatorDisableShm;
extern const ModelOptionDef OPT_SockServPort;
extern const ModelOptionDef OPT_BeowulfSelfQlen;
extern const ModelOptionDef OPT_BeowulfSelfDropLast;
extern const ModelOptionDef OPT_SingleChannelBeoServerQuickMode;
extern const ModelOptionDef OPT_BeowulfInitTimeout;
//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // BEOWULF_BEOWULFOPTS_H_DEFINED
