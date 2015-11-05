/*!@file Neuro/EnvInferoTemporal.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvInferoTemporal.C $
// $Id: EnvInferoTemporal.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef NEURO_ENVINFEROTEMPORAL_C_DEFINED
#define NEURO_ENVINFEROTEMPORAL_C_DEFINED

#include "Neuro/EnvInferoTemporal.H"

#include "Component/ModelOptionDef.H"
#include "Image/CutPaste.H"
#include "Util/StringUtil.H"
#include "rutz/mutex.h"

static const ModelOptionCateg MOC_EIT = {
  MOC_SORTPRI_3, "EnvInferoTemporal-related Options" };

static const ModelOptionDef OPT_ConfidenceThresh =
  { MODOPT_ARG(double), "ConfidenceThresh", &MOC_EIT, OPTEXP_CORE,
    "Threshold for object label confidence (between 0 and 1, inclusive), "
    "above which attention is switched to a new object",
    "confidence-thresh", '\0', "<double>", "0.0" };

static const ModelOptionDef OPT_EitAddPatchReader =
  { MODOPT_ARG_STRING, "EitAddPatchReader", &MOC_EIT, OPTEXP_CORE,
    "IP address and port on which to communicate with a "
    "patch-reader/label-server.",
    "patch-reader", '\0', "<ipaddr:port>", "" };

namespace
{
  const PixRGB<byte> colors[4] =
    {
      PixRGB<byte>(128,255,0),
      PixRGB<byte>(0,255,128),
      PixRGB<byte>(0,0,255),
      PixRGB<byte>(255,0,255)
    };
}

// ######################################################################
EnvInferoTemporal::EnvInferoTemporal(OptionManager& mgr)
  :
  ModelComponent(mgr, "Embeddable Infero-Temporal Cortex",
                 "EnvInferoTemporal"),
  itsConfidenceThresh(&OPT_ConfidenceThresh, this, ALLOW_ONLINE_CHANGES),
  itsIgnoreNomatch("EitIgnoreNomatch", this, false, ALLOW_ONLINE_CHANGES),
  itsAddPatchReader(&OPT_EitAddPatchReader, this),
  itsOnlineAddPatchReader("EitNewPatchReader", this, std::string(),
                          ALLOW_ONLINE_CHANGES)
{
  if (0 != pthread_mutex_init(&itsReadersLock, NULL))
    PLFATAL("pthread_mutex_init() failed");
}

// ######################################################################
EnvInferoTemporal::~EnvInferoTemporal()
{
  if (0 != pthread_mutex_destroy(&itsReadersLock))
    PLERROR("pthread_mutex_destroy() failed");
}

// ######################################################################
void EnvInferoTemporal::paramChanged(ModelParamBase* const param,
                                     const bool valueChanged,
                                     ParamClient::ChangeStatus* status)
{
  // We have two model params for adding new patch readers; one is
  // intended for command-line use (itsAddPatchReader) but not live
  // modification, and the other is intended for live/online
  // modifications (itsOnlineAddPatchReader). The difference is that
  // in the online case we want to "reject" the change so that the
  // GUI-displayed value always returns to an empty string, while in
  // the command-line case we must not reject the change, or else
  // command-line parsing will fail.
  if (param == &itsAddPatchReader)
    {
      this->addNewPatchReader(itsAddPatchReader.getVal());
    }
  else if (param == &itsOnlineAddPatchReader)
    {
      const std::string addr = itsOnlineAddPatchReader.getVal();
      if (addr.length() > 0)
        {
          // "reject" all changes so that our value always returns
          // back to the empty string
          *status = ParamClient::CHANGE_REJECTED;

          this->addNewPatchReader(addr);
        }
    }
}

// ######################################################################
void EnvInferoTemporal::initReaders(const std::string& addrlist)
{
  if (addrlist.empty())
    return;

  GVX_MUTEX_LOCK(&itsReadersLock);

  std::vector<std::string> addrs;
  split(addrlist, ",", std::back_inserter(addrs));

  for (size_t i = 0; i < addrs.size(); ++i)
    {
      itsReaders.push_back
        (rutz::make_shared
         (new Nv2LabelReader(colors[i%4],
                             NV2_LABEL_READER_PORT+2*i,
                             addrs[i])));
    }
}

// ######################################################################
void EnvInferoTemporal::sendPatch(const uint32_t id,
                                  const Image<PixRGB<byte> >& fullimg,
                                  const Rectangle& foa,
                                  const rutz::time& qtime,
                                  bool is_training_image,
                                  const std::string& training_label,
                                  const std::string& remote_command,
                                  Point2D<int> fixLoc)
{
  GVX_MUTEX_LOCK(&itsReadersLock);

  if (itsReaders.size() == 0)
    return;

  Image<PixRGB<byte> > foapatch;

  if (foa.isValid()
      && fullimg.rectangleOk(foa))
    foapatch = crop(fullimg, foa);
  else
    foapatch = Image<PixRGB<byte> >(1,1,ZEROS);

  for (size_t i = 0; i < itsReaders.size(); ++i)
    itsReaders[i]->sendPatch
      (id, fullimg, foa, foapatch,
       qtime,
       training_label.length() > 0,
       training_label,
       remote_command,
       fixLoc);

  if (training_label.length() > 0)
    LINFO("committing training image with label '%s'",
          training_label.c_str());
}

// ######################################################################
std::vector<Nv2LabelReader::LabeledImage>
EnvInferoTemporal::getLabeledImages(const size_t text_length)
{
  GVX_MUTEX_LOCK(&itsReadersLock);

  std::vector<Nv2LabelReader::LabeledImage> result;

  for (size_t i = 0; i < itsReaders.size(); ++i)
    {
      const Nv2LabelReader::LabeledImage p =
        itsReaders[i]->getNextLabeledImage(itsIgnoreNomatch.getVal(),
                                           text_length, 0);

      if (p.img.initialized())
        result.push_back(p);
    }

  return result;
}

// ######################################################################
bool EnvInferoTemporal::belowConfidenceThresh() const
{
  GVX_MUTEX_LOCK(&itsReadersLock);

  const double thresh = itsConfidenceThresh.getVal();

  for (size_t i = 0; i < itsReaders.size(); ++i)
    if (itsReaders[i]->getLastConfidence() > thresh)
      return false;

  return true;
}

// ######################################################################
double EnvInferoTemporal::getMaxConfidence() const
{
  GVX_MUTEX_LOCK(&itsReadersLock);

  double c = 0.0;
  for (size_t i = 0; i < itsReaders.size(); ++i)
    if (itsReaders[i]->getLastConfidence() > c)
      c = itsReaders[i]->getLastConfidence();

  return c;
}

// ######################################################################
void EnvInferoTemporal::addNewPatchReader(const std::string& addr)
{
  if (addr.length() == 0)
    return;

  GVX_MUTEX_LOCK(&itsReadersLock);
  const size_t i = itsReaders.size();

  LINFO("addr=%s, #readers = %" ZU , addr.c_str(), itsReaders.size());

  itsReaders.push_back
    (rutz::make_shared
     (new Nv2LabelReader(colors[i%4],
                         NV2_LABEL_READER_PORT+2*i,
                         addr)));

  LINFO("#readers = %" ZU , itsReaders.size());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_ENVINFEROTEMPORAL_C_DEFINED
