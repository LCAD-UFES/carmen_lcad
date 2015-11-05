/*!@file Channels/OrientationChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/OrientationChannel.C $
// $Id: OrientationChannel.C 12074 2009-11-24 07:51:51Z itti $
//

#ifndef ORIENTATIONCHANNEL_C_DEFINED
#define ORIENTATIONCHANNEL_C_DEFINED

#include "Channels/OrientationChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/GaborChannel.H"
#include "Component/OptionManager.H"
#include "Image/ImageSetOps.H"
#include "Image/PyramidOps.H"
#include "Image/PyramidCache.H"
#include "rutz/mutex.h"
#include "rutz/trace.h"
#include "Util/sformat.H"

#include <cstdio> // for sscanf()

// ######################################################################
// OrientationChannel member definitions:
// ######################################################################

// ######################################################################
OrientationChannel::OrientationChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "Orientation", "orientation", ORI),
  itsNumOrients(&OPT_NumOrientations, this), // see Channels/ChannelOpts.{H,C}
  itsInteractString(&OPT_OriInteraction, this),
  itsInteractType(NONE),
  itsDidInteractions(false),
  itsOverideTagName(false)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's build our channels; we may have to re-build them if
  // itsNumOrient get changed on us before we start():
  buildSubChans();
}

// ######################################################################
OrientationChannel::OrientationChannel(OptionManager& mgr, const char* tag, const char* desc, const char* gabortag) :
  ComplexChannel(mgr, "Orientation", "orientation", ORI),
  itsNumOrients(&OPT_NumOrientations, this), // see Channels/ChannelOpts.{H,C}
  itsInteractString(&OPT_OriInteraction, this),
  itsInteractType(NONE),
  itsDidInteractions(false),
  itsOverideTagName(true)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's build our channels; we may have to re-build them if
  // itsNumOrient get changed on us before we start():

  setDescriptiveName(sformat("%s", desc));
  setTagName(sformat("%s", tag));

  itsGaborOverideTag = gabortag;

  buildSubChans();

}
// ######################################################################
void OrientationChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our Gabor subchannels now that we know how many
  // we want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  if(itsOverideTagName)
  {
    LINFO("Using %d %s orientations spanning [0..180]deg", itsNumOrients.getVal(),itsGaborOverideTag.c_str());
    for (uint ori = 0; ori < itsNumOrients.getVal(); ++ori)
    {
      nub::ref<GaborChannel> chan =
        makeSharedComp
        (new GaborChannel(getManager(), ori,
                          180.0 * double(ori) /
                          double(itsNumOrients.getVal()),
                          itsGaborOverideTag.c_str(),
                          itsGaborOverideTag.c_str()));
      this->addSubChan(chan);

      // let's export options on our newly built channels:
      chan->exportOptions(MC_RECURSE);
    }
  }
  else
  {
    LINFO("Using %d orientations spanning [0..180]deg", itsNumOrients.getVal());
    for (uint ori = 0; ori < itsNumOrients.getVal(); ++ori)
    {
      nub::ref<GaborChannel> chan =
        makeSharedComp
        (new GaborChannel(getManager(), ori,
                          180.0 * double(ori) /
                          double(itsNumOrients.getVal())));
      this->addSubChan(chan);

      // let's export options on our newly built channels:
      chan->exportOptions(MC_RECURSE);
    }
  }
  // Here, we just use GaborChannel's default tag names
}

// ######################################################################
void OrientationChannel::parseInteractString(const std::string& value)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // need to clear the coefficients in any case
  itsInteractCoeffs.clear();

  if (value.compare("None") == 0)
    {
      itsInteractType = NONE;
      return;
    }

  if (value.compare("SubtractMean") == 0)
    {
      itsInteractType = SUB_MEAN;
      return;
    }

  itsInteractType = CUSTOM;

  // format here is "c.c,...,c.c"
  int curpos = 0, len = value.length();
  while (curpos < len)
    {
      // get end of next number
      int nextpos = value.find_first_not_of("-.0123456789eE",curpos);
      if (nextpos == -1) nextpos = len;

      // no number characters found -> bummer
      if (nextpos == curpos)
        LFATAL("Error parsing the OriInteract string '%s' - found '%c' "
               "instead of a number.",value.c_str(),value[curpos]);

      // now let's see - can we get a number here?
      float coeff;
      int nscan = sscanf(value.substr(curpos,nextpos-curpos).c_str(),"%g",&coeff);

      // couldn't read a number -> bummer
      if (nscan != 1)
        LFATAL("Error parsing OriInteract string '%s' - found '%s' instead of "
               "a number.", value.c_str(),
               value.substr(curpos,nextpos-curpos).c_str());

      // yeah! found a number -> store it
      itsInteractCoeffs.push_back(coeff);

      LDEBUG("coeff = %g; value[nextpos] = '%c'",coeff,value[nextpos]);

      // not a comma -> bummer
      if ((nextpos < len) && (value[nextpos] != ','))
        LFATAL("Error parsing the OriInteract string '%s' - found '%c' "
               "instead of ','.",value.c_str(),value[nextpos]);

      // the character right after the comma should be a number again
      curpos = nextpos + 1;
    }

  // end of string, done
  return;
}

// ######################################################################
void OrientationChannel::paramChanged(ModelParamBase* const param,
                                      const bool valueChanged,
                                      ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumOrients &&
      numChans() != itsNumOrients.getVal())
    buildSubChans();

  // if the param is our OriInteraction, then parse the string
  else if (param == &itsInteractString)
    parseInteractString(itsInteractString.getVal());
}

// ######################################################################
OrientationChannel::~OrientationChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
GaborChannel& OrientationChannel::gabor(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // Since we are dynamic_cast'ing a reference, this operation will either
  // succeed or throw an exception.
  return *(dynCast<GaborChannel>(subChan(idx)));
}

// ######################################################################
void OrientationChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.grayFloat().initialized());

  if (numChans() == 0)
    return;

  rutz::mutex_lock_class lock;
  if (inframe.pyrCache().get() != 0
      && inframe.pyrCache()->laplacian9.beginSet(inframe.grayFloat(), &lock))
    {
      inframe.pyrCache()->laplacian9.endSet
        (inframe.grayFloat(),
         buildPyrLaplacian
         (inframe.grayFloat(), gabor(0).getMinPyrLevel(),
          gabor(0).getMaxPyrLevel(), 9),
         &lock);
    }

  for (uint i = 0; i < numChans(); ++i)
    {
      gabor(i).input(inframe);
      LINFO("Orientation pyramid (%d/%d) ok.", i+1, numChans());
    }
  itsDidInteractions = false;
}

// ######################################################################
void OrientationChannel::doInteractions()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  LINFO("OriInteractionType is %s.",itsInteractString.getVal().c_str());
  switch(itsInteractType)
    {
    case NONE:
      {
        // compute oriented gabor pyramids in several basis directions:
        break;
      }

    case SUB_MEAN:
      {
        // compute the average and subtract it from each GaborPyramid
        // this is actually a special case of the CUSTOM one, but
        // this method here is more efficient
        ImageSet<float> mean;

        // compute the mean
        for (uint i = 0; i < numChans(); ++i)
          {
            gabor(i).getOutput(); // make sure that gabor(i) has a pyramid
            if (mean.isEmpty())
              mean = gabor(i).itsPq.back().pyr;
            else
              mean += gabor(i).itsPq.back().pyr;
          }
        mean /= (float)numChans();

        // clampdiff each pyramid with the mean and feed them back
        // into the channel:
        for (uint i = 0; i < numChans(); ++i)
          {
            gabor(i).getOutput(); // make sure that gabor(i) has a pyramid
            gabor(i).itsPq.back().pyr =
              clampedDiff(gabor(i).itsPq.back().pyr,mean);
            LINFO("Orientation pyramid interactions (%d/%d) ok.",
                  i+1, numChans());
          }

        // done
        break;
      }

    case CUSTOM:
      {
        // In this case we have arbitrary linear interactions between the
        // orientations - takes a bit of computation

        // make sure we have the correct number of coefficients
        ASSERT(itsInteractCoeffs.size() == numChans());
        std::vector< ImageSet<float> > resPyrs(numChans());

        // compute the pyramid interactions
        for (uint i = 0; i < numChans(); ++i)
          {
            ImageSet<float> curPyr = gabor(i).itsPq.back().pyr;
            uint coeff_ptr = i;

            // add this new pyramid with the correct coefficients
            for (uint j = 0; j < numChans(); ++j)
              {
                if (resPyrs[j].isEmpty())
                  resPyrs[j] =  (curPyr * itsInteractCoeffs[coeff_ptr]);
                else
                  resPyrs[j] += (curPyr * itsInteractCoeffs[coeff_ptr]);

                // count down the coeff_ptr and wrap it around
                // We count down here, because at parsing the input string
                // we have used push_back and hence reversed the order.
                coeff_ptr = (coeff_ptr - 1) % numChans();
              }
          }

        // now clamp these pyramids and feed them back to the channel
        for (uint i = 0; i < numChans(); ++i)
          {
            doLowThresh(resPyrs[i],0.0f,0.0f);
            gabor(i).itsPq.back().pyr = resPyrs[i];
            LINFO("Orientation pyramid interactions (%d/%d) ok.",
                  i+1, numChans());
          }

        // done
        break;
      }
    default: LFATAL("Unknown orientation interaction type: %d",
                    itsInteractType);
    } // end switch
}

// ######################################################################
Image<float> OrientationChannel::getOutput()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!itsDidInteractions)
    {
      doInteractions();
      itsDidInteractions = true;
    }
  return ComplexChannel::getOutput();
}

// ######################################################################
OrientationChannel::InteractType OrientationChannel::getInteractType()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsInteractType;
}

// ######################################################################
void OrientationChannel::setInteractType(OrientationChannel::InteractType type)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsInteractType = type;
}

// ######################################################################
std::vector<float> OrientationChannel::getInteractCoeffs()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsInteractCoeffs;
}

// ######################################################################
void OrientationChannel::setInteractCoeffs(std::vector<float>& coeffs)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsInteractCoeffs = coeffs;
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // ORIENTATIONCHANNEL_C_DEFINED
