/*!@file Psycho/PsychoAudio.H audio psychophysics stimuli */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoAudio.H $

#ifndef PSYCHOAUDIO_H_DEFINED
#define PSYCHOAUDIO_H_DEFINED

#ifdef INVT_HAVE_LIBSDL_MIXER

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Psycho/EyeTracker.H"
#include "Util/SimTime.H"

#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>

class SimTime;

//! Class to do various psychoacoustics
/*! This class is to facilitate the producion of various audio stimuli 
    using SDL mixer library. See http://www.libsdl.org/ for information on sdl. */

class PsychoAudio : public ModelComponent
{
  public:
    // ######################################################################
    /*! @name Constructors, destructor and initialization */
    //@{

    //! Constructor
    PsychoAudio(OptionManager& mgr,
                const std::string& descrName = "Psycho Audio",
                const std::string& tagName = "PsychoAudio");
    //! Destructor
    ~PsychoAudio();

    //! Optionally we may work in conjunction with an EyeTracker
    void setEyeTracker(nub::soft_ref<EyeTracker> e);

    //@}

    // ######################################################################
    /*! @name sound production functions */
    //@{
    //!produce a beep sound
    void playBeep();

    //!play a sound out of the library at the given index
    void playSound(const uint index = 0);
    //@}

  protected:
    //! get the audio mixer started
    void start1();
    
    //! stop the audio mixer
    void stop1();

    //! add a sound to the bank
    void addWAV(const std::string& filename);
    
    OModelParam<uint> itsMixerFreq; //!< frquency (Hz) of sdl mixer
    OModelParam<uint> itsMixerChannels; //!< number of channels (1 : mono 2 : stere
    OModelParam<uint> itsMixerSampleSize; //!< size of samples in bytes
    
    OModelParam<float> itsBeepTone;     //!< tone of beep stimulus
    OModelParam<SimTime> itsBeepDuration; //!< duration of beep stimulus
    
  private:
    Mix_Chunk* itsBeep;
    std::vector<Mix_Chunk*> itsSounds;
    nub::soft_ref<EyeTracker> itsEyeTracker;
};

#endif
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
