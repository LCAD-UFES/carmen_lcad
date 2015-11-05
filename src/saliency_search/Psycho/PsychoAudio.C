/*!@file Psycho/PsychoAudio.C Audio stimuli */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoAudio.C $
//


#ifdef INVT_HAVE_LIBSDL_MIXER

#include "Psycho/PsychoAudio.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Util/MathFunctions.H"

const ModelOptionCateg MOC_PsychoAudio = { MOC_SORTPRI_3, "PsychoAudio Options" };

const ModelOptionDef OPT_PsychoAudioBeepTone = 
{ MODOPT_ARG(float), "PsychoAudioBeepTone", &MOC_PsychoAudio, OPTEXP_CORE,  
  "tone of the beep stimulus", "beep-tone", '\0', "", "100.0" };  

const ModelOptionDef OPT_PsychoAudioBeepDuration = 
{ MODOPT_ARG(SimTime), "PsychoAudioBeepDuration", &MOC_PsychoAudio, OPTEXP_CORE,  
  "duration of beep stimulus", "beep-duration", '\0', "<SimTime>", "50ms" };  

const ModelOptionDef OPT_PsychoAudioMixerFreq = 
{ MODOPT_ARG(uint), "PsychoAudioMixerFreq", &MOC_PsychoAudio, OPTEXP_CORE,  
  "frequency (Hz) of audio mixer", "mixer-frequency", '\0', "", "44100" };  

const ModelOptionDef OPT_PsychoAudioMixerChannels = 
{ MODOPT_ARG(uint), "PsychoAudioMixerChannels", &MOC_PsychoAudio, OPTEXP_CORE,  
  "number of audio channels", "mixer-channels", '\0', "", "2" };  

const ModelOptionDef OPT_PsychoAudioMixerSamples = 
{ MODOPT_ARG(uint), "PsychoAudioMixerSamples", &MOC_PsychoAudio, OPTEXP_CORE,  
  "size of mixer samples", "mixer-sample-size", '\0', "", "4096" };  

// ######################################################################
PsychoAudio::PsychoAudio(OptionManager& mgr, const std::string& descrName,
                             const std::string& tagName) :
    ModelComponent(mgr, descrName, tagName),
    itsMixerFreq(&OPT_PsychoAudioMixerFreq, this), 
    itsMixerChannels(&OPT_PsychoAudioMixerChannels, this), 
    itsMixerSampleSize(&OPT_PsychoAudioMixerSamples, this),
    itsBeepTone(&OPT_PsychoAudioBeepTone, this),
    itsBeepDuration(&OPT_PsychoAudioBeepDuration, this),
    itsBeep(NULL),
    itsSounds(),
    itsEyeTracker()
{  }

// ######################################################################
PsychoAudio::~PsychoAudio()
{

}

// ######################################################################
void PsychoAudio::start1()
{
  if (SDL_Init(SDL_INIT_AUDIO) < 0) 
    LFATAL("Could not initialize SDL audio");
  
  //start up sdl mixer
  if(Mix_OpenAudio(itsMixerFreq.getVal(), MIX_DEFAULT_FORMAT, itsMixerChannels.getVal(), itsMixerSampleSize.getVal()) < 0) 
    LFATAL("Could not start up SDL mixer");
  
  //create the beep sound
  const float freq = (float)itsMixerFreq.getVal();
  const SimTime dt = SimTime::HERTZ(freq);

  const Uint32 numSamples = Uint32(freq * (itsBeepDuration.getVal().secs() / 1.0));
  const uint ns2 = numSamples / 2;
  Uint8 * const signal = (Uint8*)malloc(sizeof(Uint8) * numSamples);
  const float scale = pow(2, sizeof(Uint8)*4);
  SimTime t = SimTime::ZERO();

  for (uint i = 0; i < numSamples; ++i)
  {
    float ramp = 1.0F;;
    if (i <= ns2 / 2)
      ramp = (float)i  /  float(ns2 / 2);
    else if (i >= (ns2 + ns2 / 2))
      ramp = float(numSamples - i) / float(ns2 / 2);

    const float value = ramp * scale * sinf(t.secs() * 2.0F * D_PI * itsBeepTone.getVal());
    signal[i] = (Uint8)value;
    t += dt;
  }

  //setup the beep mix chunk
  itsBeep = (Mix_Chunk*)malloc(sizeof(Mix_Chunk));
  itsBeep->allocated = 1;
  itsBeep->abuf = signal;
  itsBeep->alen = numSamples;
  itsBeep->volume = MIX_MAX_VOLUME;
}

// ######################################################################
void PsychoAudio::stop1()
{
  if (itsBeep)
    Mix_FreeChunk(itsBeep);

  std::vector<Mix_Chunk*>::iterator chunk(itsSounds.begin()), end(itsSounds.end());
  while (chunk != end)
    Mix_FreeChunk(*chunk++);

  Mix_CloseAudio();
}

// ######################################################################
void PsychoAudio::setEyeTracker(nub::soft_ref<EyeTracker> e)
{ 
  itsEyeTracker = e; 
}

// ######################################################################
void PsychoAudio::playBeep()
{
  if (itsBeep)
    Mix_PlayChannel(-1, itsBeep, 0);
}

// ######################################################################
void PsychoAudio::playSound(const uint index)
{
  if (index < itsSounds.size())
    Mix_PlayChannel(-1, itsSounds[index], 0);
  else 
    LFATAL("Sound index out of range.");
}

// ######################################################################
void PsychoAudio::addWAV(const std::string& filename)
{
  itsSounds.push_back(Mix_LoadWAV(filename.c_str()));

  if (itsSounds.back() == NULL)
    LFATAL("could not load WAV file %s", filename.c_str());
}

#endif // HAVE_SDL_SDL_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
