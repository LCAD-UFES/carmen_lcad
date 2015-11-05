/*!@file AppPsycho/psycho-keypad.H  this is a simple virtual keypad for psychophysics experiments to collect non-speeded response using mouse*/ 

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
// Primary maintainer for this file: Nader Noori <nnoori@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoKeypad.H$


#include "Component/ModelManager.H"
#include "Psycho/PsychoDisplay.H"
#include "Psycho/EyeTrackerConfigurator.H"
#include "Psycho/EyeTracker.H"
#include "Psycho/PsychoOpts.H"
#include "Component/EventLog.H"
#include "Component/ComponentOpts.H"
#include <sys/types.h>
#include <vector>



using namespace std;

#ifndef __PSYCHO_KEYPAD_H_
#define __PSYCHO_KEYPAD_H_

#include <SDL/SDL.h>
//pass an instance of PsychoDisplay along a vector of tokens, maximum number of selected tokens and a string for separating the tokens in the monitor 
vector<string> getKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl ,uint minl , string separator="", string message="");

vector<string> getSpeededKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl , uint minl , string separtor,string message, long waitTime);

vector<string> getSquaredKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl ,uint minl , string separator="", string message="");

vector<string> getSpeededSquaredKeypadResponse ( nub::soft_ref<PsychoDisplay> d,vector<string>& tokens , uint maxl ,uint minl , string separator, string message, long waitTime);


#endif

