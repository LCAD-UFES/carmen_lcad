/**
   \file  Robots/LoBot/irccm/LoUtils.h
   \brief Utility functions.
*/

/*
 ************************************************************************
 * The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   *
 * by the University of Southern California (USC) and the iLab at USC.  *
 * See http://iLab.usc.edu for information about this project.          *
 *                                                                      *
 * Major portions of the iLab Neuromorphic Vision Toolkit are protected *
 * under the U.S. patent ``Computation of Intrinsic Perceptual Saliency *
 * in Visual Environments, and Applications'' by Christof Koch and      *
 * Laurent Itti, California Institute of Technology, 2001 (patent       *
 * pending; application number 09/912,225 filed July 23, 2001; see      *
 * http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     *
 ************************************************************************
 * This file is part of the iLab Neuromorphic Vision C++ Toolkit.       *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is free software; you can   *
 * redistribute it and/or modify it under the terms of the GNU General  *
 * Public License as published by the Free Software Foundation; either  *
 * version 2 of the License, or (at your option) any later version.     *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  *
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the   *
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      *
 * PURPOSE.  See the GNU General Public License for more details.       *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   *
 * to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   *
 * Boston, MA 02111-1307 USA.                                           *
 ************************************************************************
*/

/*
   Primary maintainer for this file: mviswana usc edu
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoUtils.h $
   $Id: LoUtils.h 13693 2010-07-24 21:59:05Z mviswana $
*/

#ifndef LOBOT_IRCCM_UTILS_DOT_H
#define LOBOT_IRCCM_UTILS_DOT_H

/*------------------------- NUMERIC FUNCTIONS -------------------------*/

/// Clamp the given value to lie within [min, max]
int lo_clamp(int value, int min, int max) ;

/// Return the sign of a number, i.e., -1 if the input number is
/// negative; +1 otherwise.
char lo_sign(int n) ;

/// Return a random number within the specified range.
//int lo_random(int min, int max) ;

/*--------------------------- BIT FIDDLING ----------------------------*/

/// Return the low byte of a 16-bit word
char lo_lobyte(int) ;

/// Return the high byte of a 16-bit word
char lo_hibyte(int) ;

/// Combine two bytes into a single word
int lo_make_word(char hi, char lo) ;

/*---------------------------------------------------------------------*/

#endif
