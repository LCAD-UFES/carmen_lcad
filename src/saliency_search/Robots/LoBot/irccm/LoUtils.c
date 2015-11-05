/**
   \file  Robots/LoBot/irccm/LoUtils.c
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
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoUtils.c $
   $Id: LoUtils.c 13693 2010-07-24 21:59:05Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoUtils.h"
#include "LoTimer.h"

// Standard C headers
#include <stdlib.h>

/*------------------------- NUMERIC FUNCTIONS -------------------------*/

int lo_clamp(int n, int min, int max)
{
   if (n < min)
      return min ;
   if (n > max)
      return max ;
   return n ;
}

char lo_sign(int n)
{
   return (n < 0) ? -1 : +1 ;
}

// DEVNOTE: Computationally expensive function due to 32-bit division
/*
int lo_random(int min, int max)
{
   static char seed ;
   if (! seed) {
      srandom(lo_ticks()) ;
      seed = 1 ;
   }

   long r = random() ;
   if (r == 0)
      ++r ;

   return min + (max - min)/((int)(((long) RANDOM_MAX)/r)) ;
}
// */

/*--------------------------- BIT FIDDLING ----------------------------*/

char lo_lobyte(int word)
{
   return (char)(word & 0xFF) ;
}

char lo_hibyte(int word)
{
   return (char)((word >> 8) & 0xFF) ;
}

int lo_make_word(char hi, char lo)
{
   int word = ((int) hi) & 0x00FF ;
   word <<= 8 ;
   return word | (((int) lo) & 0x00FF) ;
}
