/**
   \file  Robots/LoBot/irccm/LoBeep.c
   \brief Some quick functions to have the iRobot Create make some noise.

   This file defines the sound generation functions for Robolocust's
   low-level control program that runs on the iRobot Create's Command
   Module.
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
   Primary maintainer for this file: Manu Viswanathan mviswana usc edu
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoBeep.c $
   $Id: LoBeep.c 12838 2010-02-14 14:28:03Z mviswana $
*/

/*------------------------------ HEADERS ------------------------------*/

// lobot headers
#include "LoBeep.h"
#include "LoIO.h"
#include "LoTimer.h"
#include "LoOpenInterface.h"

/*-------------------------- INITIALIZATION ---------------------------*/

void lo_init_beeps(void)
{
   lo_tx(LOBOT_OI_CMD_DEFINE_SONG) ;
   lo_tx(LOBOT_BEEP_STARTUP) ;
   //lo_tx(16) ;
   lo_tx(7) ;
   lo_tx(76) ; lo_tx(29) ;
   lo_tx(79) ; lo_tx(22) ;
   lo_tx(76) ; lo_tx(15) ;
   lo_tx(76) ; lo_tx(7)  ;
   lo_tx(81) ; lo_tx(15) ;
   lo_tx(76) ; lo_tx(15) ;
   lo_tx(74) ; lo_tx(15) ;
   /*
   lo_tx(76) ; lo_tx(29) ;
   lo_tx(83) ; lo_tx(22) ;
   lo_tx(76) ; lo_tx(15) ;
   lo_tx(76) ; lo_tx(7) ;
   lo_tx(84) ; lo_tx(15) ;
   lo_tx(83) ; lo_tx(15) ;
   lo_tx(79) ; lo_tx(15) ;
   lo_tx(76) ; lo_tx(15) ;
   lo_tx(83) ; lo_tx(15) ;
   */
   lo_wait(15) ;

   lo_tx(LOBOT_OI_CMD_DEFINE_SONG) ;
   lo_tx(LOBOT_BEEP_HEARTBEAT) ;
   lo_tx(1) ;
   //lo_tx(109) ; lo_tx(4) ;
   lo_tx(36) ; lo_tx(6) ;
   lo_wait(15) ;

   lo_tx(LOBOT_OI_CMD_DEFINE_SONG) ;
   lo_tx(LOBOT_BEEP_QUITTING) ;
   lo_tx(9) ;
   lo_tx(76) ; lo_tx(15) ;
   lo_tx(83) ; lo_tx(15) ;
   lo_tx(88) ; lo_tx(15) ;
   lo_tx(76) ; lo_tx(7)  ;
   lo_tx(74) ; lo_tx(15) ;
   lo_tx(74) ; lo_tx(7)  ;
   lo_tx(71) ; lo_tx(15) ;
   lo_tx(78) ; lo_tx(15) ;
   lo_tx(76) ; lo_tx(29) ;
   lo_wait(15) ;
}

/*----------------------------- BEEPS API -----------------------------*/

void lo_beep(char beep_id)
{
   lo_tx(LOBOT_OI_CMD_PLAY_SONG) ;
   lo_tx(beep_id) ;
   //lo_wait(100) ;
}
