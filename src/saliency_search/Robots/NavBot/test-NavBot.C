/*!@file Robots/NavBot/test-navBot.C Test the navbot robot*/

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/NavBot/test-NavBot.C $
// $Id: test-NavBot.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Robots/NavBot/NavBot.H"
#include "Util/log.H"
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <curses.h>

#define DOWN_KEY 258
#define UP_KEY 259
#define LEFT_KEY 260
#define RIGHT_KEY 261

nub::soft_ref<NavBot> navBot;

void terminate(int s)
{
        LINFO("*** INTERRUPT ***");
        navBot->stopAllMotors();
        exit(0);
}


void initScreen()
{
  //init the ncurses
  (void) initscr();      /* initialize the curses library */
  keypad(stdscr, TRUE);  /* enable keyboard mapping */
  (void) nonl();         /* tell curses not to do NL->CR/NL on output */
  intrflush(stdscr, FALSE);
  (void) noecho;        /* don't echo characters */
  (void) cbreak;         /* don't wait for enter before accepting input */
  intrflush(stdscr, FALSE);
}

int getKey()
{
  return getch();
}



int main(int argc, const char **argv)
{
        // Instantiate a ModelManager:
        ModelManager manager("Navbot Controller");

  navBot = nub::soft_ref<NavBot>(new NavBot(manager));
  manager.addSubComponent(navBot);

        // catch signals and redirect them to terminate for clean exit:
        signal(SIGHUP, terminate); signal(SIGINT, terminate);
        signal(SIGQUIT, terminate); signal(SIGTERM, terminate);
        signal(SIGALRM, terminate);

  initScreen();

        // Parse command-line:
        if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

        // let's get all our ModelComponent instances started:
        manager.start();

  LINFO("Starting NavBot");
  int key = getKey();
  int speed = 30;
  while(key != 'Q')
  {
    switch (key)
    {
      case KEY_UP:
        mvprintw(0,0,"Moving forward speed %i R:(%i:%i)\n", speed,
            navBot->setMotor(NavBot::LEFT_WHEEL, speed),
            navBot->setMotor(NavBot::RIGHT_WHEEL, speed));
        break;
      case KEY_DOWN:
        mvprintw(0,0,"Moving Back speed %i R:(%i,%i)\n", speed,
            navBot->setMotor(NavBot::LEFT_WHEEL, -1*speed),
              navBot->setMotor(NavBot::RIGHT_WHEEL, -1*speed));
        break;
      case KEY_LEFT:
        mvprintw(0,0,"Moving Left speed %i R:(%i,%i)\n", speed,
          navBot->setMotor(NavBot::LEFT_WHEEL, speed),
            navBot->setMotor(NavBot::RIGHT_WHEEL, -1*speed));
        break;
      case KEY_RIGHT:
        mvprintw(0,0,"Moving Right speed %i R:(%i,%i)\n", speed,
          navBot->setMotor(NavBot::LEFT_WHEEL, -1*speed),
            navBot->setMotor(NavBot::RIGHT_WHEEL, speed));
        break;
      case ' ':
        mvprintw(0,0,"Stop!! %i\n",
          navBot->stopAllMotors());
        break;

      case 'b':
        mvprintw(1,0,"Battery voltage %0.2f\n",
            navBot->getBatteryVoltage());
        break;

      case '+':
        speed += 5;
        mvprintw(0,0,"Speed %i\n", speed);
        break;
      case '-':
        speed -= 5;
        mvprintw(0,0,"Speed %i\n", speed);
        break;


    }
    key = getKey();
  }

        // stop all our ModelComponents
        manager.stop();

        // all done!
        return 0;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
