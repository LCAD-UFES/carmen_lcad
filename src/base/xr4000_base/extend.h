 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef __INC_extend_h
#define __INC_extend_h

#ifdef __cplusplus
extern "C" {
#endif

struct N_XSonar {
  char *ID;
  char *Reference;
  double *Configuration;
};

struct N_XInfrared {
  char *ID;
  char *Reference;
  double *Configuration;
};

struct N_XInfraredSet {
  long int Dependency;
  double MainLobe;
  double Range;
  struct N_XInfrared Infrared[16];
};

struct N_XInfraredController {
  struct N_XInfraredSet InfraredSet[6];
};

struct N_XBumper {
  char *ID;
  char *Reference;
  double *Configuration;
};

struct N_XBumperSet {
  struct N_XBumper Bumper[12];
};

struct N_XBumperController {
  struct N_XBumperSet BumperSet[6];
};

struct N_XSonarSet {
  double MainLobe;
  double BlindLobe;
  double SideLobe;
  double BlindLobeAttenuation;
  double SideLobeAttenuation;
  double Range;
  struct N_XSonar Sonar[16];
};

struct N_XSonarController {
  struct N_XSonarSet SonarSet[6];
};

struct N_RobotStateExt {
  struct N_XSonarController SonarController;
  struct N_XInfraredController InfraredController;
  struct N_XBumperController BumperController;
};

struct N_RobotStateExt *N_GetRobotStateExt(long RobotID);

#ifdef __cplusplus
}
#endif

#endif
