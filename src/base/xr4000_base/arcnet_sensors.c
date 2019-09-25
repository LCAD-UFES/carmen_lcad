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

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <carmen/carmen.h>
#include "extend.h"
#include "Nclient.h"
#include "setup.h"

extern char *robd_program_name; 

void ARGCV_Create(char *string, int *argc_ret, char ***argv_ret)
{
  unsigned char literal_quote;
  unsigned char literal_slash;
  unsigned char between_args;
  long unsigned int length;
  long unsigned int si;
  long unsigned int vi;
  char *vstring;
  int argc;
  char **argv;
  
  literal_quote = 0;
  literal_slash = 0;
  between_args = 1;
  while ((*string != '\0') && (isspace(*string)))
    string++;

  length = strlen(string);
  if (length == 0) {
    *argc_ret = 0;
    *argv_ret = NULL;
    return;
  }
  
  vstring = (char *) calloc(length + 1, 1);
  carmen_test_alloc(vstring);
  argc = 0;
  
  argv = (char **) calloc(sizeof(char *), 1);
  carmen_test_alloc(argv);

  vi = 0;

  for (si = 0; si < length; si++) {
    if ((literal_quote == 0) && (literal_slash == 0) &&
	(string[si] == '"')) {
      literal_quote = 1;
      if (between_args == 1) {
	between_args = 0;
	argv = (char **) realloc(argv, (argc + 1) * sizeof(char *));
	carmen_test_alloc(argv);
	argv[argc] = vstring + vi;
	argc++;
      }
    } else if ((literal_quote == 1) && (literal_slash == 0) &&
	       (string[si] == '"')) {
      literal_quote = 0;
    } else if ((literal_slash == 0) && (string[si] == '\\')) {
      literal_slash = 1;
      if (between_args == 1) {
	between_args = 0;
	argv = (char **) realloc(argv, (argc + 1) * sizeof(char *));
	carmen_test_alloc(argv);
	argv[argc] = vstring + vi;
	argc++;
      }
    } else {
      if ((literal_quote == 0) && (literal_slash == 0)) {
	if (string[si] == 0x0a) {
	  if (between_args == 0) {
	    between_args = 1;
	    vstring[vi] = '\0';
	  }
	  vstring[vi] = '\0';
	  argv = (char **) realloc(argv, (argc + 1) * sizeof(char *));
	  carmen_test_alloc(argv);
	  argv[argc] = vstring + vi;
	  vi++;
	  argc++;
	  continue;
	} 
	if (isspace(string[si])) {     
	  if (between_args == 0) {
	    between_args = 1;
	    vstring[vi] = '\0';
	    vi++;
	  }
	  continue;
	}
      }       
      if (between_args == 1) {
	between_args = 0;
	argv = (char **) realloc(argv, (argc + 1) * sizeof(char *));
	carmen_test_alloc(argv);
	argv[argc] = vstring + vi;
	argc++;
      } 
      vstring[vi] = string[si];
      vi++;
      literal_slash = 0;
    } 
  }
  vstring[vi] = '\0';
  *argc_ret = argc;
  *argv_ret = argv;  
}

void ARGCV_Free(int argc __attribute__ ((unused)), char **argv)
{
  free(*argv);
  free(argv);
}

long int list_fill(double *mat, ...) 
{
  va_list ap; 
  long int r; 
  long int c; 
  long int i; 

  va_start(ap, mat);
  r = va_arg(ap, int);
  c = va_arg(ap, int);
  for (i = 0; i < r * c; i++) {
    (*(mat++)) = va_arg(ap, double);
  }
  va_end(ap);
  return 0;
}

static
void ParseSonarCommand(struct N_XSonar *xsonar, char *command, char *base_id)
{
  char **argv; 
  int argc; 
  double x, y, sine, cosine;

  ARGCV_Create(command, &argc, &argv);
  
  if(argc < 7)
    return;

  xsonar->ID = (char *)calloc(strlen(base_id) + strlen(argv[1]) + 2, 1);
  carmen_test_alloc(xsonar->ID);
  sprintf(xsonar->ID, "%s:%s", base_id, argv[1]);
  
  xsonar->Reference = strdup(argv[6]);
  
  x = atof(argv[2]);
  y = atof(argv[3]);
  cosine = atof(argv[4]);  
  sine = atof(argv[5]);
  
  xsonar->Configuration = (double *)calloc(9, sizeof(double));
  carmen_test_alloc(xsonar->Configuration);
  list_fill(xsonar->Configuration, 3, 3, 
	    cosine, -sine, x, sine, cosine, y, 0.0, 0.0, 1.0);
  
  ARGCV_Free(argc, argv);
  
  return;
}

void SON_Initialize(struct N_RobotState *rs, struct N_RobotStateExt *rs_ext)
{
  unsigned short s; 
  unsigned short sn; 
  char *const_string; 
  char **set_argv; 
  char **son_argv; 
  int set_argc; 
  int son_argc; 
  struct N_SonarSet *sonar_set; 
  struct N_XSonarSet *xset; 

  const_string = SETUP_GetValue("[sonar]sonar_sets");
  
  assert(const_string != ((void *)0));
  
  ARGCV_Create(const_string, &set_argc, &set_argv);

  if (set_argc > N_MAX_SONAR_SET_COUNT) {
    fprintf(stderr, "Setup: too many sonar sets in %s.\n"
	    "  Using the first %d.\n", "[sonar]sonar_sets", 
	    N_MAX_SONAR_SET_COUNT);
    rs->SonarController.SonarSetCount = N_MAX_SONAR_SET_COUNT;
  } else {
    rs->SonarController.SonarSetCount = set_argc;
  } 

  for (s = 0; s < rs->SonarController.SonarSetCount; s++) {
    sonar_set = &(rs->SonarController.SonarSet[s]);
    xset = &(rs_ext->SonarController.SonarSet[s]);

    const_string = SETUP_ExtGetValue(set_argv[s], "main_lobe");
    xset->MainLobe = (const_string != NULL) ? atof(const_string) : 0.0;
    const_string = SETUP_ExtGetValue(set_argv[s], "blind_lobe");
    xset->BlindLobe = (const_string != NULL) ? atof(const_string) : 0.0;
    const_string = SETUP_ExtGetValue(set_argv[s], "side_lobe");
    xset->SideLobe = (const_string != NULL) ? atof(const_string) : 0.0;
    const_string = SETUP_ExtGetValue(set_argv[s], "range");
    xset->Range = (const_string != NULL) ? atof(const_string) : 0.0;
    const_string = SETUP_ExtGetValue(set_argv[s], "blind_lobe_attenuation");
    xset->BlindLobeAttenuation = 
      (const_string != NULL) ? atof(const_string) : 0.0;
    const_string = SETUP_ExtGetValue(set_argv[s], "side_lobe_attenuation");
    xset->SideLobeAttenuation = 
      (const_string != NULL) ? atof(const_string) : 0.0;
    
    const_string = SETUP_ExtGetValue(set_argv[s], "sonars");
    
    if (const_string == NULL) {
      sonar_set->SonarCount = 0;
    } else { 
      ARGCV_Create(const_string, &son_argc, &son_argv);
      
      if (son_argc > N_MAX_SONAR_COUNT) {
	printf("Warning: too many sonars in [%s]%s.\n"
		"  Using the first %d.\n", set_argv[s], "sonars",
		N_MAX_SONAR_COUNT);
	sonar_set->SonarCount = N_MAX_SONAR_COUNT;
      } else {
	sonar_set->SonarCount = son_argc;
      }
      
      for (sn = 0; sn < sonar_set->SonarCount; sn++) {
	const_string = SETUP_ExtGetValue(set_argv[s], son_argv[sn]);
	if (const_string == NULL) {
	  fprintf(stderr, "Warning: could not find a setup entry for [%s]%s\n",
		  set_argv[s], son_argv[sn]);
	} else {
	  ParseSonarCommand(&(xset->Sonar[sn]), const_string, set_argv[s]);
	}
      }
      ARGCV_Free(son_argc, son_argv);
    }
  } 
  ARGCV_Free(set_argc, set_argv);
  return;
}

void INIT_InitializeSonars(long int robot_id)
{
  unsigned char active; 
  char set_key[20]; 
  char *forder_str; 
  char *end; 
  char **argv; 
  unsigned int set; 
  unsigned int sonar_num; 
  unsigned int sonar_index; 
  int argc; 
  struct N_RobotState *rs; 
  struct N_SonarSet *sonar_set; 

  rs = N_GetRobotState(robot_id);
  if (rs == NULL)
    return ;
  
  if (N_GetSonarConfiguration(robot_id))
    return ;
  
  active = 0;
  for (set = 0; set < rs->SonarController.SonarSetCount; set++) {
    sonar_set = &(rs->SonarController.SonarSet[set]);
    sprintf(set_key, "sonset%u", set);
    forder_str = SETUP_ExtGetValue(set_key, "firing_order");
    if (forder_str != NULL) {
      ARGCV_Create(forder_str, &argc, &argv);
      active = 1;
      for (sonar_index = 0; sonar_index < sonar_set->SonarCount; 
	   sonar_index++) {
	sonar_num = strtol(argv[sonar_index], &end, 0);
	if (end == argv[sonar_index]) {
	  fprintf(stderr, 
		  "%s: (warning) sonar entry %s in \"[%s]%s\" is invalid.\n",
		  robd_program_name, argv[sonar_index], set_key,
		  "firing_order");
	  break;
	} 
	if (sonar_num == 255) {
	  break;
	} 
	
	if (sonar_num >= sonar_set->SonarCount) {
	  fprintf(stderr, 
		  "%s: (warning) invalid sonar number %s in \"[%s]%s\".\n",
		  robd_program_name, argv[sonar_index], set_key, 
		  "firing_order");
	  break;
	}
	
	sonar_set->FiringOrder[sonar_index] = sonar_num;
      } 

      ARGCV_Free(argc, argv);
      
      sonar_set->FiringOrder[sonar_index] = 255;
    } 
  }
  
  if (active) {
    N_SetSonarConfiguration(robot_id);
  }
  return ;
}

void mysincos(double angle, double *sine, double *cosine) 
{
  *sine = sin(angle);
  *cosine = cos(angle);
  return;
}

static int ParseBumperCommand(struct N_XBumperSet *xset, 
			      unsigned short *bindex, char *set_id, 
			      char *command)
{
  char *id = NULL; 
  char *id_start = NULL; 
  char *reference = NULL; 
  char *tmp; 
  char **argv; 
  unsigned short n = 0; 
  unsigned short i; 
  int argc; 
  double sine; 
  double cosine; 
  double x = 0.0; 
  double y = 0.0; 
  double start_angle = 0.0; 
  double sweep = 0.0; 

  ARGCV_Create(command, &argc, &argv);
  
  if (argc <= 0) 
    return -1;
  
  if (strcasecmp(argv[0], "CreateBumperArc") == 0) {
    if (argc < 8) {
      ARGCV_Free(argc, argv);
      return -1;
    } 

    if ((n = atoi(argv[1])) == 0) {
      ARGCV_Free(argc, argv);
      return -1;
    } 
    
    id_start = argv[2];
    
    x = atof(argv[3]);
    y = atof(argv[4]);
    start_angle = atof(argv[5]);
    sweep = atof(argv[6]);
    
    if (sweep <= 0.0) {
      ARGCV_Free(argc, argv);
      return -1;
    } 
    
    reference = argv[7];
    
  } else if (strcasecmp(argv[0], "CreateBumper") == 0) { 
    if (argc < 7) {
      ARGCV_Free(argc, argv);
      return -1;
    } 
    
    id_start = NULL;
    n = 1;
    id = argv[1];
    x = atof(argv[2]);
    y = atof(argv[3]);
    start_angle = atan2(atof(argv[4]), atof(argv[5]));
    reference = argv[6];
    sweep = 0.0;
  } 

  if (*bindex + n > N_MAX_BUMPER_COUNT) {
    fprintf(stderr, "Bumper (warning): only %d bumpers per set allowed.\n",
	    N_MAX_BUMPER_COUNT);
    n = N_MAX_BUMPER_COUNT - 1 - *bindex;
  } 
  
  for (i = 0; i < n; i++) {
    if (id_start != NULL) {
      tmp = (char *) calloc(strlen(set_id) + strlen(id_start) + 
			    ((i != 0) ? 1 + ((int) log10(i)) : 1) + 2, 1);
      carmen_test_alloc(tmp);
      sprintf(tmp, "%s:%s%u", set_id, id_start, i);
    } else { 
      tmp = (char *) calloc(strlen(set_id) + strlen(id) + 2, 1);
      carmen_test_alloc(tmp);
      sprintf(tmp, "%s:%s", set_id, id);	
    } 
    
    xset->Bumper[*bindex].ID = tmp;

    xset->Bumper[*bindex].Configuration = 
      (double *)calloc(9, sizeof(double));
    carmen_test_alloc(xset->Bumper[*bindex].Configuration);

    mysincos(i / (double) n * sweep + start_angle, &sine, &cosine);
    list_fill(xset->Bumper[*bindex].Configuration, 3, 3, 
	      cosine, -sine, x, sine, cosine, y, 0.0, 0.0, 1.0);

    xset->Bumper[*bindex].Reference = strdup(reference);
    (*bindex)++;
  } 

  ARGCV_Free(argc, argv);

  return 0;
}

void BUMP_Initialize(struct N_RobotState *rs, struct N_RobotStateExt *rs_ext)
{
  char *const_string; 
  char **set_argv; 
  char **bump_argv; 
  unsigned short s; 
  unsigned short bumper_index; 
  int set_argc; 
  int bump_argc; 
  int argc_index; 
  struct N_XBumperSet *xset; 

  const_string = SETUP_GetValue("[bumper]bumper_sets");
  assert(const_string != ((void *)0));

  ARGCV_Create(const_string, &set_argc, &set_argv);
  
  if (set_argc > N_MAX_BUMPER_SET_COUNT) {
    fprintf(stderr, "Setup: too many bumper sets in %s.\n"
	    "  Using the first %d.\n", "[bumper]bumper_sets", 
	    N_MAX_BUMPER_SET_COUNT);
    rs->BumperController.BumperSetCount = N_MAX_BUMPER_SET_COUNT;
  } else {
    rs->BumperController.BumperSetCount = set_argc;
  } 
  
  
  for (s = 0; s < rs->BumperController.BumperSetCount; s++) {
    xset = &(rs_ext->BumperController.BumperSet[s]);
    if ((const_string = SETUP_ExtGetValue(set_argv[s], "bumpers")) == NULL) {
      rs->BumperController.BumperSet[s].BumperCount = 0;
    } else {
      ARGCV_Create(const_string, &bump_argc, &bump_argv);
      argc_index = 0;
      bumper_index = 0; 
      while (bumper_index < N_MAX_BUMPER_COUNT) { 
	if (argc_index == bump_argc)
	  break;
	
	const_string = SETUP_ExtGetValue(set_argv[s], bump_argv[argc_index]);
	if (const_string == NULL) {
	  fprintf(stderr, "Warning: could not find a setup entry for [%s]%s\n",
		  set_argv[s], bump_argv[argc_index]);
	  argc_index++;	  
	  continue;
	} 

	if (ParseBumperCommand(xset, &bumper_index, set_argv[s], 
			       const_string) < 0) {
	  fprintf(stderr, "Bumper (warning): invalid bumper "
		  "specification after [%s]%s.\n",
		  set_argv[s], bump_argv[argc_index]);
	} 
	argc_index++;
      }
      
      rs->BumperController.BumperSet[s].BumperCount = bumper_index;
      
      ARGCV_Free(bump_argc, bump_argv);
    }
  } 

  ARGCV_Free(set_argc, set_argv);
  
  return ;
}

static
void ParseInfraredCommand(struct N_XInfrared *xir, char *command, 
			  char *base_id)
{
  int argc; 
  char **argv; 
  double x; 
  double y; 
  double sine; 
  double cosine; 

  ARGCV_Create(command, &argc, &argv);

  assert(argc >= 7);

  xir->ID = calloc(strlen(base_id) + strlen(argv[1]) + 2, 1);
  carmen_test_alloc(xir->ID);
  sprintf(xir->ID, "%s:%s", base_id, argv[1]);
  
  xir->Reference = strdup(argv[6]);

  x = atof(argv[2]);
  y = atof(argv[3]);
  cosine = atof(argv[4]);
  sine = atof(argv[5]);
  
  xir->Configuration = (double *)calloc(9, sizeof(double));
  carmen_test_alloc(xir->Configuration);
  list_fill(xir->Configuration, 3, 3, 
	    cosine, -sine, x, sine, cosine, y, 0.0, 0.0, 1.0);
  
  ARGCV_Free(argc, argv);
  
  return;
}

void INF_Initialize(struct N_RobotState *rs, struct N_RobotStateExt *rs_ext)
{
  unsigned short s; 
  unsigned short ir; 
  char *const_string; 
  char **set_argv; 
  char **ir_argv; 
  int set_argc; 
  int ir_argc; 
  struct N_XInfraredSet *xset; 

  const_string = SETUP_GetValue("[infrared]infrared_sets");
  
  assert(const_string != ((void *)0));

  ARGCV_Create(const_string, &set_argc, &set_argv);

  if (set_argc > N_MAX_INFRARED_SET_COUNT) {
    printf("Warning: too many infrared sets in %s.\n"
	   "  Using the first %d.\n", "[infrared]infrared_sets", 
	    N_MAX_INFRARED_SET_COUNT);
    rs->InfraredController.InfraredSetCount = N_MAX_INFRARED_SET_COUNT;
  } else {
    rs->InfraredController.InfraredSetCount = set_argc;
  } 







  for (s = 0; s < rs->InfraredController.InfraredSetCount; s++) {
    xset = &(rs_ext->InfraredController.InfraredSet[s]);

    const_string = SETUP_ExtGetValue(set_argv[s], "main_lobe");
    assert(const_string != ((void *)0));
    xset->MainLobe = atof(const_string);

    const_string = SETUP_ExtGetValue(set_argv[s], "range");
    assert(const_string != ((void *)0));
    xset->Range = atof(const_string);



    const_string = SETUP_ExtGetValue(set_argv[s], "infrareds");
    assert(const_string != ((void *)0));
    ARGCV_Create(const_string, &ir_argc, &ir_argv);

    if (ir_argc > N_MAX_INFRARED_COUNT) {
      printf("Warning: too many infrareds in [%s]%s.\n"
	     "  Using the first %d.\n", set_argv[s], "infrareds",
	     N_MAX_INFRARED_COUNT);
      rs->InfraredController.InfraredSet[s].InfraredCount = 
	N_MAX_INFRARED_COUNT;
    } else {
      rs->InfraredController.InfraredSet[s].InfraredCount = ir_argc;
    } 

    for (ir = 0; ir < rs->InfraredController.InfraredSet[s].InfraredCount;
	 ir++) {
      if ((const_string = SETUP_ExtGetValue(set_argv[s], ir_argv[ir])) == 
	  NULL) {
	fprintf(stderr, "Warning: could not find a setup entry for [%s]%s\n",
		set_argv[s], ir_argv[ir]);
 
      } else { 
	ParseInfraredCommand(&(xset->Infrared[ir]), const_string,
			     set_argv[s]);
      } 
    } 
    ARGCV_Free(ir_argc, ir_argv);
  }
  ARGCV_Free(set_argc, set_argv);
  
  return ;
}


