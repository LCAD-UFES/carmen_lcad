//
// This Program is provided by Duke University and the authors as a service to the
// research community. It is provided without cost or restrictions, except for the
// User's acknowledgement that the Program is provided on an "As Is" basis and User
// understands that Duke University and the authors make no express or implied
// warranty of any kind.  Duke University and the authors specifically disclaim any
// implied warranty or merchantability or fitness for a particular purpose, and make
// no representations or warranties that the Program will not infringe the
// intellectual property rights of others. The User agrees to indemnify and hold
// harmless Duke University and the authors from and against any and all liability
// arising out of User's use of the Program.
//
// slam.cpp
// Copyright 2005 Austin Eliazar, Ronald Parr, Duke University
//
// Main slam loop
//

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <math.h>
#include <strings.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "learn_ackerman.h"
#include "mt-rand_ackerman.h"

// The initial seed used for the random number generated can be set here.
#define SEED 1

// Default names for printing map files.
// File types are automatically appended.
#define MAP_PATH_NAME "map"
#define PARTICLES_PATH_NAME "particles"


//
// Globals
//

// The current commands being given to the robot for movement. 
// Not used when the robot is reading data from a log file.
double RotationSpeed, TranslationSpeed;

char *log_filename = NULL;

//
// This calls the procedures in the other files which do all the real work. 
// If you wanted to not use hierarchical SLAM, you could remove all references here to High*, and make
// certain to set LOW_DURATION in low.h to some incredibly high number.
//
void *Slam(void * /*arg unused */)
{
  TPath *path, *trashPath;
  TSenseLog *obs, *trashObs;
  double oldmC_D = 0;
  double oldmC_T = 0;
  double oldvC_D = 0;
  double oldvC_T = 0;
  double oldmD_D = 0;
  double oldmD_T = 0;
  double oldvD_D = 0;
  double oldvD_T = 0;
  double oldmT_D = 0;
  double oldmT_T = 0;
  double oldvT_D = 0;
  double oldvT_T = 0;

  outFile = fopen("motionModel.txt", "w");
  fprintf(outFile, "This file lists the motion model parameters in the following order : \n");
  fprintf(outFile, "    - mean value from observed translation\n");
  fprintf(outFile, "    - mean value from observed rotation\n");
  fprintf(outFile, "    - variance contributed from observed translation\n");
  fprintf(outFile, "    - variance contributed from observed rotation\n");
  fprintf(outFile, "\n");
  fprintf(outFile, "------------Intermediate Models------------------\n");
  fprintf(outFile, "   m_dist m_turn   v_dist v_turn\n");
  fprintf(outFile, "C: %.4f %.4f   %.4f %.4f\n", meanC_D, meanC_T, varC_D, varC_T);
  fprintf(outFile, "D: %.4f %.4f   %.4f %.4f\n", meanD_D, meanD_T, varD_D, varD_T);
  fprintf(outFile, "T: %.4f %.4f   %.4f %.4f\n", meanT_D, meanT_T, varT_D, varT_T);
  fprintf(outFile, "\n");
  fclose(outFile);

  while ((fabs(oldmC_D-meanC_D) > 0.01)||(fabs(oldmC_T-meanC_T) > 0.03)||(fabs(oldvC_D-varC_D) > 0.01)||(fabs(oldvC_T-varC_T) > 0.1) || 
	 (fabs(oldmD_D-meanD_D) > 0.01)||(fabs(oldmD_T-meanD_T) > 0.03)||(fabs(oldvD_D-varD_D) > 0.01)||(fabs(oldvD_T-varD_T) > 0.1) || 
	 (fabs(oldmT_D-meanT_D) > 0.03)||(fabs(oldmT_T-meanT_T) > 0.01)||(fabs(oldvT_D-varT_D) > 0.03)||(fabs(oldvT_T-varT_T) > 0.01)) {

	  readFile = carmen_fopen(PLAYBACK, "r");
  if(readFile == NULL)
	carmen_die("Error: could not open file %s for reading.\n", PLAYBACK);
  logfile_index = carmen_logfile_index_messages(readFile);

    InitLowSlam();
    LowSlam(&path, &obs);
    carmen_fclose(readFile);

    oldmC_D = meanC_D;    
    oldmC_T = meanC_T;
    oldvC_D = varC_D;    
    oldvC_T = varC_T;
    oldmD_D = meanD_D;    
    oldmD_T = meanD_T;
    oldvD_D = varD_D;    
    oldvD_T = varD_T;
    oldmT_D = meanT_D;    
    oldmT_T = meanT_T;
    oldvT_D = varT_D;    
    oldvT_T = varT_T;

    outFile = fopen("motionModel.txt", "a");
    Learn(path);

    printf("C: %.4f %.4f   %.4f %.4f\n", meanC_D, meanC_T, varC_D, varC_T);
    printf("D: %.4f %.4f   %.4f %.4f\n", meanD_D, meanD_T, varD_D, varD_T);
    printf("T: %.4f %.4f   %.4f %.4f\n", meanT_D, meanT_T, varT_D, varT_T);


    fclose(outFile);

    // Get rid of the path and log of observations
    while (path != NULL) {
      trashPath = path;
      path = path->next;
      free(trashPath);
    }
    while (obs != NULL) {
      trashObs = obs;
      obs = obs->next;
      free(trashObs);
    }
  }


  outFile = fopen("motionModel.txt", "a");
  fprintf(outFile, "\n");
  fprintf(outFile, "------------Final Model------------------\n");
  fprintf(outFile, "   m_dist m_turn   v_dist v_turn\n");
  fprintf(outFile, "C: %.4f %.4f   %.4f %.4f\n", meanC_D, meanC_T, varC_D, varC_T);
  fprintf(outFile, "D: %.4f %.4f   %.4f %.4f\n", meanD_D, meanD_T, varD_D, varD_T);
  fprintf(outFile, "T: %.4f %.4f   %.4f %.4f\n", meanT_D, meanT_T, varT_D, varT_T);

  fprintf(outFile, "\n");
  fprintf(outFile, "#define meanC_D %.4f\n#define meanC_T %.4f\n#define varC_D %.4f\n#define varC_T %.4f\n\n", meanC_D, meanC_T, varC_D, varC_T);
  fprintf(outFile, "#define meanD_D %.4f\n#define meanD_T %.4f\n#define varD_D %.4f\n#define varD_T %.4f\n\n", meanD_D, meanD_T, varD_D, varD_T);
  fprintf(outFile, "#define meanT_D %.4f\n#define meanT_T %.4f\n#define varT_D %.4f\n#define varT_T %.4f\n\n", meanT_D, meanT_T, varT_D, varT_T);
  fclose(outFile);

  CloseLowSlam();
  return NULL;
}



//
// Start of main program.
// IF things are set to read from a robot's sensors and not a data log, then this would be the best place
// to actually put in controls for the robot's behaviors and actions. The main SLAM process is called as a
// seperate thread off of this function.
//
int main (int argc, char *argv[])
{
  //  carmen_warn("Random seed: %d\n", carmen_randomize(&argc, &argv));

  if (argc != 2) 
    carmen_die("Usage: model_learner <logfile>\n");

  RECORDING =  (char*) "";
  PLAYBACK = argv[1];
  log_filename = PLAYBACK;

  carmen_warn("********** World Initialization ***********\n");

  seedMT(SEED);
  // Spawn off a seperate thread to do SLAM
  //
  // Should use semaphores or similar to prevent reading of the map
  // during updates to the map.
  //
  Slam(NULL);

  return 0;
}

