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
// lowMap.h
//
// Copyright 2005, Austin Eliazar, Ronald Parr, Duke University
//
// lowMap.c contains a large amount of actual code which implement the way that maps are created,
// queried and maintained. This is basically a duplicate of highMap.*, which is in charge of 
// maintaining the high level mapping process. When not using hierarchical slam, highMap.* is
// ignored, and only lowMap.* is used.
//

#include "map_ackerman.h"

// The global map used by the low level slam process. These are pointers to MapStarter in order
// to save memory on te large amount of unobserved grid squares.
extern PMapStarter lowMap[MAP_WIDTH][MAP_HEIGHT];
// The nodes of the ancestry tree are stored here. Since each particle has a unique ID, we can 
// quickly access the particles via their ID in this array. See the structure TAncestor in map.h 
// for more details.
extern TAncestor l_particleID[ID_NUMBER];

// Our current set of particles being processed by the particle filter
extern TParticle l_particle[PARTICLE_NUMBER];
// We like to keep track of exactly how many particles we are currently using.
extern int l_cur_particles_used;


void LowInitializeFlags();
void LowInitializeWorldMap();
void LowDestroyMap();
void LowResizeArray(TMapStarter *node, int deadID);
void LowDeleteObservation(short int x, short int y, short int node);
double LowComputeProb(int x, int y, double distance, int ID);

void LowAddTrace(double startx, double starty, double MeasuredDist, double theta, int parentID, int addEnd);
double LowLineTrace(double startx, double starty, double theta, double MeasuredDist, int parentID, float culling);
