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
// map.h
//
// Copyright 2005, Austin Eliazar, Ronald Parr, Duke University
//
// Important information on the maps, they way they are stored, and some information
// about particle numbers and the information each one needs to maintain.
//

#include "laser_ackerman.h"

#define UNKNOWN -2

// When using playback from a data log, you sometimes want to start mapping
// several steps in from the beginning of the file
#define START_ITERATION 10

// LOW LEVEL DEFINITIONS
// When using hierarchical slam, these are the values that are used by the low level
// When not using hierarchical, these are the only values that matter.
// See low.h for how to turn on and off hierarchical slam

// We need to know, for various purposes, how big our map is allowed to be
#define MAP_WIDTH  1700
#define MAP_HEIGHT 1700

// This is the number of particles that we are keeping at the low level
#define PARTICLE_NUMBER 300
// This is the number of samples that we will generate each iteration. Notice that we
// generate more samples than we will keep as actual particles. This is because so many 
// are "bad" samples, and clearly won't be resampled, thus we don't need to allocate nearly
// as much memory if we acknowledge that a certain amount will never be considered particles.
// "Localize" function in low.c can help explain.
#define SAMPLE_NUMBER (PARTICLE_NUMBER*10)
// Number of unique particle ID numbers. Each particle (and ancestry particle) gets its own 
// ID.  We recycle IDs that are no longer in use, thus this number can be bounded.
// ID_NUMBER is PARTICLE_NUMBER*2 since the ancestry is a tree; the additional .1 is for 
// breathing room
#define ID_NUMBER (int) (PARTICLE_NUMBER*2.2)


#define TOP_ID_NUMBER ID_NUMBER


// A bounding box on the semicircle of possible observations.
// This is useful for creating the observation cache (basically a set of local maps) which
// allows us to run in linear time. This number will grow with increased laser ranges.
#define AREA (int) (3.1415 * 55.0 *MAP_SCALE*MAP_SCALE)


// Used for passing the corrected odometric path from the low level to high level for
// further evaluation. Only used for hierarchical slam.
struct TPath_struct {
  // The incremental motion which this robot moved during a single time step.
  // D is the major axis of lateral motion, which is along the average facing angle during this time step
  // C is the minor axis, which is rotated +pi from D
  // T is the angular change in facing angle.
  float C, D, T;
  float dist, turn;
  struct TPath_struct *next;
};
typedef struct TPath_struct TPath;

// Like TPath, this is also used to pass sets of observations from the low level to the
// high level. Only used for hierarchical slam.
struct TSenseLog_struct {
  TSense sense;
  struct TSenseLog_struct *next;
};
typedef struct TSenseLog_struct TSenseLog;


// The maps are each made up of dynamic arrays of MapNodes. 
// Each entry maintains the total distance observed through this square (distance), and corresponding number
// of scans which were observed to stop here (hits). We also keep track of the ancestor particle which made
// the observation (ID), as well as the generation of the ancestor's observation this is a modification of, 
// if any (parentGen). We also keep an index into the array of modified grid squares maintained by the ancestor 
// particle which made the observation which corresponding to this update (source).
struct MapNode_struct;
struct MapNode_struct {
  // An index into the array of observations kept by the associated ancestor node. This
  // way, the array of updates for a node, and the observation itself can point to each other.
  int source;
  // The total distance that laser traces have been observed to pass through this grid square.
  float distance;
  // The number of times that a laser has been observed to stop in this grid square (implying a possible object)
  // Density of the square is hits/distance
  short int hits;
  // The ID of the ancestor node which made this observation
  short int ID;
  // If this observation is an update of a previous observation in this grid square, this indicates the generation
  // that the previous observation was made.
  short int parentGen;
};
typedef struct MapNode_struct *PMapNode;
typedef struct MapNode_struct TMapNode;

struct MapNodeStarter_struct;
struct MapNodeStarter_struct {
  // Total is the number of entries in the array which are currently being used.
  // Size is the total size of the array.
  // Dead indicates how many of those slots currently in use are taken up by obsolete entries
  short int total, size, dead;
  // The dynamic array which holds all of the observations for this grid square
  PMapNode array;
};
typedef struct MapNodeStarter_struct TMapStarter;
typedef struct MapNodeStarter_struct *PMapStarter;


// A dynamic array is stored by each ancestor particle of the map squares it has altered. We note which grid 
// square was altered (x, y) as well as an index into that square's array, corresponding to this alteration 
// (node).
struct TEntryList_struct;
struct TEntryList_struct {
  short int node;
  short int x, y;
};
typedef struct TEntryList_struct TEntryList;


// Holds the information needed for the ancestry tree. 
// For purposes of indexing in to the map efficiently, we keep which generation (iteration 
// of the particle filter) this particle was created in. 
//
// -Each ancestor node keeps a list of all of the observations made by this ancestor node (or inherited by it)
// mapEntries is a dynamic array maintaining this list.
// -size is the total available size of this dynamic array
// -total indicates how many of those entries are currently being used.
//
// -ID is the unique ID given to this specific particle, so it can be referred to by a "name"
// ID numbers will be reused if the particle is ever removed from the ancestry tree.
// -Each iteration of the particle filter involves resampling from the previous generation of particles.
// The new particle is the child, and old particle is the parent. 
// -numChildren keeps track of the number of particles in the tree that claim this particle as their parent
// (we don't need to know explicitly who they are). 
// -parent points to the particle in the tree that is supposedly the parent of this particle. 
// However, due to the collapsing of non-branching paths in the tree (see UpdateAncestry in low.c),
// the actual particle pointed to is the most recent ancestor of this particle that is shared by at least
// one other surviving particle.
struct TAncestor_struct;
struct TAncestor_struct {
  struct TAncestor_struct *parent;
  TEntryList *mapEntries;
  int size, total;
  short int generation, ID, numChildren;
  TPath *path;  // An addition for hierarchical- maintains the partial robot path represented by this particle
  char seen;  // Used by various functions for speedy traversal of the tree. 
};
typedef struct TAncestor_struct TAncestor;
typedef struct TAncestor_struct *PAncestor;


// Each particle stores certain information, such as its hypothesized postion (x, y, theta) in the global
// sense. Also, the motion taken to get to that point from the last iteration (C, D, T) and the current
// evaluated probability of this particle. For purposes of sorting the correct map for this particle,
// we also keep a pointer tothe particle's immediate parent (from which it was resampled).
struct TParticle_struct {
  float x, y, theta; // The current position of the particle, in terms of grid squares and radians
  float C, D, T;  // Minor and major axis of motion, and change of facing, respectively
  float dist, turn;
  double probability; // The proability of the particle
  // Which ancestor node this particle corresponds to. 
  TAncestor_struct *ancestryNode; 
};
typedef struct TParticle_struct TParticle;


// These are structures used to speed up the code, and allow for an efficient use of the observation cache.
// flagMap tells us, for a given position in the map, where we should look in the observation cache to find
// the "expanded" set of information for that grid square (where map accesses are constant time into an array).
// obsX/obsY do the opposite, and tell, for each entry of the observation cache, where in the map they 
// correspond to. This is most useful for cleaning up the observation cache and flagMap after each iteration.
extern int flagMap[MAP_WIDTH][MAP_HEIGHT];
extern short int obsX[AREA], obsY[AREA];

// This is where the actual observation cache is stored. For a given position in the global map, (x,y), 
// consult i=flagMap[x][y] to get the proper index into the observationArray. Now, observationArray[i][j] 
// will be the entry for the particle whose ID is j at map position (x,y). Rather than copying over all of the 
// info from the global map, this just gives a reference index into the appropriate grid square, so if 
// k=observationArray[i][j], then the actual information for particle j at (x,y) is map[x][y]->array[k], 
// which then contains fields such as hits, distance, etc.
extern short int observationArray[AREA][TOP_ID_NUMBER];

// The number of entries of observationArray currently being used.
extern int observationID;
