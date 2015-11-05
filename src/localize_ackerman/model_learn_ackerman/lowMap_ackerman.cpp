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
// lowMap.c
//
// Copyright 2005, Austin Eliazar, Ronald Parr, Duke University
//
// Code for generating and maintaining maps (for the low level of the hierarchy)
//

#include <carmen/carmen.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <math.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "lowMap_ackerman.h"

// Unobserved grid squares are treated of having a prior of one stopped scan per 
// 8 meters of laser scan. 
#define L_PRIOR (-1.0/(MAP_SCALE*8.0))
// The strength of the prior is set to be as if 4 grid squares worth of observation 
// has already been made at the prior's density
#define L_PRIOR_DIST 4.0

// The global map for the low level, which contains all observations that any particle 
// has made to any specific grid square.
PMapStarter lowMap[MAP_WIDTH][MAP_HEIGHT];

// The nodes of the ancestry tree are stored here. Since each particle has a unique ID, 
// we can quickly access the particles via their ID in this array. See the structure 
// TAncestor in map.h for more details.
TAncestor l_particleID[ID_NUMBER];

// Our current set of particles being processed by the particle filter
TParticle l_particle[PARTICLE_NUMBER];
// We like to keep track of exactly how many particles we are currently using.
int l_cur_particles_used;
int FLAG;


//
// This process should be called at the start of each iteration of the slam process.
// It clears the observation cache so that it can be reloaded with the local maps 
// for the new iteration. 
//
void LowInitializeFlags()
{
  // Each entry observationArray corresponds to a single grid square in the global 
  // map. observationID is a count of how many of these entries there are. For each
  // one of these entries, obsX/obsY represent its x,y coordinate in the global map.
  // flagMap is an array the size of the global map, which gives a proper index into 
  // observationArray for that location. Therefore, we are resetting all non-zero 
  // entries of flagMap while resetting the arrays of obsX/obsY 
  while (observationID > 0) {
    observationID--;
    flagMap[obsX[observationID]][obsY[observationID]] = 0;
    obsX[observationID] = 0;
    obsY[observationID] = 0;
  }
  observationID = 1;
}


//
// Initializes the lowMap and the observationArray.
// Always returns 0 to indicate that it was successful.
//
void LowInitializeWorldMap()
{
  int x, y;

  for (y=0; y < MAP_HEIGHT; y++)
    for (x=0; x < MAP_WIDTH; x++) {
      // The map is a set of pointers. Null represents that it is unobserved.
      lowMap[x][y] = NULL;
      // flagMap is set to all zeros, indicating that location does not have an
      // entry in the observationArray
      flagMap[x][y] = 0;
    }

  // There are no entries in the observationArray yet, so obsX/obsY are set to 0
  for (x=0; x < AREA; x++) {
    obsX[x] = 0;
    obsY[x] = 0;
  }

  // observationArray[0] is reserved as a constant for "unused". We start the
  // array at 1.
  observationID = 1;
}



//
// Frees up all of the memory being used by the map. Completely erases all info in
// that map, making it ready for another slam implementation. In hierarchical slam, 
// this is called inbetween iterations of the high level slam, since each low level
// process runs essentially independently of previous low level processes.
//
void LowDestroyMap()
{
  int x, y;

  // Get rid of the old map.
  for (y=0; y < MAP_HEIGHT; y++)
    for (x=0; x < MAP_WIDTH; x++) {
      while (lowMap[x][y] != NULL) {
	free(lowMap[x][y]->array);
	free(lowMap[x][y]);
	lowMap[x][y] = NULL;
      }
    }
}



//
// Each grid square contains a dynamic array of the observations made at that grid 
// square. Therefore, these arrays need to be resized occasionally. In the process
// of resizing the array, we also clean up any redundant or obsolete "dead" entries.
//
void LowResizeArray(TMapStarter *node, int deadID)
{
  short int i, j, ID, x, y;
  short int hash[ID_NUMBER];
  int source, last;
  TMapNode *temp;

  // This is a special flag that can be raised when calling LowResizeArray, indicating
  // that a specific ID is "dead". Currently this is only used when the ancestry tree
  // is pruning off a dead branch, and the process of removing the corresponding 
  // observations leads to a reduction in the size of a dynamic array.
  if (deadID >= 0)
    node->dead++;

  // Create a new array of the appropriate size.
  // Don't count the dead entries in computing the new size
  node->size = (int)(ceil((node->total - node->dead)*1.75));
  temp = (TMapNode *) malloc(sizeof(TMapNode)*node->size);
  if (temp == NULL) fprintf(stderr, "Malloc failed in expansion of arrays.  %d\n", node->size);

  // Initialize our hash table.
  for (i=0; i < ID_NUMBER; i++)
    hash[i] = -1;

  j = 0;
  // Run through each entry in our old array of observations.
  for (i=0; i < node->total; i++) {
    if (node->array[i].ID == deadID && deadID >=0) {
      // Denote that this has been removed already. Therefore, we won't try to remove it later.
      // We don't bother actually removing the source, since the only way that we can have a deadID is if we are in
      // the process of removing all updates that deadID has made.
      l_particleID[deadID].mapEntries[node->array[i].source].node = -1;
    }

    // This observation is the first one of this ID entered into the new array. Just copy it over, and note its position.
    else if (hash[node->array[i].ID] == -1) {
      // Copy the information into the new array.
      temp[j].ID = node->array[i].ID;
      temp[j].source = node->array[i].source;
      temp[j].parentGen = node->array[i].parentGen;
      temp[j].hits = node->array[i].hits;
      temp[j].distance = node->array[i].distance;

      // This entry is moving- alter its source to track it
      l_particleID[ temp[j].ID ].mapEntries[ temp[j].source ].node = j;

      // Note that an observation with this ID has already been entered into the new array, and where that was entered.
      hash[node->array[i].ID] = j;
      j++;
    }

    // There is already an entry in the new array with the same ID, and this current observation is
    // actually more recent (as indicated by having seen more distance of laser scans). This current
    // observation will replace the older one.
    else if (node->array[i].distance > temp[hash[node->array[i].ID]].distance) {
      // We set a couple of values to shorter variable names, in order to reduce indirection and make 
      // reading the code easier.
      ID = node->array[i].ID;   // The ID of the observations in conflict.
      source = temp[hash[ID]].source;  // The ancestor node corresponding to that ID

      // Remove the source of the dead entry
      l_particleID[ID].total--;
      last = l_particleID[ID].total;
      l_particleID[ID].mapEntries[source].x = l_particleID[ID].mapEntries[last].x;
      l_particleID[ID].mapEntries[source].y = l_particleID[ID].mapEntries[last].y;
      l_particleID[ID].mapEntries[source].node = l_particleID[ID].mapEntries[last].node;

      // The last source entry was moved into this newly vacated position. Make sure that the 
      // observation it links to notes the new source position.
      x = l_particleID[ID].mapEntries[source].x;
      y = l_particleID[ID].mapEntries[source].y;

      if ((lowMap[x][y] == node) && (l_particleID[ID].mapEntries[source].node < i))
	temp[hash[ID]].source = source;
      else
	lowMap[x][y]->array[ l_particleID[ID].mapEntries[source].node ].source = source;

      // Copy the more recent information into the slot previously held by the dead entry
      temp[hash[ID]].source = node->array[i].source;
      temp[hash[ID]].hits = node->array[i].hits;
      temp[hash[ID]].distance = node->array[i].distance;
      // We do not copy over the parentGen- we are inheriting it from the dead entry, since it was the predecessor
      // The ID does not need to be copied, since it was necessarily the same for both observations.

      // This entry is moving- alter its source to track it
      l_particleID[ID].mapEntries[ node->array[i].source ].node = hash[ID];
    }

    // There was already an entry for this ID. This new entry is an older form of the observation already recorded. Therefore, 
    // the new entry is dead, and should not be copied over, and it's source in the ancestry tree should be removed.
    else {
      // The new entry is an older form of the one already entered. We should inherit the new parentGen
      if (node->array[i].parentGen != -1)
	temp[hash[node->array[i].ID]].parentGen = node->array[i].parentGen;

      ID = node->array[i].ID;
      source = node->array[i].source;

      // Remove the source of the dead entry
      l_particleID[ID].total--;
      last = l_particleID[ID].total;

      if (last != source) {
	l_particleID[ID].mapEntries[source].x = l_particleID[ID].mapEntries[last].x;
	l_particleID[ID].mapEntries[source].y = l_particleID[ID].mapEntries[last].y;
	l_particleID[ID].mapEntries[source].node = l_particleID[ID].mapEntries[last].node;

	// A source entry was moved. Make sure that the observation it links to notes the new source position.
	x = l_particleID[ID].mapEntries[source].x;
	y = l_particleID[ID].mapEntries[source].y;

	if ((lowMap[x][y] == node) && (l_particleID[ID].mapEntries[source].node <= i))
	  temp[hash[ID]].source = source;
	else
	  lowMap[x][y]->array[ l_particleID[ID].mapEntries[source].node ].source = source;
      }
    }

  }

  // Note the new total, which should be the previous size minus the dead.
  node->total = j;
  // After completing this process, we have removed all dead entries.
  node->dead = 0;
  free(node->array);
  node->array = temp;
}


//
// When we add a new entry to workingArray, there is a chance that we will run into a dead entry. 
// If so, we will need to delete the dead entry, by copying the last entry in the array onto its
// location. We then need to recursively add the entry (that we just copied onto that spot) to 
// the workingArray
//
static void AddToWorkingArray(int i, TMapStarter *node, short int workingArray[]) 
{
  int j, source, last;
  TEntryList *entries;

  // Keep an eye out for dead entries. They will be made apparent when two entries both have the same ID.
  if (workingArray[node->array[i].ID] == -1) 
    workingArray[node->array[i].ID] = i;

  else {
    // The node we are currently looking at is the dead one.
    if (node->array[i].distance < node->array[ workingArray[node->array[i].ID] ].distance) {
      // Otherwise, remove the source, then remove the entry. Follow with a recursive call.
      j = i;
      if (node->array[i].parentGen >= 0)
	node->array[ workingArray[node->array[i].ID] ].parentGen = node->array[i].parentGen;
    }

    // The previously entered entry is outdated. Replace it with this newer one.
    else {
      j = workingArray[node->array[i].ID];
      workingArray[node->array[i].ID] = i;
      if (node->array[j].parentGen >= 0)
	node->array[i].parentGen = node->array[j].parentGen;
    }
    
    // The node identified as "j" is dead. Remove its entry from the list of altered squares in the ancestor tree.
    l_particleID[node->array[j].ID].total--;

    entries = l_particleID[node->array[j].ID].mapEntries;
    source = node->array[j].source;
    last = l_particleID[node->array[j].ID].total;

    if (last != source) {
      entries[source].x = entries[last].x;
      entries[source].y = entries[last].y;
      entries[source].node = entries[last].node;

      // Somewhat confusing- we just removed an entry from the list of altered squares maintained by an ancestor particle (entries)
      // This means moving an entry from the end of that list to the spot which was vacated (entries[l_particleID[node->array[j].ID].total])
      // Therefore, the entry in the map corresponding to that last entry needs to point to the new entry.
      lowMap[ entries[source].x ][ entries[source].y ]->array[ entries[source].node ].source = source;
    }    

    // Now remove the node itself
    node->total--;
    node->dead--;

    if (j != node->total) {
      node->array[j].parentGen = node->array[node->total].parentGen;
      node->array[j].distance = node->array[node->total].distance;
      node->array[j].source = node->array[node->total].source;
      node->array[j].hits = node->array[node->total].hits;
      node->array[j].ID = node->array[node->total].ID;
      // We just moved the last entry in the list to position j. Update it's source entry in the ancestry tree to reflect its new position
      l_particleID[ node->array[j].ID ].mapEntries[ node->array[j].source ].node = j;

      // If the entry we just moved was in workingArray, we need to correct workingArray.
      // Also, we know that since it has been entered already, we don't need to enter it again
      if (workingArray[node->array[j].ID] == node->total)
	workingArray[node->array[j].ID] = j;
      else if (i != node->total) 
	// Final step- add this newly copied node to the working array (we don't want it skipped over)
	AddToWorkingArray(j, node, workingArray);
    }

  }
}


//
// This function is called whenever a grid square in the global map (which has at least one 
// observation associated with it) is accessed for the first time in an iteration. This
// function then creates an entry in the observationArray for this location.  This 
// effectively expands the local map by one grid square, and allows any future accesses to
// this grid square to be completed in constant time. This function itself can take O(P) time.
//
carmen_inline void LowBuildObservation(int x, int y, char usage)
{
  TAncestor *lineage;
  PAncestor stack[PARTICLE_NUMBER];
  short int workingArray[ID_NUMBER+1];
  int i, here, topStack;
  char flag = 0;

  // The size of the observationArray is not large enough- we throw out an error
  // message and stop the program
  if (observationID >= AREA) 
    fprintf(stderr, "aRoll over!\n");

  // Grab a slot in the observationArray
  flagMap[x][y] = observationID;
  obsX[observationID] = x;
  obsY[observationID] = y;
  observationID++;
  here = flagMap[x][y];

  // Initialize the slot and the ancestor particles
  for (i=0; i < ID_NUMBER; i++) {
    observationArray[here][i] = -1;
    workingArray[i] = -1;
    l_particleID[i].seen = 0;
  }

  // Fill in the particle entries of the array that made direct observations
  // to this grid square
  for (i=0; i < lowMap[x][y]->total; i++) 
    AddToWorkingArray(i, lowMap[x][y], workingArray);

  // A trick to speed up code when localizing. If an observation has no hits, 
  // then it has a 0% chance of stopping the laser, regardless of any other 
  // info. Marking that specially will remove an extra memory call, which 
  // would almost certainly be a cache miss. Also, if all entries are "empty",
  // then maybe we can skip the whole access to the observationArray entirely.
  if (usage) {
    flag = 1;
    for (i=0; i < lowMap[x][y]->total; i++) 
      if (lowMap[x][y]->array[i].hits > 0) 
	flag = 0;
      else
	workingArray[lowMap[x][y]->array[i].ID] = -2;
  }

  // Fill in the holes in the observation array, by using the value of their parents
  for (i=0; i < l_cur_particles_used; i++) {
    lineage = l_particle[i].ancestryNode;
    topStack = 0;

    // Eventually we will either get to an ancestor that we have already seen,
    // or we will hit the top of the tree (and thus its parent is NULL)
    // We never have to play with the root of the observation tree, because it has no parent
    while ((lineage != NULL) && (lineage->seen == 0)) {
      // put this ancestor on the stack to look at later
      stack[topStack] = lineage;
      topStack++;
      // Note that we already have seen this ancestor, for later lineage searches
      lineage->seen = 1;
      lineage = lineage->parent;  // Advance to this ancestor's parent
    }

    // Now trapse back down the stack, filling in each ancestor's info if need be
    while (topStack > 0) {
      topStack--;
      lineage = stack[topStack];
      // Try to fill in the holes of UNKNOWN. If the parent is also UNKNOWN, we know by construction 
      // that all of the other ancestors are also UNKNOWN, and thus the designation is correct
      if ((workingArray[lineage->ID] == -1) && (lineage->parent != NULL)) {
	workingArray[lineage->ID] = workingArray[lineage->parent->ID];
	// If any particle still has an "unobserved" value for this square, we can't use our cheat to
	// speed up code.
	if (workingArray[lineage->ID] == -1) 
	  flag = 0;
      }
    }
  }

  // If we are only localizing right now (usage) and all particles agree that this grid square
  // is empty (flag), then any access to this grid square doesn't even have to go as far as the
  // observation array- a glance at the flagMap can indicate that the desity is 0, regardless of
  // which particle is making the access.
  if ((usage) && (flag)) 
    flagMap[x][y] = -2;
  else
    for (i=0; i < ID_NUMBER; i++) 
      observationArray[here][i] = workingArray[i];
}




//
// Finds the appropriate entry in the designated grid square, and then makes a duplicate of that entry
// modified according to the input. 
//
void LowUpdateGridSquare(int x, int y, double distance, int hit, int parentID)
{
  TEntryList *tempEntry;
  int here, i;

  // If the grid square was previously unobserved, then we will need to create a new
  // entry in the observationArray for it, so that later accesses can take full advantage
  // of constant time access.
  if (lowMap[x][y] == NULL) {
    // Check to make sure there is still room left in the observation cache.
    if (observationID >= AREA) 
      fprintf(stderr, "bRoll over!\n");

    // Display ownership of this slot
    flagMap[x][y] = observationID;
    obsX[observationID] = x;
    obsY[observationID] = y;
    observationID++;

    // Since the grid square was unobserved previously, we will also need to create a
    // new entry into the map at this location, that we can then build on.
    // The first step is to create a starter structure, to keep track of the dynamic array
    // of observations.
    lowMap[x][y] = (TMapStarter *) malloc(sizeof(TMapStarter));
    if (lowMap[x][y] == NULL) fprintf(stderr, "Malloc failed in creation of Map Starter at %d %d\n", x, y);
    // No dead or obsolete entries yet.
    lowMap[x][y]->dead = 0;
    // No entries have actually been added to this location yet. We will increment this counter later.
    lowMap[x][y]->total = 0;
    // We will only have room for one observation in this grid square so far. Later, this can grow.
    lowMap[x][y]->size = 1;
    // The actual dynamic array is created here, of exactly the size for one entry.
    lowMap[x][y]->array = (TMapNode *) malloc(sizeof(TMapNode));
    if (lowMap[x][y]->array == NULL) fprintf(stderr, "Malloc failed in making initial map array for %d %d\n", x, y);

    // Initialize the slot
    for (i=0; i < ID_NUMBER; i++) 
      observationArray[flagMap[x][y]][i] = -1;
  }
  // We could have observations here, but this square hasn't been observed yet this iteration.
  // In that case, we need to build an entry into the observationArray for constant time access.
  else if (flagMap[x][y] == 0) 
    LowBuildObservation(x, y, 0);

  // Note where in the dynamic array of observations we need to look for this one particle's 
  // relevent observation. This is indicated to us by the observationArray.
  here = observationArray[flagMap[x][y]][parentID];

  // If the ID of the relevent observation is the same as our altering particle's ID, then the 
  // new observation is merely an amendment to this data, and noone else is using it yet. Just 
  // alter the source directly
  if ((here != -1) && (lowMap[x][y]->array[here].ID == parentID)) {
    lowMap[x][y]->array[here].hits = lowMap[x][y]->array[here].hits + hit;
    lowMap[x][y]->array[here].distance = lowMap[x][y]->array[here].distance + distance;
  }
  // Otherwise, we need to use that relevent observation in order to create a new observation.
  // Otherwise, we can corrupt the data for other particles.
  else {
    // We will be adding a new entry to the list- is there enough room?
    if (lowMap[x][y]->size <= lowMap[x][y]->total) {
      LowResizeArray(lowMap[x][y], -71);
      if (lowMap[x][y]->total == 0) {
	free(lowMap[x][y]->array);
	free(lowMap[x][y]);
	lowMap[x][y] = NULL;
      }
    }

    // Make all changes before incrementing lowMap[x][y]->total, since it's used as an index
    // Update the observationArray, to let it know that this ID will have its own special entry
    // in the global at this location.
    observationArray[flagMap[x][y]][parentID] = lowMap[x][y]->total;

    // Add an entry in to the list of altered map squares for this particle
    // First check to see if the size of that array is big enough to hold another entry
    if (l_particleID[parentID].size == 0) {
      l_particleID[parentID].size = 1;
      l_particleID[parentID].mapEntries = (TEntryList *) malloc(sizeof(TEntryList));
      if (l_particleID[parentID].mapEntries == NULL) fprintf(stderr, "Malloc failed in creation of entry list array\n");
    }
    else if (l_particleID[parentID].size <= l_particleID[parentID].total) {
      l_particleID[parentID].size = (int)(ceil(l_particleID[parentID].total*1.25));
      tempEntry = (TEntryList *) malloc(sizeof(TEntryList)*l_particleID[parentID].size);
      if (tempEntry == NULL) fprintf(stderr, "Malloc failed in expansion of entry list array\n");
      for (i=0; i < l_particleID[parentID].total; i++) {
	tempEntry[i].x = l_particleID[parentID].mapEntries[i].x;
	tempEntry[i].y = l_particleID[parentID].mapEntries[i].y;
	tempEntry[i].node = l_particleID[parentID].mapEntries[i].node;
      }
      free(l_particleID[parentID].mapEntries);
      l_particleID[parentID].mapEntries = tempEntry;
    }

    // Add the location of this new entry to the list in the ancestry node
    l_particleID[parentID].mapEntries[l_particleID[parentID].total].x = x;
    l_particleID[parentID].mapEntries[l_particleID[parentID].total].y = y;
    l_particleID[parentID].mapEntries[l_particleID[parentID].total].node = lowMap[x][y]->total;

    // i is used as a quick reference guide here, in order to make the code easier to read.
    i = lowMap[x][y]->total;
    // The pointers between the ancestry node's list and the map's observation list need
    // to point back towards each other, in order to coordinate data.
    lowMap[x][y]->array[i].source = l_particleID[parentID].total;
    // Assign the appropriate ID to this new observation.
    lowMap[x][y]->array[i].ID = parentID;
    // Note that we now have one more observation at this ancestor node
    l_particleID[parentID].total++;

    // Check to see if this square has been observed by an ancestor
    if (here == -1) {
      // Previously unknown; clean slate. Update directly with the observed data.
      lowMap[x][y]->array[i].hits = hit;
      // Include the strength of the prior.
      lowMap[x][y]->array[i].distance = distance + L_PRIOR_DIST;
      // A value of -2 here indicates that this observation had no preceding observation
      // that it was building off of.
      lowMap[x][y]->array[i].parentGen = -2; 
    }
    else {
      // Include the pertinent info from the old observation in with the new observation.
      lowMap[x][y]->array[i].hits = lowMap[x][y]->array[here].hits + hit;
      lowMap[x][y]->array[i].distance = distance + lowMap[x][y]->array[here].distance;
      lowMap[x][y]->array[i].parentGen = l_particleID[ lowMap[x][y]->array[here].ID ].generation;
    }

    // Now we can acknowledge that there is another observation at this grid square.
    lowMap[x][y]->total++;
  }
}




//
// Removes an observation from the map at position x,y. Node is the index into the
// dynamic array of which observation needs to be removed. This is typically gotten
// from the ancestry node, since the death of a node is currently the only way to 
// call this function.
//
void LowDeleteObservation(short int x, short int y, short int node) {
  int total;

  // We may have already removed this observation previously in the process of 
  // resizing the array at some point. In that case, there's nothing to do here.
  if ((node == -1) || (lowMap[x][y] == NULL))
    return;

  // If this is the last observation left at this location in the map, then we can 
  // revert the whole entry in the map to NULL, indicating that no current particle
  // has observed this location. 
  if (lowMap[x][y]->total - lowMap[x][y]->dead == 1) {
    free(lowMap[x][y]->array);
    free(lowMap[x][y]);
    lowMap[x][y] = NULL;
    return;
  }

  // Look to see if we need to shrink the array
  if ((int)((lowMap[x][y]->total - 1 - lowMap[x][y]->dead)*2.5) <= lowMap[x][y]->size) {
    // Let resizing the array remove this entry (it's what is considered a "dead" entry
    // now, as indicated by the second argument).
    LowResizeArray(lowMap[x][y], lowMap[x][y]->array[node].ID);
    if (lowMap[x][y]->total == 0) {
      free(lowMap[x][y]->array);
      free(lowMap[x][y]);
      lowMap[x][y] = NULL;
    }
    return;
  }

  // If we got this far, we are removing the entry manually. Make a note that we have
  // one less entry here.
  lowMap[x][y]->total--;
  // This variable is here solely to make the code mroe readable by having less 
  // redirections and indexes.
  total = lowMap[x][y]->total;
  // If the ibservation was the last one in the array, our work is done. Otherwise, 
  // we take the last observation in dynamic array, and copy it ontop of the observation
  // we are deleting. Since we don't use any of the entries beyond lowMap[x][y].total,
  // that last observation is essentially already deleted, and we don't have to worry about
  // duplicates.
  if (node != lowMap[x][y]->total) {
    lowMap[x][y]->array[node].hits      = lowMap[x][y]->array[total].hits;
    lowMap[x][y]->array[node].distance  = lowMap[x][y]->array[total].distance;
    lowMap[x][y]->array[node].ID        = lowMap[x][y]->array[total].ID;
    lowMap[x][y]->array[node].source    = lowMap[x][y]->array[total].source;
    lowMap[x][y]->array[node].parentGen = lowMap[x][y]->array[total].parentGen;
    l_particleID[ lowMap[x][y]->array[node].ID ].mapEntries[ lowMap[x][y]->array[node].source ].node = node;
  }
}



//
// Input: x, y- location of a grid square
//        distance- the length of a line passing through the square
//        variance- an output, which will be filled the variance for errors in the lasercast that 
//                  should stop here.
// Output: returns the probability of trace of the given length through this square will be stopped by an obstacle
//
carmen_inline double LowComputeProbability(int x, int y, double distance, int parentID) 
{
  // If there are no entries at this location in the map, we know that the observation
  // for any particle is UNKNOWN. Use the density of our prior for unknown grid squares
  if (lowMap[x][y] == NULL) 
    return (1.0 - exp(L_PRIOR * distance));

  // If this grid square has been observed already this iteration, the flagMap will show
  // how to get constant time access. If that value is set to 0, we know that this location
  // has yet to be accessed this iteration, and we have build the observation array entry 
  // for this square
  if (flagMap[x][y] == 0) 
    LowBuildObservation(x, y, 1);

  // If the flagMap is set to the constant -2, all particles agree that this location is
  // empty. We can avoid significant pointer redirection and memory accesses, and just
  // acknowledge that an empty square has probability 0 of stopping a scan.
  if (flagMap[x][y] == -2)
    return 0;

  // If the observationArray does not have an entry for this particle (as indicated by
  // the index of -1) then this location is considered UNKNOWN for this particle, and
  // we can use our prior value for density.
  if (observationArray[flagMap[x][y]][parentID] == -1)
    return (1.0 - exp(L_PRIOR * distance));
  // This value of -2 is a constant used to indicate that the square is empty, and
  // it is not necessary to access the lowMap, and risk a cache miss.
  if (observationArray[flagMap[x][y]][parentID] == -2)
    return 0;
  // If there is an entry in the observationArray, then we use that entry as an index
  // into the global map at the relevent location, and retrieve the information 
  // appropriate to compute the density, specifically the number of observed laser stops
  // in that grid square (hits) and the total distance of laser scans that we have
  // observed passing through that grid square (distance). density = hits/distance.
  // Note that if no laser scan have been observed to stop in this square, density is
  // zero, and no matter what the distance currently being observed to pass through the 
  // square, there is no chance that it will stop the scan. 
  if (lowMap[x][y]->array[ observationArray[flagMap[x][y]][parentID] ].hits == 0)
    return 0;
  return (1.0 - exp(-(lowMap[x][y]->array[ observationArray[flagMap[x][y]][parentID] ].hits/
		      lowMap[x][y]->array[ observationArray[flagMap[x][y]][parentID] ].distance) * distance));
}



// 
// This function performs the exact same function as the one above, except that it can work
// without the observation array. This is useful for printing out the map or doing debugging
// or similar investigation in areas which are not within the area currently being observed.
//
double LowComputeProb(int x, int y, double distance, int ID) 
{
  int i;

  if (lowMap[x][y] == NULL) 
    return UNKNOWN;

  while (1) {
    for (i=0; i < lowMap[x][y]->total; i++) {
      if (lowMap[x][y]->array[i].ID == ID) {
	if (lowMap[x][y]->array[i].hits == 0)
	  return 0;
	return (1.0 - exp(-(lowMap[x][y]->array[i].hits/lowMap[x][y]->array[i].distance) * distance));
      }
    }

    if (l_particleID[ID].parent == NULL)
      return UNKNOWN;
    else 
      ID = l_particleID[ID].parent->ID;
  }

  return UNKNOWN;
}




//
// Takes as input the parameters of a laser scan, and updates the world map appropriately
// startx and stary are the origins of the laser scan, MeasuredDist is how far it was
// was percieved to travel (the length of the line trace), and theta was the angle of the
// line (in radians). parentID lets us know which particle ID this update is associated with, 
// and addEnd = 1 when the laser scan was stopped by an object (instead of just travelling
// maximum range without seeing anything) indicating that the last grid square needs to be
// updated as occupied.
//
void LowAddTrace(double startx, double starty, double MeasuredDist, double theta, int parentID, int addEnd) {
  double overflow, slope; // Used for actually tracing the line
  int x, y, incX, incY, endx, endy;
  int xedge, yedge;       // Used in computing the midpoint. Recompensates for which edge of the square the line entered from
  double dx, dy;
  double distance, error;
  double secant, cosecant;   // precomputed for speed

  // Precomute a few numbers for speed.
  secant = 1.0/fabs(cos(theta));
  cosecant = 1.0/fabs(sin(theta));

  // This allows the user to limit the effective range of the sensor
  distance = MIN(MeasuredDist, MAX_SENSE_RANGE);

  // Mark the final endpoint of the line trace, so that we know when to stop.
  // We keep the endpoint as both float and int.
  dx = (startx + (cos(theta) * distance));
  dy = (starty + (sin(theta) * distance));
  endx = (int) (dx);
  endy = (int) (dy);

  // Ensures that we don't enter the loop: ray-tracing is done here. 
  if (endx == (int)startx && endy == (int)starty)
    return; 

  // Decide which x and y directions the line is travelling.
  // inc tells us which way to increment x and y. edge indicates whether we are computing
  // distance from the near or far edge.
  if (startx > dx) {
    incX = -1;
    xedge = 1;
  }
  else {
    incX = 1;
    xedge = 0;
  }
  
  if (starty > dy) {
    incY = -1;
    yedge = 1;
  }
  else {
    incY = 1;
    yedge = 0;
  }

  // Figure out whether primary motion is in the x or y direction. 
  // The two sections of code look nearly identical, with x and y reversed.
  if (fabs(startx - dx) > fabs(starty - dy)) {
    // The given starting point is non-integer. The line therefore starts at some point partially set in to the starting
    // square. Overflow starts at this offcenter amount, in order to make steps in the y direction at the right places.
    y = (int) (starty);
    overflow =  starty - y;
    // We always use overflow as a decreasing number- therefore positive y motion needs to 
    // adjust the overflow value accordingly.
    if (incY == 1)
      overflow = 1.0 - overflow;
    // Compute the effective slope of the line.
    slope = fabs(tan(theta));

    // The first square is a delicate thing, as we aren't doing a full square traversal in
    // either direction. So we figure out this strange portion of a step so that we can then
    // work off of the axes later. 
    // NOTE: we aren't computing the probability of this first step. Its a technical issue for
    // simplicity, and the odds of the sensor sitting on top of a solid object are sufficiently 
    // close to zero to ignore this tiny portion of a step. 
    error = fabs(((int)(startx)+incX+xedge)-startx);
    overflow = overflow - (slope*error);
    // The first step is actually in the y direction, due to the proximity of starty to the y axis. 
    if (overflow < 0.0) {
      y = y + incY;
      overflow = overflow + 1.0;
    }

    // Now we can start the actual line trace.
    for (x = (int) (startx) + incX; x != endx; x = x + incX) {
      overflow = overflow - slope;

      // Compute the distance travelled in this square
      if (overflow < 0.0)
	distance = (overflow+slope)*cosecant;
      else
	distance = fabs(slope)*cosecant;
      // Update every grid square we cross as empty...
      LowUpdateGridSquare(x, y, distance, 0, parentID);

      // ...including the overlap in the minor direction
      if (overflow < 0) {
	y = y + incY;
	distance = -overflow*cosecant;
	overflow = overflow + 1.0;
	LowUpdateGridSquare(x, y, distance, 0, parentID);
      }
    }

    // Update the last grid square seen as having a hit.
    if (addEnd) {
      if (incX < 0)
	distance = fabs((x+1) - dx)*secant;
      else
	distance = fabs(dx - x)*secant;
      LowUpdateGridSquare(endx, endy, distance, 1, parentID);
    }

  }

  // This is the same as the previous block of code, with x and y reversed.
  else {
    x = (int) (startx);
    overflow = startx - x;
    if (incX == 1)
      overflow = 1.0 - overflow;
    slope = 1.0/fabs(tan(theta));

    // (See corresponding comments in the previous half of this function)
    error = fabs(((int)(starty)+incY+yedge)-starty);
    overflow = overflow - (error*slope);
    if (overflow < 0.0) {
      x = x + incX;
      overflow = overflow + 1.0;
    }

    for (y = (int) (starty) + incY; y != endy; y = y + incY) {
      overflow = overflow - slope;
      if (overflow < 0)
	distance = (overflow+slope)*secant;
      else
	distance = fabs(slope)*secant;

      LowUpdateGridSquare(x, y, distance, 0, parentID);

      if (overflow < 0.0) {
	x = x + incX;
	distance = -overflow*secant;
	overflow = overflow + 1.0;
	LowUpdateGridSquare(x, y, distance, 0, parentID);
      }
    }

    if (addEnd) {
      if (incY < 0)
	distance = fabs(((y+1) - dy)/sin(theta));
      else
	distance = fabs((dy - y)/sin(theta));
      LowUpdateGridSquare(endx, endy, distance, 1, parentID);
    }
  }

}





//
// Inputs: x, y- starting point for the trace
//         theta- angle for the trace
//         measuredDist- the observed distance for this trace
//         parentID- the ID of the most recent member of the ancestry for the particle being considered
//         hit- really an output, this will be filled with the total probability that this laser cast
//              hit an obstruction before reaching the maximum range of the sensor
// Output: The total evaluated probability for this laser cast (unnormalized). 
//
double LowLineTrace(double startx, double starty, double theta, double MeasuredDist, int parentID, float culling) {
  double overflow, slope; // Used for actually tracing the line
  int x, y, incX, incY, endx, endy;
  double dx, dy;
  double totalProb; // Total probability that the line trace should have stopped before this step in the trace
  double eval;      // Total raw probability for the observation given this line trace through the map
  double prob, distance, error;
  double secant, cosecant;   // precomputed for speed
  double xblock, yblock;
  double xMotion, yMotion;
  double standardDist;

  // eval is the total probability for this line trace. Since this is a summation, eval starts at 0
  eval = 0.0;
  // totalProb is the total probability that the laser scan could travel this far through the map.
  // This starts at 1, and decreases as possible objects are passed.
  totalProb = 1.0;
  // a couple of variables are precomuted for speed.
  secant = 1.0/fabs(cos(theta));
  cosecant = 1.0/fabs(sin(theta));

  // If you look at Localize funtion in low.c, you can see that there are two different line traces
  // performed. The second trace is the full evaluation of the scan. The first trace is a kind of
  // approximate scan, only covering a small section near the percieved endpoint of the scan. The 
  // first trace is used as a way of culling out obviously bad particles without having to do too
  // much on them. For this "culling" trace, we specify directly how much further past the endpoint
  // we want to trace. When that culling number is set to zero, we know that it is the full trace
  // we are looking at, and we can as far out as 20 grid squares beyond the endpoint (anything further
  // has essentially zero probability to add to the scan.
  if (culling)
    distance = MeasuredDist+culling;
  else
    distance = MIN(MeasuredDist+20.0, MAX_SENSE_RANGE);

  // The endpoint of the scan, in both float and int.
  dx = (startx + (cos(theta) * distance));
  dy = (starty + (sin(theta) * distance));
  endx = (int) (dx);
  endy = (int) (dy);

  // Decide which x and y directions the line is travelling.
  if (startx > dx) {
    incX = -1;
    xblock = -startx;
  }
  else {
    incX = 1;
    xblock = 1.0-startx;
  }
  
  if (starty > dy) {
    incY = -1;
    yblock = -starty;
  }
  else {
    incY = 1;
    yblock = 1.0-starty;
  }
  
  // Two copies of the same basic code, swapping the roles of x and y, depending on which one is the primary 
  // direction of motion in the line trace.
  if (fabs(startx - dx) > fabs(starty - dy)) {
    y = (int) (starty);

    // The given starting point is non-integer. The line therefore starts at some point partially set in to the starting
    // square. Overflow starts at this off-center amount, in order to make steps in the y direction at the right places.
    overflow = starty - y;
    // Code is simpler if overflow is always decreasing towards zero. Note that slope is forced to be postive
    if (incY == 1) 
      overflow = 1.0 - overflow;

    slope = fabs(tan(theta));
    if (slope > 1.0) 
      slope = fabs((starty - dy) / (startx - dx));

    // The first square is a delicate thing, as we aren't doing a full square traversal in
    // either direction. So we figure out this strange portion of a step so that we can then
    // work off of the axes later. 
    // NOTE: we aren't computing the probability of this first step. Its a technical issue for
    // simplicity, and the odds of the sensor sitting on top of a solid object are sufficiently 
    // close to zero to ignore this tiny portion of a step. 
    dx = fabs((int)(startx)+xblock);
    dy = fabs(tan(theta)*dx);
    // The first step is actually in the y direction, due to the proximity of starty 
    // to the y axis. 
    if (overflow - dy < 0.0) {
      y = y + incY;
      overflow = overflow - dy + 1.0;
    }
    // Our first step is in fact in the x direction in this case. Set up for the overflow to 
    // be our starting offset plus this little extra we travel in the y direction.
    else 
      overflow = overflow - dy;

    // Most of the scans will be the same length across a grid square, entering and exiting on opposite
    // sides. Precompute this amount to save a little time.
    standardDist = slope*cosecant;

    // These two numbers help determine just how far away the endpoint of the scan is from the current
    // position in the scan. xMotion keeps track of the distance from the endpoint of the scan to next 
    // point where the line trace will cross the x-axis. Since each step in this loop moves us across 
    // exactly one grid square, this number will change by 1/cosine(theta) (ie secant) each iteration.
    // yMotion obviously does the same for the y axis.
    xMotion = -fabs(fabs(( ((int) (startx)) +xblock) * secant) - MeasuredDist);
    yMotion = -fabs(fabs((y+yblock) * cosecant) - MeasuredDist);

    for (x = (int) (startx) + incX; x != endx; x = x + incX) {
      // Update our two running counts.
      xMotion = xMotion + secant;
      overflow = overflow - slope;

      // Establish the distance travelled by the laser through this square. Note that this amount is
      // less than normal if the slope has overflowed, implying that the y-axis has been crossed.
      if (overflow < 0.0) 
	distance = (overflow+slope)*cosecant;
      else 
	distance = standardDist;

      // Compute the probability of the laser stopping in the square, given the particle's unique map.
      // Keep in mind that the probability of even getting this far in the trace is likely less than 1.
      prob = totalProb * LowComputeProbability(x, y, distance, parentID);
      if (prob > 0) {
	// If the scan had actually been stopped by an object in the map at this square,
	// how much error would there be in the laser? Determine which axis will be crossed 
	// next, and compute from there. (This value is actually kept as a running total now).
	if (overflow < 0.0)
	  error = fabs(yMotion);
	else
	  error = fabs(xMotion);

	// Increase the probability of the scan by the probability of stopping here, multiplied by the 
	// probability that a scan which stopped here could produce the error observed.
	// If the error is too large, the net effect on probability of the scan is essentially zero.
	// We can save some time by not computing this exponential.
	if (error < 20.0) 
	  eval = eval + (prob * exp(-(error*error)/(2*LOW_VARIANCE)));

	// Correspondingly decrease the probability that laser has continued.
	totalProb = totalProb - prob;
      }
    
      // If the overflow has dipped below zero, then the trace has crossed the y-axis, and we need to compute
      // everything for a single step in the y direction. 
      if (overflow < 0.0) {
	y += incY;
	yMotion = yMotion + cosecant;

	distance = -overflow*cosecant;
	overflow = overflow + 1.0;

	prob = totalProb * LowComputeProbability(x, y, distance, parentID);
	if (prob > 0) {
	  // There is no question about which axis will be the next crossed, since we just crossed the y-axis, 
	  // and x motion is dominant
	  error = fabs(xMotion);
	  if (error < 20.0) 
	    eval = eval + (prob * exp(-(error*error)/(2*LOW_VARIANCE)));
	}
	totalProb = totalProb - prob;
      }
    
    }
  }

  // ...second verse, same as the first...
  // Pretty much a direct copy of the previous block of code, with x and y reversed.
  else {
    x = (int) (startx);
    overflow = startx - x;
    if (incX == 1)
      overflow = 1.0 - overflow;
    slope = 1.0/fabs(tan(theta));

    // (See corresponding comments in the previous half of this function)
    dy = fabs((int)(starty)+yblock);
    dx = fabs(dy/tan(theta));
    if (overflow - dx < 0) {
      x = x + incX;
      overflow = overflow - dx + 1.0;
    }
    else 
      overflow = overflow - dx;

    standardDist = slope*secant;
    xMotion = -fabs(fabs((x+xblock) * secant) - MeasuredDist);
    yMotion = -fabs(fabs(( ((int) (starty)) +yblock) * cosecant) - MeasuredDist);

    for (y = (int) (starty) + incY; y != endy; y = y + incY) {
      yMotion = yMotion + cosecant;
      overflow = overflow - slope;

      if (overflow < 0.0) 
	distance = (overflow+slope)*secant;
      else 
	distance = standardDist;

      prob = totalProb * LowComputeProbability(x, y, distance, parentID);
      if (prob > 0) {
	if (overflow < 0.0) 
	  error = fabs(xMotion);
	else 
	  error = fabs(yMotion);
	if (error < 20.0) 
	  eval = eval + (prob * exp(-(error*error)/(2*LOW_VARIANCE)));
      }
      totalProb = totalProb - prob;
    
      if (overflow < 0.0) {
	x += incX;
	xMotion = xMotion + secant;

	distance = -overflow*secant;
	overflow = overflow + 1.0;

	prob = totalProb * LowComputeProbability(x, y, distance, parentID);
	if (prob > 0) {
	  error = fabs(yMotion);
	  if (error < 20.0) 
	    eval = eval + (prob * exp(-(error*error)/(2*LOW_VARIANCE)));
	}
	totalProb = totalProb - prob;
      }

    }
  }

  // If the laser reported a range beyond the maximum range allowed, any left-over probability that
  // the laser has not yet been stopped all has a probability of 1 to get the measured reading. We
  // therefore just add this remaining probability to the evaluation.
  if (MeasuredDist >= MAX_SENSE_RANGE) 
    return (eval + totalProb);

  // Otherwise, we know that the total probability of the laser being stopped at some point during 
  // the scan is 1. Normalize the evaluation to enforce this. 
  if (totalProb == 1)
    return 0;
  return (eval / (1.0 - totalProb));
}

