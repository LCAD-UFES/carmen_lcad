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
// low.c
// Copyright 2005 Austin Eliazar, Ronald Parr, Duke University
//


#include "low_ackerman.h"
#include "mt-rand_ackerman.h"

TOdo odometry;

extern char *log_filename;

//
// Error model for motion
// Note that the var terms are really the standard deviations. See our paper on
// learning motion models for mobile robots for a full explination of the terms.
// This model is for our ATRV Jr on a carpeted surface, at a specific period in
// in time (and thus a specific state of repair). Do not take this model as anything
// indicative of a model you should expect on your own robot.
// For a more modular design, you may find it useful to move these definitions out to
// the file "ThisRobot.c"
//
double meanC_D = 0.0;
double meanC_T = 0.0;
double varC_D = 0.2;
double varC_T = 0.2;

double meanD_D = 1.0;
double meanD_T = 0.0;
double varD_D = 0.4;
double varD_T = 0.2;

double meanT_D = 0.0;
double meanT_T = 1.0;
double varT_D = 0.2;
double varT_T = 0.4;

// Number of passes to use to cull good particles
#define PASSES 9

// Maximum error we will allow for a trace in grid squares. Basically, this is the
// level of of "background noise", or the base chance that something weird and completely
// random happened with the sensor, not conforming to our observation model.
#define MAX_TRACE_ERROR exp(-24.0/LOW_VARIANCE)
// A constant used for culling in Localize
#define WORST_POSSIBLE -10000

// Used for recognizing the format of some data logs.
#define LOG 0
#define REC 1
#define USC 2
int FILE_FORMAT;
carmen_FILE *readFile;
carmen_logfile_index_p logfile_index;

//
// Structures
//

// A sample is a lot like a short-lived particle. More samples are generated than particles,
// since we are certain that most of the generated samples will not get resampled. The basic
// difference is that since samples are not expected to be resampled, they don't need an entry
// in the ancestry tree, nor do they need to update the map.
struct TSample_struct {
   // The position of the sample, with theta being the facing angle of the robot.
  double x, y, theta; 
   // The incremental motion which this sample moved during this iteration.
   // D is the major axis of lateral motion, which is along the average facing angle during this time step
   // C is the minor axis, which is rotated +pi from D
   // T is the angular change in facing angle.
  double C, D, T;
   // The current probability of this sample, given the observations and this sample's map
  double probability;
   // The index into the array of particles, indicating the parent particle that this sample was resampled 
   // from. This is mostly used to determine the correct map to use when evaluating the sample.
  int parent;
};
typedef struct TSample_struct TSample;

 // The number of iterations between writing out the map as a png. 0 is off.
int L_VIDEO = 0;

 // Every particle needs a unique ID number. This stack keeps track of the unused IDs.
int cleanID;
int availableID[ID_NUMBER];

 // We generate a large number of extra samples to evaluate during localization, much larger than the number of true particles.
 // We store the samples that are being localized over in newSample, rather than keep a true particle for each.
TSample newSample[SAMPLE_NUMBER];
 // In order to compute the amount of percieved motion from the odometry, the last odometry readings are recorded 
 // The actual percieved movement is the current odometry readings minus these recorded 'last' readings.
double lastX, lastY, lastTheta;
 // No. of children each particle gets, based on random resampling
int children[PARTICLE_NUMBER];

 // savedParticle is where we store the current set of samples which were resampled, before we have
 // created an ID and an entry in the ancestry tree for each one.
TParticle savedParticle[PARTICLE_NUMBER];
int cur_saved_particles_used;

 // Keeps track of what iteration the SLAM process is currently on.
int curGeneration;
 // Stores the most recent set of laser observations.
TSense sense;
 // This array stores the color values for each grid square when printing out the map. For some reason,
 // moving this out as a global variable greatly increases the stability of the code.
unsigned char map[MAP_WIDTH][MAP_HEIGHT];

FILE *diffFile;




//
// AddToWorldModel
//
// sense- the current laser observations
// particleNum - the index into the particle array to the entry that is making additions to the map
//
void AddToWorldModel(TSense sense, int particleNum) {
  int j;

  // Run through each point that the laser found an obstruction at
  for (j=0; j < SENSE_NUMBER; j++) 
    // Normalize readings relative to the pose of current assumed position
    LowAddTrace(l_particle[particleNum].x, l_particle[particleNum].y, sense[j].distance, (sense[j].theta + l_particle[particleNum].theta), 
		l_particle[particleNum].ancestryNode->ID, (sense[j].distance < MAX_SENSE_RANGE));
}



//
// CheckScore
//
// Determine the fitness of a laser endpoint
//
// sense- the set of all current laser observations
// index- the specific number of the laser observation that we are going to score
// sampleNum- the idnex into the sample array for the sample we are currently concerned with
//
// Returns the unnormalized posterior for current particle
//
carmen_inline double CheckScore(TSense sense, int index, int sampleNum) 
{
  double a;

  a = LowLineTrace(newSample[sampleNum].x, newSample[sampleNum].y, (sense[index].theta + newSample[sampleNum].theta), 
		   sense[index].distance, l_particle[ newSample[sampleNum].parent ].ancestryNode->ID, 0);
  return MAX(MAX_TRACE_ERROR, a);
}



// 
// QuickScore
//
// Basically the same as CheckScore, except that the value returned is just a heuristic approximation,
// based on a small distance near the percieved endpoint of the scan, which is intended to be used to 
// quickly cull out bad samples, without having to evaluate the entire trace.
// The evaluation area is currently set at 3.5 grid squares before the endpoint to 3 grid squares past 
// the endpoint. There is no special reason for these specific values, if you want to change them.
//
carmen_inline double QuickScore(TSense sense, int index, int sampleNum) 
{
  double distance, eval;

  if (sense[index].distance >= MAX_SENSE_RANGE)
    return 1;

  distance = MAX(0, sense[index].distance-3.5);
  eval = LowLineTrace((int)(newSample[sampleNum].x + (cos(sense[index].theta + newSample[sampleNum].theta) * distance)),
		      (int)(newSample[sampleNum].y + (sin(sense[index].theta + newSample[sampleNum].theta) * distance)),
		      (sense[index].theta + newSample[sampleNum].theta), 
		      3.5, l_particle[ newSample[sampleNum].parent ].ancestryNode->ID, 3);
  return MAX(MAX_TRACE_ERROR, eval);
}



//
// Localize
//
// This is where the bulk of evaluating and resampling the particles takes place. 
// Also applies the motion model
//
void Localize(TSense sense)
{
  double ftemp; 
  double threshold;  // threshhold for discarding particles (in log prob.)
  double total, normalize; 
  double turn, distance, moveAngle; // The incremental motion reported by the odometer
  double CCenter, DCenter, TCenter, CCoeff, DCoeff, TCoeff;
  double tempC, tempD;  // Temporary variables for the motion model. 
  int i, j, k, p, best;  // Incremental counters.
  int keepers = 0; // How many particles finish all rounds
  int newchildren[SAMPLE_NUMBER]; // Used for resampling
  
  // Take the odometry readings from both this time step and the last, in order to figure out
  // the base level of incremental motion. Convert our measurements from meters and degrees 
  // into terms of map squares and radians
  distance = sqrt( ((odometry.x - lastX) * (odometry.x - lastX)) 
		 + ((odometry.y - lastY) * (odometry.y - lastY)) ) * MAP_SCALE;
  turn = (odometry.theta - lastTheta);

  // Keep motion bounded between pi and -pi
  if (turn > M_PI/3)
    turn = turn - 2*M_PI;
  else if (turn < -M_PI/3)
    turn = turn + 2*M_PI;

  // Our motion model consists of motion along three variables; D is the major axis of motion, 
  // which is lateral motion along the robot's average facing angle for this time step, C is the
  // minor axis of lateral motion, which is perpendicular to D, and T is the amount of turn in 
  // the robot's facing angle. 
  // Since the motion model is probablistic, the *Center terms compute the expected center of the
  // distributions of C D and T. Note that these numbers are each a function of the reported 
  // odometric distance and turn which have been observed. The constant meanC_D is the amount of 
  // effect that the distance reported from the odometry has on our motion model's expected motion
  // along the minor axis. All of these constants are defined at the top of this file.
  CCenter = distance*meanC_D + (turn*meanC_T*MAP_SCALE);
  DCenter = distance*meanD_D + (turn*meanD_T*MAP_SCALE);
  TCenter = (distance*meanT_D/MAP_SCALE) + turn*meanT_T;

  // *Coeff computes the standard deviation for each parameter when generating gaussian noise.
  // These numbers are limited to have at least some minimal level of noise, regardless of the
  // reported motion. This is especially important for dealing with a robot skidding or sliding
  // or just general unexpected motion which may not be reported at all by the odometry (it 
  // happens more often than we would like)
  CCoeff = MAX((fabs(distance*varC_D) + fabs(turn*varC_T*MAP_SCALE)), 0.8);
  DCoeff = MAX((fabs(distance*varD_D) + fabs(turn*varD_T*MAP_SCALE)), 0.8);
  TCoeff = MAX((fabs(distance*varT_D/MAP_SCALE) + fabs(turn*varT_T)), 0.10);

  // To start this function, we have already determined which particles have been resampled, and 
  // how many times. What we still need to do is move them from their parent's position, according
  // to the motion model, so that we have the appropriate scatter.
  i = 0;
  // Iterate through each of the old particles, to see how many times it got resampled.
  for (j = 0; j < PARTICLE_NUMBER; j++) {
    // Now create a new sample for each time this particle got resampled (possibly 0)
    for (k=0; k < children[j]; k++) {
      // We make a sample entry. The first, most important value is which of the old particles 
      // is this new sample's parent. This defines which map is being inherited, which will be
      // used during localization to evaluate the "fitness" of that sample.
      newSample[i].parent = j;
      
      // Randomly calculate the 'probable' trajectory, based on the movement model. The starting
      // point is of course the position of the parent.
      tempC = CCenter + GAUSSIAN(CCoeff); // The amount of motion along the minor axis of motion
      tempD = DCenter + GAUSSIAN(DCoeff); // The amount of motion along the major axis of motion
      // Record this actual motion. If we are using hierarchical SLAM, it will be used to keep track
      // of the "corrected" motion of the robot, to define this step of the path.
      newSample[i].C = tempC / MAP_SCALE;
      newSample[i].D = tempD / MAP_SCALE;
      newSample[i].T = TCenter + GAUSSIAN(TCoeff);
      newSample[i].theta = l_particle[j].theta + newSample[i].T;

      // Assuming that the robot turned continuously throughout the time step, the major direction
      // of movement (D) should be the average of the starting angle and the final angle
      moveAngle = (newSample[i].theta + l_particle[j].theta)/2.0;

      // The first term is to correct for the LRF not being mounted on the pivot point of the robot's turns
      // The second term is to allow for movement along the major axis of movement (D)
      // The last term is movement perpendicular to the the major axis (C). We add pi/2 to give a consistent
      // "positive" direction for this term. MeanC significantly shifted from 0 would mean that the robot
      // has a distinct drift to one side.
      newSample[i].x = l_particle[j].x + (TURN_RADIUS * (cos(newSample[i].theta) - cos(l_particle[j].theta))) +
 	               (tempD * cos(moveAngle)) + (tempC * cos(moveAngle + M_PI/2));
      newSample[i].y = l_particle[j].y + (TURN_RADIUS * (sin(newSample[i].theta) - sin(l_particle[j].theta))) +
 	               (tempD * sin(moveAngle)) + (tempC * sin(moveAngle + M_PI/2));
      newSample[i].probability = 0.0;
      i++;
    }
  }

  // Go through these particles in a number of passes, in order to find the best particles. This is
  // where we cull out obviously bad particles, by performing evaluation in a number of distinct
  // steps. At the end of each pass, we identify the probability of the most likely sample. Any sample
  // which is not within the defined threshhold of that probability can be removed, and no longer 
  // evaluated, since the probability of that sample ever becoming "good" enough to be resampled is
  // negligable.
  // Note: this first evaluation is based solely off of QuickScore- that is, the evaluation is only
  // performed for a short section of the laser trace, centered on the observed endpoint. This can
  // provide a good, quick heuristic for culling off bad samples, but should not be used for final
  // weights. Something which looks good in this scan can very easily turn out to be low probability
  // when the entire laser trace is considered.

  for (i = 0; i < SAMPLE_NUMBER; i++) 
    newSample[i].probability = 1.0;
  normalize = 1.0;
  threshold = PARTICLE_NUMBER;
  for (p = 0; p < PASSES; p++){
    best = 0;
    total = 0.0;
    for (i = 0; i < SAMPLE_NUMBER; i++) {
      if ((newSample[i].probability != WORST_POSSIBLE) && 
	  (1.0-pow(1.0-(newSample[i].probability/threshold), SAMPLE_NUMBER) > 1.0/(SAMPLE_NUMBER))) {
	newSample[i].probability = newSample[i].probability / normalize;
	for (k = p; k < SENSE_NUMBER; k += PASSES) 
	  newSample[i].probability = newSample[i].probability * QuickScore(sense, k, i); 
	if (newSample[i].probability > newSample[best].probability) 
	  best = i;

	total = total + newSample[i].probability;
      }
      else 
	newSample[i].probability = WORST_POSSIBLE;
    }
    normalize = newSample[best].probability;
    threshold = total;
  }


  keepers = 0;
  for (i = 0; i < SAMPLE_NUMBER; i++) 
    if (newSample[i].probability != WORST_POSSIBLE) {
      keepers++;
      // Don't let this heuristic evaluation be included in the final eval.
      newSample[i].probability = 1.0;
    }

  // Letting the user know how many samples survived this first cut.
  fprintf(stderr, "Better %d ", keepers);
  threshold = -1;

  // Now reevaluate all of the surviving samples, using the full laser scan to look for possible
  // obstructions, in order to get the most accurate weights. While doing this evaluation, we can
  // still keep our eye out for unlikely samples before we are finished.
  keepers = 0;
  normalize = 1.0;
  threshold = PARTICLE_NUMBER;
  for (p = 0; p < PASSES; p++){
    best = 0;
    total = 0.0;
    for (i = 0; i < SAMPLE_NUMBER; i++) {
      if ((newSample[i].probability != WORST_POSSIBLE) && 
	  (1.0-pow(1.0-(newSample[i].probability/threshold), SAMPLE_NUMBER) > 30.0/(SAMPLE_NUMBER))) {
	if (p == PASSES -1)
	  keepers++;
	newSample[i].probability = newSample[i].probability / normalize;
	for (k = p; k < SENSE_NUMBER; k += PASSES) 
	  newSample[i].probability = newSample[i].probability * CheckScore(sense, k, i); 
	if (newSample[i].probability > newSample[best].probability) 
	  best = i;

	total = total + newSample[i].probability;
      }
      else 
	newSample[i].probability = WORST_POSSIBLE;
    }
    normalize = newSample[best].probability;
    threshold = total;
  }

  // Report how many samples survived the second cut. These numbers help the user have confidence that
  // the threshhold values used for culling are reasonable.
  fprintf(stderr, "Best of %d ", keepers);

  // All probabilities are currently in log form. Exponentiate them, but weight them by the prob of the
  // the most likely sample, to ensure that we don't run into issues of machine precision at really small
  // numbers.
  total = 0.0;
  threshold = newSample[best].probability;
  for (i = 0; i < SAMPLE_NUMBER; i++) 
    // If the sample was culled, it has a weight of 0
    if (newSample[i].probability == WORST_POSSIBLE)
      newSample[i].probability = 0.0;
    else 
      total = total + newSample[i].probability;

  // Renormalize to ensure that the total probability is now equal to 1.
  for (i=0; i < SAMPLE_NUMBER; i++)
    newSample[i].probability = newSample[i].probability/total;

  total = 0.0;
  // Count how many children each particle will get in next generation
  // This is done through random resampling.
  for (i = 0; i < SAMPLE_NUMBER; i++) {
    newchildren[i] = 0;
    total = total + newSample[i].probability;
  }

  i = j = 0;  // i = no. of survivors, j = no. of new samples
  while ((j < SAMPLE_NUMBER) && (i < PARTICLE_NUMBER)) {
    k = 0;
    ftemp = MTrandDec()*total;
    while (ftemp > (newSample[k].probability)) {
      ftemp = ftemp - newSample[k].probability;
      k++;
    }    
    if (newchildren[k] == 0)
      i++;
    newchildren[k]++;
    j++;
  }

  // Report exactly how many samples are kept as particles, since they were actually
  // resampled.
  fprintf(stderr, "(%d kept) ", i);

  // Do some cleaning up
  // Is this even necessary?
  for (i = 0; i < PARTICLE_NUMBER; i++) {
    children[i] = 0;
    savedParticle[i].probability = 0.0;
  }

  // Now copy over new particles to savedParticles
  best = 0;
  k = 0; // pointer into saved particles
  for (i = 0; i < SAMPLE_NUMBER; i++)
    if (newchildren[i] > 0) {
      savedParticle[k].probability = newSample[i].probability;
      savedParticle[k].x = newSample[i].x;
      savedParticle[k].y = newSample[i].y;
      savedParticle[k].theta = newSample[i].theta;
      savedParticle[k].C = newSample[i].C;
      savedParticle[k].D = newSample[i].D;
      savedParticle[k].T = newSample[i].T;
      savedParticle[k].dist = distance / MAP_SCALE;
      savedParticle[k].turn = turn;
      // For savedParticle, the ancestryNode field actually points to the parent of this saved particle
      savedParticle[k].ancestryNode = l_particle[ newSample[i].parent ].ancestryNode;
      savedParticle[k].ancestryNode->numChildren++;
      children[k] = newchildren[i];

      if (savedParticle[k].probability > savedParticle[best].probability) 
	best = k;

      k++;
    }

  // This number records how many saved particles we are currently using, so that we can ignore anything beyond this
  // in later computations.
  cur_saved_particles_used = k;

  // We might need to continue generating children for particles, if we reach PARTICLE_NUMBER worth of distinct parents early
  // We renormalize over the chosen particles, and continue to sample from there.
  if (j < SAMPLE_NUMBER) {
    total = 0.0;
    // Normalize particle probabilities. Note that they have already been exponentiated
    for (i = 0; i < cur_saved_particles_used; i++) 
      total = total + savedParticle[i].probability;

    for (i=0; i < cur_saved_particles_used; i++)
      savedParticle[i].probability = savedParticle[i].probability/total;

    total = 0.0;
    for (i = 0; i < cur_saved_particles_used; i++) 
      total = total + savedParticle[i].probability;

    while (j < SAMPLE_NUMBER) {
      k = 0;
      ftemp = MTrandDec()*total;
      while (ftemp > (savedParticle[k].probability)) {
	ftemp = ftemp - savedParticle[k].probability;
	k++;
      }    
      children[k]++;

      j++;
    }
  }

  // Some useful information concerning the current generation of particles, and the parameters for the best one.
  fprintf(stderr, "-- %.3d (%.4f, %.4f, %.4f) : %.4f\n", curGeneration, savedParticle[best].x, savedParticle[best].y, 
	  savedParticle[best].theta, savedParticle[best].probability);
}



//
// DisposeAncestry
//
// When the SLAM process is complete, this function will clean up the memory being used by the ancestry
// tree, and remove all of the associated entries from the low level map.
//
void DisposeAncestry(TAncestor particleID[]) 
{
  int i, j;
  TPath *tempPath, *trashPath;
  TEntryList *entry;

  for (i = 0; i < ID_NUMBER; i++) {
    if (particleID[i].ID == i) {
      // Free up memory
      entry = particleID[i].mapEntries;
      for (j=0; j < particleID[i].total; j++)
	LowDeleteObservation(entry[j].x, entry[j].y, entry[j].node);
      free(entry);
      particleID[i].mapEntries = NULL;
      
      tempPath = particleID[i].path;
      while (tempPath != NULL) {
	trashPath = tempPath;
	tempPath = tempPath->next;
	free(trashPath);
      }
      particleID[i].path = NULL;

      particleID[i].ID = -123;
    }

    for (cleanID=0; cleanID < ID_NUMBER; cleanID++)
      availableID[cleanID] = cleanID;
    cleanID = ID_NUMBER;
  }
}



//
// UpdateAncestry
//
// Every iteration, after particles have been resampled and evaluated (ie after Localize has been run),
// we will need to update the ancestry tree. This consists of four main steps.
// a) Remove dead nodes. These are defined as any ancestor node which has no descendents in the current 
//    generation. This is caused by certain particles not being resampled, or by a node's children all
//    dying off on their own. These nodes not only need to be removed, but every one of their observations
//    in their observations in the map also need to be removed.
// b) Collapse branches of the tree. We want to restrict each internal node to having a branching factor
//    of at least two. Therefore, if a node has only one child, we merge the information in that node with
//    the one child node. This is especially common at the root of the tree, as the different hypotheses 
//    coalesce.
// c) Add the new particles into the tree. 
// d) Update the map for each new particle. We needed to wait until they were added into the tree, so that
//    the ancestry tree can keep track of the different observations associated with the particle.
//
void UpdateAncestry(TSense sense, TAncestor particleID[])
{
  int i, j;
  TAncestor *temp, *hold, *parentNode;
  TEntryList *entry, *workArray;
  TMapStarter *node;
  TPath *tempPath, *trashPath;

  // Remove Dead Nodes - 
  // Go through the current particle array, and prune out all particles that did not spawn any particles. We know that the particle
  // had to spawn samples, but those samples may not have become particles themselves, through not generating any samples for the 
  // next generation. Recurse up through there.
  for (i=0; i < l_cur_particles_used; i++) {
    temp = l_particle[i].ancestryNode;

    // This is a "while" loop for purposes of recursing up the tree.
    while (temp->numChildren == 0) {
      // Free up the memory in the map by deleting the associated observations.
      for (j=0; j < temp->total; j++) 
	LowDeleteObservation(temp->mapEntries[j].x, temp->mapEntries[j].y, temp->mapEntries[j].node);

      // Get rid of the memory being used by this ancestor
      free(temp->mapEntries);
      temp->mapEntries = NULL;

      // This is used exclusively for the low level in hierarchical SLAM. 
      // Get rid of the hypothesized path that this ancestor used.
      tempPath = temp->path;
      while (tempPath != NULL) {
	trashPath = tempPath;
	tempPath = tempPath->next;
	free(trashPath);
      }
      temp->path = NULL;

      // Recover the ID, so that it can be used later.
      cleanID++;
      availableID[cleanID] = temp->ID;
      temp->generation = curGeneration;
      temp->ID = -42;

      // Remove this node from the tree, while keeping track of its parent. We need that for recursing
      // up the tree (its parent could possibly need to be removed itself, now)
      hold = temp;
      temp = temp->parent;
      hold->parent = NULL;

      // Note the disappearance of this particle (may cause telescoping of particles, or outright deletion)
      temp->numChildren--;
    }
  }

  // Collapse Branches -
  // Run through the particle IDs, checking for those node IDs that are currently in use. Essentially is 
  // an easy way to pass through the entire tree.  If the node is in use, look at its parent.
  // If the parent has only one child, give all of the altered map squares of the parent to the child, 
  // and dispose of the parent. This collapses those ancestor nodes which have a branching factor of only one.
  for (i = 0; i < ID_NUMBER-1; i++) {
    // These booleans mean (in order) that the ID is in use, it has a parent (ie is not the root of the ancestry tree),
    // and that its parent has only one child (which is necessarily this ID) 
    if ((particleID[i].ID == i) && (particleID[i].parent != NULL) && (particleID[i].parent->numChildren == 1)) {
      // This is a special value for redirections. If the node's parent has already participated in a collapse
      // during this generation, then that node has already been removed from the tree, and we need to go up
      // one more step in the tree. 
      while (particleID[i].parent->generation == -111)
	particleID[i].parent = particleID[i].parent->parent;

      parentNode = particleID[i].parent;

      // Check to make sure that the parent's array is large enough to accomadate all of the entries of the child
      // in addition to its own. If not, we need to increase the dynamic array.
      if (parentNode->size < (parentNode->total + particleID[i].total)) {
	parentNode->size = (int)(ceil((parentNode->size + particleID[i].size)*1.5));
	workArray = (TEntryList *)malloc(sizeof(TEntryList)*parentNode->size);
	if (workArray == NULL) fprintf(stderr, "Malloc failed for workArray\n");

	for (j=0; j < parentNode->total; j++) {
	  workArray[j].x = parentNode->mapEntries[j].x;
	  workArray[j].y = parentNode->mapEntries[j].y;
	  workArray[j].node = parentNode->mapEntries[j].node;
	}
	// Note that parentNode->total hasn't changed- that will grow as the child's entries are added in
	free(parentNode->mapEntries);
	parentNode->mapEntries = workArray;
      }

      // Change all map entries of the parent to have the ID of the child
      // Also check to see if this entry supercedes an entry currently attributed to the parent.
      // Since collapses can merge all of the entries between the parent and the current child into the parent, this check is performed
      // by comparing to see if the generation of the last observation (before the child's update) is at least as recent as parent's
      // generation. If so, note that there is another "dead" entry in the observation array. It will be cleaned up later. If this puts
      // the total number of used slot, minus the number of "dead", below the threshold, shrink the array (which cleans up the dead)
      entry = particleID[i].mapEntries;
      for (j=0; j < particleID[i].total; j++) {
	node = lowMap[entry[j].x][entry[j].y];

	// Change the ID
	node->array[entry[j].node].ID = parentNode->ID;
	node->array[entry[j].node].source = parentNode->total;

	parentNode->mapEntries[parentNode->total].x = entry[j].x;
	parentNode->mapEntries[parentNode->total].y = entry[j].y;
	parentNode->mapEntries[parentNode->total].node = entry[j].node;
	parentNode->total++;

	// Check for pre-existing observation in the parent's list
	if (node->array[entry[j].node].parentGen >= parentNode->generation) {
	  node->array[entry[j].node].parentGen = -1;
	  node->dead++;
	}
      }

      // We do this in a second pass for a good reason. If there are more than one update for a given grid square which uses the child's
      // ID (as a consequence of an earlier collapse), then we want to make certain that the resizing doesn't take place until after all
      // entries have changed their ID appropriately.
      for (j=0; j < particleID[i].total; j++) {
	node = lowMap[entry[j].x][entry[j].y];
	if ((node->total - node->dead)*2.5 < node->size) 
	  LowResizeArray(node, -7);
      }

      // We're done with it- remove the array of updates from the child.
      free(particleID[i].mapEntries);
      particleID[i].mapEntries = NULL;

      // Inherit the path
      tempPath = parentNode->path;
      while (tempPath->next != NULL) 
	tempPath = tempPath->next;
      tempPath->next = particleID[i].path;
      particleID[i].path = NULL;

      // Inherit the number of children
      parentNode->numChildren = particleID[i].numChildren;

      // Subtlety of the ancestry tree: since we only keep pointers up to the parent, we can't exactly change all of the
      // descendents of the child to now point to the parent. What we can do, however, is mark the change for later, and
      // update all of the ancestor particles in a single go, later. That will take a single O(P) pass
      particleID[i].generation = -111;
    }
  }

  // This is the step where we correct for redirections that arise from the collapse of a branch of the ancestry tree
  for (i=0; i < ID_NUMBER-1; i++) 
    if (particleID[i].ID == i) {
      while (particleID[i].parent->generation == -111) 
	particleID[i].parent = particleID[i].parent->parent;
    }

  // Wipe the slate clean, so that we don't get confused by the mechinations of the previous changes
  // from deletes and merges. Updates can make thier own tables, as needed.
  LowInitializeFlags();


  // Add the current savedParticles into the ancestry tree, and copy them over into the 'real' particle array
  j = 0;
  for (i = 0; i < cur_saved_particles_used; i++) {
    // Check for redirection of parent pointers due to collapsing of branches (see above)
    while (savedParticle[i].ancestryNode->generation == -111) 
      savedParticle[i].ancestryNode = savedParticle[i].ancestryNode->parent;

    // A saved particle has ancestryNode denote the parent particle for that saved particle
    // If that parent has only this one child, due to resampling, then we want to perform a collapse
    // of the branch, but it hasn't been done yet, because the savedParticle hasn't been entered into
    // the tree yet. Therefore, we just designate the parent as the "new" entry for this savedParticle.
    // We then update the already created ancestry node as if it were the new node.
    if (savedParticle[i].ancestryNode->numChildren == 1) {
      // Change the generation of the node
      savedParticle[i].ancestryNode->generation = curGeneration;
      // Now that it represents the new node as well, it no longer is considered to have children.
      savedParticle[i].ancestryNode->numChildren = 0;
      // We're copying the savedParticles to the main particle array
      l_particle[j].ancestryNode = savedParticle[i].ancestryNode;

      // Add a new entry to the path of the ancestor node.
      trashPath = (TPath *)malloc(sizeof(TPath));
      trashPath->C = savedParticle[i].C;
      trashPath->D = savedParticle[i].D;
      trashPath->T = savedParticle[i].T;
      trashPath->dist = savedParticle[i].dist;
      trashPath->turn = savedParticle[i].turn;
      trashPath->next = NULL;
      tempPath = l_particle[i].ancestryNode->path;
      while (tempPath->next != NULL)
	tempPath = tempPath->next;
      tempPath->next = trashPath;

      l_particle[j].x = savedParticle[i].x;
      l_particle[j].y = savedParticle[i].y;
      l_particle[j].theta = savedParticle[i].theta;
      l_particle[j].probability = savedParticle[i].probability;
      j++;
    }

    // IF the parent has multiple children, then each child needs its own new ancestor node in the tree
    else if (savedParticle[i].ancestryNode->numChildren > 0) {
      // Find a new entry in the array of ancestor nodes. This is done by taking an unused ID off of the
      // stack, and using that slot. 
      temp = &(particleID[ availableID[cleanID] ]);
      temp->ID = availableID[cleanID];
      // That ID on the top of the stack is now being used.
      cleanID--;

      if (cleanID < 0) {
	fprintf(stderr, " !!! Insufficient Number of Particle IDs : Abandon Ship !!!\n");
	cleanID = 0;
      }

      // This new node needs to have its info filled in
      temp->parent = savedParticle[i].ancestryNode;
      // No updates to the map have been made yet for this node
      temp->mapEntries = NULL;
      temp->total = 0;
      temp->size = 0;
      // The generation of this node is important for collapsing branches of the tree. See above.
      temp->generation = curGeneration;
      temp->numChildren = 0;
      temp->seen = 0;

      // This is where we add a new entry to this node's hypothesized path for the robot
      trashPath = (TPath *)malloc(sizeof(TPath));
      trashPath->C = savedParticle[i].C;
      trashPath->D = savedParticle[i].D;
      trashPath->T = savedParticle[i].T;
      trashPath->dist = savedParticle[i].dist;
      trashPath->turn = savedParticle[i].turn;
      trashPath->next = NULL;
      temp->path = trashPath;

      // Transfer this entry over to the main particle array
      l_particle[j].ancestryNode = temp;
      l_particle[j].x = savedParticle[i].x;
      l_particle[j].y = savedParticle[i].y;
      l_particle[j].theta = savedParticle[i].theta;
      l_particle[j].probability = savedParticle[i].probability;
      j++;
    }
  }

  l_cur_particles_used = cur_saved_particles_used;

  // Here's where we actually go through and update the map for each particle. We had to wait
  // until now, so that the appropriate structures in the ancestry had been created and updated.
  for (i=0; i < l_cur_particles_used; i++) 
    AddToWorldModel(sense, i);

  // Clean up the ancestry particles which disappeared in branch collapses. Also, recover their IDs.
  // We waited until now because we needed to allow for redirection of parents.
  for (i=0; i < ID_NUMBER-1; i++) 
    if (particleID[i].generation == -111) {
      particleID[i].generation = -1;
      particleID[i].numChildren = 0;
      particleID[i].parent = NULL;
      particleID[i].mapEntries = NULL;
      particleID[i].path = NULL;
      particleID[i].seen = 0;
      particleID[i].total = 0;
      particleID[i].size = 0;

      // Recover the ID. 
      cleanID++;
      availableID[cleanID] = i;
      particleID[i].ID = -3;
    }
}



//
// ReadLog
//
// Reads back into the sensor data structures the raw readings that were stored to file by WriteLog (above)
// Reads a single line from the file, and interprets it by its delineator (either Laser or Odometry). If the line
// is not delineated for some reason, the function prints out an error message, and advances to the next line.
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
//
int ReadLog(carmen_FILE *logFile, carmen_logfile_index_p logfile_index, 
	    TSense &sense) {
  int i, max;
  char line[4096];
  //int laser;
  carmen_robot_ackerman_laser_message laser_msg;

  if (carmen_logfile_eof(logfile_index)) {
    fprintf(stderr, "End of Log File.\n");
    return 1;
  }

  do {
    carmen_logfile_read_next_line(logfile_index, logFile, 4095, line);
  } while (strncmp(line, "ROBOTLASER1 ", 12) != 0);

  memset(&laser_msg, 0, sizeof(laser_msg));
  carmen_string_to_robot_ackerman_laser_message(line, &laser_msg);

  max = laser_msg.num_readings;
  if (max > SENSE_NUMBER)
    max = SENSE_NUMBER;
      
  // Now read in the whole list of laser readings.
  for (i = 0; i < max; i++) {
    sense[i].theta = (i*M_PI/max) - M_PI/2;
    sense[i].distance = laser_msg.range[i] * MAP_SCALE;
    if (sense[i].distance > MAX_SENSE_RANGE)
      sense[i].distance = MAX_SENSE_RANGE;
  }
  // Read x and y coordinates. 
  odometry.x = laser_msg.laser_pose.x;
  odometry.y = laser_msg.laser_pose.y;
  // Read the facing angle of the robot.
  odometry.theta = laser_msg.laser_pose.theta;
  odometry.theta = carmen_normalize_theta(odometry.theta);

  return 0;
}



//
// PrintMap
//
// name - name of map file
// parent - a pointer to the specific particle you want to print out the map for
// particles - flag to indicate if position of all current particles should be shown on the map
// overlay* - if you want the specific particle's latest information which is just being entered into the
//            map to show up as different colors, you need to specify the position of the particle here. 
//            Values of -1 here turn this off.
//
void PrintMap(char *name, TAncestor *parent, int particles, double overlayX, double overlayY, double overlayTheta)
{
  FILE *printFile;
  int x, y, i;
  int width, height;
  int startx, starty, lastx, lasty;
  char sysCall[128];
  double hit, theta;

  width = MAP_WIDTH;
  height = MAP_HEIGHT;

  for(x=0; x < width; x++)
    for(y=0; y<height; y++)
      map[x][y] = 0;

  lastx = 0;
  lasty = 0;
  startx = width-1;
  starty = height-1;

  for (x = 0; x < width; x++) 
    for (y = 0; y < height; y++) {
      // The density of each grid square is reported, assuming a full diagonaly traversal of the square.
      // This gives good contrast.
      hit = LowComputeProb(x, y, 1.4, parent->ID);
      // All unknown areas are the same color. You can specify what color that is later
      if (hit == UNKNOWN) 
	map[x][y] = 255;
      else {
	// This specifies the range of grey values for the different squares in the map. Black is occupied.
	map[x][y] = (int) (230 - (hit * 230));
	// This allows us to only print out those sections of the map where there is something interesting happening.
	if (x > lastx)
	  lastx = x;
	if (y > lasty)
	  lasty = y;
	if (x < startx)
	  startx = x;
	if (y < starty)
	  starty = y;
      }
    }

  // If the command was given to print out the set of particles on the map, that's done here.
  if (particles) 
    for (i = 0; i < l_cur_particles_used; i++) 
      if ((l_particle[i].x > 0) && (l_particle[i].x < MAP_WIDTH) && (l_particle[i].y > 0) && (l_particle[i].y < MAP_HEIGHT))
	map[(int) (l_particle[i].x)][(int) (l_particle[i].y)] = 254;

  // And this is where the endpoints of the current scan are visualized, if requested.
  if (overlayX != -1) {
    map[(int) (overlayX)][(int) (overlayY)] = 254;
    for (i = 0; i < SENSE_NUMBER; i++) {
      theta = overlayTheta + sense[i].theta;
      x = (int) (overlayX + (cos(theta) * sense[i].distance));
      y = (int) (overlayY + (sin(theta) * sense[i].distance));

      if ((map[x][y] < 250) || (map[x][y] == 255)) {
	if (sense[i].distance < MAX_SENSE_RANGE) {
	  if (map[x][y] < 200)
	    map[x][y] = 251;
	  else 
	    map[x][y] = 252;
	}
	else
	  map[x][y] = 253;
      }
    }
  }


  // Header file for a ppm
  sprintf(sysCall, "%s.ppm", name);
  printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n",
	  lastx-startx+1, lasty-starty+1);
  fprintf(printFile, "255\n");

  // And this is where we finally print out the map to file. Note that there are number of special
  // values which could have been specified, which get special, non-greyscale values. Really,
  // you can play with those colors to your aesthetics.
  for (y = lasty; y >= starty; y--) 
    for (x = startx; x <= lastx; x++) {
      if (map[x][y] == 254) 
	fprintf(printFile, "%c%c%c", 255, 0, 0);
      else if (map[x][y] == 253) 
	fprintf(printFile, "%c%c%c", 0, 255, 200);
      else if (map[x][y] == 252) 
	fprintf(printFile, "%c%c%c", 255, 55, 55);
      else if (map[x][y] == 251) 
	fprintf(printFile, "%c%c%c", 50, 150, 255);
      else if (map[x][y] == 250) 
	fprintf(printFile, "%c%c%c", 250, 200, 200);
      else if (map[x][y] == 0) 
	fprintf(printFile, "%c%c%c", 100, 250, 100);
      else
	fprintf(printFile, "%c%c%c", map[x][y], map[x][y], map[x][y]);
    }
      
  // We're finished making the ppm file, and now convert it to png, for compressed storage and easy viewing.
  fclose(printFile);
  sprintf(sysCall, "convert %s.ppm %s.png", name, name);
  system(sysCall);
  sprintf(sysCall, "chmod 666 %s.ppm", name);
  system(sysCall);
  sprintf(sysCall, "chmod 666 %s.png", name);
  system(sysCall);
  fprintf(stderr, "Map dumped to file\n");
}



//
// The function to call (only once) before LowSlam is called, and initializes all values.
//
void InitLowSlam()
{
  int i;

  // All angle values will remain static
  for (i = 0; i < SENSE_NUMBER; i++) 
    sense[i].theta = (i*M_PI/180.0) - M_PI/2;

  curGeneration = 0;
}



//
// This function cleans up the memory and maps that were used by LowSlam.
// Well, it used to. Now LowSlam takes care of almost all of that by itself. 
//
void CloseLowSlam()
{
}


//
// The main function for performing SLAM at the low level. The first argument will return 
// whether there is still information to be processed by SLAM (set to 1). The second and third
// arguments return the corrected odometry for the time steps, and the corresponding list of
// observations. This can be used for the higher level SLAM process when using hierarchical SLAM.
//
void LowSlam(TPath **path, TSenseLog **obs)
{
  //int cnt;
  int i, j, overflow = 0;
  char name[32];
  TPath *tempPath;
  TSenseLog *tempObs;
  TAncestor *lineage;

  // Initialize the worldMap
  LowInitializeWorldMap();

  // Initialize the ancestry and particles
  cleanID = ID_NUMBER - 2;    // ID_NUMBER-1 is being used as the root of the ancestry tree.

  // Initialize all of our unused ancestor particles to look unused.
  for (i = 0; i < ID_NUMBER; i++) {
    availableID[i] = i;

    l_particleID[i].generation = -1;
    l_particleID[i].numChildren = 0;
    l_particleID[i].ID = -1;
    l_particleID[i].parent = NULL;
    l_particleID[i].mapEntries = NULL;
    l_particleID[i].path = NULL;
    l_particleID[i].seen = 0;
    l_particleID[i].total = 0;
    l_particleID[i].size = 0;
  }

  // Initialize the root of our ancestry tree.
  l_particleID[ID_NUMBER-1].generation = 0;
  l_particleID[ID_NUMBER-1].numChildren = 1;
  l_particleID[ID_NUMBER-1].size = 0;
  l_particleID[ID_NUMBER-1].total = 0;
  l_particleID[ID_NUMBER-1].ID = ID_NUMBER-1;
  l_particleID[ID_NUMBER-1].parent = NULL;
  l_particleID[ID_NUMBER-1].mapEntries = NULL;

  // Create all of our starting particles at the center of the map.
  for (i = 0; i < PARTICLE_NUMBER; i++) {
    l_particle[i].ancestryNode = &(l_particleID[ID_NUMBER-1]);
    l_particle[i].x = MAP_WIDTH / 2;
    l_particle[i].y = MAP_HEIGHT / 2;
    l_particle[i].theta = 0.001;
    l_particle[i].probability = 0;
    children[i] = 0;
  }
  // We really only use the first particle, since they are all essentially the same.
  l_particle[0].probability = 1;
  l_cur_particles_used = 1;
  children[0] = SAMPLE_NUMBER;

  // We don't need to initialize the savedParticles, since Localization will create them for us, and they first are used in 
  // UpdateAncestry, which is called after Localization. This statement isn't necessary, then, but serves as a sort of placeholder 
  // when reading the code.
  cur_saved_particles_used = 0;

  overflow = 1;

  for (i=0; i < START_ITERATION; i++)
    ReadLog(readFile, logfile_index, sense);

  curGeneration = 0;
  // Add the first thing that you see to the worldMap at the center. This gives us something to localize off of.
  ReadLog(readFile, logfile_index, sense);
  AddToWorldModel(sense, 0);
  curGeneration = 1;

  // Make a record of what the first odometry readings were, so that we can compute relative movement across time steps.
  lastX = odometry.x;
  lastY = odometry.y;
  lastTheta = odometry.theta;


  // Get our observation log started.
  (*obs) = (TSenseLog *)malloc(sizeof(TSenseLog));
  for (i=0; i < SENSE_NUMBER; i++) {
    (*obs)->sense[i].distance = sense[i].distance;
    (*obs)->sense[i].theta = sense[i].theta;
  }
  (*obs)->next = NULL;

  while (curGeneration < LEARN_DURATION) {
    // Collect information from the data log. If either reading returns 1, we've run out of log data, and
    // we need to stop now.
    if (ReadLog(readFile, logfile_index, sense) == 1)
      overflow = 0;
    else 
      overflow = 1;

    // We don't necessarily want to use every last reading that comes in. This allows us to make certain that the 
    // robot has moved at least a minimal amount (in terms of meters and radians) before we try to localize and update.
    if ((sqrt(SQUARE(odometry.x - lastX) + SQUARE(odometry.y - lastY)) < 0.10) && (fabs(odometry.theta - lastTheta) < 0.04))
      overflow = 0;

    if (overflow > 0) {
      overflow--;

      // Wipe the slate clean 
      LowInitializeFlags();

      // Apply the localization procedure, which will give us the N best particles
      Localize(sense);

      // Add these maintained particles to the FamilyTree, so that ancestry can be determined, and then prune dead lineages
      UpdateAncestry(sense, l_particleID);

      // Update the observation log (used only by hierarchical SLAM)
      tempObs = (*obs);
      while (tempObs->next != NULL)
	tempObs = tempObs->next;
      tempObs->next = (TSenseLog *)malloc(sizeof(TSenseLog));
      if (tempObs->next == NULL) fprintf(stderr, "Malloc failed in making a new observation!\n");
      for (i=0; i < SENSE_NUMBER; i++) {
	tempObs->next->sense[i].distance = sense[i].distance;
	tempObs->next->sense[i].theta = sense[i].theta;
      }
      tempObs->next->next = NULL;

      curGeneration++;

      // Remember these odometry readings for next time. This is what lets us know the incremental motion.
      lastX = odometry.x;
      lastY = odometry.y;
      lastTheta = odometry.theta;
    }
  }


  // Find the most likely particle. Return its path
  j = 0;
  for (i=0; i < l_cur_particles_used; i++) 
    if (l_particle[i].probability > l_particle[j].probability)
      j = i;

  (*path) = NULL;
  i = 0;
  lineage = l_particle[j].ancestryNode;
  while ((lineage != NULL) && (lineage->ID != ID_NUMBER-1)) {
    tempPath = lineage->path;
    i++;
    while (tempPath->next != NULL) {
      i++;
      tempPath = tempPath->next;
    }
    tempPath->next = (*path);

    (*path) = lineage->path;
    lineage->path = NULL;
    lineage = lineage->parent;
  }

  // Print out the map.
  sprintf(name, "map");
  j = 0;
  for (i = 0; i < l_cur_particles_used; i++)
    if (l_particle[i].probability > l_particle[j].probability)
      j = i;
  PrintMap(name, l_particle[j].ancestryNode, FALSE, -1, -1, -1);
  sprintf(name, "rm map.ppm");
  system(name);

  // Clean up the memory being used.
  DisposeAncestry(l_particleID);
  LowDestroyMap();
}

