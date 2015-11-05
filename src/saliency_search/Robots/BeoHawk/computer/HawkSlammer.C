// File: HawkSlammer.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include "Robots/BeoHawk/computer/HawkSlammer.H"

// ######################################################################
HawkSlammer::HawkSlammer(std::string myName, int argc, char* argv[])
: HawkAgent(myName, argc, argv),
window(Dims(320, 240)) {
	// Help section
	helpTitle = "HawkSlammer";
	helpDescription = "Runs an implementation of DP-SLAM for the BeoHawk.";
	helpOptions.push_back("\t--pause\t\t(false) Wait for keyboard input after each iteration.");
	helpOptions.push_back("\t--mode\t\t(simulation) Try realTimeDrop, realTimeCombine, or simulation.");
	helpOptions.push_back("\t--particles\t(100) Number of particles in the simulation.");
	helpOptions.push_back("\t--resolution\t(30) Size of each grid cell in mm.");
	helpOptions.push_back("\t--maxLaserDistance\t(6000) Largest distance that the laser scanner can detect.");
	
	// Parameters
	pauseEachStep = loadBoolParameter("pause", false);
	numberOfParticles = loadIntParameter("particles", 100);
	mapResolution = loadIntParameter("resolution", 30); // mm per grid cell
	maxLaserDistance = loadIntParameter("maxLaserDistance", 6000);
	robotXStdDev = loadDoubleParameter("robotXStdDev", 50); // 50 mm
	robotYStdDev = loadDoubleParameter("robotYStdDev", 50); // 50 mm
	robotThetaStdDev = loadDoubleParameter("robotThetaStdDev", 0.1); // ~6 deg
	scannerVariance = loadDoubleParameter("scannerVariance", 20); // 20 mm
	std::string modeIn = loadStringParameter("mode", "simulation");
	if(modeIn == "realTimeDrop") mode = REALTIME_DROP;
	else if(modeIn == "realTimeCombine") mode = REALTIME_COMBINE;
	else mode = SIMULATION;
	
	// Initialize SLAM
	initializeSlam();
}

// ######################################################################
void HawkSlammer::registerTopics() {
	registerPublisher("SlamDataMessage");
	registerSubscription("SensorDataMessage");
}

// ######################################################################
bool HawkSlammer::scheduler() {
	if(sensorDataMessages.size() > 0) {
		slam();
		return true;
	}
	
	return false;
}

// ######################################################################
void HawkSlammer::catchMessage(const HawkMessages::MessagePtr& hawkMessage, const Ice::Current&) {
	if(hawkMessage->ice_isA("::HawkMessages::SensorDataMessage"))
	{
		// Save message to storage queue
		HawkMessages::SensorDataMessagePtr msg = HawkMessages::SensorDataMessagePtr::dynamicCast(hawkMessage);
		sensorDataMessages.push_back(msg);
		wakeUp();
	}
}

// ######################################################################
void HawkSlammer::initializeSlam() {
	// Initialize grid
	int initialPixels = (int)(2 * maxLaserDistance / mapResolution);
	occupancyGrid = occupancyMap_t(initialPixels, initialPixels, NO_INIT);
	
	// Initialize tree head
	boost::shared_ptr<Particle> head(new Particle());
	head->parent.reset();
	head->robotPose.x = initialPixels/2;
	head->robotPose.y = initialPixels/2;
	head->robotPose.z = 0;
	head->robotPose.theta = 0;
	head->probability = 1;
	ancestryTree.head = head;
	
	// Initialize tree leaves
	ancestryTree.leaves.push_back(head);
}

// ######################################################################
void HawkSlammer::slam() {
	std::cout << "slam(): " << sensorDataMessages.size() << " messages in queue" << std::endl;
	
	// Select correct message
	SensorDataMessagePtr msg;
	if(mode == REALTIME_COMBINE) {
		// create a pseudo message by adding all attemptedMoves
		msg = sensorDataMessages.back();
		for(size_t i = 0; i < (sensorDataMessages.size()-1); i++) {
			// fill this in
			// is not simply x += x if non-zero rotation
		}
		sensorDataMessages.clear();
	}
	else if(mode == REALTIME_DROP) {
		// just take the last message received and drop the rest
		msg = sensorDataMessages.front();
		sensorDataMessages.clear();
	}
	else {
		// just take the last message received and save the rest
		msg = sensorDataMessages.front();
		sensorDataMessages.erase(sensorDataMessages.begin());
	}
	
	// Algorithm core (SLAM)
	updateParticles(msg);
    sendSlamDataMessage(); // we can send this immediately and then worry about cleaning up
    resampleParticles();
	std::cout << "active particles: " << 1+countDescendants(ancestryTree.head) << std::endl;
	if(pauseEachStep) {
		std::cout << "Please press enter to continue...";
		std::cin.get();
		std::cout << std::endl;
	}
    pruneParticles();
	
	// Debug items
	std::cout << "active particles: " << 1+countDescendants(ancestryTree.head) << std::endl;
	if(pauseEachStep) {
		std::cout << "Please press enter to continue...";
		std::cin.get();
		std::cout << std::endl;
	}
}

// ######################################################################
void HawkSlammer::updateParticles(SensorDataMessagePtr msg) {
	std::cout << "updateParticles()" << std::endl;
	
	// Prep
	ancestryTree.mostLikely.reset();
	double totalProbability = 0;
	
	// Apply new laser scan to every particle
	for(size_t i = 0; i < ancestryTree.leaves.size(); i++) {
		std::cout << "Working with leaf #" << i << ". ";
		// Add gaussian noise to attempted move to make our particle's move
		Pose move = msg->attemptedMove;
		move.x = (move.x + gaussian(robotXStdDev)) / mapResolution;
		move.y = (move.y + gaussian(robotYStdDev)) / mapResolution;
		move.theta += gaussian(robotThetaStdDev);
		
		// Add movement to absolute position while converting from robot frame of reference to world FoR
		double worldTheta = cleanAngle(ancestryTree.leaves.at(i)->robotPose.theta + move.theta);
		ancestryTree.leaves.at(i)->robotPose.theta = worldTheta; // theta = 0 along x axis
		ancestryTree.leaves.at(i)->robotPose.x += move.x*cos(worldTheta) + move.y*sin(worldTheta);
		ancestryTree.leaves.at(i)->robotPose.y += move.y*cos(worldTheta) - move.x*sin(worldTheta);
		
		// Apply each laser cast of the laser scan set
		int offset = msg->scannerData.size()/2;
		std::cout << "Applying laser scan... ";
		for(int j = 0; j < (int)msg->scannerData.size(); j++) {
			addLaserCast(ancestryTree.leaves.at(i), (j-offset)*msg->angularResolution, msg->scannerData.at(j) / mapResolution);
		}
		
		// Quick score (only look around the endpoint of the scan)
		
		// Compute probability of entire laser scan (unnormalized)
		double probabilityOfLaserScan = 0;
		std::cout << "Scoring laser scan... ";
		for(int j = 0; j < (int)msg->scannerData.size(); j++) {
			probabilityOfLaserScan += scoreLaserCast(ancestryTree.leaves.at(i), (j-offset)*msg->angularResolution, msg->scannerData.at(j) / mapResolution);
		}
		
		// Affect this particle's probability
		std::cout << probabilityOfLaserScan << std::endl;
		ancestryTree.leaves.at(i)->probability *= probabilityOfLaserScan; // DP-SLAM uses max(, th) here to provide a min-prob threshold
		totalProbability += ancestryTree.leaves.at(i)->probability;
		
		// Search for most likely particle
		if(!ancestryTree.mostLikely || ancestryTree.leaves.at(i)->probability > ancestryTree.mostLikely->probability) {
			ancestryTree.mostLikely = ancestryTree.leaves.at(i);
		}
	}
	
	// Normalize particles
	for(size_t i = 0; i < ancestryTree.leaves.size(); i++) {
		if(totalProbability == 0) ancestryTree.leaves.at(i)->probability = 1 / ancestryTree.leaves.size();
		else ancestryTree.leaves.at(i)->probability = ancestryTree.leaves.at(i)->probability / totalProbability;
	}
}

// ######################################################################
double HawkSlammer::randomDouble() {
	return double(rand()) / (double(RAND_MAX) + 1.0);
}

// ######################################################################
double HawkSlammer::gaussian(double stdDev) {
	double sum = 0;
	for(int i = 0; i < 12; i++) {
		sum += randomDouble() * 2 * stdDev - stdDev;
	}
	return sum / 2;
}

// ######################################################################
double HawkSlammer::cleanAngle(double angle) {
	while(angle > M_PI) angle -= 2*M_PI;
	while(angle < -M_PI) angle += 2*M_PI;
	return angle;
}

// ######################################################################
void HawkSlammer::addLaserCast(boost::shared_ptr<Particle> treeLeaf, double scanAngle, double scanDistance) {
	if(scanDistance == 0) return;
	
	double startx = treeLeaf->robotPose.x;
	double starty = treeLeaf->robotPose.y;
	double theta = cleanAngle(treeLeaf->robotPose.theta + scanAngle);

	double overflow, slope; // Used for actually tracing the line
	int x, y, incX, incY, endx, endy;
	int xedge, yedge; // Used in computing the midpoint. Recompensates for which edge of the square the line entered from
	double dx, dy;
	double distance = scanDistance, error;
	
	// Precompute these for speed
	double secant = 1.0/fabs(cos(theta));
	double cosecant = 1.0/fabs(sin(theta));
	
	// Mark the final endpoint of the line trace, so that we know when to stop.
	// We keep the endpoint as both float and int.
	dx = (startx + (cos(theta) * distance));
	dy = (starty + (sin(theta) * distance));
	endx = (int) (dx);
	endy = (int) (dy);

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
		y = (int)(starty);
		overflow = starty - y;
		
		// We always use overflow as a decreasing number- therefore positive y motion needs to 
		// adjust the overflow value accordingly.
		if(incY == 1) overflow = 1.0 - overflow;
		
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
		for(x = (int)(startx) + incX; x != endx; x = x + incX) {
			overflow = overflow - slope;

			// Compute the distance travelled in this square
			if(overflow < 0.0) distance = (overflow+slope)*cosecant;
			else distance = fabs(slope)*cosecant;
			
			// Update every grid square we cross as empty...
			addObservation(treeLeaf, x, y, distance, 0);

			// ...including the overlap in the minor direction
			if(overflow < 0) {
				y = y + incY;
				distance = -overflow*cosecant;
				overflow = overflow + 1.0;
				addObservation(treeLeaf, x, y, distance, 0);
			}
		}

		// Update the last grid square seen as having a hit.
		// make sure the laser just didn't go the full distance and not hit something...
		//if(addEnd) {
			if(incX < 0) distance = fabs((x+1) - dx)*secant;
			else distance = fabs(dx - x)*secant;
			addObservation(treeLeaf, endx, endy, distance, 1);
		//}
	}
	// This is the same as the previous block of code, with x and y reversed.
	else {
		x = (int)(startx);
		overflow = startx - x;
		if(incX == 1) overflow = 1.0 - overflow;
		slope = 1.0/fabs(tan(theta));

		error = fabs(((int)(starty)+incY+yedge)-starty);
		overflow = overflow - (error*slope);
		if(overflow < 0.0) {
			x = x + incX;
			overflow = overflow + 1.0;
		}

		for(y = (int)(starty) + incY; y != endy; y = y + incY) {
			overflow = overflow - slope;
			if (overflow < 0) distance = (overflow+slope)*secant;
			else distance = fabs(slope)*secant;

			addObservation(treeLeaf, x, y, distance, 0);

			if(overflow < 0.0) {
				x = x + incX;
				distance = -overflow*secant;
				overflow = overflow + 1.0;
				addObservation(treeLeaf, x, y, distance, 0);
			}
		}

		//if(addEnd) {
			if(incY < 0) distance = fabs(((y+1) - dy)/sin(theta));
			else distance = fabs((dy - y)/sin(theta));
			
			addObservation(treeLeaf, endx, endy, distance, 1);
		//}
	}
}

// ######################################################################
void HawkSlammer::addObservation(boost::shared_ptr<Particle> treeLeaf, int x, int y, double distanceTraveled, int hit) {
	// Make new observation
	boost::shared_ptr<Observation> o(new Observation());
	o->source = treeLeaf;
	o->x = x;
	o->y = y;
	
	// Add empty map if needed
	occupancyMapPix_t treeMap;
	if(!occupancyGrid.getVal(x, y)) {
		treeMap = occupancyMapPix_t(
			new std::map<boost::shared_ptr<Particle>, boost::shared_ptr<Observation> >);
		occupancyGrid.setVal(x, y, treeMap);
	}
	else treeMap = occupancyGrid.getVal(x, y);
	
	// Search for an ancestor observation
	boost::shared_ptr<Particle> ancestor = treeLeaf->parent;
	bool ancestorObservation = false;
	std::map< boost::shared_ptr<Particle>, boost::shared_ptr<Observation> >::const_iterator it;
	while(!ancestorObservation && ancestor) {
		it = treeMap->find(ancestor);
		if(it != treeMap->end()) ancestorObservation = true;
		else ancestor = ancestor->parent;
	}
	
	// Set new observation's parameters accordingly
	if(ancestorObservation) {
		o->laserTravel = it->second->laserTravel + distanceTraveled;
		o->laserTerminations = it->second->laserTerminations + hit;
		o->ancestor = it->second;
	}
	else {
		o->laserTravel = distanceTraveled;
		o->laserTerminations = hit;
		o->ancestor.reset();
	}
	
	// Add new observation to treeLeaf's observation list and the occupancyGrid
	treeLeaf->observations.push_back(o);
	treeMap->insert( std::pair<boost::shared_ptr<Particle>, boost::shared_ptr<Observation> >(treeLeaf, o) );
}

// ######################################################################
double HawkSlammer::scoreLaserCast(boost::shared_ptr<Particle> treeLeaf, double scanAngle, double scanDistance) {
	if(scanDistance == 0) return scoreObservation(treeLeaf, treeLeaf->robotPose.x, treeLeaf->robotPose.y, scanDistance);
	
	double startx = treeLeaf->robotPose.x;
	double starty = treeLeaf->robotPose.y;
	double theta = cleanAngle(treeLeaf->robotPose.theta + scanAngle);
	double MeasuredDist = scanDistance;
	
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
	// totalProb is the total probability that the laser scan could travel this far through the occupancyGrid
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
	/*if (culling)
	distance = MeasuredDist+culling;
	else
	distance = MIN(MeasuredDist+20.0, MAX_SENSE_RANGE);*/
	distance = MeasuredDist;

	// The endpoint of the scan, in both float and int.
	dx = (startx + (cos(theta) * distance));
	dy = (starty + (sin(theta) * distance));
	endx = (int) (dx);
	endy = (int) (dy);

	// Decide which x and y directions the line is travelling.
	if(startx > dx) {
		incX = -1;
		xblock = -startx;
	}
	else {
		incX = 1;
		xblock = 1.0-startx;
	}

	if(starty > dy) {
		incY = -1;
		yblock = -starty;
	}
	else {
		incY = 1;
		yblock = 1.0-starty;
	}

	// Two copies of the same basic code, swapping the roles of x and y, depending on which one is the primary 
	// direction of motion in the line trace.
	if(fabs(startx - dx) > fabs(starty - dy)) {
		y = (int) (starty);

		// The given starting point is non-integer. The line therefore starts at some point partially set in to the starting
		// square. Overflow starts at this off-center amount, in order to make steps in the y direction at the right places.
		overflow = starty - y;
		// Code is simpler if overflow is always decreasing towards zero. Note that slope is forced to be postive
		if(incY == 1) overflow = 1.0 - overflow;

		slope = fabs(tan(theta));
		if(slope > 1.0) slope = fabs((starty - dy) / (startx - dx));

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
		else overflow = overflow - dy;

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

		for(x = (int) (startx) + incX; x != endx; x = x + incX) {
			// Update our two running counts.
			xMotion = xMotion + secant;
			overflow = overflow - slope;

			// Establish the distance travelled by the laser through this square. Note that this amount is
			// less than normal if the slope has overflowed, implying that the y-axis has been crossed.
			if (overflow < 0.0) distance = (overflow+slope)*cosecant;
			else distance = standardDist;

			// Compute the probability of the laser stopping in the square, given the particle's unique map.
			// Keep in mind that the probability of even getting this far in the trace is likely less than 1.
			prob = totalProb * scoreObservation(treeLeaf, x, y, distance);
			if(prob > 0) {
				// If the scan had actually been stopped by an object in the map at this square,
				// how much error would there be in the laser? Determine which axis will be crossed 
				// next, and compute from there. (This value is actually kept as a running total now).
				if(overflow < 0.0) error = fabs(yMotion);
				else error = fabs(xMotion);

				// Increase the probability of the scan by the probability of stopping here, multiplied by the 
				// probability that a scan which stopped here could produce the error observed.
				// If the error is too large, the net effect on probability of the scan is essentially zero.
				// We can save some time by not computing this exponential.
				if(error < 20.0) eval = eval + (prob * exp(-(error*error)/(2*scannerVariance)));

				// Correspondingly decrease the probability that laser has continued.
				totalProb = totalProb - prob;
			}

			// If the overflow has dipped below zero, then the trace has crossed the y-axis, and we need to compute
			// everything for a single step in the y direction. 
			if(overflow < 0.0) {
				y += incY;
				yMotion = yMotion + cosecant;

				distance = -overflow*cosecant;
				overflow = overflow + 1.0;

				prob = totalProb * scoreObservation(treeLeaf, x, y, distance);
				if(prob > 0) {
					// There is no question about which axis will be the next crossed, since we just crossed the y-axis, 
					// and x motion is dominant
					error = fabs(xMotion);
					if(error < 20.0) eval = eval + (prob * exp(-(error*error)/(2*scannerVariance)));
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
		if(incX == 1) overflow = 1.0 - overflow;
		slope = 1.0/fabs(tan(theta));

		// (See corresponding comments in the previous half of this function)
		dy = fabs((int)(starty)+yblock);
		dx = fabs(dy/tan(theta));
		if(overflow - dx < 0) {
			x = x + incX;
			overflow = overflow - dx + 1.0;
		}
		else overflow = overflow - dx;

		standardDist = slope*secant;
		xMotion = -fabs(fabs((x+xblock) * secant) - MeasuredDist);
		yMotion = -fabs(fabs(( ((int) (starty)) +yblock) * cosecant) - MeasuredDist);

		for(y = (int) (starty) + incY; y != endy; y = y + incY) {
			yMotion = yMotion + cosecant;
			overflow = overflow - slope;

			if(overflow < 0.0) distance = (overflow+slope)*secant;
			else distance = standardDist;

			prob = totalProb * scoreObservation(treeLeaf, x, y, distance);
			if (prob > 0) {
				if(overflow < 0.0) error = fabs(xMotion);
				else error = fabs(yMotion);
				if(error < 20.0) eval = eval + (prob * exp(-(error*error)/(2*scannerVariance)));
			}
			totalProb = totalProb - prob;

			if(overflow < 0.0) {
				x += incX;
				xMotion = xMotion + secant;

				distance = -overflow*secant;
				overflow = overflow + 1.0;

				prob = totalProb * scoreObservation(treeLeaf, x, y, distance);
				if(prob > 0) {
					error = fabs(yMotion);
					if(error < 20.0) eval = eval + (prob * exp(-(error*error)/(2*scannerVariance)));
				}
				totalProb = totalProb - prob;
			}

		}
	}
	
	// If the laser reported a range beyond the maximum range allowed, any left-over probability that
	// the laser has not yet been stopped all has a probability of 1 to get the measured reading. We
	// therefore just add this remaining probability to the evaluation.
	if(MeasuredDist >= maxLaserDistance) return (eval + totalProb);

	// Otherwise, we know that the total probability of the laser being stopped at some point during 
	// the scan is 1. Normalize the evaluation to enforce this. 
	if(totalProb == 1) return 0;
	return (eval / (1.0 - totalProb));
}

// ######################################################################
double HawkSlammer::scoreObservation(boost::shared_ptr<Particle> treeLeaf, int x, int y, double distanceTraveled) {
	// This is easy, no particle has observed this, return unkown probability
	if(occupancyGrid.getVal(x, y)) {
		std::map< boost::shared_ptr<Particle>, boost::shared_ptr<Observation> >::const_iterator it = occupancyGrid.getVal(x, y)->find(treeLeaf);
		if(it == occupancyGrid.getVal(x, y)->end())
			return (1.0 - exp(-1.0/(8000 * mapResolution) * distanceTraveled));
		else return (1.0 - exp(-(it->second->laserTerminations / it->second->laserTravel) * distanceTraveled));
	}
	else return (1.0 - exp(-1.0/(8000 * mapResolution) * distanceTraveled));
}

// ######################################################################
void HawkSlammer::sendSlamDataMessage() {
	std::cout << "sendSlamDataMessage()" << std::endl;
    
	// Create Message
	HawkMessages::SlamDataMessagePtr msg = new HawkMessages::SlamDataMessage;
    msg->hawkPose = ancestryTree.mostLikely->robotPose;
    Image< PixRGB<byte> > mapImage = makeMapImage(ancestryTree.mostLikely);
    if(ancestryTree.leaves.size() >= 3) {
		Image< PixRGB<byte> > topImage = concatX(mapImage, makeMapImage(ancestryTree.leaves.at(0)));
		Image< PixRGB<byte> > bottomImage = concatX(makeMapImage(ancestryTree.leaves.at(1)), makeMapImage(ancestryTree.leaves.at(2)));
		mapImage = concatY(topImage, bottomImage);
    }
    
    // Debug print to screen
    std::cout << "\trobotPose: (" << msg->hawkPose.x << ", " << msg->hawkPose.y << ", " << msg->hawkPose.z << ", " << msg->hawkPose.theta << ")" << std::endl;
    window.drawImage(mapImage, 0, 0, true);
    
    // Send Message
	publish("SlamDataMessage", msg);
}

// ######################################################################
void HawkSlammer::resampleParticles() {
	std::cout << "resampleParticles()" << std::endl;
	
    // Calculate a Cumulative Distribution Function for our particle weights
    std::vector<float> CDF;
    CDF.resize(ancestryTree.leaves.size());
    CDF.at(0) = ancestryTree.leaves.at(0)->probability;
    for(int i = 1; i < (int)CDF.size(); i++) {
        CDF.at(i) = CDF.at(i-1) + ancestryTree.leaves.at(i)->probability;
    }
    
    // Roulette wheel through particles
    std::vector<boost::shared_ptr<Particle> > lonelyLeaves = ancestryTree.leaves;
    std::vector<boost::shared_ptr<Particle> > newLeaves;
    int i = 0;
    float newProb = 1.0/float(numberOfParticles);
    float u = randomDouble()* newProb;
    for(int j = 0; j < numberOfParticles; j++) {
    	// Keep the wheel moving
        while(i < (int)CDF.size() && u > CDF.at(i)) i++;
        u += newProb;
        
        // Make new particle
        boost::shared_ptr<Particle> p(new Particle());
        p->parent = ancestryTree.leaves.at(i);
        p->robotPose = ancestryTree.leaves.at(i)->robotPose;
        p->probability = newProb;
        
        // Tell everyone!
        ancestryTree.leaves.at(i)->children.push_back(p);
        newLeaves.push_back(p);
        
        // Remove from lonelyLeaves vector
		std::vector<boost::shared_ptr<Particle> >::iterator it;
		it = std::find(lonelyLeaves.begin(), lonelyLeaves.end(), ancestryTree.leaves.at(i));
		if(it != lonelyLeaves.end()) lonelyLeaves.erase(it);
    }
    
    // Set new particles as tree leaves
    ancestryTree.leaves = newLeaves;
    
    // Delete Un-resampled leaves
	std::cout << "before resampling, active particles: " << 1+countDescendants(ancestryTree.head) << std::endl;
	std::cout << "we have " << lonelyLeaves.size() << " lonely leaves to be deleted" << std::endl;
    for(size_t i = 0; i < lonelyLeaves.size(); i++) {
		std::vector<boost::shared_ptr<Particle> >::iterator it;
		it = find(lonelyLeaves.at(i)->parent->children.begin(), lonelyLeaves.at(i)->parent->children.end(), lonelyLeaves.at(i));
    	lonelyLeaves.at(i)->parent->children.erase(it);
    	// with no references left, particle should now get deleted by boost
    }
	std::cout << "after resampling, active particles: " << 1+countDescendants(ancestryTree.head) << std::endl;
}

// ######################################################################
void HawkSlammer::pruneParticles() {
	std::cout << "pruneParticles()" << std::endl;
	
	// Start a recursive call through the tree
	collapseParticle(ancestryTree.head);
}

// ######################################################################
void HawkSlammer::collapseParticle(boost::shared_ptr<Particle> particle) {
	// Check to see if we have an only child
	if(particle->children.size() == 1) {
		std::cout << "fouund a particle to squash!" << std::endl;
		// Particle's only child
		boost::shared_ptr<Particle> child = particle->children.at(0);
		
		std::cout << "Child has " << child->observations.size() << " observations" << std::endl;
		// Delete particle's observations that that have been replaced by the child
		for(size_t j = 0; j < child->observations.size(); j++) {
			// Does child have observation in the same place as particle?
			if(child->observations.at(j)->ancestor && child->observations.at(j)->ancestor->source == particle) {
				// He does! So copy over the particle's observation ancestor
				child->observations.at(j)->ancestor = child->observations.at(j)->ancestor->ancestor;
				
				// And remove reference of particle's observation in occupancyGrid
				std::cout << "Erasing observation with " << (*occupancyGrid.getVal(child->observations.at(j)->x, child->observations.at(j)->y))[particle].use_count() << " references" << std::endl;
				occupancyGrid.getVal(child->observations.at(j)->x, child->observations.at(j)->y)->erase(particle);
			}
		}
		std::cout << "Child now has " << child->observations.size() << " observations" << std::endl;
		std::cin.get();

		// Particle's remaining observations should become the child's
		for(int j = 0; j < (int)particle->observations.size(); j++) {
			particle->observations.at(j)->source = child;
			child->observations.push_back(particle->observations.at(j));
		}
		
		// Update child's parent reference
		child->parent = particle->parent;
		
		// Update parent's child reference
		if(!particle->parent) {
			// This is the head of the tree, replace the head
			ancestryTree.head = child;
		}
		else {
			// This is a normal node, replace the pointer in the children vector
			std::vector<boost::shared_ptr<Particle> >::iterator it;
			it = find(particle->parent->children.begin(), particle->parent->children.end(), particle);
			*it = child;
		}
	
		// Delete particle (automatic with boost)
		particle->parent.reset();
		particle->children.clear();
		particle->observations.clear();

		// Now re-run the algorithm on the child
		collapseParticle(child);
	}
	else {
		// Run this algorithm recursively for each child
		for(size_t i = 0; i < particle->children.size(); i++) {
			collapseParticle(particle->children.at(i));
		}
	}
}

// ######################################################################
int HawkSlammer::countDescendants(boost::shared_ptr<Particle> particle) {
	int particles = particle->children.size();
	
	for(size_t i = 0; i < particle->children.size(); i++) {
		particles += countDescendants(particle->children.at(i));
	}
	
	return particles;
}

// ######################################################################
Image< PixRGB<byte> > HawkSlammer::makeMapImage(boost::shared_ptr<Particle> primaryParticle) {
	Image< PixRGB<byte> > mapImage(occupancyGrid.getWidth(), occupancyGrid.getHeight(), NO_INIT);
    PixRGB<byte> hitDot, nohitDot, notscannedDot;
    hitDot.set(0, 0, 0); //black
    nohitDot.set(195, 195, 195); //light grey
    notscannedDot.set(255, 255, 255); //white
	
	// Make the image
	for(int i = 0; i < mapImage.getWidth(); i++) {
		for(int j = 0; j < mapImage.getHeight(); j++) {
			// Find an observation from the particle or his ancestors
			boost::shared_ptr<Particle> particle = primaryParticle;
		 	boost::shared_ptr<Observation> observation;
			std::map<boost::shared_ptr<Particle>, boost::shared_ptr<Observation> >::iterator it;
			while(!observation && particle) {
				if(occupancyGrid.getVal(i, j)) {
					it = occupancyGrid.getVal(i, j)->find(particle);
					if(it != occupancyGrid.getVal(i, j)->end()) observation = it->second;
					else particle = particle->parent;
				}
				else particle = particle->parent;
			}
			
			// Color map accordingly
			if(observation) {
				if(observation->laserTerminations / observation->laserTravel > 0.5)
					mapImage.setVal(i, j, hitDot);
				else mapImage.setVal(i, j, nohitDot);
			}
			else mapImage.setVal(i, j, notscannedDot);
		}
	}
	
	return mapImage;
}

// ######################################################################
Image< PixRGB<byte> > HawkSlammer::makeScannerImage(SensorDataMessagePtr msg) {
	double angularResolution = msg->angularResolution;
	LongSeq scannerData = msg->scannerData;
	Image< PixRGB<byte> > scannerImage(2*HALF_WINDOW_SIZE,2*HALF_WINDOW_SIZE,ZEROS);
	
	// Plot scanner points
	long min = maxLaserDistance, max = 0;
	int offset = scannerData.size()/2;
	for(size_t i = 0; i < scannerData.size(); i++) {
		if(scannerData.at(i) > max) max = scannerData.at(i);
		if(scannerData.at(i) < min) min = scannerData.at(i);
		// Calculate distance
		float distance = scannerData.at(i); 
		float angle = (i-offset)*angularResolution;
		distance = distance/maxLaserDistance*HALF_WINDOW_SIZE; //((distance - min)/(max-min))*256;
		if (distance < 0) distance = 1.0;
		
		// Draw point
		Point2D<int> pt;
		pt.i = HALF_WINDOW_SIZE - int(distance*sin(angle));
		pt.j = HALF_WINDOW_SIZE - int(distance*cos(angle));
		drawCircle(scannerImage, pt, 1, PixRGB<byte>(255,0,0));
		
		// Draw range lines
		if(i == 0 || i == (scannerData.size() - 1)) {
			pt.i = HALF_WINDOW_SIZE - int(HALF_WINDOW_SIZE*sin(angle));
			pt.j = HALF_WINDOW_SIZE - int(HALF_WINDOW_SIZE*cos(angle));
			drawLine(scannerImage, Point2D<int>(HALF_WINDOW_SIZE,HALF_WINDOW_SIZE),pt,PixRGB<byte>(0,0,255));
		}

		/*if((i >= (-50 + 141)) && (i <= (49 + 141)))
			drawLine(scannerImage, Point2D<int>(HALF_WINDOW_SIZE,HALF_WINDOW_SIZE),pt,PixRGB<byte>(0,0,255));
		else
			drawLine(scannerImage, Point2D<int>(HALF_WINDOW_SIZE,HALF_WINDOW_SIZE),pt,PixRGB<byte>(0,255,0));*/
	}
	std::cout << "min, max: " << min << ", " << max << std::endl;
	
	return scannerImage;
}

// ######################################################################
int main (int argc, char* argv[]) {
	std::cout << "HawkSlammer: starting..." << std::endl;
	
	HawkSlammer agent("HawkSlammer", argc, argv);
	agent.start();
	
	std::cout << "HawkSlammer: all done!" << std::endl;
}
