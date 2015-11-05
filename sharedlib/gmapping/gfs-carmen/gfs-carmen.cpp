/*****************************************************************
 *
 * This file is part of the GMAPPING project
 *
 * GMAPPING Copyright (c) 2004 Giorgio Grisetti,
 * Cyrill Stachniss, and Wolfram Burgard
 *
 * This software is licensed under the "Creative Commons
 * License (Attribution-NonCommercial-ShareAlike 2.0)"
 * and is copyrighted by Giorgio Grisetti, Cyrill Stachniss,
 * and Wolfram Burgard.
 *
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/2.0/
 *
 * GMAPPING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 *
 *****************************************************************/


#include <utils/commandline.h>
#include <carmenwrapper/carmenwrapper.h>
#include <gridfastslam/gridslamprocessor.h>
#include <utils/orientedboundingbox.h>
#include <configfile/configfile.h>

#define DEBUG cout << __PRETTY_FUNCTION__

/*
Example file for interfacing carmen, and gfs.

if you want to look for a specific topic search for one of the following keywords in the file comments

KEYWORDS:
	CREATION
	INITIALIZATION
	SENSOR MAP
	BEST PARTICLE INDEX
	PARTICLE VECTOR
	PARTICLE TRAJECTORIES
	BEST MAP
 	BOUNDING BOX
*/

using namespace GMapping;
using namespace std;

void
save_map(ScanMatcherMap smap)
{
	static int map_number = 0;
	FILE *map_file;
	char map_file_name[1000];

	sprintf(map_file_name, "map_file%d.pnm", map_number);
	map_file = fopen(map_file_name, "w");

	// PNM file header
	fprintf(map_file, "P3\n#PNM criado por Alberto\n");
	fprintf(map_file, "%d %d\n255\n", smap.getMapSizeX()/2, smap.getMapSizeY()/2);

	for (int y = 0; y < smap.getMapSizeY()/2; y++)
	{
		for (int x = 0; x < smap.getMapSizeX()/2; x++)
		{
			double v = smap.cell(x,y);
			if (v >= 0)
			{
				int grayValue=255-(int)(255.*v);
				fprintf(map_file, "%d\n%d\n%d\n", grayValue, grayValue, grayValue);
			}
			else
			{
				fprintf(map_file, "%d\n%d\n%d\n", 120, 120, 255);
			}
		}
	}
	fclose(map_file);
	map_number++;
}


int main(int argc, char ** argv){

	std::string outfilename="";
	double xmin=-10.;
	double ymin=-10.;
	double xmax=10.;
	double ymax=10.;
	double delta=0.1;

	//scan matching parameters
	double sigma=0.05;
	double maxrange=80.;
	double maxUrange=80.;
	double regscore=1e4;
	double lstep=.05;
	double astep=.05;
	int kernelSize=1;
	int iterations=5;
	double critscore=0.;
	double maxMove=1.;
	double lsigma=.075;
	double ogain=3;
	int lskip=0;

	//motion model parameters
	double srr = 0.01, srt = 0.01, str = 0.01, stt = 0.01;
	//particle parameters
	int particles = 10;


	//gfs parameters
	double angularUpdate = 0.2;
	double linearUpdate = 0.2;
	double resampleThreshold = 0.5;
	bool generateMap = true;
	float *myFloatMap = NULL;

	int oldX = 0;
	int oldY = 0;

	std::string configfilename = "";

	CMD_PARSE_BEGIN_SILENT(1,argc);
		parseStringSilent("-cfg",configfilename);
	CMD_PARSE_END_SILENT;

	if(configfilename.length() > 0){
		ConfigFile cfg(configfilename);
		outfilename = (std::string) cfg.value("gfs","outfilename",outfilename);
		xmin = cfg.value("gfs","xmin", xmin);
		xmax = cfg.value("gfs","xmax",xmax);
		ymin = cfg.value("gfs","ymin",ymin);
		ymax = cfg.value("gfs","ymax",ymax);
		delta =  cfg.value("gfs","delta",delta);
		maxrange = cfg.value("gfs","maxrange",maxrange);
		maxUrange = cfg.value("gfs","maxUrange",maxUrange);
		regscore = cfg.value("gfs","regscore",regscore);
		critscore = cfg.value("gfs","critscore",critscore);
		kernelSize = cfg.value("gfs","kernelSize",kernelSize);
		sigma = cfg.value("gfs","sigma",sigma);
		iterations = cfg.value("gfs","iterations",iterations);
		lstep = cfg.value("gfs","lstep",lstep);
		astep = cfg.value("gfs","astep",astep);
		maxMove = cfg.value("gfs","maxMove",maxMove);
		srr = cfg.value("gfs","srr", srr);
		srt = cfg.value("gfs","srt", srt);
		str = cfg.value("gfs","str", str);
		stt = cfg.value("gfs","stt", stt);
		particles = cfg.value("gfs","particles",particles);
		angularUpdate = cfg.value("gfs","angularUpdate", angularUpdate);
		linearUpdate = cfg.value("gfs","linearUpdate", linearUpdate);
		lsigma = cfg.value("gfs","lsigma", lsigma);
		ogain = cfg.value("gfs","lobsGain", ogain);
		lskip = (int)cfg.value("gfs","lskip", lskip);
		//	  randseed = cfg.value("gfs","randseed", randseed);
		resampleThreshold = cfg.value("gfs","resampleThreshold", resampleThreshold);
		generateMap = cfg.value("gfs","generateMap", generateMap);
	}


	CMD_PARSE_BEGIN(1,argc);
		parseString("-cfg",configfilename);
		parseString("-outfilename",outfilename);
		parseDouble("-xmin",xmin);
		parseDouble("-xmax",xmax);
		parseDouble("-ymin",ymin);
		parseDouble("-ymax",ymax);
		parseDouble("-delta",delta);
		parseDouble("-maxrange",maxrange);
		parseDouble("-maxUrange",maxUrange);
		parseDouble("-regscore",regscore);
		parseDouble("-critscore",critscore);
		parseInt("-kernelSize",kernelSize);
		parseDouble("-sigma",sigma);
		parseInt("-iterations",iterations);
		parseDouble("-lstep",lstep);
		parseDouble("-astep",astep);
		parseDouble("-maxMove",maxMove);
		parseDouble("-srr", srr);
		parseDouble("-srt", srt);
		parseDouble("-str", str);
		parseDouble("-stt", stt);
		parseInt("-particles",particles);
		parseDouble("-angularUpdate", angularUpdate);
		parseDouble("-linearUpdate", linearUpdate);
		parseDouble("-lsigma", lsigma);
		parseDouble("-lobsGain", ogain);
		parseInt("-lskip", lskip);
		parseDouble("-resampleThreshold", resampleThreshold);
		parseFlag("-generateMap", generateMap);
	CMD_PARSE_END;

	cerr << "Parameter parsed, connecting to Carmen!";

	CarmenWrapper::initializeIPC(argv[0]);
	CarmenWrapper::init_localize_filter(argc, argv, particles);
	CarmenWrapper::start(argv[0]);

	while (!CarmenWrapper::sensorMapComputed()){
		usleep(500000);
		cerr << "." << flush;
	}

	//CREATION

	GridSlamProcessor* processor = new GridSlamProcessor;

	//SENSOR MAP
	//loads from the carmen wrapper the laser and robot settings
	SensorMap sensorMap=CarmenWrapper::sensorMap();
	cerr << "Connected " << endl;
	processor->setSensorMap(sensorMap);

	//set the command line parameters
	processor->setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, lsigma, ogain, lskip);
	processor->setMotionModelParameters(srr, srt, str, stt);
	processor->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);
	processor->setgenerateMap(generateMap);

	OrientedPoint initialPose(xmin+xmax/2, ymin+ymax/2, 0);
	//OrientedPoint initialPose = CarmenWrapper::getInitialPose();


	//INITIALIZATION
	processor->init(particles, xmin, ymin, xmax, ymax, delta, initialPose);
	if (outfilename.length()>0)
		processor->outputStream().open(outfilename.c_str());

	bool running = true;

	GridSlamProcessor* ap, *copy = processor->clone();
	ap = processor; processor = copy; copy = ap;

	//this is the CORE LOOP;
	unsigned int best_idx = 0;
	RangeReading rr(0,0);
	while (running)
	{
		while (CarmenWrapper::getReading(rr))
		{
			bool processed = processor->processScan(rr);

			//this returns true when the algorithm effectively processes (the traveled path since the last processing is over a given threshold)
			if (processed){
				cerr << "PROCESSED" << endl;
				//for searching for the BEST PARTICLE INDEX
				best_idx = processor->getBestParticleIndex();

				//if you want to access to the PARTICLE VECTOR
				const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
				//remember to use a const reference, otherwise it copys the whole particles and maps

				//this is for recovering the tree of PARTICLE TRAJECTORIES (obtaining the ancestor of each particle)
				cerr << "Particle reproduction story begin" << endl;
				for (unsigned int i = 0; i < particles.size(); i++){
					cerr << particles[i].previousIndex << "->"  << i << " ";
				}
				cerr << "Particle reproduction story end" << endl;

				//then if you want to access the BEST MAP,
				//of course by copying it in a plain structure
				Map<double, DoubleArray2D, false>* mymap = processor->getParticles()[best_idx].map.toDoubleMap();


				int y, x;

				if (myFloatMap == NULL){
					myFloatMap = (float *) calloc (mymap->getMapSizeX() * mymap->getMapSizeY(), sizeof(float));

					oldX = mymap->getMapSizeX();
					oldY = mymap->getMapSizeY();
				}

				if (oldX != mymap->getMapSizeX()){

					delete(myFloatMap);

					myFloatMap = (float *) calloc (mymap->getMapSizeX() * mymap->getMapSizeY(), sizeof(float));

					oldX = mymap->getMapSizeX();
					oldY = mymap->getMapSizeY();

				}else{

					if(oldY != mymap->getMapSizeY()){
						delete(myFloatMap);

						myFloatMap = (float *) calloc (mymap->getMapSizeX() * mymap->getMapSizeY(), sizeof(float));

						oldY = mymap->getMapSizeY();
					}
				}

				for(x = 0; x < mymap->getMapSizeX(); x++)
				{
					for(y = 0; y < mymap->getMapSizeY(); y++)
					{
						myFloatMap[x * mymap->getMapSizeY() + y] = mymap->cell(x, y);
					}
				}

				CarmenWrapper::gfsMapToCarmenMap(myFloatMap, mymap->getMapSizeX(), mymap->getMapSizeY(), mymap->getMapResolution());

				CarmenWrapper::send_particles(particles, processor->m_matcher.laserBeams());

				//at this point mymap is yours. Can do what you want.
				//save_map(myMap);

				double best_weight = particles[best_idx].weightSum;
				cerr << "Best Particle is " << best_idx << " with weight " << best_weight << endl;


				cerr << __PRETTY_FUNCTION__  << "CLONING... " << endl;
				GridSlamProcessor* newProcessor = processor->clone();
				cerr << "DONE" << endl;
				cerr << __PRETTY_FUNCTION__  << "DELETING... " << endl;
				delete processor;
				cerr << "DONE" << endl;
				processor = newProcessor;

			}
		}
	}
	ScanMatcherMap mymap = processor->getParticles()[best_idx].map; //.toDoubleMap();
	//save_map(mymap);
	return 0;
}

