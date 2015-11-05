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

#include <iostream>
#include <deque>
#include <pthread.h>
#include <semaphore.h>
#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/slam_messages.h>
#include <gridfastslam/gridslamprocessor.h>
#include <utils/commandline.h>
#include <utils/orientedboundingbox.h>
#include <configfile/configfile.h>
#include <log/carmenconfiguration.h>
#include <log/sensorstream.h>
#include <log/sensorlog.h>
#include <sensor/sensor_base/sensor.h>
#include <sensor/sensor_range/rangesensor.h>
#include <sensor/sensor_range/rangereading.h>

#define DEBUG cout << __PRETTY_FUNCTION__

using namespace GMapping;
using namespace std;

//static vars for the carmenwrapper
GMapping::SensorMap m_sensorMap;
deque<RangeReading> m_rangeDeque;
pthread_mutex_t m_mutex;
sem_t m_dequeSem;
pthread_mutex_t m_lock;
pthread_t m_readingThread;
RangeSensor* m_frontLaser=0;
RangeSensor* m_rearLaser=0;
bool m_threadRunning=false;
OrientedPoint m_truepos;
OrientedPoint m_initialPose(-1, -1, -1);
bool stopped=true;

carmen_robot_laser_message front_laser;

void lock(){
	pthread_mutex_lock(&m_lock);
}

void unlock(){
	pthread_mutex_unlock(&m_lock);
}

void * m_reading_function(void*){
	while (true) {
		lock();
		IPC_listen(100);
		unlock();
		usleep(20000);
	}
	return 0;
}

void shutdown_module(int sig){
  if(sig == SIGINT) {
      carmen_ipc_disconnect();

      fprintf(stderr, "\nDisconnecting (shutdown_module(%d) called).\n",sig);
      exit(0);
  }
}

void addReading(RangeReading& reading){
	pthread_mutex_lock(&m_mutex);
	m_rangeDeque.push_back(reading);
	pthread_mutex_unlock(&m_mutex);
	sem_post(&m_dequeSem);
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
}

RangeReading carmen2reading(const carmen_robot_laser_message& msg){
	//either front laser or rear laser
	double dth=msg.laser_pose.theta-msg.robot_pose.theta;
	dth=atan2(sin(dth), cos(dth));

	if (msg.laser_pose.theta==msg.robot_pose.theta && !m_frontLaser){
		double res=0;
		res = msg.config.angular_resolution;
		assert(res>0);
		string sensorName="FLASER";
		OrientedPoint rpose(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta);
		OrientedPoint lpose(msg.laser_pose.x, msg.laser_pose.y, msg.laser_pose.theta);
		OrientedPoint dp=absoluteDifference(lpose, rpose);
		m_frontLaser=new RangeSensor(sensorName,msg.num_readings, res, OrientedPoint(0,0,msg.laser_pose.theta-msg.robot_pose.theta), 0,
					      msg.config.maximum_range);
		m_frontLaser->updateBeamsLookup();
		m_sensorMap.insert(make_pair(sensorName, m_frontLaser));

//		cout << __PRETTY_FUNCTION__
//		     << ": " << sensorName <<" configured."
//		     << " Readings " << m_frontLaser->beams().size()
//		     << " Resolution " << res << endl;
	}
	if (msg.laser_pose.theta!=msg.robot_pose.theta && !m_rearLaser){
		double res=0;
		res = msg.config.angular_resolution;
		assert(res>0);
		OrientedPoint rpose(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta);
		OrientedPoint lpose(msg.laser_pose.x, msg.laser_pose.y, msg.laser_pose.theta);
		OrientedPoint dp=absoluteDifference(lpose, rpose);
		string sensorName="RLASER";
		m_rearLaser=new RangeSensor(sensorName,msg.num_readings, res, OrientedPoint(0,0,msg.laser_pose.theta-msg.robot_pose.theta), 0,
					      msg.config.maximum_range);
		m_rearLaser->updateBeamsLookup();
		m_sensorMap.insert(make_pair(sensorName, m_rearLaser));

//		cout << __PRETTY_FUNCTION__
//		     << ": " << sensorName <<" configured."
//		     << " Readings " << m_rearLaser->beams().size()
//		     << " Resolution " << res << endl;
	}

	const RangeSensor * rs=(msg.laser_pose.theta==msg.robot_pose.theta)?m_frontLaser:m_rearLaser;
	RangeReading reading(rs, msg.timestamp);
	reading.resize(rs->beams().size());
	for (unsigned int i=0; i< (unsigned int)msg.num_readings; i++){
		reading[i]=(double)msg.range[i];
	}
	reading.setPose(OrientedPoint(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta));
	return reading;
}

void robot_frontlaser_handler(carmen_robot_laser_message* frontlaser) {

	RangeReading reading=carmen2reading(*frontlaser);
	addReading(reading);
}

void robot_rearlaser_handler(carmen_robot_laser_message* rearlaser) {
	RangeReading reading=carmen2reading(*rearlaser);
	addReading(reading);
}

void navigator_go_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  carmen_navigator_go_message msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_navigator_go_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return
    (err, "Could not unmarshall", IPC_msgInstanceName(msgRef));
//  cerr<<"go"<<endl;
  stopped=false;
}

void navigator_stop_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  carmen_navigator_stop_message msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_navigator_stop_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return
    (err, "Could not unmarshall", IPC_msgInstanceName(msgRef));
//  cerr<<"stop"<<endl;
  stopped=true;
}

void simulator_truepos_handler(carmen_simulator_truepos_message* truepos){
	m_truepos.x=truepos->truepose.x;
	m_truepos.y=truepos->truepose.y;
	m_truepos.theta=truepos->truepose.theta;
}

void localize_frontlaser_handler(carmen_robot_laser_message *flaser);

void slam_initialize_handler(carmen_slam_initialize_message *msg)
{
	if (msg->distribution_modes > 0)
	{
		m_initialPose.x = msg->mean[0].x;
		m_initialPose.y = msg->mean[0].y;
		m_initialPose.theta = msg->mean[0].theta;
	}
}

//void publish_map(float *mymap, int width, int height, double resolution)
//{
//	carmen_map_config_t carmen_map_config;
//	carmen_grid_map_message carmen_map_msg;
//
//	IPC_RETURN_TYPE err;
//
//	carmen_map_config.x_size = width;
//	carmen_map_config.y_size = height;
//	carmen_map_config.resolution = resolution;
//	carmen_map_config.map_name = (char *) malloc ((strlen("GridSlam Map") + 1) * sizeof(char));
//	strcpy(carmen_map_config.map_name, "GridSlam Map");
//
//	carmen_map_msg.map = (unsigned char *) mymap;
//	carmen_map_msg.size = width * height * sizeof(float);
//	carmen_map_msg.compressed = 0;
//	carmen_map_msg.config = carmen_map_config;
//
//	carmen_map_msg.err_mesg = NULL;
//
//	carmen_map_msg.timestamp = carmen_get_time();
//	carmen_map_msg.host = carmen_get_host();
//
//	lock();
//	err = IPC_publishData(CARMEN_MAP_GRIDMAP_UPDATE_NAME, &carmen_map_msg);
//	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_GRIDMAP_UPDATE_NAME);
//	//fprintf(stderr, "SM");
//	unlock();
//}

bool start(){
	if (m_threadRunning)
		return false;

	carmen_robot_subscribe_frontlaser_message(&front_laser, (carmen_handler_t) robot_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_robot_subscribe_rearlaser_message(NULL, (carmen_handler_t) robot_rearlaser_handler, CARMEN_SUBSCRIBE_LATEST);
//	carmen_simulator_subscribe_truepos_message(NULL,(carmen_handler_t) simulator_truepos_handler, CARMEN_SUBSCRIBE_LATEST);

	//carmen_slam_subscribe_initialize_message(NULL, (carmen_handler_t) slam_initialize_handler, CARMEN_SUBSCRIBE_LATEST);

	IPC_RETURN_TYPE err;

	err = IPC_subscribe(CARMEN_NAVIGATOR_GO_NAME, navigator_go_handler,  NULL);
	carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_NAVIGATOR_GO_NAME);
	IPC_setMsgQueueLength(CARMEN_NAVIGATOR_GO_NAME, 1);

	err = IPC_subscribe(CARMEN_NAVIGATOR_STOP_NAME, navigator_stop_handler,  NULL);
	carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_NAVIGATOR_STOP_NAME);
	IPC_setMsgQueueLength(CARMEN_NAVIGATOR_STOP_NAME, 1);

	err = IPC_defineMsg(CARMEN_MAP_GRIDMAP_UPDATE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_GRIDMAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_GRIDMAP_UPDATE_NAME);

	err = IPC_defineMsg(CARMEN_SLAM_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_SLAM_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SLAM_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_SLAM_PARTICLES_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_SLAM_PARTICLES_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SLAM_PARTICLES_NAME);

	err = IPC_defineMsg(CARMEN_SLAM_LASER_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_SLAM_LASER_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SLAM_LASER_NAME);

	signal(SIGINT, shutdown_module);
	pthread_mutex_init(&m_mutex, 0);
	pthread_mutex_init(&m_lock, 0);
	sem_init(&m_dequeSem, 0, 0);
	m_threadRunning = true;
	pthread_create(&m_readingThread, 0, m_reading_function, 0);
	return true;
}

bool laserSensorStarted(){
	pthread_mutex_lock(&m_mutex);
	bool smok = m_frontLaser;
	pthread_mutex_unlock(&m_mutex);
	return smok;
}

bool initialPoseDefined(){
	pthread_mutex_lock(&m_mutex);
	bool smok = (
			(m_initialPose.x != -1) &&
			(m_initialPose.y != -1) &&
			(m_initialPose.theta != -1)
			);
	pthread_mutex_unlock(&m_mutex);
	return smok;
}

bool isRunning(){
	return m_threadRunning;
}

bool isStopped(){
	return stopped;
}

int queueLength(){
	int ql=0;
	pthread_mutex_lock(&m_mutex);
	ql=m_rangeDeque.size();
	pthread_mutex_unlock(&m_mutex);
	return ql;
}

OrientedPoint getTruePos(){
	return m_truepos;
}

bool getReading(RangeReading& reading){
	bool present=false;
	sem_wait(&m_dequeSem);
	pthread_mutex_lock(&m_mutex);
	if (!m_rangeDeque.empty()){
		reading=m_rangeDeque.front();
		m_rangeDeque.pop_front();
		present=true;
	}
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
	pthread_mutex_unlock(&m_mutex);
	return present;
}

carmen_robot_laser_message reading2carmen(const RangeReading& reading){
	carmen_robot_laser_message frontlaser;
	frontlaser.num_readings=reading.size();
	frontlaser.range = new float[frontlaser.num_readings];
	frontlaser.tooclose=0;
	frontlaser.laser_pose.x=frontlaser.robot_pose.x=reading.getPose().x;
	frontlaser.laser_pose.y=frontlaser.robot_pose.y=reading.getPose().y;
	frontlaser.laser_pose.theta=frontlaser.robot_pose.theta=reading.getPose().theta;
	frontlaser.tv=frontlaser.rv=0;
	frontlaser.forward_safety_dist=frontlaser.side_safety_dist=0;
	frontlaser.turn_axis=0;
	frontlaser.timestamp=reading.getTime();
	for (unsigned int i=0; i< reading.size(); i++){
		frontlaser.range[i]=(float)reading[i];
	}
	return frontlaser;
}

carmen_point_t point2carmen (const OrientedPoint& p){
	return (carmen_point_t){p.x,p.y,p.theta};
}

OrientedPoint carmen2point (const carmen_point_t& p){
	return OrientedPoint(p.x, p.y, p.theta);
}

void
save_map(ScanMatcherMap smap, GridSlamProcessor::Particle pose)
{
	static int map_number = 0;
	char map_file_name[1000];
	FILE *map_file;

	sprintf(map_file_name, "map_file%d.pnm", map_number);
	map_file = fopen(map_file_name, "w");

	IntPoint robot = smap.world2map(pose);

	// PNM file header
	fprintf(map_file, "P3\n#PNM criado por Alberto\n");
	fprintf(map_file, "%d %d\n255\n", smap.getMapSizeX(), smap.getMapSizeY());

	for (int y = smap.getMapSizeY()-1; y >= 0; y--)
	{
		for (int x = 0; x < smap.getMapSizeX(); x++)
		{
			double v = smap.cell(x,y);
			if ( (abs(x - robot.x) <= 2) && (abs(y - robot.y) <= 2) )
				fprintf(map_file, "%d\n%d\n%d\n", 255, 0, 0);
			else if (v >= 0)
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

//void publish_globalpos(const carmen_slam_globalpos_message* message)
//{
//  lock();
//
//  static carmen_slam_globalpos_message globalpos;
//
//  globalpos.timestamp = carmen_get_time();
//  globalpos.host = carmen_get_host();
//  globalpos.globalpos = message->globalpos;
//  globalpos.globalpos_std = message->globalpos_std;
//  globalpos.globalpos_xy_cov = message->globalpos_xy_cov;
//  globalpos.odometrypos = message->odometrypos;
//
//  IPC_RETURN_TYPE err = IPC_publishData(CARMEN_SLAM_GLOBALPOS_NAME, &globalpos);
//  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SLAM_GLOBALPOS_NAME);
//
//  unlock();
//}

//void publish_particles(const carmen_slam_particles_message *filter, const carmen_slam_globalpos_message *message)
//{
//  lock();
//  static carmen_slam_particles_message pmsg;
//
//  pmsg.timestamp = carmen_get_time();
//  pmsg.host = carmen_get_host();
//  pmsg.globalpos = message->globalpos;
//  pmsg.num_particles = filter->num_particles;
//  pmsg.particles = (carmen_slam_particles_p)filter->particles;
//  IPC_RETURN_TYPE err = IPC_publishData(CARMEN_SLAM_PARTICLES_NAME, &pmsg);
//  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SLAM_PARTICLES_NAME);
//  fprintf(stderr, "P");
//  unlock();
//}

//void publish_laser(const carmen_slam_laser_message* message)
//{
//  lock();
//
//  static carmen_slam_laser_message message_copy;
//
//  message_copy.timestamp = carmen_get_time();
//  message_copy.host = carmen_get_host();
//  message_copy.num_laser = message->num_laser;
//  message_copy.num_readings = message->num_readings;
//  message_copy.laser_skip = message->laser_skip;
//  message_copy.range = (float*)message->range;
//  message_copy.pose = message->pose;
//
//  IPC_RETURN_TYPE err = IPC_publishData(CARMEN_SLAM_LASER_NAME, &message_copy);
//  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SLAM_LASER_NAME);
//
//  unlock();
//}

void copy_globalpos_to_message(const carmen_slam_particles_message* particle_message,
			       carmen_slam_globalpos_message* pose_message, int best_idx)
{
  float mean_x, mean_y, mean_theta_x, mean_theta_y;
  float diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;
  float *weights, max_weight = particle_message->particles[0].weight;
  float total_weight = 0;
  int i;

  weights = (float *)calloc(particle_message->num_particles, sizeof(float));
  carmen_test_alloc(weights);
  for(i = 0; i < particle_message->num_particles; i++)
    if(particle_message->particles[i].weight > max_weight)
      max_weight = particle_message->particles[i].weight;
  for(i = 0; i < particle_message->num_particles; i++) {
    weights[i] = exp(particle_message->particles[i].weight - max_weight);
    total_weight += weights[i];
  }

  /* compute mean particle pose */
  mean_x = 0;
  mean_y = 0;
  mean_theta_x = 0;
  mean_theta_y = 0;

  if(best_idx != -1)
  {
  	mean_x = particle_message->particles[best_idx].x;
  	mean_y = particle_message->particles[best_idx].y;
  	mean_theta_x = cos(particle_message->particles[best_idx].theta);
  	mean_theta_y = sin(particle_message->particles[best_idx].theta);

  	pose_message->globalpos.x = mean_x;
		pose_message->globalpos.y = mean_y;
		if(mean_theta_x == 0)
			pose_message->globalpos.theta = 0;
		else
			pose_message->globalpos.theta = atan2(mean_theta_y, mean_theta_x);

		pose_message->globalpos_std.x = 0;
		pose_message->globalpos_std.y = 0;
		pose_message->globalpos_std.theta = 0;
		pose_message->globalpos_xy_cov = 0;
  }
  else
  {
		for(i = 0; i < particle_message->num_particles; i++) {
			mean_x += particle_message->particles[i].x * weights[i];
			mean_y += particle_message->particles[i].y * weights[i];
			mean_theta_x += cos(particle_message->particles[i].theta) * weights[i];
			mean_theta_y += sin(particle_message->particles[i].theta) * weights[i];
		}
		pose_message->globalpos.x = mean_x / total_weight;
		pose_message->globalpos.y = mean_y / total_weight;
		if(mean_theta_x == 0)
			pose_message->globalpos.theta = 0;
		else
			pose_message->globalpos.theta = atan2(mean_theta_y, mean_theta_x);
		//  pose_message->odometrypos = particle_message->last_odometry_position; TODO: Where do this come from?

		/* compute std particle pose */
		  std_x = 0;
		  std_y = 0;
		  std_theta = 0;
		  xy_cov = 0;
		  for(i = 0; i < particle_message->num_particles; i++) {
		    diff_x = (particle_message->particles[i].x - pose_message->globalpos.x);
		    diff_y = (particle_message->particles[i].y - pose_message->globalpos.y);
		    diff_theta = carmen_normalize_theta(particle_message->particles[i].theta -
							pose_message->globalpos.theta);
		    std_x += carmen_square(diff_x);
		    std_y += carmen_square(diff_y);
		    std_theta += carmen_square(diff_theta);
		    xy_cov += diff_x * diff_y;
		  }
		  pose_message->globalpos_std.x = sqrt(std_x / particle_message->num_particles);
		  pose_message->globalpos_std.y = sqrt(std_y / particle_message->num_particles);
		  pose_message->globalpos_std.theta = sqrt(std_theta / particle_message->num_particles);
		  pose_message->globalpos_xy_cov = sqrt(xy_cov / particle_message->num_particles);
  }

  free(weights);
}

void copy_particles_to_message(const GridSlamProcessor::ParticleVector& particles, carmen_slam_particles_message* message)
{
	for(int i=0; i < message->num_particles; i++)
	{
		message->particles[i].x = particles[i].pose.x;
		message->particles[i].y = particles[i].pose.y;
		message->particles[i].theta = particles[i].pose.theta;
		message->particles[i].weight = particles[i].weight;
	}
}

void copy_laserbeams_to_message(const RangeReading& laser, carmen_slam_laser_message* message)
{
	OrientedPoint pose = laser.getPose();
	message->pose.x = pose.x;
	message->pose.y = pose.y;
	message->pose.theta = pose.theta;
	for(int i=0; i < message->num_readings; i++)
	{
		message->range[i] = laser[i];
	}
}

int main(int argc, char ** argv)
{
	std::string outfilename="";
	double xmin=0.0;
	double ymin=0.0;
	double xmax=20.0;
	double ymax=20.0;
	double delta=0.1;

	//scan matching parameters
	double sigma=0.05;
	double maxrange=80.; //80.
	double maxUrange=80.; //80.
	double regscore=1e4;
	double lstep=.05;
	double astep=.05;
	int kernelSize=1;
	int iterations=3;
	double critscore=0.;
	double maxMove=1.;
	double lsigma=.075;
	double ogain=3;
	int lskip=0;

	//motion model parameters
	double srr = 0.01, srt = 0.01, str = 0.01, stt = 0.01;
	//particle parameters
	int num_particles = 30;


	//gfs parameters
	double angularUpdate = 0.2;
	double linearUpdate = 0.2;
	double resampleThreshold = 0.5;
	bool generateMap = true;
	float *myFloatMap = NULL;

	int oldX = 0;
	int oldY = 0;

	std::string configfilename = "";

	carmen_slam_particles_message particle_msg;
	carmen_slam_globalpos_message pose_msg;
	carmen_slam_laser_message laser_msg;

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
		num_particles = cfg.value("gfs","particles", num_particles);
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
		parseInt("-particles", num_particles);
		parseDouble("-angularUpdate", angularUpdate);
		parseDouble("-linearUpdate", linearUpdate);
		parseDouble("-lsigma", lsigma);
		parseDouble("-lobsGain", ogain);
		parseInt("-lskip", lskip);
		parseDouble("-resampleThreshold", resampleThreshold);
		parseFlag("-generateMap", generateMap);
	CMD_PARSE_END;

//	cerr << "Parameter parsed, connecting to Carmen!";

	carmen_ipc_initialize(1, argv);

	start();

//	cerr << "Awaiting laser readings.\n" << flush;
	while (!laserSensorStarted()){
		usleep(500000);
//		cerr << "." << flush;
	}

//	cerr << "Awaiting initial pose has been defined.\n" << flush;
//	while (!initialPoseDefined()){
//		usleep(500000);
//		cerr << "." << flush;
//	}

	m_initialPose.x = (float)(xmax - xmin)/2.0;
	m_initialPose.y = (float)(ymax - ymin)/2.0;
	m_initialPose.theta = 0.0;

	//CREATION

	GridSlamProcessor* processor = new GridSlamProcessor;

	//SENSOR MAP
	//loads from the carmen wrapper the laser and robot settings
	GMapping::SensorMap sensorMap=m_sensorMap;
//	cerr << "Connected " << endl;
	processor->setSensorMap(sensorMap);

	laser_msg.num_readings = processor->m_matcher.laserBeams();
	laser_msg.range = (float*) malloc(laser_msg.num_readings * sizeof(float));
	laser_msg.laser_skip = 0; //TODO integrate_angle / angular_resolution;

	particle_msg.num_particles = num_particles;
	particle_msg.particles = (carmen_slam_particles_p)malloc(particle_msg.num_particles * sizeof(carmen_slam_particles_t));

	//set the command line parameters
	processor->setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, lsigma, ogain, lskip);
	processor->setMotionModelParameters(srr, srt, str, stt);
	processor->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);
	processor->setgenerateMap(generateMap);

	//INITIALIZATION
	processor->init(num_particles, xmin, ymin, xmax, ymax, delta, m_initialPose);
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
		while (getReading(rr))
		{
			bool processed = processor->processScan(rr);

			//this returns true when the algorithm effectively processes (the traveled path since the last processing is over a given threshold)
			if (processed){
//				cerr << "PROCESSED" << endl;
				//for searching for the BEST PARTICLE INDEX
				best_idx = processor->getBestParticleIndex();

				//if you want to access to the PARTICLE VECTOR
				const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
				//remember to use a const reference, otherwise it copys the whole particles and maps

				//this is for recovering the tree of PARTICLE TRAJECTORIES (obtaining the ancestor of each particle)
//				cerr << "Particle reproduction story begin" << endl;
//				for (unsigned int i = 0; i < particles.size(); i++){
//					cerr << particles[i].previousIndex << "->"  << i << " ";
//				}
//				cerr << "Particle reproduction story end" << endl;

				//then if you want to access the BEST MAP,
				//of course by copying it in a plain structure
				Map<double, DoubleArray2D, false>* mymap = particles[best_idx].map.toDoubleMap();

				int y, x;

				if (myFloatMap == NULL)
				{
					myFloatMap = (float *) calloc (mymap->getMapSizeX() * mymap->getMapSizeY(), sizeof(float));

					oldX = mymap->getMapSizeX();
					oldY = mymap->getMapSizeY();
				}


				if (oldX != mymap->getMapSizeX())
				{
					delete(myFloatMap);

					myFloatMap = (float *) calloc (mymap->getMapSizeX() * mymap->getMapSizeY(), sizeof(float));

					oldX = mymap->getMapSizeX();
					oldY = mymap->getMapSizeY();
				}
				else
				{
					if(oldY != mymap->getMapSizeY())
					{
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

				copy_particles_to_message(particles, &particle_msg);

				copy_globalpos_to_message(&particle_msg, &pose_msg, -1);

				copy_laserbeams_to_message(rr, &laser_msg);

				lock();
				carmen_slam_publish_laser(&laser_msg);
				unlock();

				lock();
				carmen_slam_publish_globalpos(&pose_msg);
				unlock();

				lock();
				carmen_slam_publish_particles(&particle_msg, &pose_msg);
				unlock();

				lock();
				carmen_slam_publish_map(myFloatMap, mymap->getMapSizeX(), mymap->getMapSizeY(), mymap->getMapResolution());
				unlock();

				save_map(particles[best_idx].map, particles[best_idx]);

				double best_weight = particles[best_idx].weightSum;
//				cerr << "Best Particle is " << best_idx << " with weight " << best_weight << endl;

//				cerr << __PRETTY_FUNCTION__  << "CLONING... " << endl;
				GridSlamProcessor* newProcessor = processor->clone();
//				cerr << "DONE" << endl;
//				cerr << __PRETTY_FUNCTION__  << "DELETING... " << endl;
				delete processor;
//				cerr << "DONE" << endl;
				processor = newProcessor;

			}
		}
	}
	if (particle_msg.particles != NULL)
		free(particle_msg.particles);
	return 0;
}

