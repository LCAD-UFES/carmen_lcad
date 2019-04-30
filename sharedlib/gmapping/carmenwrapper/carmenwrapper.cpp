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


#include "carmenwrapper.h"
#include <carmen/localizecore.h>
#include <carmen/likelihood_map.h>

using namespace GMapping;
using namespace std;

//static vars for the carmenwrapper
SensorMap CarmenWrapper::m_sensorMap;
deque<RangeReading> CarmenWrapper::m_rangeDeque;
pthread_mutex_t CarmenWrapper::m_mutex;
sem_t CarmenWrapper::m_dequeSem;
pthread_mutex_t CarmenWrapper::m_lock;
pthread_t CarmenWrapper::m_readingThread;
RangeSensor* CarmenWrapper::m_frontLaser=0;
RangeSensor* CarmenWrapper::m_rearLaser=0;
bool CarmenWrapper::m_threadRunning=false;
OrientedPoint CarmenWrapper::m_truepos;
bool CarmenWrapper::stopped=true;

OrientedPoint CarmenWrapper::initialPose;

carmen_localize_map_t localize_map;
carmen_localize_param_t localize_param;
carmen_localize_summary_t localize_summary;
carmen_localize_particle_filter_p localize_filter;

carmen_robot_laser_message front_laser;

void localize_frontlaser_handler(carmen_robot_laser_message *flaser);

void CarmenWrapper::initializeIPC(const char* name) {
  carmen_ipc_initialize(1,(char **)&name);
}

void CarmenWrapper::gfsMapToCarmenMap(float *mymap, int width, int height, double resolution)
{
	carmen_map_config_t carmen_map_config;
	carmen_grid_map_message carmen_map_msg;

	IPC_RETURN_TYPE err;

	carmen_map_config.x_size = width;
	carmen_map_config.y_size = height;
	carmen_map_config.resolution = resolution;
	carmen_map_config.map_name = (char *) malloc ((strlen("GridSlam Map") + 1) * sizeof(char));
	strcpy(carmen_map_config.map_name, "GridSlam Map");

	carmen_map_msg.map = (unsigned char *) mymap;
	carmen_map_msg.size = width * height * sizeof(float);
	carmen_map_msg.compressed = 0;
	carmen_map_msg.config = carmen_map_config;

	carmen_map_msg.err_mesg = NULL;

	carmen_map_msg.timestamp = carmen_get_time();
	carmen_map_msg.host = carmen_get_host();

	lock();
	err = IPC_publishData(CARMEN_MAP_GRIDMAP_UPDATE_NAME, &carmen_map_msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_GRIDMAP_UPDATE_NAME);
	//fprintf(stderr, "SM");
	unlock();
}


int CarmenWrapper::registerLocalizationMessages(){
  lock();
  IPC_RETURN_TYPE err;

  /* register globalpos message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_GLOBALPOS_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_GLOBALPOS_NAME);

  /* register robot particle message */
  err = IPC_defineMsg(CARMEN_LOCALIZE_PARTICLE_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_LOCALIZE_PARTICLE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_PARTICLE_NAME);

  unlock();
  return 0;
}

bool CarmenWrapper::start(const char* name){
	if (m_threadRunning)
		return false;

	carmen_robot_subscribe_frontlaser_message(&front_laser, (carmen_handler_t) robot_frontlaser_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_robot_subscribe_rearlaser_message(NULL, (carmen_handler_t) robot_rearlaser_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_simulator_subscribe_truepos_message(NULL,(carmen_handler_t) simulator_truepos_handler, CARMEN_SUBSCRIBE_LATEST);

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

	err = IPC_defineMsg(CARMEN_LOCALIZE_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_LOCALIZE_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_GLOBALPOS_NAME);

	registerLocalizationMessages();

	signal(SIGINT, shutdown_module);
	pthread_mutex_init(&m_mutex, 0);
	pthread_mutex_init(&m_lock, 0);
	sem_init(&m_dequeSem, 0, 0);
	m_threadRunning = true;
	pthread_create(&m_readingThread, 0, m_reading_function, 0);
	return true;
}

void CarmenWrapper::lock(){
	//cerr <<"LOCK" << endl;
	pthread_mutex_lock(&m_lock);
}

void CarmenWrapper::unlock(){
	//cerr <<"UNLOCK" << endl;
	pthread_mutex_unlock(&m_lock);
}

bool CarmenWrapper::sensorMapComputed(){
	pthread_mutex_lock(&m_mutex);
	bool smok = m_frontLaser;
	pthread_mutex_unlock(&m_mutex);
	return smok;
}

const SensorMap& CarmenWrapper::sensorMap(){
	return m_sensorMap;
}

bool CarmenWrapper::isRunning(){
	return m_threadRunning;
}

bool CarmenWrapper::isStopped(){
	return stopped;
}

int CarmenWrapper::queueLength(){
	int ql=0;
	pthread_mutex_lock(&m_mutex);
	ql=m_rangeDeque.size();
	pthread_mutex_unlock(&m_mutex);
	return ql;
}

OrientedPoint CarmenWrapper::getTruePos(){
	return m_truepos;
}

bool CarmenWrapper::getReading(RangeReading& reading){
	bool present=false;
	sem_wait(&m_dequeSem);
	pthread_mutex_lock(&m_mutex);
	if (!m_rangeDeque.empty()){
//		cerr << __PRETTY_FUNCTION__ << ": queue size=" <<m_rangeDeque.size() << endl;
		reading=m_rangeDeque.front();
		m_rangeDeque.pop_front();
		present=true;
	}
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
//	cerr << "fetch. elements= "<< m_rangeDeque.size() << " sval=" << sval <<endl;
	pthread_mutex_unlock(&m_mutex);
	return present;
}

void CarmenWrapper::addReading(RangeReading& reading){
	pthread_mutex_lock(&m_mutex);
	m_rangeDeque.push_back(reading);
	pthread_mutex_unlock(&m_mutex);
	sem_post(&m_dequeSem);
	int sval;
	sem_getvalue(&m_dequeSem,&sval);
//	cerr << "post. elements= "<< m_rangeDeque.size() << " sval=" << sval <<endl;
}

void CarmenWrapper::robot_frontlaser_handler(carmen_robot_laser_message* frontlaser) {
//	localize_frontlaser_handler(frontlaser);
	RangeReading reading=carmen2reading(*frontlaser);
	addReading(reading);
}

void CarmenWrapper::robot_rearlaser_handler(carmen_robot_laser_message* rearlaser) {
	RangeReading reading=carmen2reading(*rearlaser);
	addReading(reading);
}

void CarmenWrapper::navigator_go_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  carmen_navigator_go_message msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_navigator_go_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return
    (err, "Could not unmarshall", IPC_msgInstanceName(msgRef));
  cerr<<"go"<<endl;
  stopped=false;
}

void CarmenWrapper::navigator_stop_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) {
  carmen_navigator_stop_message msg;
  FORMATTER_PTR formatter;
  IPC_RETURN_TYPE err;

  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &msg,
			   sizeof(carmen_navigator_stop_message));
  IPC_freeByteArray(callData);

  carmen_test_ipc_return
    (err, "Could not unmarshall", IPC_msgInstanceName(msgRef));
  cerr<<"stop"<<endl;
  stopped=true;
}

void CarmenWrapper::simulator_truepos_handler(carmen_simulator_truepos_message* truepos){
	m_truepos.x=truepos->truepose.x;
	m_truepos.y=truepos->truepose.y;
	m_truepos.theta=truepos->truepose.theta;
}

RangeReading CarmenWrapper::carmen2reading(const carmen_robot_laser_message& msg){
	//either front laser or rear laser
	double dth=msg.laser_pose.theta-msg.robot_pose.theta;
	dth=atan2(sin(dth), cos(dth));

	if (msg.laser_pose.theta==msg.robot_pose.theta && !m_frontLaser){
		double res=0;
		res = msg.config.angular_resolution;
// 		if (msg.num_readings==180 || msg.num_readings==181)
// 			res=M_PI/180;
// 		if (msg.num_readings==360 || msg.num_readings==361)
// 			res=M_PI/360;
		assert(res>0);
		string sensorName="FLASER";
		OrientedPoint rpose(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta);
		OrientedPoint lpose(msg.laser_pose.x, msg.laser_pose.y, msg.laser_pose.theta);
		OrientedPoint dp=absoluteDifference(lpose, rpose);
		m_frontLaser=new RangeSensor(sensorName,msg.num_readings, res, OrientedPoint(0,0,msg.laser_pose.theta-msg.robot_pose.theta), 0,
					      msg.config.maximum_range);
		m_frontLaser->updateBeamsLookup();
		m_sensorMap.insert(make_pair(sensorName, m_frontLaser));

		cout << __PRETTY_FUNCTION__
		     << ": " << sensorName <<" configured."
		     << " Readings " << m_frontLaser->beams().size()
		     << " Resolution " << res << endl;
	}
	if (msg.laser_pose.theta!=msg.robot_pose.theta && !m_rearLaser){
		double res=0;
		res = msg.config.angular_resolution;
// 		if (msg.num_readings==180 || msg.num_readings==181)
// 			res=M_PI/180;
// 		if (msg.num_readings==360 || msg.num_readings==361)
// 			res=M_PI/360;
		assert(res>0);
		OrientedPoint rpose(msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta);
		OrientedPoint lpose(msg.laser_pose.x, msg.laser_pose.y, msg.laser_pose.theta);
		OrientedPoint dp=absoluteDifference(lpose, rpose);
		string sensorName="RLASER";
		m_rearLaser=new RangeSensor(sensorName,msg.num_readings, res, OrientedPoint(0,0,msg.laser_pose.theta-msg.robot_pose.theta), 0,
					      msg.config.maximum_range);
		m_rearLaser->updateBeamsLookup();
		m_sensorMap.insert(make_pair(sensorName, m_rearLaser));

		cout << __PRETTY_FUNCTION__
		     << ": " << sensorName <<" configured."
		     << " Readings " << m_rearLaser->beams().size()
		     << " Resolution " << res << endl;
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

void CarmenWrapper::publish_globalpos(carmen_localize_summary_p summary)
{
  lock();
  static carmen_localize_globalpos_message globalpos;
  IPC_RETURN_TYPE err;

  globalpos.timestamp = carmen_get_time();
  globalpos.host = carmen_get_host();
  globalpos.globalpos = summary->mean;
  globalpos.globalpos_std = summary->std;
  globalpos.globalpos_xy_cov = summary->xy_cov;
  globalpos.odometrypos = summary->odometry_pos;
  globalpos.converged = summary->converged;
  err = IPC_publishData(CARMEN_LOCALIZE_GLOBALPOS_NAME, &globalpos);
  carmen_test_ipc_exit(err, "Could not publish",
		       CARMEN_LOCALIZE_GLOBALPOS_NAME);
  unlock();
}

/* publish a particle message */
void CarmenWrapper::publish_particles(carmen_localize_particle_filter_p filter,
		       carmen_localize_summary_p summary)
{
  lock();
  static carmen_localize_particle_message pmsg;
  IPC_RETURN_TYPE err;

  pmsg.timestamp = carmen_get_time();
  pmsg.host = carmen_get_host();
  pmsg.globalpos = summary->mean;
  pmsg.globalpos_std = summary->mean;
  pmsg.num_particles = filter->param->num_particles;
  pmsg.particles = (carmen_localize_particle_ipc_p)filter->particles;
  err = IPC_publishData(CARMEN_LOCALIZE_PARTICLE_NAME, &pmsg);
  carmen_test_ipc_exit(err, "Could not publish",
		       CARMEN_LOCALIZE_PARTICLE_NAME);
  fprintf(stderr, "P");
  unlock();
}

void summarize_particles(carmen_localize_particle_filter_p filter,
			       carmen_localize_summary_p summary,
			       int num_readings,
//			       float *range,
//			       double angular_resolution,
//			       double first_beam_angle,
//			       double forward_offset,
			       int backwards)
{
  float mean_x, mean_y, mean_theta_x, mean_theta_y, angle;
  float diff_x, diff_y, diff_theta, std_x, std_y, std_theta, xy_cov;
  float *weights, max_weight = filter->particles[0].weight;
  float total_weight = 0;
  int i;//, x, y;

  summary->converged = !filter->converged;

  weights = (float *)calloc(filter->param->num_particles, sizeof(float));

  for(i = 0; i < filter->param->num_particles; i++)
    if(filter->particles[i].weight > max_weight)
      max_weight = filter->particles[i].weight;
  for(i = 0; i < filter->param->num_particles; i++) {
    weights[i] = exp(filter->particles[i].weight - max_weight);
    total_weight += weights[i];
  }

//  if(filter->param->do_scanmatching)
//    carmen_localize_laser_scan_gd(summary->num_readings,
//				  range,
//				  angular_resolution,
//				  first_beam_angle,
//				  &summary->mean,
//				  forward_offset, map, 1);

  /* compute mean particle pose */
  mean_x = 0;
  mean_y = 0;
  mean_theta_x = 0;
  mean_theta_y = 0;
  for(i = 0; i < filter->param->num_particles; i++) {
    mean_x += filter->particles[i].x * weights[i];
    mean_y += filter->particles[i].y * weights[i];
    mean_theta_x += cos(filter->particles[i].theta) * weights[i];
    mean_theta_y += sin(filter->particles[i].theta) * weights[i];
  }
  summary->mean.x = mean_x / total_weight;
  summary->mean.y = mean_y / total_weight;
  if(mean_theta_x == 0)
    summary->mean.theta = 0;
  else
    summary->mean.theta = atan2(mean_theta_y, mean_theta_x);
  summary->odometry_pos = filter->last_odometry_position;

  /* compute std particle pose */
  std_x = 0;
  std_y = 0;
  std_theta = 0;
  xy_cov = 0;
  for(i = 0; i < filter->param->num_particles; i++) {
    diff_x = (filter->particles[i].x - summary->mean.x);
    diff_y = (filter->particles[i].y - summary->mean.y);
    diff_theta = carmen_normalize_theta(filter->particles[i].theta -
					summary->mean.theta);
    std_x += carmen_square(diff_x);
    std_y += carmen_square(diff_y);
    std_theta += carmen_square(diff_theta);
    xy_cov += diff_x * diff_y;
  }
  summary->std.x = sqrt(std_x / filter->param->num_particles);
  summary->std.y = sqrt(std_y / filter->param->num_particles);
  summary->std.theta = sqrt(std_theta / filter->param->num_particles);
  summary->xy_cov = sqrt(xy_cov / filter->param->num_particles);

  /* compute mean scan */
//  summary->num_readings = num_readings;
//  for(i = 0; i < num_readings; i++) {
//    summary->mean_scan[i].range = range[i];
//    summary->mean_scan[i].mask = filter->laser_mask[i];
//    if(backwards) {
//      angle = summary->mean.theta + M_PI +
//	first_beam_angle + i * angular_resolution;
//      summary->mean_scan[i].x = summary->mean.x - forward_offset *
//	cos(summary->mean.theta) + cos(angle) * range[i];
//      summary->mean_scan[i].y = summary->mean.y - forward_offset *
//	sin(summary->mean.theta) + sin(angle) * range[i];
//    }
//    else {
//      angle = summary->mean.theta +
//	first_beam_angle + i * angular_resolution;
//      summary->mean_scan[i].x = summary->mean.x + forward_offset *
//	cos(summary->mean.theta) + cos(angle) * range[i];
//      summary->mean_scan[i].y = summary->mean.y + forward_offset *
//	sin(summary->mean.theta) + sin(angle) * range[i];
//    }
//    x = (summary->mean_scan[i].x / map->config.resolution);
//    y = (summary->mean_scan[i].y / map->config.resolution);
//    if(x < 0 || y < 0 || x >= map->config.x_size || y >= map->config.y_size ||
//       map->carmen_map.map[x][y] == -1)
//      summary->mean_scan[i].prob = filter->param->tracking_beam_minlikelihood; //SMALL_PROB;
//    else
//      summary->mean_scan[i].prob = exp(map->prob[x][y]);
//  }

  free(weights);
}

void read_localize_parameters(int argc, char **argv, carmen_localize_param_p param)
{
  double integrate_angle_deg;
  integrate_angle_deg=1.0;

  carmen_param_t param_list[] = {
    {"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE,
     &param->front_laser_offset, 0, NULL},
    {"robot", "rearlaser_offset", CARMEN_PARAM_DOUBLE,
     &param->rear_laser_offset, 0, NULL},
    {"localize", "use_rear_laser", CARMEN_PARAM_ONOFF,
     &param->use_rear_laser, 0, NULL},
    {"localize", "num_particles", CARMEN_PARAM_INT,
     &param->num_particles, 0, NULL},
    {"localize", "laser_max_range", CARMEN_PARAM_DOUBLE, &param->max_range, 1, NULL},
    {"localize", "min_wall_prob", CARMEN_PARAM_DOUBLE,
     &param->min_wall_prob, 0, NULL},
    {"localize", "outlier_fraction", CARMEN_PARAM_DOUBLE,
     &param->outlier_fraction, 0, NULL},
    {"localize", "update_distance", CARMEN_PARAM_DOUBLE,
     &param->update_distance, 0, NULL},
    {"localize", "integrate_angle_deg", CARMEN_PARAM_DOUBLE,
     &integrate_angle_deg, 0, NULL},
    {"localize", "do_scanmatching", CARMEN_PARAM_ONOFF,
     &param->do_scanmatching, 1, NULL},
    {"localize", "constrain_to_map", CARMEN_PARAM_ONOFF,
     &param->constrain_to_map, 1, NULL},
#ifdef OLD_MOTION_MODEL
    {"localize", "odom_a1", CARMEN_PARAM_DOUBLE, &param->odom_a1, 1, NULL},
    {"localize", "odom_a2", CARMEN_PARAM_DOUBLE, &param->odom_a2, 1, NULL},
    {"localize", "odom_a3", CARMEN_PARAM_DOUBLE, &param->odom_a3, 1, NULL},
    {"localize", "odom_a4", CARMEN_PARAM_DOUBLE, &param->odom_a4, 1, NULL},
#endif
    {"localize", "occupied_prob", CARMEN_PARAM_DOUBLE,
     &param->occupied_prob, 0, NULL},
    {"localize", "lmap_std", CARMEN_PARAM_DOUBLE,
     &param->lmap_std, 0, NULL},
    {"localize", "global_lmap_std", CARMEN_PARAM_DOUBLE,
     &param->global_lmap_std, 0, NULL},
    {"localize", "global_evidence_weight", CARMEN_PARAM_DOUBLE,
     &param->global_evidence_weight, 0, NULL},
    {"localize", "global_distance_threshold", CARMEN_PARAM_DOUBLE,
     &param->global_distance_threshold, 1, NULL},
    {"localize", "global_test_samples", CARMEN_PARAM_INT,
     &param->global_test_samples, 1, NULL},
    {"localize", "use_sensor", CARMEN_PARAM_ONOFF,
     &param->use_sensor, 0, NULL},
    {"localize", "tracking_beam_minlikelihood", CARMEN_PARAM_DOUBLE,
     &param->tracking_beam_minlikelihood, 0, NULL},
    {"localize", "global_beam_minlikelihood", CARMEN_PARAM_DOUBLE,
     &param->global_beam_minlikelihood, 0, NULL}
  };

  carmen_param_install_params(argc, argv, param_list,
			      sizeof(param_list) / sizeof(param_list[0]));

  param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);

}

void CarmenWrapper::init_localize_filter(int argc, char **argv, int num_particles)
{
	read_localize_parameters(argc, argv, &localize_param);

	if (localize_param.num_particles != num_particles)
		carmen_die("Wrong number of particles!\n");

	localize_filter = carmen_localize_particle_filter_new(&localize_param);
	localize_filter->initialized = 1;
	localize_filter->first_odometry = 1;
	localize_filter->converged = 1;
	localize_filter->distance_travelled = 0;

	carmen_localize_subscribe_initialize_message(NULL,
			(carmen_handler_t)
			init_handler,
			CARMEN_SUBSCRIBE_LATEST);
}

void CarmenWrapper::init_handler(carmen_localize_initialize_message *init_msg)
{
	carmen_localize_initialize_particles_uniform(localize_filter, &front_laser, &localize_map);
}

void CarmenWrapper::send_particles(const GridSlamProcessor::ParticleVector &vector, unsigned int num_laser_beams)
{
	for(int i=0; i<localize_filter->param->num_particles; i++)
	{
		localize_filter->particles[i].x = vector[i].pose.x;
		localize_filter->particles[i].y = vector[i].pose.y;
		localize_filter->particles[i].theta = vector[i].pose.theta;
		localize_filter->particles[i].weight = vector[i].weight;
	}
    summarize_particles(localize_filter, &localize_summary, num_laser_beams, 0);
    CarmenWrapper::publish_globalpos(&localize_summary);
    CarmenWrapper::publish_particles(localize_filter, &localize_summary);
}

void localize_frontlaser_handler(carmen_robot_laser_message *flaser)
{
  carmen_localize_run(localize_filter, &localize_map, flaser,
		  localize_filter->param->front_laser_offset, 0);
  if(localize_filter->initialized) {
    carmen_localize_summarize(localize_filter, &localize_summary, &localize_map, flaser->num_readings,
			      flaser->range, localize_filter->param->front_laser_offset,
			      flaser->config.angular_resolution,
			      flaser->config.start_angle, 0);
    CarmenWrapper::publish_globalpos(&localize_summary);
    CarmenWrapper::publish_particles(localize_filter, &localize_summary);
  }else{
	  localize_filter->initialized = 1;
	  localize_filter->first_odometry = 1;
	  localize_filter->converged = 1;
	  localize_filter->distance_travelled = 0;
  }
}

void * CarmenWrapper::m_reading_function(void*){
	while (true) {
		lock();
		IPC_listen(100);
		unlock();
		usleep(20000);
	}
	return 0;
}

void CarmenWrapper::shutdown_module(int sig){
  if(sig == SIGINT) {
      carmen_ipc_disconnect();

      fprintf(stderr, "\nDisconnecting (shutdown_module(%d) called).\n",sig);
      exit(0);
  }
}

carmen_robot_laser_message CarmenWrapper::reading2carmen(const RangeReading& reading){
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

carmen_point_t CarmenWrapper::point2carmen (const OrientedPoint& p){
	return (carmen_point_t){p.x,p.y,p.theta};
}

OrientedPoint CarmenWrapper::carmen2point (const carmen_point_t& p){
	return OrientedPoint(p.x, p.y, p.theta);
}

/*
int main (int argc, char** argv) {
	CarmenWrapper::start(argc, argv);
	while(1){
		sleep(2);
		RangeReading reading(0,0);
		while(CarmenWrapper::getReading(reading)){
			cout << "FLASER " <<  reading.size();
			for (int i=0; i<reading.size(); i++)
				cout << " " << reading[i];
			cout << reading.getPose().x << " "
			     << reading.getPose().y << " "
			     << reading.getPose().theta << " 0 cazzo 0" << endl;
		}
		cout << endl;
	}
	return 1;
}
*/

