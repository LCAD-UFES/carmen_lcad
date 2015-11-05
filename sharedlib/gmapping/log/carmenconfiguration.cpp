#include <cstdlib>
#include "carmenconfiguration.h"
#include <iostream>
#include <sstream>
#include <assert.h>
#include <sys/types.h>
#include <sensor_odometry/odometrysensor.h>
#include <sensor_range/rangesensor.h>


#define LINEBUFFER_SIZE 10000

namespace GMapping {

using namespace std;

inline int round(double X){
  if (X >= 0)
    return (int)(X + 0.5);
  else
    return (int)(X - 0.5);
}

istream& CarmenConfiguration::load(istream& is){
	configValues.clear();
	char buf[LINEBUFFER_SIZE];
	bool laseron=false;
	bool rlaseron=false;
	bool rlaser1=false;
	bool rlaser2=false;

	string beams;
	string rbeams;

	while (is){
		is.getline(buf, LINEBUFFER_SIZE);
		istringstream lis(buf);

		string qualifier;
		string name;

		if (lis)
			lis >> qualifier;
		else
			continue;
		//this is a workaround for carmen log files
		//the number lf laser beams should be specofoed in the config
		//part of the log
		if (qualifier=="FLASER"){
			laseron=true;
			lis >> beams;
		}
		if (qualifier=="RLASER"){
			rlaseron=true;
			lis >> rbeams;
		}
		if (qualifier=="ROBOTLASER1"){
			string laser_type, start_angle, field_of_view, angular_resolution, maximum_range, accuracy, remission_mode;
			lis >> laser_type>> start_angle>> field_of_view>> angular_resolution>> maximum_range>> accuracy>> remission_mode>> beams;
			rlaser1=true;
		}
		if (qualifier=="ROBOTLASER2"){
			string laser_type, start_angle, field_of_view, angular_resolution, maximum_range, accuracy, remission_mode;
			lis >> laser_type>> start_angle>> field_of_view>> angular_resolution>> maximum_range>> accuracy>> remission_mode>> rbeams;
			rlaser2=true;
		}
		if (qualifier!="PARAM")
			continue;
		if (lis)
			lis >> name;
		else continue;


		vector<string> v;
		while (lis){
			string cparm;
			lis >> cparm;
			if (lis)
				v.push_back(cparm);
		}
		configValues.insert(make_pair(name, v));
	}
	if (laseron || rlaser1){
		vector<string> v;
		v.push_back(beams);
		configValues.insert(make_pair("laser_beams", v));
		cerr << "FRONT LASER BEAMS FROM LOG: " << beams << endl;
		v.clear();
		v.push_back("on");
		configValues.insert(make_pair("robot_use_laser", v));
	}
	if (rlaseron || rlaser2){
		vector<string> v;
		v.push_back(rbeams);
		configValues.insert(make_pair("rear_laser_beams", v));
		cerr << "REAR LASER BEAMS FROM LOG: " << beams << endl;
		v.clear();
		v.push_back("on");
		configValues.insert(make_pair("robot_use_rear_laser", v));
	}
	return is;
}

void CarmenConfiguration::loadSonarParams(GMapping::RangeSensor* sonar) const {

	if (sonar == NULL)
		return;

	//the center of the sonar is the center of the base
	sonar->m_pose.x=sonar->m_pose.y=sonar->m_pose.theta=0;
	std::map<std::string, std::vector<std::string> >::const_iterator key;

	double maxrange=10.;
	key=configValues.find("robot_max_sonar");
	if (key!=configValues.end()){
		maxrange=atof(key->second.front().c_str());
		cerr << "max sonar:" << maxrange << endl;
	}

	unsigned int sonar_num=0;
	key=configValues.find("robot_num_sonars");
	if (key!=configValues.end()){
		sonar_num=atoi(key->second.front().c_str());
		cerr << "robot_num_sonars" << sonar_num << endl;
	}

	key=configValues.find("robot_sonar_offsets");
	if (key!=configValues.end()){
		const vector<string> & soff=key->second;

		if( (soff.size()/3<sonar_num)){
			cerr << __PRETTY_FUNCTION__ << ": Error " << soff.size()
			<< " parameters for defining the sonar offsets"
			<< " while the specified number of sonars requires "
			<< sonar_num*3 << " parameters at least" << endl;
		} else {
			cerr << __PRETTY_FUNCTION__ << ": Ok " << soff.size() << " parameters for defining the sonar offsets of " << sonar_num << " devices" << endl;
		}


		RangeSensor::Beam beam;

		for (unsigned int i=0; i<sonar_num*3; i+=3){
			beam.span=M_PI/180.*7.5;
			beam.pose.x=atof(soff[i].c_str());
			beam.pose.y=atof(soff[i+1].c_str());
			beam.pose.theta=atof(soff[i+2].c_str());
			beam.maxRange=maxrange;
			sonar->m_beams.push_back(beam);
			cerr << "beam_x" << beam.pose.x;
			cerr << " beam_y" << beam.pose.y;
			cerr << " beam_theta" << beam.pose.theta << endl;;
		}
	}
	sonar->updateBeamsLookup();
}

void CarmenConfiguration::loadLaserParams(GMapping::RangeSensor* laser, bool front) const {

	if (laser == NULL)
		return;

	laser->newFormat=true;
	cerr << laser->getName() << (front ? " FRONT " : " REAR ") << "inserted" << endl;

	//by default the center of the robot is the center of the laser
	laser->m_pose.x=laser->m_pose.y=laser->m_pose.theta=0;
	std::map<std::string, std::vector<std::string> >::const_iterator key;

	if (front)
		key=configValues.find("robot_frontlaser_offset");
	else
		key=configValues.find("robot_rearlaser_offset");

	if (key!=configValues.end()){
		laser->m_pose.x=atof(key->second.front().c_str());
		cerr << "OFFSET= " << laser->m_pose.x << endl;
	}

	RangeSensor::Beam beam;

	unsigned int beam_no=1 + round(this->fov_degrees / this->res_degrees);

	if (front)
		key=configValues.find("laser_beams");
	else
		key=configValues.find("rear_laser_beams");

	if (key!=configValues.end()){
		beam_no=atoi(key->second.front().c_str());
	}

	double maxrange=this->max_range;
	double resolution=this->res_degrees;

	if (front)
		key=configValues.find("laser_front_laser_resolution");
	else
		key=configValues.find("laser_rear_laser_resolution");

	if (key!=configValues.end()){
		resolution=atof(key->second.front().c_str());
		cerr << "RES " << resolution << endl;
	}

	/*
	if (beam_no==180 || beam_no==181)
	  resolution =1.;
	else if (beam_no==360 || beam_no==361)
	  resolution =.5;
	else if (beam_no==540 || beam_no==541)
	  resolution =.5;
	else if (beam_no==769) {
	  resolution =360./1024.;
	  maxrange = 4.1;
	}
	else if (beam_no==682) {
	  resolution =360./1024.;
	  maxrange = 4.1;
	}
	else if (beam_no==683) {
	  resolution =360./1024.;
	  maxrange = 5.5;
	}
	else if (beam_no==633) { // Kinect
	  resolution =56.9/633.;
	  maxrange = 10.0;
	}
	 */

	laser->m_beams.resize(beam_no);
	double center_beam=(double)beam_no/2.;
	uint low_index=(uint)floor(center_beam);
	uint up_index=(uint)ceil(center_beam);
	double step=resolution*M_PI/180.;
	double angle=beam_no%2?0:step;
	unsigned int i=beam_no%2?0:1;
	for (; i<low_index+1; i++, angle+=step){
		beam.span=0;
		beam.pose.x=0;
		beam.pose.y=0;
		beam.s = 1;
		beam.c = 1;
		beam.pose.theta=-angle;
		beam.maxRange=maxrange;
		laser->m_beams[low_index-i]=beam;
		beam.pose.theta=angle;
		laser->m_beams[up_index+i-1]=beam;
	}
	laser->updateBeamsLookup();
	cerr << "beams " << beam_no << endl;
	cerr << "maxrange " << maxrange << endl;
}

SensorMap CarmenConfiguration::computeSensorMap() const{

	SensorMap smap;
	//odometry
	OdometrySensor* odometry=new OdometrySensor("ODOM");
	OdometrySensor* truepos=new OdometrySensor("TRUEPOS", true);
	OdometrySensor* odometry_ack=new OdometrySensor("ODOM_ACK");
	OdometrySensor* truepos_ack=new OdometrySensor("TRUEPOS_ACK", true);

	smap.insert(make_pair(odometry->getName(), odometry));
	smap.insert(make_pair(truepos->getName(), truepos));
	smap.insert(make_pair(odometry_ack->getName(), odometry_ack));
	smap.insert(make_pair(truepos_ack->getName(), truepos_ack));
	//sonars
	std::map<std::string, std::vector<std::string> >::const_iterator key;

	key=configValues.find("robot_use_sonar");
	if (key!=configValues.end() && key->second.front()=="on"){
		RangeSensor* sonar=new RangeSensor("SONAR");

		this->loadSonarParams(sonar);

		smap.insert(make_pair(sonar->getName(), sonar));
	}

	//laser
	key=configValues.find("robot_use_laser");

	if (key!=configValues.end() && key->second.front()=="on"){
		RangeSensor* flaser = new RangeSensor("FLASER");
		RangeSensor* laser1 = new RangeSensor("ROBOTLASER1");
		RangeSensor* flaser_ack = new RangeSensor("FLASER_ACK");
		RangeSensor* laser1_ack = new RangeSensor("ROBOTLASER_ACK1");

		this->loadLaserParams(flaser);
		this->loadLaserParams(laser1);
		this->loadLaserParams(flaser_ack);
		this->loadLaserParams(laser1_ack);

		smap.insert(make_pair(flaser->getName(), flaser));
		smap.insert(make_pair(laser1->getName(), laser1));
		smap.insert(make_pair(flaser_ack->getName(), flaser_ack));
		smap.insert(make_pair(laser1_ack->getName(), laser1_ack));
	}

	//vertical laser
	key=configValues.find("robot_use_rear_laser");

	if (key!=configValues.end() && key->second.front()=="on"){
		RangeSensor* rlaser=new RangeSensor("RLASER");
		RangeSensor* laser2=new RangeSensor("ROBOTLASER2");
		RangeSensor* rlaser_ack=new RangeSensor("RLASER_ACK");
		RangeSensor* laser2_ack=new RangeSensor("ROBOTLASER_ACK2");

		this->loadLaserParams(rlaser, false);
		this->loadLaserParams(laser2, false);
		this->loadLaserParams(rlaser_ack, false);
		this->loadLaserParams(laser2_ack, false);

		smap.insert(make_pair(rlaser->getName(), rlaser));
		smap.insert(make_pair(laser2->getName(), laser2));
		smap.insert(make_pair(rlaser_ack->getName(), rlaser_ack));
		smap.insert(make_pair(laser2_ack->getName(), laser2_ack));
	}

	return smap;
}

void CarmenConfiguration::initLaserParams(double max_range, double fov_degrees, double res_degrees){
	this->max_range = max_range;
	this->fov_degrees = fov_degrees;
	this->res_degrees = res_degrees;
}

};

