#ifndef CARMENCONFIGURATION_H
#define CARMENCONFIGURATION_H

#include <map>
#include <string>
#include <vector>
#include <istream>
#include <sensor/sensor_base/sensor.h>
#include <sensor_range/rangesensor.h>
#include "configuration.h"

namespace GMapping {

class CarmenConfiguration: public Configuration{
	private:
		double max_range;
		double fov_degrees;
		double res_degrees;
		std::map<std::string, std::vector<std::string> > configValues;
		void loadSonarParams(GMapping::RangeSensor* sonar) const;
		void loadLaserParams(GMapping::RangeSensor* laser, bool front = true) const;
	public:
		virtual std::istream& load(std::istream& is);
		virtual SensorMap computeSensorMap() const;
		void initLaserParams(double max_range, double fov_degrees, double res_degrees);
};

};

#endif

