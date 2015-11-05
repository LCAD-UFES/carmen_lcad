/*
 * accelerator.h
 *
 *  Created on: Mar 21, 2013
 *      Author: cayo
 */

#ifndef ACCELERATOR_H_
#define ACCELERATOR_H_

#include <GL/glew.h>
#include <math.h>
#include "utils/utils.h"
#include "characters/character.h"

class Accelerator
{
public:
	Accelerator();
	~Accelerator();
	void draw(void);

	virtual void update(double velocity, double time) = 0;

protected:
	static const double maxRangeAcceleration = 1.612903226;
	double velocity;
	double acceleration;
	double interval;
	double intervalAccumulation;

private:
	static const double lineWidth = 2.0;
	static const double width = 8.0;
	static const double height = 80.0;
	static const double colorRange = 0.0322580645;
	static const int maxMarkers = 31.0;
	static const double widthMarker = 8.0;
	static const double heightMarker = 1.25;

	int hundredAcceleration;
	int dozenAcceleration;
	int unitAcceleration;
	int floatPointAcceleration;
	double colorR;
	double colorG;
	Character *character;
	polygon polygonsFloatPoint[9];
	polygon polygonsUnit[9];
	polygon polygonsDozen[9];
	colors color[6];

	void drawInstensity(void);
	void buildColor(void);
};

#endif /* ACCELERATOR_H_ */
