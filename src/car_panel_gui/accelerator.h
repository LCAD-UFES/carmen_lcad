/*
 * accelerator.h
 *
 *  Created on: Mar 21, 2013
 *      Author: cayo
 */

#ifndef ACCELERATOR_H_
#define ACCELERATOR_H_

#include <math.h>
#include "utils.h"
#include "character.h"

class Accelerator
{
public:
	Accelerator();
	virtual ~Accelerator();
	void draw(void);

	virtual void update(double velocity, double time) = 0;

protected:
	static constexpr double maxRangeAcceleration = 3.0;
	double velocity;
	double time;
	double acceleration;

private:
	double lineWidth;
	double width;
	double height;
	double colorRange;
	int maxMarkers;
	double widthMarker;
	double heightMarker;

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
