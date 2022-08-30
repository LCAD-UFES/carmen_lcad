/*
 * accelerator.cpp
 *
 *  Created on: Mar 21, 2013
 *      Author: cayo
 */

#include <GL/glew.h>
#include "accelerator.h"

Accelerator::Accelerator()
{
	lineWidth = 2.0;
	width = 8.0;
//	height = 80.0;
	height = 40.0;
	colorRange = 0.0322580645;
//	maxMarkers = 43.0;
	maxMarkers = 21.0;
	widthMarker = 8.0;
	heightMarker = 1.0;
//	heightMarker = 0.5;

	colorG = 1.0;
	colorR = 0.0;
	velocity = 0.0;
	time = 0.0;
	acceleration = 0.0;
	floatPointAcceleration = 0;
	hundredAcceleration = 0;
	dozenAcceleration = 0;
	unitAcceleration = 0;
	character = new Character();
	buildColor();
}


Accelerator::~Accelerator()
{
	delete character;
}


void
Accelerator::draw(void)
{
	Utils::getDigitsFromValue(acceleration, &hundredAcceleration, &dozenAcceleration, &unitAcceleration, &floatPointAcceleration);

	glLineWidth(lineWidth);

	glTranslatef(-12, 60, 0);
	character->draw(fabs(dozenAcceleration), polygonsDozen, 8, color);
	glTranslatef(12, -60, 0);

	glTranslatef(2, 60, 0);
	character->draw(fabs(unitAcceleration), polygonsUnit, 8, color);
	glTranslatef(-2, -60, 0);

	glTranslatef(6, 52, 0);
	character->drawSinglePoint(color);
	glTranslatef(-6, -52, 0);

	glTranslatef(20, 60, 0);
	character->draw(fabs(floatPointAcceleration), polygonsFloatPoint, 8, color);
	glTranslatef(-20, -60, 0);

	glBegin(GL_LINE_LOOP);
	{
		glColor3f(0.0, 0.0, 1.0);

		glVertex2f(-width, height);
		glVertex2f(-width, -height);
		glVertex2f(width, -height);
		glVertex2f(width, height);
	}
	glEnd();

	glBegin(GL_QUADS);
	{
		glColor3f(colorR, colorG, 0.0);

		glVertex2f(-widthMarker, heightMarker);
		glVertex2f(-widthMarker, -heightMarker);
		glVertex2f(widthMarker, -heightMarker);
		glVertex2f(widthMarker, heightMarker);
	}
	glEnd();

	drawInstensity();
}


void
Accelerator::drawInstensity(void)
{
	float markers = round((maxMarkers * fabs(acceleration / maxRangeAcceleration)) > maxMarkers) ? maxMarkers : maxMarkers * fabs(acceleration / maxRangeAcceleration);
	float sign = (acceleration >= 0.0) ? 1.0 : -1.0;

	for (float i = 1.0; i <= fabs(markers); i += 1.0)
	{
		glBegin(GL_QUADS);
		{
			glColor3f(colorR + (i * colorRange), colorG - (i * colorRange), 0.0);

			glVertex2f(-widthMarker, (sign) * (((i + 1.0) * heightMarker) + (i * heightMarker)));
			glVertex2f(-widthMarker, (sign) * ((i * heightMarker) + (i * heightMarker)));
			glVertex2f(widthMarker, (sign) * ((i * heightMarker) + (i * heightMarker)));
			glVertex2f(widthMarker, (sign) * (((i + 1.0) * heightMarker) + (i * heightMarker)));
		}
		glEnd();
	}
}


void
Accelerator::buildColor(void)
{
	color[0] = character->buildColor(1.0f, 0.0f, 0.0f);
	color[1] = character->buildColor(1.0f, 0.0f, 0.0f);
	color[2] = character->buildColor(0.0f, 1.0f, 0.0f);
	color[3] = character->buildColor(0.0f, 0.0f, 1.0f);
	color[4] = character->buildColor(0.0f, 0.0f, 1.0f);
	color[5] = character->buildColor(0.0f, 1.0f, 1.0f);
}
