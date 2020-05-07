/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * planning.cpp
 * Copyright (C) Juan Carlos Elizondo-Leal 2010 <jelizondo@tamps.cinvestav.mx>
 * 
 * planning.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * planning.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PLANNING_H_
#define _PLANNING_H_
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <pthread.h> 

class Planning
{
public:
	Planning();
	void setMap(std::vector<std::vector<double> > _map);
	std::vector<std::vector<double> > expandObstacles(double _dist);
	std::vector<std::vector<double> > pathDR(int _x, int _y);
	std::vector<std::vector<double> > getPath();

	std::vector<std::vector<double> > getExpandedMap();
private:
	struct bp
	{
		int x;
		int y;
	};
	struct bp2
	{
		int x;
		int y;
		double dist;
	};

	std::vector<std::vector<double> > map;
	std::vector<std::vector<double> > expandedMap;
	std::vector<std::vector<bp> > p;
	std::vector<std::vector<bp2> > p2;
	std::vector<std::vector<double> > path;
	double valuePath;
	bool neighbor(int x0, int y0, int x1, int y1, bool first);
	bool BHM(int x0, int y0, int x1, int y1);
	bool obtacleInLine(int x0, int y0, int x1, int y1);
	int INVERSE;    // 0 : false        1 : true
	int SIGN;
};

#endif // _PLANNING_H_
