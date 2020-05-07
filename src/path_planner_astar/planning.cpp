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

#include "planning.hpp"
#include <float.h>
#define d1 (double) 1
#define d2 (double) sqrt(2)
#define d3 10
#define d4 (float) sqrt(2)*10

#define OCCUPIED_CELL (50000)


using namespace std;
Planning::Planning()
{
	this->valuePath = 0.0;
	this->INVERSE = 0;
	this->SIGN = 0;
}

void Planning::setMap(std::vector<std::vector<double> > _map)
{
	this->map = _map;
	this->path = _map;
}


std::vector<std::vector<double> > Planning::expandObstacles(double _dist)
{
	std::vector<std::vector<double> > tmpMap;
	expandedMap = map;
	tmpMap = map;
	this->p.resize(map.size());
	for (int x = 0; x < p.size(); x++)
	{
		this->p[x].resize(map[0].size());
	}

	// initialize map
	for (int y = 0; y < tmpMap[0].size(); y++)
		for (int x = 0; x < tmpMap.size(); x++)
		{
			tmpMap[x][y] = 2555;
			this->p[x][y].x = -1;
			this->p[x][y].y = -1;
		}
	//tmpMap[88][88]=0;p[88][88].x=88; p[88][88].y=88;
	// initialize borders
	for (int y = 0; y < tmpMap[0].size(); y++)
		for (int x = 0; x < tmpMap.size(); x++)
		{
			//if(map[x][y]<0){
			if (x - 1 >= 0)
				if ((map[x - 1][y] <= 0) && (map[x][y] > 0))
				{
					tmpMap[x][y] = 0;
					this->p[x][y].x = x;
					this->p[x][y].y = y;
				}
			if (x + 1 < tmpMap.size())
				if ((map[x + 1][y] <= 0) && (map[x][y] > 0))
				{
					tmpMap[x][y] = 0;
					this->p[x][y].x = x;
					this->p[x][y].y = y;
				}
			if (y - 1 >= 0)
				if ((map[x][y - 1] <= 0) && (map[x][y] > 0))
				{
					tmpMap[x][y] = 0;
					this->p[x][y].x = x;
					this->p[x][y].y = y;
				}
			if ((map[x][y + 1] <= 0) && (map[x][y] > 0))
			{
				tmpMap[x][y] = 0;
				this->p[x][y].x = x;
				this->p[x][y].y = y;
			}
			//}
		}

	//perform the first pass
	for (int y = 0; y < tmpMap[0].size(); y++)
		for (int x = 0; x < tmpMap.size(); x++)
		{
			if (map[x][y] <= 0 || tmpMap[x][y] == 0)
			{
				if (x - 1 >= 0 && y - 1 >= 0)
					if (tmpMap[x - 1][y - 1] + d2 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x - 1][y - 1];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
				if (y - 1 >= 0)
					if (tmpMap[x][y - 1] + d1 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x][y - 1];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
				if (x + 1 < tmpMap.size() && y - 1 >= 0)
					if (tmpMap[x + 1][y - 1] + d2 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x + 1][y - 1];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
				if (x - 1 >= 0)
					if (tmpMap[x - 1][y] + d1 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x - 1][y];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
			}
		}

	//perform the final pass
	for (int y = tmpMap[0].size() - 1; y >= 0; y--)
		for (int x = tmpMap.size() - 1; x >= 0; x--)
		{
			if (map[x][y] <= 0 || tmpMap[x][y] == 0)
			{
				if (x + 1 < tmpMap.size())
					if (tmpMap[x + 1][y] + d1 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x + 1][y];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
				if (x - 1 >= 0 && y + 1 < tmpMap[0].size())
					if (tmpMap[x - 1][y + 1] + d2 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x - 1][y + 1];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
				if (y + 1 < tmpMap[0].size())
					if (tmpMap[x][y + 1] + d1 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x][y + 1];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
				if (x + 1 < tmpMap.size() && y + 1 < tmpMap[0].size())
					if (tmpMap[x + 1][y + 1] + d2 < tmpMap[x][y])
					{
						this->p[x][y] = this->p[x + 1][y + 1];
						tmpMap[x][y] = sqrt((x - this->p[x][y].x) * (x - this->p[x][y].x) + (y - this->p[x][y].y) * (y - this->p[x][y].y));
					}
			}
		}
	for (int y = 0; y < tmpMap[0].size(); y++)
		for (int x = 0; x < tmpMap.size(); x++)
		{
			if (tmpMap[x][y] <= _dist || this->map[x][y] > 0)
				this->expandedMap[x][y] = 1;
			else if (this->map[x][y] == 0)
				this->expandedMap[x][y] = 0;
			else
				this->expandedMap[x][y] = -1;
		}
	return this->expandedMap;
}

std::vector<std::vector<double> > Planning::pathDR(int _x, int _y)
{
	std::vector<std::vector<double> > dist(this->path.size(), std::vector<double>(this->path[0].size()));
	std::vector<std::vector<int> > cambios(this->path.size(), std::vector<int>(this->path[0].size()));
	int count = 1;
	int ciclos = 0;
	clock_t t_ini, t_fin;

	this->p2.resize(map.size());

	for (int x = 0; x < p2.size(); x++)
	{
		this->p2[x].resize(map[0].size());
	}

	// initialize map
	for (int y = 0; y < this->path[0].size(); y++)
		for (int x = 0; x < this->path.size(); x++)
		{
			this->path[x][y] = OCCUPIED_CELL;
			this->p2[x][y].x = -1;
			this->p2[x][y].y = -1;
			this->p2[x][y].dist = 99955555.0;
			cambios[x][y] = 0;
			dist[x][y] = (double) sqrt(x * x + y * y);
		}

	this->p2[_x][_y].x = _x;
	this->p2[_x][_y].y = _y;
	this->p2[_x][_y].dist = 0;
	this->path[_x][_y] = 0;

// 	this->p2[1500][2000].x=1500; this->p2[1500][2000].y=2000;
// 	this->p2[1500][2000].dist=0;
// 	this->path[1500][2000]=0;
// 	std::cout<<"Inicia " <<std::endl;
	t_ini = clock();

	while (count != 0)
	{
		count = 0;
		//perform the first pass
		for (int y = 1; y < this->path[0].size() - 1; y++)
		{
			for (int x = 1; x < this->path.size() - 1; x++)
			{
				if (this->expandedMap[x][y] < 0)
				{
					//if(y-1>=0){
					//if(x-1>=0)
					if (this->p2[x - 1][y - 1].x > -1)
						if (dist[abs(x - this->p2[x - 1][y - 1].x)][abs(y - this->p2[x - 1][y - 1].y)] + this->p2[x - 1][y - 1].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x-1][y-1].x)*(x-this->p2[x-1][y-1].x) + (y-this->p2[x-1][y-1].y)*(y-this->p2[x-1][y-1].y) ) +this->p2[x-1][y-1].dist < this->path[x][y]){
							if (this->neighbor(this->p2[x - 1][y - 1].x, this->p2[x - 1][y - 1].y, x, y, true))
							{
								this->p2[x][y] = this->p2[x - 1][y - 1];
								this->path[x][y] = dist[abs(x - this->p2[x - 1][y - 1].x)][abs(y - this->p2[x - 1][y - 1].y)] + this->p2[x - 1][y - 1].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x - 1][y - 1] + d2 < this->path[x][y])
							{					//std::cout << "false1 "<< x << " " << y << " "<<  x-1<< " " <<y-1<< std::endl;
								this->path[x][y] = this->path[x - 1][y - 1] + d2;
								this->p2[x][y].x = x - 1;
								this->p2[x][y].y = y - 1;
								this->p2[x][y].dist = this->path[x - 1][y - 1];
								count++;
								cambios[x][y]++;
							}
						}

					if (this->p2[x][y - 1].x > -1)
						if (dist[abs(x - this->p2[x][y - 1].x)][abs(y - this->p2[x][y - 1].y)] + this->p2[x][y - 1].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x][y-1].x)*(x-this->p2[x][y-1].x) + (y-this->p2[x][y-1].y)*(y-this->p2[x][y-1].y) ) +this->p2[x][y-1].dist < this->path[x][y]){
							if (this->neighbor(this->p2[x][y - 1].x, this->p2[x][y - 1].y, x, y, true))
							{
								this->p2[x][y] = this->p2[x][y - 1];
								this->path[x][y] = dist[abs(x - this->p2[x][y - 1].x)][abs(y - this->p2[x][y - 1].y)] + this->p2[x][y - 1].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x][y - 1] + d1 < this->path[x][y])
							{					//std::cout << "false2 "<< x << " " << y << " "<<  x<< " " <<y-1<< std::endl;
								this->path[x][y] = this->path[x][y - 1] + d1;
								this->p2[x][y].x = x;
								this->p2[x][y].y = y - 1;
								this->p2[x][y].dist = this->path[x][y - 1];
								count++;
								cambios[x][y]++;
							}
						}
					//if(x+1<this->path.size())
					if (this->p2[x + 1][y - 1].x > -1)
						if (dist[abs(x - this->p2[x + 1][y - 1].x)][abs(y - this->p2[x + 1][y - 1].y)] + this->p2[x + 1][y - 1].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x+1][y-1].x)*(x-this->p2[x+1][y-1].x) + (y-this->p2[x+1][y-1].y)*(y-this->p2[x+1][y-1].y) ) +this->p2[x+1][y-1].dist < this->path[x][y]){
							if (this->neighbor(this->p2[x + 1][y - 1].x, this->p2[x + 1][y - 1].y, x, y, true))
							{
								this->p2[x][y] = this->p2[x + 1][y - 1];
								this->path[x][y] = dist[abs(x - this->p2[x + 1][y - 1].x)][abs(y - this->p2[x + 1][y - 1].y)] + this->p2[x + 1][y - 1].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x + 1][y - 1] + d2 < this->path[x][y])
							{					//std::cout << "false3 "<< x << " " << y << " "<<  x+1<< " " <<y-1<< std::endl;
								this->path[x][y] = this->path[x + 1][y - 1] + d2;
								this->p2[x][y].x = x + 1;
								this->p2[x][y].y = y - 1;
								this->p2[x][y].dist = this->path[x + 1][y - 1];
								count++;
								cambios[x][y]++;
							}
						}
					//}
					//if(x-1>=0)
					if (this->p2[x - 1][y].x > -1)
						if (dist[abs(x - this->p2[x - 1][y].x)][abs(y - this->p2[x - 1][y].y)] + this->p2[x - 1][y].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x-1][y].x)*(x-this->p2[x-1][y].x) + (y-this->p2[x-1][y].y)*(y-this->p2[x-1][y].y) ) +this->p2[x-1][y].dist< this->path[x][y]){
							if (this->neighbor(this->p2[x - 1][y].x, this->p2[x - 1][y].y, x, y, true))
							{
								this->p2[x][y] = this->p2[x - 1][y];
								this->path[x][y] = dist[abs(x - this->p2[x - 1][y].x)][abs(y - this->p2[x - 1][y].y)] + this->p2[x - 1][y].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x - 1][y] + d1 < this->path[x][y])
							{					//std::cout << "false4 "<< x << " " << y << " "<<  x-1<< " " <<y<< std::endl;
								this->path[x][y] = this->path[x - 1][y] + d1;
								this->p2[x][y].x = x - 1;
								this->p2[x][y].y = y;
								this->p2[x][y].dist = this->path[x - 1][y];
								count++;
								cambios[x][y]++;
							}
						}

				}
			}
			for (int x = this->path.size() - 2; x > 0; x--)
			{
				if (this->expandedMap[x][y] < 0)
				{
					//if(x+1<this->path.size())
					if (this->p2[x + 1][y].x > -1)
						if (dist[abs(x - this->p2[x + 1][y].x)][abs(y - this->p2[x + 1][y].y)] + this->p2[x + 1][y].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x+1][y-1].x)*(x-this->p2[x+1][y-1].x) + (y-this->p2[x+1][y-1].y)*(y-this->p2[x+1][y-1].y) ) +this->p2[x+1][y-1].dist < this->path[x][y]){
							if (this->neighbor(this->p2[x + 1][y].x, this->p2[x + 1][y].y, x, y, true))
							{
								this->p2[x][y] = this->p2[x + 1][y];
								this->path[x][y] = dist[abs(x - this->p2[x + 1][y].x)][abs(y - this->p2[x + 1][y].y)] + this->p2[x + 1][y].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x + 1][y] + d1 < this->path[x][y])
							{					//std::cout << "false3 "<< x << " " << y << " "<<  x+1<< " " <<y-1<< std::endl;
								this->path[x][y] = this->path[x + 1][y] + d1;
								this->p2[x][y].x = x + 1;
								this->p2[x][y].y = y;
								this->p2[x][y].dist = this->path[x + 1][y];
								count++;
								cambios[x][y]++;
							}
						}
				}
			}
		}

		//perform the final pass
		for (int y = this->path[0].size() - 2; y > 0; y--)
		{
			for (int x = this->path.size() - 2; x > 0; x--)
			{
				if (this->expandedMap[x][y] < 0)
				{
					//if(x+1<this->path.size())
					if (this->p2[x + 1][y].x > -1)
						if (dist[abs(x - this->p2[x + 1][y].x)][abs(y - this->p2[x + 1][y].y)] + this->p2[x + 1][y].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x+1][y].x)*(x-this->p2[x+1][y].x) + (y-this->p2[x+1][y].y)*(y-this->p2[x+1][y].y) ) +this->p2[x+1][y].dist< this->path[x][y]){
							if (this->neighbor(this->p2[x + 1][y].x, this->p2[x + 1][y].y, x, y, false))
							{
								this->p2[x][y] = this->p2[x + 1][y];
								this->path[x][y] = dist[abs(x - this->p2[x + 1][y].x)][abs(y - this->p2[x + 1][y].y)] + this->p2[x + 1][y].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x + 1][y] + d1 < this->path[x][y])
							{					//std::cout << "false5 "<< x << " " << y << " "<< x+1<< " " <<y<< std::endl;
								this->path[x][y] = this->path[x + 1][y] + d1;
								this->p2[x][y].x = x + 1;
								this->p2[x][y].y = y;
								this->p2[x][y].dist = this->path[x + 1][y];
								count++;
								cambios[x][y]++;
							}
						}
					//if( y+1<this->path[0].size()){
					//if(x-1>=0 )
					if (this->p2[x - 1][y + 1].x > -1)
						if (dist[abs(x - this->p2[x - 1][y + 1].x)][abs(y - this->p2[x - 1][y + 1].y)] + this->p2[x - 1][y + 1].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x-1][y+1].x)*(x-this->p2[x-1][y+1].x) + (y-this->p2[x-1][y+1].y)*(y-this->p2[x-1][y+1].y) ) +this->p2[x-1][y+1].dist< this->path[x][y]){
							if (this->neighbor(this->p2[x - 1][y + 1].x, this->p2[x - 1][y + 1].y, x, y, false))
							{
								this->p2[x][y] = this->p2[x - 1][y + 1];
								this->path[x][y] = dist[abs(x - this->p2[x - 1][y + 1].x)][abs(y - this->p2[x - 1][y + 1].y)] + this->p2[x - 1][y + 1].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x - 1][y + 1] + d2 < this->path[x][y])
							{					//std::cout << "false6 "<< x << " " << y << " "<<  x-1<< " " <<y+1<< std::endl;
								this->path[x][y] = this->path[x - 1][y + 1] + d2;
								this->p2[x][y].x = x - 1;
								this->p2[x][y].y = y + 1;
								this->p2[x][y].dist = this->path[x - 1][y + 1];
								count++;
								cambios[x][y]++;
							}
						}

					if (this->p2[x][y + 1].x > -1)
						if (dist[abs(x - this->p2[x][y + 1].x)][abs(y - this->p2[x][y + 1].y)] + this->p2[x][y + 1].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x][y+1].x)*(x-this->p2[x][y+1].x) + (y-this->p2[x][y+1].y)*(y-this->p2[x][y+1].y) ) +this->p2[x][y+1].dist< this->path[x][y]){
							if (this->neighbor(this->p2[x][y + 1].x, this->p2[x][y + 1].y, x, y, false))
							{
								this->p2[x][y] = this->p2[x][y + 1];
								this->path[x][y] = dist[abs(x - this->p2[x][y + 1].x)][abs(y - this->p2[x][y + 1].y)] + this->p2[x][y + 1].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x][y + 1] + d1 < this->path[x][y])
							{					//std::cout << "false7 "<< x << " " << y << " "<<  x << " " << y+1 << std::endl;
								this->path[x][y] = this->path[x][y + 1] + d1;
								this->p2[x][y].x = x;
								this->p2[x][y].y = y + 1;
								this->p2[x][y].dist = this->path[x][y + 1];
								count++;
								cambios[x][y]++;
							}
						}
					//if(x+1<this->path.size())
					if (this->p2[x + 1][y + 1].x > -1)
						if (dist[abs(x - this->p2[x + 1][y + 1].x)][abs(y - this->p2[x + 1][y + 1].y)] + this->p2[x + 1][y + 1].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x+1][y+1].x)*(x-this->p2[x+1][y+1].x) + (y-this->p2[x+1][y+1].y)*(y-this->p2[x+1][y+1].y) ) +this->p2[x+1][y+1].dist< this->path[x][y]){
							if (this->neighbor(this->p2[x + 1][y + 1].x, this->p2[x + 1][y + 1].y, x, y, false))
							{
								this->p2[x][y] = this->p2[x + 1][y + 1];
								this->path[x][y] = dist[abs(x - this->p2[x + 1][y + 1].x)][abs(y - this->p2[x + 1][y + 1].y)] + this->p2[x + 1][y + 1].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x + 1][y + 1] + d2 < this->path[x][y])
							{					//std::cout << "false8 "<< x << " " << y << " "<<  x+1 << " " << y+1 << std::endl;
								this->path[x][y] = this->path[x + 1][y + 1] + d2;
								this->p2[x][y].x = x + 1;
								this->p2[x][y].y = y + 1;
								this->p2[x][y].dist = this->path[x + 1][y + 1];
								count++;
								cambios[x][y]++;
							}
						}
					//}
				}
			}
			for (int x = 1; x < this->path.size() - 1; x++)
			{
				if (this->expandedMap[x][y] < 0)
				{
					//if(x-1>=0)
					if (this->p2[x - 1][y].x > -1)
						if (dist[abs(x - this->p2[x - 1][y].x)][abs(y - this->p2[x - 1][y].y)] + this->p2[x - 1][y].dist < this->path[x][y])
						{//sqrt( (x-this->p2[x-1][y].x)*(x-this->p2[x-1][y].x) + (y-this->p2[x-1][y].y)*(y-this->p2[x-1][y].y) ) +this->p2[x-1][y].dist< this->path[x][y]){
							if (this->neighbor(this->p2[x - 1][y].x, this->p2[x - 1][y].y, x, y, false))
							{
								this->p2[x][y] = this->p2[x - 1][y];
								this->path[x][y] = dist[abs(x - this->p2[x - 1][y].x)][abs(y - this->p2[x - 1][y].y)] + this->p2[x - 1][y].dist;//sqrt( (x-this->p2[x][y].x)*(x-this->p2[x][y].x) + (y-this->p2[x][y].y)*(y-this->p2[x][y].y) ) +this->p2[x][y].dist;
								count++;
								cambios[x][y]++;
							}
							else if (this->path[x - 1][y] + d1 < this->path[x][y])
							{					//std::cout << "false4 "<< x << " " << y << " "<<  x-1<< " " <<y<< std::endl;
								this->path[x][y] = this->path[x - 1][y] + d1;
								this->p2[x][y].x = x - 1;
								this->p2[x][y].y = y;
								this->p2[x][y].dist = this->path[x - 1][y];
								count++;
								cambios[x][y]++;
							}
						}

				}

			}
		}
		ciclos++;
// 		std::cout<<"ciclo"<<std::endl;

	}
	t_fin = clock();
	double secs = (t_fin - t_ini) / (double) CLOCKS_PER_SEC;
// 	printf("%f \n", secs);

	//this->path=this->path;
	int _max = 0, _i, _j;
	for (int y = cambios[0].size() - 1; y >= 0; y--)
		for (int x = cambios.size() - 1; x >= 0; x--)
		{
			if (cambios[x][y] > _max)
			{
				_max = cambios[x][y];
				_i = x;
				_j = y;
			}
		}
// 	std::cout<<ciclos<<std::endl;
// 	std::cout<<"cambios "<< _max <<" " << _i << " " << _j<<std::endl;
	return this->path;
}

bool Planning::neighbor(int x0, int y0, int x1, int y1, bool first)
{
	//std::cout<< "in neigh"<< std::endl;

	if (x1 == x0)
	{
		//if(y1-1>0){
		if (this->p2[x1][y1 - 1].x == x0 && this->p2[x1][y1 - 1].y == y0)
			return true;
		//}
		//if( y1+1<this->path[0].size()){
		if (this->p2[x1][y1 + 1].x == x0 && this->p2[x1][y1 + 1].y == y0)
			return true;
		else
			return false;
		//}
		//else return false;
	}
	if (y1 == y0)
	{
		//if(x1-1>0){
		if (this->p2[x1 - 1][y1].x == x0 && this->p2[x1 - 1][y1].y == y0)
			return true;
		//}
		//if(x1+1<this->path.size()){
		if (this->p2[x1 + 1][y1].x == x0 && this->p2[x1 + 1][y1].y == y0)
			return true;
		else
			return false;
		//} return false;
	}
	if (first)
	{
		if (abs(x1 - x0) == abs(y1 - y0))
		{
			if (x1 > x0 && y1 > y0)
			{
				if (this->p2[x1 - 1][y1 - 1].x == x0 && this->p2[x1 - 1][y1 - 1].y == y0)
					return true;
				else
					return false;
			}
			if (x1 < x0 && y1 > y0)
			{
				if (this->p2[x1 + 1][y1 - 1].x == x0 && this->p2[x1 + 1][y1 - 1].y == y0)
					return true;
				else
					return false;
			}
		}

		if (x1 > x0 && y1 > y0)
		{
			if ((x1 - x0) < (y1 - y0))
			{
				if ((this->p2[x1 - 1][y1 - 1].x == x0 && this->p2[x1 - 1][y1 - 1].y == y0) && (this->p2[x1][y1 - 1].x == x0 && this->p2[x1][y1 - 1].y == y0))
					return true;
				else
					return !this->obtacleInLine(x0, y0, x1, y1);			//false;
			}
		}

		if (x1 < x0 && y1 > y0)
		{
			if ((x0 - x1) < (y1 - y0))
			{
				if ((this->p2[x1 + 1][y1 - 1].x == x0 && this->p2[x1 + 1][y1 - 1].y == y0) && (this->p2[x1][y1 - 1].x == x0 && this->p2[x1][y1 - 1].y == y0))
					return true;
				else
					return !this->obtacleInLine(x0, y0, x1, y1);			//false;
			}
		}
	}
	else if (!first)
	{
// 		if( x1==x0 && y1<y0){
// 			if(this->p2[x1][y1+1].x == x0 && this->p2[x1][y1+1].y == y0)
// 				return true;
// 			else return false;
// 		}
// 		if( y1==y0 && x1<x0 ){
// 			if(this->p2[x1+1][y1].x == x0 && this->p2[x1+1][y1].y == y0)
// 				return true;
// 			else return false;
// 		}
		if (abs(x1 - x0) == abs(y1 - y0))
		{
			if (x1 < x0 && y1 < y0)
			{
				if (this->p2[x1 + 1][y1 + 1].x == x0 && this->p2[x1 + 1][y1 + 1].y == y0)
					return true;
				else
					return false;
			}
			if (x1 > x0 && y1 < y0)
			{
				if (this->p2[x1 - 1][y1 + 1].x == x0 && this->p2[x1 - 1][y1 + 1].y == y0)
					return true;
				else
					return false;
			}
		}
		if (x1 < x0 && y1 < y0)
		{
			if ((x0 - x1) < (y0 - y1))
			{
				if ((this->p2[x1 + 1][y1 + 1].x == x0 && this->p2[x1 + 1][y1 + 1].y == y0) && (this->p2[x1][y1 + 1].x == x0 && this->p2[x1][y1 + 1].y == y0))
					return true;
				else
					return !this->obtacleInLine(x0, y0, x1, y1);			//false;
			}
		}
		if (x1 > x0 && y1 < y0)
		{
			if ((x1 - x0) < (y0 - y1))
			{
				if ((this->p2[x1 - 1][y1 + 1].x == x0 && this->p2[x1 - 1][y1 + 1].y == y0) && (this->p2[x1][y1 + 1].x == x0 && this->p2[x1][y1 + 1].y == y0))
					return true;
				else
					return !this->obtacleInLine(x0, y0, x1, y1);			//false;
			}
		}
	}

	///Verifica (0-45 grados)
	if (x1 < x0 && y1 > y0)
	{
		if (x0 - x1 > (y1 - y0))
		{
			if ((this->p2[x1 + 1][y1 - 1].x == x0 && this->p2[x1 + 1][y1 - 1].y == y0) && (this->p2[x1 + 1][y1].x == x0 && this->p2[x1 + 1][y1].y == y0))
				return true;
			else
				return !this->obtacleInLine(x0, y0, x1, y1);			//false;
		}
	}

	///Verifica (315-360 grados)
	if (x1 < x0 && y1 < y0)
	{
		if ((x0 - x1) > (y0 - y1))
		{
			if ((this->p2[x1 + 1][y1 + 1].x == x0 && this->p2[x1 + 1][y1 + 1].y == y0) && (this->p2[x1 + 1][y1].x == x0 && this->p2[x1 + 1][y1].y == y0))
				return true;
			else
				return !this->obtacleInLine(x0, y0, x1, y1);			//false;
			//else return false;
		}
	}

	///Verifica 135-180
	if (x1 > x0 && y1 > y0)
	{
		if ((x1 - x0) > (y1 - y0))
		{
			if ((this->p2[x1 - 1][y1 - 1].x == x0 && this->p2[x1 - 1][y1 - 1].y == y0) && (this->p2[x1 - 1][y1].x == x0 && this->p2[x1 - 1][y1].y == y0))
				return true;
			else
				return !this->obtacleInLine(x0, y0, x1, y1);				//false;
		}
	}
	///Verifica (180-225 grados)
	if (x1 > x0 && y1 < y0)
	{
		if ((x1 - x0) > (y0 - y1))
		{
			if ((this->p2[x1 - 1][y1 + 1].x == x0 && this->p2[x1 - 1][y1 + 1].y == y0) && (this->p2[x1 - 1][y1].x == x0 && this->p2[x1 - 1][y1].y == y0))
				return true;
			else
				return !this->obtacleInLine(x0, y0, x1, y1);				//false;

		}
	}
	return false;
}

std::vector<std::vector<double> > Planning::getPath()
{
	return this->path;
}

std::vector<std::vector<double> > Planning::getExpandedMap()
{
	return this->expandedMap;
}

bool Planning::obtacleInLine(int x0, int y0, int x1, int y1)
{
	INVERSE = 0;
	SIGN = 0;
	double m = 0;
	int t;
	int infinity = (x1 == x0);
	if (!infinity)
		m = (y1 - y0) / (double) (x1 - x0);
	if (m < 0)
		SIGN = 1;
	if (infinity || fabs(m) > 1)
	{
		INVERSE = 1;
		if (SIGN)
		{
			t = -x0;
			x0 = -y0;
			y0 = t;
			t = -x1;
			x1 = -y1;
			y1 = t;
		}
		else
		{
			t = x0;
			x0 = y0;
			y0 = t;
			t = x1;
			x1 = y1;
			y1 = t;
		}
	}
	if (x1 < x0)
		return BHM(x1, y1, x0, y0);
	else
		return BHM(x0, y0, x1, y1);
}

bool Planning::BHM(int x0, int y0, int x1, int y1)
{
	int twoDY = 2 * (y1 - y0), twoDX = 2 * (x1 - x0), DX = x1 - x0;
	int incY = ((twoDY < 0) ? -1 : 1);
	twoDY = abs(twoDY);
	int p = twoDY - DX;
	int x = x0, y = y0;
	int *_x, *_y;
	if (INVERSE == 1)
	{
		if (SIGN == 1)
		{
			if (this->expandedMap[-y][-x] >= 0)
				return true;				//
		}
		else
		{
			_x = &y;
			_y = &x;
			if (this->expandedMap[*_x][*_y] >= 0)
				return true;
		}
	}
	else
	{
		_x = &x;
		_y = &y;
		if (this->expandedMap[*_x][*_y] >= 0)
			return true;
	}

	while (x < x1)
	{
		++x;
		if (!(p < 0))
		{
			y += incY;
			p -= twoDX;
		}
		p += twoDY;
		if (INVERSE == 1 && SIGN == 1)
		{
			if (this->expandedMap[-y][-x] >= 0)
				return true;				//
		}
		else
		{
			if (this->expandedMap[*_x][*_y] >= 0)
				return true;
		}
	}
	return false;
}
