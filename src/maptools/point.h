 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef _POINT_H_
#define _POINT_H_
#include <assert.h>
#include <math.h>
#include <iostream>

template <class T>
struct point{
	carmen_inline point(){}
	carmen_inline point(T _x, T _y):x(_x),y(_y){}
	T x, y;
};

template <class T>
carmen_inline point<T> operator+(const point<T>& p1, const point<T>& p2){
	return point<T>(p1.x+p2.x, p1.y+p2.y);
}

template <class T>
carmen_inline point<T> operator - (const point<T> & p1, const point<T> & p2){
	return point<T>(p1.x-p2.x, p1.y-p2.y);
}

template <class T>
carmen_inline point<T> operator * (const point<T>& p, const T& v){
	return point<T>(p.x*v, p.y*v);
}

template <class T>
carmen_inline point<T> operator * (const T& v, const point<T>& p){
	return point<T>(p.x*v, p.y*v);
}

template <class T>
carmen_inline T operator * (const point<T>& p1, const point<T>& p2){
	return p1.x*p2.x+p1.y*p2.y;
}


template <class T, class A>
struct orientedpoint: public point<T>{
	carmen_inline orientedpoint(){}
	carmen_inline orientedpoint(const point<T>& p);
	carmen_inline orientedpoint(T x, T y, A _theta): point<T>(x,y), theta(_theta){}
	A theta;
};

template <class T, class A>
orientedpoint<T,A>::orientedpoint(const point<T>& p){
	this->x=p.x;
	this->y=p.y;
	this->theta=0.;
}


template <class T, class A>
orientedpoint<T,A> operator+(const orientedpoint<T,A>& p1, const orientedpoint<T,A>& p2){
	return orientedpoint<T,A>(p1.x+p2.x, p1.y+p2.y, p1.theta+p2.theta);
}

template <class T, class A>
orientedpoint<T,A> operator - (const orientedpoint<T,A> & p1, const orientedpoint<T,A> & p2){
	return orientedpoint<T,A>(p1.x-p2.x, p1.y-p2.y, p1.theta-p2.theta);
}

template <class T, class A>
orientedpoint<T,A> operator * (const orientedpoint<T,A>& p, const T& v){
	return orientedpoint<T,A>(p.x*v, p.y*v, p.theta*v);
}

template <class T, class A>
orientedpoint<T,A> operator * (const T& v, const orientedpoint<T,A>& p){
	return orientedpoint<T,A>(p.x*v, p.y*v, p.theta*v);
}

template <class T>
struct pointcomparator{
	bool operator ()(const point<T>& a, const point<T>& b) const {
		return a.x<b.x || (a.x==b.x && a.y<b.y);
	}	
};

typedef point<int> IntPoint;
typedef point<double> Point;
typedef orientedpoint<double, double> OrientedPoint;

#endif
