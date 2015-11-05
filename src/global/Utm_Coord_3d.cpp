#include "Utm_Coord_3d.h"
#include <stdio.h>

Utm_Coord_3d::Utm_Coord_3d() : 
	x(0.0), y(0.0), z(0.0), zone(0), hemisphere_north(false)  {}


Utm_Coord_3d::Utm_Coord_3d(double X, double Y, double Z, short Zone, bool hemi_n) :
	x(X), y(Y), z(Z), zone(Zone), hemisphere_north(hemi_n)  {}

std::string Utm_Coord_3d::toString() {
	char buff[100];
	if( hemisphere_north ) {
	    sprintf( buff, " n: %g e: %g h: %g zone: %d north", y, x, z, zone );
	} else {
	    sprintf( buff, " n: %g e: %g h: %g zone: %d south", y, x, z, zone );
	}
	return std::string( buff );
}
