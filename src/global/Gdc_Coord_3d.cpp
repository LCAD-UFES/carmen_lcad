#include "Gdc_Coord_3d.h"
#include <stdio.h>

Gdc_Coord_3d::Gdc_Coord_3d() : 
		longitude(0.0), latitude(0.0), elevation(0.0) {}

Gdc_Coord_3d::Gdc_Coord_3d( double lat, double lon, double e ) :
		longitude(lon), latitude(lat), elevation(e) {}

std::string Gdc_Coord_3d::toString() {
	char buff[100];
	sprintf( buff, " lat: %g long: %g h: %g", latitude, longitude, elevation );
	return std::string( buff );
}
