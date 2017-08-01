#ifndef RDDF_CONVERT_FROM_GMAPS_TO_TXT_H
#define RDDF_CONVERT_FROM_GMAPS_TO_TXT_H

#include <kml/engine.h>
#include <kml/dom.h>

#define THRESHOLDING 20.0 //Meters
#define EARTH_RADIUS_KM 6371.0
#define EARTH_RADIUS_M (EARTH_RADIUS_KM * 1000)
#define EARTH_RADIUS_CM (EARTH_RADIUS_M * 100)

typedef std::vector<kmldom::PlacemarkPtr> placemark_vector_t;

typedef struct GPS_Position
{
	double height;
	double latitude;
	double longitude;
	struct GPS_Position *prev;
	struct GPS_Position *next;
} GPS_Position_t;

double deg2Rad(double degrees);

double haversine_distance (double latSrc, double lngSrc, double latDst, double lngDst, double sphereRadius);

void carmen_rddf_convert(char *carmen_rddf_kml, char *carmen_rddf_txt);

void carmen_rddf_play_open_kml(const char *filename, placemark_vector_t *placemark_vector);

kmldom::PlacemarkPtr create_placemark(double latitude, double longitude);

#endif