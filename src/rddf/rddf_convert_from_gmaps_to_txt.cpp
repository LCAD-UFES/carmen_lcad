#include <set>
#include <algorithm>

// GNU Scientific Library - GSL
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

// CARMEN
#include <carmen/carmen.h>
#include <carmen/carmen_gps_wrapper.h>

// RDDF
#include "rddf_messages.h"
#include "rddf_convert_from_gmaps_to_txt.h"

// KML
#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>

using namespace std;

kmldom::KmlPtr kml;
kmldom::LineStringPtr path;
kmldom::KmlFactory *factory;
kmldom::DocumentPtr document;
kmldom::PlacemarkPtr placemark;
kmldom::CoordinatesPtr waypoints;

kmlengine::KmlFilePtr kml_file;

struct cmp_gps_position
{
	inline bool operator() (const GPS_Position_t &struct1, const GPS_Position_t &struct2)
    {
        return (struct1.latitude < struct2.latitude);
    }
};

struct cmp_gps_distance_from_src
{
	inline bool operator() (const GPS_Position_t &struct1, const GPS_Position_t &struct2)
    {
        double dist1 = haversine_distance(struct1.prev->latitude, struct1.prev->longitude, struct1.latitude, struct1.longitude, EARTH_RADIUS_M);
        double dist2 = haversine_distance(struct1.prev->latitude, struct1.prev->longitude, struct2.latitude, struct2.longitude, EARTH_RADIUS_M);

        return (dist1 < dist2);
    }
};

GPS_Position_t getPositionFrom(const kmldom::PlacemarkPtr placemark)
{
	GPS_Position_t gps_position;

	gps_position.height = 0.0;
	gps_position.latitude = (placemark->has_geometry() && placemark->get_geometry()->IsA(kmldom::Type_Point)) ?
	                        kmldom::AsPoint(placemark->get_geometry())->get_coordinates()->get_coordinates_array_at(0).get_latitude():
	                        0.0;
	gps_position.longitude = (placemark->has_geometry() && placemark->get_geometry()->IsA(kmldom::Type_Point)) ?
	                        kmldom::AsPoint(placemark->get_geometry())->get_coordinates()->get_coordinates_array_at(0).get_longitude():
	                        0.0;
	gps_position.prev = NULL;
	gps_position.next = NULL;

	return gps_position;
}

kmldom::PlacemarkPtr create_placemark(double latitude, double longitude)
{
	kmldom::CoordinatesPtr newCoordinates = factory->CreateCoordinates();
	kmldom::PointPtr newPoint = factory->CreatePoint();
	kmldom::PlacemarkPtr newPlacemark = factory->CreatePlacemark();
	
	newCoordinates->add_latlng(latitude, longitude);
	newPoint->set_coordinates(newCoordinates);
	newPlacemark->set_geometry(newPoint);
	
	return newPlacemark;
}

double deg2Rad(double degrees)
{
    return degrees * (M_PI / 180.0);
}

double haversine_distance (double latSrc, double lngSrc, double latDst, double lngDst, double sphereRadius)
{	
    double dlng = deg2Rad(lngDst - lngSrc);
    double dlat = deg2Rad(latDst - latSrc);

    double a = pow(sin(dlat/2.0), 2) + cos(deg2Rad(latSrc)) * cos(deg2Rad(latDst)) * pow(sin(dlng/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = sphereRadius * c;

    return d;
}

void
SavePlacemarks(const kmldom::FeaturePtr& feature, placemark_vector_t* placemarks)
{
	unsigned int i;

	factory = kmldom::KmlFactory::GetFactory();
	
	if (kmldom::PlacemarkPtr placemark = kmldom::AsPlacemark(feature))
	{
		if(const kmldom::LineStringPtr lineString = kmldom::AsLineString(placemark->get_geometry())){
			if(const kmldom::CoordinatesPtr coordinates = kmldom::AsCoordinates(lineString->get_coordinates())){
				for(i=0; i<coordinates->get_coordinates_array_size(); i++)
				{
					kmldom::CoordinatesPtr newCoordinates = factory->CreateCoordinates();
					kmldom::PointPtr newPoint = factory->CreatePoint();
					kmldom::PlacemarkPtr newPlacemark = factory->CreatePlacemark();
					double latitude = coordinates->get_coordinates_array_at(i).get_latitude();
					double longitude = coordinates->get_coordinates_array_at(i).get_longitude();
					
					newCoordinates->add_latlng(latitude, longitude);
					newPoint->set_coordinates(newCoordinates);
					newPlacemark->set_geometry(newPoint);
					
					placemarks->push_back(newPlacemark);
		      	}
			}
		}
	}
	else if (const kmldom::ContainerPtr container = kmldom::AsContainer(feature))
	{
		for (size_t i = 0; i < container->get_feature_array_size(); ++i)
		{
			SavePlacemarks(container->get_feature_array_at(i), placemarks);
		}
	}
}

void
calculate_yaws(placemark_vector_t *placemark_vector)
{
	for (unsigned int i = 1; i < placemark_vector->size(); i++)
	{
		kmldom::ExtendedDataPtr extendeddata = factory->CreateExtendedData();
		kmldom::DataPtr data = factory->CreateData();
		kmldom::PlacemarkPtr srcPlacemark = placemark_vector->at(i-1);
		kmldom::PlacemarkPtr dstPlacemark = placemark_vector->at(i);
		char buffer[64];
		double deltaLat = 0.0, deltaLng = 0.0, yaw = 0.0;
		GPS_Position_t srcGPS, dstGPS;

		// Obter deltaLat (Lat2 - Lat1)
		// Obter deltaLng (Lng2 - Lng1)
		srcGPS = getPositionFrom(srcPlacemark);
		dstGPS = getPositionFrom(dstPlacemark);
		
		deltaLat = dstGPS.latitude - srcGPS.latitude;
		deltaLng = dstGPS.longitude - srcGPS.longitude;

		// yaw = ATan2 (DeltaY, DeltaX)
		yaw = atan2(deltaLng, deltaLat);

		//set Yaw[i-1] = yaw;
		sprintf(buffer, "%lf", carmen_normalize_theta(yaw));
		std::string theta_string = buffer;
		data->set_name("theta");
		data->set_value(theta_string);

		extendeddata->add_data(data);
		srcPlacemark->set_extendeddata(extendeddata);

		if(i+1 == placemark_vector->size())
			srcPlacemark->set_extendeddata(extendeddata); //Add the same extended data or the last placemark's theta will be unknown
	}
}

void generate_placemarks(placemark_vector_t *placemark_vector, GPS_Position_t srcGPS, double latitude[], double longitude[], unsigned int size, const gsl_interp_type *type)
{
	double xi, yi;
	std::vector<GPS_Position_t> positions;
	gsl_interp_accel *acc = gsl_interp_accel_alloc ();
	gsl_spline *spline = gsl_spline_alloc (type, size);

	gsl_spline_init (spline, latitude, longitude, size);
	
	for (xi = latitude[0]; xi < latitude[size-1]; xi += 0.00001)
	{
		GPS_Position_t auxGPS;
		yi = gsl_spline_eval (spline, xi, acc);

		//placemark_vector->push_back(create_placemark(xi, yi));
		auxGPS.latitude = xi;
		auxGPS.longitude = yi;
		auxGPS.prev = &srcGPS;

		positions.push_back(auxGPS);
	}

	std::sort(positions.begin(), positions.end(), cmp_gps_distance_from_src());

	gsl_spline_free (spline);
	gsl_interp_accel_free (acc);

	for(unsigned int i = 0; i < positions.size()-1; i++)
		placemark_vector->push_back(create_placemark(positions.at(i).latitude, positions.at(i).longitude));
}

void generate_linear_placemarks(placemark_vector_t *placemark_vector, GPS_Position_t srcGPS, double latitude[], double longitude[], unsigned int size)
{
	generate_placemarks(placemark_vector, srcGPS, latitude, longitude, size, gsl_interp_linear);
}

void generate_spline_placemarks(placemark_vector_t *placemark_vector, GPS_Position_t srcGPS, double latitude[], double longitude[], unsigned int size)
{
	generate_placemarks(placemark_vector, srcGPS, latitude, longitude, size, gsl_interp_cspline);
}

void
generate_placemarks(placemark_vector_t *placemark_vector)
{
	placemark_vector_t newPlacemark;

	for (unsigned int i = 1; i < placemark_vector->size(); ++i)
	{
		kmldom::PlacemarkPtr srcPlacemark = placemark_vector->at(i-1);
		kmldom::PlacemarkPtr dstPlacemark = placemark_vector->at(i);
		GPS_Position_t srcGPS = getPositionFrom(srcPlacemark),
		               dstGPS = getPositionFrom(dstPlacemark);
		double distance = haversine_distance(srcGPS.latitude, srcGPS.longitude, dstGPS.latitude, dstGPS.longitude, EARTH_RADIUS_M);
		
        if(distance > THRESHOLDING)
        {
        	std::vector<GPS_Position_t> positions;
        	positions.clear();

        	positions.push_back(srcGPS);
        	positions.push_back(dstGPS);

        	double *latitude  = (double*)malloc(sizeof(double)*positions.size()),
			       *longitude = (double*)malloc(sizeof(double)*positions.size());

			std::sort(positions.begin(), positions.end(), cmp_gps_position());

		    for(unsigned int k = 0; k < positions.size(); k++)
			{
				latitude [k] = positions.at(k).latitude;
				longitude[k] = positions.at(k).longitude;
			}
        	
        	generate_linear_placemarks(&newPlacemark, srcGPS, latitude, longitude, 2);
        }
        else
        {
        	std::set<GPS_Position_t,cmp_gps_position> positions;

        	positions.clear();
        	positions.insert(srcGPS);
        	positions.insert(dstGPS);
        	//Add new placemarks until sum is >=, then Generate placemarks and put in vector using "gsl_interp_cspline"
        	for (unsigned int j = i+1 ; j < placemark_vector->size(); ++j)
        	{
        		GPS_Position_t pos1 = getPositionFrom(placemark_vector->at(j-1));
        		GPS_Position_t pos2 = getPositionFrom(placemark_vector->at(j));
        		distance += haversine_distance (pos1.latitude, pos1.longitude, pos2.latitude, pos2.longitude, EARTH_RADIUS_M);
        		
        		if(distance <= THRESHOLDING)
        		{
        			positions.insert(getPositionFrom(placemark_vector->at(j)));
        		}
        		else
        		{
        			double *latitude  = (double*)malloc(sizeof(double)*positions.size()),
        			       *longitude = (double*)malloc(sizeof(double)*positions.size());

					std::set<GPS_Position_t>::iterator it;
					unsigned int k;
					for(it = positions.begin(), k = 0; it!=positions.end(); it++)
					{
						latitude [k] = (*it).latitude;
						longitude[k] = (*it).longitude;
						k++;
					}

        			if(positions.size() > 2)
        				generate_spline_placemarks(&newPlacemark, srcGPS, latitude, longitude, positions.size());
        			else
        				generate_linear_placemarks(&newPlacemark, srcGPS, latitude, longitude, positions.size());

        			i = j-1;
        			break;
        		}
        	}
        }
	}
/*
	FILE *fp = fopen("files/gps_points.out", "wt");
	for(unsigned int i = 0; i < newPlacemark.size(); i++){
		GPS_Position_t pos = getPositionFrom(newPlacemark.at(i));
		fprintf(fp, "{ lat:%lf, lng:%lf, nome:\"#%d\" }", pos.latitude, pos.longitude, i+1);
		if(i != newPlacemark.size()-1)
			fprintf(fp, ",\n");
	}
*/

	placemark_vector->swap(newPlacemark);
}

int
carmen_rddf_play_copy_kml(kmldom::PlacemarkPtr waypoint, carmen_fused_odometry_message *message, int *waypoint_annotation)
{
	int north = 0;
	double zone = 0, x = 0, y = 0, z = 0;
	double latitude = 0, longitude = 0;
	double theta = 0, timestamp = 0;
	double driver_speed = 0;
	std::string waypoint_description;

	kmldom::PointPtr point;
	kmldom::CoordinatesPtr coord;
	kmldom::ExtendedDataPtr annotations;

	kmldom::DataPtr data;

	if (waypoint->has_geometry())
	{
		if (waypoint->get_geometry()->IsA(kmldom::Type_Point))
		{
			point = kmldom::AsPoint(waypoint->get_geometry());
			coord = point->get_coordinates();

			latitude = coord->get_coordinates_array_at(0).get_latitude();
			longitude = coord->get_coordinates_array_at(0).get_longitude();

			carmen_Gdc3_Utm(&x, &y, &z, &zone, &north, latitude, longitude, 0.0);
		}
		else
		{
			return 0;
		}

		message->pose.position.x = y;
		message->pose.position.y = -x;
		message->pose.position.z = z;

		std::string speed_string = waypoint->get_name();

		// OBS: o set da velocidade esta mais abaixo!
		// message->velocity.x = speed;
		message->velocity.y = 0;
		message->velocity.z = 0;

		(*waypoint_annotation) = RDDF_ANNOTATION_TYPE_NONE;

		waypoint_description = waypoint->get_description();

		if (waypoint_description.compare("") != 0)
		{
			if (waypoint_description.compare("human intervention") == 0)
				(*waypoint_annotation) = RDDF_ANNOTATION_TYPE_HUMAN_INTERVENTION;
			else
				exit(printf("Error: Unknown description annotation: \"%s\"\n", waypoint_description.c_str()));
		}

		if (waypoint->has_extendeddata())
		{
			annotations = waypoint->get_extendeddata();

			for (unsigned int i = 0; i < annotations->get_data_array_size(); i++)
			{
				data = annotations->get_data_array_at(i);

				if (data->has_name())
				{
					if (data->get_value().size() > 0)
					{
						if (std::string("theta").compare(data->get_name()) == 0)
						{
							std::string data_string = data->get_value();
							theta = atof( data_string.c_str() );
						}
						else if (std::string("timestamp").compare(data->get_name()) == 0)
						{
							std::string data_string = data->get_value();
							timestamp = atof( data_string.c_str() );
						}
						else if (std::string("driver_velocity").compare(data->get_name()) == 0)
						{
							std::string data_string = data->get_value();
							driver_speed = atof(data_string.c_str());
						}
					}
				}
			}
		}

		message->velocity.x = driver_speed;
		message->pose.orientation.roll= 0;
		message->pose.orientation.pitch= 0;
		message->pose.orientation.yaw = theta;
		message->gps_position_at_turn_on.x= 0;
		message->gps_position_at_turn_on.y= 0;
		message->gps_position_at_turn_on.z= 0;
		message->angular_velocity.roll= 0;
		message->angular_velocity.pitch= 0;
		message->angular_velocity.yaw= 0;
		message->timestamp = timestamp;
		message->phi= 0;
	}

	return 1;
}

void
carmen_rddf_convert(char *carmen_rddf_kml, char *carmen_rddf_txt)
{
	int annotation = 0;
	carmen_fused_odometry_message message;
	placemark_vector_t placemark_vector;

	FILE *fptr = fopen(carmen_rddf_txt, "w");
	carmen_rddf_play_open_kml(carmen_rddf_kml, &placemark_vector);

	for (unsigned int i = 0; i < placemark_vector.size(); i++)
	{
		if (carmen_rddf_play_copy_kml(placemark_vector[i], &message, &annotation))
		{
			fprintf(fptr, "%lf %lf %lf %lf %lf %lf\n",
				message.pose.position.x, message.pose.position.y,
				message.pose.orientation.yaw, message.velocity.x, message.phi,
				message.timestamp);
		}
	}

	fclose(fptr);
}

void
carmen_rddf_play_open_kml(const char *filename, placemark_vector_t *placemark_vector)
{
	std::string kml;
	std::string errors;

	if (!kmlbase::File::ReadFileToString(filename, &kml))
		carmen_die("Error reading KML file.");

	kml_file = kmlengine::KmlFile::CreateFromParse(kml, &errors);

	if (!kml_file)
		carmen_die("Error parsing KML file.");

	SavePlacemarks(kmlengine::GetRootFeature(kml_file->get_root()), placemark_vector);
	generate_placemarks(placemark_vector);
	calculate_yaws(placemark_vector);
}

int
main(int argc, char **argv)
{
	setlocale(LC_ALL, "C");

	if (argc < 2)
		exit(printf("Error: Use %s <gmaps-kml> <rddf-txt>\n", argv[0]));

	char *gmaps_kml = argv[1];
	char *carmen_rddf_txt = argv[2];

	carmen_rddf_convert(gmaps_kml, carmen_rddf_txt);

	return 0;
}
