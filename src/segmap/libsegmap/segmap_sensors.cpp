
#include "segmap_sensors.h"
#include "segmap_util.h"
#include <cstdlib>

static const double _velodyne_vertical_angles[32] =
{
        -30.6700000, -29.3300000, -28.0000000, -26.6700000, -25.3300000, -24.0000000, -22.6700000, -21.3300000,
        -20.0000000, -18.6700000, -17.3300000, -16.0000000, -14.6700000, -13.3300000, -12.0000000, -10.6700000,
        -9.3299999, -8.0000000, -6.6700001, -5.3299999, -4.0000000, -2.6700001, -1.3300000, 0.0000000, 1.3300000,
        2.6700001, 4.0000000, 5.3299999, 6.6700001, 8.0000000, 9.3299999, 10.6700000
};

static const int velodyne_ray_order[32] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};


LidarShot::LidarShot(int n_rays)
{
    n = n_rays;
    ranges = (double *) calloc (n, sizeof(double));
    v_angles = (double *) calloc (n, sizeof(double));
    intensities = (unsigned char *) calloc (n, sizeof(unsigned char));
    h_angle = 0;
}


LidarShot::~LidarShot()
{
    free(ranges);
    free(v_angles);
    free(intensities);
}


CarmenLidarLoader::CarmenLidarLoader(const char *cloud_path, int n_rays, unsigned char ***calibration)
{
    _n_rays = n_rays;
    _calibration = calibration;
    
    _shot = new LidarShot(_n_vert);
    _raw_ranges = (short int*) calloc (_n_vert, sizeof(short int));
    _raw_intensities = (unsigned char*) calloc (_n_vert, sizeof(unsigned char));
    
    printf("cloud path: %s\n", cloud_path);
    _fptr = safe_fopen(cloud_path, "rb");

    for (int i = 0; i < _n_vert; i++)
        _shot->v_angles[i] = degrees_to_radians(_velodyne_vertical_angles[i]);
    
    _n_readings = 0;
}


CarmenLidarLoader::~CarmenLidarLoader()
{
    delete(_shot);
    free(_raw_ranges);
    free(_raw_intensities);
    fclose(_fptr);
}


int
CarmenLidarLoader::_get_distance_index(double distance)
{	
    // Gera um indice para cada faixa de distancias. 
    // As faixas crescem de ~50% a cada intervalo. 
    // Indices de zero a 9 permitem distancias de zero a ~70 metros
	if (distance < 3.5)
		return (0);

	int distance_index = (int) ((log(distance - 3.5 + 1.0) / log(1.45)) +  0.5);
	
    if (distance_index > 9)
		return (9);

	return (distance_index);
}


LidarShot* 
CarmenLidarLoader::next()
{
    double range_floor;
    unsigned char calib_intensity;
    unsigned char raw_intensity;
    int range_index;

    fread(&(_shot->h_angle), sizeof(double), 1, _fptr);
    fread(_raw_ranges, sizeof(unsigned short), _n_vert, _fptr);
    fread(_raw_intensities, sizeof(unsigned char), _n_vert, _fptr);

    _shot->h_angle = -degrees_to_radians(_shot->h_angle);

    for (int i = 0; i < _n_vert; i++)
    {
        // reorder and convert range to meters
        _shot->ranges[i] = ((double) _raw_ranges[velodyne_ray_order[i]]) / 500.;

        // reorder and calibrate intensity
        raw_intensity = _raw_intensities[velodyne_ray_order[i]];

        //double range_floor = _shot->ranges[i] * cos(_shot->v_angles[i]);
        //range_index = _get_distance_index(range_floor);
        //calib_intensity = _calibration[i][range_index][raw_intensity];
        
        _shot->intensities[i] = raw_intensity;
    }

    _n_readings++;
    return _shot;
}


bool 
CarmenLidarLoader::done()
{
    return feof(_fptr) || (_n_readings >= _n_rays);
}


void 
CarmenLidarLoader::reset()
{
    rewind(_fptr);
}


void
load_as_pointcloud(CarmenLidarLoader *loader, PointCloud<PointXYZRGB>::Ptr cloud)
{
    int i;
    double x, y, z; 
    LidarShot *shot;
    PointXYZRGB point;

    cloud->clear();

    while (!loader->done())
    {
        shot = loader->next();

        for (i = 0; i < shot->n; i++)
        {
            spherical2cartersian(shot->v_angles[i], shot->h_angle, shot->ranges[i], &x, &y, &z);

            point.x = x;
            point.y = y;
            point.z = z;
            point.r = point.g = point.b = shot->intensities[i];

            cloud->push_back(point);
        }
    }
}
