
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "segmap_sensors.h"
#include "segmap_util.h"
#include <cstdlib>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace pcl;


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
	ranges = (double *) calloc(n, sizeof(double));
	v_angles = (double *) calloc(n, sizeof(double));
	intensities = (unsigned char *) calloc(n, sizeof(unsigned char));
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

	// auxiliary vectors for reading data from file.
	_raw_ranges = (unsigned short int*) calloc(_n_vert, sizeof(unsigned short int));
	_raw_intensities = (unsigned char*) calloc(_n_vert, sizeof(unsigned char));

	_fptr = safe_fopen(cloud_path, "rb");

	for (int i = 0; i < _n_vert; i++)
		_shot->v_angles[i] = normalize_theta(degrees_to_radians(_velodyne_vertical_angles[i]));

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
	fread(&(_shot->h_angle), sizeof(double), 1, _fptr);
	fread(_raw_ranges, sizeof(unsigned short), _n_vert, _fptr);
	fread(_raw_intensities, sizeof(unsigned char), _n_vert, _fptr);

	_shot->h_angle = -normalize_theta(degrees_to_radians(_shot->h_angle));

	for (int i = 0; i < 32; i++)
	{
		// reorder and convert range to meters
		_shot->ranges[i] = ((double) _raw_ranges[velodyne_ray_order[i]]) / (double) 500.;
		_shot->intensities[i] = _raw_intensities[velodyne_ray_order[i]];
	}

	_n_readings++;
	return _shot;
}


bool 
CarmenLidarLoader::done()
{
	return (_n_readings >= _n_rays);
}


void 
CarmenLidarLoader::reset()
{
	rewind(_fptr);
}



SemanticSegmentationLoader::SemanticSegmentationLoader(const char *log_path, const char *data_path)
{
	vector<string> log_path_splitted = string_split(log_path, "/");
	string log_name = log_path_splitted[log_path_splitted.size() - 1];
	_log_data_dir = data_path + string("/data_") + log_name + string("/semantic");
}


SemanticSegmentationLoader::~SemanticSegmentationLoader()
{
}


cv::Mat
SemanticSegmentationLoader::load(DataSample *sample)
{
	// static to prevent reallocation.
	static char seg_img_path[256];

	sprintf(seg_img_path, "%s/%lf-r.png", _log_data_dir.c_str(), sample->image_time);

	// just to make sure the file exists.
	FILE *f = safe_fopen(seg_img_path, "rb");
	fclose(f);

	return cv::imread(seg_img_path);
}


void
load_as_pointcloud(CarmenLidarLoader *loader, PointCloud<PointXYZRGB>::Ptr cloud)
{
	int i;
	double x, y, z;
	LidarShot *shot;

	cloud->clear();

	while (!loader->done())
	{
		shot = loader->next();

		for (i = 0; i < shot->n; i++)
		{
			spherical2cartersian(shot->v_angles[i], shot->h_angle, shot->ranges[i], &x, &y, &z);

			PointXYZRGB point;

			point.x = (float) x;
			point.y = (float) y;
			point.z = (float) z;
			point.r = point.g = point.b = (unsigned char) shot->intensities[i];

			cloud->push_back(point);
		}
	}
}


void 
get_pixel_position(double x, double y, double z, Matrix<double, 4, 4> &lidar2cam,
                   Matrix<double, 3, 4> &projection, cv::Mat &img, cv::Point *ppixel, int *is_valid)
{
	static Matrix<double, 4, 1> plidar, pcam;
	static Matrix<double, 3, 1> ppixelh;

	*is_valid = 0;
	plidar << x, y, z, 1.;
	pcam = lidar2cam * plidar;

	if (pcam(0, 0) / pcam(3, 0) > 0)
	{
		ppixelh = projection * pcam;

		ppixel->y = (ppixelh(1, 0) / ppixelh(2, 0)) * img.rows;
		ppixel->x = (ppixelh(0, 0) / ppixelh(2, 0)) * img.cols;

		if (ppixel->x >= 0 && ppixel->x < img.cols && ppixel->y >= 0 && ppixel->y < img.rows)
			*is_valid = 1;
	}
}


cv::Mat 
load_image(DataSample *sample)
{
	static int image_size = 0.;
	static unsigned char *raw_right = NULL;
	static Mat img_r;

	int sample_image_size = sample->image_height * sample->image_width * 3;

	// realloc if necessary
	if (sample_image_size != image_size)
	{
		image_size = sample_image_size;

		if (raw_right != NULL) free(raw_right);

		raw_right = (unsigned char*) calloc (image_size, sizeof(unsigned char));
		img_r = Mat(sample->image_height, sample->image_width, CV_8UC3, raw_right, 0);
	}

	FILE *image_file = safe_fopen(sample->image_path.c_str(), "rb");
	// jump the left image
	fseek(image_file, image_size * sizeof(unsigned char), SEEK_SET);
	fread(raw_right, image_size, sizeof(unsigned char), image_file);
	fclose(image_file);
	// carmen images are stored as rgb
	cvtColor(img_r, img_r, COLOR_RGB2BGR);

	return img_r;
}

