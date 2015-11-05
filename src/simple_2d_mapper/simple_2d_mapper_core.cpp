#include "simple_2d_mapper_core.h"

float sin_rot_table[36000];
float cos_rot_table[36000];
float sin_vert_table[32];
float cos_vert_table[32];

double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0,
        -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001, -13.33, 8.0,
        -12.0, 9.3299999, -10.67, 10.67 };

Mapper2D::Mapper2D(int _world_size_x, int _world_size_y, float _resolution)
{
	this->world_size_x = _world_size_x;
	this->world_size_y = _world_size_y;
	this->resolution   = _resolution;

	this->map_size_x 	 = this->world_size_x / resolution + 1;
	this->map_size_y 	 = this->world_size_y / resolution + 1;

	this->offset_x = this->map_size_x / 2;
	this->offset_y = this->map_size_y / 2;

	map_1d = (unsigned char*) malloc ((this->map_size_x * this->map_size_y) * sizeof(unsigned char));
	map_2d = (unsigned char**) malloc ((this->map_size_y) * sizeof(unsigned char*));

	for(int i = 0; i < this->map_size_x; i++)
	{
		map_2d[i] = map_1d + (i * this->map_size_x);
		memset(map_2d[i], 255, (this->map_size_y));
	}

	SetupRotationalAndVerticalTables();
}

Mapper2D::~Mapper2D()
{
	if(map_1d != NULL)
	{
		free(map_1d);
	}
}

void Mapper2D::InsertScanData(scan_data_t velodyne_scan)
{
	int shot_size = 0;

	float xy_distance, distance;
	float cos_vert_angle, sin_vert_angle, cos_rot_angle, sin_rot_angle;
	carmen_vector_3D_t vpoint;

	for(int p = 0; p < 360; p++)
	{
		for(unsigned int i = 0; i < velodyne_scan.scan[p].shots.size(); i++)
		{
			for(int j = 0; j < 32; j++)
			{
				distance = 0.2 * velodyne_scan.scan[p].shots[i].distance[j];

				if(distance > 50.0 && distance < 3000.0)
				{
					cos_vert_angle = cos_vert_table[j];
					sin_vert_angle = sin_vert_table[j];
					cos_rot_angle = cos_rot_table[(int)(velodyne_scan.scan[p].shots[i].angle * 100.0)];
					sin_rot_angle = sin_rot_table[(int)(velodyne_scan.scan[p].shots[i].angle * 100.0)];

					xy_distance = distance * cos_vert_angle;

					vpoint.x = (xy_distance * sin_rot_angle) / 100.0;
					vpoint.y = (xy_distance * cos_rot_angle) / 100.0;
					vpoint.z = (distance * sin_vert_angle) / 100.0;

					tf::Transform w2v;

					w2v.setOrigin(tf::Vector3(velodyne_scan.pose.position.x, velodyne_scan.pose.position.y, velodyne_scan.pose.position.z));
					w2v.setRotation(tf::Quaternion(velodyne_scan.pose.orientation.yaw, velodyne_scan.pose.orientation.pitch, velodyne_scan.pose.orientation.roll));

					tf::Vector3 origin = w2v * tf::Vector3(0.0, 0.0, 0.0);
					tf::Vector3 obstacle = w2v * tf::Vector3(vpoint.x, vpoint.y, vpoint.z);

					AddShotReading(origin, obstacle);
				}
			}
		}
		shot_size += velodyne_scan.scan[p].shots.size();

	}
}

void Mapper2D::AddShotReading(tf::Vector3 origin, tf::Vector3 obstacle)
{
	origin 	 /= this->resolution;
	obstacle /= this->resolution;

	if(obstacle.getZ() >= -1.8 && obstacle.getZ() <= 0.2)
	{
		int cx  = floor(obstacle.getX()) + this->offset_x;
		int cy  = floor(obstacle.getY()) + this->offset_y;

		if((cx < this->map_size_x || cx >= 0) && (cy < this->map_size_y || cy >=0))
		{
			this->map_2d[cx][cy] = 0;
		}
	}
}

void Mapper2D::SetupRotationalAndVerticalTables()
{
		// Set up cached values for sin and cos of all the possible headings
		for (unsigned short rot_index = 0; rot_index < 36000; ++rot_index)
		{
				float rotation = carmen_degrees_to_radians(0.01 * rot_index);
				cos_rot_table[rot_index] = cosf(rotation);
				sin_rot_table[rot_index] = sinf(rotation);
		}

		for (unsigned short vert_index = 0; vert_index < 32; ++vert_index)
		{
				float vert_radian = carmen_degrees_to_radians(vertical_correction[vert_index]);
				cos_vert_table[vert_index] = cosf(vert_radian);
				sin_vert_table[vert_index] = sinf(vert_radian);
		}
}

int Mapper2D::SaveImageMap( char* 	szFilename,
		unsigned char* data, int width, int	height )
{
	FILE* stream;
	stream = fopen( szFilename, "wb" );
	if( stream == NULL)
	{
		perror( "Can't open image file" );
		return 1;
	}

	fprintf( stream, "P5\n%u %u 255\n", width, height );
	fwrite( data, width, height, stream );
	fclose( stream );
	return 0;
}


