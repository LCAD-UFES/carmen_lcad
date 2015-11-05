#ifndef SIMPLE_2D_MAPPER_CORE_H_
#define SIMPLE_2D_MAPPER_CORE_H_

#include <tf.h>
#include <vector>
#include <carmen/carmen.h>

typedef struct {
	std::vector<carmen_velodyne_32_laser_shot> shots;
	bool visited;
}velodyne_shot_t;

typedef struct {
	velodyne_shot_t scan[360];
	carmen_pose_3D_t pose;
}scan_data_t;

class Mapper2D {

public:

	unsigned char* map_1d;
	unsigned char** map_2d;

	int map_size_x, map_size_y;
	int world_size_x, world_size_y;
	int offset_x, offset_y;
	float resolution;

	Mapper2D(int _world_size_x, int _world_size_y, float _resolution);
	~Mapper2D();

	void InsertScanData(scan_data_t scan);
	void GetMapSize(int* _map_size_x, int* _map_size_y) {	*_map_size_x = this->map_size_x; *_map_size_y = this->map_size_y; }
	int SaveImageMap(char* filename, unsigned char* data, int width, int height);

private:

	void AddShotReading(tf::Vector3 origin, tf::Vector3 obstacle);
	void SetupRotationalAndVerticalTables();

};

#endif /* SIMPLE_2D_MAPPER_CORE_H_ */
