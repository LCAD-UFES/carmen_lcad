
#ifndef __SEGMAP_SUPER_MAP_H__
#define __SEGMAP_SUPER_MAP_H__

#include <cmath>
#include <cstdlib>
#include "segmap_abstract_map.h"


// This guy should inherit from AbstractMap because it
// makes sense conceptualy, but also to avoid code replication.
template<class MapType>
class SuperMap
{
public:

	SuperMap(int n_tiles_y, int n_tiles_x, double tile_size_x_m, double tile_size_y_m,
					 double resolution)
	{
		_ny = n_tiles_y, _nx = n_tiles_x;
		_tile_size_x_m = tile_size_x_m;
		_tile_size_y_m = tile_size_y_m;
		_resolution = resolution;

		_tiles = new MapType**[_ny];

		for (int i = 0; i < _ny; i++)
		{
			_tiles[i] = new MapType*[_nx];

			for (int j = 0; j < _nx; j++)
			{
				GridMapHeader header(tile_size_y_m, tile_size_x_m, resolution,
														 i * tile_size_y_m, j * tile_size_x_m);

				_tiles[i][j] = new MapType(header);
			}
		}
	}


	double origin_from_coordinate(double coordinate, double size_m)
	{
		return floor(coordinate / size_m) * size_m;
	}


	int _ny, _nx;
	double _tile_size_x_m;
	double _tile_size_y_m;
	double _resolution;

	MapType ***_tiles;
};

#endif
