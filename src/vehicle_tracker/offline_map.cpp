#include "offline_map.h"

struct Index
{
	int i;

	int j;

	Index(int i, int j)
	{
		this->i = i;
		this->j = j;
	}
};

typedef std::vector<Index> Indexes;

struct Grid
{
	double range;

	g2d::Field x_origin;

	g2d::Field y_origin;

	double *complete_map;

	int x_size;

	int y_size;

	double resolution;

	std::vector<bool> clustered;

	Grid(double range, carmen_localize_ackerman_globalpos_message *globalpos, carmen_mapper_map_message *map_message):
		x_origin(map_message->config.x_origin),
		y_origin(map_message->config.y_origin),
		complete_map(map_message->complete_map),
		x_size(map_message->config.x_size),
		y_size(map_message->config.y_size),
		resolution(map_message->config.resolution),
		clustered(x_size * y_size, false)
	{
		this->range = range;
		if (globalpos != NULL)
		{
			x_origin -= globalpos->globalpos.x;
			y_origin -= globalpos->globalpos.y;
		}
	}

	void append(int i, int j, OfflineMap &map)
	{
		// Compute the obstacle point's coordinates relative to the car.
		g2d::Field x = i * resolution + x_origin;
		g2d::Field y = j * resolution + y_origin;

		g2d::Field t = atan2(y, x);
		g2d::Field d = sqrt(x*x + y*y);

		if (d > range)
			return;

		map.cartesian.points.emplace_back(x, y);
		map.polar.emplace_back(t, d);
	}

	void append(int i, int j, g2d::Points &cluster)
	{
		// Mark this point as clustered, even if it's dropped.
		clustered[j + i * y_size] = true;

		// Compute the obstacle point's coordinates relative to the car.
		g2d::Field x = i * resolution + x_origin;
		g2d::Field y = j * resolution + y_origin;

		g2d::Field d = sqrt(x*x + y*y);
		if (d > range)
			return;

		cluster.emplace_back(x, y);
	}

	Indexes query(int i, int j)
	{
		static const int L_2 = 1;

		int i_0 = std::max(0, i - L_2);
		int i_n = std::min(x_size, i + L_2 + 1);

		int j_0 = std::max(0, j - L_2);
		int j_n = std::min(y_size, j + L_2 + 1);

		Indexes neighbors;
		neighbors.reserve((2 * L_2 + 1) * (2 * L_2 + 1));

		for (i = i_0; i < i_n; i++)
		{
			for (j = j_0; j < j_n; j++)
			{
				// Ignore uncertain obstacle points.
				if (complete_map[j + i * y_size] <= 0.5)
					continue;

				neighbors.emplace_back(i, j);
			}
		}

		return neighbors;
	}

	void scan(OfflineMap &map, size_t density)
	{
		std::vector<g2d::Points> &clusters = map.cartesian.clusters;
		for (int i = 0; i < x_size; i++)
		{
			for (int j = 0; j < y_size; j++)
			{
				// Ignore already clustered or uncertain obstacle points.
				int k = j + i * y_size;
				if (complete_map[k] <= 0.5)
					continue;

				append(i, j, map);

				if (clustered[k])
					continue;

				// Ignore points without enough neighbors.
				Indexes neighbors = query(i, j);
				if (neighbors.size() < density)
					continue;

				clusters.emplace_back();
				g2d::Points &cluster = clusters.back();
				append(i, j, cluster);

				for (size_t h = 0; h < neighbors.size(); h++)
				{
					Index &near = neighbors[h];
					int k = near.j + near.i * y_size;
					if (clustered[k])
						continue;

					append(near.i, near.j, cluster);

					Indexes farther = query(near.i, near.j);
					if (farther.size() >= density)
						neighbors.insert(neighbors.end(), farther.begin(), farther.end());
				}
			}
		}
	}
};

OfflineMap::OfflineMap()
{
	//nothing to do.
}

OfflineMap::OfflineMap(double range, carmen_localize_ackerman_globalpos_message *globalpos, carmen_mapper_map_message *map_message)
{
	this->range = range;

	if (map_message == NULL)
		return;

	Grid grid(range, globalpos, map_message);
	grid.scan(*this, 4);

// 	double *complete_map = map_message->complete_map;
//
// 	g2d::Field x_origin = map_message->config.x_origin;
// 	g2d::Field y_origin = map_message->config.y_origin;
//
// 	int x_size = map_message->config.x_size;
// 	int y_size = map_message->config.y_size;
//
// 	if (globalpos != NULL)
// 	{
// 		x_origin -= globalpos->globalpos.x;
// 		y_origin -= globalpos->globalpos.y;
// 	}
//
// 	resolution = map_message->config.resolution;
// 	center = g2d::Point(x_origin, y_origin);
//
// 	for (int i = 0; i < x_size; i++)
// 	{
// 		for (int j = 0; j < y_size; j++)
// 		{
// 			// Drop uncertain obstacle points.
// 			int index = j + i * y_size;
// 			if (complete_map[index] <= 0.5)
// 				continue;
//
// 			// Compute the obstacle point's coordinates relative to the car.
// 			g2d::Field x = i * resolution + x_origin;
// 			g2d::Field y = j * resolution + y_origin;
//
// 			g2d::Field t = atan2(y, x);
// 			g2d::Field d = sqrt(x*x + y*y);
//
// 			if (d > range)
// 				continue;
//
// 			cartesian.points.push_back(g2d::Point(x, y));
// 			polar.push_back(g2d::Point(t, d));
// 		}
// 	}
}

size_t OfflineMap::size() const
{
	return polar.size();
}
