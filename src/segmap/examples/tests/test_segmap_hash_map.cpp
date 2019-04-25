
#include <carmen/segmap_hash_map.h>
#include <cstdio>
#include <iostream>


int
main()
{
	KeyGen2D gen2d;
	KeyGen3D gen3d;

	int x, y, z;
	unsigned long key = gen2d.coordinates_to_key(3, 7, 5);
	 std::cout << std::bitset<64>(key) << std::endl;
	gen2d.key_to_coordinates(key, &x, &y, &z);
	printf("%d %d %d\n", x, y, z);


	key = gen3d.coordinates_to_key(3, 7, 5);
	 std::cout << std::bitset<64>(key) << std::endl;
	gen3d.key_to_coordinates(key, &x, &y, &z);
	printf("%d %d %d\n", x, y, z);

	pcl::PointXYZRGB point;
	HashGridMap<ColorCell> map;

	for (int i = 0; i < 100; i++)
	{
		point.r = rand() % 255;
		point.g = rand() % 255;
		point.b = rand() % 255;

		map.add(point);
	}

	printf("mean: %lf %lf %lf std: %lf %lf %lf\n",
				 map.cell.r_statistics.mean,
				 map.cell.g_statistics.mean,
				 map.cell.b_statistics.mean,
				 map.cell.r_statistics.std,
				 map.cell.g_statistics.std,
				 map.cell.b_statistics.std);

	return 0;
}

