
#include <carmen/segmap_key_generators.h>
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

	return 0;
}

